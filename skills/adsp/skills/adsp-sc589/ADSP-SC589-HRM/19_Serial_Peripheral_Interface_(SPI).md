## 16   Serial Peripheral Interface (SPI)

The serial peripheral interface is an industry-standard synchronous serial link that supports communication with multiple SPI-compatible devices. The baseline SPI peripheral is a synchronous, four-wire interface consisting of two data pins, one device select pin, and a gated clock pin. The two data pins allow full-duplex operation to other SPIcompatible devices. Two extra (optional) data pins are provided on specific SPIs to support quad SPI operation. Enhanced modes of operation such as flow control, fast mode, and dual-I/O mode (DIOM) are also supported. In addition, a direct memory access (DMA) mode allows for transferring several words with minimal CPU interaction.

With a range of configurable options, the SPI ports provide a glueless hardware interface with other SPI-compatible devices in master mode, slave mode, and multimaster environments. The SPI peripheral includes programmable baud rates, clock phase, and clock polarity. The peripheral can operate in a multimaster environment by interfacing with several other devices, acting as either a master device or a slave device. In a multimaster environment, the SPI peripheral uses open-drain outputs to avoid data bus contention. The flow control features enable slow slave devices to interface with fast master devices by providing an SPI ready pin which flexibly controls the transfers.

NOTE: All SPI peripherals (SPI0, SPI1 and SPI2) on the ADSP-SC58x/ADSP-2158x processors operate on the SCLK1\_0 domain. The term SCLK in this chapter is a generic reference to SCLK1\_0. For more details on SCLK1\_0 programming refer the Clock Generation Unit (CGU) chapter.

## SPI Features

The SPI module supports the following features:

- Full-duplex, synchronous serial interface
- 8, 16-bit and 32-bit word sizes
- Programmable baud rate, clock phase, and polarity
- Programmable inter-frame latency
- Flow control
- Support for Fast and DIOM modes
- Quad and memory-mapped modes are supported by SPI2 only
- Independent receive and transmit DMA channels

- Burst transfer mode for non-DMA write accesses

## SPI Functional Description

This section provides information on the function of the SPI module.

## Shift register functionality

The SPI is essentially a shift register that serially transmits and receives data bits to or from other SPI devices. During an SPI transfer, data is simultaneously transmitted (shifted out serially) and received (shifted in serially). A serial clock line synchronizes shifting and sampling of the information on the two serial data lines.

## Master slave functionality

During a data transfer, one SPI system acts as the link master which controls the data flow. The other system acts as the slave, which has data shifted into and out of it by the master. Different devices can take turn being masters, and one master can simultaneously shift data into multiple slaves (broadcast mode). However, only one slave can drive its output to write data back to the master at any given time. This rule must be enforced in the broadcast mode. Several slaves can be selected to receive data from the master in this mode. But only one slave can be enabled to send data back to the master.

## Enhanced operating modes

SPI supports enhanced modes of operation like fast mode, DIOM, and Quad-SPI, and optional flow control. In fast mode, received data is sampled on the transmit edge instead of the standard receive edge, thus enabling a full-cycle path for the received data. In DIOM, both MOSI and MISO are configured as input or output pins, and 2 bits are shifted in or out on each receive or transmit edge. In Quad-SPI mode, SPI\_D3:0 are configured as input or output pins and 4 bits are shifted in or out on each receive or transmit edge. A slower slave can use flow control to stall a faster master device.

## Single and multi-master use

The SPI can be used in a single master as well as multi-master environment. The SPI\_MOSI , SPI\_MISO , and the SPI\_CLK signals are all tied together in both configurations. SPI transmission and reception can be enabled simultaneously or individually, depending on SPI\_RXCTL and SPI\_TXCTL settings. In broadcast mode, several slaves can be enabled to receive, but only one slave must be in transmit mode and driving the SPI\_MISO line.

## ADSP-SC58x SPI Register List

The Serial Peripheral Interface (SPI) provides a full-duplex, synchronous serial interface, which supports both master/slave modes and multi-master environments. The SPI's baud rate and clock phase/polarities are programmable, and it has integrated DMA channels for both transmit and receive data streams. A set of registers governs SPI operations. For more information on SPI functionality, see the SPI register descriptions.

Table 16-1: ADSP-SC58x SPI Register List

| Name         | Description                              |
|--------------|------------------------------------------|
| SPI_CLK      | Clock Rate Register                      |
| SPI_CTL      | Control Register                         |
| SPI_DLY      | Delay Register                           |
| SPI_ILAT     | Masked Interrupt Condition Register      |
| SPI_ILAT_CLR | Masked Interrupt Clear Register          |
| SPI_IMSK     | Interrupt Mask Register                  |
| SPI_IMSK_CLR | Interrupt Mask Clear Register            |
| SPI_IMSK_SET | Interrupt Mask Set Register              |
| SPI_MMRDH    | Memory Mapped Read Header (Only on SPI2) |
| SPI_MMTOP    | SPI Memory Top Address (Only on SPI2)    |
| SPI_RFIFO    | Receive FIFO Data Register               |
| SPI_RWC      | Received Word Count Register             |
| SPI_RWCR     | Received Word Count Reload Register      |
| SPI_RXCTL    | Receive Control Register                 |
| SPI_SLVSEL   | Slave Select Register                    |
| SPI_STAT     | Status Register                          |
| SPI_TFIFO    | Transmit FIFO Data Register              |
| SPI_TWC      | Transmitted Word Count Register          |
| SPI_TWCR     | Transmitted Word Count Reload Register   |
| SPI_TXCTL    | Transmit Control Register                |

## ADSP-SC58x SPI Interrupt List

Table 16-2: ADSP-SC58x SPI Interrupt List

|   Interrupt ID | Name       | Description        | Sensitivity   |   DMA Channel |
|----------------|------------|--------------------|---------------|---------------|
|             68 | SPI2_TXDMA | SPI2 TX DMAChannel | Level         |            26 |
|             69 | SPI2_RXDMA | SPI2 RX DMAChannel | Level         |            27 |
|             70 | SPI2_STAT  | SPI2 Status        | Level         |               |
|             71 | SPI2_ERR   | SPI2 Error         | Level         |               |
|            106 | SPI0_TXDMA | SPI0 TX DMAChannel | Level         |            22 |
|            107 | SPI0_RXDMA | SPI0 RX DMAChannel | Level         |            23 |

Table 16-2: ADSP-SC58x SPI Interrupt List (Continued)

|   Interrupt ID | Name           | Description              | Sensitivity   |   DMA Channel |
|----------------|----------------|--------------------------|---------------|---------------|
|            108 | SPI0_STAT      | SPI0 Status              | Level         |               |
|            109 | SPI0_ERR       | SPI0 Error               | Level         |               |
|            110 | SPI1_TXDMA     | SPI1 TX DMAChannel       | Level         |            24 |
|            111 | SPI1_RXDMA     | SPI1 RX DMAChannel       | Level         |            25 |
|            112 | SPI1_STAT      | SPI1 Status              | Level         |               |
|            113 | SPI1_ERR       | SPI1 Error               | Level         |               |
|            188 | SPI2_TXDMA_ERR | SPI2 TX DMAChannel Error | Level         |               |
|            189 | SPI2_RXDMA_ERR | SPI2 RX DMAChannel Error | Level         |               |
|            198 | SPI0_TXDMA_ERR | SPI0 TX DMAChannel Error | Level         |               |
|            199 | SPI0_RXDMA_ERR | SPI0 RX DMAChannel Error | Level         |               |
|            200 | SPI1_TXDMA_ERR | SPI1 TX DMAChannel Error | Level         |               |
|            201 | SPI1_RXDMA_ERR | SPI1 RX DMAChannel Error | Level         |               |

## ADSP-SC58x SPI Trigger List

Table 16-3: ADSP-SC58x SPI Trigger List Masters

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|           40 | SPI0_TXDMA | SPI0 TX DMAChannel | Edge          |
|           41 | SPI0_RXDMA | SPI0 RX DMAChannel | Edge          |
|           42 | SPI1_TXDMA | SPI1 TX DMAChannel | Edge          |
|           43 | SPI1_RXDMA | SPI1 RX DMAChannel | Edge          |
|           44 | SPI2_TXDMA | SPI2 TX DMAChannel | Edge          |
|           45 | SPI2_RXDMA | SPI2 RX DMAChannel | Edge          |

Table 16-4: ADSP-SC58x SPI Trigger List Slaves

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|           33 | SPI0_TXDMA | SPI0 TX DMAChannel | Pulse         |
|           34 | SPI0_RXDMA | SPI0 RX DMAChannel | Pulse         |
|           35 | SPI1_TXDMA | SPI1 TX DMAChannel | Pulse         |
|           36 | SPI1_RXDMA | SPI1 RX DMAChannel | Pulse         |
|           37 | SPI2_TXDMA | SPI2 TX DMAChannel | Pulse         |

Table 16-4: ADSP-SC58x SPI Trigger List Slaves (Continued)

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|           38 | SPI2_RXDMA | SPI2 RX DMAChannel | Pulse         |

## ADSP-SC58x SPI DMA Channel List

Table 16-5: ADSP-SC58x SPI DMA Channel List

| DMAID   | DMAChannel Name   | Description        |
|---------|-------------------|--------------------|
| DMA22   | SPI0_TXDMA        | SPI0 TX DMAChannel |
| DMA23   | SPI0_RXDMA        | SPI0 RX DMAChannel |
| DMA24   | SPI1_TXDMA        | SPI1 TX DMAChannel |
| DMA25   | SPI1_RXDMA        | SPI1 RX DMAChannel |
| DMA26   | SPI2_TXDMA        | SPI2 TX DMAChannel |
| DMA27   | SPI2_RXDMA        | SPI2 RX DMAChannel |

## SPI Block Diagram

The SPI Controller Block Diagram illustrates the blocks of the SPI module. The module is comprised of three primary parts:

- SPI core contains the receive and transmit FIFOs and their associated shift registers
- Control blocks contain the synchronizer and logic to control the data flow through the data pipelines
- Register block

Figure 16-1: SPI Controller Block Diagram, Quad Mode

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000000_da37a944cfb5053c07a7c3a17d557a3b4524eda1523061657857282d80156913.png)

## Transfer Protocol

The SPI module implements two channels that are independent of each other. The SPI module uses the SPI\_RXCTL and SPI\_TXCTL dedicated control registers to control these channels. Except in dual and quad modes, SPI can enable and use both channels simultaneously.

The SPI protocol supports four different combinations of serial clock phase and polarity. These combinations are selected through the SPI\_CTL.CPOL and SPI\_CTL.CPHA bits.

The SPI Transfer Protocol figures demonstrate the two basic transfer formats as defined by the CPHA bit. T wo waveforms are shown for SPI\_CLK ; one for SPI\_CTL.CPOL =0 and the other for SPI\_CTL.CPOL =1. The diagrams can be interpreted as master or slave timing diagrams since the SPI\_CLK , SPI\_MISO , and SPI\_MOSI pins are directly connected between the master and the slave. The SPI\_MISO signal is the output from the slave (slave transmission), and the SPI\_MOSI signal is the output from the master (master transmission). The master generates the SPI\_CLK signal. The SPI\_SS signal is the slave device select input to the slave from the master. The diagrams represent an 8-bit transfer ( SPI\_CTL.SIZE =0) with the MSB first ( SPI\_CTL.LSBF =0). Any combination of the SPI\_CTL.SIZE and SPI\_CTL.LSBF bits is permissible. For example, a 16-bit transfer with the LSB first is another possible configuration.

The clock polarity and the clock phase could be identical for the master device and the slave device involved in the communication link. The transfer format from the master can be changed between transfers to adjust for various requirements of a slave device.

The SPI module uses the SPI\_CTL.ASSEL bit to determine when the SPI hardware or software control the SPI\_SEL[n] line. When SPI\_CTL.ASSEL =1, the slave select line must be set to the polarity set in the SPI\_CTL.SELST field between each serial transfer. The actual behavior of SPI\_SEL[n] also depends on the parameters programmed into the SPI\_DLY register. The SPI hardware logic automatically controls this functionality. When SPI\_CTL.ASSEL =0, SPI\_SEL[n] can either remain active between successive transfers or be inactive. The software must control this activity through manipulation of the SPI\_SLVSEL register.

The SPI Transfer Protocol pair of figures illustrates the case when SPI\_CTL.ASSEL = 1 and the SPI\_SEL[n] line is inactive between frames. If ASSEL = 0, the SPI\_SEL[n] line can remain active between frames; however, the first bit is only driven when an active transition of SPI\_CLK occurs.

Figure 16-2: SPI Transfer Protocol for CPHA=0

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000001_5f4183b92c521add4f274cfa42ba24af8fcf6e9aec9b7d6ba945175d8538a20f.png)

Figure 16-3: SPI Transfer Protocol for CPHA=1

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000002_7cfb270e303059dd5d70c3eae0e7afe451abe414f7e757c4d8e2abf0623c5b1c.png)

## Clock Considerations

The SPI\_CLK signal is a gated clock that is only active during data transfers, for the time of the transferred word. In normal mode, the number of active edges is equal to the number of bits to be transmitted or received. In dualI/O mode, it is half of the number of bits to be transmitted or received, and in quad-SPI mode it is one-fourth of

the number. The clock rate can be as high as the SCLK1\_0 rate, and both even and odd dividers from SCLK1\_0 are supported. For master devices, the SPI uses the SPI\_CLK register value to determine the clock rate, whereas this value is ignored for slave devices.

When the SPI controller is a master, SPI\_CLK is an output signal. Conversely, when the SPI controller is a slave, SPI\_CLK is an input signal. Slave devices ignore the SPI clock when the slave select input is driven inactive. The SPI uses the SPI\_CLK signal to shift out and shift in the data driven onto the SPI\_MISO and SPI\_MOSI lines. The data is always shifted out on one edge of the clock (the active edge) and sampled on the opposite edge of the clock (the sampling edge). Clock polarity and clock phase relative to data are programmable through the SPI\_CTL register and define the transfer format.

## Controlling Delay Between Frames

The SPI Timing with Lead and Lag Programming (Independent of SPI\_CTL.CPHA Setting) figure illustrates SPI timing using the SPI\_DLY.LEADX and SPI\_DLY.LAGX programming. The SPI uses the SPI\_DLY.LAGX bits to control the timing between the slave select ( SPI\_SS ) signal assertion and the first SPI\_CLK edge. The SPI uses the SPI\_DLY.LEADX bits to control the timing between the last SPI\_CLK edge and deassertion of the SPI\_SS signal. The lead and lag timing can be extended by a 1 SPI\_CLK duration to ease timing restrictions on the slave device.

Figure 16-4: SPI Timing with Lead and Lag Programming (Independent of SPI\_CTL.CPHA Setting)

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000003_43c0e13f95cb22102e4c2607f97a5de1cbfed5dfd4e0f4095c8aed3e2e1ca393.png)

The SPI Timing with SPI\_DLY.STOP Programming (Independent of SPI\_CTL.CPHA Setting) figure illustrates SPI timing with STOP programming. The SPI module uses this timing to insert multiples of SPI\_CLK period delays between transfers. The SPI\_SS line is deasserted for the duration specified in the SPI\_DLY.STOP bit field, assuming the SPI\_CTL.SELST bit is configured for deassertion between transfers.

If the SPI\_DLY.STOP bit =0, the master operates in a continuous mode . This mode causes an immediate start of the second word after the last bit is transferred from the first word. During this mode of operation, the slave select line is continuously asserted.

Figure 16-5: SPI Timing with SPI\_DLY.STOP Programming (Independent of SPI\_CTL.CPHA Setting)

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000004_bfcdcf90aa207a2f5f7269756ea0dde10a16826b7f7390d1a70cabc5487de1c4.png)

When the SPI\_DLY.STOP bit =0 and initial conditions for a transfer are not met, the interface pauses before the next transfer. During this pause, the SPI uses the SPI\_CTL.SELST bit to determine the state of the slave select pin. The SPI uses the SPI\_DLY.LEADX and SPI\_DLY.LAGX bits to determine the timing between SPI\_CLK edges and the slave select line.

## Flow Control

In master mode, the slave device must drive the SPI\_RDY pin. The pin acts as an input signal. The slave can deassert the SPI\_RDY pin to stop the master from initiating any new transfer. If SPI\_RDY is deasserted in the middle of a transfer, the current transfer continues, and the next transfer will not start unless the slave asserts the SPI\_RDY signal. Whenever the slave deasserts SPI\_RDY and stalls the master, the SPI controller goes into a waiting state, and the SPI\_STAT.FCS bit is set. When the slave asserts SPI\_RDY , the SPI controller resumes operation, and the SPI\_STAT.FCS bit is cleared.

In slave mode, the SPI\_RDY pin acts as an output signal. Flow control can be configured on either the TX channel or the RX channel. The SPI uses the SPI\_CTL.FCCH bit to control this configuration. If flow control is configured on the TX channel, as the SPI\_TFIFO status nears the empty condition, the SPI\_RDY pin is deasserted. If flow control is configured on the RX channel, as the SPI\_RFIFO status nears the full condition, the SPI\_RDY

pin is deasserted. The SPI uses the SPI\_CTL.FCWM bits to control the FIFO status at which SPI\_RDY deassertion takes place. Flow control in slave mode is purely based on the FIFO status and does not depend on the word counters.

The SPI Flow Control Timing in Master Mode figure illustrates this timing.

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000005_b08edffd5a0d5cfd662a10b9fb85c01072f44398e9c290e1b4b55a2d94c8d540.png)

Figure 16-6: SPI Flow Control Timing in Master Mode.

## Slave Select Operation

If the SPI is in slave mode, SPI\_SS acts as the slave select input. When SPI is enabled as a master, SPI\_SS can serve as an error detection input for the SPI in a multi-master environment. The SPI\_CTL.PSSE bit enables this feature. When SPI\_CTL.PSSE =1, the SPI\_SS input is the master mode error input. Otherwise, SPI\_SS is ignored.

The SPI\_SS signal is an active-low signal. The master asserts the signal during the transfer. The signal can be deasserted or remain asserted between transfers. When SPI\_SS is deasserted, SPI\_CLK and inputs are ignored, and outputs are three-stated.

The slave select bits ( SPI\_SLVSEL.SSEL1 -SPI\_SLVSEL.SSEL7 ) are used in a multiple-slave SPI environment. For example, if there are eight SPI devices in the system including a processor master, the master processor can support the SPI mode transactions across the other seven devices. This configuration requires only one master processor in this multi-slave environment.

For example, assume that the SPI of the processor is the master. The SPI\_SLVSEL.SSEL1 -SPI\_SLVSEL.SSEL7 bits on the processor can be connected to the slave select pin of each slave device. In this configuration, the slave select bits can be used in three ways. In cases 1 and 2, the processor is the master and the seven microcontrollers or peripherals with SPI interfaces are slaves. The processor can do one of the following:

1. Transmit to all seven SPI devices at the same time in a broadcast mode. Here, all slave select bits are set.
2. Receive and transmit from one SPI device by enabling only one slave SPI device at a time.
3. If all the slaves are also processors, then the requester can receive data from only one processor at a time. (The functionality is enabled by clearing the SPI\_CTL.EMISO bit in the six other slave processors.) The requestor can transmit broadcast data to all seven at the same time. This MISO enabling feature is available in some other microcontrollers. Therefore, it is possible to use the MISO enabling feature with any other SPI device that includes this functionality.

Figure 16-7: Single-Master, Multiple-Slave Configuration

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000006_1ffe020f9f9aa88b88fff20eab2af14a257c09791d2a323cbfc2f9862e421e93.png)

## Beginning and Ending a Non-DMA SPI Transfer

The start and finish of a non-DMA SPI transfer depend on the following settings.

1. Whether the device is configured as a master or a slave.
2. The state of the SPI\_CTL.ASSEL bit, which selects between hardware and software control over SPI\_SLVSEL .

When SPI\_CTL.CPHA =0, the enabled slave select outputs are driven active. However, the SPI\_CLK signal remains inactive for the first half of the first cycle of SPI\_CLK . For a slave with SPI\_CTL.CPHA =0, the transfer starts as soon as the SPI\_SS input goes low.

When SPI\_CTL.CPHA =1, a transfer starts with the first active edge of SPI\_CLK for both slave and master devices. For a master device, a transfer is complete after it sends the last data and simultaneously receives the last data bit. A transfer for a slave device ends after the last sampling edge of SPI\_CLK . If SPI\_CTL.ASSEL =0, the hardware maintains responsibility for toggling SPI\_SS between frames. If SPI\_CTL.ASSEL =1, software controls the SPI\_SS line and can keep it active between frames.

The SPI\_STAT.RFE bit defines when the receive buffer can be read, indicating that SPI\_RFIFO is not empty. The SPI\_STAT.TFF bit defines when the transmit buffer can be written, indicating that the SPI\_TFIFO is not full. The end of a single word transfer occurs when the SPI\_STAT.RFE bit is cleared. The status indicates that a new word has been received and written into the receive FIFO. The SPI\_STAT.RFE bit remains cleared as long as the receive FIFO has valid data.

To maintain software compatibility with other SPI devices, the SPI\_STAT.SPIF bit is also available for polling. This bit can have a slightly different behavior from other commercially available devices.

In master mode with the SPI\_CTL.ASSEL bit cleared, software manually asserts the required slave select signal before starting the transaction. After all data transfers, software typically releases the slave select line.

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

In Dual I/O mode, the SPI\_MISO and SPI\_MOSI pins are configured to operate in the same direction which doubles bandwidth. The SPI uses the SPI\_CTL.SOSI bit to determine the order of bits on the pins. When set, the processor sends the first bit on the SPI\_MOSI pin and the second bit on the SPI\_MISO pin. If the

SPI\_CTL.SOSI bit is cleared, the order is reversed. Since dual I/O mode uses both pins to transmit or receive data, only one channel can be enabled, either transmit or receive. Flow control through the SPI\_RDY pin is supported. Interrupt request generation is unaffected by dual I/O mode. However, the interrupt service interval is reduced, since the individual transfer latency is halved.

Changing to quad SPI mode must be done when the SPI is in a quiescent state.

Figure 16-8: Dual I/O Mode Transfer Protocol for CPHA=0, SOSI=1, 8-Bit Transfer, LSBF=0.

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000007_f9b5513a67b600c558406a095330970a072e8ee8fb9635feaf32bfc692e8c633.png)

Cycle

Number

SPI\_CLK

(CPOL=0)

SPI\_CLK

(CPOL=1)

SPI\_MOSI

SPI\_MISO

Slave

Select

Figure 16-9: Dual I/O Mode Transfer Protocol for CPHA=1, SOSI=0, 8-Bit Transfer, LSBF=0.

## Quad I/O Mode (SPI2 only)

In quad SPI mode, the SPI\_MISO and SPI\_MOSI pins, in tandem with the SPI\_D2 and SPI\_D3 pins, are configured to operate in the same direction. The SPI uses the SPI\_CTL.SOSI bit to determine the order of bits on the pins. When set, the processor sends:

- The first bit on the SPI\_MOSI pin
- The second bit on the SPI\_MISO pin
- The third bit on the SPI\_D2 pin
- The fourth bit on the SPI\_D3 pin

If the SPI\_CTL.SOSI bit is cleared, the order is reversed. Since quad SPI mode uses all four pins to transmit or receive data, only one channel can be enabled as either transmit or receive. Flow control through the SPI\_RDY pin is supported. Interrupt generation is unaffected by quad SPI mode.

1

6

MSB

2

3

4

LSB

1

4

5

2

3

Changing to quad SPI mode must be done when the SPI is in a quiescent state.

While using dual or quad I/O mode for communicating with flash devices, program the SPI\_CTL.CPHA and the SPI\_CTL.CPOL bits =1. This programming avoids bus contention during read operations, because the flash device starts driving out the bits immediately after dummy cycles in read header.

Figure 16-10: Quad Mode Timing for CPHA=0, SOSI=1, 16-Bit Transfer, LSBF=0.

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000008_bc6f41beedc9d363f42cb0bab5c751f2a7228ce0c078b307682bc204cba29f24.png)

NOTE: The SPI does support quad SPI 8-bit transfer in slave continuous mode of operation with an SCLK: SPI\_CLK ratio of less than 1:2. A minimum of 2 SCLK cycles is required between transfers in 8bit quad SPI slave mode with an SCLK: SPI\_CLK ratio of less than 1:2.

## Fast Mode

Fast mode is similar to the normal mode of operation when transmitting. When receiving, data is sampled at the next transmit edge allowing a full cycle of timing in the receive direction. This mode is valid in master mode operation only. When the SPI operates in fast mode, the slave drives the data for one full cycle.

Figure 16-11: SPI Transfer Protocol in Fast Mode for SPI\_CTL.CPHA = 0

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000009_baf522c4a4e77e409242572ef6e5ea8e1234fd156509cbc2e377b789b573aa65.png)

Figure 16-12: SPI Transfer Protocol in Fast Mode for SPI\_CTL.CPHA = 1

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000010_1213841ed0dbd3c1063e9bc891c1428b7585dbbf200d14e430da62ec18acdb25.png)

## Memory-Mapped Mode (SPI2 only)

The SPI supports direct memory-mapped read accesses from a SPI memory device, enabled by setting the SPI\_CTL.MMSE bit. This mode allows for direct execution of instructions from a SPI memory device without the need for a low-level software driver, as hardware handles all overhead tasks (for example, transmission of the read header, pin turnaround timing, and receive data sizing). The SPI features configurable options in the memory-mapped read header register ( SPI\_MMRDH ) to provide compatibility with a wide range of SPI memory devices.

In non-memory-mapped mode, the software is responsible for providing the command and required dummy words for the read response, whereas this is all handled by hardware when the SPI is in memory-mapped mode. The memory of the SPI device is accessible directly through reads of the processor address space. The read accesses can be code or data accesses in core mode or when using memory DMA (MDMA). These accesses allow code to execute directly from SPI memory devices (true eXecute-In-Place operations), and the contents can be cached to improve performance. It is not necessary to access the SPI data buffer registers nor poll status bits; however, the hardware does not support peripheral DMA accesses nor write operations to the SPI memory space.

The Types of Operations table is a comparison of the permitted operations in the non-memory-mapped and memory-mapped modes supported by the SPI controller.

Table 16-6: Types of Operations

| SPI Operation                                         | Non-Memory-Mapped Mode   | Memory-Mapped Mode   |
|-------------------------------------------------------|--------------------------|----------------------|
| Core data write                                       | Yes                      | No                   |
| Core data read                                        | Yes                      | Yes                  |
| Code fetch: Execute-In-Place (XIP)                    | No                       | Yes                  |
| Read/Write accesses using SPI PeripheralDMA           | Yes                      | No                   |
| Read/Write accesses by other peripheral DMAchan- nels | No                       | No                   |
| MDMAread                                              | No                       | Yes                  |

Table 16-6: Types of Operations (Continued)

| SPI Operation   | Non-Memory-Mapped Mode   | Memory-Mapped Mode   |
|-----------------|--------------------------|----------------------|
| MDMAwrite       | No                       | No                   |

## Memory-Mapped Description of Operation

Memory-mapped mode is enabled by setting the SPI\_CTL.MMSE bit. When enabled, the SPI (if ready) accepts the read requests through a dedicated on-chip slave interface. The memory subsystem master drives this dedicated interface through the SCB fabric.

In a typical scenario, the memory subsystem master issues read requests to the fabric, and the fabric routes these requests to the slave port of the SPI peripheral. The master describes the read access using a number of parameters such as starting address, transfer size, and burst type. The SPI responds to this read access request when it is ready for a new transfer. It loads the opcode, a specified number of address bytes, and an optional mode byte into the transmit FIFO. The SPI memory state machine begins when both the transmit and receive channels of the SPI are enabled:

- the transmit transfer initiation bit is set ( SPI\_TXCTL.TTI =1), and
- the receive initiation bit is cleared ( SPI\_RXCTL.RTI =0)

The SPI memory read sequence starts with the assertion of SPI\_SEL1 . If the SPI memory state machine is in the reset state, it looks for a command. The SPI hardware then sends the specific 8-bit read command (which can be optionally skipped), followed by the SPI memory read address. After this, a dummy period is inserted, in which a mode byte is optionally sent and the pins are held or three-stated during the dummy clocking period.

NOTE: This read header is transmitted over the SPI standard protocol pins ( SPI\_CLK , SPI\_MOSI , SPI\_MISO , SPI\_SEL1 ) or over the extended SPI protocol pins ( SPI\_CLK , SPI\_MOSI , SPI\_MISO , SPI\_D2 , SPI\_D3 , SPI\_SEL1 ), based on the SPI\_MMRDH.CMDPINS , SPI\_MMRDH.ADRPINS , and SPI\_CTL.MIOM bit settings. SPI memory devices usually support communication in MSB-first mode. In dual mode, the SPI typically uses SPI\_MISO as IO1 and SPI\_MOSI as IO0. In quad mode, the SPI typically uses SPI\_D3 pin as IO3, SPI\_D2 as IO2, SPI\_MISO as IO1, and SPI\_MOSI as IO0.

When all I/O data pins are three-stated, the SPI continues clocking the SPI memory device, which drives out the data bits at the addressed location, until all bytes are received. The SPI hardware reads the data as configured by the SPI\_CTL.MIOM bit setting. Upon reception of the last byte, the SPI typically deasserts SPI\_SEL1 to prepare for the next requested read header.

Application code must ensure that the opcode sent is consistent with multiple I/O programming and that the parameters specified in the memory-mapped read header register are consistent with flash read access timing.

The SPI Memory-Mapped Register Operations Flow diagram shows how the fields of the SPI\_MMRDH register determine the read header while initiating transfers in memory-mapped mode.

Figure 16-13: SPI Memory-Mapped Register Operations Flow

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000011_6074c7bb04c6fbf93052562315f550dbfc855cd2330e9d30fa45cdc9640b0a2d.png)

## Memory-Mapped Architectural Concepts

In memory-mapped mode, the SPI accepts read requests through a dedicated on-chip slave interface. The SPI (if ready) accepts these requests and begins the process of assembling the read header based on access attributes described in both the SPI\_MMRDH register and the internal bus request. After the read header transmission is complete, a pin turnaround period is timed and the receiver is enabled. The SPI continues clocking the SPI memory device until all bytes are received.

The SPI memory-mapped hardware accommodates various memory devices with different read timing. The capabilities include extra mode bits, flexible dummy period timing, and three-state control, as configured in the SPI\_MMRDH register.

The Memory-Mapped Protocol figure shows the protocol for the SPI controller in memory-mapped mode.

Figure 16-14: Memory-Mapped Protocol

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000012_eb4997bba16330ede47ef538acbef63cae1080f623e2f0b4186d653526be10bf.png)

As shown in the figure, the COMMAND field ( SPI\_MMRDH.OPCODE ) is transmitted upon assertion of the SPI\_SEL[n] signal. The SPI memory interprets this 8-bit value as a read command. Any 8-bit read opcode whose timing is compliant with the processor SPI and features provided by memory-mapped hardware is allowed, the most common being:

- Standard Read (0x03)
- Fast Read (0x0B)
- Fast Read Dual Output (0x3B)
- Fast Read Dual I/O (0x6B)
- Fast Read Quad Output (0xBB)
- Fast Read Quad I/O (0xEB)
- Word Read Quad I/O (0xE7)
- Octal Word Read Quad I/O (0xE3)

NOTE: The SPI hardware does not validate the content of the SPI\_MMRDH.OPCODE field prior to transmitting.

## DMYSIZE (Number of Dummy Bytes)

When operating at a high clock frequency in multi-IO modes, most flash devices require some dummy clocks after the address bits. These dummy clock cycles allow the internal circuits of the device extra time for setting up the initial address. These bits specify the number of bytes separating address transmission and read data return.

The number of dummy cycles required varies per manufacturer, the read command used, and the SPI access time. The SPI hardware allows dummy cycles to be programmed in bytes in the SPI\_MMRDH.DMYSIZE field, the value

of which is a function of the number of pins used to transmit the address ( SPI\_MMRDH.ADRPINS ), as shown in the Pins Used to Transmit the Address (ADRPINS) table.

Table 16-7: Pins Used to Transmit the Address (ADRPINS)

|                    | Dummy clock cycles                                                     | Dummy clock cycles                                                       | Dummy clock cycles                                              |
|--------------------|------------------------------------------------------------------------|--------------------------------------------------------------------------|-----------------------------------------------------------------|
| SPI_MMRDH. DMYSIZE | ( SPI_MMRDH.ADRPINS =0, SPI_CTL.MIOM =x) Dummy bytes elapse over 1-pin | ( SPI_MMRDH.ADRPINS =1, SPI_CTL.MIOM =1) Dummy bytes elapse over 2- pins | ( SPI_MMRDH.ADRPINS =1, MIOM=2) Dummy bytes elapse over 4- pins |
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

The SPI hardware supports the most commonly used read operations.

- Two standard SPI reads (read and read fast), which use the unidirectional SPI\_MOSI and SPI\_MISO pins in addition to SPI\_SEL[n] and SPI\_CLK
- Four extended SPI multiple I/O reads: dual output, quad output, dual I/O, and quad I/O reads

The SPI Read Operations table and SPI Flash Fast Read Sequence figures summarize the types of read operations. Program each read operation in a way that is compatible with the description given in the SPI flash data sheet.

Table 16-8: SPI Read Operations

| Operation        | Read Command (Opcode)   | CMDPIN   |   ADRPIN | DMYSIZE   | Three-state   | Multiple I/O Mode   |   Data Pins |
|------------------|-------------------------|----------|----------|-----------|---------------|---------------------|-------------|
| Read             | 0x03                    | 1        |        1 | Zero      | No            | No                  |           1 |
| Fast Read        | 0x0B                    | 1        |        1 | Non-Zero  | Yes           | No                  |           1 |
| Dual Output Read | 0x3B                    | 1        |        1 | Non-Zero  | Yes           | Yes(IO0-1)          |           2 |
| Quad Output Read | 0x6B                    | 1        |        1 | Non-Zero  | Yes           | Yes(IO0-3)          |           4 |
| Dual I/O Read    | 0xBB                    | 1, 2     |        2 | Non-Zero  | Yes           | Yes (IO0-1)         |           2 |

Table 16-8: SPI Read Operations (Continued)

| Operation     | Read Command (Opcode)   | CMDPIN   |   ADRPIN | DMYSIZE   | Three-state   | Multiple I/O Mode   |   Data Pins |
|---------------|-------------------------|----------|----------|-----------|---------------|---------------------|-------------|
| Quad I/O Read | 0xEB                    | 1, 4     |        4 | Non-Zero  | Yes           | Yes (IO0-3)         |           4 |

Some memory devices also support word quad I/O read (0xE7) and octal quad I/O read (0xE3) operations. These operations require fewer dummy cycles than normal quad I/O read operations.

Figure 16-15: SPI Flash Fast Read Sequence

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000013_2e3632cbdda4626b43a9c68818536bd21646d5531cc8ef254c658e183a4f7a5e.png)

Figure 16-16: SPI Flash Fast Read (Dual Output) Sequence

Figure 16-17: SPI Flash Fast Read (Dual I/O) Sequence

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000014_0afd1671e6207e5f45611de51b70d1bccea1f016eda8f14a30533c7692f4d690.png)

Figure 16-18: SPI Flash Quad Output Read Sequence

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000015_91a2311e699eb89ea8a51ed1fd2f8c1489948592ab57beed2d6441420dd8b8de.png)

Figure 16-19: SPI Flash Quad I/O Read Sequence

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000016_c1cb71d8bea4610312efb47a9348e6a84b8943c7ccb42bd3357b103eaeff7ea8.png)

SPI memory-mapped reads can be made cacheable in the core's internal memory by properly configuring the region as cacheable memory without bypass (see the related core's cache configuration documentation for details). In the figures, the number of read data bytes (N) is based on the following:

- For an instruction fetch by core (when in XIP mode); the number of instruction bytes to fetch depends the cache line size of the cache.
- For a data fetch by the core (data read), the number of data bytes to fetch depends on the cache line size of the cache.

Although the minimum size of a memory-mapped data read transfer is 4 bytes, applications can fetch a single byte or a 2-byte data. (For example, it can fetch an unsigned char or short access in C code). In this case, only the required bytes are provided to the core and the other bytes are cached.

The on-chip memory subsystem master provides a starting address for the burst and the SPI hardware issues this address as part of the read header. The address provided is N-byte aligned. For example, to read the 30th byte from SPI memory, then the typical address to provide is:

- 28 (0x0000\_001C) for a 32-bit cache line
- 24 (0x0000\_0018) for a 64-bit cache line
- 16 (0x0000\_0010) for a 128-bit cache line
- 0 (0x0000\_0000) for a 256-bit cache line

The read data is returned to the memory subsystem in the order provided by the SPI memory. There can be considerable delay for the expected data provided to the master.

To minimize this delay, the wrap feature can be used where the memory subsystem provides the address of the critical word.

- For MDMA reads, the number of read data bytes (N) is always equal to 4 bytes. The MDMA read does not depend on the cache setting. For MDMA reads, limit the DMA\_CFG.MSIZE field to 1, 2 or 4 bytes. The address provided by the memory subsystem master to the SPI hardware is always 4 byte-aligned.

## Memory-Mapped High-Performance Features

In addition to automating the SPI memory read accesses, the memory-mapped hardware also provides some features to improve SPI memory fetches and increase the system performance. The following sections describe these features.

## Merged Read Accesses

It is common for the memory subsystem to fetch two or more cache lines from consecutive addresses (the address sequencing is linear without any jumps). To take advantage of this situation, the SPI memory-mapped hardware provides a feature called merging. Enable merging by setting the SPI\_MMRDH.MERGE bit.

When enabled, the hardware compares the address of an incoming read request to the address of a request the SPI memory is actively servicing. It can decide to merge two accesses when the address for the second access is incremental. For example, if the first address of a 32-byte cache line fetch is 0x0000\_0000 and the second fetch is to address 0x0000\_0020, then these two accesses can be merged. Merging increases efficiency and overall fetch bandwidth by eliminating the read header for those accesses which only require continuation of the SPI clock.

## Wrap Around Accesses

Many SPI flash memory devices support wrapping which is used to enhance critical word fetching of cache lines. In this mode, the SPI device automatically wraps the read address to the base of a cache line once the end of the cache line is reached.

Wrap around accesses are enabled by setting the SPI\_MMRDH.WRAP bit.

Some flash devices require programs to send a Set Wrap command to place the device in wrap mode. Other flash devices provide a configuration register which must be programmed to set the flash in wrap mode. Since the SPI memory-mapped hardware does not support any write operations to flash, perform this step in non-memory-mapped mode ( SPI\_CTL.MMSE =0) by accessing the SPI registers.

Table 16-9: Wrap Modes

| Cache Line Size   | Wrap Mode        | Comments                                                                                |
|-------------------|------------------|-----------------------------------------------------------------------------------------|
| 4-byte            | Not applicable   | Usually flash does not support 4-byte wrapping.                                         |
| 8-byte            | 8 byte wrapping  | Read data wraps within an aligned 8-byte boundary starting from the specified address.  |
| 16-byte           | 16 byte wrapping | Read data wraps within an aligned 16-byte boundary starting from the specified address. |
| 32-byte           | 32 byte wrapping | Read data wraps within an aligned 32-byte boundary starting from the specified address. |

Data access is limited to 8-byte, 16-byte, or 32-byte sections of flash page in wrap mode. The ARM core uses the Wrap 4 access (64-bit data) for L1 cache. The ARM core uses Wrap 4 and Wrap 8 accesses (64-bit data) for L2 cache. The cores use the Wrap 8 accesses for unaligned accesses. During the read request to the SPI memory-mapped hardware, the memory subsystem master of the processor provides the address of a critical word instead of the line base. The read-data starts at the address specified in the instruction. Once it reaches the end boundary of the 8, 16, or 32-byte section, the output automatically wraps around to the beginning boundary to the line base address. The data fetch continues. It is not necessary to deassert the SPI SPI\_SEL[n] signal or resend the read header to wrap to the cache line base when servicing misaligned cache fill requests.

The Byte Sequence in Wrap Modes table shows byte sequences in various wrap modes.

Table 16-10: Byte Sequence in Wrap Modes

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

Execute-In-Place, most commonly known as XIP , allows software code to execute directly from an SPI flash device rather than downloading the code and executing it out of RAM. XIP , also known as Command Skip mode, is a general term and can be applied to fetching data as well.

There is a difference between XIP mode and standard mode. In XIP mode, after the SPI memory device is selected (CS# =LOW), the memory device does not decode the first input byte as command code. Instead, it expects the read header to directly start with address bytes. In standard mode, the memory decodes the first input byte it receives as a command code.

The XIP mode dramatically reduces random access time for applications that require fast code execution without shadowing the memory content on a RAM. The SPI memory-mapped hardware provides a control bit, SPI\_MMRDH.CMDSKIP to skip the command from read header.

Some SPI memory devices require configuration of their control register to enable the XIP mode of operation, using the non-memory-mapped mode of the processor SPI. Typically, during the dummy cycle period, the mode bits are used to confirm the XIP operation and the SPI\_MMRDH.MODE field must be set appropriately. A dummy

memory-mapped access may be needed before setting the SPI\_MMRDH.CMDSKIP bit in order to set the SPI memory device in Command Skip mode.

For more details about how to configure SPI memories into XIP mode, refer to the device data sheet.

- NOTE: When configuring the flash to XIP mode from the SHARC+ core, ensure that the routine that configures flash to XIP is not routed through the L2CC. This is accomplished by first configuring the flash to XIP mode, then enabling the L2CC from the core.

## Memory-Mapped Mode Error Status Bits

The SPI memory-mapped hardware provides bits in the SPI\_STAT register to report errors. It provides these bits for notification only and their state has no effect on SPI operations. The status register bits are sticky. A W1C (write-1-to-clear) operation clears the bits.

- Memory-Mapped Write Error ( SPI\_STAT.MMWE ). This bit is set (=1) if an attempt is made to write to address space that is reserved for memory-mapped SPI memory. The SPI memory-mapped hardware does not support automated write access to SPI memory space.
- Memory-Mapped Read Error ( SPI\_STAT.MMRE ). This bit is set (=1) if an attempt is made to read address space reserved for memory-mapped SPI memory while memory mapping is disabled ( SPI\_CTL.MMSE =0).
- Memory-Mapped Access Error ( SPI\_STAT.MMAE ). This bit is set (=1) if an attempt is made to access either the TX or RX FIFO while memory-mapped access of SPI memory is enabled. In this case, attempts to communicate with the SPI device using legacy methods are blocked and receive fabric error responses. Legacy methods include any direct access made to the TX and RX FIFOs, whether by DMA or processor MMR.
- Memory-Mapped Write Error Mask ( SPI\_CTL.MMWEM ) bit specifies whether an error response is returned to the fabric on write attempts to address space that is reserved for memory-mapped SPI memory reads. Regardless of whether a write error response is masked using this bit, the memory-mapped write error ( SPI\_STAT.MMWE ) sticky notification bit is still set.

NOTE: Unlike other bits in the SPI\_STAT register, these memory-mapped mode error bits do not have associated bits in the SPI interrupt mask ( SPI\_IMSK ) and SPI interrupt condition ( SPI\_ILAT ) registers.

The memory-mapped top register ( SPI\_MMTOP ) is used to specify the upper limit of the SPI memory address. The memory-mapped accesses to SPI memory addresses equal to or above this range are considered illegal. The accesses are blocked and a bus error response is generated.

This register is useful to block the invalid SPI memory address accesses. Some SPI memory vendors do not clearly specify (guarantee) that overrange address bits are ignored (address spaces can be wrapped).

## Memory-Mapped Programming Guidelines

Seting the SPI\_CTL.MMSE bit enables SPI memory-mapped mode. When enabled, the SPI interface is forced to be consistent with SPI memory requirements regardless the settings of certain control bits. The following tables specify typical settings for configuring the SPI in memory-mapped mode:

Table 16-11: SPI Control (SPI\_CTL) Register

| Bits                                                | Typical values to set   | Description                               | Comments                                                                                  |
|-----------------------------------------------------|-------------------------|-------------------------------------------|-------------------------------------------------------------------------------------------|
| SPI_CTL.MSTR                                        | 1                       | Master mode enable                        |                                                                                           |
| SPI_CTL.PSSE                                        | 0                       | Protected slave select ena- ble           |                                                                                           |
| SPI_CTL.ODM                                         | 0                       | Open-drain mode enable                    |                                                                                           |
| SPI_CTL.CPHASPI_ CTL.CPOL                           | 0-0 or 1-1              | SPI mode of communica- tion               | Flash dependent, usually SPI flash supports mode-0 (CPHA=CPOL=0) and mode-3 (CPHA=CPOL=1) |
| SPI_CTL.ASSEL                                       | 1                       | Hardware slave select pin control         |                                                                                           |
| SPI_CTL.SELST                                       | 1                       | Slave select asserted be- tween transfers |                                                                                           |
| SPI_CTL.EMISO                                       | 1                       | MISO pin enable                           |                                                                                           |
| SPI_CTL.SIZE                                        | 2                       | 32-bit transfer size                      |                                                                                           |
| SPI_CTL.LSBF                                        | 0                       | MSB bit first mode                        | Flash dependent, usually SPI flash commu- nicates in MSB bit first mode                   |
| SPI_CTL.FCEN SPI_CTL.FCCH SPI_CTL.FCPL SPI_CTL.FCWM | 0                       | Hardware flow control re- lated bits      |                                                                                           |
| SPI_CTL.FMODE                                       | 1                       | Fast mode enable                          | Typically set to 1 for full cycle timing, 0 only works at low speed                       |
| SPI_CTL.SOSI                                        | 0                       | Treat SPI_MOSI pin as IO0 pin.            |                                                                                           |

Table 16-12: SPI Receive Control Register

| Bits            |   Typical values to set | Description                            |
|-----------------|-------------------------|----------------------------------------|
| SPI_RXCTL.REN   |                       1 | Receive channel enable                 |
| SPI_RXCTL.RTI   |                       0 | Receive transfer initiation disable    |
| SPI_RXCTL.RWCEN |                       0 | Receive word counter disable           |
| SPI_RXCTL.RDR   |                       0 | Receive data request disable           |
| SPI_RXCTL.RDO   |                       0 | Discard incoming data if RFIFO is full |
| SPI_RXCTL.RRWM  |                       0 | Receive FIFO regular watermark         |
| SPI_RXCTL.RUWM  |                       0 | Receive FIFO urgent watermark disable  |

Table 16-13: SPI Transmit Control Register

| Bits            |   Typical values to set | Description                            |
|-----------------|-------------------------|----------------------------------------|
| SPI_TXCTL.TEN   |                       1 | Transmit channel enable                |
| SPI_TXCTL.TTI   |                       1 | Transmit transfer initiation disable   |
| SPI_TXCTL.TWCEN |                       0 | Transmit word counter disable          |
| SPI_TXCTL.TDR   |                       0 | Transmit data request disable          |
| SPI_TXCTL.TDU   |                       0 | Send last word when TFIFO is empty     |
| SPI_TXCTL.TRWM  |                       0 | Transmit FIFO regular watermark        |
| SPI_TXCTL.TUWM  |                       0 | Transmit FIFO urgent watermark disable |

Table 16-14: SPI DLY Control Register

| Bits          |   Typical values to set | Description                    | Comments See Flash data sheet for CS (for example. SSEL) timing specs   |
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

- The SPI memory-mapped hardware does not check the flash status before initiating the access. It assumes that SPI memory is always able to respond to a read access. Befor enabling memory-mapped mode (for example, setting the SPI\_CTL.MMSE bit) ensure that SPI flash is ready for a read access. When using non-memorymapped mode, a write-complete status can be examined prior to enabling the SPI in memory-mapped mode. (See the write in progress bit in the SPI flash memory status register.) Also, immediately after initial power-up, SPI memory devices can be inaccessible for a vendor-specified period.
- When SPI is enabled in memory-mapped mode, attempts to communicate with the SPI device using legacy methods are blocked. Legacy methods include any direct access made to the transmit or receive FIFOs, whether initiated by DMA or by a processor MMR access.

- To use some of the features offered by SPI memory devices, programs can first configure the SPI memory device by setting its control word or sending some commands. Since SPI memory-mapped hardware does not allow any type of SPI write operations, configure the SPI in non-memory-mapped mode prior to enabling memory-mapped mode.
- The memory-mapped hardware does not interpret the opcode. It does not check the validity of the timing that is specified in the SPI\_MMRDH register for a particular opcode. Programs must set the fields of the SPI\_MMRDH register to be consistent with the read-type selected.
- When the core requests the data or code fetch, the memory-mapped transfer depends on cache settings. The cache configuration register in the SPI memory device must be appropriately configured before enabling memory-mapped mode. Some of the high performance modes like merge, wrap, and transfer size depend on cache parameters.
- SPI memory-mapped MDMA reads do not support wrapping. For MDMA reads, limit the DMA\_CFG.MSIZE field to 1 byte, 2 bytes or 4 bytes.
- There is not always tool support to change the SPI memory-mapped hardware setting or cache settings on-thefly. Changing these settings can optimize the performance of code that accesses SPI memory in memory-mapped mode. It is expected that the SPI memory, SPI peripheral, and cache are programmed to one specific set of control settings for the whole application. Profiling or benchmarking of the actual application can be done to find the setting that works best.

## SPI Interrupt Signals

The SPI controller supports three types of interrupt request signals that correspond to data, status, and error conditions.

## Data Interrupts

The SPI peripheral supports two data interrupt channels - receive and transmit. These interrupt signals are multiplexed into the DMA request lines. Since the peripheral interfaces with separate read and write interfaces with DMA, the read and write data interrupts are independent. When the DMA channels are not used, the interrupts are routed directly to the system event controller. The interrupts occupy the same vector locations as the corresponding DMA channels.

Each of the data interrupt requests can be individually controlled. Program the SPI\_RXCTL.RDR and SPI\_TXCTL.TDR bit fields for receive and transmit, respectively. When receive is enabled, the RX interrupt request is issued whenever there is data available in the receive datapath for reading. (The event occurs according to the SPI\_RXCTL.RDR bit setting.) When transmit is enabled, the TX interrupt request is issued whenever the transmit datapath can be written. (The event occurs according to the SPI\_TXCTL.TDR setting.) DMA data interrupts are compatible with second-generation DMA to incorporate urgent data requests and transfer finish interrupt requests apart from the usual data request interrupts. T ransmit interrupt requests operate independently from the word counter-value in the SPI\_TWC register.

## Status Interrupts

The SPI controller supports several status interrupt requests to indicate different conditions of the receiver and transmitter. All status interrupt requests can be masked. Status interrupt requests are signaled directly through a single SPI status IRQ line. The line cannot be combined with the SPI error IRQ line for some processors. The SPI Status Interrupts table describes the status interrupt requests that are available for the SPI controller.

Table 16-15: SPI Status Interrupts

| SPI_STAT Bit   | Description                                                                                                                                                                                                                                                                                                                                     |
|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| SPI_STAT.RUWM  | Receive FIFO urgent watermark interrupt request. Issued when the level of the RFIFO breaches the water- mark set in the SPI_RXCTL.RUWM field. It is cleared when the level of the RFIFO reaches the watermark set in the SPI_RXCTL.RRWM field. If the RX channel is configured in DMAmode, SPI_RXCTL.RUWM is multiplexed with the data request. |
| SPI_STAT.TUWM  | Receive FIFO urgent watermark interrupt request. Issued when the level of the TFIFO breaches the water- mark set using the SPI_TXCTL.TUWM bit. It is cleared when the level of the TFIFO reaches the watermark set in the SPI_TXCTL.TRWM field. If the TX channel is configured in DMAmode, SPI_STAT.TUWM is multiplexed with the data request. |
| SPI_STAT.TS    | Transmit start interrupt request. Issued when the start of a transmit burst is detected by loading of the SPI_TWC register with the contents of the SPI_TWCR register.                                                                                                                                                                          |
| SPI_STAT.RS    | Receive start interrupt request. Issued when the start of a receive burst is detected by the loading of SPI_RWC with the contents of SPI_RWCR .                                                                                                                                                                                                 |
| SPI_STAT.TF    | Transmit finish interrupt request. Issued when a transmit burst completes ( SPI_TWC decrements to zero).                                                                                                                                                                                                                                        |
| SPI_STAT.RF    | Receive finish interrupt request. Issued when a receive burst completes ( SPI_RWC decrements to zero).                                                                                                                                                                                                                                          |

## Error Conditions

The SPI controller supports interrupt requests upon several different error conditions. All interrupt requests are maskable. The individual error indications combine into a single SPI error IRQ signal, which can be multiplexed on some processors with the aggregated SPI status IRQ signal. The SPI Error Interrupts table details the possible error indications.

Error conditions arise depending on which of the channels (transmit or receive) are enabled. If a channel is disabled, all errors related to it are ignored. When both channels are enabled, errors from both channels are enabled.

Table 16-16: SPI Error Interrupts

| Bit          | Description                                                                                                                                                                                                                                                                                            |
|--------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| SPI_STAT.MF  | Mode fault. Signaled when another device also tries to be a master in a multi-master system and drives the SPI_SS input low. This error is signaled in master mode operation.                                                                                                                          |
| SPI_STAT.TUR | Transmission error. Signaled when an underflow condition occurs on the transmit channel. This event occurs when a new transfer starts but SPI_TFIFO is empty. This error does not occur in master transmit initiating mode since SPI_TFIFO Not Empty is one of the conditions for transfer initiation. |

Table 16-16: SPI Error Interrupts (Continued)

| Bit          | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| SPI_STAT.ROR | Reception error. Signaled when an overflow condition occurs on the receive channel. This event occurs when a new data word is received, but the SPI_RFIFO is full. This error condition does not occur in master receive initiating mode since SPI_RFIFO Not Full is one of the conditions for transfer initiation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| SPI_STAT.TC  | Transmit collision error. Signaled when loading data to the transmit shift register happens near the first trans- mitting edge of SPI_CLK . In slave mode of operation, the SPI controller is unaware of when the next trans- fer starts. Loading of data to the transmit shift register can happen just after the transmitting edge. This event results in the setup time not being met for the first bit transmitted. The transmitted data is corrupt. In SPI_CTL.CPHA 1 mode, the first SPI_CLK edge is taken as the first transmitting edge. If SPI_CTL.CPHA =0, then the last SPI_CLK edge of the last transmission ( SPI_CTL.SELST =1) or slave select deassertion ( SPI_CTL.SELST =0) is taken as the first transmitting edge. This error is signaled only in the slave mode of operation. In master mode of operation, loading of data happens before the first transmit- ting edge of SPI_CLK . |

## SPI Programming Concepts

The following sections provide general programming guidelines and procedures.

## Programming Guidelines

It is acceptable to program SPI\_RXCTL and SPI\_TXCTL registers after programming the SPI\_CTL register. However, program the initiating mode register and its counter-register, if enabled, after the non-initiating mode register. For example, if transmit is the initiating mode and receive is the non-initiating mode, then program the SPI\_RXCTL and SPI\_RWC registers before the SPI\_TXCTL and SPI\_TWC registers. If enabling both transmit and receive in initiating mode, enable the SPI\_CTL register after programming both the SPI\_RXCTL and SPI\_TXCTL registers.

These programming guidelines prevent SPI from starting a transfer when SPI registers are not fully programmed. Other ways of programming are also allowed as long as the initiating conditions prevent the start of communication until after programming of SPI registers is complete.

Take precautions to avoid data corruption when changing the SPI module configuration. Do not change the configuration during a data transfer. Additionally, change the clock polarity only when no slaves are selected. However, an exception to this rule exists. When an SPI communication link consists of a single master and slave, SPI\_CTL.ASSEL = 0. The slave select input of the slave is permanently tied low. In this case, the slave is always selected. Avoid data corruption by enabling the slave only after both the master and slave devices are configured.

The module supports 8, 16-bit and 32-bit word sizes. To ensure correct operation, configure both the master and slave with the same word size.

## Master Operation in Non-DMA Modes

This section describes the operation of the SPI as a master in non-DMA mode.

1. Write to the SPI\_SLVSEL register, setting one or more of the SPI select enable bits. This operation ensures that the desired slaves are properly deselected while the master is configured.
2. The SPI\_RXCTL.RTI and SPI\_TXCTL.TTI bits determine the SPI initiating mode. The initiating mode defines the primary transfer channel, and also the initiating condition for the transfer.
3. Write to the SPI\_CLK , SPI\_CTL , SPI\_RXCTL , and SPI\_TXCTL registers. This operation enables the device as a master and configures the SPI system. It specifies the transfer modes and channels, appropriate word length, transfer format, baud rate, and other control information.
4. ADDITIONAL INFORMATION: If SPI\_RXCTL.RTI is enabled and SPI\_TXCTL.TTI is not, write to the SPI\_RXCTL register after writing into SPI\_CTL , SPI\_TXCTL , and SPI\_TFIFO registers to prevent a transmit underrun for the first transfer.
4. If SPI\_CTL.ASSEL =0, activate the desired slaves by clearing one or more of the SPI\_SLVSEL flag bits. Otherwise, the SPI hardware performs slave activation.
5. The SPI controller then generates the programmed clock pulses on SPI\_CLK and simultaneously shifts data out of SPI\_MOSI while shifting data in from SPI\_MISO . Before a shift, the shift register is loaded with the contents of the SPI\_TFIFO register. At the end of the transfer, the contents of the shift register are loaded into SPI\_RFIFO .
6. Whenever the initiating conditions are satisfied, the SPI continues to send and receive words. If the transmit buffer remains empty or the receive buffer remains full, the device operates according to the states of the SPI\_TXCTL.TDU and SPI\_RXCTL.RDO bits.
7. It is possible to program a secondary channel in addition to the initiating channel. This feature allows usage of available channel resources for receives or transmits simultaneously with the initiating channel.

## Slave Operation in Non-DMA Modes

When a device is enabled as a slave in a non-DMA mode, a transition of the SPI\_SS select signal to the active state (low) triggers the the start of a transfer. Or, the first active edge of SPI\_CLK triggers the start, depending on the state of SPI\_CTL.CPHA bit. The interface operates in the following manner.

1. The core writes to the SPI\_CTL , SPI\_RXCTL , and SPI\_TXCTL registers. The operation defines the mode of the serial link to be the same as the mode setup in the SPI master.
2. To prepare for the data transfer, the core writes data to be transmitted into SPI\_TFIFO .
3. Once the SPI\_SS falling edge is detected, the slave starts sending data on active SPI\_CLK edges and sampling data on inactive SPI\_CLK edges.
4. Reception or transmission continues until SPI\_SS is released or until the slave has received the proper number of clock cycles.
5. The slave device continues to receive or transmit with each new falling edge transition on SPI\_SS or active SPI\_CLK edge. If the transmit buffer remains empty or the receive buffer remains full, the device operates according to the states of the SPI\_TXCTL.TDU and SPI\_RXCTL.RDO bits.

## Configuring DMA Master Mode

The SPI interface supports a write DMA channel and a read DMA channel. It can use these functions individually or in a lock-step manner in duplex mode ( SPI\_TXCTL.TTI = SPI\_RXCTL.RTI =1).

1. Write to the appropriate DMA registers to enable the SPI DMA channel and to configure the necessary work units, access direction, word count, and so on.
2. Write to the SPI\_SLVSEL register, setting one or more of the SPI flag select bits.
3. Write to the SPI\_CLK and SPI\_CTL registers, enabling the device as a master and configuring the SPI system by specifying the appropriate word length, transfer format, baud rate, and so forth.
4. Write to SPI\_RXCTL to configure SPI master receive mode, or write to SPI\_TXCTL to configure SPI master transmit mode.
5. Finally, write to the SPI\_RXCTL.REN bit to enable the receive channel, or write to SPI\_TXCTL.TEN to enable the transmit channel.
6. If the SPI\_RXCTL.RTI bit is enabled, a receive transfer is initiated upon enabling SPI\_CTL.EN bit. If the receive word counter is enabled ( SPI\_RXCTL.RWCEN ), then the SPI\_RWC register must be non-zero for a transfer to initiate.

ADDITIONAL INFORMATION: If enabling both receive and transmit DMA channels, but not enabling SPI\_TXCTL.TTI , write to the SPI\_RXCTL register after writing the SPI\_CTL and SPI\_TXCTL registers. In this way, a transmit underrun can be prevented for the first transfer. Subsequent transfers are initiated as the SPI reads data from the receive shift register and writes to the SPI receive FIFO. The SPI then requests a write from DMA to memory. Upon a DMA grant, the DMA engine reads a word from the SPI receive FIFO and writes to memory. New requests continue to be initiated as long as the receive FIFO does not fill up, when SPI\_RWC does not become zero while SPI\_RXCTL.RWCEN =1.

7. If SPI\_TXCTL.TTI is enabled, the SPI controller requests DMA reads from memory as long as there is space for more data in the transmit pipe. Upon a DMA grant, the DMA engine reads a word from memory and writes to the transmit FIFO. As long as transmit data is available in the FIFO, and the SPI\_TWC register is non-zero when SPI\_TXCTL.TWCEN =1, the SPI continues to initiate transfers until disabled.
8. If both the SPI\_TXCTL.TTI and SPI\_RXCTL.RTI bits are enabled, the SPI controller requests a DMA read from memory. However, there must be space for more data in the transmit pipe and the number of words written into the SPI must be less than SPI\_TWC if SPI\_TXCTL.TWCEN =1. Upon a DMA grant, the DMA engine reads a word from memory and writes to the transmit FIFO.

ADDITIONAL INFORMATION: As the SPI writes data from the transmit FIFO into the transmit shift register, it initiates a transfer on the SPI link. Data received from the transfer is moved from the SPI receive shift register to the receive FIFO. The SPI controller requests a write from DMA to memory. Upon a DMA grant, the DMA engine reads a word from the receive FIFO and writes to memory. Transfer continues to be initiated as long as both receives and transmits can accommodate new data.

9. If the receive pipe fills up due to unavailability of DMA grants, the transmit pipe stalls until the pipe is drained. If the transmit pipe fills up, the SPI stops requesting for DMA writes. If the value in SPI\_RWC expires, further write-requests to DMA stop. However, data already written into the transmit FIFO is sent, and read requests to DMA continue until the receive data is read from the receive FIFO.
10. The SPI then generates the programmed clock pulses on SPI\_CLK and simultaneously shifts data out of SPI\_MOSI while shifting data in from SPI\_MISO . For receive transfers, the value in the shift register is loaded into the SPI\_RFIFO register at the end of the transfer. For transmit transfers, the value in the SPI\_TFIFO register is loaded into the shift register at the start of the transfer.

## Configuring DMA Slave Mode Operation

This mode occurs when the SPI is enabled as a slave and the DMA engine is configured to transmit or receive data. A transition of the SPI\_SS signal to the active-low state triggers the start of a transfer. Or, the first active edge of SPI\_CLK triggers the start of a transfer, depending on the state of the SPI\_CTL.CPHA bit. The following steps illustrate the SPI receive or transmit DMA sequence in an SPI slave (in response to a master command). The SPI supports a receive DMA channel and a transmit DMA channel.

1. Write to the appropriate DMA registers to enable the SPI DMA channel and configure the necessary work units, access direction, word count, and so on.
2. Write to the SPI\_CTL , SPI\_RXCTL , and SPI\_TXCTL registers to define the mode of the serial link to be the same as the mode configured in the SPI master.
3. If the receive channel is enabled ( SPI\_RXCTL.REN is asserted), the following actions occur:
- a. Once the slave select input is active, the slave starts receiving and transmitting data on active SPI\_CLK edges.
- b. The value in the shift register is loaded into the SPI\_RFIFO register at the end of the transfer.
- c. Once SPI\_RFIFO has valid data, it requests a write from DMA to memory.
- d. Upon a DMA grant, the DMA engine reads a word from the receive FIFO and writes to memory.
- e. As long as there is data in the receive FIFO, the SPI slave continues to request a DMA write to memory. The DMA engine continues to read a word from the FIFO and writes to memory until the SPI\_RWC counts to zero. The SPI slave continues receiving words on active SPI\_CLK edges as long as the SPI\_SS input is active.
- f. If the data collected in the receive pipe breaches the set level, and the DMA engine cannot keep up with the receive rate, the slave can deassert the SPI\_RDY signal. This signaling throttles the master. The receive pipe level is set according to the SPI\_CTL.FCWM field. The signal is deasserted as the DMA drains the receive FIFO. Alternatively, the SPI can use the SPI\_RXCTL.RDO bit to decide when the incoming data is discarded or overwritten into the receive FIFO (when SPI\_CTL.FCEN is inactive).
4. If the transmit channel is enabled ( SPI\_TXCTL.TEN is asserted), the following actions occur:
- a. The SPI requests a DMA read from memory.

- b. Upon a DMA grant, the DMA engine reads a word from memory and writes to the transmit FIFO.
- c. The SPI then reads DMA data from the transmit FIFO and writes to the transmit shift register, awaiting the start of the next transfer.
- d. Once the slave select input is active, the slave starts receiving and transmitting data on active SPI\_CLK edges.
- e. As long as there is room in the transmit FIFO, the SPI slave continues to request a DMA read from memory. The DMA engine continues to read a word from memory and write to the transmit FIFO until the SPI\_TWC register value counts down to 0. The SPI slave continues transmitting words on active SPI\_CLK edges as long as the SPI\_SS input is active.
- f. If the number of outstanding data entries in the transmit pipe breaches the level set and the DMA cannot keep up with the transmit rate, the slave deasserts the SPI\_RDY signal. This signaling throttles the master. The transmit pipe level is set according to the SPI\_CTL.FCWM field. The signal is deasserted as the DMA fills the transmit FIFO. Alternately, the SPI\_TXCTL.TDU bit decides the state of the transmit data (when SPI\_CTL.FCEN is deasserted).
5. If both receive and transmit channels are enabled, the following actions occur after the actions for each channel. T ransfers continue as long as both receive and transmit channels can accommodate new data.
- a. If the receive pipe fills up due to the unavailability of DMA grant, the SPI interface stalls the master by asserting the SPI\_RDY pin. This signal is deasserted as the DMA drains the receive FIFO. Alternately, the SPI uses the SPI\_RXCTL.RDO bit to decide when the incoming data is discarded or overwritten in the receive FIFO (when SPI\_CTL.FCEN is deasserted).
- b. If the transmit pipe fills up, the SPI stops requesting DMA writes until the pipe clears.
- c. If there is an underflow problem in the transmit pipe, the slave stalls the master by deasserting SPI\_RDY while the DMA fills the transmit FIFO. Alternately, the SPI uses the SPI\_TXCTL.TDU bit to decide the state of the transmit data (when SPI\_CTL.FCEN is deasserted).

## ADSP-SC58x SPI Register Descriptions

Serial Peripheral Interface (SPI) contains the following registers.

Table 16-17: ADSP-SC58x SPI Register List

| Name         | Description                         |
|--------------|-------------------------------------|
| SPI_CLK      | Clock Rate Register                 |
| SPI_CTL      | Control Register                    |
| SPI_DLY      | Delay Register                      |
| SPI_ILAT     | Masked Interrupt Condition Register |
| SPI_ILAT_CLR | Masked Interrupt Clear Register     |

Table 16-17: ADSP-SC58x SPI Register List (Continued)

| Name         | Description                              |
|--------------|------------------------------------------|
| SPI_IMSK     | Interrupt Mask Register                  |
| SPI_IMSK_CLR | Interrupt Mask Clear Register            |
| SPI_IMSK_SET | Interrupt Mask Set Register              |
| SPI_MMRDH    | Memory Mapped Read Header (Only on SPI2) |
| SPI_MMTOP    | SPI Memory Top Address (Only on SPI2)    |
| SPI_RFIFO    | Receive FIFO Data Register               |
| SPI_RWC      | Received Word Count Register             |
| SPI_RWCR     | Received Word Count Reload Register      |
| SPI_RXCTL    | Receive Control Register                 |
| SPI_SLVSEL   | Slave Select Register                    |
| SPI_STAT     | Status Register                          |
| SPI_TFIFO    | Transmit FIFO Data Register              |
| SPI_TWC      | Transmitted Word Count Register          |
| SPI_TWCR     | Transmitted Word Count Reload Register   |
| SPI_TXCTL    | Transmit Control Register                |

## Clock Rate Register

The SPI\_CLK register selects the baud rate for SPI data transfers, relating this rate to the SPI serial clock (SPI clock) and the system clock (SCLK1\_0).

Figure 16-20: SPI\_CLK Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000017_3d02f6e3c6a3d63759f6afe6ac0d81e7b27cd98977b220e2b604a013ab032d84.png)

Table 16-18: SPI\_CLK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | BAUD       | Baud Rate. The SPI_CLK.BAUD bits set the SPI baud rate according to the formula: BAUD = (SCLK1_0 / SPI Clock) - 1 |

## Control Register

The SPI\_CTL register enables the SPI and configures settings for operating modes, communication protocols, and buffer operations.

Figure 16-21: SPI\_CTL Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000018_550044b5943e9c3dd6cc8280cab7f0ba5c1adadc16cac149e2e544651e14c350.png)

Table 16-19: SPI\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | MMSE       | Memory-Mapped SPI Enable (Only on SPI2). When the SPI_CTL.MMSE bit is asserted, communication to an SPI memory device is automated such that the memory it contains is accessible directly through the read of processor address space assigned to it. (As far as the SPI peripheral is concerned, this includes all read accesses received by the SPI peripherals system crossbar slave port.) Note that when memory-mapped access of SPI memory is enabled, attempts to com- municate with the SPI device using legacy methods are blocked and receive fabric error responses are generated. Legacy methods include any direct access made to the Tx and Rx FIFOs, whether initiated by DMAor processor MMRaccess.                                    | Memory-Mapped SPI Enable (Only on SPI2). When the SPI_CTL.MMSE bit is asserted, communication to an SPI memory device is automated such that the memory it contains is accessible directly through the read of processor address space assigned to it. (As far as the SPI peripheral is concerned, this includes all read accesses received by the SPI peripherals system crossbar slave port.) Note that when memory-mapped access of SPI memory is enabled, attempts to com- municate with the SPI device using legacy methods are blocked and receive fabric error responses are generated. Legacy methods include any direct access made to the Tx and Rx FIFOs, whether initiated by DMAor processor MMRaccess.                                    |
| 31 (R/W)           | MMSE       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Hardware automated access of memory-mapped SPI memory disabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 31 (R/W)           | MMSE       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Hardware-automated access of memory-mapped SPI memory enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 30 (R/W)           | MMWEM      | Memory Mapped Write Error Mask (Only on SPI2). The SPI_CTL.MMWEM bit specifies whether an error response is returned to the fab- ric upon write attempts to address space reserved for memory-mapped reads of SPI memory.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Memory Mapped Write Error Mask (Only on SPI2). The SPI_CTL.MMWEM bit specifies whether an error response is returned to the fab- ric upon write attempts to address space reserved for memory-mapped reads of SPI memory.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 30 (R/W)           | MMWEM      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Write error response returned upon write attempts to memory-mapped SPI memory                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 30 (R/W)           | MMWEM      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Write error response masked (not returned) upon write attempts to memory-mapped SPI memory                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 22 (R/W)           | SOSI       | Start on MOSI (Only on SPI2). The SPI_CTL.SOSI bit is valid only when SPI_CTL.MIOM is enabled for either DIOM or QIOM, and this bit selects the starting pin and the bit placement on pins for these modes. In DIOM, by default, ( SPI_CTL.SOSI =0) SPI sends the first bit on the SPI_MISO pin and the second bit on the SPI_MOSI pin. In QIOM, by default, the SPI sends the first bit on the SPI_D3 pin, the second bit on the SPI_D2 pin, the third bit on the SPI_MISO pin and the fourth bit on the SPI_MOSI pin. This order can be reversed by setting the SPI_CTL.SOSI bit. When this bit is set, the SPI sends the first bit on the SPI_MOSI pin. The first bit referred to here depends on the SPI_CTL.LSBF bit setting (MSB bit or LSB bit). | Start on MOSI (Only on SPI2). The SPI_CTL.SOSI bit is valid only when SPI_CTL.MIOM is enabled for either DIOM or QIOM, and this bit selects the starting pin and the bit placement on pins for these modes. In DIOM, by default, ( SPI_CTL.SOSI =0) SPI sends the first bit on the SPI_MISO pin and the second bit on the SPI_MOSI pin. In QIOM, by default, the SPI sends the first bit on the SPI_D3 pin, the second bit on the SPI_D2 pin, the third bit on the SPI_MISO pin and the fourth bit on the SPI_MOSI pin. This order can be reversed by setting the SPI_CTL.SOSI bit. When this bit is set, the SPI sends the first bit on the SPI_MOSI pin. The first bit referred to here depends on the SPI_CTL.LSBF bit setting (MSB bit or LSB bit). |
| 22 (R/W)           | SOSI       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Start on MISO (DIOM) or start on SPI_D3 (Only on SPI2)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 22 (R/W)           | SOSI       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Start on MOSI                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 16-19: SPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21:20 (R/W)        | MIOM       | Multiple I/O Mode (Only on SPI2). The SPI_CTL.MIOM bits enable SPI operation in dual I/O mode (DIOM) or quad I/O mode (QIOM). These bits can only be changed when the SPI is disabled ( SPI_CTL.EN =0).                                                                                                                                                                                                      |
| 21:20 (R/W)        | MIOM       | 0 No MIOM (disabled)                                                                                                                                                                                                                                                                                                                                                                                         |
| 21:20 (R/W)        | MIOM       | 1 DIOM operation                                                                                                                                                                                                                                                                                                                                                                                             |
| 21:20 (R/W)        | MIOM       | 2 QIOM operation (Only on SPI2)                                                                                                                                                                                                                                                                                                                                                                              |
| 21:20 (R/W)        | MIOM       | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                                                   |
| 18 (R/W)           | FMODE      | Fast-Mode Enable. The SPI_CTL.FMODE bit enables fast mode operation for SPI receive transfers. SPI transmit operations in fast mode are the same as normal mode.                                                                                                                                                                                                                                             |
| 18 (R/W)           | FMODE      | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                    |
| 18 (R/W)           | FMODE      | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                     |
| 17:16 (R/W)        | FCWM       | Flow Control Watermark. The SPI_CTL.FCWM bits select the watermark level of the transmit channel ( SPI_TFIFO buffer) or receive channel ( SPI_RFIFO buffer) that triggers flow con- trol operation. These bits are applicable only when the SPI is a slave ( SPI_CTL.MSTR = 0) and flow control is enabled ( SPI_CTL.FCEN =1). When the watermark condition is met, the SPI slave deasserts the SPI_RDY pin. |
| 17:16 (R/W)        | FCWM       | 0 TFIFO empty or RFIFO full                                                                                                                                                                                                                                                                                                                                                                                  |
| 17:16 (R/W)        | FCWM       | 1 TFIFO 75% or more empty, or RFIFO 75% or more full                                                                                                                                                                                                                                                                                                                                                         |
| 17:16 (R/W)        | FCWM       | 2 TFIFO 50% or more empty, or RFIFO 50% or more full                                                                                                                                                                                                                                                                                                                                                         |
| 17:16 (R/W)        | FCWM       | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                                                   |
| 15 (R/W)           | FCPL       | Flow Control Polarity. The SPI_CTL.FCPL bit selects flow control polarity for the SPI_RDY pin when flow control is enabled. When the SPI_RDY pin is active, the SPI is indicating it is ready for data transfer.                                                                                                                                                                                             |
| 15 (R/W)           | FCPL       | 0 Active-low RDY                                                                                                                                                                                                                                                                                                                                                                                             |
| 15 (R/W)           | FCPL       | 1 Active-high RDY                                                                                                                                                                                                                                                                                                                                                                                            |

Table 16-19: SPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | FCCH       | Flow Control Channel Selection. The SPI_CTL.FCCH bit selects whether the SPI applies flow control to the transmit channel ( SPI_TFIFO buffer) or receive channel ( SPI_RFIFO buffer). This bit is applicable only when the SPI is a slave and flow control is enabled. |
| 14 (R/W)           | FCCH       | 0 Flow control on RX buffer                                                                                                                                                                                                                                            |
| 14 (R/W)           | FCCH       | 1 Flow control on TX buffer                                                                                                                                                                                                                                            |
| 13 (R/W)           | FCEN       | Flow Control Enable. The SPI_CTL.FCEN bit enables SPI flow control operation, which permits slow slave devices to interface with fast master devices. This bit controls the operation of the SPI_RDY pin.                                                              |
| 13 (R/W)           | FCEN       | Note that options for flow control operation are available using the SPI_CTL.FCCH , SPI_CTL.FCPL , and SPI_CTL.FCWM bits.                                                                                                                                              |
| 13 (R/W)           | FCEN       | 0 Disable                                                                                                                                                                                                                                                              |
| 13 (R/W)           | FCEN       | 1 Enable                                                                                                                                                                                                                                                               |
| 12 (R/W)           | LSBF       | Least Significant Bit First. The SPI_CTL.LSBF bit selects whether the SPI transmits/receives data as LSB first (little endian) or MSB first (big endian). This bit can only be changed when the SPI is disabled.                                                       |
| 12 (R/W)           | LSBF       | 0 MSB sent/received first (big endian)                                                                                                                                                                                                                                 |
| 12 (R/W)           | LSBF       | 1 LSB sent/received first (little endian)                                                                                                                                                                                                                              |
| 10:9 (R/W)         | SIZE       | Word Transfer Size.                                                                                                                                                                                                                                                    |
| 10:9 (R/W)         | SIZE       | 0 8-bit word                                                                                                                                                                                                                                                           |
| 10:9 (R/W)         | SIZE       | 1 16-bit word                                                                                                                                                                                                                                                          |
| 10:9 (R/W)         | SIZE       | 2 32-bit word                                                                                                                                                                                                                                                          |
| 10:9 (R/W)         | SIZE       | 3 Reserved                                                                                                                                                                                                                                                             |
| 8 (R/W)            | EMISO      | Enable MISO. The SPI_CTL.EMISO bit enables master-in-slave-out (MISO) mode. This SPI mode is applicable only when the SPI is a slave.                                                                                                                                  |
| 8 (R/W)            | EMISO      | 0 Disable                                                                                                                                                                                                                                                              |
| 8 (R/W)            | EMISO      | 1 Enable                                                                                                                                                                                                                                                               |

Table 16-19: SPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | SELST      | Slave Select Polarity Between Transfers. The SPI_CTL.SELST bit selects the state (polarity) for the SPI_SEL[n] pin be- tween SPI transfers when the SPI is a master and hardware slave select assertion is ena- bled ( SPI_CTL.ASSEL =1). In slave mode, this bit affects the detection of both transmit collision ( SPI_STAT.TC and underrun ( SPI_STAT.TUR ) errors.                                                                                                                                                                                                                                                                                                                                                                          |
| 7 (R/W)            | SELST      | 0 Deassert slave select (high)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 7 (R/W)            | SELST      | 1 Assert slave select (low)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 6 (R/W)            | ASSEL      | Slave Select Pin Control. The SPI_CTL.ASSEL bit selects whether the SPI hardware sets the SPI_SEL[n] pin output value (ignoring the slave select SPI_SLVSEL.SSEL1 - SPI_SLVSEL.SSEL7 bits) or whether software control of the slave select bits set the SPI_SEL[n] pin output value. This feature is applicable only when the SPI is a master. When hardware control is enabled, the SPI_SEL[n] pin output is asserted during the transfers, and the pin polarity between transfers is selected by the SPI_CTL.SELST bit. When software control is enabled, the SPI_SEL[n] pin output value is set through software control of the slave select bits, and as such, the pin may either remain asserted (low) or be deasserted between transfers. |
| 6 (R/W)            | ASSEL      | 0 Software slave select control                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 5 (R/W)            | CPOL       | 1 Hardware slave select control Clock Polarity. The SPI_CTL.CPOL bit selects whether the SPI uses an active-low or active-high signal for the SPI clock ( SPI_CLK ). This bit works with the SPI_CTL.CPHA bit to select combinations of clock phase and polarity for the SPI_CLK pin. This bit can only be changed when the SPI is disabled.                                                                                                                                                                                                                                                                                                                                                                                                    |
| 5 (R/W)            | CPOL       | 0 Active-high SPI CLK                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 5 (R/W)            | CPOL       | 1 Active-low SPI CLK                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 4 (R/W)            | CPHA       | Clock Phase. The SPI_CTL.CPHA bit selects whether the SPI starts toggling the signal for the SPI clock ( SPI_CLK ) from the start of the first data bit or from the middle of the first data bit. The SPI_CTL.CPHA bit works with the SPI_CTL.CPOL bit to select combinations of clock phase and polarity for the SPI_CLK pin. This bit can only be                                                                                                                                                                                                                                                                                                                                                                                             |
| 4 (R/W)            | CPHA       | 0 SPI CLK toggles from middle                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 4 (R/W)            | CPHA       | 1 SPI CLK toggles from start                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

Table 16-19: SPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | ODM        | Open Drain Mode. The SPI_CTL.ODM bit configures the data output pins ( SPI_MOSI and SPI_MISO ) to behave as open drain outputs, which prevents contention and possible damage to pin drivers in multi-master or multi-slave SPI systems. When SPI_CTL.ODM is enabled and the SPI is a master, the SPI three-states the SPI_MOSI pin when the data driven out on MOSI is a logic-high. The SPI does not three-state the SPI_MOSI pin when the driven data is a logic-low. When SPI_CTL.ODM is enabled and the SPI is a slave, the SPI three-states the SPI_MISO pin when the data driven out on SPI_MISO is a logic-high. Note that an external pull-up resistor is required on both the SPI_MOSI and SPI_MISO pins when SPI_CTL.ODM is enabled. 0 Disable |
| 2 (R/W)            | PSSE       | Protected Slave Select Enable. The SPI_CTL.PSSE bit enables the SPI_SS pin to provide error detection input in a multi-master environment when the SPI is in master mode. If some other device in the system asserts the SPI_SS pin while SPI is enabled as master (and SPI_CTL.PSSE is enabled), this condition causes a mode fault error.                                                                                                                                                                                                                                                                                                                                                                                                               |
| 1 (R/W)            | MSTR       | Master/Slave. The SPI_CTL.MSTR bit toggles the SPI between master mode and slave mode. This bit can only be changed when the SPI is disabled. 0 Slave                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 0 (R/W)            | EN         | Enable. The SPI_CTL.EN bit enables SPI operation. 0 Disable SPI module                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

## Delay Register

The SPI\_DLY register selects a transfer delay and the lead/lag timing between slave select signals and SPI clock edge assertion/deassertion.

Figure 16-22: SPI\_DLY Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000019_96c9ccfb5ad3a206bbc130d952417f1bb0c48ba633960ef24a956ec2dea80ce3.png)

Table 16-20: SPI\_DLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | LAGX       | Extended SPI Clock Lag Control. The SPI_DLY.LAGX bit enables insertion of a 1-SPI_CLK cycle lag (extend lag) in the timing between the slave select ( SPI_SEL[n] ) assertion and first SPI clock edge.                                                                                                                                                                                                                     |
| 8 (R/W)            | LEADX      | Extended SPI Clock Lead Control. The SPI_DLY.LEADX bit enables insertion of a 1-SPI_CLK cycle lead (extend lead) in the timing between the slave select ( SPI_SEL[n] ) deassertion and last SPI clock edge.                                                                                                                                                                                                                |
| 7:0 (R/W)          | STOP       | Transfer Delay Time in Multiples of SPI Clock Period. The SPI_DLY.STOP bits select a delay (number of stop bits in multiples of SPI clock duration) at the end of each SPI transfer. The default delay is the minimum val- ue required to comply with the SPI protocol (1-bit duration). The SPI_DLY.STOP bits can be programmed with smaller delay values, resulting in continuous operation (for example, stop bits =0). |

## Masked Interrupt Condition Register

The SPI\_ILAT register latches interrupts, queuing the interrupt requests for service. When a condition is indicated by a bit in the SPI\_STAT register and the corresponding interrupt request is unmasked in SPI\_IMSK , the SPI latches the interrupt request bit in SPI\_ILAT .

Figure 16-23: SPI\_ILAT Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000020_6a6fb9df45e96c5effb4eacb21a3fbf5ab1c80e3d3b6225dfbca737051b4ce30.png)

Table 16-21: SPI\_ILAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 11                 | TF         | Transmit Finish Interrupt Latch.                                         |
| 10 (R/NW)          | RF         | Receive Finish Interrupt Latch. 0 No interrupt request                   |
| 9 (R/NW)           | TS         | Transmit Start Interrupt Latch. 0 No interrupt request interrupt request |
| 8                  | RS         | 1 Latched Receive Start Interrupt Latch. 0 No                            |
| (R/NW)             |            | interrupt request 1 Latched interrupt request                            |

Table 16-21: SPI\_ILAT Register Fields (Continued)

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

Figure 16-24: SPI\_ILAT\_CLR Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000021_dda16b9d69fdb432d72308b3e0a0a4ffddaeea53662f7355c8deb46ec3b7ae5b.png)

Table 16-22: SPI\_ILAT\_CLR Register Fields

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

Table 16-22: SPI\_ILAT\_CLR Register Fields (Continued)

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

Table 16-22: SPI\_ILAT\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | RUWM       | Clear Receive Urgent Watermark. The SPI_ILAT_CLR.RUWM bit clears the corresponding mask bit in the SPI_ILAT register. |

## Interrupt Mask Register

The SPI\_IMSK register unmasks (enables) or masks (disables) SPI interrupt requests. When a condition is indicated by a bit in the SPI\_STAT register and the corresponding interrupt request is unmasked in SPI\_IMSK , the SPI latches the interrupt request bit in the SPI\_ILAT register, queuing the interrupt request for service.

Figure 16-25: SPI\_IMSK Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000022_61500b9f08a188079394124eabaafd75cf5575abd00e81a4b8bb2f782213a837.png)

Table 16-23: SPI\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/NW)          | TF         | Transmit Finish. The SPI_IMSK.TF bit unmasks (enables) or masks (disables) the TF interrupt. 0 Disable (mask) interrupt request 1 Enable (unmask) interrupt request |
| 10 (R/NW)          | RF         | Receive Finish. The SPI_IMSK.RF bit unmasks (enables) or masks (disables) the RF interrupt. 0 Disable (mask) interrupt request 1 Enable (unmask) interrupt request  |
| 9 (R/NW)           | TS         | Transmit Start. The SPI_IMSK.TS bit unmasks (enables) or masks (disables) the TS interrupt. 0 Disable (mask) interrupt request 1 Enable (unmask) interrupt request  |

Table 16-23: SPI\_IMSK Register Fields (Continued)

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

Figure 16-26: SPI\_IMSK\_CLR Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000023_1acad9c56360bff6b91c0a3b9106440d5ec8a7b7b11a7f26de5a99a575bfd909.png)

Table 16-24: SPI\_IMSK\_CLR Register Fields

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

Table 16-24: SPI\_IMSK\_CLR Register Fields (Continued)

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

Table 16-24: SPI\_IMSK\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | RUWM       | Clear Receive Urgent Watermark. The SPI_IMSK_CLR.RUWM bit clears the corresponding mask bit in the SPI_IMSK register. |
| 1 (R/W1C)          | RUWM       | 0 No effect                                                                                                           |
| 1 (R/W1C)          | RUWM       | 1 Clear mask bit                                                                                                      |

## Interrupt Mask Set Register

The SPI\_IMSK\_SET register permits setting individual mask bits in the SPI\_IMSK register without affecting other bits in the register. Use write-1-to-set on a bit in the SPI\_IMSK\_SET register to set the corresponding bit in the SPI\_IMSK register.

Figure 16-27: SPI\_IMSK\_SET Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000024_594795df236ced3b2b2a2e086dd8c2fefaf849b5fdea5554fc9f6c8b8857003b.png)

Table 16-25: SPI\_IMSK\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 11 (R/W1S)         | TF         | Set Transmit Finish. The SPI_IMSK_SET.TF bit sets the corresponding mask bit in the SPI_IMSK register.            |
| 10 (R/W1S)         | RF         | Set Receive Finish. The SPI_IMSK_SET.RF bit sets the corresponding mask bit in the SPI_IMSK register.             |
| 9 (R/W1S)          | TS         | Set Transmit Start. The SPI_IMSK_SET.TS bit sets the corresponding mask bit in the SPI_IMSK register. 0 No effect |

Table 16-25: SPI\_IMSK\_SET Register Fields (Continued)

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

Table 16-25: SPI\_IMSK\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1S)          | RUWM       | Set Receive Urgent Watermark. The SPI_IMSK_SET.RUWM bit sets the corresponding mask bit in the SPI_IMSK register. |
| 1 (R/W1S)          |            | 0 No effect                                                                                                       |
| 1 (R/W1S)          |            | 1 Set mask bit                                                                                                    |

## Memory Mapped Read Header (Only on SPI2)

The SPI\_MMRDH register enables the use of memory-mapped mode. This mode allows direct memory-mapped read accesses of an SPI memory device and is primarily used to directly execute instructions from an SPI FLASH memory without using a low-level software driver. All overhead tasks such as transmission of the read header, pin turnaround timing and receive data sizing are handled in hardware.

The memory-mapped access mode is enabled by setting the SPI\_CTL.MMSE bit. The features within the SPI\_MMRDH register include a command skip mode, variable length byte addressing, and independent multi-pin support for command transmission, address transmission and data reception. In addition, the command opcode and mode bytes are fully programmable.

Figure 16-28: SPI\_MMRDH Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000025_bf34b8419778fafdc72088d2d001f84ba2b79ba124710cecd1121dda9d1104db.png)

Table 16-26: SPI\_MMRDH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | CMDPINS    | Pins Used for Command. The SPI_MMRDH.CMDPINS bit specifies the number of pins to be used for com- mand transmission. This bit must be set consistent with the expectations established by the read opcode. Hardware does not interpret SPI_MMRDH.OPCODE , but rather relies on this bit to specify behavior. When cleared, it overrides the SPI_CTL.MIOM bits. When set, it uses bits specified by the SPI_CTL.MIOM bit setting. |
| 29 (R/W)           | CMDPINS    | 0 Use only one pin: MOSI (overrides SPI_CTL.MIOM bits)                                                                                                                                                                                                                                                                                                                                                                           |
| 29 (R/W)           | CMDPINS    | 1 Use pins specified by SPI_CTL.MIOM bits                                                                                                                                                                                                                                                                                                                                                                                        |

Table 16-26: SPI\_MMRDH Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | CMDSKIP    | Command Skip Enable. The SPI_MMRDH.CMDSKIP bit enables command skip mode where the address is sent first and the OPCODE field is not sent ( SPI_MMRDH.CMDSKIP bit =1). This mode is useful for supporting XIP (Execute-In-Place) operation where only the address is sent and the same read command is assumed. The SPI flash device must be primed with an initial read command, before the SPI_MMRDH.CMDSKIP bit is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Command Skip Enable. The SPI_MMRDH.CMDSKIP bit enables command skip mode where the address is sent first and the OPCODE field is not sent ( SPI_MMRDH.CMDSKIP bit =1). This mode is useful for supporting XIP (Execute-In-Place) operation where only the address is sent and the same read command is assumed. The SPI flash device must be primed with an initial read command, before the SPI_MMRDH.CMDSKIP bit is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 28 (R/W)           | CMDSKIP    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | OPCODE field is sent first followed by address                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 27 (R/W)           | WRAP       | 1 OPCODE field is not sent; address is sent first SPI Memory Wrap Indicator. The SPI_MMRDH.WRAP bit must be set by software if software places a connected SPI memory device into a 8-byte, 16-byte or 32-byte wrap mode based on the ILINE and DLINE field setting of the cache configuration register address wrap mode. Soft- ware achieves this by transmitting a vendor specified command to the SPI memory de- vice while the SPI_CTL.MMSE bit =0. If the SPI_MMRDH.WRAP bit =1, the SPI does not need to deassert the SPI slave se- lect signal and resend the read header in order to wrap to the cache line base when servicing misaligned cache fill requests. Although this improves cache fill efficiency, it requires that the SPI deassert the SPI slave select pin and resend the read header when- ever a DMAburst requests crosses 32 byte alignments. Setting this bit improves cache throughput but decreases DMAthroughput. | 1 OPCODE field is not sent; address is sent first SPI Memory Wrap Indicator. The SPI_MMRDH.WRAP bit must be set by software if software places a connected SPI memory device into a 8-byte, 16-byte or 32-byte wrap mode based on the ILINE and DLINE field setting of the cache configuration register address wrap mode. Soft- ware achieves this by transmitting a vendor specified command to the SPI memory de- vice while the SPI_CTL.MMSE bit =0. If the SPI_MMRDH.WRAP bit =1, the SPI does not need to deassert the SPI slave se- lect signal and resend the read header in order to wrap to the cache line base when servicing misaligned cache fill requests. Although this improves cache fill efficiency, it requires that the SPI deassert the SPI slave select pin and resend the read header when- ever a DMAburst requests crosses 32 byte alignments. Setting this bit improves cache throughput but decreases DMAthroughput. |
| 27 (R/W)           | WRAP       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | SPI Memory auto increments address purely sequential- ly                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 27 (R/W)           | WRAP       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | SPI Memory auto increments address but wraps within 32 Byte lines                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 26 (R/W)           | MERGE      | Merge Enable. When the SPI_MMRDH.MERGE bit is set, SPI hardware combines the two successive transfers. This increases the throughput rate when accessing a large number of sequen- tial memory locations. For more information refer to the Merged Read Accesses sec- tion.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Merge Enable. When the SPI_MMRDH.MERGE bit is set, SPI hardware combines the two successive transfers. This increases the throughput rate when accessing a large number of sequen- tial memory locations. For more information refer to the Merged Read Accesses sec- tion.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 16-26: SPI\_MMRDH Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:24 (R/W)        | TRIDMY     | Tristate Dummy Timing. The SPI_MMRDH.TRIDMY bits specify whether and when output pins are three- stated during the interval of time specified by the SPI_MMRDH.DMYSIZE bits. Out- put pins potentially three-stated include all pins which were used to transmit the ad-                                                                                                                                                                              | Tristate Dummy Timing. The SPI_MMRDH.TRIDMY bits specify whether and when output pins are three- stated during the interval of time specified by the SPI_MMRDH.DMYSIZE bits. Out- put pins potentially three-stated include all pins which were used to transmit the ad-                                                                                                                                                                              |
| 25:24 (R/W)        | TRIDMY     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Tristate outputs immediately                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 25:24 (R/W)        | TRIDMY     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Tristate outputs after 4 bits of dummy/mode are trans- mitted                                                                                                                                                                                                                                                                                                                                                                                         |
| 25:24 (R/W)        | TRIDMY     | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Tristate outputs after 8 bits of dummy/mode are trans- mitted                                                                                                                                                                                                                                                                                                                                                                                         |
| 25:24 (R/W)        | TRIDMY     | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Never tristate outputs (previously specified output state is held)                                                                                                                                                                                                                                                                                                                                                                                    |
| 23:16 (R/W)        | MODE       | Mode Field. These bits specify up to a leading byte to be transmitted during the interval of time specified by the SPI_MMRDH.DMYSIZE bit field. This first byte, or a portion of it, is interpreted as mode bits when certain opcodes are used in conjunction with certain SPI memory devices. Mode bits are sent using the same number of pins which were used to transmit the address. Once sent, output pins will be held in their final resultant | Mode Field. These bits specify up to a leading byte to be transmitted during the interval of time specified by the SPI_MMRDH.DMYSIZE bit field. This first byte, or a portion of it, is interpreted as mode bits when certain opcodes are used in conjunction with certain SPI memory devices. Mode bits are sent using the same number of pins which were used to transmit the address. Once sent, output pins will be held in their final resultant |
| 14:12 (R/W)        | DMYSIZE    | Bytes of Dummy/Mode. The SPI_MMRDH.DMYSIZE bit field specifies the number of bytes separating ad- dress transmission and read data return. Dummy bytes elapse assuming dummy bits are transmitted using the same number of pins which were used to transmit address.                                                                                                                                                                                  | Bytes of Dummy/Mode. The SPI_MMRDH.DMYSIZE bit field specifies the number of bytes separating ad- dress transmission and read data return. Dummy bytes elapse assuming dummy bits are transmitted using the same number of pins which were used to transmit address.                                                                                                                                                                                  |
| 14:12 (R/W)        | DMYSIZE    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 0 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | DMYSIZE    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 1 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | DMYSIZE    | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 2 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | DMYSIZE    | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 3 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | DMYSIZE    | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 4 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | DMYSIZE    | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 5 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | DMYSIZE    | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 6 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | DMYSIZE    | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 7 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 16-26: SPI\_MMRDH Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | ADRPINS    | Pins Used for Address. The SPI_MMRDH.ADRPINS bit specifies the number of pins to be used for address transmission. This bit must be set consistent with expectations established by read op- code. Hardware does not interpret the SPI_MMRDH.OPCODE , but rather relies on this bit to specify behavior.                                                                                                                                                                                                                                                                             |
| 10:8 (R/W)         | ADRSIZE    | Bytes of Read Address. The SPI_MMRDH.ADRSIZE bit field defines the number of bytes used to specify the read address. The read address is sent immediately following the transmission of op- code. Unlike opcode bits, address bits may be sent using either one or multiple pins. The number of pins is selected using the SPI_MMRDH.ADRPINS bit. The address sent to a connected SPI memory device is an echo of the read address received by the SPI peripheral slave port. The least significant bytes of address are sent when the entire address is not sent. 0 1 Byte 1 1 Byte |
| 7:0 (R/W)          | OPCODE     | Read Opcode. The SPI_MMRDH.OPCODE bit field specifies the initial bits transmitted in response to a read request of SPI memory. Although any opcode may be sent, values 0x03, 0x0B, 0x3B, 0x6B, 0xBB, and 0xEB are likely to be the most commonly used. SPI_MMRDH.OPCODE is sent by the SPI without interpretation; the states of these bits have no effect beyond specifying what is initially shifted across the SPI interface.                                                                                                                                                    |

## SPI Memory Top Address (Only on SPI2)

The SPI\_MMTOP register specifies the top populated address of a connected SPI memory device.

Figure 16-29: SPI\_MMTOP Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000026_b04cbc2d602a4eaeba8062c1e78244084ce4c7211b202319da05bfd97d2bcb3b.png)

Table 16-27: SPI\_MMTOP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TOPADR     | SPI Memory Top Address. The SPI_MMTOP.TOPADR bit field specifies the top populated address of a connect- ed SPI memory device. Attempts to access SPI memory are not blocked if this address is exceeded and an error is generated as part of the read response. |

## Receive FIFO Data Register

The SPI\_RFIFO register has an interface to the receive shift register in the SPI and has an interface to the processor's data buses. The top level of the buffer is visible to programs as the 32-bit SPI\_RFIFO register, but the size (number of word locations) of the receive FIFO is actually flexible with transfer word size. The size of the receive FIFO is 8 if the word size is 8-bit, or the size is 4 if the word size is 16-bit, or the size is 2 if the word size is 32-bit.

Both masters and slaves may stop or stall receive transfers based on FIFO status. When the receive FIFO is full, the SPI master stops initiating new transfers on the SPI if SPI\_RXCTL.RTI is enabled. A slave may stall the SPI interface when the content of the FIFO crosses the selected watermark. If data reception continues after SPI\_RFIFO is full, the data in the receive FIFO is invalid. The SPI indicates this condition with receive overrun ( SPI\_STAT.ROR ) error. This condition is possible when SPI\_RXCTL.RTI =0 and SPI\_RXCTL.REN =1 for a master, or for a slave that does not exercise flow control.

Note that the receive FIFO is reset (cleared) when the SPI is disabled after being enabled.

Figure 16-30: SPI\_RFIFO Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000027_87f88f6ff815e8b8645b815d6f10d80c25085698f8e9f234f8c1eb7e33a2d0f8.png)

Table 16-28: SPI\_RFIFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 31:0               | DATA       | Receive FIFO Data.                                           |
| (R/NW)             |            | The SPI_RFIFO.DATA bit field contains the FIFO receive data. |

## Received Word Count Register

The SPI\_RWC register holds a count of the number of words remaining to be received by the SPI. To start the decrement of the word count in SPI\_RWC , enable the receive word counter ( SPI\_RXCTL.RWCEN =1). The SPI uses the word count to control the duration of transfers and to signal the completion of a burst of transfers with the receive finish interrupt ( SPI\_ILAT.RF ). In DMA mode, the SPI uses the SPI\_RWC register to ensure that the number of frames received during a DMA transfer is equal to the number of words programmed in the DMA channel controller. The values programmed into the SPI\_RWC registers should match the word count in the DMA configuration. The SPI\_RWC register maintains the number of frames to be received in a transfer. The SPI\_RWC should only be changed when the counter is disabled.

Figure 16-31: SPI\_RWC Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000028_29d8d151ab5f7841923564c9d591d16c548da3afc8ad9fa0b5bb6a21f27eb04d.png)

Table 16-29: SPI\_RWC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 15:0               | VALUE      | Received Word Count.                                         |
| (R/W)              |            | The SPI_RWC.VALUE bits hold the receive transfer word count. |

## Received Word Count Reload Register

The SPI\_RWCR register holds the receive word count value that the SPI loads into the SPI\_RWC register when the transfer count decrements to zero. To prevent the SPI from reloading the counter, use zero for the reload count value. The SPI\_RWCR register should only be changed when the counter is disabled.

Figure 16-32: SPI\_RWCR Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000029_9dd2969d549bdb62084c180058b89c931654996656d721eb24aa329c89c47cac.png)

Table 16-30: SPI\_RWCR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 15:0               | VALUE      | Received Word Count Reload.                                                |
| (R/W)              |            | The SPI_RWCR.VALUE bits hold the receive transfer word count reload value. |

## Receive Control Register

The SPI\_RXCTL register enables the SPI receive channel, initiates receive transfers, and configures SPI\_RFIFO buffer watermark settings.

Figure 16-33: SPI\_RXCTL Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000030_dc4c71c6b43f1be66b060b04afd708f5ed1095ac7d4fe2f5e0e9133f8267ef78.png)

Table 16-31: SPI\_RXCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18:16 (R/W)        | RUWM       | Receive FIFO Urgent Watermark. The SPI_RXCTL.RUWM bits select the receive FIFO ( SPI_RFIFO ) watermark level for urgent data bus requests. The SPI also uses this watermark level for generation of the SPI_ILAT.RUWM interrupt. When an urgent SPI_RFIFO watermark is ena- bled with SPI_RXCTL.RUWM , the SPI_RXCTL.RRWM selection is used as the deassertion condition for any SPI_ILAT.RUWM interrupts that are latched. | Receive FIFO Urgent Watermark. The SPI_RXCTL.RUWM bits select the receive FIFO ( SPI_RFIFO ) watermark level for urgent data bus requests. The SPI also uses this watermark level for generation of the SPI_ILAT.RUWM interrupt. When an urgent SPI_RFIFO watermark is ena- bled with SPI_RXCTL.RUWM , the SPI_RXCTL.RRWM selection is used as the deassertion condition for any SPI_ILAT.RUWM interrupts that are latched. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                           | Disabled                                                                                                                                                                                                                                                                                                                                                                                                                    |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                           | 25% full RFIFO                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                           | 50% full RFIFO                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                           | 75% full RFIFO                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                           | Full RFIFO                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                           | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                    |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                           | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                    |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                           | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                    |

Table 16-31: SPI\_RXCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:12 (R/W)        | RRWM       | Receive FIFO Regular Watermark. The SPI_RXCTL.RRWM bits select the receive FIFO ( SPI_RFIFO ) watermark level for regular data bus requests. When an urgent SPI_RFIFO watermark is enabled with SPI_RXCTL.RUWM , the SPI_RXCTL.RRWM selection is used as the deasser- tion condition for any SPI_ILAT.RUWM interrupts that are latched. |
| 13:12 (R/W)        | RRWM       | 0 Empty RFIFO                                                                                                                                                                                                                                                                                                                           |
| 13:12 (R/W)        | RRWM       | 1 RFIFO less than 25% full                                                                                                                                                                                                                                                                                                              |
| 13:12 (R/W)        | RRWM       | 2 RFIFO less than 50% full                                                                                                                                                                                                                                                                                                              |
| 13:12 (R/W)        | RRWM       | 3 RFIFO less than 75% full                                                                                                                                                                                                                                                                                                              |
| 8 (R/W)            | RDO        | Receive Data Overrun. The SPI_RXCTL.RDO bit selects handling for receive data requests when the receive buffer ( SPI_RFIFO ) is full. If enabled and SPI_RFIFO is full, the SPI overwrites                                                                                                                                              |
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
| 3 (R/W)            | RWCEN      | Receive Word Counter Enable. The SPI_RXCTL.RWCEN bit enables the decrement of the SPI_RWC register when the count is not zero and SPI_RXCTL.RTI is enabled. Enabling SPI_RXCTL.RWCEN prevents receive overrun errors from occurring. The SPI_RXCTL.RWCEN bit is valid only when the SPI is a master.                                    |
| 3 (R/W)            | RWCEN      | 0 Disable                                                                                                                                                                                                                                                                                                                               |
| 3 (R/W)            | RWCEN      | 1 Enable                                                                                                                                                                                                                                                                                                                                |

Table 16-31: SPI\_RXCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | RTI        | Receive Transfer Initiate. The SPI_RXCTL.RTI bit enables initiation of receive transfers if the receive FIFO ( SPI_RFIFO ) is not full. The bit also enables this initiation if SPI_RWC is not zero when SPI_RXCTL.RWCEN is enabled. Enabling SPI_RXCTL.RTI prevents re- ceive overrun errors from occurring. The SPI_RXCTL.RTI bit is valid only when the SPI is a master. 0 Disable |
| 0 (R/W)            | REN        | Receive Enable. The SPI_RXCTL.REN bit enables SPI receive channel operation. 0 Disable 1 Enable                                                                                                                                                                                                                                                                                       |

## Slave Select Register

The SPI\_SLVSEL register enables the SPI\_SEL[n] pins for output and indicates the state (high or low) of these pins when enabled.

Figure 16-34: SPI\_SLVSEL Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000031_db348a30384f3def7116cec295a2fe34b3f5d9cfdd3b23309b65b554554b7801.png)

Table 16-32: SPI\_SLVSEL Register Fields

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000032_5dc6e1b683931dc757cbf001cf1e14be83a5b7bae4af876ccc17fcb8d68b7fc7.png)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | SSEL7      | Slave Select 7 Output. The SPI_SLVSEL.SSEL7 bit state indicates the value driven on the related SPI_SEL[n] pin. 0 Low        |
| 14 (R/W)           | SSEL6      | Slave Select 6 Output. The SPI_SLVSEL.SSEL6 bit state indicates the value driven on the related SPI_SEL[n] pin. 0 Low 1 High |

Table 16-32: SPI\_SLVSEL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | SSEL5      | Slave Select 5 Output. The SPI_SLVSEL.SSEL5 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                                                                                                                                                                                                                                                                                    |
| 13 (R/W)           | SSEL5      | 0 Low                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 13 (R/W)           | SSEL5      | 1 High                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 12 (R/W)           | SSEL4      | Slave Select 4 Output. The SPI_SLVSEL.SSEL4 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                                                                                                                                                                                                                                                                                    |
| 12 (R/W)           | SSEL4      | 0 Low                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 12 (R/W)           | SSEL4      | 1 High                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 11 (R/W)           | SSEL3      | Slave Select 3 Output. The SPI_SLVSEL.SSEL3 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                                                                                                                                                                                                                                                                                    |
| 11 (R/W)           | SSEL3      | 0 Low                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 11 (R/W)           | SSEL3      | 1 High                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 10 (R/W)           | SSEL2      | Slave Select 2 Output. The SPI_SLVSEL.SSEL2 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                                                                                                                                                                                                                                                                                    |
| 10 (R/W)           | SSEL2      | 0 Low                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 10 (R/W)           | SSEL2      | 1 High                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 9 (R/W)            | SSEL1      | Slave Select 1 Output. The SPI_SLVSEL.SSEL1 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                                                                                                                                                                                                                                                                                    |
| 9 (R/W)            | SSEL1      | 0 Low                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 9 (R/W)            | SSEL1      | 1 High                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 7 (R/W)            | SSE7       | Slave Select 7 Enable. The SPI_SLVSEL.SSE7 bit enables the related SPI_SEL[n] pin for output. If disabled, the SPI three-states the related SPI_SEL[n] pin. When the SPI is a slave, the master (not the SPI) asserts the input during the transfer. The input may be deas- serted or remain asserted between transfers. While the input is deasserted, the SPI ig- nores the SPI clock, ignores inputs, and three-states outputs. |
| 7 (R/W)            | SSE7       | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 7 (R/W)            | SSE7       | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 16-32: SPI\_SLVSEL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | SSE6       | Slave Select 6 Enable. The SPI_SLVSEL.SSE6 bit enables the related SPI_SEL[n] pin for output. See the SPI_SLVSEL.SSE7 bit description for more information.         |
| 6 (R/W)            | SSE6       | 0 Disable                                                                                                                                                           |
| 6 (R/W)            | SSE6       | 1 Enable                                                                                                                                                            |
| 5 (R/W)            | SSE5       | Slave Select 5 Enable. The SPI_SLVSEL.SSE5 bit enables the related SPI_SEL[n] pin for output. See the SPI_SLVSEL.SSE7 bit description for more information.         |
| 5 (R/W)            | SSE5       | 0 Disable                                                                                                                                                           |
| 5 (R/W)            | SSE5       | 1 Enable                                                                                                                                                            |
| 4 (R/W)            | SSE4       | Slave Select 4 Enable. The SPI_SLVSEL.SSE4 bit enables the related SPI_SEL[n] pin for output. See the SPI_SLVSEL.SSE7 bit description for more information.         |
| 4 (R/W)            | SSE4       | 0 Disable                                                                                                                                                           |
| 3 (R/W)            | SSE3       | Slave Select 3 Enable. The SPI_SLVSEL.SSE3 bit enables the related SPI_SEL[n] pin for output. See the SPI_SLVSEL.SSE7 bit description for more information.         |
| 3 (R/W)            | SSE3       | 0 Disable                                                                                                                                                           |
| 3 (R/W)            | SSE3       | 1 Enable                                                                                                                                                            |
| 2 (R/W)            | SSE2       | Slave Select 2 Enable. The SPI_SLVSEL.SSE2 bit enables the related SPI_SEL[n] pin for output. See the SPI_SLVSEL.SSE7 bit description for more information. Disable |
| 2 (R/W)            | SSE2       | 0                                                                                                                                                                   |
| 2 (R/W)            | SSE2       | 1 Enable                                                                                                                                                            |
| 1 (R/W)            | SSE1       | Slave Select 1 Enable. The SPI_SLVSEL.SSE1 bit enables the related SPI_SEL[n] pin for output. See the SPI_SLVSEL.SSE7 bit description for more information. Disable |
| 1 (R/W)            | SSE1       | 0                                                                                                                                                                   |
| 1 (R/W)            | SSE1       | 1 Enable                                                                                                                                                            |

## Status Register

The SPI\_STAT register indicates SPI status including FIFO status, error conditions, and interrupt conditions. When an interrupt condition from this register is unmasked (enabled) by the corresponding bit in the SPI\_IMSK register, the interrupt request is latched into the corresponding bit in the SPI\_ILAT register.

Figure 16-35: SPI\_STAT Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000033_a7f6bde81f9b0c605a6c6ced1aea27bb41c5dd25d0dfea067c0e27fa26aa2fb2.png)

Table 16-33: SPI\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | MMAE       | Memory Mapped Access Error. The SPI_STAT.MMAE bit =1 if an attempt is made to access either the Tx or Rx FIFO while memory-mapped access of SPI memory is enabled (see the SPI_CTL.MMSE bit). The SPI_STAT.MMAE bit =0 when a 1 is written to it. The SPI_STAT.MMAE bit is provided for software notification only. Its state has no fur- ther effect. |

Table 16-33: SPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W1C)         | MMRE       | Memory Mapped Read Error. The SPI_STAT.MMRE bit =1 if an attempt is made to read address space reserved for memory-mapped SPI memory while memory mapping is disabled (see the SPI_CTL.MMSE bit). The SPI_STAT.MMRE bit =0 when a 1 is written to it. This bit is provided for software notification only. Its state has no further effect. |
| 28 (R/W1C)         | MMWE       | Memory Mapped Write Error. The SPI_STAT.MMWE bit =1 if an attempt is made to write address space reserved for memory-mapped SPI memory. The SPI_STAT.MMWE bit =0 when a 1 is written to it. This bit is provided for software notification only. Its state has no further effect.                                                           |
| 23 (R/NW)          | TFF        | SPI_TFIFO Full. The SPI_STAT.TFF bit indicates whether the SPI_TFIFO is full or not full. 0 Not full Tx FIFO                                                                                                                                                                                                                                |
| 22 (R/NW)          | RFE        | SPI_RFIFO Empty. The SPI_STAT.RFE bit indicates whether the SPI_RFIFO is empty or not empty. 0 Rx FIFO not empty                                                                                                                                                                                                                            |
| 20 (R/NW)          | FCS        | 1 Rx FIFO empty Flow Control Stall Indication. The SPI_STAT.FCS bit indicates whether a slave has deasserted the SPI_RDY pin to stall the SPI master while the slave is unable to service the SPI masters request. This bit is valid only when the SPI is a master ( SPI_CTL.MSTR =1) and flow control is enabled ( SPI_CTL.FCEN =1).       |
| 18:16 (R/NW)       | TFS        | SPI_TFIFO Status. The SPI_STAT.TFS bits indicate the status of the SPI_TFIFO . The SPI uses this status when evaluating transmit watermark conditions.                                                                                                                                                                                      |

Table 16-33: SPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:12 (R/NW)       | RFS        | SPI_RFIFO Status. The SPI_STAT.RFS bits indicate the status of the SPI_RFIFO . The SPI uses this status when evaluating receive watermark conditions.                                                                                                                            |
| 11 (R/W1C)         | TF         | 4 Full RFIFO Transmit Finish Indication. The SPI_STAT.TF bit indicates that the SPI has detected the finish of a transmit burst transfer (the SPI_TWC count decrements to zero). This condition can only oc- cur when SPI_TXCTL.TTI and SPI_TXCTL.TWCEN are enabled. 0 No status |
| 10 (R/W1C)         | RF         | 1 Transmit finish detected Receive Finish Indication. The SPI_STAT.RF bit indicates that the SPI has detected the finish of a receive burst transfer (the SPI_RWC count decrements to zero). This condition can only oc- cur when SPI_RXCTL.RTI and SPI_RXCTL.RWCEN are enabled. |
| 9 (R/W1C)          | TS         | 0 No status 1 Receive finish detected Start. SPI_STAT.TS bit indicates that the SPI has detected the start of a transmit transfer. A transmit bursts starts with the load of from the                                                                                            |
|                    |            | Transmit The burst SPI_TWC SPI_TWCR . This condition can only occur when SPI_TXCTL.TTI and SPI_TXCTL.TWCEN are enabled. 0 No status 1 Transmit start detected                                                                                                                    |
| 8 (R/W1C)          |            | Receive Start. The SPI_STAT.RS bit indicates that the SPI has detected the start of a receive burst transfer. A receive bursts starts with the load of SPI_RWC from the SPI_RWCR . This condition can only occur when SPI_RXCTL.RTI and SPI_RXCTL.RWCEN are enabled.             |
|                    | RS         | 0 No status                                                                                                                                                                                                                                                                      |
|                    | 1 Receive  | start detected                                                                                                                                                                                                                                                                   |

Table 16-33: SPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | MF         | Mode Fault Indication. The SPI_STAT.MF bit, when SPI is a master and SPI_CTL.PSSE is enabled, in- dicates that multiple masters have asserted slave select inputs.                                                                                                                               |
| 7 (R/W1C)          | MF         | 0 No status                                                                                                                                                                                                                                                                                      |
| 6 (R/W1C)          | TC         | Transmit Collision Indication. The SPI_STAT.TC bit, when SPI is a slave, indicates that the load of data into the shift register has occurred too close to the first transmitting edge of the SPI clock. status                                                                                  |
| 6 (R/W1C)          | TC         | 0 No                                                                                                                                                                                                                                                                                             |
| 5 (R/W1C)          | TUR        | Transmit Underrun Indication. The SPI_STAT.TUR bit, when the transmit FIFO ( SPI_TFIFO ) is empty, indi- cates that the last word in the transmit FIFO has been re-sent as transmit data. Alter- nately, it indicates that zero has been sent as transmit data.                                  |
| 5 (R/W1C)          | TUR        | 0 No status                                                                                                                                                                                                                                                                                      |
| 4 (R/W1C)          | ROR        | Receive Overrun Indication. The SPI_STAT.ROR bit, when the receive FIFO ( SPI_RFIFO ) is full, indicates that a word in the receive FIFO has been overwritten with incoming receive data. Al- ternately, it indicates that incoming receive data has been discarded.                             |
| 4 (R/W1C)          | ROR        | 0 No status                                                                                                                                                                                                                                                                                      |
| 2 (R/NW)           | TUWM       | 1 Receive overrun occurred Transmit Urgent Watermark Breached. The SPI_STAT.TUWM bit indicates that the transmit urgent watermark ( SPI_TXCTL.TUWM ) has been reached. This condition is cleared when the transmit FIFO fills enough to reach the transmit regular watermark ( SPI_TXCTL.TRWM ). |
| 2 (R/NW)           | TUWM       | 0 Tx regular watermark reached                                                                                                                                                                                                                                                                   |
| 1 (R/NW)           | RUWM       | Receive Urgent Watermark Breached. The SPI_STAT.RUWM bit indicates that the receive urgent watermark ( SPI_RXCTL.RUWM ) has been reached. This condition is cleared when the receive FIFO empties enough to reach the receive regular watermark ( SPI_RXCTL.RRWM                                 |
| 1 (R/NW)           | RUWM       | 0 Rx regular watermark reached                                                                                                                                                                                                                                                                   |
| 1 (R/NW)           | RUWM       | ).                                                                                                                                                                                                                                                                                               |
|                    |            | 1 Rx urgent watermark                                                                                                                                                                                                                                                                            |
|                    |            | breached                                                                                                                                                                                                                                                                                         |

Table 16-33: SPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------|
| 0 (R/NW)           | SPIF       | SPI Finished. The SPI_STAT.SPIF bit indicates that a single word transfer is complete. |
| 0 (R/NW)           | SPIF       | 0 No status                                                                            |
| 0 (R/NW)           | SPIF       | 1 Completed single word transfer                                                       |

## Transmit FIFO Data Register

The SPI\_TFIFO register has an interface to the transmit shift register in the SPI and has an interface to the processor's data buses. The top level of the buffer is visible to programs as the 32-bit SPI\_TFIFO register, but the size (number of word locations) of the transmit FIFO is actually flexible with transfer word size. The size of the transmit FIFO is 8 if the word size is 8-bit, or the size is 4 if the word size is 16-bit, or the size is 2 if the word size is 32-bit.

Both masters and slaves may stop or stall transmit transfers based on FIFO status. When the transmit FIFO is empty, the SPI master stops initiating new transfers on the SPI if SPI\_TXCTL.TTI is enabled. A slave may stall the SPI interface when the content of the FIFO crosses the selected watermark. If data transmit requests continue after SPI\_TFIFO is empty, the data sent from the transmit FIFO is invalid, and the SPI indicates this condition with transmit underrun ( SPI\_STAT.TUR ). This condition is possible when SPI\_TXCTL.TTI =0 and SPI\_TXCTL.TEN =1 for a master, or for a slave that does not exercise flow control.

Note that the transmit FIFO is reset (cleared) when the SPI is disabled after being enabled.

Figure 16-36: SPI\_TFIFO Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000034_f9a54d15804f8d9f48e8357dc29d915dfa14c5056e693ba291bc126ffbfe0f40.png)

Table 16-34: SPI\_TFIFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                       |
|--------------------|------------|---------------------------------------------------------------|
| 31:0               | DATA       | Transmit FIFO Data.                                           |
| (R/W)              |            | The SPI_TFIFO.DATA bit field contains the FIFO transmit data. |

## Transmitted Word Count Register

The SPI\_TWC register holds a count of the number of words remaining to be transmitted by the SPI. To start the decrement of the word count in SPI\_TWC , enable the transmit word counter ( SPI\_TXCTL.TWCEN =1). The SPI uses the word count to control the duration of transfers and to signal the completion of a burst of transfers with the transmit finish interrupt request. In DMA mode, the SPI uses the SPI\_TWC to ensure that the number of frames transmitted during a DMA transfer is equal to the number of words programmed in the DMA channel controller. The values programmed into the SPI\_TWC registers should match the word count in the DMA configuration. The SPI\_TWC maintains the number of frames to be transmitted in a transfer. The SPI\_TWC should only be changed when the counter is disabled.

Figure 16-37: SPI\_TWC Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000035_203152ecf6b37faa72ed35eac6a196f3276d33129517d32cdd67671425edcd3d.png)

Table 16-35: SPI\_TWC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                       |
|--------------------|------------|---------------------------------------------------------------|
| 15:0               | VALUE      | Transmitted Word Count.                                       |
| (R/W)              |            | The SPI_TWC.VALUE bits hold the transmit transfer word count. |

## Transmitted Word Count Reload Register

The SPI\_TWCR register holds the transmit word count value that the SPI loads into the SPI\_TWC register when the transfer count decrements to zero. To prevent the SPI from reloading the counter, use zero for the reload count value. The SPI\_TWCR should only be changed when the counter is disabled.

Figure 16-38: SPI\_TWCR Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000036_1ab53e2c63dcfd6c5b9f8266a79d710718e1b115c86a8c0d02b7ef288c84ede5.png)

Table 16-36: SPI\_TWCR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                     |
|--------------------|------------|-----------------------------------------------------------------------------|
| 15:0               | VALUE      | Transmitted Word Count Reload.                                              |
| (R/W)              |            | The SPI_TWCR.VALUE bits hold the transmit transfer word count reload value. |

## Transmit Control Register

The SPI\_TXCTL register enables the SPI transmit channel, initiates transmit transfers, and configures SPI\_TFIFO buffer watermark settings.

Figure 16-39: SPI\_TXCTL Register Diagram

![Image](19_Serial_Peripheral_Interface_(SPI)_artifacts/image_000037_20f61a6691ec696eef502beb59fb2fa556c7c15cf6ea5b78675b043018674e05.png)

Table 16-37: SPI\_TXCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18:16 (R/W)        | TUWM       | FIFO Urgent Watermark. The SPI_TXCTL.TUWM bits select the transmit FIFO ( SPI_TFIFO ) watermark level for urgent data bus requests. The SPI also uses this watermark level for generation of the SPI_ILAT.TUWM interrupt request. When an urgent SPI_TFIFO water- mark is enabled with SPI_TXCTL.TUWM , the SPI_TXCTL.TRWM selection is used as the deassertion condition for any SPI_ILAT.TUWM interrupt requests that are latched. | FIFO Urgent Watermark. The SPI_TXCTL.TUWM bits select the transmit FIFO ( SPI_TFIFO ) watermark level for urgent data bus requests. The SPI also uses this watermark level for generation of the SPI_ILAT.TUWM interrupt request. When an urgent SPI_TFIFO water- mark is enabled with SPI_TXCTL.TUWM , the SPI_TXCTL.TRWM selection is used as the deassertion condition for any SPI_ILAT.TUWM interrupt requests that are latched. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                    | Disabled                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                    | 25% empty TFIFO                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                    | 50% empty TFIFO                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                    | 75% empty TFIFO                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                    | Empty TFIFO                                                                                                                                                                                                                                                                                                                                                                                                                          |

Table 16-37: SPI\_TXCTL Register Fields (Continued)

| Bit No. (Access) Bit   | Description/Enumeration                                                                                                                                                                                                                                                                                                                             |
|------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:12 (R/W) TRWM       | FIFO Regular Watermark. The SPI_TXCTL.TRWM bits select the transmit FIFO ( SPI_TFIFO ) watermark level for regular data bus requests. When an urgent SPI_TFIFO watermark is ena- bled with SPI_TXCTL.TUWM , the SPI_TXCTL.TRWM selection is used as the deassertion condition for any SPI_ILAT.TUWM interrupt requests that are latched.            |
| 8 (R/W) TDU            | Transmit Data Underrun. The SPI_TXCTL.TDU bit selects handling for transmit data requests when the trans- mit buffer ( SPI_TFIFO ) is empty. If enabled and SPI_TFIFO is empty, the SPI transmits zero as data. If disabled and SPI_TFIFO is empty, the SPI transmits the last word in the buffer as data. 0 Send last word when SPI_TFIFO is empty |
| 6:4 (R/W) TDR          | 1 Send zeros when SPI_TFIFO is empty Transmit Data Request. bits select transmit FIFO ( SPI_TFIFO ) watermark condi- direct the SPI to generate a transmit status interrupt request. 0 Disabled 1 Not full TFIFO 2 25% empty TFIFO                                                                                                                  |
|                        | The SPI_TXCTL.TDR tions that 3 50% empty TFIFO 4 75% empty TFIFO                                                                                                                                                                                                                                                                                    |
| 3 (R/W)                | 5 Empty TFIFO Transmit Word Counter Enable. The SPI_TXCTL.TWCEN bit enables the decrement of the transmit word count ( SPI_TWC ) register when the count is not zero and SPI_TXCTL.TTI is Enabling SPI_TXCTL.TWCEN prevents transmit underrun errors from The bit is valid only when the SPI is a master.                                           |
| TWCEN                  | enabled. occurring. SPI_TXCTL.TWCEN                                                                                                                                                                                                                                                                                                                 |
|                        | 0 Disable                                                                                                                                                                                                                                                                                                                                           |
|                        | Enable                                                                                                                                                                                                                                                                                                                                              |
|                        | 1                                                                                                                                                                                                                                                                                                                                                   |

Table 16-37: SPI\_TXCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | TTI        | Transmit Transfer Initiate. The SPI_TXCTL.TTI bit enables initiation of transmit transfers if the transmit FIFO ( SPI_TFIFO ) is not empty. The bit also enables this initiation if SPI_TWC is not zero when SPI_TXCTL.TWCEN is enabled. Enabling SPI_TXCTL.TTI pre- vents transmit underrun errors from occurring. The SPI_TXCTL.TTI bit is valid only when the SPI is a master. 0 Disable |
| 0 (R/W)            | TEN        | Transmit Enable. The SPI_TXCTL.TEN bit enables SPI transmit channel operation. 0 Disable 1 Enable                                                                                                                                                                                                                                                                                           |