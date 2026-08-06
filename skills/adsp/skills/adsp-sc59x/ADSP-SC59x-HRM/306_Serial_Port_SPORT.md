# Serial Port (SPORT)

<!-- source: 306_Serial_Port_SPORT.pdf | original pages 2762–2860 -->

## 32   Serial Port (SPORT)

The programmable serial ports (SPORTs) support various protocols for serial data communication and provide a glueless hardware interface to many industry-standard data converters and codecs. They have high data rates and dual half-duplex datapaths and are ideal for establishing a direct serial connection among two or more processors in a multiprocessor system, as many processors provide compatible serial interfaces.

The SPORT top module consists of two half SPORTs with identical functionality and programming requirements. Each half SPORT can be independently configured as either a transmitter or receiver and can be coupled with the other half SPORT within the same SPORT top module. Further, each half SPORT provides two synchronous halfduplex data lines to double the total supported throughput. As such, a single SPORT top module can be used to provide up to four unidirctional or up to two full-duplex data streams. Further channels are possible as well, but utilization of multiple SPORT top modules is required, thus requiring external connections to provide a common time base.

## Features

An individual SPORT top module consists of two independently configurable SPORT halves with identical functionality. These SPORT halves offer the following features:

- Up to two bidirectional data lines - each half SPORT supports up to two transmit or receive channels, thus allowing two unidirectional streams into or out of each half SPORT and providing greater flexibility for serial communications. If full-duplex functionality is desired, two SPORT halves can be combined to enable dualstream bidirectional communication.
- Six operating modes:
1. Standard DSP serial mode
2. I 2 S mode
3. Left-Justified mode
4. Right-Justified mode
5. Multichannel (TDM) mode
6. Packed mode

- Supports internally or externally generated clock.
- Support for both even and odd SCLK0 to SPORT clock (SPORT\_CLK) ratios. If both data lines of a half SPORT are active, the maximum throughput is 2 x SPORT\_CLK bps.
- Configurable rising or falling edge of the SPORT\_CLK for driving and sampling data and frame syncs.
- Gated clock mode support for internally or externally generated clocks in DSP serial mode and stereo modes (left-justified and I 2 S mode).
- Supports frameless operation.
- Supports internally or externally generated frame sync signals.
- Programmable frame sync polarity.
- Programmable frame sync timing (synchronous to data or 1 SPORT clock in advance of it).
- Detection of prematurely received external frame syncs (with optional interrupt request generation).
- Programmable level-/edge-sensitivity for external frame syncs.
- Programmable (4-32-bit) data length, either in most significant bit (MSB) first or in least significant bit (LSB) first format, with optional sign-extension on received data.
- Optional 16-bit to 32-bit word packing (as receiver) and 32-bit to 16-bit word unpacking (as transmitter).
- Support for A-law and µ-law compression/decompression hardware companding, according to the G.711 specification, on transmitted/received words in all operating modes.
- Transmit underrun and receive overflow detection (with optional interrupt request generation).
- TDM mode transfers data on 128 contiguous channels from a stream of up to 1024 total channels (useful for H.100/H.110 and other telephony interfaces as a network communication scheme for multiple processors).
- Performs interrupt-driven, single word core transfers to and from on-chip or off-chip memory.
- Dedicated DMA channel for each SPORT half supporting autobuffer (for a repeated, identical range of transfers) and numerous descriptor-based (individual or repeated ranges of transfers with differing DMA parameters) modes.
- Generator and receiver trigger functionality.
- Unique transfer finish interrupt (TFI) signaling when the last transmit word is fully out of the transmit shift register.
- Multiplexer to internally connect critical timing signals between SPORT halves.
- Support for Global Sport Enable functionality, where enabling multiple SPORTs need to be synchronized so that all of them start and/or end at the same time.
- Support for late reception of SPORT data and frame sync.

## Signal Descriptions

Each half SPORT module has five dedicated signals, as described in the SPORT Signal Descriptions table. The actual pin name varies with different SPORT halves. Individual SPORT halves do not share any of its signals across the pair that comprises the SPORT top module; however, it is possible to connect the clock and frame sync signals between the SPORT half pair, as explained in the Multiplexer Logic section.

All of the SPORT signals are multiplexed on the PORT pins, possibly sharing functionality with other peripherals on the device. By default, the PORT pins are in GPIO mode and must be reconfigured for SPORT functionality by setting the appropriate bits in the PORT\_FER and PORT\_MUX registers. Consult the processor data sheet for details regarding which ports the SPORT signals are available and to ensure to configure the PORT\_MUX register before the PORT\_FER register.

Table 32-1: SPORT Signal Descriptions

| Internal Node   | Direction   | Description                                                                                                                                                                                                         |
|-----------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| SPORTx_CLK      | I/O         | Transmit or receive serial clock. Data and frame syncs are driven or sampled on this clock's edges. This signal can be either internally or externally generated.                                                   |
| SPORTx_FS       | I/O         | Transmit or receive frame sync. The frame sync pulse initiates shifting of serial data. This sig- nal is either internally or externally generated.                                                                 |
| SPORTx_D0       | I/O         | Primary transmit or receive data channel. This signal can be configured as an output to trans- mit serial data or as an input to receive serial data.                                                               |
| SPORTx_D1       | I/O         | Secondary transmit or receive data channel. This signal can be configured as an output to transmit serial data or as an input to receive serial data.                                                               |
| SPORTx_TDV      | O           | Multichannel transmit data valid. This signal is only active in multichannel transmit mode and is asserted during enabled slots, as defined by the channel selection registers ( SPORT_CS0_A through SPORT_CS3_B ). |

The data channel signals are transmit signals when the serial port is configured in transmit mode ( SPORT\_CTL\_A.SPTRAN = 1). They are receive signals when the serial port is configured in receive mode ( SPORT\_CTL\_A.SPTRAN = 0). The following sections further describe the SPORT signals.

NOTE: These sections explicitly refer to the registers associated with half SPORT A, but the same concepts also apply to half SPORT B.

## Serial Clock

The serial port clock ( SPORT\_ACLK ) is either a receive serial clock or a transmit serial clock, depending on the transfer direction ( SPORT\_CTL\_A.SPTRAN ), governing when the data bits are serially shifted into or out of the SPORT and when the frame sync signal is driven (in internal frame sync mode) or sampled (in external frame sync mode). It can be internally generated from the processor's system clock (SCLK) or externally provided. If the half SPORT is configured in internal clock mode ( SPORT\_CTL\_A.ICLK = 1), then the SPORT\_DIV\_A.CLKDIV field specifies the divisor applied to SCLK to generate the SPORT clock. As it is a 16-bit divisor, a wide range of serial clock rates is possible. Use the following equation to calculate the serial clock frequency:

```
SPORT_ACLK = [SCLK ÷ ( SPORT_DIV_A.CLKDIV + 1)]
```

From this, the following equation can be used to determine the value of SPORT\_DIV\_A.CLKDIV , given the SCLK frequency and the desired frequency of the SPORT clock:

```
SPORT_DIV_A.CLKDIV = [(SCLK ÷ SPORT_ACLK ) - 1]
```

The half SPORT also supports a 1:1 SPORT\_ACLK to ratio (per the equations above, program the clock divisor field to zero). In this case, the resulting SPORT clock frequency is equal to SCLK.

NOTE: Be careful not to exceed the maximum SPORT\_ACLK frequency specified in the processor data sheet.

In certain operating modes, the SPORT can be configured to generate a gated clock, which is active only during valid data. In some applications, a SPORT uses it to generate a general-purpose clock in the system. In this case, enable the SPORT with the appropriate SPORT\_DIV\_A.CLKDIV divisor field in internal clock mode.

If a SPORT is configured in external clock mode ( SPORT\_CTL\_A.ICLK = 0), the serial clock is an input signal, thus making the SPORT operate in target mode. In this mode, the SPORT\_DIV\_A.CLKDIV is irrelevant and is ignored. The optional loopback capability provided by the internal SPORT multiplexer (SPMUX) block allows the target SPORT to use the serial clock from the neighboring SPORT half in the same SPORT top module.

An externally-supplied serial clock does not need to be synchronous with the processor clocks. Further, the external clock can be a gated clock, but it must comply with the requirements described in the Gated Clock Mode section.

Refer to the product data sheet for exact AC timing specifications.

## Frame Sync

The SPORT frame sync ( SPORT\_AFS ) signal is either a receive frame sync or a transmit frame sync, depending on the transfer direction ( SPORT\_CTL\_A.SPTRAN ), which is used to determine the start of a new word or frame. When this signal goes active, the serial port starts serially shifting data into or out of the SPORT. The frame sync signal can be internally generated based on its serial clock ( SPORT\_ACLK ) or externally provided, as configured by the SPORT\_CTL\_A.IFS bit.

If the half SPORT is configured to generate frame syncs ( SPORT\_CTL\_A.IFS = 1), then the SPORT\_DIV\_A.FSDIV field specifies the divisor used to generate the periodic SPORT\_AFS signal from the SPORT clock. As this is a 16-bit divisor, a wide range of frame sync rates to initiate periodic transfers is possible. Whether the serial clock is internally or externally generated, this divisor is a count of SPORT clock cycles between frame sync pulses, the formula for which is:

```
Number of serial clocks between frame syncs = ( SPORT_DIV_A.FSDIV + 1)
```

From this, the following equation can be used to determine the value of SPORT\_DIV\_A.FSDIV , given the serial clock frequency and the desired frame sync frequency:

```
SPORT_DIV_A.FSDIV = [( SPORT_ACLK ÷ SPORT_AFS ) - 1]
```

The frame sync is continuously active when SPORT\_DIV\_A.FSDIV = 0. The value of SPORT\_DIV\_A.FSDIV cannot be less than the serial word length minus one (the value of the SPORT\_CTL\_A.SLEN bit field). Failure to

adhere to this guideline can cause an external device to abort the current operation or cause other unpredictable results.

NOTE: After enabling the SPORT, the first internal frame sync appears after a delay of SPORT\_DIV\_A.FSDIV + 3 SPORT clock cycles.

If a SPORT is configured for external frame syncs ( SPORT\_CTL\_A.IFS = 0), then SPORT\_AFS is an input signal and the SPORT\_DIV\_A.FSDIV field of the SPORT\_DIV\_A register is irrelevant and ignored. By default, this external signal is level-sensitive, but it can be configured as an edge-sensitive signal by setting the SPORT\_CTL\_A.FSED bit. The frame sync is expected to be synchronous with the serial clock. If not, it must meet the timing requirements that appear in the data sheet.

The SPORT can be used as a counter for dividing an external clock to generate periodic pulses or periodic interrupts. To do so, enable the SPORT with the appropriate SPORT\_DIV\_A.FSDIV divisor field with the SPORT configured for an external clock and internal data-independent frame syncs.

In some of the operating modes, the SPORT can be programmed to treat the frame sync signal as an optional signal by clearing the SPORT\_CTL\_A.FSR bit. Even with this bit cleared, the SPORT requires a single frame sync assertion to start the continuous transfers, after which it is ignored (for externally supplied frame syncs) or not generated (for internally-generated frame syncs). Characteristics of the frame sync depend on the settings in the SPORT control registers and the operating mode of the SPORT. For more information, refer to the SPORT\_CTL\_A register.

## Data Signals

Each half SPORT has two bidirectional data lines known as the primary ( SPORT\_AD0 ) and secondary ( SPORT\_AD1 ) data channels. Both data lines can be configured as either transmitters or receivers using the SPORT\_CTL\_A.SPTRAN bit, thus permitting dual unidirectional data streams to increase the data throughput of the SPORT.

- NOTE: Configuring one transmit data channel and one receive data channel on a single half SPORT is not supported.

The primary and secondary data lines can be individually enabled or disabled using the SPORT\_CTL\_A.SPENPRI and the SPORT\_CTL\_A.SPENSEC bits, respectively. However, if using both, enable or disable them concurrently. These data lines operate in a synchronous manner (sharing a clock and frame sync) but have separate datapaths with unique data buffers, shift registers and optional companding logic. All of the SPORT control settings are common for both channels, but the single DMA channel per half SPORT serves both the primary and secondary data channels.

When a SPORT is configured in multichannel transmit mode, the data pins three-state during inactive channel slots, thus allowing multiple transmitters to operate on the same bus with different active channels.

See the Architectural Concepts section for more details about data transfer operation.

## Transmit Data Valid Signal

The transmit data valid ( SPORT\_ATDV ) signal is available only in transmit multichannel modes (including packed mode). It is driven active during enabled multichannel slots, and it is driven inactive during the disabled channels. In other words, the SPORT\_ATDV signal is active when data is being driven to the data pins and inactive when the data pins are being three-stated. As such, the SPORT\_ATDV signal can serve as an output-enable signal for the data transmit pin.

## SRU Programming

The SPORT uses the SRU (signal routing unit) to connect the SPORT data, serial clock, frame sync, and external sync (if external synchronization is required). Inputs also must be routed through the SRU. Program the corresponding SRU registers to connect the outputs to the required destinations. For details of the routing, see DAI Routing Capabilities in the Digital Audio Interface (DAI) chapter.

## SRU SPORT Receive Controller

If the SPORT is receiver, it must feed its controller output clock back to its input clock. This is required to trigger the SPORT's state machine. Using SPORT 0A as an example receiver, programs should route SPT0\_ACLK\_O to SPT0\_ACLK\_I . This is not required if the SPORT is operating as a transmitter in controller mode.

## Functional Description

The following sections provide general information about the functionality of the processor's serial ports:

- Architectural Concepts
- Data Types and Companding
- Transmit Path
- Receive Path

## ADSP-SC59x SPORT Register List

The Serial Port (SPORT) controller, with its range of clock and frame synchronization options, supports a variety of serial communication protocols and provides a glueless hardware interface to many industry-standard data converters and CODECs. Each SPORT has two independent halves (A and B), and each half contains two channels (primary and secondary). A set of registers governs SPORT operations. For more information on SPORT functionality, see the SPORT register descriptions.

Table 32-2: ADSP-SC59x SPORT Register List

| Name        | Description                                      |
|-------------|--------------------------------------------------|
| SPORT_CS0_A | Half SPORT 'A' Multichannel 0-31 Select Register |
| SPORT_CS0_B | Half SPORT 'B' Multichannel 0-31 Select Register |

Table 32-2: ADSP-SC59x SPORT Register List (Continued)

| Name          | Description                                        |
|---------------|----------------------------------------------------|
| SPORT_CS1_A   | Half SPORT 'A' Multichannel 32-63 Select Register  |
| SPORT_CS1_B   | Half SPORT 'B' Multichannel 32-63 Select Register  |
| SPORT_CS2_A   | Half SPORT 'A' Multichannel 64-95 Select Register  |
| SPORT_CS2_B   | Half SPORT 'B' Multichannel 64-95 Select Register  |
| SPORT_CS3_A   | Half SPORT 'A' Multichannel 96-127 Select Register |
| SPORT_CS3_B   | Half SPORT 'B' Multichannel 96-127 Select Register |
| SPORT_CTL2_A  | Half SPORT 'A' Control 2 Register                  |
| SPORT_CTL2_B  | Half SPORT 'B' Control 2 Register                  |
| SPORT_CTL_A   | Half SPORT 'A' Control Register                    |
| SPORT_CTL_B   | Half SPORT 'B' Control Register                    |
| SPORT_DIV_A   | Half SPORT 'A' Divisor Register                    |
| SPORT_DIV_B   | Half SPORT 'B' Divisor Register                    |
| SPORT_ERR_A   | Half SPORT 'A' Error Register                      |
| SPORT_ERR_B   | Half SPORT 'B' Error Register                      |
| SPORT_MCTL_A  | Half SPORT 'A' Multichannel Control Register       |
| SPORT_MCTL_B  | Half SPORT 'B' Multichannel Control Register       |
| SPORT_MSTAT_A | Half SPORT 'A' Multichannel Status Register        |
| SPORT_MSTAT_B | Half SPORT 'B' Multichannel Status Register        |
| SPORT_RXPRI_A | Half SPORT 'A' Rx Buffer (Primary) Register        |
| SPORT_RXPRI_B | Half SPORT 'B' Rx Buffer (Primary) Register        |
| SPORT_RXSEC_A | Half SPORT 'A' Rx Buffer (Secondary) Register      |
| SPORT_RXSEC_B | Half SPORT 'B' Rx Buffer (Secondary) Register      |
| SPORT_TXPRI_A | Half SPORT 'A' Tx Buffer (Primary) Register        |
| SPORT_TXPRI_B | Half SPORT 'B' Tx Buffer (Primary) Register        |
| SPORT_TXSEC_A | Half SPORT 'A' Tx Buffer (Secondary) Register      |
| SPORT_TXSEC_B | Half SPORT 'B' Tx Buffer (Secondary) Register      |

## ADSP-SC59x SPORT Interrupt List

Table 32-3: ADSP-SC59x SPORT Interrupt List

|   Interrupt ID | Name          | Description             | Sensitivity   |   DMA Channel |
|----------------|---------------|-------------------------|---------------|---------------|
|             81 | SPORT0_A_DMA  | SPORT0 ChannelADMA      | Level         |             0 |
|             82 | SPORT0_A_STAT | SPORT0 Channel A Status | Level         |               |
|             83 | SPORT0_B_DMA  | SPORT0 ChannelBDMA      | Level         |             1 |
|             84 | SPORT0_B_STAT | SPORT0 Channel B Status | Level         |               |
|             85 | SPORT1_A_DMA  | SPORT1 ChannelADMA      | Level         |             2 |
|             86 | SPORT1_A_STAT | SPORT1 Channel A Status | Level         |               |
|             87 | SPORT1_B_DMA  | SPORT1 ChannelBDMA      | Level         |             3 |
|             88 | SPORT1_B_STAT | SPORT1 Channel B Status | Level         |               |
|             89 | SPORT2_A_DMA  | SPORT2 ChannelADMA      | Level         |             4 |
|             90 | SPORT2_A_STAT | SPORT2 Channel A Status | Level         |               |
|             91 | SPORT2_B_DMA  | SPORT2 ChannelBDMA      | Level         |             5 |
|             92 | SPORT2_B_STAT | SPORT2 Channel B Status | Level         |               |
|             93 | SPORT3_A_DMA  | SPORT3 ChannelADMA      | Level         |             6 |
|             94 | SPORT3_A_STAT | SPORT3 Channel A Status | Level         |               |
|             95 | SPORT3_B_DMA  | SPORT3 ChannelBDMA      | Level         |             7 |
|             96 | SPORT3_B_STAT | SPORT3 Channel B Status | Level         |               |
|             97 | SPORT4_A_DMA  | SPORT4 ChannelADMA      | Level         |            10 |
|             98 | SPORT4_A_STAT | SPORT4 Channel A Status | Level         |               |
|             99 | SPORT4_B_DMA  | SPORT4 ChannelBDMA      | Level         |            11 |
|            100 | SPORT4_B_STAT | SPORT4 Channel B Status | Level         |               |
|            101 | SPORT5_A_DMA  | SPORT5 ChannelADMA      | Level         |            12 |
|            102 | SPORT5_A_STAT | SPORT5 Channel A Status | Level         |               |
|            103 | SPORT5_B_DMA  | SPORT5 ChannelBDMA      | Level         |            13 |
|            104 | SPORT5_B_STAT | SPORT5 Channel B Status | Level         |               |
|            105 | SPORT6_A_DMA  | SPORT6 ChannelADMA      | Level         |            14 |
|            106 | SPORT6_A_STAT | SPORT6 Channel A Status | Level         |               |
|            107 | SPORT6_B_DMA  | SPORT6 ChannelBDMA      | Level         |            15 |
|            108 | SPORT6_B_STAT | SPORT6 Channel B Status | Level         |               |
|            109 | SPORT7_A_DMA  | SPORT7 ChannelADMA      | Level         |            16 |
|            110 | SPORT7_A_STAT | SPORT7 Channel A Status | Level         |               |
|            111 | SPORT7_B_DMA  | SPORT7 ChannelBDMA      | Level         |            17 |

Table 32-3: ADSP-SC59x SPORT Interrupt List (Continued)

|   Interrupt ID | Name             | Description               | Sensitivity   | DMA Channel   |
|----------------|------------------|---------------------------|---------------|---------------|
|            112 | SPORT7_B_STAT    | SPORT7 Channel B Status   | Level         |               |
|            297 | SPORT0_A_DMA_ERR | SPORT0 Channel A DMAError | Level         |               |
|            298 | SPORT0_B_DMA_ERR | SPORT0 Channel B DMAError | Level         |               |
|            299 | SPORT1_A_DMA_ERR | SPORT1 Channel A DMAError | Level         |               |
|            300 | SPORT1_B_DMA_ERR | SPORT1 Channel B DMAError | Level         |               |
|            301 | SPORT2_A_DMA_ERR | SPORT2 Channel A DMAError | Level         |               |
|            302 | SPORT2_B_DMA_ERR | SPORT2 Channel B DMAError | Level         |               |
|            303 | SPORT3_A_DMA_ERR | SPORT3 Channel A DMAError | Level         |               |
|            304 | SPORT3_B_DMA_ERR | SPORT3 Channel B DMAError | Level         |               |
|            305 | SPORT4_A_DMA_ERR | SPORT4 Channel A DMAError | Level         |               |
|            306 | SPORT4_B_DMA_ERR | SPORT4 Channel B DMAError | Level         |               |
|            307 | SPORT5_A_DMA_ERR | SPORT5 Channel A DMAError | Level         |               |
|            308 | SPORT5_B_DMA_ERR | SPORT5 Channel B DMAError | Level         |               |
|            309 | SPORT6_A_DMA_ERR | SPORT6 Channel A DMAError | Level         |               |
|            310 | SPORT6_B_DMA_ERR | SPORT6 Channel B DMAError | Level         |               |
|            311 | SPORT7_A_DMA_ERR | SPORT7 Channel A DMAError | Level         |               |
|            312 | SPORT7_B_DMA_ERR | SPORT7 Channel B DMAError | Level         |               |

## ADSP-SC59x SPORT Trigger List

Table 32-4: ADSP-SC59x SPORT Trigger List Generators

|   Trigger ID | Name         | Description        | Sensitivity   |
|--------------|--------------|--------------------|---------------|
|           85 | SPORT0_A_DMA | SPORT0 ChannelADMA | Edge          |
|           86 | SPORT0_B_DMA | SPORT0 ChannelBDMA | Edge          |
|           87 | SPORT1_A_DMA | SPORT1 ChannelADMA | Edge          |
|           88 | SPORT1_B_DMA | SPORT1 ChannelBDMA | Edge          |
|           89 | SPORT2_A_DMA | SPORT2 ChannelADMA | Edge          |
|           90 | SPORT2_B_DMA | SPORT2 ChannelBDMA | Edge          |
|           91 | SPORT3_A_DMA | SPORT3 ChannelADMA | Edge          |
|           92 | SPORT3_B_DMA | SPORT3 ChannelBDMA | Edge          |
|           93 | SPORT4_A_DMA | SPORT4 ChannelADMA | Edge          |

Table 32-4: ADSP-SC59x SPORT Trigger List Generators (Continued)

|   Trigger ID | Name         | Description        | Sensitivity   |
|--------------|--------------|--------------------|---------------|
|           94 | SPORT4_B_DMA | SPORT4 ChannelBDMA | Edge          |
|           95 | SPORT5_A_DMA | SPORT5 ChannelADMA | Edge          |
|           96 | SPORT5_B_DMA | SPORT5 ChannelBDMA | Edge          |
|           97 | SPORT6_A_DMA | SPORT6 ChannelADMA | Edge          |
|           98 | SPORT6_B_DMA | SPORT6 ChannelBDMA | Edge          |
|           99 | SPORT7_A_DMA | SPORT7 ChannelADMA | Edge          |
|          100 | SPORT7_B_DMA | SPORT7 ChannelBDMA | Edge          |

Table 32-5: ADSP-SC59x SPORT Trigger List Receivers

|   Trigger ID | Name         | Description        | Sensitivity   |
|--------------|--------------|--------------------|---------------|
|           60 | SPORT0_A_DMA | SPORT0 ChannelADMA | Pulse         |
|           61 | SPORT0_B_DMA | SPORT0 ChannelBDMA | Pulse         |
|           62 | SPORT1_A_DMA | SPORT1 ChannelADMA | Pulse         |
|           63 | SPORT1_B_DMA | SPORT1 ChannelBDMA | Pulse         |
|           64 | SPORT2_A_DMA | SPORT2 ChannelADMA | Pulse         |
|           65 | SPORT2_B_DMA | SPORT2 ChannelBDMA | Pulse         |
|           66 | SPORT3_A_DMA | SPORT3 ChannelADMA | Pulse         |
|           67 | SPORT3_B_DMA | SPORT3 ChannelBDMA | Pulse         |
|           68 | SPORT4_A_DMA | SPORT4 ChannelADMA | Pulse         |
|           69 | SPORT4_B_DMA | SPORT4 ChannelBDMA | Pulse         |
|           70 | SPORT5_A_DMA | SPORT5 ChannelADMA | Pulse         |
|           71 | SPORT5_B_DMA | SPORT5 ChannelBDMA | Pulse         |
|           72 | SPORT6_A_DMA | SPORT6 ChannelADMA | Pulse         |
|           73 | SPORT6_B_DMA | SPORT6 ChannelBDMA | Pulse         |
|           74 | SPORT7_A_DMA | SPORT7 ChannelADMA | Pulse         |
|           75 | SPORT7_B_DMA | SPORT7 ChannelBDMA | Pulse         |

## ADSP-SC59x SPORT DMA Channel List

Table 32-6: ADSP-SC59x SPORT DMA Channel List

| DMAID   | DMAChannel Name   | Description        |
|---------|-------------------|--------------------|
| DMA0    | SPORT0_A_DMA      | SPORT0 ChannelADMA |

Table 32-6: ADSP-SC59x SPORT DMA Channel List (Continued)

| DMAID   | DMAChannel Name   | Description        |
|---------|-------------------|--------------------|
| DMA1    | SPORT0_B_DMA      | SPORT0 ChannelBDMA |
| DMA2    | SPORT1_A_DMA      | SPORT1 ChannelADMA |
| DMA3    | SPORT1_B_DMA      | SPORT1 ChannelBDMA |
| DMA4    | SPORT2_A_DMA      | SPORT2 ChannelADMA |
| DMA5    | SPORT2_B_DMA      | SPORT2 ChannelBDMA |
| DMA6    | SPORT3_A_DMA      | SPORT3 ChannelADMA |
| DMA7    | SPORT3_B_DMA      | SPORT3 ChannelBDMA |
| DMA10   | SPORT4_A_DMA      | SPORT4 ChannelADMA |
| DMA11   | SPORT4_B_DMA      | SPORT4 ChannelBDMA |
| DMA12   | SPORT5_A_DMA      | SPORT5 ChannelADMA |
| DMA13   | SPORT5_B_DMA      | SPORT5 ChannelBDMA |
| DMA14   | SPORT6_A_DMA      | SPORT6 ChannelADMA |
| DMA15   | SPORT6_B_DMA      | SPORT6 ChannelBDMA |
| DMA16   | SPORT7_A_DMA      | SPORT7 ChannelADMA |
| DMA17   | SPORT7_B_DMA      | SPORT7 ChannelBDMA |

## Block Diagram

Each SPORT top module consists of two separate blocks, known as half SPORTs (HSPORT) A and B, each with identical functionality and programming models. The Half Serial Port Block Diagram shows a detailed block diagram of a half SPORT.

Figure 32-1: Half Serial Port Block Diagram

<!-- image -->

## Architectural Concepts

Each half SPORT (HSPORT) block has its own set of control registers and data buffers, grouped per SPORT module. The HSPORT A and B blocks can be independently configured as either a transmitter or a receiver, with the option to be coupled together internally within the single SPORT top module. Each HSPORT also supports two synchronous bidirectional datapaths, referred to as the primary ( D0 ) and secondary ( D1 ) data lines, as shown in the Top-Level SPORT Diagram figure.

Figure 32-2: Top-Level SPORT Diagram

The SPORT\_CTL\_A.SPTRAN bit controls the direction for both datapaths of the HSPORT. Depending on whether the HSPORT is a transmitter or a receiver, the pair of data signals respectfully transmit or receive data bits synchronously. The dual data signals of each HSPORT cannot transmit and receive the data simultaneously in support of full-duplex operation, however, two HSPORTs can be combined to achieve this.

Serial communications are synchronized to the serial clock signal, where a valid clock pulse must accompany each data bit. Each HSPORT can take its clock from an external source or internally generate it from the processor's system clock using the SPORT\_DIV\_A.CLKDIV clock divisor bit field. Both primary and secondary data channels shift data based on the SPORT\_CLK rate and the clock polarity defined by the SPORT\_CTL\_A.CKRE bit.

In addition to the serial clock signal, a frame synchronization signal is used to signify the beginning of an individual data word or a multichannel data stream (block of words). Each SPORT can take the frame sync signal from an external source or generate it ( SPORT\_FS ), depending on the SPORT\_CTL\_A.IFS bit. An internally generated frame sync is derived from the SPORT clock using the SPORT\_DIV\_A.FSDIV divisor field. Both primary and secondary datapaths start shifting data either synchronous to or one serial clock in advance of detecting/generating a valid frame sync signal, as determined by the SPORT\_CTL\_A.LAFS bit. Various communication protocols for serial data can be emulated according to the frame sync format, and all frame sync options are available whether the signal is generated internally or externally.

NOTE: These SPORTs are not UARTs and cannot communicate with an RS-232 device or any other asynchronous communications protocol.

## Multiplexer Logic

The SPORT multiplexing block (SPMUX) is situated between the SPORT hardware block and the processor's pin multiplexing logic. It allows the flexibility to route and share the clock and frame sync signals between the HSPORT A and B halves within each SPORT top module, which can double the data throughput (if both SPORT halves are transmitters or both are receivers) or provide full-duplex capabilities (if one HSPORT is a receiver and the other a

<!-- image -->

transmitter) without the need to allocate pins for the peripheral or make physical connections outside the processor. The SPORT\_CTL2\_A register is used to configure this loopback feature.

NOTE: Throughout this section, HSPORT A is used as a reference, but all concepts also apply to HSPORT B.

The multiplexing depends on the configuration of the SPORT\_CTL\_A.IFS and SPORT\_CTL\_A.ICLK bits, and the SPORT\_CTL2\_A.CKMUXSEL and SPORT\_CTL2\_A.FSMUXSEL bit settings control the multiplexing. The Frame Sync Combinations and Clock Combinations tables show the valid combinations for the bit settings.

NOTE: All other settings are illegal. However, hardware does not check or prevent the illegal settings. Ensure that programs use only legal combinations.

The column headers in the Frame Sync Combinations table are defined as follows:

- FS Combination = Frame sync combination, referenced in the notes that follow the Clock Combinations table.
- HSA\_IFS = the setting of the SPORT\_CTL\_A.IFS configuration bit.
- HSB\_IFS = the setting of the SPORT\_CTL\_B.IFS configuration bit.
- FSAMUX = the setting of the SPORT\_CTL2\_A.FSMUXSEL configuration bit.
- FSBMUX = the setting of the SPORT\_CTL2\_B.FSMUXSEL configuration bit.

The Routing column in the Frame Sync Combinations table defines how the signals are used between the SPORT halves and which pin is used for the frame sync (whether it is an input or an output). Within the column, the inequality characters ( ≤ and ≥ ) are used to show the direction of the signal, and the following abbreviations are used (where x = A or B):

- HSx\_FI = Frame sync input signal, provided by an external device or the complementing HSPORT.
- HSx\_FO = Frame sync output signal.
- SPx\_FS = HSPORT's frame sync pin, where the signal is either:
- Provided by an external source and distributed to both HSPORT frame sync signals.
- Internally generated by one HSPORT and routed to both the pin and to the complementary HSPORT frame sync signal.

Table 32-7: Frame Sync Combinations

|   FS Combination |   HSA_IFS |   HSB_IFS |   FSAMUX |   FSBMUX | Routing             |
|------------------|-----------|-----------|----------|----------|---------------------|
|                1 |         0 |         0 |        0 |        0 | Native FS Operation |
|                2 |         0 |         1 |        0 |        0 | Native FS Operation |
|                3 |         1 |         0 |        0 |        0 | Native FS Operation |
|                4 |         1 |         1 |        0 |        0 | Native FS Operation |

Table 32-7: Frame Sync Combinations (Continued)

|   FS Combination |   HSA_IFS |   HSB_IFS |   FSAMUX |   FSBMUX | Routing                          |
|------------------|-----------|-----------|----------|----------|----------------------------------|
|                5 |         0 |         0 |        1 |        0 | HSA_FI ≤ SPB_FS; HSB_FI ≤ SPB_FS |
|                6 |         0 |         1 |        1 |        0 | HSA_FI ≤ HSB_FO ≥ SPB_FS         |
|                7 |         0 |         0 |        0 |        1 | HSB_FI ≤ SPA_FS; HSA_FI ≤ SPA_FS |
|                8 |         1 |         0 |        0 |        1 | HSB_FI ≤ HSA_FO ≥ SPA_FS         |

The column headers in the Clock Combinations table are defined as follows:

- CLK Combination = Clock combination, referenced in the notes that follow the table.
- HSA\_ICLK = the setting of the SPORT\_CTL\_A.ICLK configuration bit.
- HSB\_ICLK = the setting of the SPORT\_CTL\_B.ICLK configuration bit.
- CKAMUX = the setting of the SPORT\_CTL2\_A.CKMUXSEL configuration bit.
- CKBMUX = the setting of the SPORT\_CTL2\_B.CKMUXSEL configuration bit.

The Routing column in the Clock Combinations table defines how the signals are used between the SPORT halves and which pin is used for the serial clock (whether it is an input or an output). Within the column, the inequality characters ( ≤ and ≥ ) are used to show the direction of the signal, and the following abbreviations are used (x = A or B):

- HSx\_CI = Serial clock input signal, provided by an external device or the complementing HSPORT.
- HSx\_CO = Serial clock output signal.
- SPx\_CLK = HSPORT's serial clock pin, where the signal is either:
- provided by an external source and distributed to both HSPORT serial clock signals, or
- internally generated by one HSPORT and routed to both the pin and to the complementary HSPORT serial clock signal.

Table 32-8: Clock Combinations

|   CLK Combination |   HSA_ICLK |   HSB_ICLK |   CKAMUX |   CKBMUX | Routing              |
|-------------------|------------|------------|----------|----------|----------------------|
|                 9 |          0 |          0 |        0 |        0 | Native CLK Operation |
|                10 |          0 |          1 |        0 |        0 | Native CLK Operation |
|                11 |          1 |          0 |        0 |        0 | Native CLK Operation |
|                12 |          1 |          1 |        0 |        0 | Native CLK Operation |

Table 32-8: Clock Combinations (Continued)

|   CLK Combination |   HSA_ICLK |   HSB_ICLK |   CKAMUX |   CKBMUX | Routing                            |
|-------------------|------------|------------|----------|----------|------------------------------------|
|                13 |          0 |          0 |        1 |        0 | HSA_CI ≤ SPB_CLK; HSB_CI ≤ SPB_CLK |
|                14 |          0 |          1 |        1 |        0 | HSA_CI ≤ HSB_CO ≥ SPB_CLK          |
|                15 |          0 |          0 |        0 |        1 | HSB_CI ≤ SPA_CLK; HSA_CI ≤ SPA_CLK |
|                16 |          1 |          0 |        0 |        1 | HSB_CI ≤ HSA_CO ≥ SPA_CLK          |

The following is a comprehensive list of the legal combinations for the above described frame sync and clock multiplexing configurations:

- FS Combinations 1-4 are compatible with all CLK Combinations (9-16)
- CLK Combinations 9-12 are compatible with all FS Combinations (1-8)
- FS Combination 5 is only compatible with CLK Combination 13 (and vice versa)
- FS Combination 6 is only compatible with CLK Combination 14 (and vice versa)
- FS Combination 7 is only compatible with CLK Combination 15 (and vice versa)
- FS Combination 8 is only compatible with CLK Combination 16 (and vice versa)

NOTE: Program only the SPORT\_CTL2 register of the HSPORT that is accepting the signal from the other HSPORT. However, be sure to set the SPORT\_CTL\_A.CKRE and SPORT\_CTL\_A.LFS polarity bits to have identical settings between the HSPORTs when making internal connections via the SPMUX block.

## Data Types and Companding

The SPORT uses the data type select field SPORT\_CTL\_A.DTYPE bit to specify one of the four data formats supported by serial ports. These formats apply to any of the operating modes of serial port.

Table 32-9: Data Type Bit Field Settings

|   DTYPE field | SPORT Receiver                                          | SPORT Transmitter    |
|---------------|---------------------------------------------------------|----------------------|
|            00 | Right-justify, zero-fill unused most significant bits   | Normal operation     |
|            01 | Right-justify, sign-extend unused most significant bits | Reserved             |
|            10 | Expand using µ-law                                      | Compress using µ-law |
|            11 | Expand using A-law                                      | Compress using A-law |

These formats apply to data words loaded into the SPORT transmit or receive data buffers. The first two data formats ( 00 and 01 values of SPORT\_CTL\_A.DTYPE ) are applicable only when SPORT is configured as receiver.

When SPORT is configured as transmitter, only the significant bits are transmitted (per the field defined in control register). Therefore, the transmit data buffers are not actually zero-filled or sign-extended.

The other two data formats enable the companding logic on the transmit or receive path. Companding (compressing or expanding) is the process of logarithmically encoding and decoding data to minimize the number of bits sent. The SPORTs of the processor support the two most widely used companding algorithms, A-law, and µ-law. The algorithms are performed according to the CCITT G.711 specification.

If selected, companding applies to both the enabled data channels. When enabled as SPORT transmitter, writes to transmit buffer make the content compressed to 8 bits according to algorithm selected. (The content is zero filled to the width of the transmit word.) Similarly, if configured in receive mode, the 8 bits in the receive data buffers expand in right-justified, zero fill format per the algorithm selected. If companding is enabled in multichannel mode, it applies to all the active channels.

The compression for transmit data requires a minimum word length of 8 for proper function. If SPORT\_CTL\_A.SLEN is less than 7, then expansion does not work correctly. Also, if the data value is greater than 13-bit A-law or 14-bit µ-law maximum, it automatically compresses to the maximum value.

NOTE: The processor companding logic supports in-place companding feature. So, companding can be used for debug without enabling SPORT.

## Companding as a Function

Since the values in the transmit and receive buffers are companded in place, the SPORT can use the companding hardware without transmitting or receiving data, which can be useful during testing or debugging. For companding to execute properly, program the SPORT registers prior to loading data values into the SPORT buffers.

To compress data in place without transmitting, use the following procedure:

1. Set the SPORT as a transmitter ( SPORT\_CTL\_A.SPTRAN = 1) with both primary and secondary data channels disabled ( SPORT\_CTL\_A.SPENPRI = SPORT\_CTL\_A.SPENSEC = 0).
2. Enable companding in the SPORT\_CTL\_A.DTYPE field.
3. Write a 32-bit data word to the transmit buffer.
4. Wait two system clock cycles to allow the SPORT companding hardware to reload the transmit buffer with the companded value. Any instructions that do not access the transmit buffer can be used to cause this delay.
5. Read the 8-bit compressed value from the transmit buffer.

To expand data in place, use the same sequence of operations with the receive buffer instead of the transmit buffer.

## Transmit Path

When the SPORT\_CTL\_A.SPTRAN control bit is set, the HSPORT is in transmit mode. Primary and secondary transmit data paths are then enabled using the SPORT\_CTL\_A.SPENPRI and SPORT\_CTL\_A.SPENSEC bits, respectively. The primary and secondary datapaths are unique and identical, each including its own transmit data buffer, optional companding logic, and transmit shift register.

The data buffer on the primary transmit data path is SPORT\_TXPRI\_A , and the data buffer on the secondary transmit data path is SPORT\_TXSEC\_A . The transmit data buffer and output shift register form a FIFO type of structure. When packing is disabled ( SPORT\_CTL\_A.PACK = 0), the SPORT can hold as many as three data words. If packing is enabled ( SPORT\_CTL\_A.PACK = 1), the serial port can hold two packed data words at any time.

The transmit data for primary and secondary channels is written to the SPORT\_TXPRI\_A and SPORT\_TXSEC\_A transmit data buffers, respectively. The transmit data buffers can be accessed in core mode through the peripheral bus or in DMA mode through the DMA bus. When a SPORT is configured in transmit mode, the receive paths are deactivated and do not respond to serial clock or frame sync signals. Because the receive data buffers and receive shift registers are also deactivated, reading from an empty and inactive receive data buffer can cause the core to hang indefinitely.

NOTE: Be sure to avoid accesses to inactive data buffers. Such accesses can cause unpredictable SPORT behavior or a hang condition and are not reported in any status register.

This data can optionally be compressed in hardware according to the selected algorithm (µ-law or A-law) and then automatically transferred to the transmit shift register. The shift register, clocked by the SPORT\_ACLK signal, then serially outputs this data on the SPORT\_AD0 and/or SPORT\_AD1 pins (if both are enabled, these output data bits are transmitted synchronously). If the SPORT uses a framing signal, the SPORT\_AFS signal indicates the start of the serial word transmission.

When using DMA mode, a single DMA feeds the data buffers of the enabled channels (primary and/or secondary). When using both channels, interleave the data of these channels starting with the primary channel in the transmit buffer.

When the SPORT is configured in non-multichannel mode as a transmitter, the enabled SPORT data pins ( SPORT\_AD0 and/or SPORT\_AD1 ) are always driven. When a SPORT channel is enabled, data from the transmit data buffer is loaded into the transmit shift register. The shift register then immediately latches the first bit of data (either the LSB or MSB, depending on the SPORT\_CTL\_A.LSBF configuration bit) and drives it to the respective data pin such that it is ready when the frame sync signal asserts. Similarly, if the frame sync period exceeds the serial word length, then the data pins are driven with the first bit of the next word for transmission immediately after the active word completes, and the outputs are held during the inactive serial clock cycles (clock cycles between frame sync pulses).

When the SPORT is configured in multichannel mode, the data pins are driven only during active transmit channels and are always three-stated during inactive channel slots.

The SPORT provides status of transmit data buffers and also error detection logic for transmit errors such as an underrun condition. See the Error Detection (Status) Interrupt section for more details.

## Receive Path

When the SPORT\_CTL\_A.SPTRAN bit is cleared, the SPORT is in receive mode. Primary and/or secondary receive data paths can be enabled by setting the SPORT\_CTL\_A.SPENPRI and SPORT\_CTL\_A.SPENSEC configuration bits, respectively. These data paths are unique but identical, each with a receive shift register, optional companding logic, and a receive data buffer.

The data buffer on the primary receive path is SPORT\_RXPRI\_A , and the data buffer on the secondary receive path is SPORT\_RXSEC\_A . The receive data paths act like a three-deep (32-bit words) FIFO because they have two data registers plus an input shift register.

Upon enabling the SPORT data channels, the input shift register shifts in data bits on the SPORT\_AD0 and/or SPORT\_AD1 pins, synchronous to the SPORT clock signal. If the SPORT uses a framing signal, the SPORT\_AFS signal indicates the beginning of the serial word (or frame) to be received. When an entire word is shifted into the primary and secondary channels, the data can be optionally expanded in hardware according to a selected algorithm (µ-law or A-law) and then automatically transferred to the SPORT\_RXPRI\_A and/or SPORT\_RXSEC\_A data buffers.

The receive data buffers can be read in core mode through the peripheral bus or in DMA mode through the DMA bus. When the SPORT uses DMA mode, a single DMA reads the data buffers of the enabled channels (primary and/or secondary) and interleaves them in memory beginning with the primary channel when both channels are enabled. When using both channels, software must de-interleave the data of these channels.

The SPORT provides the status of receive data buffers and also error detection logic for receive errors such as overflow. See the Error Detection (Status) Interrupt section for more details.

When a SPORT is configured in receive mode, the transmit paths are deactivated and do not respond to serial clock or frame sync signals. As the transmit data buffers and transmit shift registers in the data paths are also deactivated, programs must not try to access them.

NOTE: Be sure to avoid accesses to inactive data buffers. Such accesses can cause unpredictable SPORT behavior or a hang condition and are not reported in any status register.

## Operating Modes and Options

The SPORT has a number of operating modes:

- Standard DSP Serial mode
- I 2 S mode
- Left-Justified mode
- Right-Justified mode
- Multichannel (TDM) mode
- Packed I 2 S mode

The SPORT halves within a SPORT top module can be independently configured in any of these operating modes unless they are coupled together using SPMUX logic, in which case they must be configured identically. Each half SPORT has its own set of control and data registers and is programmed similarly.

The Control Bits for SPORT Operating Modes table lists all the programmable configuration bits in the SPORT\_CTL\_A control register, which combine to determine the overall function and operating mode of the SPORT. The columns are arranged according to the setting of the SPORT\_CTL\_A.OPMODE bit that selects between standard DSP/multichannel modes and the various I 2 S modes, and the cell contents are defined as follows:

- Yes - bit is programmable for this mode of operation and may be written
- Reserved - bit is not programmable for this mode of operation and must not be written
- = value - bit must be set to this value to enable this mode of operation
- FUNCTION - indicates alternate function for this bit in this mode

NOTE: When changing operating modes, first clear the SPORT\_CTL\_A register before again writing the register with the new configuration settings.

Table 32-10: Control Bits for SPORT Operating Modes

| Name (Bit #)   | Standard DSP Seri- al Mode   | I 2 S and Left-Justi- fied Mode   | Right-Justified Mode   | Multichannel (TDM) Mode   | Packed I 2 S Mode   |
|----------------|------------------------------|-----------------------------------|------------------------|---------------------------|---------------------|
| SPENPRI (0)    | Valid                        | Valid                             | Valid                  | Valid                     | Valid               |
| DTYPE (2:1)    | Valid                        | Reserved                          | Reserved               | Valid                     | Valid               |
| LSBF (3)       | Valid                        | Reserved                          | Reserved               | Valid                     | Valid               |
| SLEN (8:4)     | Valid                        | Valid                             | Valid                  | Valid                     | Valid               |
| PACK (9)       | Valid                        | Valid                             | Valid                  | Valid                     | Valid               |
| ICLK (10)      | Valid                        | Valid                             | Valid                  | Valid                     | Valid               |
| OPMODE (11)    | = 0                          | = 1                               | = 1                    | = 0                       | = 1                 |
| CKRE (12)      | Valid                        | Valid                             | Valid                  | Valid                     | Valid               |
| FSR (13)       | Valid                        | Reserved                          | Reserved               | Reserved                  | Reserved            |
| IFS (14)       | Valid                        | Valid                             | Valid                  | Valid                     | Valid               |
| DIFS (15)      | Valid                        | Valid                             | Valid                  | Reserved                  | Reserved            |
| LFS (16)       | Valid                        | L_FIRST/PLFS                      | L_FIRST/PLFS           | Valid                     | L_FIRST/PLFS        |
| LAFS (17)      | Valid                        | OPMODE2                           | Valid                  | Reserved                  | Reserved            |
| RJUST (18)     | Reserved                     | Reserved                          | = 1                    | Reserved                  | Reserved            |
| FSED (19)      | Valid                        | Reserved                          | Reserved               | Valid                     | Reserved            |
| TFIEN (20)     | Valid                        | Valid                             | Valid                  | Valid                     | Valid               |
| GCLKEN (21)    |                              | Valid                             | Reserved               | Reserved                  | Reserved            |

Table 32-10: Control Bits for SPORT Operating Modes (Continued)

| Name (Bit #)   | Standard DSP Seri- al Mode   | I 2 S and Left-Justi- fied Mode   | Right-Justified Mode   | Multichannel (TDM) Mode   | Packed I 2 S Mode   |
|----------------|------------------------------|-----------------------------------|------------------------|---------------------------|---------------------|
| SPENSEC (24)   |                              |                                   | Valid                  |                           |                     |
| SPTRAN (25)    |                              |                                   | Valid                  |                           |                     |

## Serial Word Length

The SPORT uses the SPORT\_CTL\_A.SLEN field to determine the word length of the serial data to transmit or receive. Each half SPORT can independently handle word lengths up to 32 bits, and the value that must be programmed to the SPORT\_CTL\_A.SLEN field is obtained from:

## SLEN = Desired SPORT word length - 1

The minimal word length depends on the selected operating mode. Words smaller than 32 bits are right justified in the transmit or receive buffers; however, data can be shifted in or out in MSB or LSB first format, as configured by the SPORT\_CTL\_A.LSBF bit. The received word can also be sign-extended or zero-filled when storing the data to processor memory, as governed by the SPORT\_CTL\_A.DTYPE bit.

The Data Lengths for SPORT Operating Modes table shows the range of valid word lengths for each of the supported SPORT operating modes.

Table 32-11: Data Lengths for SPORT Operating Modes

| Mode                | SPORT Word Length (SLEN+1)   |
|---------------------|------------------------------|
| Standard DSP Serial | 4-32                         |
| I 2 S               | 5-32                         |
| Left-Justified      | 5-32                         |
| Right-Justified     | 5-32                         |
| Multichannel (TDM)  | 5-32                         |
| Packed I 2 S        | 5-32                         |

NOTE: When the companding feature is enabled on the datapath, it limits the word length settings. See Data Types and Companding for more details about word lengths required for companding. When more than 32 bits per frame sync are required to transmit or receive, use the multichannel mode to spread the data across numerous continuous channels.

## Clock Sample and Drive Edges

The SPORT uses two control signals to sample or drive the serial data:

1. Serial clock ( SPORT\_ACLK ) - bit clock for the serial data.
2. Frame sync ( SPORT\_AFS ) - divides the incoming data stream into frames.

These control signals can be internally generated or externally provided, as determined by the SPORT\_CTL\_A.ICLK and SPORT\_CTL\_A.IFS bit settings, respectively.

Data and frame syncs can be sampled on the rising or falling edges of the SPORT clock signal, as determined by the SPORT\_CTL\_A.CKRE bit. By default, the SPORT\_CTL\_A.CKRE = 0 setting configures the falling edge of the SPORT\_ACLK signal as the sampling edge for receive data and externally supplied frame syncs. The receive data and frame syncs can be sampled on the rising edges of SPORT\_ACLK when SPORT\_CTL\_A.CKRE = 1.

NOTE: The SPORT drives transmit data and internal frame sync signals on the opposite serial clock edge of the sampling edge. Be sure to select the same value for SPORT\_CTL\_A.CKRE for transmit and receive functions for any two HSPORTs that are connected, and always verify the correct polarity for any external device connected to the SPORT.

The Frame Sync and Data Driven on Rising Edge figure provides an example of the drive and sample edges when two HSPORTs are connected , each with SPORT\_CTL\_A.CKRE = 0. In this example, the HSPORT that is configured as the transmitter drives the serial clock and frame sync signals, and both HSPORTs are configured for early, active high frame syncs and a word length of eight bits.

Figure 32-3: Frame Sync and Data Driven on Rising Edge

<!-- image -->

NOTE: The SCLK in the Frame Sync and Data Driven on Rising Edge figure is SCLK0.

As shown, the transmitting HSPORT provides the clock and generates the frame sync. Because the HSPORTs are configured for early frame mode, the first bit of data is driven one serial clock later, with subsequent bits being driven on the following rising clock edges in the signal train. When the receiving HSPORT samples the frame sync signal (as indicated in the SAMPLED FS waveform), the SPORT\_CTL\_A.SLEN bit counter is loaded with the SPORT\_CTL\_A.SLEN setting, after which each SPORT\_ACLK decrements the SPORT\_CTL\_A.SLEN counter until the full word is received. In this figure, the DRIVE FS and SAMPLED FS waveforms show the frame sync required for the next word in a continuous data stream. Note that it is legal for this frame sync to be sampled synchronous to the last bit of the previous data being sampled, as the early frame mode means that the data lags the frame sync by one serial clock cycle. If the frame sync were sampled as asserted before the D0 bit is sampled, the frame sync error is logged in the receiver's status register.

Since the transmitter drives the internally-generated frame sync and data on the rising edge of the serial clock, the receiver must use the falling edge to sample the externally-supplied frame sync and data.

## Frame Sync Options

The following sections provide details regarding the programmable aspects of the SPORT frame sync signal. See the specific operating mode sections for more information regarding frame sync requirements and behavior for each specific operating mode.

## Data-Dependent versus Data-Independent Frame Syncs

By default, the generation of a frame sync signal is data-dependent:

- When the SPORT is configured as a transmitter ( SPORT\_CTL\_A.SPTRAN = 1), an internally generated transmit frame sync is output when a new data word has been loaded into the channel transmit buffer of the SPORT (by either the core or the DMA engine).
- When the SPORT is configured as a receiver ( SPORT\_CTL\_A.SPTRAN = 0), an internally-generated receive frame sync is output only when the receive data buffer is not full.

The data-independent frame sync option, enabled by setting the SPORT\_CTL\_A.DIFS bit, allows for the generation of a periodic framing signal, regardless of the status of the data buffers. When this bit is set, the frame sync output will be continuous and periodic, according to the setting of the SPORT\_DIV\_A.FSDIV field.

## Support for Edge-Detected and Level-Sensitive Frame Syncs

The level-sensitive nature of frame sync signals operates well in a noise-free environment. However, if noise corrupts the signals coming into the SPORT, the internal logic can lose synchronization. For example, excessive noise on the frame sync signal may cause the frame sync to be sampled as inactive on the clock edge that it is intended to be synchronous to, but then be sampled at the correct active level one cycle later. Similarly, a noisy clock signal can cause an unintended clock edge, resulting in potential premature sampling of the frame sync signal being applied to the pin.

The Level-Sensitive Frame Sync versus Edge Sensitive Frame Sync figure describes a scenario where an external frame sync signal is corrupted due to noise, causing the receiving SPORT module to incorrectly sample the signal. If the frame sync is driven on the rising edge of the serial clock at t A, the SPORT would normally sample the signal on the falling edge of the serial clock at t B. Due to the noise, however, the SPORT misses the first edge of the frame sync and instead samples it at t C.

Figure 32-4: Level-Sensitive Frame Sync versus Edge Sensitive Frame Sync

<!-- image -->

## NOTE: SCLK in the Level-Sensitive Frame Sync versus Edge Sensitive Frame Sync figure is SCLK0.

When the above occurs, the internal word length counter runs for a period equal to the SPORT\_CTL\_A.SLEN field of the control register, but it erroneously expires at t E rather than at the appropriate point at t D , thus receiving incorrect data. Further, if a new level-sensitive frame sync edge arrives at time t D, the SPORT samples this framing signal again at t E. As such, the frame sync sampling continues to be misaligned with the external data.

To help address this, the SPORT module provides an option to configure the frame sync signal to instead be edgesensitive via the SPORT\_CTL\_A.FSED configuration bit. When this bit is set with active high frame syncs enabled ( SPORT\_CTL\_A.LFS = 0), the rising edge of the frame sync is valid. Conversely, when the frame sync is active low ( SPORT\_CTL\_A.LFS = 1), the falling edge is defined to be valid.

NOTE: SPORT\_CTL\_A.FSED is valid only in external frame sync mode. In internal frame sync mode, the setting of this bit is irrelevant and ignored.

In the above example, an edge-sensitive frame sync signal is not detected at t E  because the edge of the framing signal already occurred in the previous cycle (t D ) and there is no new edge to detect at t E. As a result, the internal word length counter remains idle for this frame, thus ignoring the incorrect data, and the counter correctly resumes operation at t F  when a new frame sync edge is detected.

This activity sets the SPORT\_ERR\_A.FSERRSTAT bit and optionally generates a premature frame sync error interrupt.

Frame sync edge detection is used by default for stereo modes. MCM mode and DSP serial mode choose between edge detection and normal mode of FS detection.

NOTE: When the SPORT is first enabled, an already active externally applied frame sync will not commence operation. The SPORT will wait for a valid change in the frame sync's state from inactive to active before operation begins.

## Early versus Late Frame Syncs

Frame sync signals can occur in the same serial clock cycle as the first bit of the data word (late) or one serial clock cycle before the first bit (early), as controlled by the SPORT\_CTL\_A.LAFS bit.

By default, the frame sync signal is configured to be early ( SPORT\_CTL\_A.LAFS = 0). The first bit of the transmit data word will be driven one serial clock cycle after the frame sync is asserted (whether sensed externally or internally provided), and the first bit of the receive data word is expected to lag the frame sync by one serial clock cycle. The frame sync is not checked again until the entire word has been transferred.

If data transmission is continuous in early framing mode, then an internally-generated frame sync signal will be asserted (pulsed active for one serial clock cycle) synchronous to the last data bit of the current transfer, as the first bit of the next transfer will be immediately driven in the next serial clock cycle (no clocks are wasted). This event is not a premature frame sync error, so the SPORT\_ERR\_A.FSERRSTAT bit is not set.

The frame sync can alternatively be configured as late ( SPORT\_CTL\_A.LAFS = 1), in which case the first bit of the transmit data word is available in the same serial clock cycle that the frame sync is asserted (whether sensed

externally or internally provided), and the first bit of the receive data word is also latched in the same cycle. Serial clock edges latch the receive data bits, but the frame sync signal is checked only during the first bit of each word. Internally generated frame syncs remain asserted for the entire length of the data word in late framing mode.

The Normal Framing (Early Frame Sync) Versus Alternate Framing (Late Frame Sync) figure illustrates these concepts.

Figure 32-5: Normal Framing (Early Frame Sync) Versus Alternate Framing (Late Frame Sync)

<!-- image -->

## Framed versus Unframed Frame Syncs

The use of a frame sync signal is optional for SPORT operation, as controlled by the SPORT\_CTL\_A.FSR bit. When the frame sync is configured to be required ( SPORT\_CTL\_A.FSR = 1), the data is defined to be framed (a frame sync signal must accompany every data word). To allow continuous transmission from the processor, ensure that a new data word is loaded into the transmit buffer before the ongoing transfer is completed (this is automatically cared for when DMA is used to transmit blocks of data).

Data words can be transferred continuously in what is referred to as unframed data mode, which is appropriate for continuous reception, by setting SPORT\_CTL\_A.FSR = 0. In this configuration, a single frame sync is still required to initiate communication, but it is subsequently unrequired once the communication begins. From that point onward, externally provided frame syncs are ignored and internally generated frame syncs are not driven. The Framed versus Unframed Data Stream figure shows the differences in SPORT operation between framed and unframed data modes with the frame sync configured to be early ( SPORT\_CTL\_A.LAFS = 0).

NOTE: When DMA is enabled in a mode where frame syncs are not required, chaining can delay DMA requests. DMA requests are not always serviced frequently enough to guarantee continuous unframed data flow. Monitor status bits or check for a SPORT error interrupt to detect underflow or overflow of data.

Figure 32-6: Framed versus Unframed Data Stream

<!-- image -->

## Frame Sync Polarity

The framing signals can be active high or active low, as governed by the SPORT\_CTL\_A.LFS bit:

- When SPORT\_CTL\_A.LFS = 0, the corresponding frame sync signal is active high.
- When SPORT\_CTL\_A.LFS = 1, the corresponding frame sync signal is active low.

Active high is the default polarity of the frame sync signal.

## Premature Frame Sync Error Detection

A SPORT framing signal is used to synchronize transmit or receive data. In external frame sync mode, any frame sync received during an active frame is premature and invalid. When this occurs, the SPORT\_ERR\_A.FSERRSTAT bit is set to indicate the framing error, and an optional error interrupt request can be generated for this event by setting the SPORT\_ERR\_A.FSERRMSK bit.

NOTE: The SPORT\_ERR\_A.FSERRSTAT bit is not set in the presence of uncleared underflow or overflow errors.

Refer to the Frame Sync Error Detection figure. The frame sync error bit gets set when an unexpected frame sync occurs during the ongoing data transfer (transmission or reception).

Figure 32-7: Frame Sync Error Detection

<!-- image -->

NOTE: SCLK in the Frame Sync Error Detection figure is SCLK0.

Whether a SPORT is receiving or transmitting, its bit count is set to the programmed serial word length when the frame sync is sampled, which is then decremented every subsequent serial clock cycle until the transfer has completed. At this point, the bit count reaches zero and will be reset to the programmed serial length when the next frame sync is sampled. As such, the bit count value is non-zero during an active transfer, and the frame sync error is asserted if a frame sync is sampled when this count is non-zero.

## Mode Selection

The SPORT's operating mode is configured in the SPORT\_CTL\_A and SPORT\_MCTL\_A registers. The SPORT Operating Modes table provides specific guidance to properly program these SPORT control registers for the desired mode of operation.

Table 32-12: SPORT Operating Modes

| Operating Modes     |   SPORT_CTL_A. OPMODE | SPORT_CTL_A.LAFS   | SPORT_CTL_A. RJUST   |   SPORT_MCTL_A.MCE |
|---------------------|-----------------------|--------------------|----------------------|--------------------|
| Standard DSP Serial |                     0 | Programmable       | Reserved             |                  0 |
| I 2 S               |                     1 | 0                  | Reserved             |                  0 |
| Left-Justified      |                     1 | 1                  | Reserved             |                  0 |
| Right-Justified     |                     1 | 1                  | 1                    |                  0 |
| Multichannel        |                     0 | Reserved           | Reserved             |                  1 |
| Packed I 2 S mode   |                     1 | Reserved           | Reserved             |                  1 |

The following sections provide detailed information for each of the supported SPORT modes of operation.

## Standard DSP Serial Mode

The SPORT can be configured in standard DSP serial mode by clearing the SPORT\_CTL\_A.OPMODE and SPORT\_MCTL\_A.MCE bits. This mode provides great flexibility in terms of programmable options to configure the SPORTs to communicate with various serial devices such as serial data converters and audio codecs. In order to properly connect to such devices, various clocking, framing, and data formatting options are available.

## Timing Control Bits

Several bits in the SPORT\_CTL\_A control register define the configuration of the SPORT in standard DSP serial mode:

- SLEN: serial word length (4-32 bits)
- LSBF: shift LSB or MSB first
- ICLK: internally generated or externally provided serial clock
- CKRE: sample on rising or falling edge of the serial clock
- IFS: internally generated or externally provided frame sync
- FSR: framed or continuous operation
- DIFS: data-dependent or data-independent frame sync
- LFS: active high or active low frame sync
- LAFS: frame sync synchronous to data or one clock cycle before it
- PACK: 16-bit to 32-bit packing option

- GCLKEN: free-running or gated clock

## Clocking Options

In standard DSP serial mode, the SPORTs can either accept an external serial clock or generate one internally, as controlled by the SPORT\_CTL\_A.ICLK bit. For internally generated serial clocks ( SPORT\_CTL\_A.ICLK = 1), the SPORT\_DIV\_A.CLKDIV field configures the serial clock rate from the system clock.

The SPORT clock can also be gated, where it is only valid during an active transfer, as controlled by the SPORT\_CTL\_A.GCLKEN bit.

The SPORT clock edge used for driving and sampling of serial data and frame syncs is configured using the SPORT\_CTL\_A.CKRE bit:

- If SPORT\_CTL\_A.CKRE = 0, input data and frame sync signals are sampled on the falling edge of the serial clock, and output data and frame sync signals are driven on the rising edge.
- If SPORT\_CTL\_A.CKRE = 1, input data and frame sync signals are sampled on the rising edge of the serial clock, and output data and frame sync signals are driven on the falling edge.

## Stereo Modes

The SPORTs support three widely used stereo modes of operation:

- I 2 S mode
- Left-Justified mode
- Right-Justified mode

In these modes, the serial data stream consists of left and right channels. The following sections describe these modes in more detail.

## Channel Order

The active low frame sync ( SPORT\_CTL\_A.LFS ) bit is used to determine the polarity of the frame sync signal in the non-stereo modes of operation. For the stereo modes of operation, it instead controls whether the right or left channel is first in the data transfer. The Channel Order Bit Settings table shows which word is transmitted or received first, based on the setting of the SPORT\_CTL\_A.LFS bit.

Table 32-13: Channel Order Bit Settings

| Mode                              | SPORT_CTL_A.LFS =0   | SPORT_CTL_A.LFS =1   |
|-----------------------------------|----------------------|----------------------|
| Left-Justified or Right-Justified | Left channel first   | Right channel first  |
| I 2 S or Packed I 2 S             | Right channel first  | Left channel first   |

## I 2 S Mode

I 2 S mode is a commonly used stereo mode, where left and right channel data words are interleaved in the serial data stream and each transition of the frame sync signal is associated with one of the channels. The left channel data is transferred during the low segment of the frame sync signal, and the right channel data is transferred during the high segment of the frame sync signal. As such, the frame sync signal is treated as a left-right (L/R) clock in this mode.

To set up the SPORT in I 2 S mode, the following configuration is required:

- SPORT\_CTL\_A.OPMODE = 1
- SPORT\_CTL\_A.LFS = 0
- SPORT\_MCTL\_A.MCE = 0

## Protocol Configuration Options

Several bits in the SPORT\_CTL\_A control register must be configured to be compliant with the I 2 S standard, but they can be otherwise configured to support non-standard operations

- SLEN-programmable (allowable word lengths are 5-32 bits)
- LSBF-set to 0 (MSB first)
- ICLK-programmable (serial bit clock can be internally generated or externally provided)
- IFS-programmable (serial L/R clock source must match serial bit clock source)
- LFS-set to 1 (left channel first)
- CKRE-set to 1 (sample L/R clock and data on rising edge of bit clock)

## Serial Bit Clock and L/R Clock Rates

When the SPORT is configured to generate the bit clock and the L/R clock ( SPORT\_CTL\_A.ICLK = SPORT\_CTL\_A.IFS = 1), set the serial bit clock rate using the SPORT\_DIV\_A.CLKDIV bit field and the L/R clock rate using the SPORT\_DIV\_A.FSDIV bit field.

The Word Select Timing in I 2 S Mode figure shows the SPORT timing in I 2 S mode. The data lags the L/R clock transition by one SCLK0 cycle, and the transfer begins with the left channel data word first.

Figure 32-8: Word Select Timing in I 2 S Mode

<!-- image -->

## Left-Justified Mode

Left-justified mode is a stereo mode subset of the I 2 S standard. As in I 2 S mode, the frame sync signal acts as a leftright clock (L/R clock), where left and right data samples are transferred each L/R clock period. The left channel is associated with the high segment of the frame sync, and the right channel aligns with the low segment of the frame sync. The difference between left-justified mode and standard I 2 S mode is that the channel data is driven in the same bit clock cycle as the L/R clock transition (rather than one bit clock cycle later), such that the MSB is synchronous with the leading edge of the frame sync transition.

To set the SPORT up in left-justified mode, the following configuration is required:

- SPORT\_CTL\_A.OPMODE = 1
- SPORT\_CTL\_A.LAFS = 1
- SPORT\_MCTL\_A.MCE = 0

## Protocol Configuration Options

Several bits in the SPORT\_CTL\_A control register must be configured to operate the SPORT in left-justified mode, but they can be otherwise configured as well:

- SLEN: programmable (allowable word lengths are 5-32 bits)
- LSBF: set to 0 (MSB first)
- ICLK: programmable (serial bit clock can be internally generated or externally provided)
- IFS: programmable (serial L/R clock source must match serial bit clock source)
- LFS: set to 0 (left channel first)
- CKRE: set to 1 (sample L/R clock and data on rising edge of bit clock)

## Serial Bit Clock and L/R Clock Rates

If the SPORT is configured to generate the bit clock and the L/R clock ( SPORT\_CTL\_A.ICLK = SPORT\_CTL\_A.IFS = 1), set the serial bit clock rate using the SPORT\_DIV\_A.CLKDIV bit field and the L/R clock rate using the SPORT\_DIV\_A.FSDIV bit field.

The Word Select Timing in Left-Justified Mode figure shows the SPORT timing in left-justified mode. The start of a data sample is synchronous to the L/R clock transition, and the transfer begins with the left channel data word first.

Figure 32-9: Word Select Timing in Left-Justified Mode

<!-- image -->

## Right-Justified Mode

Right-justified mode is a stereo mode subset of the I 2 S standard. As in I 2 S mode and left-justified mode, the frame sync signal acts as a left-right clock (L/R clock), where left and right data samples are transferred each L/R clock period. The left channel is associated with the high segment of the frame sync, and the right channel aligns with the low segment of the frame sync. The difference between right-justified mode and standard I 2 S mode is that the LSB of the channel data ends at the point that the L/R clock transitions to frame the next sample (rather than one bit clock cycle after the L/R clock transition).

To set the SPORT up in right-justified mode, the following configuration is required:

- SPORT\_CTL\_A.OPMODE = 1

```
· SPORT_CTL_A.RJUST = 1 · SPORT_MCTL_A.MCE = 0
```

## Timing Control Bits

Several bits in the SPORT\_CTL\_A control register must be configured to operate the SPORT in right-justified mode, but they can be otherwise configured as well:

- SLEN: programmable (allowable word lengths are 5-32 bits)
- LSBF: set to 0 (MSB first)
- ICLK: programmable (serial bit clock can be internally generated or externally provided)
- IFS: programmable (serial L/R clock source must match serial bit clock source)
- LFS: set to 0 (left channel first)
- CKRE: set to 1 (sample L/R clock and data on rising edge of bit clock)

## Serial Bit Clock and L/R Clock Rates

If the SPORT is configured to generate the bit clock and the L/R clock ( SPORT\_CTL\_A.ICLK = SPORT\_CTL\_A.IFS = 1), set the serial bit clock rate using the SPORT\_DIV\_A.CLKDIV bit field and the L/R clock rate using the SPORT\_DIV\_A.FSDIV bit field.

The Word Select Timing in Right-Justified Mode figure shows the SPORT timing in right-justified mode. The transmitter aligns the transmit data such that the last bit of the serial word is sent in the last clock cycle of the L/R clock (frame sync) signal marking the channels.

Figure 32-10: Word Select Timing in Right-Justified Mode

<!-- image -->

NOTE: For some SPORT-compatible ADCs or DACs such as the AD1871, right-justified mode is limited to commonly used ratios such as 64 FS and 128 FS. FS is the sampling frequency of ADCs and DACs, referred to as the SPORT's L/R clock (frame sync) signal.

Consider the SPORT timing for right-justified mode, as shown in the Timing Comparison Between Different Stereo Modes figure. The frame sync width is limited to 32 SPORT clock periods (or 32 bits per channel) if:

- the SPORT's frame sync (L/R clock) runs at the FS rate, and
- the SPORT's serial bit clock runs at the 64 FS rate

The limitation applies to the frame sync width of either channel. If the data is confined to 24 bits, the SPORT introduces a 32-24=8-bit clock delay before it starts to transmit or capture data.

Figure 32-11: Timing Comparison Between Different Stereo Modes

<!-- image -->

Similarly, to support the 128 FS bit clock frequency, the frame sync width becomes 64 serial bit clock periods per channel. The delay can be a maximum of 59 bit clocks (64 - 5, which is the minimum serial data length in rightjustified mode).

The starting point of the first bit is delayed so that the LSB of the serial data aligns properly with the end of the channel. A 6-bit counter is added for this purpose in the stereo mode counter, which is programmed by writing the least significant six bits of the SPORT\_MCTL\_A.WOFFSET field. Though this is a multichannel mode configuration register, the SPORT uses these bits in right-justified mode to configure the offset from the transition of the L/R

clock to where the first data bit must be driven to have the end of the last bit align properly with the next L/R clock transition. The software must program this register with the appropriate delay.

## Multichannel (TDM) Mode

The multichannel mode of SPORT operation allows the SPORT to communicate as part of a time division multiplexed (TDM) serial system. In TDM communications, a large frame of streamed serial data words is defined to be a particular length. It consists of a specific number of channels, and each channel contains one serial data word of the defined data length. For example, a 24-word block of 24-bit data can be defined to be a window within a frame, having a duration of 576 bit clocks and comprised of 24 continuous channels. The SPORT is configured to transfer on specific channels within a defined window in this frame.

To set the SPORT up in multichannel mode, the following configuration is required:

- SPORT\_CTL\_A.OPMODE = 0
- SPORT\_MCTL\_A.MCE = 1

In multichannel mode, the SPORT can selectively transfer data on up to a maximum window size of 128 continuous channels out of a maximum 1024-channel frame while ignoring all the disabled channels within the window and all the channels outside the window. The SPORT can do any of the following on each channel:

- Transmit data ( SPORT\_CTL\_A.SPTRAN = 1)
- Receive data ( SPORT\_CTL\_A.SPTRAN = 0)
- Do nothing (during inactive channels)

Channel selection is configured in the half SPORT multichannel select registers ( SPORT\_CS0\_A -SPORT\_CS3\_A ) before enabling SPORT operation for multichannel mode. Programming of these registers is especially important in DMA data unpacked mode, since the SPORT data buffers begin operation immediately after the SPORT data lines are enabled. Be sure to enable multichannel operation (set the SPORT\_MCTL\_A.MCE bit) prior to enabling the SPORT itself.

## Clocking Options

In multichannel mode, the SPORTs can either accept an external serial clock or generate one internally, as governed by the SPORT\_CTL\_A.ICLK bit. For an internally-generated serial clock ( SPORT\_CTL\_A.ICLK = 1), the SPORT\_DIV\_A.CLKDIV bit field is used to configure the serial clock rate, as derived from the system clock.

The serial clock edges used to drive and sample data and frame syncs are also configurable using the SPORT\_CTL\_A.CKRE bit:

- If SPORT\_CTL\_A.CKRE = 0, input data and frame sync signals are sampled on the falling edge of the serial clock, and output data and frame sync signals are driven on the rising edge.
- If SPORT\_CTL\_A.CKRE = 1, input data and frame sync signals are sampled on the rising edge of the serial clock, and output data and frame sync signals are driven on the falling edge.

## Frame Sync Options

The frame sync signal synchronizes the channels and restarts each multichannel sequence, starting with the channel 0 data word. For internally-generated frame syncs ( SPORT\_CTL\_A.IFS = 1), the frame sync period in multichannel mode is defined as:

FS period = [( SPORT\_CTL\_A.SLEN + 1) × number of channels] - 1

The active level for the frame sync signal is also configurable by programming the SPORT\_CTL\_A.LFS bit. Set this bit to make the frame sync an active low signal, and clear it to make it active high.

In multichannel mode, frame sync timing resembles late framing mode (although the SPORT\_CTL\_A.LAFS bit is reserved in this mode). The first bit of the transmit data word is driven and the first bit of the receive data word is sampled in the same serial clock cycle as the frame sync, provided there is no programmed frame delay ( SPORT\_MCTL\_A.MFD = 0).

Once the frame sync signal is asserted, word transfers are performed continuously for the duration of the active window, and no further frame syncs are required for different channels within the window. As such, internally-generated frame syncs are always data-independent, and the SPORT\_CTL\_A.DIFS bit is reserved.

## Transmit Data Valid (TDV)

Each SPORT features a transmit data valid signal ( SPORT\_ATDV ), which is driven high during enabled transmit channels. Because the SPORT output data signals are three-stated during inactive channels, the SPORT\_ATDV signal signifies when the processor is actively driving the SPORT data outputs, thus serving as an output-enable signal for the data transmit pin(s).

## Active Channel Selection Registers

In multichannel mode, the SPORT supports a window size of up to 128 channels for transmitting or receiving data, where it can selectively receive or transmit data in any of these 128 channels. Each channel can be individually enabled or disabled using the multichannel selection registers ( SPORT\_CS0\_A to SPORT\_CS3\_A ) to select the channels in which to transfer data during a multichannel communication stream. Data words associated with enabled channels are transmitted or received in the respective channels, while disabled channels cause a transmit SPORT to three-state the data output pins and a receive SPORT to ignore the data.

The four 32-bit multichannel selection registers combine to form up to a 128-bit meta-register to accommodate the maximum window size of 128 channels. Setting any bit within these registers enables the associated channel. The 128 channels are sequentially numbered from bit 0 in the SPORT\_CS0\_A register (corresponding to channel 0 of the window) to bit 31 of the SPORT\_CS3\_A register (corresponding to channel 127 of the window). For example, setting bit 13 of the SPORT\_CS1\_A register enables channel number 45 (add 32 for the channels in the SPORT\_CS0\_A ). Likewise, setting bit 5 of the SPORT\_CS3\_A register enables channel number 101 (add 96 for the 32 channels in each of the SPORT\_CS0\_A , SPORT\_CS1\_A , and SPORT\_CS2\_A registers).

## Multichannel Frame Delay (MFD)

The multichannel frame delay ( SPORT\_MCTL\_A.MFD ) field specifies the delay in serial bit clocks between the frame sync pulse and the first data bit in the frame. This configurability allows the processor to work with different types of telephony interface devices.

As SPORT\_MCTL\_A.MFD is a 4-bit field, the maximum value allowed for the frame delay is 15 serial clock cycles. When set to 0, the frame sync is concurrent with the first data bit. If SPORT\_MCTL\_A.MFD &gt;0, a new frame sync can occur during the last channel(s) of a previous frame and still be valid (does not cause a frame sync error).

NOTE: When the required frame delay exceeds 15 serial clocks, use the window offset field ( SPORT\_MCTL\_A.WOFFSET ) to delay the start of channel 0 in increments of the serial word length, and then adjust SPORT\_MCTL\_A.MFD accordingly. For example, when the serial word length is 12 bits and the desired frame delay is 16 serial clock cycles, set the SPORT\_MCTL\_A.WOFFSET field to 1 to insert a 12-bit delay after the frame sync to where the channel 0 data begins, and then program SPORT\_MCTL\_A.MFD to four (that is, 16 - 12).

NOTE: When the ASRC module is used with SPORTs, note the following:

- In ASRC TDM mode, the delay between the frame sync (FS) and the data is one clock cycle.
- For the SPORT MFD feature with respect to ASRC TDM mode, select the MFD ( SPORT\_MCTL\_A.MFD / SPORT\_MCTL\_B.MFD ) bit as follows:
- The ASRC input is expecting one clock cycle between the FS and the data. Configure the MFD transmit bit to 1 ( the SPORT which transmits the data to ASRC serial in)
- The ASRC output has one clock cycle delay between the FS and the data. Configure the MFD receive bit to 1 (for the SPORT which receives the ASRC serial out data).

## Window Size (WSIZE)

Select the number of channels used in multichannel operation by programming the 7-bit SPORT\_MCTL\_A.WSIZE field. This field must be set to the actual number of channels minus one ( SPORT\_MCTL\_A.WSIZE = Number of channels -1).

The 10-bit SPORT\_MSTAT\_A.CURCHAN field holds the channel number currently being serviced during multichannel operation.

## Window Offset (WOFFSET)

The window offset ( SPORT\_MCTL\_A.WOFFSET ) field specifies where in the 1024-channel frame to place the start of the active window (up to 128 channels long). A value of 0 specifies no channel offset from the frame sync (channel 0 immediately follows it). Any non-zero value indicates the number of channels that come between the frame sync and the start of channel 0 of the active frame, with 896 (for example, 1024-128) being the largest value that permits using all 128 channels.

As an example, a program could define an active window comprised of eight channels ( SPORT\_MCTL\_A.WSIZE = 7) with a window offset of 93 ( SPORT\_MCTL\_A.WOFFSET = 93). If configured in this fashion, the 8-channel window that the SPORT will transfer within resides in the channel range from 93 to 100 in the up-to-1024-channel frame.

Do not change the window offset or the number of multichannel slots ( SPORT\_MCTL\_A.WSIZE ) while the SPORT is enabled. If the combination of the window size and offset place any portion of the window out-of-range relative to the channel counter, none of the channels are enabled.

## Companding Selection

Like the other operating modes, companding logic can optionally be applied to serial data (compression logic for transmit mode or expansion logic for receive mode). The two widely used companding algorithms, A-law and µ-law, are selectable using the SPORT\_CTL\_A.DTYPE field.

If companding is enabled, the companding algorithm is applied to both the primary and secondary datapaths. In multichannel mode, companding can be applied to either all or none of the enabled channels (companding cannot be selected on a per-channel basis).

## Multichannel DMA Data Packing (MCPDE)

Multichannel DMA data packing and unpacking are enabled using the SPORT\_MCTL\_A.MCPDE bit.

When set, data is packed, and the SPORT expects the data in the DMA buffer to correspond only with enabled SPORT channels. For example, if only channels 1 and 9 are enabled in a 10-channel window ( SPORT\_MCTL\_A.WSIZE = 9), the SPORT expects the buffer to be exactly two words in length, where channel 1 is associated with the first element in the buffer and channel 9 is associated with the second.

When cleared, data is unpacked, and the SPORT expects the DMA buffer to have a word for each of the channels in the active window, whether the channel is enabled or not. As such, the DMA buffer size must be exactly the size of the window. Using the same example as the packed case above, if only channels 1 and 9 are enabled in a 10-channel window ( SPORT\_MCTL\_A.WSIZE = 9), then the DMA buffer size is ten words. The data at offsets 1 and 9 within the buffer are associated with the data transfers of channels 1 and 9, respectively. The rest of the words in the buffer are unused.

## Packed I 2 S Mode

The SPORT supports a packed I 2 S mode, which can be used for audio codec communications using multiple channels. This mode allows applications to send more than the standard 32 bits per channel available through standard I 2 S mode. Packed mode is implemented using standard multichannel mode (and is therefore programmed similarly to multichannel mode).

To set the SPORT up in packed I 2 S mode, the following configuration is required:

- SPORT\_CTL\_A.OPMODE = 1

```
· SPORT_MCTL_A.MCE = 1
```

Like multichannel mode, packed I 2 S mode also supports a maximum of 128 channels, where up to 128 channels of data can be transferred for every transition of the frame sync signal acting as an L/R clock (for example, up to 128 left-channel words transfer during the high portion of the L/R clock, and up to 128 right-channel words transfer during the low portion).

As shown in the Packed I 2 S Mode 128 Operation figure, the packed waveforms are the same as those waveforms used in multichannel mode, except the frame sync is toggled for every frame and emulates I 2 S mode.

Figure 32-12: Packed I 2 S Mode 128 Operation

<!-- image -->

## Serial Bit Clock Options

In packed I 2 S mode, the SPORTs can either accept an external serial bit clock or generate one internally, as governed by the SPORT\_CTL\_A.ICLK configuration bit. For an internally-generated serial bit clock ( SPORT\_CTL\_A.ICLK = 1), use the SPORT\_DIV\_A.CLKDIV bit field to configure the serial bit clock rate from the system clock.

The serial bit clock edge that is used for sampling or driving data and frame syncs is programmable using the SPORT\_CTL\_A.CKRE bit.

## L/R Clock (Frame Sync) Options

The frame sync period in packed I 2 S mode is defined as:

FS period = [( SPORT\_CTL\_A.SLEN + 1) × number of channels] - 1.

The L/R clock can be supplied externally or internally generated depending on the SPORT\_CTL\_A.IFS bit setting. The logic level of the L/R clock associated with the left and right channel data can be changed using the SPORT\_CTL\_A.LFS configuration bit.

## Gated Clock Mode

Some system components such as ADCs and DACs utilize a SPI-compatible protocol for the interface. To communicate with such devices, the SPORT must support a gated clock, where the data valid information is embedded in

the clock (for example, the clock only toggles when data is valid). This gated clock feature is enabled using the SPORT\_CTL\_A.GCLKEN bit.

To enable the gated clock mode of operation, program the SPORT to comply with the following requirements.

- Do not enable gated clock functionality in right-justified or multichannel mode
- Gated clock mode has the following requirements for other control bits:
- The serial clock and frame sync signals must have the same source ( SPORT\_CTL\_A.ICLK = SPORT\_CTL\_A.IFS )
- Unframed mode is not supported ( SPORT\_CTL\_A.FSR must be set)
- Clear the SPORT\_CTL\_A.DIFS bit in transmit mode; set it in receive mode
- Satisfy the following necessary conditions when gated clock mode is enabled:
- Seven serial clock cycles are required between enabling the SPORT and the first frame sync. If this requirement is not met, the SPORT can drop the first data (for subsequent data, this requirement is not applicable)
- For externally-provided clock and frame sync, the frame sync must be inactive during clock synchronization after the SPORT has been enabled
- For an edge-detected frame sync ( SPORT\_CTL\_A.FSED = 1), the frame sync must transition back to the inactive state before the current word transfer is complete (or when the clock is still running). If this requirement is not met, the SPORT does not recognize the next valid frame sync and skips the channel. The SPORT continues to skip the frame syncs until the frame sync transitions back to an inactive state while the clock is active.

## Data Transfers and Interrupts

SPORT data can be transferred to or from internal or external memory by two methods:

- Core-driven, single-word transfers
- DMA-driven, multiple-word transfers (optionally with multiple work units)

Core-driven transfers use SPORT interrupts to signal the processor core to perform MMR-based single-word transfers to or from the SPORT data buffers. DMA can be set up to automatically transfer a configurable number of serial words between the SPORT transmit/receive data buffers and memory, and then generate a data completion interrupt request when a work unit or a series of work units completes, thus signaling by the SEC to the processor core that a block of data has been transferred.

The following sections provide information on core-driven and DMA-driven data transfers.

## Data Buffers

When programming the serial port data channels (primary or secondary) as a transmitter by setting SPORT\_CTL\_A.SPTRAN =1, only the corresponding transmit data buffers ( SPORT\_TXPRI\_A and SPORT\_TXSEC\_A ) become active. The receive data buffers ( SPORT\_RXPRI\_A and SPORT\_RXSEC\_A ) remain inactive. Similarly, when the SPORT data channels are programmed for receive operation ( SPORT\_CTL\_A.SPTRAN =0), then only corresponding receive data buffers ( SPORT\_RXPRI\_A and SPORT\_RXSEC\_A ) are active. Do not attempt to read or write inactive data buffers. When the processor operates on the inactive transmit or receive buffers while the SPORT is enabled, unpredictable results can occur.

Each of these buffers is 32-bit wide (corresponds to maximum serial data word length). When using word lengths less than 32 bits for SPORT operation, the data in these buffers is automatically right justified. (The LSB bit of data is at the bit 0 location of the buffer). The upper unused bits can be zero-filled or sign-extended depending on SPORT\_CTL\_A.DTYPE field.

## Transmit Data Buffers ( SPORT\_TXPRI\_A and SPORT\_TXSEC\_A )

When enabled as a transmitter ( SPORT\_CTL\_A.SPTRAN =1), each SPORT half has its own set of transmit data buffers. The primary (0) and secondary (1) datapaths of each SPORT half have separate data buffers, referred to as SPORT\_TXPRI\_A and SPORT\_TXSEC\_A , respectively.

These transmit data buffers are 32 bits wide. Load these buffers with the data for transmission on the primary and secondary data channels. The DMA controller loads the data automatically. Or, the program running on the processor core loads the data manually.

Together with the output shift register, transmit data buffers act like a two-location FIFO. If data packing is disabled ( SPORT\_CTL\_A.PACK =0), the transmit path can hold as many as three data words. If data packing is enabled ( SPORT\_CTL\_A.PACK =1), it can hold two packed data words at any given time.

When the transmit shift register becomes empty (transfer out all the bits of previous word), data in the transmit data buffer is automatically loaded into it. An interrupt occurs when the output transmit shift register has been loaded, signifying that the transmit data buffer is empty and ready to accept the next word. This interrupt does not occur when serial port is operating in DMA mode or when the corresponding interrupt enable mask bit is set.

If only the primary datapath of a SPORT half is enabled, programs must not write to the inactive secondary transmit data buffer and conversely. If the core keeps writing to the inactive buffer, the status of that transmit buffer becomes full. This state can cause the core to hang indefinitely, since data is never transmitted to the output shift register.

## Receive Data Buffers ( SPORT\_RXPRI\_A and SPORT\_RXSEC\_A )

When enabled as receiver ( SPORT\_CTL\_A.SPTRAN =0), each SPORT half has its own set of receive data buffers. The primary (0) and secondary (1) datapaths of each SPORT half have separate data buffers, referred as SPORT\_RXPRI\_A and SPORT\_RXSEC\_A , respectively. T ogether with input shift register, the receive data buffers act like a three-location FIFO, as the receive path has two data registers.

These receive data buffers are the 32 bits wide. These buffers are automatically loaded from the receive shift register when a complete word has been received into it. An interrupt occurs when the receive data buffer is loaded, signifying that new data is available in the receive data buffer and is ready to read. This interrupt does not occur when the serial port is operating in DMA mode or when the corresponding interrupt enable mask bit is set.

If only the primary datapath of a SPORT half is enabled, programs must not read from the inactive secondary receive data buffer and conversely. If the core keeps reading from the inactive buffer, the status of that receive buffer becomes empty. This state can cause the core to hang indefinitely since new data is never received through the input shift register.

## Data Buffer Status

The SPORT provides status information about its primary and secondary data buffers through the SPORT\_CTL\_A.DXSPRI and SPORT\_CTL\_A.DXSSEC bits, respectively. It also provides error status information through the corresponding SPORT\_CTL\_A.DERRPRI and SPORT\_CTL\_A.DERRSEC bits, respectively. Depending on the SPORT\_CTL\_A.SPTRAN bit setting, these bits reflect the status of either the pair of transmit ( SPORT\_TXPRI\_A and SPORT\_TXSEC\_A ) or receive ( SPORT\_RXPRI\_A and SPORT\_RXSEC\_A ) buffers, indicating whether the buffer is full, partially full, or empty.

When attempting to read from an empty receive buffer or write to a full transmit buffer, the SPORT delays access until the buffer is ready, potentially resulting in excessive MMR bus response times. To avoid this when doing coredriven transfers, always check the buffer status to determine if the access can be made. The SPORT updates the status bits in the SPORT\_CTL\_A register during reads and writes by the core processor.

NOTE: These status bits are updated during reads and writes from the core processor even when the SPORT is disabled.

Two complete 32-bit words can be stored in the receive buffer while a third word shifts in. Therefore, almost three complete words can be received without the receive buffer being read before an overflow occurs. After receiving the third word completely, the shift register contents overwrite the second word, which will occur if the first word has not yet been read by the processor core or the DMA controller. This receive overflow condition is flagged through the error status bits of the SPORT\_CTL\_A register on the last bit of the third word.

## Data Buffer Packing

When the SPORT is configured as a receiver with a serial data word length of 16 or less, the received data words can be packed into a 32-bit word. Similarly, if the SPORT is configured as a transmitter with a serial data word length of 16 or less, then 32-bit words being transmitted can be unpacked into 16-bit words. The SPORT\_CTL\_A.PACK bit is used to select this packing or unpacking feature.

When SPORT\_CTL\_A.PACK = 1, two consecutive received words are packed into a single 32-bit word, or each 32-bit word is unpacked and transmitted as two 16-bit words. The first 16-bit (or smaller) word is right-justified in bits 15-0 of the packed word, and the second 16-bit (or smaller) word is right-justified in bits 31-16. This packing method applies to both receive (packing) and transmit (unpacking) operations. In this case, the transmit and receive interrupt requests are generated for the 32-bit packed words, not for each 16-bit word.

NOTE: When 16-bit received data is packed into 32-bit words and stored in normal word space in the processor's internal memory, the 16-bit words can be read or written using short word space addressing.

## Single-Word (Core) Transfers

The SPORTs can transmit or receive individual data words with interrupt requests occurring as each data word is transferred. When a SPORT is enabled with the corresponding DMA channel disabled, interrupt requests are generated when:

- a complete word has been received in the receive data buffer or
- the transmit data buffer is not full

When performing core transfers, be sure to access only those buffers that are associated with enabled datapaths, as governed by the transfer direction ( SPORT\_CTL\_A.SPTRAN ) bit and the primary/secondary ( SPORT\_CTL\_A.SPENPRI / SPORT\_CTL\_A.SPENSEC ) data enable bits. If inactive SPORT data buffers are read from or written to by the core while the SPORT is enabled, the core can hang. For example, if a half SPORT is programmed to be a transmitter and the core reads from one of the receive buffers associated with that half SPORT, the core can hang as if it were reading an empty buffer that is active and awaiting new data to arrive. Because this is a transmitting HSPORT, that data will never arrive, thus locking the core up until the SPORT is reset. To avoid such a situation, be sure to check the status of the appropriate data buffer before attempting a core access to it by interrogating the SPORT\_CTL\_A.DXSPRI or SPORT\_CTL\_A.DXSSEC status bits.

## DMA Transfers

Direct memory access (DMA) provides a mechanism for transferring an entire block of serial data before an interrupt is generated. The processor's on-chip DMA controller automatically handles the DMA transfer, thus allowing the processor core to run in parallel until the entire block of data is transferred. When the interrupt request occurs, a service routine can then process the entire block of data (rather than react to single words), thus significantly reducing overhead.

Each half SPORT has a dedicated DMA channel that serves both the primary and secondary datapaths. When configured as a transmitter ( SPORT\_CTL\_A.SPTRAN = 1) with both the primary and secondary datapaths enabled ( SPORT\_CTL\_A.SPENPRI = SPORT\_CTL\_A.SPENSEC = 1), the DMA channel requires that the source DMA buffer interleave the data beginning with the primary channel, as it will alternately load to the primary and secondary transmit data buffers once it is enabled. The complementary operation is true in receive mode ( SPORT\_CTL\_A.SPTRAN = 0) when both datapaths are enabled, as the DMA channel alternately reads from the primary and secondary receive data buffers and interleaves them in the destination DMA buffer. As such, software must de-interleave the data corresponding to the primary and secondary channels from the receive DMA buffer.

If the SPORT is configured in stereo mode, the same DMA channel handles both the left and right channels of both datapaths (primary and/or secondary). Therefore, for a transmit DMA with only one datapath enabled, the source buffer must be populated such that the left- and right-channel data is interleaved. If both datapaths are enabled, the DMA channel alternately loads to the primary and secondary transmit data buffers once it is enabled. As such, the interleaving requirement is for the primary left-channel data to be followed by the secondary left-channel data, then the primary right-channel data, and finally the secondary right-channel data. The complementary operation is true

in receive mode, where the DMA channel alternately reads from the primary and secondary receive data buffers and interleave them in the destination DMA buffer. For the stereo modes of operation, the destination DMA buffer is interleaved as left-right data for a single data input. If both datapaths are enabled, the destination DMA buffer is written with the primary and secondary left-channel data followed by the primary and secondary right-channel data. As such, software must de-interleave the primary and secondary left- and right-channel data from the receive DMA buffer, as defined by this scheme.

Since both the primary and secondary datapaths share the single DMA channel, each half SPORT has a single interrupt request for data completion, as well as an error interrupt request. The DMA controller can generate an interrupt request at the end of a chain of DMA work units (when using multiple descriptors) or at the end of individual DMA work unit.

The SPORT DMA channels are assigned a higher priority than all the other DMA channels (for example, the SPI port). Having higher priority causes the SPORT DMA transfers to execute first when multiple DMA requests occur in the same cycle. The SPORT DMA channels are numbered and prioritized in the DMA channel list table in the DMA chapter.

Although the most efficient DMA transfers execute with 32-bit words, the SPORTs can handle word sizes from 4 to 32 bits (as defined by SPORT\_CTL\_A.SLEN field). If the serial data length is 16 bits or smaller, two pieces of data can be packed into 32-bit words for each DMA transfer, as selected by setting the SPORT\_CTL\_A.PACK bit. When this bit is set, the SPORT generates the transmit and receive interrupts for the 32-bit packed words, not for each 16-bit word. For more information, see the Data Buffer Status section.

NOTE: The SPORT DMA channel can access both internal memory and external memory of the processor without any core overhead.

## Data Transfer Interrupt

Each half SPORT features a data transfer interrupt request that is shared by both the primary and secondary data channels in both transmit and receive modes. To determine the source of the data transfer interrupt request, applications can check the primary and secondary data buffer status bits ( SPORT\_CTL\_A.DXSPRI and , SPORT\_CTL\_A.DXSSEC , respectively).

When using core-driven transfers, this interrupt's meaning depends on the direction of the SPORT:

- As transmitter ( SPORT\_CTL\_A.SPTRAN = 1) - the transmit data buffer is empty
- As receiver ( SPORT\_CTL\_A.SPTRAN = 0) - new data is available in the receive data buffer

NOTE: When data packing is enabled ( SPORT\_CTL\_A.PACK = 1), the core-driven transmit and receive interrupt requests are generated for 32-bit packed words, not for each 16-bit word.

In both cases, the interrupt request can be used to signal the core that an individual transfer has completed. For transmit operations, it indicates that the transmit data buffer can be safely loaded (either the buffer is already empty, or the last data has moved from the data buffer to the shift register). For receive operations, it indicates that new data has arrived and can be read (or must be read before a subsequent word overwrites it).

When the SPORT is configured to use DMA to move data between memory and the peripheral (the most generic way to use dedicated DMA for sport data transfers), the same data transfer interrupt request instead indicates the completion of the transfer of a block of serial data (rather than a single word). When DMA is used, the DMA count register must be initialized to specify the number of words to transfer. This count decrements after each DMA transfer on the channel, and the data transfer interrupt request signal is asserted when the word count reaches zero (for example, a DMA work unit has finished).

For transmit DMA, the interrupt request is raised when the last word in the DMA work unit is loaded from the source memory to the HSPORT FIFO. This interrupt request can signal to the core that a new DMA work unit can be configured or that other software threads can now run. The transmit interrupt request can optionally be deferred until the last word of the work unit has fully shifted out of the shift register (see the T ransfer Finish Interrupt (TFI) section for details).

For receive DMA, the interrupt request is raised when the last word is loaded to the destination memory. In addition to that described for transmit DMA, this interrupt request also serves as an indication to the core that there is a buffer of newly acquired data that is ready to be processed.

See the DMA chapter for further details regarding enabling of the DMA interrupt requests associated with the various modes of DMA operation.

NOTE: As a single DMA channel services both the primary and secondary datapaths associated with the SPORT, there is a single DMA completion interrupt request.

## Transfer Finish Interrupt (TFI)

When configured for transmit DMA ( SPORT\_CTL\_A.SPTRAN =1), the data transfer interrupt request gets generated by the DMA engine itself when it decrements its count register upon loading the last element from memory to the HSPORT hardware. Alternately, the SPORT can use a Transmit Finish Interrupt (TFI) to signal the actual end of the transmission (for example, when the last bit of the last data word of the buffer has shifted out of the SPORT to the system) by setting the SPORT\_CTL\_A.TFIEN bit. When this bit is set, then DMA signal that would normally assert the data transfer interrupt request instead signals the SPORT that the DMA work unit is complete. The SPORT then waits until all the data in the FIFO is shifted out (including the transmit shift register) and asserts the TFI interrupt request upon completion.

NOTE: To enable this functionality in the DMA engine, be sure to configure the interrupt type field in the DMA configuration register for Peripheral interrupt. See the DMA chapter for further details.

## Error Detection (Status) Interrupt

In addition to the dedicated data transfer interrupt request, each half SPORT also features an optional error status interrupt request that can be triggered when error conditions occur relative to data or frame syncs associated with the half SPORT.

Data-related errors depend on the direction of the SPORT and reflect overflow or underflow conditions, which are depicted in the SPORT\_CTL\_A control register as read-only sticky bits SPORT\_CTL\_A.DERRPRI and SPORT\_CTL\_A.DERRSEC (for the primary and secondary channels, respectively).

- When the SPORT is configured as a transmitter, these bits provide transmit data buffer underflow status. When the frame sync signal occurs when the transmit data buffer is empty, the underflow bit corresponding with the offending transmit data buffer is set, as the SPORT will transmit data whenever it detects a valid frame sync signal, whether new data is present or not.
- When the SPORT is configured as a receiver, these bits provide receive overflow status. When a channel receives new data while the receive buffer is already full, the new data overwrites the existing data, thus causing an overflow. When this occurs, the overflow bit corresponding with the offending receive data buffer is set, as the SPORT receives data whenever it detects a valid frame sync signal, whether there is room in the receive buffer or not.

Each half SPORT also features an error register ( SPORT\_ERR\_A ), which is the source for the assertion of the described data-related error status bits. When a data-related error occurs on the primary or secondary datapaths, the error is logged in the SPORT\_ERR\_A.DERRPSTAT or SPORT\_ERR\_A.DERRSSTAT bits, respectively. T o enable these status bits to generate the HSPORT status interrupt request in the SEC, the corresponding SPORT\_ERR\_A.DERRPMSK and SPORT\_ERR\_A.DERRSMSK bits must be set (for the primary and secondary datapaths, respectively).

The SPORT\_CTL\_A.DERRPRI and SPORT\_CTL\_A.DERRSEC channel error status bits are sticky read-only bits that can be cleared in two ways:

- Reset the error detection logic by disabling the channel associated with the error condition (clear the SPORT\_CTL\_A.SPENPRI or SPORT\_CTL\_A.SPENSEC control bit).
- Clear the source of the interrupt by writing-1-to-clear the SPORT\_ERR\_A.FSERRSTAT , SPORT\_ERR\_A.DERRPSTAT , or SPORT\_ERR\_A.DERRSSTAT status bits.

In addition to data-related errors, SPORT\_ERR\_A also tracks frame sync errors in the SPORT\_ERR\_A.FSERRSTAT status bit. Similar to the data-related errors, the frame sync error can be enabled as a source for raising the error status interrupt request via the SEC by setting the SPORT\_ERR\_A.FSERRMSK bit. A frame sync error occurs when the frame sync is detected prematurely, as explained in the Premature Frame Sync Error Detection section.

A frame sync error is not detected in the following cases:

- When there is no active transmit or receive data, and the frame sync pulse occurs due to noise on the input signal-when there is no active transfer, a noise-induced frame sync pulse is valid.
- When there is an active underflow or overflow error-frame sync errors cannot be detected because the SPORT error logic does not run after one of the data errors has occurred and remains un-serviced.
- When the frame sync pulse does not the meet minimum timing requirements-when the frame sync pulse is shorter than a SPORT clock period, there is no guarantee that it gets sampled and may go unnoticed.

## Grouping of SPORTs

For some applications, enabling multiple SPORTs need to be synchronized so that all of them start and/or end at the same time. It may also be required to generate/receive a single interrupt/trigger for the DMA transfer of all the SPORTs in the same group.

The DAI\_GBL\_SP\_EN register is used to control enabling/disabling. The DAI\_GBL\_INT\_EN register can be used to control interrupt/trigger generation/reception by multiple SPORTs in a group.

NOTE: All references to SPORT 0-3 in the context of DAI1 must be read as SPORT4-7.

## SPORT Enable Grouping

The figure shows the block diagram of possible group enabling options for the SPORTs in DAI0 and DA1.

Figure 32-13: Group Enabling Options in DAI0 and DA1

<!-- image -->

It shows one of the channels in each half sport, same is applicable for other channel as well. For example, GBL\_SP0A\_EN means GBL\_SP0A\_PC\_EN or GBL\_SP0A\_SC\_EN .

Each DAI has a DAI\_GBL\_SP\_EN register. T o enable the SPORTs grouped within the same DAI at the same time, write to the corresponding DAI\_GBL\_SP\_EN register is required. When a group of SPORTs in different DAIs must be enabled simultaneously, writing to the DAI\_GBL\_SP\_EN.GBL\_SP\_EN field is required. This bit is reserved for the DAI1.

The following group enable options are possible:

- Up to 16 channels in 8 half SPORTs in DAI0
- Up to 16 channels in 8 half SPORTs in DAI1
- Up to 8 channels in 4 half SPORTs in each DAI
- Up to 32 channels in 16 half SPORTs (8 half SPORTs in DAI0 and 8 half SPORTs in DAI1)

- NOTE: While grouping SPORTs across DAIs, two interrupts (one per DAI) are generated and managed by software. Grouping can be done only once as the DAI\_GBL\_SP\_EN.GBL\_SP\_EN field is used.
- NOTE: The grouping of SPORTs and its interrupts/triggers are two separate entities. When the interrupts and triggers must be grouped similar to the SPORT grouping, use the group interrupt/trigger enable register.

## SPORT Interrupt/Trigger Grouping

Each DAI has a DAI\_GBL\_INT\_EN register. Each of these registers has two group interrupt enable bits DAI\_GBL\_INT\_EN.GRP0\_INT\_EN and DAI\_GBL\_INT\_EN.GRP1\_INT\_EN , two group trigger (controller) enable bits DAI\_GBL\_INT\_EN.GRP0\_TRG\_EN and DAI\_GBL\_INT\_EN.GRP1\_TRG\_EN . There are individual GRPx\_SPyINT\_EN bits for each group to enable both interrupt/trigger for up to 8 channels (4 half SPORTs). Enable these bits as per the grouping in the DAI\_GBL\_SP\_EN register.

- NOTE: To enable or to disable the SPORT interrupt/trigger grouping feature, the individual sport interrupt/trigger enable bits must be programmed along with the respective group interrupt/trigger enable bits in the DAI\_GBL\_INT\_EN register.

Regarding grouping of SPORT interrupts/triggers:

- All the interrupts/triggers in a single group are ANDed and a single interrupt/trigger goes to SEC/TRU.
- To generate group interrupt/trigger, all the SPORTs in the corresponding group should be configured to generate interrupt ( DMA\_CFG.INT )/trigger ( DMA\_CFG.TRIG ).
- The interrupts/triggers generated by the SPORTs in a group can either be used as a single group interrupt/ trigger (enabled by the DAI\_GBL\_INT\_EN register) or the individual SPORT DMA done interrupts/triggers.
- As there are no separate status registers for global interrupts, to clear the global interrupts, clear (W1C) the individual SPORT DMA\_STAT.IRQDONE bits. Theoretically, the global interrupt gets cleared by clearing the DMA\_STAT.IRQDONE bit of at least one SPORT in the group. However, it is recommended to clear the DMA\_STAT.IRQDONE bits of all the SPORTs in the group to achieve a clean state for the next global interrupt generation.
- The DAI\_GBL\_INT\_EN.GRP1\_TRG\_EN / DAI\_GBL\_INT\_EN.GRP0\_TRG\_EN bits enables group controller trigger functionality for SPORTs in a group.

## SPORT Programming Model

The following sections provide programming guidance for setting up the SPORTs for use in an application:

- Initializing Core-Driven (Non-MCM) Transfers
- Initializing Multichannel T ransfers
- Using DMA for SPORT Transfers
- Using Companding as a Function

## Initializing Core-Driven (Non-MCM) Transfers

The following programming model applies to all of Standard DSP Serial Mode, I 2 S Mode, Left-Justified Mode, and Right-Justified Mode for core-driven transfers. More steps are required to properly initialize the SEC to service the SPORT interrupts (see the SEC chapter for details).

NOTE: This example uses half SPORT A registers. With appropriate changes to register names, this example also applies to half SPORT B.

1. Clear the SPORT\_CTL\_A and SPORT\_MCTL\_A configuration registers.
2. ADDITIONAL INFORMATION: Clearing these registers ensures that the SPORT logic (including the multichannel logic) is fully reset before attempting to reprogram it.
2. Optionally program the SPORT\_DIV\_A clock divisor register.
4. ADDITIONAL INFORMATION: This step is only required for internally-generated timing signals. Configure the serial bit clock and/or frame sync (or L/R clock, for stereo modes) rates according to the guidance in the Serial Clock and Frame Sync sections.
3. Program the SPORT\_CTL\_A primary configuration register.
6. ADDITIONAL INFORMATION: Set the SPORT operating mode along with the configurable clock, frame sync, word length, direction, and data format options (see the Operating Modes and Options section for details). Do not set the SPORT\_CTL\_A.SPENPRI and/or SPORT\_CTL\_A.SPENSEC buffer enable bits in this step.
4. Optionally program the SPORT\_CTL2\_A secondary configuration register. ADDITIONAL INFORMATION: This step is required only if internal multiplexing logic must be enabled to share clock and frame sync signals between a top SPORT module's A and B halves (see the Multiplexer Logic

section for details).

5. Optionally program the SPORT\_ERR\_A error register. ADDITIONAL INFORMATION: This step is required only if a separate SPORT error interrupt is desired (see

the Error Detection (Status) Interrupt section for details).

6. For Right-Justified mode only, program the SPORT\_MCTL\_A.WOFFSET field.
2. ADDITIONAL INFORMATION: In Right-Justified mode, this field serves as the delay count ( DCNT ) required to align the LSB of each stereo channel with the L/R clock transition and must be programmed manually (see the Right-Justified Mode section for details).
7. Enable the primary/secondary datapath(s) in the SPORT\_CTL\_A register.
4. ADDITIONAL INFORMATION: This should be performed in a read-modify-write operation setting the SPORT\_CTL\_A.SPENPRI and/or SPORT\_CTL\_A.SPENSEC bits, as appropriate.

8. Write data to be transmitted to the transmit buffer ( SPORT\_TXPRI\_A and/or SPORT\_TXSEC\_A ) or read data that has been received from the receive buffer ( SPORT\_RXPRI\_A and/or SPORT\_RXSEC\_A ).

ADDITIONAL INFORMATION: These accesses are typically performed in the context of an interrupt service routine. See the SEC chapter for further information. Do not attempt to read or write inactive data buffers. If the core attempts to access inactive transmit or receive buffers while the SPORT is enabled, unpredictable results may occur.

## Initializing Multichannel Transfers

When in Multichannel (TDM) Mode or Packed I 2 S Mode, the SPORT is in a multichannel operational mode. Follow the steps below to properly initialize the SPORT for multichannel modes of operation. More steps are required to properly (see the SEC chapter for details).

- NOTE: This example uses half SPORT A registers. With appropriate register changes, this example also applies to half SPORT B.
1. Clear the SPORT\_CTL\_A and SPORT\_MCTL\_A registers.
- ADDITIONAL INFORMATION: Clearing these registers ensures that the SPORT logic (including the multi-

channel logic) is fully reset before attempting to reprogram it.

2. Optionally program the SPORT\_DIV\_A clock divisor register.
2. ADDITIONAL INFORMATION: This step is only required for internally-generated timing signals. Configure the serial bit clock and/or frame sync (or L/R clock, for stereo modes) rates according to the guidance in the Serial Clock and SPORT\_CTL2\_A sections.
3. Program the SPORT\_CS0\_A -SPORT\_CS3\_A channel select registers.
4. Program the SPORT\_MCTL\_A multichannel configuration register.
5. ADDITIONAL INFORMATION: The SPORT supports many multichannel options. For more information, see the Multichannel (TDM) Mode section. Do not set the SPORT\_MCTL\_A.MCE enable bit in this step.
5. Program the SPORT\_CTL\_A primary configuration register.
7. ADDITIONAL INFORMATION: Set the SPORT operating mode along with the configurable clock, frame sync, word length, direction, and data format options (see the Operating Modes and Options section for details). Do not set the SPORT\_CTL\_A.SPENPRI and/or SPORT\_CTL\_A.SPENSEC buffer enable bits in this step.
6. Optionally program the SPORT\_CTL2\_A secondary configuration register. ADDITIONAL INFORMATION: This step is required only if internal multiplexing logic must be enabled to share clock and frame sync signals between a top SPORT module's A and B halves (see the Multiplexer Logic section for details).
7. Optionally program the SPORT\_ERR\_A error register.

ADDITIONAL INFORMATION: This step is required only if a separate SPORT error interrupt request is desired (see the Error Detection (Status) Interrupt section for details).

8. Set the SPORT\_MCTL\_A.MCE bit to enable multichannel mode.
9. Enable the primary/secondary datapath(s) in the SPORT\_CTL\_A register.
3. ADDITIONAL INFORMATION: This should be performed in a read-modify-write operation setting the SPORT\_CTL\_A.SPENPRI and/or SPORT\_CTL\_A.SPENSEC bits, as appropriate. DMA mode is recommended for multichannel modes of operation. For more information, see the Using DMA for SPORT Transfers programming model.

## Using DMA for SPORT Transfers

DMA is supported in all SPORT operating modes (Standard DSP Serial Mode, I 2 S Mode, Left-Justified Mode, Right-Justified Mode, Multichannel (TDM) Mode or Packed I 2 S Mode). To enable DMA operation with the SPORT, execute the steps described in this section after initializing and enabling the SPORT. Instead of using the single word read or write operations described in the referenced programming models, the DMA engine automates accesses to the enabled SPORT data buffers.

NOTE: This example uses half SPORT A registers. With appropriate changes to register names, it also applies to half SPORT B.

1. Follow the guidance in the multichannel (Initializing Multichannel T ransfers) or non-multichannel (Initializing Core-Driven (Non-MCM) Transfers) programming models to properly initialize and enable the SPORT hardware.
2. Prepare the data buffers in memory.
3. ADDITIONAL INFORMATION: Ensure that the DMA buffer is defined according to the DMA Transfers section. For the multichannel modes of operation, be sure to also consider the setting of the SPORT\_MCTL\_A.MCPDE bit, as described in the Multichannel DMA Data Packing (MCPDE) section.
3. Initialize and enable the DMA channel allocated for the SPORT, as described in the Direct Memory Access (DMA) chapter.

## Using Companding as a Function

The data in the transmit and receive buffers are companded in place. The following programming model can be used to exercise the companding hardware without transferring data, which is useful for test and debug purposes.

- NOTE: This example uses half SPORT A registers. With appropriate changes to register names, this example also applies to half SPORT B.
1. Configure the SPORT as a transmitter ( SPORT\_CTL\_A.SPTRAN =1) with both the primary and secondary data channels disabled ( SPORT\_CTL\_A.SPENPRI =0 and SPORT\_CTL\_A.SPENSEC =0).

2. Enable the desired companding scheme in the SPORT\_CTL\_A.DTYPE field.
3. Write a 32-bit word to one of the transmit buffers.
4. Wait two system clock cycles.

ADDITIONAL INFORMATION: This delay is required to allow the SPORT companding hardware to reload the transmit buffer with the companded result. Any instructions that do not access the transmit buffer can be used to cause this delay.

5. Read the 8-bit compressed value from the transmit buffer written above.

To expand data in place, use the same sequence of operations with the receive buffer instead of the transmit buffer. When expanding data in this way, set the appropriate serial word length ( SPORT\_CTL\_A.SLEN ).

## Programming Global SPORT Groups

Complete the following steps to program global SPORT groups.

NOTE: To complete all of the SPORT group transfers simultaneously, all of the SPORTs in the group must have a similar data/clock/frame sync configuration.

1. For each SPORT, program the following registers for the desired SPORT mode of operation.
- SPORT\_DIV\_A / SPORT\_DIV\_B
- SPORT\_CTL2\_A / SPORT\_CTL2\_B
- SPORT\_CTL\_A / SPORT\_CTL\_B
- SPORT\_MCTL\_A / SPORT\_MCTL\_B
- SPORT\_CS0\_A / SPORT\_CS0\_B
- SPORT\_CS1\_ASPORT\_CS1\_B
- SPORT\_CS2\_ASPORT\_CS2\_B
- SPORT\_CS3\_ASPORT\_CS3\_B

ADDITIONAL INFORMATION: The sport enable control bits (in SPORT\_CTL2\_A / SPORT\_CTL2\_B registers) must be cleared (disabled).

2. For a global group trigger as the controller, program each SPORT DMA in the group as the controller trigger.
3. Program the sport enable and group enable bits in the DAI\_GBL\_SP\_EN register for the desired global sport grouping.

ADDITIONAL INFORMATION: The grouping of SPORTs (enable and disable) and the associated interrupts/ triggers are separate activities. If the interrupts and triggers must be grouped similar to the SPORTs (enable and disable) grouping, use the DAI\_GBL\_INT\_EN register.

4. Program the DAI\_GBL\_INT\_EN register to enable the grouped interrupt or trigger generation for the SPORT group.

ADDITIONAL INFORMATION: To enable the SPORT grouped interrupt and trigger feature, each sport interrupt or trigger enable bit must be programmed with the respective group interrupt and trigger enable bits in the DAI\_GBL\_INT\_EN register.

5. Program the DAI\_GBL\_SP\_EN.GBL\_SP\_EN bit to enable the global sport groups at the same instance.

## Disabling Global SPORT Groups

Complete the following steps to disable global SPORT groups.

1. Use the individual SPORT DMA status register to clear the group interrupt. There is no specific status register for global interrupts.
2. Disable the global SPORT group interrupt and trigger bits for the group in the DAI\_GBL\_INT\_EN register. ADDITIONAL INFORMATION: To disable the SPORT grouped interrupt and trigger feature, each sport interrupt and trigger enable bit must be programmed with the respective group interrupt and trigger enable bits in the DAI\_GBL\_INT\_EN register.
3. Disable the global individual SPORT enable and the global SPORT group enable bits in the DAI\_GBL\_SP\_EN register.
4. Disable the DMAs for the SPORTs when the transfer is complete.

## ADSP-SC59x SPORT Register Descriptions

Serial Port (SPORT) contains the following registers.

Table 32-14: ADSP-SC59x SPORT Register List

| Name         | Description                                        |
|--------------|----------------------------------------------------|
| SPORT_CS0_A  | Half SPORT 'A' Multichannel 0-31 Select Register   |
| SPORT_CS0_B  | Half SPORT 'B' Multichannel 0-31 Select Register   |
| SPORT_CS1_A  | Half SPORT 'A' Multichannel 32-63 Select Register  |
| SPORT_CS1_B  | Half SPORT 'B' Multichannel 32-63 Select Register  |
| SPORT_CS2_A  | Half SPORT 'A' Multichannel 64-95 Select Register  |
| SPORT_CS2_B  | Half SPORT 'B' Multichannel 64-95 Select Register  |
| SPORT_CS3_A  | Half SPORT 'A' Multichannel 96-127 Select Register |
| SPORT_CS3_B  | Half SPORT 'B' Multichannel 96-127 Select Register |
| SPORT_CTL2_A | Half SPORT 'A' Control 2 Register                  |
| SPORT_CTL2_B | Half SPORT 'B' Control 2 Register                  |

Table 32-14: ADSP-SC59x SPORT Register List (Continued)

| Name          | Description                                   |
|---------------|-----------------------------------------------|
| SPORT_CTL_A   | Half SPORT 'A' Control Register               |
| SPORT_CTL_B   | Half SPORT 'B' Control Register               |
| SPORT_DIV_A   | Half SPORT 'A' Divisor Register               |
| SPORT_DIV_B   | Half SPORT 'B' Divisor Register               |
| SPORT_ERR_A   | Half SPORT 'A' Error Register                 |
| SPORT_ERR_B   | Half SPORT 'B' Error Register                 |
| SPORT_MCTL_A  | Half SPORT 'A' Multichannel Control Register  |
| SPORT_MCTL_B  | Half SPORT 'B' Multichannel Control Register  |
| SPORT_MSTAT_A | Half SPORT 'A' Multichannel Status Register   |
| SPORT_MSTAT_B | Half SPORT 'B' Multichannel Status Register   |
| SPORT_RXPRI_A | Half SPORT 'A' Rx Buffer (Primary) Register   |
| SPORT_RXPRI_B | Half SPORT 'B' Rx Buffer (Primary) Register   |
| SPORT_RXSEC_A | Half SPORT 'A' Rx Buffer (Secondary) Register |
| SPORT_RXSEC_B | Half SPORT 'B' Rx Buffer (Secondary) Register |
| SPORT_TXPRI_A | Half SPORT 'A' Tx Buffer (Primary) Register   |
| SPORT_TXPRI_B | Half SPORT 'B' Tx Buffer (Primary) Register   |
| SPORT_TXSEC_A | Half SPORT 'A' Tx Buffer (Secondary) Register |
| SPORT_TXSEC_B | Half SPORT 'B' Tx Buffer (Secondary) Register |

## Half SPORT 'A' Multichannel 0-31 Select Register

Each of the bits (when set, =1) of the SPORT\_CS0\_A register correspond to an active channel for the half SPORT in multichannel mode. When the register activates a channel (corresponding bit =1), the half SPORT transmits or receives the word in that channel's position of the data stream. When the register deactivates a channel (corresponding bit =0), the half SPORT either three-states its data transmit pin (during the channel's transmit time slot) or ignores incoming data (during the channel's receive time slot).

Figure 32-14: SPORT\_CS0\_A Register Diagram

<!-- image -->

Table 32-15: SPORT\_CS0\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Channel Enable 0-31.      |

## Half SPORT 'B' Multichannel 0-31 Select Register

Each of the bits (when set, =1) of the SPORT\_CS0\_B register correspond to an active channel for the half SPORT in multichannel mode. When the register activates a channel (corresponding bit =1), the half SPORT transmits or receives the word in that channel's position of the data stream. When the register deactivates a channel (corresponding bit =0), the half SPORT either three-states its data transmit pin (during the channel's transmit time slot) or ignores incoming data (during the channel's receive time slot).

Figure 32-15: SPORT\_CS0\_B Register Diagram

<!-- image -->

Table 32-16: SPORT\_CS0\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Channel Enable 0-31.      |

## Half SPORT 'A' Multichannel 32-63 Select Register

Each of the bits (when set, =1) of the SPORT\_CS1\_A register correspond to an active channel for the half SPORT in multichannel mode. When the register activates a channel (corresponding bit =1), the half SPORT transmits or receives the word in that channel's position of the data stream. When the register deactivates a channel (corresponding bit =0), the half SPORT either three-states its data transmit pin (during the channel's transmit time slot) or ignores incoming data (during the channel's receive time slot).

Figure 32-16: SPORT\_CS1\_A Register Diagram

<!-- image -->

Table 32-17: SPORT\_CS1\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Channel Enable 32-63.     |

## Half SPORT 'B' Multichannel 32-63 Select Register

Each of the bits (when set, =1) of the SPORT\_CS1\_B register correspond to an active channel for the half SPORT in multichannel mode. When the register activates a channel (corresponding bit =1), the half SPORT transmits or receives the word in that channel's position of the data stream. When the register deactivates a channel (corresponding bit =0), the half SPORT either three-states its data transmit pin (during the channel's transmit time slot) or ignores incoming data (during the channel's receive time slot).

Figure 32-17: SPORT\_CS1\_B Register Diagram

<!-- image -->

Table 32-18: SPORT\_CS1\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Channel Enable 32-63.     |

## Half SPORT 'A' Multichannel 64-95 Select Register

Each of the bits (when set, =1) of the SPORT\_CS2\_A register correspond to an active channel for the half SPORT in multichannel mode. When the register activates a channel (corresponding bit =1), the half SPORT transmits or receives the word in that channel's position of the data stream. When the register deactivates a channel (corresponding bit =0), the half SPORT either three-states its data transmit pin (during the channel's transmit time slot) or ignores incoming data (during the channel's receive time slot).

Figure 32-18: SPORT\_CS2\_A Register Diagram

<!-- image -->

Table 32-19: SPORT\_CS2\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Channel Enable 64-95.     |

## Half SPORT 'B' Multichannel 64-95 Select Register

Each of the bits (when set, =1) of the SPORT\_CS2\_B register correspond to an active channel for the half SPORT in multichannel mode. When the register activates a channel (corresponding bit =1), the half SPORT transmits or receives the word in that channel's position of the data stream. When the register deactivates a channel (corresponding bit =0), the half SPORT either three-states its data transmit pin (during the channel's transmit time slot) or ignores incoming data (during the channel's receive time slot).

Figure 32-19: SPORT\_CS2\_B Register Diagram

<!-- image -->

Table 32-20: SPORT\_CS2\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Channel Enable 64-95.     |

## Half SPORT 'A' Multichannel 96-127 Select Register

Each of the bits (when set, =1) of the SPORT\_CS3\_A register correspond to an active channel for the half SPORT in multichannel mode. When the register activates a channel (corresponding bit =1), the half SPORT transmits or receives the word in that channel's position of the data stream. When the register deactivates a channel (corresponding bit =0), the half SPORT either three-states its data transmit pin (during the channel's transmit time slot) or ignores incoming data (during the channel's receive time slot).

Figure 32-20: SPORT\_CS3\_A Register Diagram

<!-- image -->

Table 32-21: SPORT\_CS3\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Channel Enable 96-127.    |

## Half SPORT 'B' Multichannel 96-127 Select Register

Each of the bits (when set, =1) of the SPORT\_CS3\_B register correspond to an active channel for the half SPORT in multichannel mode. When the register activates a channel (corresponding bit =1), the half SPORT transmits or receives the word in that channel's position of the data stream. When the register deactivates a channel (corresponding bit =0), the half SPORT either three-states its data transmit pin (during the channel's transmit time slot) or ignores incoming data (during the channel's receive time slot).

Figure 32-21: SPORT\_CS3\_B Register Diagram

<!-- image -->

Table 32-22: SPORT\_CS3\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Channel Enable 96-127.    |

## Half SPORT 'A' Control 2 Register

The SPORT\_CTL2\_A register controls multiplexing options for sharing serial clock and frame sync signals across the related half SPORTs.

Figure 32-22: SPORT\_CTL2\_A Register Diagram

<!-- image -->

Table 32-23: SPORT\_CTL2\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | RLRE       | Receiver Late Sampling Enable. The SPORT_CTL2_A.RLRE bit enables the half SPORT's late sampling of received data and frame sync by half of the operating clock cycle. Note: This mode of operation is applicable when Clock/FS are internally generated, and data is externally generated (i.e data and control are in opposite direction).                                      | Receiver Late Sampling Enable. The SPORT_CTL2_A.RLRE bit enables the half SPORT's late sampling of received data and frame sync by half of the operating clock cycle. Note: This mode of operation is applicable when Clock/FS are internally generated, and data is externally generated (i.e data and control are in opposite direction).                                      |
| 1 (R/W)            | CKMUXSEL   | Clock Multiplexer Select. The SPORT_CTL2_A.CKMUXSEL bit enables multiplexing of the half SPORT's se- rial clock. In this mode, the serial clock of the related half SPORT is used instead of the half SPORT's own serial clock. For example, if the SPORT_CTL2_A.CKMUXSEL bit is enabled, half SPORT 'A' uses SPORT_BCLK instead of SPORT_ACLK .                                 | Clock Multiplexer Select. The SPORT_CTL2_A.CKMUXSEL bit enables multiplexing of the half SPORT's se- rial clock. In this mode, the serial clock of the related half SPORT is used instead of the half SPORT's own serial clock. For example, if the SPORT_CTL2_A.CKMUXSEL bit is enabled, half SPORT 'A' uses SPORT_BCLK instead of SPORT_ACLK .                                 |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                | Disable serial clock multiplexing                                                                                                                                                                                                                                                                                                                                                |
| 0 (R/W)            | FSMUXSEL   | 1 Enable serial clock multiplexing Frame Sync Multiplexer Select. The SPORT_CTL2_A.FSMUXSEL bit enables multiplexing of the half SPORT's frame sync. In this mode, the frame sync of the related half SPORT is used instead of the half SPORT's own frame sync. For example, if the SPORT_CTL2_A.FSMUXSEL bit is enabled, half SPORT 'A' uses SPORT_BFS in- stead of SPORT_AFS . | 1 Enable serial clock multiplexing Frame Sync Multiplexer Select. The SPORT_CTL2_A.FSMUXSEL bit enables multiplexing of the half SPORT's frame sync. In this mode, the frame sync of the related half SPORT is used instead of the half SPORT's own frame sync. For example, if the SPORT_CTL2_A.FSMUXSEL bit is enabled, half SPORT 'A' uses SPORT_BFS in- stead of SPORT_AFS . |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                | Disable frame sync multiplexing                                                                                                                                                                                                                                                                                                                                                  |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                | Enable frame sync multiplexing                                                                                                                                                                                                                                                                                                                                                   |

## Half SPORT 'B' Control 2 Register

The SPORT\_CTL2\_B register controls multiplexing options for sharing serial clock and frame sync signals across the related half SPORTs.

Figure 32-23: SPORT\_CTL2\_B Register Diagram

<!-- image -->

Table 32-24: SPORT\_CTL2\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | RLRE       | Receiver Late Reception Enable. The SPORT_CTL2_B.RLRE bit enables the half SPORT's late sampling of received data and frame sync by half of the operating clock cycle.                                                                                                                                                                           | Receiver Late Reception Enable. The SPORT_CTL2_B.RLRE bit enables the half SPORT's late sampling of received data and frame sync by half of the operating clock cycle.                                                                                                                                                                           |
| 1 (R/W)            | CKMUXSEL   | Clock Multiplexer Select. The SPORT_CTL2_B.CKMUXSEL bit enables multiplexing of the half SPORT's se- rial clock. In this mode, the serial clock of the related half SPORT is used instead of the half SPORT's own serial clock. For example, if the SPORT_CTL2_B.CKMUXSEL bit is enabled, half SPORT 'B' uses SPORT_ACLK instead of SPORT_BCLK . | Clock Multiplexer Select. The SPORT_CTL2_B.CKMUXSEL bit enables multiplexing of the half SPORT's se- rial clock. In this mode, the serial clock of the related half SPORT is used instead of the half SPORT's own serial clock. For example, if the SPORT_CTL2_B.CKMUXSEL bit is enabled, half SPORT 'B' uses SPORT_ACLK instead of SPORT_BCLK . |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                | Disable serial clock multiplexing                                                                                                                                                                                                                                                                                                                |
| 0 (R/W)            | FSMUXSEL   | Frame Sync Multiplexer Select. The SPORT_CTL2_B.FSMUXSEL bit enables multiplexing of the half SPORT's frame sync. In this mode, the frame sync of the related half SPORT is used instead of the half SPORT's own frame sync. For example, if the SPORT_CTL2_B.FSMUXSEL bit is enabled, half SPORT 'B' uses SPORT_AFS in- stead of SPORT_BFS .    | Frame Sync Multiplexer Select. The SPORT_CTL2_B.FSMUXSEL bit enables multiplexing of the half SPORT's frame sync. In this mode, the frame sync of the related half SPORT is used instead of the half SPORT's own frame sync. For example, if the SPORT_CTL2_B.FSMUXSEL bit is enabled, half SPORT 'B' uses SPORT_AFS in- stead of SPORT_BFS .    |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                | Disable frame sync multiplexing                                                                                                                                                                                                                                                                                                                  |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                | Enable frame sync multiplexing                                                                                                                                                                                                                                                                                                                   |

## Half SPORT 'A' Control Register

The SPORT\_CTL\_A register contains transmit and receive control bits for SPORT half 'A', including serial port mode selection for the half SPORT's primary and secondary channels. The function of some bits in the SPORT\_CTL\_A register vary depending on the SPORT's operating mode. For more information, see the SPORT operating modes description. If reading reserved bits, the read value is the last written value to these bits or is the reset value of these bits.

Figure 32-24: SPORT\_CTL\_A Register Diagram

<!-- image -->

Table 32-25: SPORT\_CTL\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:30 (R/NW)       | DXSPRI     | Data Transfer Buffer Status (Primary). The SPORT_CTL_A.DXSPRI bit field indicates the status of the half SPORT's pri- mary channel data buffer. 0 Empty 1 Reserved 2 Partially full Full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 31:30 (R/NW)       | DXSPRI     | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 29 (R/NW)          | DERRPRI    | Data Error Status (Primary). The SPORT_CTL_A.DERRPRI bit reports the half SPORT's primary channel trans- mit underflow status or receive overflow status, depending on the SPORT transfer di- rection. If the SPORT_CTL_A.FSR bit =1, the SPORT_CTL_A.DERRPRI bit indicates whether the SPORT_AFS signal (from an internal or external source) occurred while the SPORT_TXPRI_A data buffer was empty (during transmit) or the SPORT_RXPRI_A data buffer was full (during receive). The SPORT transmits or re- ceives data whenever it detects the SPORT_AFS signal. It is important to note that, as a receiver, the SPORT_CTL_A.DERRPRI bit indicates when the channel has re- ceived new data while the SPORT_RXPRI_A receive buffer is full. This new data overwrites existing data. If the SPORT_CTL_A.FSR bit =0, the SPORT_CTL_A.DERRPRI bit is set when- ever the SPORT is required to transmit while the SPORT_TXPRI_A transmit buffer is empty. It is also set whenever the SPORT is required to receive while the SPORT_RXPRI_A receive buffer is full. The SPORT clears the SPORT_CTL_A.DERRPRI bit if the SPORT_ERR_A.DERRPSTAT bit is cleared. 0 No error 1 Error (Tx underflow or Rx overflow) |
| 28:27 (R/NW)       | DXSSEC     | Data Transfer Buffer Status (Secondary). The SPORT_CTL_A.DXSSEC bit field indicates the status of the half SPORT's sec- ondary channel data buffer.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 28:27 (R/NW)       | DXSSEC     | 0 Empty                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 28:27 (R/NW)       | DXSSEC     | 1 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 28:27 (R/NW)       | DXSSEC     | 2 Partially full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 28:27 (R/NW)       | DXSSEC     | 3 Full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 32-25: SPORT\_CTL\_A Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/NW)          | DERRSEC    | Data Error Status (Secondary). The SPORT_CTL_A.DERRSEC bit reports the half SPORT's secondary channel transmit underflow status or receive overflow status, depending on the SPORT transfer direction. If the SPORT_CTL_A.FSR bit =1, the SPORT_CTL_A.DERRSEC bit indicates whether the SPORT_AFS signal (from an internal or external source) occurred while the SPORT_TXSEC_A data buffer was empty (during transmit) or the SPORT_RXSEC_A data buffer was full (during receive). The SPORT transmits or re- ceives data whenever it detects the SPORT_AFS signal. It is important to note that, as a receiver, the SPORT_CTL_A.DERRSEC bit indicates when the channel has re- ceived new data while the SPORT_RXSEC_A receive buffer is full. This new data overwrites existing data. If the SPORT_CTL_A.FSR bit =0, the SPORT_CTL_A.DERRSEC bit is set when- ever the SPORT is required to transmit while the SPORT_TXSEC_A transmit buffer is empty. It is also set whenever the SPORT is required to receive while the SPORT_RXSEC_A receive buffer is full. The SPORT clears the SPORT_CTL_A.DERRSEC bit if the SPORT_ERR_A.DERRSSTAT bit is cleared. | Data Error Status (Secondary). The SPORT_CTL_A.DERRSEC bit reports the half SPORT's secondary channel transmit underflow status or receive overflow status, depending on the SPORT transfer direction. If the SPORT_CTL_A.FSR bit =1, the SPORT_CTL_A.DERRSEC bit indicates whether the SPORT_AFS signal (from an internal or external source) occurred while the SPORT_TXSEC_A data buffer was empty (during transmit) or the SPORT_RXSEC_A data buffer was full (during receive). The SPORT transmits or re- ceives data whenever it detects the SPORT_AFS signal. It is important to note that, as a receiver, the SPORT_CTL_A.DERRSEC bit indicates when the channel has re- ceived new data while the SPORT_RXSEC_A receive buffer is full. This new data overwrites existing data. If the SPORT_CTL_A.FSR bit =0, the SPORT_CTL_A.DERRSEC bit is set when- ever the SPORT is required to transmit while the SPORT_TXSEC_A transmit buffer is empty. It is also set whenever the SPORT is required to receive while the SPORT_RXSEC_A receive buffer is full. The SPORT clears the SPORT_CTL_A.DERRSEC bit if the SPORT_ERR_A.DERRSSTAT bit is cleared. |
| 26 (R/NW)          | DERRSEC    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | No error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 25 (R/W)           | SPTRAN     | Serial Port Transfer Direction. The SPORT_CTL_A.SPTRAN bit selects the transfer direction (receive or transmit) for the half SPORT's primary and secondary channels. When the direction is receive, the half SPORT activates the receive buffers, and the                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Serial Port Transfer Direction. The SPORT_CTL_A.SPTRAN bit selects the transfer direction (receive or transmit) for the half SPORT's primary and secondary channels. When the direction is receive, the half SPORT activates the receive buffers, and the                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 25 (R/W)           | SPTRAN     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Receive                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 25 (R/W)           | SPTRAN     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Transmit                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 24 (R/W)           | SPENSEC    | Serial Port Enable (Secondary). The SPORT_CTL_A.SPENSEC bit enables the half SPORT's secondary channel. When this bit is cleared (changes from =1 to =0), the half SPORT automatically flush- es the channel's data buffers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Serial Port Enable (Secondary). The SPORT_CTL_A.SPENSEC bit enables the half SPORT's secondary channel. When this bit is cleared (changes from =1 to =0), the half SPORT automatically flush- es the channel's data buffers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 24 (R/W)           | SPENSEC    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 24 (R/W)           | SPENSEC    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 32-25: SPORT\_CTL\_A Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | GCLKEN     | Gated Clock Enable. The SPORT_CTL_A.GCLKEN bit enables gated clock operation for the half SPORT when in DSP serial mode or left-justified stereo modes ( SPORT_CTL_A.OPMODE = 0 or 1). This bit is ignored when the half SPORT is in right-justified mode ( SPORT_CTL_A.RJUST =1) or multichannel mode ( SPORT_MCTL_A.MCE =1). When the SPORT_CTL_A.GCLKEN bit is enabled, the SPORT clock is active when the SPORT is transferring data or when the frame sync changes (transitions to active state). 0 Disable                                                    |
| 20 (R/W)           | TFIEN      | 1 Enable Transmit Finish Interrupt Enable. The SPORT_CTL_A.TFIEN bit selects when the half SPORT issues its transmission complete interrupt request, if a DMAcomplete interrupt request is enabled by the DMA_CFG.INT configuration. When enabled ( SPORT_CTL_A.TFIEN =1), the DMAcomplete peripheral interrupt request is generated when the last bit of last word in the DMAis shifted out. When disabled ( SPORT_CTL_A.TFIEN =0), theDMA interrupt request is generated when the DMAcounter expires (the last word goes to                                       |
| 19 (R/W)           | FSED       | 1 Last bit sent (Tx buffer done) interrupt Frame Sync Edge Detect. The SPORT_CTL_A.FSED bit enables the half SPORT to start transmitting or re- ceiving after detecting an active edge of an external frame sync. The SPORT_CTL_A.FSED bit may be enabled even during an active frame sync, and the half SPORT starts the transfer on the next valid rising or falling edge of external frame sync. If disabled ( SPORT_CTL_A.FSED =0), the half SPORT operates in the stand- ard level-sensitive detection mode for external frame sync. 0 Level detect frame sync |

Table 32-25: SPORT\_CTL\_A Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | RJUST      | Right-Justified Operation Mode. The SPORT_CTL_A.RJUST bit enables the half SPORT (if SPORT_CTL_A.OPMODE =1) to transfer data in right-justified operation mode. In this mode, the half SPORT aligns data to the end of the frame sync, rather than the start of the frame sync. When using right-justified mode, systems should program an appropriate delay count to introduce a clock delay before the half SPORT state ma- chine starts to capture data. This value is set in the DCNT field (right-justified mode usage of the SPORT_MCTL_A.WOFFSET field). For information about appropriate                                                                                                | Right-Justified Operation Mode. The SPORT_CTL_A.RJUST bit enables the half SPORT (if SPORT_CTL_A.OPMODE =1) to transfer data in right-justified operation mode. In this mode, the half SPORT aligns data to the end of the frame sync, rather than the start of the frame sync. When using right-justified mode, systems should program an appropriate delay count to introduce a clock delay before the half SPORT state ma- chine starts to capture data. This value is set in the DCNT field (right-justified mode usage of the SPORT_MCTL_A.WOFFSET field). For information about appropriate                                                                                                |
| 18 (R/W)           | RJUST      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 18 (R/W)           | RJUST      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 17 (R/W)           | LAFS       | Late Frame Sync / OPMODE2. When the half SPORT is in DSP standard mode ( SPORT_CTL_A.OPMODE =0) or in right-justified mode ( SPORT_CTL_A.RJUST =1), the SPORT_CTL_A.LAFS bit selects whether the half SPORT generates a late frame sync ( SPORT_AFS during first data bit) or generates an early frame sync signal ( SPORT_AFS during serial clock cycle before first data bit). When the half SPORT is in I 2 S / left-justified mode ( SPORT_CTL_A.OPMODE =1), the SPORT_CTL_A.LAFS bit acts as OP- MODE2, selecting whether the half SPORT is in left-justified mode or I 2 S mode. When the half SPORT is in multichannel mode ( SPORT_MCTL_A.MCE =1), the SPORT_CTL_A.LAFS bit is reserved. | Late Frame Sync / OPMODE2. When the half SPORT is in DSP standard mode ( SPORT_CTL_A.OPMODE =0) or in right-justified mode ( SPORT_CTL_A.RJUST =1), the SPORT_CTL_A.LAFS bit selects whether the half SPORT generates a late frame sync ( SPORT_AFS during first data bit) or generates an early frame sync signal ( SPORT_AFS during serial clock cycle before first data bit). When the half SPORT is in I 2 S / left-justified mode ( SPORT_CTL_A.OPMODE =1), the SPORT_CTL_A.LAFS bit acts as OP- MODE2, selecting whether the half SPORT is in left-justified mode or I 2 S mode. When the half SPORT is in multichannel mode ( SPORT_MCTL_A.MCE =1), the SPORT_CTL_A.LAFS bit is reserved. |
| 17 (R/W)           | LAFS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Early frame sync (or I 2 S mode)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 17 (R/W)           | LAFS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Late frame sync (or left-justified mode)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 16 (R/W)           | LFS        | Active-Low Frame Sync / L_FIRST / PLFS. When the half SPORT is in DSP standard mode and multichannel mode ( SPORT_CTL_A.OPMODE =0), the SPORT_CTL_A.LFS bit selects whether the half SPORT uses active low or active high frame sync. When the half SPORT is in I 2 S / packed / left-justified mode ( SPORT_CTL_A.OPMODE =1), the SPORT_CTL_A.LFS bit acts as L_FIRST, selecting whether the half SPORT trans- fers data first for the left or right channel.                                                                                                                                                                                                                                   | Active-Low Frame Sync / L_FIRST / PLFS. When the half SPORT is in DSP standard mode and multichannel mode ( SPORT_CTL_A.OPMODE =0), the SPORT_CTL_A.LFS bit selects whether the half SPORT uses active low or active high frame sync. When the half SPORT is in I 2 S / packed / left-justified mode ( SPORT_CTL_A.OPMODE =1), the SPORT_CTL_A.LFS bit acts as L_FIRST, selecting whether the half SPORT trans- fers data first for the left or right channel.                                                                                                                                                                                                                                   |
| 16 (R/W)           | LFS        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Active high frame sync (DSP standard mode) or rising edge frame sync (multichannel mode) or right channel first (I 2 S/packed mode) or left channel first (left-justified mode)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 16 (R/W)           | LFS        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Active low frame sync (DSP standard mode) or falling edge frame sync (multichannel mode) or left channel first (I 2 S/packed mode) or right channel first (left-justified mode)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 32-25: SPORT\_CTL\_A Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | DIFS       | Data-Independent Frame Sync. The SPORT_CTL_A.DIFS bit selects whether the half SPORT uses a data-inde- pendent or data-dependent frame sync. When using a data-independent frame sync, the half SPORT generates the sync at the interval selected by the SPORT_DIV_A.FSDIV bit. When using a data-dependent frame sync, the half SPORT generates the sync on the selected interval when the transmit buffer is not empty or when the receive buffer is not full. Note that the SPORT_CTL_A.DIFS bit                                                                                         | Data-Independent Frame Sync. The SPORT_CTL_A.DIFS bit selects whether the half SPORT uses a data-inde- pendent or data-dependent frame sync. When using a data-independent frame sync, the half SPORT generates the sync at the interval selected by the SPORT_DIV_A.FSDIV bit. When using a data-dependent frame sync, the half SPORT generates the sync on the selected interval when the transmit buffer is not empty or when the receive buffer is not full. Note that the SPORT_CTL_A.DIFS bit                                                                                         |
| 15 (R/W)           | DIFS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Data-dependent frame sync                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 14 (R/W)           | IFS        | Internal Frame Sync. The SPORT_CTL_A.IFS bit selects whether the half SPORT uses an internal frame sync or uses an external frame sync. Note that the externally-generated frame sync does not need to be synchronous with the processor's system clock.                                                                                                                                                                                                                                                                                                                                    | Internal Frame Sync. The SPORT_CTL_A.IFS bit selects whether the half SPORT uses an internal frame sync or uses an external frame sync. Note that the externally-generated frame sync does not need to be synchronous with the processor's system clock.                                                                                                                                                                                                                                                                                                                                    |
| 14 (R/W)           | IFS        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | External frame sync                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14 (R/W)           | IFS        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Internal frame sync                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 13 (R/W)           | FSR        | Frame Sync Required. The SPORT_CTL_A.FSR bit selects whether or not the half SPORT requires frame sync for data transfer. This bit is automatically set when the half SPORT is in I 2 S / packed / left-justified mode ( SPORT_CTL_A.OPMODE =1) or is in multichannel mode ( SPORT_MCTL_A.MCE =1).                                                                                                                                                                                                                                                                                          | Frame Sync Required. The SPORT_CTL_A.FSR bit selects whether or not the half SPORT requires frame sync for data transfer. This bit is automatically set when the half SPORT is in I 2 S / packed / left-justified mode ( SPORT_CTL_A.OPMODE =1) or is in multichannel mode ( SPORT_MCTL_A.MCE =1).                                                                                                                                                                                                                                                                                          |
| 13 (R/W)           | FSR        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | No frame sync required                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 13 (R/W)           | FSR        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Frame sync required                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 12 (R/W)           | CKRE       | Clock Rising Edge. The SPORT_CTL_A.CKRE bit selects the rising or falling edge of the SPORT_ACLK clock for the half SPORT to sample receive data and frame sync. Note that the half SPORT changes the state of transmit data and frame sync signals on the non-selected edge of the SPORT_ACLK . Also, note that the transmit and receive relat- ed SPORT halves (A and B) should be programmed with the same value for the SPORT_CTL_A.CKRE bit. This programming drives the internally-generated signals on one edge of SPORT_ACLK and samples the received signals on the opposite edge. | Clock Rising Edge. The SPORT_CTL_A.CKRE bit selects the rising or falling edge of the SPORT_ACLK clock for the half SPORT to sample receive data and frame sync. Note that the half SPORT changes the state of transmit data and frame sync signals on the non-selected edge of the SPORT_ACLK . Also, note that the transmit and receive relat- ed SPORT halves (A and B) should be programmed with the same value for the SPORT_CTL_A.CKRE bit. This programming drives the internally-generated signals on one edge of SPORT_ACLK and samples the received signals on the opposite edge. |
| 12 (R/W)           | CKRE       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Clock falling edge                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 12 (R/W)           | CKRE       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Clock rising edge                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 32-25: SPORT\_CTL\_A Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | OPMODE     | Operation mode. The SPORT_CTL_A.OPMODE bit selects whether the half SPORT operates in DSP standard/multichannel mode or operates in I 2 S/packed/left-justified mode. The mode selection affects the operation of the SPORT_CTL_A.LAFS and SPORT_CTL_A.LFS bits. Also, the SPORT_CTL_A.OPMODE bit enables or disa- bles operation of the SPORT_CTL_A.GCLKEN , SPORT_CTL_A.FSED ,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Operation mode. The SPORT_CTL_A.OPMODE bit selects whether the half SPORT operates in DSP standard/multichannel mode or operates in I 2 S/packed/left-justified mode. The mode selection affects the operation of the SPORT_CTL_A.LAFS and SPORT_CTL_A.LFS bits. Also, the SPORT_CTL_A.OPMODE bit enables or disa- bles operation of the SPORT_CTL_A.GCLKEN , SPORT_CTL_A.FSED ,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 11 (R/W)           | OPMODE     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | DSP standard/multichannel mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 10 (R/W)           | ICLK       | Internal Clock. When the half SPORT is in DSP standard mode ( SPORT_CTL_A.OPMODE =0), the SPORT_CTL_A.ICLK bit selects whether the half SPORT uses an internal or exter- nal clock. For internal clock enabled, the half SPORT generates the SPORT_ACLK clock signal, and SPORT_ACLK is an output. The SPORT_DIV_A.CLKDIV serial clock divisor value determines the clock frequency. For internal clock disabled, the SPORT_ACLK clock signal is an input, and the serial clock divisor is ignored. Note that the externally-generated serial clock does not need to be synchronous with the processor's system clock.                                                                                                                                                                                                                                                              | Internal Clock. When the half SPORT is in DSP standard mode ( SPORT_CTL_A.OPMODE =0), the SPORT_CTL_A.ICLK bit selects whether the half SPORT uses an internal or exter- nal clock. For internal clock enabled, the half SPORT generates the SPORT_ACLK clock signal, and SPORT_ACLK is an output. The SPORT_DIV_A.CLKDIV serial clock divisor value determines the clock frequency. For internal clock disabled, the SPORT_ACLK clock signal is an input, and the serial clock divisor is ignored. Note that the externally-generated serial clock does not need to be synchronous with the processor's system clock.                                                                                                                                                                                                                                                              |
| 10 (R/W)           | ICLK       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | External clock                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 10 (R/W)           | ICLK       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Internal clock                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 9 (R/W)            | PACK       | Packing Enable. The SPORT_CTL_A.PACK bit enables the half SPORT to perform 16- to 32-bit packing on received data and to perform 32- to 16-bit unpacking on transmitted data. The receive packing operation packs two successive received words into a single 32-bit word. The transmit unpacking operation unpacks each 32-bit word and transmits it as two 16-bit words. The first 16-bit (or smaller) word is right-justified in bits 15:0 of the packed word, and the second 16-bit (or smaller) word is right-justified in bits 31:16. This format applies to both receive (packing) and transmit (unpacking) operations. Companding may be used with word packing or unpacking. The half SPORT gener- ates data transfer related interrupts when packing is enabled. The transmit and receive interrupts are generated for the 32-bit packed words, not for each 16-bit word. | Packing Enable. The SPORT_CTL_A.PACK bit enables the half SPORT to perform 16- to 32-bit packing on received data and to perform 32- to 16-bit unpacking on transmitted data. The receive packing operation packs two successive received words into a single 32-bit word. The transmit unpacking operation unpacks each 32-bit word and transmits it as two 16-bit words. The first 16-bit (or smaller) word is right-justified in bits 15:0 of the packed word, and the second 16-bit (or smaller) word is right-justified in bits 31:16. This format applies to both receive (packing) and transmit (unpacking) operations. Companding may be used with word packing or unpacking. The half SPORT gener- ates data transfer related interrupts when packing is enabled. The transmit and receive interrupts are generated for the 32-bit packed words, not for each 16-bit word. |
| 9 (R/W)            | PACK       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 9 (R/W)            | PACK       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 32-25: SPORT\_CTL\_A Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8:4 (R/W)          | SLEN       | Serial Word Length. The SPORT_CTL_A.SLEN bits selects word length in bits for the half SPORT's data transfers. Word may be from 4- to 32-bits in length. The formula for selecting the word length in bits is: SPORT_CTL_A.SLEN = (serial word length in bits) - 1 For DSP standard mode ( SPORT_CTL_A.OPMODE =0), use SPORT_CTL_A.SLEN of 3 to 31 bits. For I 2 S / packed / left-justified mode ( SPORT_CTL_A.OPMODE =1), use SPORT_CTL_A.SLEN of 4 to 31 bits. |
| 3 (R/W)            | LSBF       | Least-Significant Bit First. The SPORT_CTL_A.LSBF bit selects whether the half SPORT transmits or receives data LSB first or MSB first.                                                                                                                                                                                                                                                                                                                           |
| 2:1 (R/W)          | DTYPE      | Data Type. The SPORT_CTL_A.DTYPE bits selects the data type formatting for the half SPORT's data transfers in DSP standard mode ( SPORT_CTL_A.OPMODE =0).                                                                                                                                                                                                                                                                                                         |
| 2:1 (R/W)          | DTYPE      | 0 Right-justify data, zero-fill unused MSBs                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 2:1 (R/W)          | DTYPE      | 1 Right-justify data, sign-extend unused MSBs                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 2:1 (R/W)          | DTYPE      | 2 u-law compand data                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 0 (R/W)            | SPENPRI    | Serial Port Enable (Primary). The SPORT_CTL_A.SPENPRI bit enables the half SPORT's primary channel. When this bit is cleared (changes from =1 to =0), the half SPORT automatically flush- es the channel's data buffers. Disable                                                                                                                                                                                                                                  |
| 0 (R/W)            | SPENPRI    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | SPENPRI    | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                          |

## Half SPORT 'B' Control Register

The SPORT\_CTL\_B register contains transmit and receive control bits for SPORT half 'B', including serial port mode selection for the half SPORT's primary and secondary channels. The function of some bits in the SPORT\_CTL\_B register vary, depending on the SPORT's operating mode. For more information, see the SPORT operating modes description. If reading reserved bits, the read value is the last written value to these bits or is the reset value of these bits.

Figure 32-25: SPORT\_CTL\_B Register Diagram

<!-- image -->

Table 32-26: SPORT\_CTL\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:30 (R/NW)       | DXSPRI     | Data Transfer Buffer Status (Primary). The SPORT_CTL_B.DXSPRI bit field indicates the status of the half SPORT's pri- mary channel data buffer. 0 Empty 1 Reserved 2 Partially full Full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 31:30 (R/NW)       | DXSPRI     | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 29 (R/NW)          | DERRPRI    | Data Error Status (Primary). The SPORT_CTL_B.DERRPRI bit reports the half SPORT's primary channel trans- mit underflow status or receive overflow status, depending on the SPORT transfer di- rection. If the SPORT_CTL_B.FSR bit =1, the SPORT_CTL_B.DERRPRI bit indicates whether the SPORT_BFS signal (from an internal or external source) occurred while the SPORT_TXPRI_B data buffer was empty (during transmit) or the SPORT_RXPRI_B data buffer was full (during receive). The SPORT transmits or re- ceives data whenever it detects the SPORT_BFS signal. It is important to note that, as a receiver, the SPORT_CTL_B.DERRPRI bit indicates when the channel has re- ceived new data while the SPORT_RXPRI_B receive buffer is full. This new data overwrites existing data. If the SPORT_CTL_B.FSR bit =0, the SPORT_CTL_B.DERRPRI bit is set when- ever the SPORT is required to transmit while the SPORT_TXPRI_B transmit buffer is empty and is set whenever the SPORT is required to receive while the SPORT_RXPRI_B receive buffer is full. The SPORT clears the SPORT_CTL_B.DERRPRI bit if the SPORT_ERR_B.DERRPSTAT bit is cleared. 0 No error 1 Error (Tx underflow or Rx overflow) |
| 28:27 (R/NW)       | DXSSEC     | Data Transfer Buffer Status (Secondary). The SPORT_CTL_B.DXSSEC bit field indicates the status of the half SPORT's sec- ondary channel data buffer.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 28:27 (R/NW)       | DXSSEC     | 0 Empty                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 28:27 (R/NW)       | DXSSEC     | 1 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 28:27 (R/NW)       | DXSSEC     | 2 Partially full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 28:27 (R/NW)       | DXSSEC     | 3 Full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 32-26: SPORT\_CTL\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/NW)          | DERRSEC    | Data Error Status (Secondary). The SPORT_CTL_B.DERRSEC bit reports the half SPORT's secondary channel transmit underflow status or receive overflow status, depending on the SPORT transfer direction. If the SPORT_CTL_B.FSR bit =1, the SPORT_CTL_B.DERRSEC bit indicates whether the SPORT_BFS signal (from an internal or external source) occurred while the SPORT_TXSEC_B data buffer was empty (during transmit) or the SPORT_RXSEC_B data buffer was full (during receive). The SPORT transmits or re- ceives data whenever it detects the SPORT_BFS signal. It is important to note that, as a receiver, the SPORT_CTL_B.DERRSEC bit indicates when the channel has re- ceived new data while the SPORT_RXSEC_B receive buffer is full. This new data overwrites existing data. If the SPORT_CTL_B.FSR bit =0, the SPORT_CTL_B.DERRSEC bit is set when- ever the SPORT is required to transmit while the SPORT_TXSEC_B transmit buffer is empty. It is also set whenever the SPORT is required to receive while the SPORT_RXSEC_B receive buffer is full. The SPORT clears the SPORT_CTL_B.DERRSEC bit if the SPORT_ERR_B.DERRSSTAT bit is cleared. | Data Error Status (Secondary). The SPORT_CTL_B.DERRSEC bit reports the half SPORT's secondary channel transmit underflow status or receive overflow status, depending on the SPORT transfer direction. If the SPORT_CTL_B.FSR bit =1, the SPORT_CTL_B.DERRSEC bit indicates whether the SPORT_BFS signal (from an internal or external source) occurred while the SPORT_TXSEC_B data buffer was empty (during transmit) or the SPORT_RXSEC_B data buffer was full (during receive). The SPORT transmits or re- ceives data whenever it detects the SPORT_BFS signal. It is important to note that, as a receiver, the SPORT_CTL_B.DERRSEC bit indicates when the channel has re- ceived new data while the SPORT_RXSEC_B receive buffer is full. This new data overwrites existing data. If the SPORT_CTL_B.FSR bit =0, the SPORT_CTL_B.DERRSEC bit is set when- ever the SPORT is required to transmit while the SPORT_TXSEC_B transmit buffer is empty. It is also set whenever the SPORT is required to receive while the SPORT_RXSEC_B receive buffer is full. The SPORT clears the SPORT_CTL_B.DERRSEC bit if the SPORT_ERR_B.DERRSSTAT bit is cleared. |
| 26 (R/NW)          | DERRSEC    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | No error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 25 (R/W)           | SPTRAN     | Serial Port Transfer Direction. The SPORT_CTL_B.SPTRAN bit selects the transfer direction (receive or transmit) for the half SPORT's primary and secondary channels.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Serial Port Transfer Direction. The SPORT_CTL_B.SPTRAN bit selects the transfer direction (receive or transmit) for the half SPORT's primary and secondary channels.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 25 (R/W)           | SPTRAN     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Receive                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 25 (R/W)           | SPTRAN     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Transmit                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 24 (R/W)           | SPENSEC    | Serial Port Enable (Secondary). The SPORT_CTL_B.SPENSEC bit enables the half SPORT's secondary channel. When this bit is cleared (changes from =1 to =0), the half SPORT automatically flush- es the channel's data buffers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Serial Port Enable (Secondary). The SPORT_CTL_B.SPENSEC bit enables the half SPORT's secondary channel. When this bit is cleared (changes from =1 to =0), the half SPORT automatically flush- es the channel's data buffers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 24 (R/W)           | SPENSEC    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 24 (R/W)           | SPENSEC    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 32-26: SPORT\_CTL\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | GCLKEN     | Gated Clock Enable. The SPORT_CTL_B.GCLKEN bit enables gated clock operation for the half SPORT when in DSP serial mode or left-justified stereo modes ( SPORT_CTL_B.OPMODE = 0 or 1). This bit is ignored when the half SPORT is in right-justified mode ( SPORT_CTL_B.RJUST =1) or multichannel mode ( SPORT_MCTL_B.MCE =1). When SPORT_CTL_B.GCLKEN is enabled, the SPORT clock is active when the SPORT is transferring data or when the frame sync changes (transitions to active state). 0 Disable                                      |
| 20 (R/W)           | TFIEN      | 1 Enable Transmit Finish Interrupt Enable. The SPORT_CTL_B.TFIEN bit selects when the half SPORT issues its transmission complete interrupt request, if a DMAcomplete interrupt request is enabled by the DMA_CFG.INT configuration. When enabled ( SPORT_CTL_B.TFIEN =1), the DMAcomplete peripheral interrupt request is generated when the last bit of last word in the DMAis shifted out. When disabled ( SPORT_CTL_B.TFIEN =0), theDMA interrupt request is generated when the DMAcounter expires (the last word goes to                 |
| 19 (R/W)           | FSED       | Frame Sync Edge Detect. The SPORT_CTL_B.FSED bit enables the half SPORT to start transmitting or re- ceiving after detecting an active edge of an external frame sync. The SPORT_CTL_B.FSED may be enabled even during an active frame sync, and the half SPORT starts the transfer on the next valid rising or falling edge of external frame sync. If disabled ( SPORT_CTL_B.FSED =0), the half SPORT operates in the stand- ard level-sensitive detection mode for external frame sync. 0 Level detect frame sync 1 Edge detect frame sync |

Table 32-26: SPORT\_CTL\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | RJUST      | Right-Justified Operation Mode. The SPORT_CTL_B.RJUST bit enables the half SPORT (if SPORT_CTL_B.OPMODE =1) to transfer data in right-justified operation mode. In this mode, the half SPORT aligns data to the end of the frame sync, rather than the start of the frame sync. When using right-justified mode, systems should program an appropriate delay count to introduce a clock delay before the half SPORT state ma- chine starts to capture data. This value is set in the DCNT field (right-justified mode usage of the SPORT_MCTL_B.WOFFSET field). For information about appropriate delay selections, see the SPORT operating modes section.                                     | Right-Justified Operation Mode. The SPORT_CTL_B.RJUST bit enables the half SPORT (if SPORT_CTL_B.OPMODE =1) to transfer data in right-justified operation mode. In this mode, the half SPORT aligns data to the end of the frame sync, rather than the start of the frame sync. When using right-justified mode, systems should program an appropriate delay count to introduce a clock delay before the half SPORT state ma- chine starts to capture data. This value is set in the DCNT field (right-justified mode usage of the SPORT_MCTL_B.WOFFSET field). For information about appropriate delay selections, see the SPORT operating modes section.                                     |
| 18 (R/W)           | RJUST      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 17 (R/W)           | LAFS       | Late Frame Sync / OPMODE2. When the half SPORT is in DSP standard mode ( SPORT_CTL_B.OPMODE =0) or in right-justified mode ( SPORT_CTL_B.RJUST =1), the SPORT_CTL_B.LAFS bit selects whether the half SPORT generates a late frame sync ( SPORT_BFS during first data bit) or generates an early frame sync signal ( SPORT_BFS during serial clock cycle before first data bit). When the half SPORT is in I 2 S / left-justified mode ( SPORT_CTL_B.OPMODE =1), the SPORT_CTL_B.LAFS bit acts as OPMODE2, selecting whether the half SPORT is in left-justified mode or I 2 S mode. When the half SPORT is in multichannel mode ( SPORT_MCTL_B.MCE =1), the SPORT_CTL_B.LAFS bit is reserved. | Late Frame Sync / OPMODE2. When the half SPORT is in DSP standard mode ( SPORT_CTL_B.OPMODE =0) or in right-justified mode ( SPORT_CTL_B.RJUST =1), the SPORT_CTL_B.LAFS bit selects whether the half SPORT generates a late frame sync ( SPORT_BFS during first data bit) or generates an early frame sync signal ( SPORT_BFS during serial clock cycle before first data bit). When the half SPORT is in I 2 S / left-justified mode ( SPORT_CTL_B.OPMODE =1), the SPORT_CTL_B.LAFS bit acts as OPMODE2, selecting whether the half SPORT is in left-justified mode or I 2 S mode. When the half SPORT is in multichannel mode ( SPORT_MCTL_B.MCE =1), the SPORT_CTL_B.LAFS bit is reserved. |
| 17 (R/W)           | LAFS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Early frame sync (or I 2 S mode)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 17 (R/W)           | LAFS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Late frame sync (or left-justified mode)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 32-26: SPORT\_CTL\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | LFS        | Active-Low Frame Sync / L_FIRST / PLFS. When the half SPORT is in DSP standard mode and multichannel mode ( SPORT_CTL_B.OPMODE =0), the SPORT_CTL_B.LFS bit selects whether the half SPORT uses active low or active high frame sync. When the half SPORT is in I 2 S / packed / left-justified mode ( SPORT_CTL_B.OPMODE =1), the SPORT_CTL_B.LFS bit acts as L_FIRST, se- lecting whether the half SPORT transfers data first for the left or right channel. | Active-Low Frame Sync / L_FIRST / PLFS. When the half SPORT is in DSP standard mode and multichannel mode ( SPORT_CTL_B.OPMODE =0), the SPORT_CTL_B.LFS bit selects whether the half SPORT uses active low or active high frame sync. When the half SPORT is in I 2 S / packed / left-justified mode ( SPORT_CTL_B.OPMODE =1), the SPORT_CTL_B.LFS bit acts as L_FIRST, se- lecting whether the half SPORT transfers data first for the left or right channel. |
| 16 (R/W)           | LFS        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Active high frame sync (DSP standard mode) or rising edge frame sync (multichannel mode) or right channel first (I 2 S/packed mode) or left channel first (left-justified mode)                                                                                                                                                                                                                                                                                |
| 15 (R/W)           | DIFS       | Data-Independent Frame Sync. The SPORT_CTL_B.DIFS bit selects whether the half SPORT uses a data-inde- pendent or data-dependent frame sync. When using a data-independent frame sync, the half SPORT generates the sync at the interval selected by SPORT_DIV_B.FSDIV . When using a data-dependent frame sync, the half SPORT generates the sync on the selected interval when the transmit buffer is not empty or                                           | Data-Independent Frame Sync. The SPORT_CTL_B.DIFS bit selects whether the half SPORT uses a data-inde- pendent or data-dependent frame sync. When using a data-independent frame sync, the half SPORT generates the sync at the interval selected by SPORT_DIV_B.FSDIV . When using a data-dependent frame sync, the half SPORT generates the sync on the selected interval when the transmit buffer is not empty or                                           |
| 15 (R/W)           | DIFS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Data-dependent frame sync                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 15 (R/W)           | DIFS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Data-independent frame sync                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 14 (R/W)           | IFS        | Internal Frame Sync. The SPORT_CTL_B.IFS bit selects whether the half SPORT uses an internal frame sync or uses an external frame sync. Note that the externally-generated frame sync does not need to be synchronous with the processor's system clock.                                                                                                                                                                                                       | Internal Frame Sync. The SPORT_CTL_B.IFS bit selects whether the half SPORT uses an internal frame sync or uses an external frame sync. Note that the externally-generated frame sync does not need to be synchronous with the processor's system clock.                                                                                                                                                                                                       |
| 14 (R/W)           | IFS        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                              | External frame sync                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14 (R/W)           | IFS        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Internal frame sync                                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 32-26: SPORT\_CTL\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | FSR        | Frame Sync Required. The SPORT_CTL_B.FSR selects whether or not the half SPORT requires frame sync for data transfer. This bit is automatically set when the half SPORT is in I 2 S / packed / left-justified mode ( SPORT_CTL_B.OPMODE =1) or is in multichannel mode ( SPORT_MCTL_B.MCE =1).                                                                                                                                                                                                                                                                                                                             |
| 13 (R/W)           | FSR        | 0 No frame sync required                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 12 (R/W)           | CKRE       | 1 Frame sync required Clock Rising Edge. The SPORT_CTL_B.CKRE selects the rising or falling edge of the SPORT_BCLK clock for the half SPORT to sample receive data and frame sync. Note that the half SPORT changes the state of transmit data and frame sync signals on the non-selected edge of the SPORT_BCLK . Also note that the transmit and receive related SPORT halves (A and B) should be programmed with the same value for SPORT_CTL_B.CKRE . This programming drives the internally-generated signals on one edge of SPORT_BCLK and samples the received signals on the opposite edge.                        |
| 12 (R/W)           | CKRE       | 0 Clock falling edge                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 11 (R/W)           | OPMODE     | 1 Clock rising edge Operation Mode. The SPORT_CTL_B.OPMODE bit selects whether the half SPORT operates in DSP standard / multichannel mode or operates in I 2 S / packed / left-justified mode. The mode selection affects the operation of the SPORT_CTL_B.LAFS and SPORT_CTL_B.LFS bits. Also, the SPORT_CTL_B.OPMODE bit enables or disa- bles operation of the SPORT_CTL_B.GCLKEN , SPORT_CTL_B.FSED , SPORT_CTL_B.RJUST , SPORT_CTL_B.DIFS , SPORT_CTL_B.FSR , and SPORT_CTL_B.CKRE bits.                                                                                                                             |
| 11 (R/W)           | OPMODE     | 0 DSP standard/multichannel mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 11 (R/W)           | OPMODE     | 1 I 2 S/packed/left-justified mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 10 (R/W)           | ICLK       | Internal Clock. When the half SPORT is in DSP standard mode ( SPORT_CTL_B.OPMODE =0), the SPORT_CTL_B.ICLK bit selects whether the half SPORT uses an internal or exter- nal clock. For internal clock enabled, the half SPORT generates the SPORT_BCLK clock signal, and the SPORT_BCLK is an output. The SPORT_DIV_B.CLKDIV serial clock divisor value determines the clock frequency. For internal clock disabled, the SPORT_BCLK clock signal is an input, and the serial clock divisor is ignored. Note that the externally-generated serial clock does not need to be synchronous with the processor's system clock. |
| 10 (R/W)           | ICLK       | 0 External clock                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 10 (R/W)           | ICLK       | 1 Internal clock                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 32-26: SPORT\_CTL\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | PACK       | Packing Enable. The SPORT_CTL_B.PACK bit enables the half SPORT to perform 16- to 32-bit packing on received data and to perform 32- to 16-bit unpacking on transmitted data. The receive packing operation packs two successive received words into a single 32-bit word. The transmit unpacking operation unpacks each 32-bit word and transmits it as two 16-bit words. The first 16-bit (or smaller) word is right-justified in bits 15:0 of the packed word, and the second 16-bit (or smaller) word is right-justified in bits 31:16. This format applies to both receive (packing) and transmit (unpacking) operations. Companding may be used with word packing or unpacking. The half SPORT gener- ates data transfer related interrupts when packing is enabled. The transmit and receive interrupts are generated for the 32-bit packed words, not for each 16-bit word. | Packing Enable. The SPORT_CTL_B.PACK bit enables the half SPORT to perform 16- to 32-bit packing on received data and to perform 32- to 16-bit unpacking on transmitted data. The receive packing operation packs two successive received words into a single 32-bit word. The transmit unpacking operation unpacks each 32-bit word and transmits it as two 16-bit words. The first 16-bit (or smaller) word is right-justified in bits 15:0 of the packed word, and the second 16-bit (or smaller) word is right-justified in bits 31:16. This format applies to both receive (packing) and transmit (unpacking) operations. Companding may be used with word packing or unpacking. The half SPORT gener- ates data transfer related interrupts when packing is enabled. The transmit and receive interrupts are generated for the 32-bit packed words, not for each 16-bit word. |
| 9 (R/W)            | PACK       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 8:4 (R/W)          | SLEN       | Serial Word Length. The SPORT_CTL_B.SLEN bits selects word length in bits for the half SPORT's data transfers. Word may be from 4- to 32-bits in length. The formula for selecting the word length in bits is: SPORT_CTL_B.SLEN = (serial word length in bits) - 1 For DSP standard mode ( SPORT_CTL_B.OPMODE =0), use SPORT_CTL_B.SLEN of 3 to 31 bits. For I 2 S / packed / left-justified mode ( SPORT_CTL_B.OPMODE =1), use                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Serial Word Length. The SPORT_CTL_B.SLEN bits selects word length in bits for the half SPORT's data transfers. Word may be from 4- to 32-bits in length. The formula for selecting the word length in bits is: SPORT_CTL_B.SLEN = (serial word length in bits) - 1 For DSP standard mode ( SPORT_CTL_B.OPMODE =0), use SPORT_CTL_B.SLEN of 3 to 31 bits. For I 2 S / packed / left-justified mode ( SPORT_CTL_B.OPMODE =1), use                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 3 (R/W)            | LSBF       | Least-Significant Bit First. The SPORT_CTL_B.LSBF bit selects whether the half SPORT transmits or receives data LSB first or MSB first.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Least-Significant Bit First. The SPORT_CTL_B.LSBF bit selects whether the half SPORT transmits or receives data LSB first or MSB first.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 3 (R/W)            | LSBF       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | MSB first sent/received (big endian)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 3 (R/W)            | LSBF       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | LSB first sent/received (little endian)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 2:1 (R/W)          | DTYPE      | Data Type. The SPORT_CTL_B.DTYPE bits selects the data type formatting for the half SPORT's data transfers in DSP standard mode ( SPORT_CTL_B.OPMODE =0).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Data Type. The SPORT_CTL_B.DTYPE bits selects the data type formatting for the half SPORT's data transfers in DSP standard mode ( SPORT_CTL_B.OPMODE =0).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 2:1 (R/W)          | DTYPE      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Right-justify data, zero-fill unused MSBs                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 2:1 (R/W)          | DTYPE      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Right-justify data, sign-extend unused MSBs                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2:1 (R/W)          | DTYPE      | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | u-law compand data                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 2:1 (R/W)          | DTYPE      | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | A-law compand data                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 32-26: SPORT\_CTL\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | SPENPRI    | Serial Port Enable (Primary). The SPORT_CTL_B.SPENPRI bit enables the half SPORT's primary channel. When this bit is cleared (changes from =1 to =0), the half SPORT automatically flush- es the channel's data buffers. |
| 0 (R/W)            | SPENPRI    | 0 Disable                                                                                                                                                                                                                |
| 0 (R/W)            | SPENPRI    | 1 Enable                                                                                                                                                                                                                 |

## Half SPORT 'A' Divisor Register

The SPORT\_DIV\_A register contains divisor values that determine frequencies of internally-generated clocks and frame syncs for half SPORT 'A'.

Figure 32-26: SPORT\_DIV\_A Register Diagram

<!-- image -->

Table 32-27: SPORT\_DIV\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | FSDIV      | Frame Sync Divisor. The SPORT_DIV_A.FSDIV bits select the number of transmit or receive clock cy- cles that the half SPORT counts before generating a frame sync pulse. The half SPORT counts serial clock cycles whether these are from an internally- or an external- ly-generated serial clock. The formula relating SPORT_DIV_A.FSDIV to the num- ber of cycles between frame sync pulses is: SPORT_DIV_A.FSDIV = (number of serial clocks between frame syncs) - 1 Use the following equation to determine the value of SPORT_DIV_A.FSDIV , given the serial clock frequency and desired frame sync frequency: FSDIV = ( SPORT_ACLK / SPORT_AFS ) - 1 Note that the frame sync is continuously active when SPORT_DIV_A.FSDIV = 0. The value of SPORT_DIV_A.FSDIV should not be less than the serial word length ( SPORT_CTL_A.SLEN ), as this may cause an external device to abort the current operation or cause other unpredictable results. |
| 15:0 (R/W)         | CLKDIV     | Clock Divisor. The SPORT_DIV_A.CLKDIV bits select the divisor that the half SPORT uses to calculate the serial clock ( SPORT_ACLK ) from the processor system clock ( SCLK0). The divisor is a 16-bit value, allowing a wide range of serial clock rates. When config- ured for internal clock ( SPORT_CTL_A.ICLK =1), legal SPORT_DIV_A.CLKDIV values are 0 to 65535. Given the processor system clock frequency and desired serial clock frequency, use the following formula to calculate the value of SPORT_DIV_A.CLKDIV : CLKDIV = ( SCLK0 / SPORT_ACLK ) - 1 For the maximum serial clock frequency, see the processor data sheet.                                                                                                                                                                                                                                                                                                             |

## Half SPORT 'B' Divisor Register

The SPORT\_DIV\_B contains divisor values that determine frequencies of internally-generated clocks and frame syncs for SPORT half 'B'.

Figure 32-27: SPORT\_DIV\_B Register Diagram

<!-- image -->

Table 32-28: SPORT\_DIV\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | FSDIV      | Frame Sync Divisor. The SPORT_DIV_B.FSDIV bits select the number of transmit or receive clock cy- cles that the half SPORT counts before generating a frame sync pulse. The half SPORT counts serial clock cycles whether these are from an internally- or an external- ly-generated serial clock. The formula relating SPORT_DIV_B.FSDIV to the num- ber of cycles between frame sync pulses is: SPORT_DIV_B.FSDIV = (number of serial clocks between frame syncs) - 1 Use the following equation to determine the value of SPORT_DIV_B.FSDIV , given the serial clock frequency and desired frame sync frequency: FSDIV = ( SPORT_BCLK / SPORT_BFS ) - 1 Note that the frame sync is continuously active when SPORT_DIV_B.FSDIV = 0. The value of SPORT_DIV_B.FSDIV should not be less than the serial word length ( SPORT_CTL_B.SLEN ), as this may cause an external device to abort the current operation or cause other unpredictable results. |
| 15:0 (R/W)         | CLKDIV     | Clock Divisor. The SPORT_DIV_B.CLKDIV bits select the divisor that the half SPORT uses to calculate the serial clock ( SPORT_BCLK ) from the processor system clock ( SCLK0). The divisor is a 16-bit value, allowing a wide range of serial clock rates. When config- ured for internal clock ( SPORT_CTL_B.ICLK =1), legal SPORT_DIV_B.CLKDIV values are 0 to 65535. Given the processor system clock frequency and desired serial clock frequency, use the following formula to calculate the value of SPORT_DIV_B.CLKDIV : CLKDIV = ( SCLK0 SPORT_BCLK ) - 1 For the maximum serial clock frequency, see the processor data sheet.                                                                                                                                                                                                                                                                                                               |

## Half SPORT 'A' Error Register

The SPORT\_ERR\_A register contains error status and error interrupt mask bits for SPORT half 'A', including error handling bits for the half SPORT's primary and secondary channels and frame sync. Detected errors are frame sync violations or buffer over/underflow conditions.

Figure 32-28: SPORT\_ERR\_A Register Diagram

<!-- image -->

Table 32-29: SPORT\_ERR\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1C)          | FSERRSTAT  | Frame Sync Error Status. The SPORT_ERR_A.FSERRSTAT bit indicates that the half SPORT has detected a frame sync when the bit count (bits remaining in the frame) is non-zero. When a half SPORT is receiving or transmitting, its bit count is set to a word length (for example, SPORT_CTL_A.SLEN = 31). After each serial clock edge, the half SPORT decre- ments the transfer's bit count. After the word is received or transmitted, the transfer's bit count reaches zero, and the half SPORT resets it (for example, to 32) on next frame sync. Normal SPORT data transfers always have a non-zero bit count value when ac- tive transmission or reception is occurring. Normal SPORT frame syncs occur after the bit count becomes zero. | Frame Sync Error Status. The SPORT_ERR_A.FSERRSTAT bit indicates that the half SPORT has detected a frame sync when the bit count (bits remaining in the frame) is non-zero. When a half SPORT is receiving or transmitting, its bit count is set to a word length (for example, SPORT_CTL_A.SLEN = 31). After each serial clock edge, the half SPORT decre- ments the transfer's bit count. After the word is received or transmitted, the transfer's bit count reaches zero, and the half SPORT resets it (for example, to 32) on next frame sync. Normal SPORT data transfers always have a non-zero bit count value when ac- tive transmission or reception is occurring. Normal SPORT frame syncs occur after the bit count becomes zero. |
| 6 (R/W1C)          | FSERRSTAT  | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | No error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 6 (R/W1C)          | FSERRSTAT  | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Error (non-zero bit count at frame sync)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 32-29: SPORT\_ERR\_A Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W1C)          | DERRSSTAT  | Data Error Secondary Status. The SPORT_ERR_A.DERRSSTAT bit indicates the error status for the half SPORT's secondary channel data buffers. During transmit ( SPORT_CTL_A.SPTRAN =1), the SPORT_ERR_A.DERRSSTAT bit indicates the transmit underflow status. During receive ( SPORT_CTL_A.SPTRAN =0), the SPORT_ERR_A.DERRSSTAT bit indicates the receive overflow status. This bit is used to clear the latch of SPORT status interrupt request when triggered by a secon- dary data error. This bit can also be used to clear the read-only                         |
| 4 (R/W1C)          | DERRPSTAT  | Data Error Primary Status. The SPORT_ERR_A.DERRPSTAT bit indicates the error status for the half SPORT's primary channel data buffers. During transmit ( SPORT_CTL_A.SPTRAN =1), the SPORT_ERR_A.DERRPSTAT bit indicates the transmit underflow status. During receive ( SPORT_CTL_A.SPTRAN =0), the SPORT_ERR_A.DERRPSTAT bit indicates the receive overflow status. This bit is used to clear the latch of SPORT status interrupt request when triggered by a primary data error. This bit can also be used to clear the read-only SPORT_CTL_A.DERRPRI status bit. |
| 2 (R/W)            | FSERRMSK   | 1 Error (transmit underflow or receive overflow) Frame Sync Error (Interrupt) Mask. The SPORT_ERR_A.FSERRMSK unmasks (enables) the half SPORT to generate the frame sync error interrupt request.                                                                                                                                                                                                                                                                                                                                                                    |
| 1                  | DERRSMSK   | 0 Mask (disable) 1 Unmask (enable) Data Error Secondary (Interrupt) Mask.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| (R/W)              | DERRPMSK   | The SPORT_ERR_A.DERRSMSK unmasks (enables) the half SPORT to generate the data error interrupt request for the secondary channel. 0 Mask (disable) 1 Unmask (enable) Data Error Primary (Interrupt) Mask.                                                                                                                                                                                                                                                                                                                                                            |
| 0 (R/W)            |            | The SPORT_ERR_A.DERRPMSK unmasks (enables) the half SPORT to generate the data error interrupt request for the primary channel. 0 Mask (disable)                                                                                                                                                                                                                                                                                                                                                                                                                     |

## Half SPORT 'B' Error Register

The SPORT\_ERR\_B register contains error status and error interrupt mask bits for SPORT half 'B', including error handling bits for the half SPORT's primary and secondary channels and frame sync. Detected errors are frame sync violations or buffer over/underflow conditions.

Figure 32-29: SPORT\_ERR\_B Register Diagram

<!-- image -->

Table 32-30: SPORT\_ERR\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1C)          | FSERRSTAT  | Frame Sync Error Status. The SPORT_ERR_B.FSERRSTAT bit indicates that the half SPORT has detected a frame sync when the bit count (bits remaining in the frame) is non-zero. When a half SPORT is receiving or transmitting, its bit count is set to a word length (for example, SPORT_CTL_B.SLEN = 31). After each serial clock edge, the half SPORT decre- ments the transfer's bit count. After the word is received or transmitted, the transfer's bit count reaches zero, and the half SPORT resets it (for example, to 32) on next frame sync. Normal SPORT data transfers always have a non-zero bit count value when ac- tive transmission or reception is occurring. Normal SPORT frame syncs occur after the bit count becomes zero. No error |
| 6 (R/W1C)          | FSERRSTAT  | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 6 (R/W1C)          | FSERRSTAT  | 1 Error (non-zero bit count at frame sync)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 32-30: SPORT\_ERR\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W1C)          | DERRSSTAT  | Data Error Secondary Status. The SPORT_ERR_B.DERRSSTAT bit indicates the error status for the half SPORT's secondary channel data buffers. During transmit ( SPORT_CTL_B.SPTRAN =1), SPORT_ERR_B.DERRSSTAT indicates the trans- mit underflow status. During receive ( SPORT_CTL_B.SPTRAN =0), SPORT_ERR_B.DERRSSTAT indicates the receive overflow status. This bit is used to clear the latch of SPORT status interrupt request when triggered by a secondary da- ta error. This bit can also be used to clear the read-only SPORT_CTL_B.DERRSEC |
| 4 (R/W1C)          | DERRPSTAT  | Data Error Primary Status. The SPORT_ERR_B.DERRPSTAT bit indicates the error status for the half SPORT's primary channel data buffers. During transmit ( SPORT_CTL_B.SPTRAN =1), the SPORT_ERR_B.DERRPSTAT bit indicates the transmit underflow status. During receive ( SPORT_CTL_B.SPTRAN =0), the SPORT_ERR_B.DERRPSTAT bit indicates the receive overflow status. This bit is used to clear the latch of SPORT status interrupt request when triggered by a primary data error. This bit can also be                                           |
| 2 (R/W)            | FSERRMSK   | Frame Sync Error (Interrupt) Mask. The SPORT_ERR_B.FSERRMSK unmasks (enables) the half SPORT to generate the frame sync error interrupt request.                                                                                                                                                                                                                                                                                                                                                                                                   |
| 1                  | DERRSMSK   | 0 Mask (disable) 1 Unmask (enable) Data Error Secondary (Interrupt) Mask.                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| (R/W) 0 (R/W)      | DERRPMSK   | The SPORT_ERR_B.DERRSMSK unmasks (enables) the half SPORT to generate the data error interrupt request for the secondary channel. 0 Mask (disable) 1 Unmask (enable) Data Error Primary (Interrupt) Mask. The SPORT_ERR_B.DERRPMSK unmasks (enables) the half SPORT to generate the data error interrupt request for the primary channel. 0 Mask (disable)                                                                                                                                                                                         |

## Half SPORT 'A' Multichannel Control Register

The SPORT\_MCTL\_A register controls the half SPORT's multichannel operations. This register enables multichannel operation, enables multichannel data packing, selects the multichannel frame delay, selects the number of multichannel slots, and selects the multichannel window offset size.

Figure 32-30: SPORT\_MCTL\_A Register Diagram

<!-- image -->

Table 32-31: SPORT\_MCTL\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:16 (R/W)        | WOFFSET    | Window Offset. The SPORT_MCTL_A.WOFFSET bits select the start location for the half SPORT's active window of channels within the 1024-channel range. A value of 0 specifies no offset and 896 is the largest value that permits using all 128 channels. When multi- channel mode is disabled ( SPORT_MCTL_A.MCE =0) and the right-justified mode is enabled ( SPORT_CTL_A.RJUST =1), the least significant 6 bits of SPORT_MCTL_A.WOFFSET serve as the delay count (DCNT) field. These bits in- troduce a clock delay before the half SPORT state machine starts to capture data. For information about appropriate delay selections, see the SPORT operating modes sec- tion. |
| 14:8 (R/W)         | WSIZE      | Window Size. The SPORT_MCTL_A.WSIZE bits select the window size for the half SPORT's ac- tive window of channels. Use the following formula to calculate the window size value: SPORT_MCTL_A.WSIZE = (number of channel slots) -1                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 7:4 (R/W)          | MFD        | Multichannel Frame Delay. The SPORT_MCTL_A.MFD bits select the delay (in serial clock cycles) between the half SPORT's multichannel frame sync pulse and channel 0. The 4-bit field allows se- lecting multichannel frame delay of 0-15 serial clocks.                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 32-31: SPORT\_MCTL\_A Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | MCPDE      | Multichannel Packing DMAEnable. The SPORT_MCTL_A.MCPDE bit enables DMAdata packing for transmit and ena- bles DMAdata unpacking for the half SPORT's multichannel data transfers. 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | MCE        | 1 Enable Multichannel enable. The SPORT_MCTL_A.MCE bit enables multichannel operations for the half SPORT. The half SPORT is configured in normal multichannel mode if SPORT_CTL_A.OPMODE =0; while it is configured in packed mode if SPORT_CTL_A.OPMODE =1. When configuring in these modes, the multichannel enable bit ( SPORT_MCTL_A.MCE ) should be set before enabling the SPORT data channel enable bits ( SPORT_CTL_A.SPENPRI and/or SPORT_CTL_A.SPENSEC ). When these channel bits transition from 1 to 0, note that the half SPORT's data transfer buffers are cleared, and the SPORT_CTL_A.DERRPRI and SPORT_CTL_A.DERRSEC bits are cleared. 0 Disable 1 Enable |

## Half SPORT 'B' Multichannel Control Register

The SPORT\_MCTL\_B register controls the half SPORT's multichannel operations. This register enables multichannel operation, enables multichannel data packing, selects the multichannel frame delay, selects the number of multichannel slots, and selects the multichannel window offset size.

Figure 32-31: SPORT\_MCTL\_B Register Diagram

<!-- image -->

Table 32-32: SPORT\_MCTL\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:16 (R/W)        | WOFFSET    | Window Offset. The SPORT_MCTL_B.WOFFSET bits select the start location for the half SPORT's active window of channels within the 1024-channel range. A value of 0 specifies no offset and 896 is the largest value that permits using all 128 channels. When multi- channel mode is disabled ( SPORT_MCTL_B.MCE =0) and right-justified mode is en- abled ( SPORT_CTL_B.RJUST =1), the least significant 6 bits of SPORT_MCTL_B.WOFFSET serve as the delay count (DCNT) field. These bits in- troduce a clock delay before the half SPORT state machine starts to capture data. For information about appropriate delay selections, see the SPORT operating modes sec- tion. |
| 14:8 (R/W)         | WSIZE      | Window Size. The SPORT_MCTL_B.WSIZE bits select the window size for the half SPORT's ac- tive window of channels. Use the following formula to calculate the window size value: SPORT_MCTL_B.WSIZE = (number of channel slots) -1                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 7:4 (R/W)          | MFD        | Multichannel Frame Delay. The SPORT_MCTL_B.MFD bits select the delay (in serial clock cycles) between the half SPORT's multichannel frame sync pulse and channel 0. The 4-bit field allows se- lecting a multichannel frame delay of 0-15 serial clocks.                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 32-32: SPORT\_MCTL\_B Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | MCPDE      | Multichannel Packing DMAEnable. The SPORT_MCTL_B.MCPDE bit enables DMAdata packing for transmit and ena- bles DMAdata unpacking for the half SPORT's multichannel data transfers. 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 0 (R/W)            | MCE        | 1 Enable Multichannel Enable. The SPORT_MCTL_B.MCE bit enables multichannel operations for the half SPORT. The half SPORT is configured in normal multichannel mode if SPORT_CTL_B.OPMODE =0; while it is configured in packed mode if SPORT_CTL_B.OPMODE =1. When configuring in these modes, the multichannel enable bit ( SPORT_MCTL_B.MCE ) should be set before enabling SPORT data chan- nel enable bits ( SPORT_CTL_B.SPENPRI and/or SPORT_CTL_B.SPENSEC ). When these channel bits transition from 1 to 0, note that the half SPORT's data trans- fer buffers are cleared, and the SPORT_CTL_B.DERRPRI and SPORT_CTL_B.DERRSEC bits are cleared. 0 Disable Enable |
| 0 (R/W)            |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 0 (R/W)            |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

## Half SPORT 'A' Multichannel Status Register

The SPORT\_MSTAT\_A register indicates the current multichannel being serviced among the half SPORT's active channels in multichannel mode. The half SPORT increments the value by one in this register as each channel is serviced. The value in the SPORT\_MSTAT\_A register restarts at 0 at each frame sync.

Figure 32-32: SPORT\_MSTAT\_A Register Diagram

<!-- image -->

Table 32-33: SPORT\_MSTAT\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------|
| 9:0 (R/NW)         | CURCHAN    | Current Channel. The SPORT_MSTAT_A.CURCHAN bits indicate the half SPORT's current channel being serviced in multichannel mode. |

## Half SPORT 'B' Multichannel Status Register

The SPORT\_MSTAT\_B register indicates the current multichannel being serviced among the half SPORT's active channels in multichannel mode. The half SPORT increments the value by one in this register as each channel is serviced. The value in the SPORT\_MSTAT\_B register restarts at 0 at each frame sync.

Figure 32-33: SPORT\_MSTAT\_B Register Diagram

<!-- image -->

Table 32-34: SPORT\_MSTAT\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------|
| 9:0 (R/NW)         | CURCHAN    | Current Channel. The SPORT_MSTAT_B.CURCHAN bits indicate the half SPORT's current channel being serviced in multichannel mode. |

## Half SPORT 'A' Rx Buffer (Primary) Register

The SPORT\_RXPRI\_A register buffers the half SPORT's primary channel receive data. This buffer becomes active when the half SPORT is configured to receive data on the primary channel. After a complete word has been received in the receive shifter, it is placed into the SPORT\_RXPRI\_A register. This data can be read in core mode (in interrupt-based or polling-based mechanism) or directly transferred into processor memory using the DMA controller. With a data buffer and an input shift register, the SPORT\_RXPRI\_A register acts as a two-location buffer. So, the SPORT can keep a maximum of two 32-bit received words at any given time (independent of the SPORT\_CTL\_A.PACK bit setting).

Figure 32-34: SPORT\_RXPRI\_A Register Diagram

<!-- image -->

Table 32-35: SPORT\_RXPRI\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Receive Buffer (Primary). The SPORT_RXPRI_A.VALUE bits hold the half SPORT's primary channel receive data. Note that changes to the half SPORT operation mode (for example, toggling the SPORT_MCTL_A.MCE ) empty the contents of this data buffer. For more informa- tion, see the SPORT_CTL_A and SPORT_MCTL_A register descriptions. |

## Half SPORT 'B' Rx Buffer (Primary) Register

The SPORT\_RXPRI\_B register buffers the half SPORT's primary channel receive data. This buffer becomes active when the half SPORT is configured to receive data on the primary channel. After a complete word has been received in the receive shifter, it is placed into the SPORT\_RXPRI\_B register. This data can be read in core mode (in interrupt-based or polling-based mechanism) or directly transferred into processor memory using the DMA controller. With a data buffer and an input shift register, the SPORT\_RXPRI\_B register acts as a two-location buffer. So, the SPORT can keep a maximum of two 32-bit received words at any given time (independent of the SPORT\_CTL\_A.PACK bit setting).

Figure 32-35: SPORT\_RXPRI\_B Register Diagram

<!-- image -->

Table 32-36: SPORT\_RXPRI\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Receive Buffer (Primary). The SPORT_RXPRI_B.VALUE bits hold the half SPORT's primary channel receive data. Note that changes to the half SPORT operation mode (for example, toggling the SPORT_MCTL_B.MCE ) empty the contents of this data buffer. For more informa- tion, see the SPORT_CTL_B and SPORT_MCTL_B register descriptions. |

## Half SPORT 'A' Rx Buffer (Secondary) Register

The SPORT\_RXSEC\_A register buffers the half SPORT's secondary channel receive data. This buffer becomes active when the half SPORT is configured to receive data on the secondary channel. After a complete word has been received in the receive shifter, it is placed into the SPORT\_RXSEC\_A register. This data can be read in core mode (in interrupt-based or polling-based mechanism) or directly transferred into processor memory using the DMA controller. With a data buffer and an input shift register, the SPORT\_RXSEC\_A register acts as a two-location buffer. So, the SPORT can keep a maximum of two 32-bit received words at any given time (independent of the SPORT\_CTL\_A.PACK bit setting).

Figure 32-36: SPORT\_RXSEC\_A Register Diagram

<!-- image -->

Table 32-37: SPORT\_RXSEC\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Receive Buffer (Secondary). The SPORT_RXSEC_A.VALUE bits hold the half SPORT's secondary channel re- ceive data. Note that changes to the half SPORT operation mode (for example, tog- gling the SPORT_MCTL_A.MCE ) empty the contents of this data buffer. For more information, see the SPORT_CTL_A and SPORT_MCTL_A register descriptions. |

## Half SPORT 'B' Rx Buffer (Secondary) Register

The SPORT\_RXSEC\_B register buffers the half SPORT's secondary channel receive data. This buffer becomes active when the half SPORT is configured to receive data on the secondary channel. After a complete word has been received in the receive shifter, it is placed into the SPORT\_RXSEC\_B register. This data can be read in core mode (in interrupt-based or polling-based mechanism) or directly transferred into processor memory using the DMA controller. With a data buffer and an input shift register, the SPORT\_RXSEC\_B register acts as a two-location buffer. So, the SPORT can keep a maximum of two 32-bit received words at any given time (independent of the SPORT\_CTL\_A.PACK bit setting).

Figure 32-37: SPORT\_RXSEC\_B Register Diagram

<!-- image -->

Table 32-38: SPORT\_RXSEC\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Receive Buffer (Secondary). The SPORT_RXSEC_B.VALUE bits hold the half SPORT's secondary channel re- ceive data. Note that changes to the half SPORT operation mode (for example, tog- gling the SPORT_MCTL_B.MCE ) empty the contents of this data buffer. For more information, see the SPORT_CTL_B and SPORT_MCTL_B register descriptions. |

## Half SPORT 'A' Tx Buffer (Primary) Register

The SPORT\_TXPRI\_A register buffers the half SPORT's primary channel transmit data. This register must be loaded with the data to be transmitted if the half SPORT is configured to transmit on the primary channel. Either a program running on the processor core loads the data into the buffer (word-by-word process) or the DMA controller automatically loads the data into the buffer (DMA process).

The SPORT\_TXPRI\_A register acts as a three-location buffer if SPORT data packing is disabled ( SPORT\_CTL\_A.PACK =0); while it acts as a two-location buffer when packing is enabled ( SPORT\_CTL\_A.PACK =1). So, depending on the PACK bit setting, two 32-bit words or three 32-bit words can be stored in the transmit queue at any time. When the transmit register is loaded and any previous word has been transmitted, the SPORT\_TXPRI\_A register contents are automatically loaded into the output shifter. The half SPORT can issue an interrupt request (transmit buffer is not full) when it has loaded the output transmit shifter, signifying that the transmit buffer is ready to accept the next word. This interrupt request does not occur when the half SPORT is executing a DMA-based transfer.

Figure 32-38: SPORT\_TXPRI\_A Register Diagram

<!-- image -->

Table 32-39: SPORT\_TXPRI\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Transmit Buffer (Primary). The SPORT_TXPRI_A.VALUE bits hold the half SPORT's primary channel trans- mit data. Note that changes to the half SPORT operation mode (for example, toggling the SPORT_MCTL_A.MCE ) empty the contents of this data buffer. For more infor- mation, see the SPORT_CTL_A and SPORT_MCTL_A register descriptions. |

## Half SPORT 'B' Tx Buffer (Primary) Register

The SPORT\_TXPRI\_B register buffers the half SPORT's primary channel transmit data. This register must be loaded with the data to be transmitted if the half SPORT is configured to transmit on the primary channel. Either a program running on the processor core loads the data into the buffer (word-by-word process) or the DMA controller automatically loads the data into the buffer (DMA process).

The SPORT\_TXPRI\_B register acts as a three-location buffer if SPORT data packing is disabled ( SPORT\_CTL\_B.PACK =0); while it acts as a two-location buffer when packing is enabled ( SPORT\_CTL\_B.PACK =1). So, depending on the PACK bit setting, two 32-bit words or three 32-bit words can be stored in the transmit queue at any time. When the transmit register is loaded and any previous word has been transmitted, the SPORT\_TXPRI\_B register contents are automatically loaded into the output shifter. The half SPORT can issue an interrupt request (transmit buffer is not full) when it has loaded the output transmit shifter, signifying that the transmit buffer is ready to accept the next word. This interrupt request does not occur when the half SPORT is executing a DMA-based transfer.

Figure 32-39: SPORT\_TXPRI\_B Register Diagram

<!-- image -->

Table 32-40: SPORT\_TXPRI\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Transmit Buffer (Primary). The SPORT_TXPRI_B.VALUE bits hold the half SPORT's primary channel trans- mit data. Note that changes to the half SPORT operation mode (for example, toggling the SPORT_MCTL_B.MCE ) empty the contents of this data buffer. For more infor- mation, see the SPORT_CTL_B and SPORT_MCTL_B register descriptions. |

## Half SPORT 'A' Tx Buffer (Secondary) Register

The SPORT\_TXSEC\_A register buffers the half SPORT's secondary channel transmit data. This register must be loaded with the data to be transmitted if the half SPORT is configured to transmit on the secondary channel. Either a program running on the processor core loads the data into the buffer (word-by-word process) or the DMA controller automatically loads the data into the buffer (DMA process).

The SPORT\_TXSEC\_A register acts as a three-location buffer if SPORT data packing is disabled ( SPORT\_CTL\_A.PACK =0); while it acts as a two-location buffer when packing is enabled ( SPORT\_CTL\_A.PACK =1). So, depending on the PACK bit setting, two 32-bit words or three 32-bit words can be stored in the transmit queue at any time. When the transmit register is loaded and any previous word has been transmitted, the SPORT\_TXSEC\_A register contents are automatically loaded into the output shifter. The half SPORT can issue an interrupt request (transmit buffer is not full) when it has loaded the output transmit shifter, signifying that the transmit buffer is ready to accept the next word. This interrupt request does not occur when the half SPORT is executing a DMA-based transfer.

Figure 32-40: SPORT\_TXSEC\_A Register Diagram

<!-- image -->

Table 32-41: SPORT\_TXSEC\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Transmit Buffer (Secondary). The SPORT_TXSEC_A.VALUE bits hold the half SPORT's secondary channel transmit data. Note that changes to the half SPORT operation mode (for example, toggling the SPORT_MCTL_A.MCE ) empty the contents of this data buffer. For more information, see the SPORT_CTL_A and SPORT_MCTL_A register descriptions. |

## Half SPORT 'B' Tx Buffer (Secondary) Register

The SPORT\_TXSEC\_B register buffers the half SPORT's secondary channel transmit data. This register must be loaded with the data to be transmitted if the half SPORT is configured to transmit on the secondary channel. Either a program running on the processor core loads the data into the buffer (word-by-word process) or the DMA controller automatically loads the data into the buffer (DMA process).

The SPORT\_TXSEC\_B register acts as a three-location buffer if SPORT data packing is disabled ( SPORT\_CTL\_B.PACK =0); while it acts as two-location buffer when packing is enabled ( SPORT\_CTL\_B.PACK =1). So, depending on the PACK bit setting, two 32-bit words or three 32-bit words can be stored in the transmit queue at any time. When the transmit register is loaded and any previous word has been transmitted, the SPORT\_TXSEC\_B register contents are automatically loaded into the output shifter. The half SPORT can issue an interrupt request (transmit buffer is not full) when it has loaded the output transmit shifter, signifying that the transmit buffer is ready to accept the next word. This interrupt request does not occur when the half SPORT is executing a DMA-based transfer.

Figure 32-41: SPORT\_TXSEC\_B Register Diagram

<!-- image -->

Table 32-42: SPORT\_TXSEC\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Transmit Buffer (Secondary). The SPORT_TXSEC_B.VALUE bits hold the half SPORT's secondary channel transmit data. Note that changes to the half SPORT operation mode (for example, toggling the SPORT_MCTL_B.MCE ) empty the contents of this data buffer. For more information, see the SPORT_CTL_B and SPORT_MCTL_B register descriptions. |