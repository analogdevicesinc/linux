## 18   Enhanced Parallel Peripheral Interface (EPPI)

The Enhanced Parallel Peripheral Interface (EPPI) is a half-duplex, bidirectional port with a dedicated clock pin and three frame sync (FS) pins. It can support direct connections to active TFT LCDs, parallel A/D and D/A converters, video encoders and decoders, image sensor modules and other general-purpose peripherals. Each EPPI has two DMA channels associated with it. Moreover, in some modes, an EPPI can use an extra DMA channel.

## EPPI Features

The EPPI module supports the following features.

- Programmable data length from 8 bits to up to 24 bits per clock cycle (depending on the product model)
- Bidirectional and half-duplex port
- Internal or external clock source
- Clock gating by an external device asserting the clock gating control signal
- Various framed and non-framed operating modes, as well as internal or external frame syncs
- Various general-purpose modes with 0, 1, 2, and 3 frame sync modes for both receive and transmit
- Ignores premature external frame syncs for data consistency
- SMPTE274M and SMPTE 296M high definition format support
- ITU-656, SMPTE 296M and SMPTE 274M status word error detection and correction for ITU-656 receive modes
- ITU-656, SMPTE 296M and SMPTE 274M receive modes - active video only, vertical blanking only, and entire field
- ITU-656, SMPTE 296M and SMPTE 274M preamble and status word decode
- Optional packing and unpacking of data to or from 32 bits from or to 8, 16 bits and 24 bits. If packing or unpacking is enabled, endianness can be altered to change the order of packing or unpacking of bytes/words
- Optional sign extension or zero-fill and alternate even or odd data sample filter for receive modes

- RGB888 to RGB666 or RGB565 conversion for transmit modes
- 4:2:2 YCrCb data Tx/Rx interleaving or de-interleaving modes
- Configurable LCD data enable (DEN) output available on frame sync 3
- Delayed start of PPI frame syncs
- Data clipping and mirroring
- Horizontal and vertical windowing for general purpose 2 and 3 frame sync modes
- Preamble, blanking and stripping support
- Multiplexing dual input

## EPPI Functional Description

The EPPI has the following functionality.

## RGB data formats

For transmit modes, the EPPI can convert RGB888 data in memory to either RGB565 or RGB666 at the output using bits in the Control register.

## Data clipping

The EPPI contains two registers to define the lower and upper limits for the Luma and Chroma components. This functionality is used for clipping data values during 8-bit, 10-bit, 12-bit or 16-bit transmit modes.

## Data mirroring

A data mirroring feature is available which mirrors the EPPI data bits 15-0. This functionality is available in both transmit and receive modes.

## Windowing

The EPPI supports windowing for general-purpose input modes.

## Preamble, blanking and stripping support

The EPPI can embed blanking information and clip active data to be transmitted. This functionality is available for single channel data, interleaved data, and parallel data and supports data lengths equal to 16 bits, 20 bits or 24 bits.

## ADSP-SC58x EPPI Register List

The EPPI is a half-duplex, bidirectional parallel port. It comprises a clock pin, 3 frame sync pins, and a set of data pins. For more information on EPPI functionality, see the EPPI register descriptions.

Table 18-1: ADSP-SC58x EPPI Register List

| Name           | Description                                                             |
|----------------|-------------------------------------------------------------------------|
| EPPI_CLKDIV    | Clock Divide Register                                                   |
| EPPI_CTL       | Control Register                                                        |
| EPPI_CTL2      | Control Register 2 Register                                             |
| EPPI_EVENCLIP  | Clipping Register for EVEN (Luma) Data Register                         |
| EPPI_FRAME     | Lines Per Frame Register                                                |
| EPPI_FS1_DLY   | Frame Sync 1 Delay Value Register                                       |
| EPPI_FS1_PASPL | FS1 Period Register / EPPI Active Samples Per Line Register             |
| EPPI_FS1_WLHB  | FS1 Width Register / EPPI Horizontal Blanking Samples Per Line Register |
| EPPI_FS2_DLY   | Frame Sync 2 Delay Value Register                                       |
| EPPI_FS2_PALPF | FS2 Period Register / EPPI Active Lines Per Field Register              |
| EPPI_FS2_WLVB  | FS2 Width Register / EPPI Lines Of Vertical Blanking Register           |
| EPPI_HCNT      | Horizontal Transfer Count Register                                      |
| EPPI_HDLY      | Horizontal Delay Count Register                                         |
| EPPI_IMSK      | Interrupt Mask Register                                                 |
| EPPI_LINE      | Samples Per Line Register                                               |
| EPPI_ODDCLIP   | Clipping Register for ODD(Chroma) Data Register                         |
| EPPI_STAT      | Status Register                                                         |
| EPPI_VCNT      | Vertical Transfer Count Register                                        |
| EPPI_VDLY      | Vertical Delay Count Register                                           |

## ADSP-SC58x EPPI Interrupt List

Table 18-2: ADSP-SC58x EPPI Interrupt List

|   Interrupt ID | Name              | Description              | Sensitivity   |   DMA Channel |
|----------------|-------------------|--------------------------|---------------|---------------|
|             81 | EPPI0_CH0_DMA     | EPPI0 Channel0DMA        | Level         |            28 |
|             82 | EPPI0_CH1_DMA     | EPPI0 Channel1DMA        | Level         |            29 |
|             83 | EPPI0_STAT        | EPPI0 Status             | Level         |               |
|            215 | EPPI0_CH0_DMA_ERR | EPPI0 DMAChannel 0 Error |               |               |

Table 18-2: ADSP-SC58x EPPI Interrupt List (Continued)

|   Interrupt ID | Name              | Description              | Sensitivity   | DMA Channel   |
|----------------|-------------------|--------------------------|---------------|---------------|
|            216 | EPPI0_CH1_DMA_ERR | EPPI0 DMAChannel 1 Error |               |               |

## ADSP-SC58x EPPI Trigger List

Table 18-3: ADSP-SC58x EPPI Trigger List Masters

|   Trigger ID | Name          | Description       | Sensitivity   |
|--------------|---------------|-------------------|---------------|
|           62 | EPPI0_CH0_DMA | EPPI0 Channel0DMA | Edge          |
|           63 | EPPI0_CH1_DMA | EPPI0 Channel1DMA | Edge          |

Table 18-4: ADSP-SC58x EPPI Trigger List Slaves

|   Trigger ID | Name          | Description       | Sensitivity   |
|--------------|---------------|-------------------|---------------|
|           46 | EPPI0_CH0_DMA | EPPI0 Channel0DMA | Pulse         |
|           47 | EPPI0_CH1_DMA | EPPI0 Channel1DMA | Pulse         |

## RGB Data Formats

For transmit modes, the EPPI can convert RGB888 data in memory to RGB666 at the output when the EPPI\_CTL.RGBFMTEN bit is set and the EPPI\_CTL.DLEN value is equal to 18 bits. Similarly, the EPPI can convert RGB888 data in memory to RGB565 at the output when the EPPI\_CTL.RGBFMTEN bit is set and EPPI\_CTL.DLEN is equal to 16 bits.

This conversion is performed as follows:

- If EPPI\_CTL.PACKEN =1, the EPPI first unpacks according to the EPPI\_CTL.SWAPEN bit setting, and the three 32-bit data words from the DMA are broken into four 24-bit data words to be transmitted out, as described earlier.
- If EPPI\_CTL.PACKEN =0, the EPPI takes the lower 24 bits of the 32-bit DMA as the data to be transmitted. Then, the EPPI truncates this 24-bit data word to the required data width. It removes the lower 2 bits of G and the lower 2 bits or 3 bits of R and B.

## Data Clipping

The EPPI contains two registers to define the lower and upper limits for the Luma and Chroma components. It uses these registers for clipping data values during 8-bit, 10-bit, 12-bit or 16-bit transmit modes. All data values for odd samples which are less than the value in the EPPI\_ODDCLIP.LOWODD bit field are replaced with the value in the EPPI\_ODDCLIP.LOWODD field. All data values for even samples which are less than the value in the EPPI\_EVENCLIP.LOWEVEN field are replaced with the value in the EPPI\_EVENCLIP.LOWEVEN field.

In the same manner, all data values for odd samples which are more than the value in the

EPPI\_ODDCLIP.HIGHODD bit field are replaced with the value in the EPPI\_ODDCLIP.HIGHODD field. All data values for even samples which are more than the values in the EPPI\_EVENCLIP.HIGHEVEN field are replaced with the values in the EPPI\_EVENCLIP.HIGHEVEN field.

Depending on the programmed EPPI length, only the corresponding bits (least aligned) are considered for clipping. For example, if the EPPI is programmed in 10-bit mode, bits 9:0 and bits 25:16 constitute the clipping thresholds. The higher bits are ignored. The EPPI supports 8-bit, 10-bit, 12-bit, and 16-bit clipping thresholds.

For the 4:2:2 YCrCb color space, Luma and Chroma typically have different lower and upper thresholds. Separate thresholds can be required for even and odd data samples. For monochrome (Y only) or some non-video clipping applications, the value in the EPPI\_ODDCLIP.LOWODD field can be the same as the value in the EPPI\_EVENCLIP.LOWEVEN field. The value in the EPPI\_ODDCLIP.HIGHODD field can be the same as the value in the EPPI\_EVENCLIP.HIGHEVEN field.

In GP 0 FS mode with internal blanking generation, clipping is valid only for the active video part of the transmitted data. ITU-R 656 preambles, status words, and blanking data bypass the clipping logic.

If the EPPI is programmed in 16, 20-bit or 24-bit mode with the EPPI\_CTL.SPLTWRD bit set, the YDATA (luma data) gets the clipping threshold levels of the EPPI\_EVENCLIP register. The CDATA (chroma data) gets the clipping threshold levels of the EPPI\_ODDCLIP register.

The clipping registers are ignored when the EPPI\_CTL.RGBFMTEN bit is set.

## Data Mirroring

To increase the pin multiplexing options for the EPPI data pins, a data mirroring feature is available which mirrors the EPPI data bits 15:0. This feature is available in both transmit and receive modes. It is enabled by setting the EPPI\_CTL.DMIRR bit.

Figure 18-1: Data Mirroring Receive

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000000_77b0353116fce28a8b50d1cbf57ecbe475ca04b3743796985bf0a39b25ad17b2.png)

Figure 18-2: Data Mirroring Transmit

## Windowing

The EPPI supports windowing for general-purpose input modes. The module can be configured to bring in a region of interest instead of the entire frame of data, which helps reduce bandwidth requirements.

## Preamble, Blanking and Stripping Support

The EPPI supports embedding blanking information and clipping of active data for transmission. This functionality is available for single-channel data, interleaved data, and parallel data and supports a data length ( EPPI\_CTL.DLEN ) of 16, 20 bits or 24 bits.

Support of preamble generation or detection and stripping of blanking information is also provided for ITU656 mode and the SMPTE 274M and 296M HD formats. The Video Mode Comparison table shows the SMPTE standards in comparison with the ITU656 modes. Preambles for SMPTE and ITU656 modes are identical. Extension of preamble to 12 bits is also supported.

| Video Mode    |   Frame Rate | Frame Resolution   | Active Video Resolu- tion   |   Sampling Frequency (MHz) | Remarks         |
|---------------|--------------|--------------------|-----------------------------|----------------------------|-----------------|
| ITU656 (NTSC) |           30 | 1716x525           | 720x480                     |                      27.02 | Y-C interleaved |
| ITU656 (PAL)  |           25 | 1728x625           | 720x576                     |                      27.00 | Y-C interleaved |
| SMPTE 296M    |           30 | 3300x750           | 1280x720                    |                      74.25 | Y,C separate    |
|               |           60 | 1650x750           | 1280x720                    |                      74.25 | Y,C separate    |

Table 18-5: Video Mode Comparison

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000001_56edb2ab6cb807950163e27de6729ef0d9311238b63f8fab1e678f7f9194d748.png)

Table 18-5: Video Mode Comparison (Continued)

| Video Mode   |   Frame Rate | Frame Resolution   | Active Video Resolu- tion   |   Sampling Frequency (MHz) | Remarks      |
|--------------|--------------|--------------------|-----------------------------|----------------------------|--------------|
| SMPTE 274M   |           30 | 2200x1125          | 1920x1080                   |                      74.25 | Y,C separate |
| SMPTE 274M   |           60 | 2200x1125          | 1920x1080                   |                     148.50 | Y,C separate |
| SMPTE 274M   |           25 | 2640x1125          | 1920x1080                   |                      74.25 | Y,C separate |
| SMPTE 274M   |           50 | 2640x1125          | 1920x1080                   |                     148.50 | Y,C separate |
| SMPTE 274M   |           24 | 2750x1125          | 1920x1080                   |                      74.25 | Y,C separate |

See the clock operating conditions section of the data sheet for the maximum sampling frequency for this product.

## EPPI Definitions

The following definitions are helpful when using the EPPI module.

## ITU-R BT.-656

Description of a digital video protocol for interfaces and data stream format required to send uncompressed PAL or NTSC standard definition TV (525 or 625 lines) signals.

## YUV422

YUV is a color space where luminance (Y) and chrominance (UV) components define the pixels. The suffix signifies how the chrominance components have been decimated and provide formatting information. In this case, the YUV422 format has the chrominance decimated by two, meaning only half of each chrominance component is available. Typical YUV422 formatting interleaves the luminance and chrominance (for example, U1Y1V1Y2U2Y3V2Y4).

## RGB888

RGB is a color space where three color values, one red (R), one green (G) and one blue (B), define the pixels. The suffix signifies the bit widths for these color components. In this case, RGB888 means that each red, green, and blue value is 8 bits.

## RGB565

RGB is a color space where three color values, one red (R), one green (G) and one blue (B) define the pixels. The suffix signifies the bit widths for these color components. In this case, RGB565 means that the red (R) and blue (B) are 5 bits each while the green (G) is 6 bits. When packed together, each RGB565 pixel can be represented in a 16bit data word. LCD display panels commonly use this format.

## SMPTE 274M

An HD standard defining the spatial resolution (image sample structure) and frame rates for 1920x1080.

## SMPTE 296M

An HD standard for defining the spatial resolution (image sample structure) and frame rates fro 1280x720.

## EPPI Block Diagram

The EPPI Block Diagram figure shows the functional blocks within the EPPI.

Figure 18-3: EPPI Block Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000002_7abcc8103a1b6edda9231f60123d281c607edaa9db8e4bb86f826b6b1b67962a.png)

## EPPI Architectural Concepts

The following sections describe the architectural concepts.

- EPPI Interface
- Reset Operation
- Frame Sync Polarity and Sampling Edge
- Direct Memory Access (DMA)
- EPPI Clock

## EPPI Interface

A block diagram of the architecture for the EPPI interface is shown in the EPPI DMA Interface figure.

Figure 18-4: EPPI DMA Interface

Figure 18-5: Receive Data Path

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000003_6a9406bdd2a6dc7d001ee75b1db0fd47b77eb92d4d353d89e1b4999292ac3e5d.png)

Figure 18-6: Transmit Data Path

## Reset Operation

On a hardware reset, the entire EPPI is reset. All MMRs return to their default values. EPPI interrupt and DMA requests become inactive and internally generated EPPI\_CLK and frame syncs are aborted.

In software, write 0 to the EPPI\_CTL.EN bit to reset and reconfigure the EPPI. When disabled in this manner, only the EPPI\_STAT register is cleared to its reset value. Interrupts and DMA requests become inactive and internally generated clock and frame syncs are aborted.

## Frame Sync Polarity and Sampling Edge

The EPPI\_CTL.POLS and EPPI\_CTL.POLC bits provide a mechanism to select the active level of the frame syncs and the sampling or driving edge of the EPPI clock, respectively. This functionality allows the EPPI to connect to data sources and receivers with a wide array of control signal polarities. Often, the remote data source or receiver also offer configurable signal polarities. In these cases, the EPPI\_CTL.POLS and EPPI\_CTL.POLC bits add flexibility.

| Bit Setting   | Frame Sync 2   | Frame Sync 1   |
|---------------|----------------|----------------|
| POLS = b#00   | Active high    | Active high    |
| POLS = b#01   | Active high    | Active low     |
| POLS = b#10   | Active low     | Active high    |
| POLS = b#11   | Active low     | Active low     |

Table 18-6: Frame Sync Polarity Selections and Frame Sync Pin States

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000004_e0a9bdd536da5c19b909fc7a5d8315582726fed79e9ec96c2827d46f2ea49fbe.png)

Table 18-7: Clock Polarity Selections and Receive/Transmit Pin States

|             | Receive      | Receive            | Transmit     | Transmit           |
|-------------|--------------|--------------------|--------------|--------------------|
| Bit Setting | Sample Data  | Sample/Drive Syncs | Drive Data   | Sample/Drive Syncs |
| POLC = b#00 | Falling edge | Falling edge       | Rising edge  | Rising edge        |
| POLC = b#01 | Falling edge | Rising edge        | Rising edge  | Falling edge       |
| POLC = b#10 | Rising edge  | Falling edge       | Falling edge | Rising edge        |
| POLC = b#11 | Rising edge  | Rising edge        | Falling edge | Falling edge       |

## Direct Memory Access (DMA)

The EPPI has a native DMA controller with two channels. A local arbiter arbitrates between these channels and requests are forwarded to the system crossbar. The EPPI has one connection to the fabric.

Figure 18-7: EPPI DMA Interface

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000005_eecdfd80bfef813f36983fa339d901ae9ae00cf59563a50144d2e838fd8d881b.png)

The EPPI must be used with DMA. Configuring the EPPI DMA channels is a necessary step toward using the EPPI interface. The channels can be configured for either transmit or receive operation, and have a maximum throughput of ( EPPI\_CLK ) x (32 bits/transfer). In modes where data lengths permit, packing can increase transfer bandwidth.

The DMA engine generates interrupts at the completion of a row, frame, or partial-frame transfer. The DMA engine also coordinates the source or destination point for the data that is transferred through the EPPI.

The 2D DMA capability allows the processor to be interrupted at the end of a line or after a frame of video is transferred, or if a DMA error occurs. The DMA\_XCNT and DMA\_YCNT registers allow for flexible data interrupt points. For example, assume the DMA\_XMOD = DMA\_YMOD =1. If a data frame contains 320 × 240 bytes (240 rows of 320 bytes each), the following conditions hold.

- Setting DMA\_XCNT =320, DMA\_YCNT = 240, and DMA\_CFG.INT =1 interrupts on every row transferred, for the entire frame.
- Setting DMA\_XCNT =320, DMA\_YCNT =240, and DMA\_CFG.INT =2 interrupts only on the completion of the frame (when 240 rows of 320 bytes have been transferred).

- Setting DMA\_XCNT =38,400 (320 x 120), DMA\_YCNT =2, and DMA\_CFG.INT =1 causes an interrupt when half of the frame is transferred, and again when the whole frame is transferred.

The following is the general procedure for setting up DMA operation with the EPPI.

1. Configure the DMA registers as appropriate for the desired DMA operating mode.
2. Enable the DMA channel for operation.
3. Configure appropriate EPPI registers.
4. Enable the EPPI by writing 1 to the EPPI\_CTL.EN bit.

## EPPI Clock

The EPPI can be supplied with an external clock, or the clock can be generated internally and supplied to external devices. For information on the maximum PPI\_CLK specification in internal and external clock modes, see the product-specific data sheet.

When using an external EPPI\_CLK , there can be up to two cycles latency before valid data is received or transmitted.

The internal clock can be generated from SCLK1\_0 when the EPPI\_CTL.ICLKGEN bit is set. The value in the EPPI\_CLKDIV register determines the generated clock frequency. The internally generated EPPI clock frequency is:

f PCLK = f SCLK0 /( EPPI\_CLKDIV + 1)

where:

f PCLK - frequency of internally generated EPPI clock f SCLK - frequency of SCLK1\_0

EPPI\_CLKDIV - Clock division value programmed in the EPPI\_CLKDIV register.

The Relationship Between CLKDIV and the Ratio of SCLK0 to EPPI Clock table gives a few examples.

Table 18-8: Relationship Between CLKDIV and the Ratio of SCLK0 to EPPI Clock

| CLKDIV15-0   | EPPI/SCLK0 Clock Ratio   |
|--------------|--------------------------|
| 0x0002       | 1:3                      |
| 0x0003       | 1:4                      |
| 0x0004       | 1:5                      |
| 0x0005       | 1:6                      |
| ...          | ...                      |

## EPPI Operating Modes

The EPPI supports various receive and transmit modes of operation which include the detection and generation of preamble data. Specifically, the EPPI supports data formats described in the specifications ITU656, SMPTE 274M and SMPTE 296M. In addition to these modes, the EPPI also supports general-purpose receive and transmit using up to three frame syncs (FS).

The control register ( EPPI\_CTL ) includes most of the bits used for configuring operating modes. The 'Register Descriptions' section of this chapter provides complete descriptions of these bits.

## ITU-R 656 Modes

The EPPI supports three input modes and one output mode for ITU-R 656 framed data. This section describes these modes.

## ITU-R 656 Background

In ITU-R 656 mode, the horizontal (H), vertical (V), and field (F) signals are sent as an embedded part of the video data stream. The signals are sent in a series of bytes that form a control word. ITU-R 656 was formerly known as CCIR-656.

The letter H is used to distinguish between the start of active video (SAV) and end of active video (EAV) signals. These signals indicate the beginning and end of active video data in each line. The SAV occurs on a 1-to-0 transition of H, and EAV occurs on a 0-to-1 transition of H. The space between EAV and SAV is filled with horizontal blanking data. Therefore, H = 1 during the horizontal blanking portion of the data stream, and H = 0 during the active video portion of the data stream.

The letter V is used to denote the vertical blanking portion of the data stream. A transition in V can occur only in the EAV sequence. When V = 1, the data stream contains vertical blanking data, and when V = 0, the data stream contains active video data.

The letter F is used to distinguish Field 1 from Field 2. Interlaced video has two fields in a frame of data. It requires each field to be handled uniquely, and alternate rows of each field combined to create the actual video image.

For interlaced video, F = 0 represents Field 1 ( Odd Field ) and F = 1 represents Field 2 ( Even Field ). Progressive video makes no distinction between Field 1 and Field 2, and F is always 0 for progressive video. Interlaced video requires each field to be handled uniquely because alternate rows of each field combine to create the actual video image.

An entire field of video is comprised of active video plus horizontal blanking (the space between an EAV and SAV code) and vertical blanking (the space where V = 1). A field of video commences on a transition of the F bit.

Figure 18-8: Typical Video Frame Partitioning for NTSC/PAL Systems in Interlaced and Progressive ITU-R BT.656 Systems

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000006_19c9b4a7126772d084d6e9436c4ac13e98c8cab21db3531cfe7622703322883d.png)

Figure 18-9: ITU-R 656 8-Bit Parallel Data Stream from NTSC (PAL) Systems

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000007_7a5767e36497519924727514fdec81e099b4b0f04e7544e757df7a58af60ba0f.png)

NOTE: Refer to the Control Sequences for 8-Bit and 10-Bit ITU-R 656 Video table. There is a defined preamble of three data elements (for example, in the case of 8-bit video: 0xFF , 0x00, 0x00), followed by the XY status word. The status word contains four protection bits for error detection and correction excluding the F (field), V (vertical blanking), and H (horizontal blanking) bits. F and V are only allowed to change as part of EAV sequences (that is, transition from H = 0 to H = 1).

The bit definitions are as follows:

- F = 0 for field 1
- F = 1 for field 2

- V = 1 during vertical blanking
- V = 0 when not in vertical blanking
- H = 0 at SAV
- H = 1 at EAV
- P3 = V XOR H
- P2 = F XOR H
- P1 = F XOR V
- P0 = F XOR V XOR H

P3-P0 are protection bits that enable 1-bit and 2-bit error detection, and 1-bit error correction at the receiver. The EPPI corrects the error if it detects 1-bit errors in F , V , or H. Errors in the protection bits themselves are detected but not corrected.

Table 18-9: Control Sequences for 8-Bit and 10-Bit ITU-R 656 Video

|              | 8-Bit Data   | 8-Bit Data   | 8-Bit Data   | 8-Bit Data   | 8-Bit Data   | 8-Bit Data   | 8-Bit Data   | 8-Bit Data   | 10-Bit Data   | 10-Bit Data   |
|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|---------------|---------------|
|              | D9 (MSB      | D8           | D7           | D6           | D5           | D4           | D3           | D2           | D1            | D0            |
| Preamble     | 1            | 1            | 1            | 1            | 1            | 1            | 1            | 1            | 1             | 1             |
|              | 0            | 0            | 0            | 0            | 0            | 0            | 0            | 0            | 0             | 0             |
|              | 0            | 0            | 0            | 0            | 0            | 0            | 0            | 0            | 0             | 0             |
| Control Byte | 1            | F            | V            | H            | P3           | P2           | P1           | P0           | 0             | 0             |

The EPPI\_STAT register contains 2 bits, EPPI\_STAT.ERRDET and EPPI\_STAT.ERRNCOR , that are used to report the status of error detected and error not corrected, respectively.

The EPPI\_STAT.ERRDET bit is set whenever an error is detected in the status word. However, this bit does not generate an interrupt. The EPPI\_STAT.ERRNCOR bit is set when more than a 1-bit error is detected in the status word. An interrupt is generated when the EPPI\_STAT.ERRNCOR bit is set. It can be cleared by clearing the EPPI\_STAT.ERRNCOR and EPPI\_STAT.ERRDET bits. Both bits are sticky and W1C.

In many applications, video streams other than the standard NTSC/PAL formats (for example, CIF , QCIF) can be employed. The processor interface is flexible enough to accommodate different row and field lengths. In general, as long as the incoming video has the proper EAV/SAV codes, the EPPI can read it in. A CIF image could be formatted to be 656-compliant, where EAV and SAV values define the range of the image for each line. The V and F codes are used to delimit fields and frames.

The following sections provide descriptions of EPPI operations.

Table 18-10: Operating Modes and Generic EPPI Operation

|                 |               | How to configure                     | Useful for                                                                          | How to configure in ITU R 656 Tx Mode       |
|-----------------|---------------|--------------------------------------|-------------------------------------------------------------------------------------|---------------------------------------------|
| ITU-R BT.656 Rx | Entire field  | DIR= 0 XFRTYPE = b#01                |                                                                                     |                                             |
| ITU-R BT.656 Rx | Active video  | DIR = 0 XFRTYPE = b#00               |                                                                                     |                                             |
| ITU-R BT.656 Rx | Blanking only | DIR = 0 XFRTYPE = b#10               |                                                                                     |                                             |
| GP 0 FS         | Tx            | DIR = 1 XFRTYPE = b#11 FSCFG = b#00  | Applications where peri- odic frame syncs are not used to frame the data            | BLANKGEN = 1 DLEN = (b#000, b#001 or b#100) |
| GP 0 FS         | Rx            | DIR = 0 XFRTYPE = b#11 FSCFG = b#00  | Applications where peri- odic frame syncs are not used to frame the data            |                                             |
| GP 1 FS         | Tx            | DIR = 1 XFRTYPE = b#11 FSCFG =b#01   | Interfacing with ADCs, DACs, and other general- purpose devices                     | BLANKGEN = 1 DLEN = (b#000, b#001 or b#100) |
| GP 1 FS         | Rx            | DIR = 0 XFRTYPE = b#11 FSCFG = b#01  | Interfacing with ADCs, DACs, and other general- purpose devices                     |                                             |
| GP 2 FS         | Tx            | DIR = 1 XFRTYPE = b#11 FSCFG = b#10  | Video applications that use two hardware syn- chronization signals, HSYNC and VSYNC | BLANKGEN = 1 DLEN = (b#000, b#001 or b#100) |
| GP 2 FS         | Rx            | DIR = 0 XFRTYPE = b#11 FS_CFG = b#10 | Video applications that use two hardware syn- chronization signals, HSYNC and VSYNC |                                             |
| GP 3 FS         | Tx            | DIR = 1 XFRTYPE = b#11 FSCFG = b#11  | Video applications that use three hardware sync signals, HSYNC, VSYNC, and FIELD    | BLANKGEN = 1 DLEN = (b#000, b#001 or b#100) |
| GP 3 FS         | Rx            | DIR = 0 XFRTYPE = b#11 FSCFG = b#11  | Video applications that use three hardware sync signals, HSYNC, VSYNC, and FIELD    |                                             |

## ITU-R 656 Input Modes

In the ITU-R 656 input modes, the video source provides the clock or the system supplies it externally.

As shown in the ITU-R 656 Input Submodes figure and described in the following sections, there are three submodes supported for ITU-R 656 inputs: entire field, active video only, and vertical blanking interval only.

Figure 18-10: ITU-R 656 Input Submodes

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000008_5a24ce84d496018f10c956a4b22860530ce6908caee776b329426f639511e247.png)

## Entire Field

In this mode, the EPPI reads the entire incoming bit stream. This stream includes active video as well as control byte sequences and ancillary data that can be embedded in horizontal and vertical blanking intervals.

Data transfer starts immediately after Field 1 synchronization occurs. The transfer does not include the first EAV code that contains the F = 0 assignment for interlaced video or the V = 0 assignment for progressive video.

## Active Video

The EPPI uses this mode when only the active video portion of a field is of interest. The EPPI ignores (does not read in) all data between EAV and SAV, as well as all data present when V = 1. Furthermore, the control byte sequences are not stored to memory. The EPPI filters the sequences. After the start of Field 1 synchronizes, the EPPI ignores incoming samples until it sees an SAV.

In active video mode, programs must specify the number of total (active plus vertical blanking) lines per frame in the EPPI\_FRAME register. Programs must specify the number of total (active plus horizontal blanking plus 8) samples per line in the EPPI\_LINE register.

In this mode, any input data sequence that is considered part of the preamble is not sent to memory such as in 8-bit ITU mode. If 0xFF or 0x00 appear in the input data stream, these values are considered part of the preamble. The part of the preamble can appear individually and not be tagged along with the preamble sequence FF , 00, 00. This functionality also applies to vertical blanking interval mode.

## Vertical Blanking Interval (VBI)

In this mode, data transfer is only active while V = 1 is in the control byte sequence. This functionality indicates that the video source is in the midst of the vertical blanking interval (VBI), which is sometimes used for ancillary data transmission. The ITU-R 656 recommendation specifies the format for these ancillary data packets, but the EPPI is not equipped to decode the packets themselves. Software must handle this task. Horizontal blanking data is logged where it coincides with the rows of the VBI.

The VBI is split into two regions within each field. The EPPI considers these two separate regions as one contiguous space. However, frame synchronization begins at the start of Field 1, which does not necessarily correspond to the start of vertical blanking. For instance, in 525/60 systems, the start of Field 1 (F = 0) corresponds to line 4 of the VBI.

In VBI mode, the program must specify the number of total (active plus vertical blanking) lines per frame in the EPPI\_FRAME register. The program must specify the number of total (active plus horizontal blanking plus 8) samples per line in the EPPI\_LINE register.

In this mode, any input data sequence that is considered as part of the preamble is not sent to memory such as in 8bit ITU mode. If 0xFF or 0x00 appears in the input data stream, these values are considered part of the preamble. The part of the preamble can appear individually, and not be tagged along with the preamble sequence FF , 00, 00. This functionality applies to active video mode too.

## ITU-R 656 Output in General-Purpose Transmit Modes

In GP transmit mode, the EPPI frames an ITU-R 656 output stream with the proper preambles and blanking intervals by setting the EPPI\_CTL.BLANKGEN bit. The EPPI fetches active data from memory through the DMA channel, saving DMA bandwidth. The EPPI generates and embeds the proper preamble, status word (EAV and SAV sequences), and blanking data along with the active video from memory. Program the EPPI\_FS1\_PASPL , EPPI\_FS2\_WLVB , EPPI\_FS2\_PALPF , and EPPI\_FS1\_WLHB registers to perform the desired functions. The EPPI can also drive out the frame syncs using the EPPI\_CTL.FSCFG bit setting.

The 16-Bit Transmit with Internal Blanking Generation figure shows the bit stream format in 16-bit transmit modes with blanking generation ( EPPI\_CTL.BLANKGEN enabled). Each 16-bit data sample consists of 8-bit luma (Y) and 8-bit chroma (Cr or Cb) components. During transmission, the chroma data and blanking bytes of value 0x80 are placed on the upper half (MSBs) of the data lines. The luma data and blacking bytes of value 0x10 are placed on the lower half (LSBs) of the data lines.

Figure 18-11: 16-Bit Transmit with Internal Blanking Generation

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000009_7d43007619dbfa9800298637966f8045d26f93b3e3b0a2f7dd740a2bb60f947f.png)

The Generated Blanking Preamble Sequence figure shows the data transmitted by the EPPI in this mode. After the EPPI is enabled, and if the EPPI FIFO is not empty, the transmission starts by sending out an EAV sequence for a vertical blanking line. For interlaced video, F starts at 1. For progressive video, F is always 0.

NOTE: Internal blanking generation functionality is valid only when the data length is 8, 10, or 16 bits and when the EPPI is in GP transmit modes. The EPPI\_CTL.BLANKGEN bit generates preambles even in GP 2FS mode.

The internal blanking generation functionality of the ITU-R 656 output mode can also be bypassed by clearing the EPPI\_CTL.BLANKGEN bit. (For example, if sending ancillary data in the blanking interval). The EPPI\_CTL.BLANKGEN bit generates preambles even in GP 2FS mode.

Figure 18-12: Generated Blanking Preamble Sequence

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000010_e36877a407a8750558bccefad44691cff3414de5777e54cb1cc170bf3516e6aa.png)

## Frame Synchronization in ITU-R 656 Modes

For interlaced video, the start of frame synchronization occurs when a high-to-low transition is detected in F , the field indicator. For progressive video, the start of frame synchronization occurs when a high-to-low transition is detected in V, the vertical blanking indicator. These transitions in F and V can occur only in the EAV sequence. A start of line is detected on a low-to-high transition in H, the horizontal blanking indicator, which occurs in the EAV sequence as well.

For interlaced video, the start of frame corresponds to the start of field 1. Therefore, up to two fields can be ignored before the EPPI receives data. (For example, if field 1 started before the EPPI-to-camera channel was established). For progressive video, the start of frame corresponds to the start of active video.

Because all H and V signaling is embedded in the data stream in ITU-R 656 modes, the EPPI ignores the count registers ( EPPI\_HCNT , EPPI\_VCNT ). However, the EPPI still uses the EPPI\_FRAME register to check for synchronization errors. Therefore, program this MMR with the number of lines expected in each frame of video.

The EPPI monitors the number of EAV-to-SAV transitions that occur from the start of a frame until it decodes the end of frame condition. (For example, a transition from F = 1 to F = 0 for interlaced video and a transition from V = 1 to V = 0 for progressive video).

At the end of frame condition, the actual number of lines processed is compared against the value in EPPI\_FRAME . If there is a mismatch, a frame track error is asserted in the EPPI\_STAT register. For example, if an SAV transition was missed, the current field only has NUM\_ROWS - 1 rows. But, resynchronization occurs at the start of the next frame. When the EPPI receives the entire field, the field status bit is toggled in the EPPI\_STAT register. This way, an interrupt service routine (ISR) can discern which field was previously read in.

## General-Purpose EPPI Modes

The general-purpose (GP) EPPI modes accommodate a wide variety of data capture and transmission applications.

Each EPPI has three bidirectional frame sync pins. The EPPI internally generates frame syncs, or an external device communicating with the EPPI generates them.

GP modes differ based on the number of frame syncs used and the EPPI supports GP 0 FS-GP 3 FS modes.

All the GP modes, except 0 FS mode, support horizontal windowing. GP modes with 2 and 3 frame syncs also support vertical windowing.

For GP transmit modes with internal clock or frame syncs, the EPPI starts generating the clock or frame syncs only when the EPPI FIFO is full for the first time. For GP 0 FS transmit mode, the EPPI only starts transmitting when the EPPI FIFO is full for the first time.

## General-Purpose 0 Frame Sync Mode

This mode is useful for applications where periodic frame syncs are not used to frame the data.

After the initial trigger, the EPPI receives or transmits data samples on every clock cycle. However, if the EPPI\_CTL.SKIPEN bit is set for receive mode, the EPPI receives only alternate data samples.

The EPPI\_LINE , EPPI\_FRAME , EPPI\_HCNT , EPPI\_HDLY , EPPI\_VCNT , and EPPI\_VDLY registers are not valid for GP 0 FS mode. Therefore, windowing is not possible in this mode. Also, line and frame track errors are not applicable in this mode.

GP 0 FS receive mode is further divided into two submodes; internal trigger ( EPPI\_CTL.FLDSEL bit =0) and external trigger ( EPPI\_CTL.FLDSEL bit =1). The submodes are based on how the processor initiates data transmission or reception. GP 0 FS transmit mode is always internally triggered. DMA handles all subsequent data manipulation.

- Frame synchronization in GP 0 FS external trigger mode. When the EPPI is programmed in external trigger mode, it does not generate the EPPI\_FS1 signal and the external device must provide a trigger. The EPPI starts receiving the data as soon as an EPPI\_FS1 signal assertion is detected. After that, the DMA handles all subsequent data manipulation and any activity on EPPI\_FS1 is ignored.
- Frame synchronization in GP 0 FS internal trigger mode. When the EPPI is programmed in internal trigger mode, it starts receiving or transmitting data as soon as the EPPI clock is enabled and synchronized. There can be up to four PPI clock cycles of latency before valid data is received or transmitted.

## General-Purpose 1 Frame Sync Mode

This mode is useful for interfacing the EPPI with analog-to-digital converters (ADCs), digital-to-analog converters (DACs), and other general-purpose devices. This mode works for both transmit and receive.

The EPPI\_FRAME , EPPI\_VDLY , and EPPI\_VCNT registers have no effect in GP 1 FS mode. As a result, frame track errors and vertical windowing are not available.

## General-Purpose 2 Frame Sync Mode

This mode is useful for video applications that use two hardware synchronization signals, HSYNC and VSYNC. The HSYNC signal can be connected to EPPI\_FS1 and the VSYNC signal can be connected to EPPI\_FS2 .

## Data Enable in General-Purpose 2 Frame Sync Transmit Mode

The EPPI\_FS3 pin functions as a data enable (DEN) pin, when EPPI is configured in GP 2 FS transmit mode and generating the frame sync internally. The bits EPPI\_CTL.MUXSEL and EPPI\_CTL.CLKGATEN are not enabled. The functionality of the DEN pin is described in the following two cases.

## Case 1

Blanking generation is configured using the EPPI\_CTL.BLANKGEN bit. EPPI data length ( EPPI\_CTL.DLEN bit) is configured for 8, 10, or 16-bit transfers. The EPPI\_FS3 pin asserts during the active data regions, aligned with EPPI\_CLK according to the clock polarity ( EPPI\_CTL.POLC bit) settings. For this mode, the pin EPPI\_FS3 is driven based on the EPPI\_CTL.POLC setting. The pin EPPI\_FS3 is driven out on the same EPPI clock edge that drives out data. The frame sync polarity ( EPPI\_CTL.POLS ) setting does not apply here-EPPI\_FS3 is always active high in this mode.

## Case 2

Blanking generation ( EPPI\_CTL.BLANKGEN =0) is disabled. Or blanking generation is enabled, but the EPPI data length ( EPPI\_CTL.DLEN bit) is configured for a transfer size other than 8, 10, or 16 bits. The EPPI\_FS3 pin asserts at the start of the active data region on each line, aligned with EPPI\_CLK according to the EPPI\_CTL.POLC bit settings. For this mode, the pin EPPI\_FS3 is driven based on the EPPI\_CTL.POLC setting. The EPPI\_FS3 signal is driven out on the same EPPI clock edge that drives out data.

The EPPI\_CTL.POLS bit setting does not apply for case 2. The EPPI\_FS3 signal is always active high in this mode. Once asserted, EPPI\_FS3 stays asserted for the number of clock cycles per line configured in the EPPI\_HCNT register, then it deasserts. This behavior on each line continues for the total number of lines programmed in the EPPI\_VCNT register per frame. The behavior repeats at the start of subsequent video frames.

In case 2, if transmission of valid data is held off due to delays programmed in the EPPI\_HDLY or EPPI\_VDLY registers, the assertion of EPPI\_FS3 is also held off. The delay is on a per-line or per-frame basis.

## General-Purpose 3 Frame Sync Mode

This mode is useful for video applications that use three synchronization signals for hardware: HSYNC, VSYNC, and FIELD. The HSYNC connects to the EPPI\_FS1 pin, VSYNC connects to the EPPI\_FS2 pin, and FIELD connects to the EPPI\_FS3 pin.

GP 3 FS mode is similar in operation to GP 2 FS mode. However, the start of frame synchronization in GP 3 FS also considers the state of the EPPI\_FS3 pin. All the windowing register settings ( EPPI\_FRAME , EPPI\_LINE , EPPI\_HDLY , EPPI\_HCNT , EPPI\_VDLY , and EPPI\_VCNT registers), as well as data reception or transmission and error generation are the same as for GP 2 FS mode. In addition, for GP 3 FS mode with internal frame syncs, the EPPI\_CTL.FLDSEL bit setting specifies the condition under which the transfer begins.

The EPPI generates the EPPI\_FS3 signal and toggles during every assertion of EPPI\_FS2 or a combination of EPPI\_FS2 and EPPI\_FS1 . The toggle depends on the EPPI\_CTL.FLDSEL bit setting. The EPPI skips an EPPI\_FS2 signal when the EPPI\_FS3 value is high. Because of this condition, program the EPPI\_FS2 period value to half of the total number of pixels in the frame as in GP 3 FS mode. When in GP 2 FS mode, program the EPPI\_FS2 period with the value equal to the number of pixels per frame.

## Supported Data Formats

The following sections describe EPPI receive and transmit data formats.

## Receive Data Formats

The EPPI Receive Data Formats table provides information about EPPI configuration for specific use models for receive data.

Table 18-11: EPPI Receive Data Formats

|   Input Data Width | Use Model     | Splitting/Packing Options                                                                                                                                                                                                                                            |
|--------------------|---------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                  8 | NTSC/PAL data | EPPI_CTL.SPLTEO =1 EPPI_CTL.SUBSPLTODD =1 if necessary to separate chroma compo- nents                                                                                                                                                                               |
|                    | RGB sensor    | No splitting possible. EPPI_CTL.PACKEN =1 - Four EPPI words are packed to 32-bitDMA data. EPPI_CTL.PACKEN =0 - Each EPPI word is sent as 8-bit data on the 32-bit DMAbus. This transfer consumes 4 times the DMAbandwidth of the 8-bit case with EPPI_CTL.PACKEN =1; |
|                    | ADCs          | Gives I (in phase) and Q(quadrature) components. EPPI_CTL.SPLTEO =1 EPPI_CTL.SUBSPLTODD =0 since there are only two components.                                                                                                                                      |

Table 18-11: EPPI Receive Data Formats (Continued)

|   Input Data Width | Use Model     | Splitting/Packing Options                                                                                                                                                                                                                                                                                                                                         |
|--------------------|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                 10 | NTSC/PAL data | Each EPPI word is zero filled or sign extended to 16 bits. EPPI_CTL.SPLTEO =1. EPPI_CTL.SUBSPLTODD =1 if necessary to separate chroma compo- nents.                                                                                                                                                                                                               |
|                 10 | RGB sensor    | No splitting possible. EPPI_CTL.PACKEN =1. Two EPPI words are zero filled or sign exten- ded to 16 bits and packed to 32-bit DMAdata. EPPI_CTL.PACKEN =0. Each EPPI word can be zero filled or sign ex- tended to 16 bits and sent as a 16-bit data on the 32-bit DMAbus. This transfer consumes double the bandwidth of the 10-bit case with EPPI_CTL.PACKEN =1; |
|                 10 | ADCs          | Each EPPI word is zero filled or sign extended to 16 bits. EPPI_CTL.SPLTEO =1 S EPPI_CTL.SUBSPLTODD = 0 since there are only two components.                                                                                                                                                                                                                      |
|                 12 | RGB sensor    | No splitting possible. EPPI_CTL.PACKEN =1. Two EPPI words are zero filled or sign exten- ded to 16 bits and packed to 32-bit DMAdata. EPPI_CTL.PACKEN =0. Each EPPI word can be zero filled or sign ex- tended to 16 bits and sent as a 16-bit data on the 32-bit DMAbus. This transfer consumes double the bandwidth of the 12-bit case with EPPI_CTL.PACKEN =1; |
|                 12 | ADCs          | Each EPPI word is zero filled or sign extended to 16 bits. EPPI_CTL.SPLTEO =1 EPPI_CTL.SUBSPLTODD =0 since there are only two components.                                                                                                                                                                                                                         |
|                 14 | ADCs          | Each EPPI word is zero filled or sign extended to 16 bits. EPPI_CTL.SPLTEO =1 EPPI_CTL.SUBSPLTODD =0 since there are only two components.                                                                                                                                                                                                                         |

Table 18-11: EPPI Receive Data Formats (Continued)

|   Input Data Width | Use Model                             | Splitting/Packing Options                                                                                                                                                                                                                                                                                                                                          |
|--------------------|---------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                 16 | 8-bit luma/chroma pair for NTSC or HD | EPPI_CTL.SPLTEO =1, EPPI_CTL.SPLTWRD =1, EPPI_CTL.SUBSPLTODD =1 if necessary to separate chroma compo- nents.                                                                                                                                                                                                                                                      |
|                 16 | 16-bit luma/chroma pair for NTSC orHD | EPPI_CTL.SPLTEO =1, EPPI_CTL.SPLTWRD =0, EPPI_CTL.SUBSPLTODD =1 if necessary to separate chroma compo- nents.                                                                                                                                                                                                                                                      |
|                 16 | RGB565 sensor                         | No splitting possible. EPPI_CTL.PACKEN =1. Two EPPI words are packed to a 32-bitDMA data. EPPI_CTL.RGBFMTEN is valid only in transmit modes. So, RGB565 cannot be byte aligned in memory. EPPI_CTL.PACKEN =0. Each EPPI word is sent as a 16-bit data on the 32-bit DMAbus. This transfer consumes double the bandwidth of the 16-bit case with EPPI_CTL.PACKEN =1 |
|                 16 | 8-bit ADCs I/Q pair                   | EPPI_CTL.SPLTEO =1, EPPI_CTL.SPLTWRD =1, EPPI_CTL.SUBSPLTODD =0.                                                                                                                                                                                                                                                                                                   |
|                 16 | 16-bit ADCs I/Q pair                  | EPPI_CTL.SPLTEO =1, EPPI_CTL.SPLTWRD =0, EPPI_CTL.SUBSPLTODD =0.                                                                                                                                                                                                                                                                                                   |

## Transmit Data Formats

The EPPI Transmit Data Formats table provides information about EPPI configuration for specific use models for transmit data.

Table 18-12: EPPI Transmit Data Formats

|   Output Data Width | Use Model                            | Splitting/Packing Options                                                                                                                                                                                                                                                                                                 |
|---------------------|--------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                   8 | NTSC/PAL data                        | EPPI_CTL.SPLTEO =1 EPPI_CTL.SUBSPLTODD =1 if the chroma components (U and V) come in separate DMAwords.                                                                                                                                                                                                                   |
|                   8 | Serial RGB for lower-resolution LCDs | No splitting possible. EPPI_CTL.PACKEN =1. The 32-bit DMAdata is unpacked to drive four EPPI words. EPPI_CTL.PACKEN =0. The lowest 8 bits of the DMAdata is driven on the EPPI data and the rest of the 24 bits are discarded. This transfer consumes 4 times the DMAbandwidth of the 8-bit case with EPPI_CTL.PACKEN =1. |
|                  10 | NTSC/PAL data                        | EPPI_CTL.SPLTEO =1 EPPI_CTL.SUBSPLTODD =1 if the chroma components (U and V) come in separate DMAwords.                                                                                                                                                                                                                   |
|                  10 | DACs                                 | EPPI_CTL.SPLTEO =1, EPPI_CTL.SUBSPLTODD =0.                                                                                                                                                                                                                                                                               |

Table 18-12: EPPI Transmit Data Formats (Continued)

|   Output Data Width | Use Model                             | Splitting/Packing Options                                                                                                                         |
|---------------------|---------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
|                  12 | DACs                                  | EPPI_CTL.SPLTEO =1, EPPI_CTL.SUBSPLTODD =0.                                                                                                       |
|                  14 | DACs                                  | EPPI_CTL.SPLTEO =1, EPPI_CTL.SUBSPLTODD =0.                                                                                                       |
|                  16 | 8-bit luma/chroma pair for NTSC orHD  | EPPI_CTL.SPLTEO =1, EPPI_CTL.SPLTWRD =1, EPPI_CTL.SUBSPLTODD =1 if the chroma components (U and V) come in separate DMAwords.                     |
|                  16 | 16-bit luma/chroma pair for NTSC orHD | EPPI_CTL.SPLTEO =1, EPPI_CTL.SPLTWRD =0, EPPI_CTL.SUBSPLTODD =1 if the chroma components (U and V) come in separate DMAwords.                     |
|                  16 | RGB565 LCD                            | No splitting possible. EPPI_CTL.RGBFMTEN =1. Takes RGB888 data from the memory and drops the LSBs from each component to drive out RGB565 data.   |
|                  16 | 8-bit ADCs I/Q pair                   | EPPI_CTL.SPLTEO =1, EPPI_CTL.SPLTWRD =1, EPPI_CTL.SUBSPLTODD = 0                                                                                  |
|                  16 | 16-bit ADCs I/Q pair                  | EPPI_CTL.SPLTEO =1, EPPI_CTL.SPLTWRD =0, EPPI_CTL.SUBSPLTODD =1                                                                                   |
|                  18 | RGB666 LCD                            | No splitting possible. EPPI_CTL.RGBFMTEN =1. Takes RGB888 data from the memory and drops the 2 LSBs from each component to drive out RGB666 data. |

## Data Transfer Modes

The following sections describe EPPI data transfer modes, including receive or transmit data packing, sign extension, zero fill, receive or transmit split modes, clock gating, delayed start, and data consistency management.

## Data Packing for Receive Modes

For receive modes, if the EPPI\_CTL.PACKEN bit =1 and the DMA is 32 bits, the EPPI packs the incoming data into 32-bit words based on the EPPI\_CTL.DLEN and EPPI\_CTL.SWAPEN bit settings. When EPPI\_CTL.SWAPEN =0, the EPPI puts the first data in the least significant bits and when EPPI\_CTL.SWAPEN =1, the EPPI puts the first data in the most significant bits. The packing options for the EPPI\_CTL.DLEN bits are as follows.

- When EPPI\_CTL.DLEN =8, four 8-bit words can be packed into one 32-bit word.
- When EPPI\_CTL.DLEN =16, two 16-bit words can be packed into one 32-bit word.
- For EPPI\_CTL.DLEN values that are more than 8 bits but less than 16 bits, two such words are either signextended or zero-filled to 16 bits, and packed into one 32-bit word.
- When EPPI\_CTL.DLEN =18, the EPPI sign-extends or zero-fills the 18-bit data to 24 bits and packs four 24-bit words into three 32-bit words.

- When EPPI\_CTL.DLEN =24, the EPPI packs four 24-bit words into three 32-bit words.

When EPPI\_CTL.PACKEN =0, the EPPI receives the incoming data and sends it on the bus as-is. If EPPI\_CTL.DLEN is less than or equal to 16 bits, the DMA is a 16-bit DMA; otherwise it is a 32-bit DMA.

## Data Packing for Transmit Modes

For transmit modes, if the EPPI\_CTL.DLEN bit =1 and the DMA is a 32-bit DMA, the EPPI unpacks the 32-bit word according to the EPPI\_CTL.DLEN and EPPI\_CTL.SWAPEN bit settings.

If EPPI\_CTL.SWAPEN =1, the EPPI transmits the most significant bits as the first data, and if EPPI\_CTL.SWAPEN =0, the EPPI transmits the least significant bits as the first data. The unpacking options for the EPPI\_CTL.DLEN bits are as follows.

- When EPPI\_CTL.DLEN =8, the EPPI transmits one 32-bit word from memory as four 8-bit data words.
- For EPPI\_CTL.DLEN values greater than 8 bits but less than or equal to 16 bits, the EPPI transmits one 32bit word from memory as two 16-bit data words.
- When EPPI\_CTL.DLEN =18 or 24, the EPPI transmits three 32-bit words from memory as four data words.

## Sign-Extended and Zero-Filled Data

The following list describes the bit settings and functionality for sign-extending and zero-filling data.

- For EPPI\_CTL.DLEN equal to 10, 12 or 14, data is zero-filled or sign-extended to 16 bits.
- For EPPI\_CTL.DLEN equal to 18 bits, data is zero-filled or sign-extended to 24 bits if packing is enabled, and zero-filled or sign-extended to 32 bits if packing is disabled.
- For EPPI\_CTL.DLEN equal to 24 bits, data is zero-filled or sign-extended to 32 bits if packing is disabled.
- For EPPI\_CTL.DLEN equal to 8 bits, data is zero-filled or sign-extended to 16 bits if packing is disabled.
- If EPPI\_CTL.SIGNEXT =1, then the data is sign-extended, otherwise it is zero-filled.

## Split Receive Modes

The control register has three control bits for split receive modes: EPPI\_CTL.SPLTEO , EPPI\_CTL.SUBSPLTODD , and EPPI\_CTL.DMACFG . Packing is not valid in split modes.

- If EPPI\_CTL.SPLTEO =1, the EPPI splits the incoming data stream into two substreams, an even stream, and an odd stream, and packs them separately.
- The EPPI\_CTL.SUBSPLTODD bit is available only when EPPI\_CTL.SPLTEO =1. When EPPI\_CTL.SUBSPLTODD =1, the EPPI subsplits the odd substream, and packs the streams separately.
- The EPPI\_CTL.DMACFG bit is also available only if EPPI\_CTL.SPLTEO =1. If EPPI\_CTL.DMACFG =1, the EPPI uses two DMA channels and if EPPI\_CTL.DMACFG =0, the EPPI uses only one DMA channel.

## Split Transmit Modes

The EPPI\_CTL register has three control bits for split transmit modes: EPPI\_CTL.SPLTEO , EPPI\_CTL.SUBSPLTODD , and EPPI\_CTL.DMACFG . The DMA is always a 32-bit DMA. Packing is not valid in split modes.

- If EPPI\_CTL.SPLTEO =1, the EPPI receives the Luma (Y3Y2Y1Y0) and interleaved Chroma (Cr1Cb1Cr0Cb0) data as 32 bits from the DMA channel. The EPPI interleaves the data to form a 4:2:2 YCrCb data stream to transmit.
- The EPPI\_CTL.SUBSPLTODD bit is available only when EPPI\_CTL.SPLTEO =1. In this case, if EPPI\_CTL.SUBSPLTODD =1, the EPPI receives the Luma (Y3Y2Y1Y0) and deinterleaved Chroma (Cb3Cb2Cb1Cb0 and Cr3Cr2Cr1Cr0). The EPPI interleaves the data to form a 4:2:2 YCrCb data stream to transmit. (The EPPI does not decimate the chroma data when formatting it into 4:2:2.)
- The EPPI\_CTL.DMACFG bit is also valid only if EPPI\_CTL.SPLTEO =1. If EPPI\_CTL.DMACFG =1, the EPPI uses two DMA channels and if EPPI\_CTL.DMACFG =0, the EPPI uses only one DMA channel.

## Clock Gating

In ITU-R BT.656 and GP 0/1/2 FS modes, EPPI\_FS3 becomes a clock-gating input. This functionality is valid for both internally and externally sourced EPPI\_CLK , in both receive and transmit modes. This clock gating signal must be synchronous with EPPI\_CLK . The external device on the rising edge of EPPI\_CLK must drive the clock gating signal. Its function is to hold the sync and data lines in their current state until EPPI\_FS3 is driven low. There are no additional latency cycles upon coming out of clock gating mode.

If clock gating is not required, the EPPI\_FS3 pin must either be tied to ground, or configured to operate as another of its multiplexed functions.

In GP 2 FS transmit mode with internally generated frame syncs, the EPPI\_FS3 pin functions as a data enable signal.

## Support for Delayed Start of EPPI Frame Syncs

The EPPI supports a delayed start of the EPPI\_FS1 and EPPI\_FS2 frame syncs. The EPPI\_FS1\_DLY and EPPI\_FS2\_DLY registers are programmable registers corresponding to EPPI\_FS1 (HSYNC) and EPPI\_FS2 (VSYNC).

The delay programmed in these registers applies to the first active edge of the internally generated frame sync. The delay starts from the first EPPI\_CLK edge. The delay counter runs only for the first time and then shuts off until the EPPI is reenabled. (The delay counter is the period counter itself, since they do not run together.) Program the delay registers prior to the first EPPI\_CLK edge (similar to the width and period registers). The EPPI Delayed Frame Sync Generation figure shows the functioning of EPPI\_FS1 and EPPI\_FS2 .

Figure 18-13: EPPI Delayed Frame Sync Generation

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000011_00710b6d8eaaf6a5f453e31cb3205d83954d7295fa242135c49d60582823ef92.png)

## Ignoring Premature External Frame Syncs for Data Consistency

Once a frame has started with a VSYNC followed by an HSYNC (or both coming together), a line is tracked. When the count expires, the state machine waits at the end of line for an HSYNC to come. With the arrival of the HSYNC, the state machine starts tracking the next line, and so on.

The number of lines tracked is counted separately. Once the end of a frame is reached, the state machine waits there for the next VSYNC/HSYNC combination. The next frame starts once they are sampled. Unfortunately, every incoming FS (VSYNC or HSYNC) resets the respective counters and the tracking starts all over (even if the FS signals are premature). The result is incomplete data (or frames) to enter into memory through the PxP interface.

To correct this problem, the EPPI waits for a frame or line completion before considering any incoming FS as valid.

- Single FS mode and line tracking in dual FS mode - When a line is in progress, when HSYNC is detected prematurely, it is ignored. A line track underflow event is generated.
- Dual FS mode - If a VSYNC is received when a frame is in progress, it is ignored. A frame track underflow error ( EPPI\_STAT.FTERRUNDR ) is generated.

Ignoring the FS ensures that once a frame starts, the amount of data that goes into the memory/PxP interface corresponds exactly to the programmed data size in a frame.

NOTE: Even if the premature FS is a valid FS, the state machine loses at most one frame and it recovers in the subsequent FS. The FS to number of data going into the memory relationship is always maintained as programmed.

When data underflow errors occur at the DMA interface, the EPPI does the following.

- If a premature line sync is detected, an LT underflow error is generated ( EPPI\_STAT.LTERRUNDR =1). All further line track errors are ignored until the EPPI detects the next valid line sync.
- If a premature frame sync is detected, an FT underflow error is generated ( EPPI\_STAT.FTERRUNDR =1). All further frame track and line track errors are ignored until the EPPI detects the next valid frame sync.

## EPPI Event Control

The following sections describe how EPPI manages events.

PPI\_FS1P

## EPPI Status, Error, and Interrupt Signals

The EPPI generates error interrupts (flagged in the EPPI\_STAT register) when any one of the following error conditions occur.

- EPPI\_STAT.YFIFOERR (YFIFO underflow or overflow)
- EPPI\_STAT.CFIFOERR (CFIFO underflow or overflow)
- EPPI\_STAT.LTERROVR (line track overflow error)
- EPPI\_STAT.LTERRUNDR (line track underflow error)
- EPPI\_STAT.FTERROVR (frame track overflow error)
- EPPI\_STAT.FTERRUNDR (frame track underflow error)
- EPPI\_STAT.ERRNCOR (ITU preamble error not corrected)

A W1C (write-1-to-clear) operation clears the error conditions. Each of the individual conditions which cause an EPPI error interrupt can be masked. The interrupt mask register ( EPPI\_IMSK ) allows the masking of individual conditions which cause error interrupts.

There is only one interrupt line from each EPPI so all interrupts are internally OR'ed and sent as a single interrupt to the core. The EPPI\_STAT register must then be read to discover specific errors. The following sections describe these errors in detail.

## Frame and Line Track Errors

In external frame sync mode, the EPPI uses line track error ( EPPI\_STAT.LTERROVR and EPPI\_STAT.LTERRUNDR ) and frame track error ( EPPI\_STAT.FTERROVR and EPPI\_STAT.FTERRUNDR ) status bits to monitor the line and frame synchronization errors. The EPPI updates the bits when there is a mismatch detected in the HSYNC and VSYNC as compared to the programmed values in EPPI\_LINE and EPPI\_FRAME count registers.

## Line Track Errors

The line track overflow ( EPPI\_STAT.LTERROVR ) and underflow errors ( EPPI\_STAT.LTERRUNDR ) generate a maskable interrupt as soon as the EPPI identifies them and not at the next frame sync.

- If the frame sync has not arrived when the EPPI\_LINE counter expires, then the EPPI\_STAT.LTERROVR error is generated.
- When the EPPI\_LINE counter is running and a frame sync is detected, the EPPI\_STAT.LTERRUNDR error is generated. A W1C operation clears boths interrupts.

## Frame Track Errors

The frame track overflow ( EPPI\_STAT.FTERROVR ) and underflow errors ( EPPI\_STAT.FTERRUNDR ) generate a maskable interrupt as soon as the EPPI identifies them. When the EPPI\_FRAME.VALUE counter expires, the EPPI\_STAT.FTERROVR error is reported before the next frame sync arrives.

When the EPPI\_FRAME counter is running and a frame sync is detected, then an EPPI\_STAT.FTERRUNDR is reported.

Both errors generate an error interrupt. Perform a W1C operation to clear the interrupts at their respective locations in the status register.

A premature frame sync results in a frame track under run error. But, the error is logged (register bit set) only after the subsequent blanking period (if any) elapses.

## Preamble Error Not Corrected Error

The EPPI supports data embedded frame syncs in ITU and SMPTE formats. In these formats, the module can receive an erroneous preamble which is not correctable. The EPPI\_STAT.ERRNCOR error signals when this event occurs.

## EPPI Programming Model

The following sections describe programming techniques, including receiving or transmitting ITU-R 656 frames; configuring transfers in GP0, GP1, GP2, and GP3 modes; and managing EPPI mode configurations.

## Receiving ITU-R 656 Frames

The EPPI supports the reception of ITU-R 656 compliant frames.

1. Configure the EPPI to receive either full ITU-R 656 frame, active video, or blanking information by configuring the EPPI\_CTL.XFRTYPE bits.
2. In both active video mode and in VBI (vertical blanking information) mode, specify the number of total (active plus vertical blanking) lines per frame in the EPPI\_FRAME register. Specify the number of total (active plus horizontal blanking plus 8) samples per line in the EPPI\_LINE register.
3. Configure DMA descriptors to move the data to memory.
4. Enable DMA.
5. Enable the EPPI.
6. To program the EPPI in internal clock mode, follow the procedure above with the EPPI\_CTL.ICLKGEN bit =0. After enabling the EPPI, add a delay of 200 SCLK1\_0 cycles (worst case) to ensure the EPPI FIFO becomes full. Then switch to internal clock mode by setting the EPPI\_CTL.ICLKGEN bit =1.

Depending on the EPPI configuration, either the full ITU-R 656 frame is moved to memory or only the active video or only the blanking information.

## Transmitting ITU-R 656 Frames in GP Transmit Modes

The EPPI can take active video from memory and generate the proper preambles and blanking information to produce valid ITU-R 656 video frames for transmission.

1. Provide active data frame in memory.
2. Set the EPPI\_CTL.BLANKGEN bit so the EPPI generates blanking information.
3. Configure the EPPI\_FS1\_WLHB , EPPI\_FS1\_PASPL , EPPI\_FS2\_WLVB , EPPI\_FS2\_PALPF registers accordingly.
4. Configure the rest of the EPPI settings.
5. Configure DMA to fetch active frame data from memory buffers.
6. Enable DMA.
7. Enable the EPPI.
8. To program the EPPI in internal clock mode, follow the procedure above with the EPPI\_CTL.ICLKGEN bit =0. After enabling the EPPI, add a delay of 200 SCLK1\_0 cycles (worst case) to ensure the EPPI FIFO becomes full. Then switch to internal clock mode by setting the EPPI\_CTL.ICLKGEN bit =1.

The EPPI takes the active data from memory, generates the blanking information, and transmits an ITU-R 656 frame

## Configuring Transfers in GP 0 FS Mode

The EPPI can be configured to not use periodic frame syncs to frame the data.

1. Configure the EPPI to operate in GP 0 FS mode by setting EPPI\_CTL.XFRTYPE = b#11 and EPPI\_CTL.FSCFG = b#00.
2. When receiving, configure the EPPI to trigger on internally or externally by setting the EPPI\_CTL.FLDSEL field appropriately. When transmitting, the EPPI always generates a trigger internally.
3. Configure DMA to move the data to or from memory.
4. Enable DMA.
5. Enable EPPI.
6. To program the EPPI in internal clock mode, follow the procedure above with the EPPI\_CTL.ICLKGEN bit =0. After enabling the EPPI, add a delay of 200 SCLK1\_0 cycles (worst case) to ensure the EPPI FIFO becomes full. Then switch to internal clock mode by setting the EPPI\_CTL.ICLKGEN bit =1.

The DMA descriptions control the amount of data transferred. The frame syncs from the EPPI do not control the amount.

## Configuring Transfers in GP 1 FS Mode

The GP 1 FS mode is useful for interfacing the EPPI with analog-to-digital converters (ADCs), digital-to-analog converters (DACs), and other general-purpose devices. This mode works for both transmit and receive.

- NOTE: The EPPI\_FRAME , EPPI\_VDLY , and EPPI\_VCNT registers have no effect in GP 1 FS mode. As a result, frame track errors and vertical windowing are not possible in this mode.
1. Configure GP 1 FS mode by setting the EPPI\_CTL.XFRTYPE bit =b#11 and the EPPI\_CTL.FSCFG bit =b#01. An external device can provide the frame syncs or the EPPI can source the frame syncs.
2. Program the EPPI\_LINE register to contain the number clock cycles expected between two assertions of the EPPI\_FS1 signal to monitor the line track errors. Program the EPPI\_LINE register before the EPPI\_HCNT register.
3. Program the EPPI\_HDLY register to contain the number of clock cycles to wait after the assertion of EPPI\_FS1 . For example, the start of frame.
4. Program the EPPI\_HCNT register to contain the number of data samples to receive or transmit for each frame.
5. Configure DMA to move the data to or from memory.
6. Enable DMA.
7. Enable the EPPI.
8. To program the EPPI in internal clock mode, follow the procedure above with the EPPI\_CTL.ICLKGEN bit =0. After enabling the EPPI, add a delay of 200 SCLK1\_0 cycles (worst case) to ensure the EPPI FIFO becomes full. Then, switch to internal clock mode by setting the EPPI\_CTL.ICLKGEN bit =1.

Data moves in or out of memory. A frame sync frames the data for every line.

## Configuring Transfers in GP 2 FS Mode

GP 2 FS mode is useful for video applications that use two hardware synchronization signals, HSYNC and VSYNC. The HSYNC connects to the EPPI\_FS1 signal and VSYNC connects to the EPPI\_FS2 signal.

1. Configure the EPPI to operate in GP 0 FS mode by setting the EPPI\_CTL.XFRTYPE bit =b#11 and the EPPI\_CTL.FSCFG bit =b#10. An external device can provide the frame syncs or the EPPI can source the frame syncs.
2. Program the EPPI\_FRAME register to contain the number of expected lines per frame. The value can be equal to the number of EPPI\_FS1 signal assertions expected between the start of each frame sync. The EPPI uses the value to monitor frame track errors. Program the EPPI\_FRAME register before the EPPI\_VCNT register.
3. Program the EPPI\_LINE register to contain the number of clock cycles expected between two assertions of the EPPI\_FS1 signal to monitor line track errors. Program the EPPI\_LINE register before the EPPI\_HCNT register.
4. Program the EPPI\_HDLY register to configure the number of clock cycles to wait after the assertion of the EPPI\_FS1 signal. (For example, the start of the line).
5. Program the EPPI\_HCNT register to contain the number of data samples to receive or transmit for each line.

6. Program the EPPI\_VDLY register to contain the number of lines to wait after the start of frame is detected.
7. Program the EPPI\_VCNT register to contain the number of lines to receive or transmit.
8. If setting up the EPPI for transmit, the data enable (DEN) pin behaves according to the enabling of the blanking generation and the data length setting (DLEN). See Data Enable in General-Purpose 2 Frame Sync Transmit Mode for more details.
9. Enable DMA.
10. Enable the EPPI.
11. To program the EPPI in internal clock mode, follow the procedure above with the EPPI\_CTL.ICLKGEN bit =0. After enabling the EPPI, add a delay of 200 SCLK1\_0 cycles (worst case) to ensure the EPPI FIFO becomes full. Then switch to internal clock mode by setting the EPPI\_CTL.ICLKGEN bit =1.

Data moves in or out of memory. A frame sync frames the data for every line and frame.

## Configuring Transfers in GP 3 FS Mode

GP 3 FS mode is useful for video applications that use three synchronization signals for hardware: HSYNC, VSYNC, and FIELD. The HSYNC connects to EPPI\_FS1 , VSYNC connects to EPPI\_FS2 , and FIELD connects to EPPI\_FS3 .

1. Configure the EPPI to operate in GP 3 FS mode by setting the EPPI\_CTL.XFRTYPE bit =b#11 and the EPPI\_CTL.FSCFG bit =b#11. An external device can provide the frame syncs or the EPPI can source the frame syncs.
2. Configure the windowing registers according to steps in GP 2 FS mode.
3. Enable DMA.
4. Enable the EPPI.
5. To program the EPPI in internal clock mode, follow the procedure above with the EPPI\_CTL.ICLKGEN bit =0. After enabling the EPPI, add a delay of 200 SCLK1\_0 cycles (worst case) to ensure the EPPI FIFO becomes full. Then switch to internal clock mode by setting the EPPI\_CTL.ICLKGEN bit =1.

Data moves in or out of memory. A frame sync frames the data for every line and frame. Operation and result are similar to operation in GP 2 FS mode but the EPPI also uses the EPPI\_FS3 signal.

## Configuring the EPPI to Use the Windowing Feature

Windowing is a useful feature for applications where the region of interest is smaller than the active video stream (for example, sensor calibration, auto-focusing, and others). It can result in significant DMA bandwidth reduction. The EPPI supports windowing for GP input modes.

1. Program the EPPI\_FRAME register with the number of lines the frame contains.

2. Program the EPPI\_LINE register with the number of samples per line in the frame.
3. Program the EPPI\_VDLY register with the number of lines to wait after the start of a new frame before starting to read or transmit data.
4. Program the EPPI\_VCNT register with the number of lines to read in or write out after EPPI\_VDLY number of lines from the start of the frame.
5. Program the EPPI\_HDLY register with the number of clock cycles to delay after the assertion of EPPI\_FS1 is detected for the start of a new line.
6. Program the EPPI\_HCNT register with the number of samples to read in or write out after EPPI\_HDLY number of cycles have expired since the assertion of EPPI\_FS1 .

## EPPI Mode Configuration

This section describes EPPI mode configurations, including support for all EPPI transmit and receive modes.

## Configuring 8-Bit Receive Mode

For 8-bit non-split receive mode and if EPPI\_CTL.PACKEN =1, the EPPI packs 4 bytes of incoming data into a 32-bit word. Alternate even or odd samples can be skipped based on the EPPI\_CTL.SKIPEN and EPPI\_CTL.SKIPEO bits. The first incoming data can be placed either in the least significant bit positions or in the most significant bit positions, based on the EPPI\_CTL.SWAPEN bit setting.

Table 18-13: 8-Bit Receive Mode with Packing Enabled

| Pin Data (8 bits)   | DMADATA SKIPEN=0 SKIPEO =X SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=1 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=1 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=1 SIGNEXT=X   |
|---------------------|-------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x11                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0x22                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0x33                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0x44                | 0x4433 2211                                     | 0x1122 3344                                    |                                                |                                                |                                                |                                                |
| 0x55                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0x66                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0x77                |                                                 |                                                | 0x7755 3311                                    |                                                | 0x1133 5577                                    |                                                |
| 0x88                | 0x8877 6655                                     | 0x5566 7788                                    |                                                | 0x8866 4422                                    |                                                | 0x2244 6688                                    |
| 0x99                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0xAA                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0xBB                |                                                 |                                                |                                                |                                                |                                                |                                                |

Table 18-13: 8-Bit Receive Mode with Packing Enabled (Continued)

| Pin Data (8 bits)   | DMADATA SKIPEN=0 SKIPEO =X SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=1 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=1 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=1 SIGNEXT=X   |
|---------------------|-------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0xCC                | 0xCCBB AA99                                     | 0x99AA BBCC                                    |                                                |                                                |                                                |                                                |
| 0xDD                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0xEE                |                                                 |                                                |                                                |                                                |                                                |                                                |
| 0xFF                |                                                 |                                                | 0xFFDD BB99                                    |                                                | 0x99BB DDFF                                    |                                                |
| 0x00                | 0x00FF EEDD                                     | 0xDDE EFF00                                    |                                                | 0x00EE CCAA                                    |                                                | 0xAACC EE00                                    |

If EPPI\_CTL.PACKEN =0, the DMA is a 16-bit DMA and the EPPI either sign-extends or zero-fills the bytes of incoming data into a 16-bit word. The EPPI\_CTL.SWAPEN bit has no effect if EPPI\_CTL.PACKEN =0.

Table 18-14: 8-Bit Receive Mode with Packing Disabled

| Pin Data (8 bits)   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=X SIGNEXT=0   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=X SIGNEXT=1   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=X SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=X SIGNEXT=1   |
|---------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x44                | 0x0044                                         | 0x0044                                         | 0x0044                                         |                                                |
| 0x55                | 0x0055                                         | 0x0055                                         |                                                | 0x0055                                         |
| 0x66                | 0x0066                                         | 0x0066                                         | 0x0066                                         |                                                |
| 0x77                | 0x0077                                         | 0x0077                                         |                                                | 0x0077                                         |
| 0x88                | 0x0088                                         | 0xFF88                                         | 0x0088                                         |                                                |
| 0x99                | 0x0099                                         | 0xFF99                                         |                                                | 0xFF99                                         |
| 0xAA                | 0x00AA                                         | 0xFFAA                                         | 0x00AA                                         |                                                |
| 0xBB                | 0x00BB                                         | 0xFFBB                                         |                                                | 0xFFBB                                         |

## Configuring 10/12/14-Bit Receive Modes

For 10, 12, or 14-bit non-split receive modes, the EPPI first either zero-fills or sign-extends the incoming data into a 16-bit word. The action depends on the setting of the EPPI\_CTL.SIGNEXT bit. If EPPI\_CTL.PACKEN =1, the EPPI then packs two of these words into one 32-bit word. Alternate even or odd samples can be skipped based on the EPPI\_CTL.SKIPEN and EPPI\_CTL.SKIPEO bits. The first incoming data can be placed either in the least significant bit positions or in the most significant bit positions, based on the EPPI\_CTL.SWAPEN bit setting.

Table 18-15: 10-Bit Receive Mode with Sign Extension, with Packing Enabled

| Pin Data (10 bits)   |   MSB | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=0 SIGNEXT=1   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=1 SIGNEXT=1   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=0 SIGNEXT=1   |
|----------------------|-------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x111                |     0 |                                                |                                                |                                                |
| 0x222                |     1 | 0xFE22 0111                                    | 0x0111 FE22                                    |                                                |
| 0x333                |     1 |                                                |                                                | 0xFF33 0111                                    |
| 0x044                |     0 | 0x0044 FF33                                    | 0xff33 0044                                    |                                                |
| 0x155                |     0 |                                                |                                                |                                                |
| 0x266                |     1 | 0xFE66 0155                                    | 0x0155 FE66                                    |                                                |
| 0x377                |     1 |                                                |                                                | 0xFF77 0155                                    |
| 0x088                |     0 | 0x0088 FF77                                    | 0xFF77 0088                                    |                                                |

Table 18-16: 10-Bit Receive Mode with Sign Extension, with Packing Enabled

| Pin Data (10 bits)   |   MSB | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=0 SIGNEXT=1   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=1 SIGNEXT=1   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=1 SIGNEXT=1   |
|----------------------|-------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x111                |     0 |                                                |                                                |                                                |
| 0x222                |     1 |                                                |                                                |                                                |
| 0x333                |     1 |                                                | 0x0011 FF33                                    |                                                |
| 0x044                |     0 | 0x0044 FE22                                    |                                                | 0xFE22 0044                                    |
| 0x155                |     0 |                                                |                                                |                                                |
| 0x266                |     1 |                                                |                                                |                                                |
| 0x377                |     1 |                                                | 0x0155 FF77                                    |                                                |
| 0x088                |     0 | 0x0088 FE66                                    |                                                | 0xFE66 0088                                    |

Table 18-17: 10-Bit Receive Mode, with Zero-Fill, with Packing Enabled

| Pin Data (10 bits)   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=0 SIGNEXT=0   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=1 SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=0 SIGNEXT=0   | DMADATA SKIP_EN=1 SKIP_EO=0 SWAPEN=0 SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=1 SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=1 SIGNEXT=0   |
|----------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|--------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x111                |                                                |                                                |                                                |                                                  |                                                |                                                |
| 0x222                | 0x0222 0111                                    | 0x0111 0222                                    |                                                |                                                  |                                                |                                                |
| 0x333                |                                                |                                                | 0x0333 0111                                    |                                                  | 0x0011 0333                                    |                                                |
| 0x044                | 0x0044 0333                                    | 0x0333 0044                                    |                                                | 0x0044 0222                                      |                                                | 0x0222 0044                                    |
| 0x155                |                                                |                                                |                                                |                                                  |                                                |                                                |
| 0x266                | 0x0266 0155                                    | 0x0155 0266                                    |                                                |                                                  |                                                |                                                |
| 0x377                |                                                |                                                | 0x0377 0155                                    |                                                  | 0x0155 0377                                    |                                                |
| 0x088                | 0x0088 0377                                    | 0x0377 0088                                    |                                                | 0x0088 0266                                      |                                                | 0x0266 0088                                    |

The 10-bit Receive Mode with Packing Disabled table shows a 10-bit receive mode example when EPPI\_CTL.PACKEN =0:

Table 18-18: 10-bit Receive Mode with Packing Disabled

| Pin Data (10 bits)   |   MSB | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=X SIGNEXT=1   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=X SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=X SIGNEXT=1   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=X SIGNEXT=0   |
|----------------------|-------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x111                |     0 | 0x0111                                         | 0x0111                                         | 0x0111                                         |                                                |
| 0x222                |     1 | 0xFE22                                         | 0x0222                                         |                                                | 0x0222                                         |
| 0x333                |     1 | 0xFF33                                         | 0x0333                                         | 0xFF33                                         |                                                |
| 0x044                |     0 | 0x0044                                         | 0x0444                                         |                                                | 0x0444                                         |
| 0x155                |     0 | 0x0155                                         | 0x0155                                         | 0x0155                                         |                                                |
| 0x266                |     1 | 0xFE66                                         | 0x0266                                         |                                                | 0x0266                                         |
| 0x377                |     1 | 0xFF77                                         | 0x0377                                         | 0xFF77                                         |                                                |
| 0x088                |     0 | 0x0088                                         | 0x0088                                         |                                                | 0x088                                          |

## Configuring 16-Bit Receive Mode

For 16-bit non-split receive mode, if EPPI\_CTL.PACKEN =1, the EPPI packs two 16-bit incoming data into one 32-bit word. Alternate even or odd samples can be skipped based on the EPPI\_CTL.SKIPEN and

EPPI\_CTL.SKIPEO bits. The first incoming data can be placed either in the least significant bit positions or in the most significant bit positions, based on the EPPI\_CTL.SWAPEN bit setting.

Table 18-19: 16-Bit Receive Mode with Packing Enabled

| Pin Data (16 bits)   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=1 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=0 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=1 SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=1 SIGNEXT=X   |
|----------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x1111               |                                                |                                                |                                                |                                                |                                                |                                                |
| 0x2222               | 0x2222 1111                                    | 0x1111 2222                                    |                                                |                                                |                                                |                                                |
| 0x3333               |                                                |                                                | 0x3333 1111                                    |                                                | 0x1111 3333                                    |                                                |
| 0x4444               | 0x4444 3333                                    | 0x3333 4444                                    |                                                | 0x4444 2222                                    |                                                | 0x2222 4444                                    |
| 0x5555               |                                                |                                                |                                                |                                                |                                                |                                                |
| 0x6666               | 0x6666 5555                                    | 0x5555 6666                                    |                                                |                                                |                                                |                                                |
| 0x7777               |                                                |                                                | 0x7777 5555                                    |                                                | 0x5555 7777                                    |                                                |
| 0x8888               | 0x8888 7777                                    | 0x7777 8888                                    |                                                | 0x8888 6666                                    |                                                | 0x6666 8888                                    |

Table 18-20: 16-bit Receive Mode with Packing Disabled

| Pin Data (16 bits)   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=X SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=X SIGNEXT=X   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=X SIGNEXT=X   |
|----------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x1111               | 0x1111                                         | 0x1111                                         |                                                |
| 0x2222               | 0x2222                                         |                                                | 0x2222                                         |
| 0x3333               | 0x3333                                         | 0x3333                                         |                                                |
| 0x4444               | 0x4444                                         |                                                | 0x4444                                         |
| 0x5555               | 0x5555                                         | 0x5555                                         |                                                |
| 0x6666               | 0x6666                                         |                                                | 0x6666                                         |
| 0x7777               | 0x7777                                         | 0x7777                                         |                                                |
| 0x8888               | 0x8888                                         |                                                | 0x8888                                         |

## Configuring 18-Bit Receive Mode

For 18-bit non-split receive mode, if EPPI\_CTL.PACKEN =0, the EPPI zero-fills or sign-extends the incoming data into a 32-bit word. If EPPI\_CTL.PACKEN =1, the EPPI first zero-fills or sign-extends the incoming data to 24 bits, and then packs four such 24-bit data words into three 32-bit words. Alternate even or odd samples can be

skipped based on the EPPI\_CTL.SKIPEN and EPPI\_CTL.SKIPEO bits. The EPPI\_CTL.SWAPEN bit has no effect.

Table 18-21: 18-bit Receive Mode with Packing Disabled

| Pin Data (18 bits)   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=X SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=X SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=X SIGNEXT=0   |
|----------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x0 6666             | 0x0000 6666                                    | 0x0000 6666                                    |                                                |
| 0x1 7777             | 0x0001 7777                                    |                                                | 0x0001 7777                                    |
| 0x2 8888             | 0x0002 8888                                    | 0x0002 8888                                    |                                                |
| 0x3 9999             | 0x0003 9999                                    |                                                | 0x0003 9999                                    |

Table 18-22: 18-bit Receive Mode with Packing Enabled

| Pin Data (18 bits)   | DMADATA SKIPEN=0 SKIPEO=X SWAPEN=X SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=1 SWAPEN=X SIGNEXT=0   | DMADATA SKIPEN=1 SKIPEO=0 SWAPEN=X SIGNEXT=0   |
|----------------------|------------------------------------------------|------------------------------------------------|------------------------------------------------|
| 0x0 1122             |                                                |                                                |                                                |
| 0x1 3344             | 0x4400 1122                                    |                                                |                                                |
| 0x2 5566             | 0x5566 0133                                    | 0x6600 1122                                    |                                                |
| 0x3 7788             | 0x0377 8802                                    |                                                | 0x8801 3344                                    |
| 0x0 99AA             |                                                | 0x99AA 0255                                    |                                                |
| 0x1 BBCC             | 0xCC00 99AA                                    |                                                | 0xBBCC 0377                                    |
| 0x2 DDEE             | 0xDDEE 01BB                                    | 0x02DD EE00                                    |                                                |
| 0x3 FF12             | 0x03FF 122D                                    |                                                | 0x03FF 1201                                    |

## Configuring 8-Bit Split Receive Mode

For 8-bit split receive mode, the EPPI\_CTL.PACKEN and EPPI\_CTL.SIGNEXT bits are not valid. The EPPI always packs 4 bytes of data into one 32-bit word.

Table 18-23: 8-bit Split Receive Mode with SKIPEN = 0 and SWAPEN = 0

| Pin Data (8 bits)   | SPLTEO=1 SUBSPLTODD= 0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD= 0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD= 0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD= 1 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD= 1 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD= 1 SWAPEN=0 SKIPEN=0 SKIPEO=X   |
|---------------------|-----------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------|
|                     | DMACFG=1                                            | DMACFG=1                                            | DMACFG=0                                            | DMACFG=1                                            | DMACFG=1                                            | DMACFG=0                                            |
|                     | Primary DMA Channel                                 | Secondary DMA Channel                               | Primary DMA Channel                                 | Primary DMA Channel                                 | Secondary DMA Channel                               | Primary DMA Channel                                 |
| V 0                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| Y 0                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| U 0                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| Y 1                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| V 1                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| Y 2                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| U 1                 |                                                     | U 1 V 1 U 0 V 0                                     | U 1 V 1 U 0 V 0                                     |                                                     |                                                     |                                                     |
| Y 3                 | Y 3 Y 2 Y 1 Y 0                                     |                                                     | Y 3 Y 2 Y 1 Y 0                                     | Y 3 Y 2 Y 1 Y 0                                     |                                                     | Y 3 Y 2 Y 1 Y 0                                     |
| V 2                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| Y 4                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| U 2                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| Y 5                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| V 3                 |                                                     |                                                     |                                                     |                                                     | V 3 V 2 V 1 V 0                                     | V 3 V 2 V 1 V 0                                     |
| Y 6                 |                                                     |                                                     |                                                     |                                                     |                                                     |                                                     |
| U 3                 |                                                     | U 3 V 3 U 2 V 2                                     | U 3 V 3 U 2 V 2                                     |                                                     | U 3 U 2 U 1 U 0                                     |                                                     |
| Y 7                 | Y 7 Y 6 Y 5 Y 4                                     |                                                     | Y 7 Y 6 Y 5 Y 4                                     | Y 7 Y 6 Y 5 Y 4                                     |                                                     | Y 7 Y 6 Y 5 Y 4                                     |
| V 4                 |                                                     |                                                     |                                                     |                                                     |                                                     | U 3 U 2 U 1 U 0                                     |

Table 18-24: 8-bit Split Receive Mode with SKIPEN = 0 and SWAPEN = 1

| Pin Data (8 bits)   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=1 SKIPEN=0 SKIPEO=X   |
|---------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|
| Pin Data (8 bits)   | DMACFG=1                                           | DMACFG=1                                           | DMACFG=0                                           | DMACFG=1                                           | DMACFG=1                                           | DMACFG=0                                           |
| Pin Data (8 bits)   | PRIMARY DMA CHANNEL                                | SECONDARY DMA CHANNEL                              | PRIMARY DMA CHANNEL                                | PRIMARY DMA CHANNEL                                | SECONDARY DMA CHANNEL                              | PRIMARY DMA CHANNEL                                |
| V 0                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| Y 0                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| U 0                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| Y 1                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| V 1                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| Y 2                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| U 1                 |                                                    | V 0 U 0 V 1 U 1                                    | V 0 U 0 V 1 U 1                                    |                                                    |                                                    |                                                    |
| Y 3                 | Y 0 Y 1 Y 2 Y 3                                    |                                                    | Y 0 Y 1 Y 2 Y 3                                    | Y 0 Y 1 Y 2 Y 3                                    |                                                    | Y 0 Y 1 Y 2 Y 3                                    |
| V 2                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| Y 4                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| U 2                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| Y 5                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| V 3                 |                                                    |                                                    |                                                    |                                                    | V 0 V 1 V 2 V 3                                    | V 0 V 1 V 2 V 3                                    |
| Y 6                 |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| U 3                 |                                                    | V 2 U 2 V 3 U 3                                    | V 2 U 2 V 3 U 3                                    |                                                    | U 0 U 1 U 2 U 3                                    |                                                    |
| Y 7                 | Y 4 Y 5 Y 6 Y 7                                    |                                                    | Y 4 Y 5 Y 6 Y 7                                    | Y 4 Y 5 Y 6 Y 7                                    |                                                    | Y 4 Y 5 Y 6 Y 7                                    |
| V 4                 |                                                    |                                                    |                                                    |                                                    |                                                    | U 0 U 1 U 2 U 3                                    |

When the bits settings are EPPI\_CTL.SPLTEO =1, EPPI\_CTL.SUBSPLTODD =1 and EPPI\_CTL.DMACFG =0, the EPPI packs the second Chroma component sent over the DMA bus completely before the Luma component. However, it is intentionally held until that previous word is moved out. This functionality allows the separation of Luma and Chroma values into individual buffers when using 2D-DMA. The second Chroma component is U0U1U2U3 in the 8-bit Split Receive Mode with SKIPEN = 0 and SWAPEN = 0 and 8-bit Split Receive Mode with SKIPEN = 0 and SWAPEN = 1 tables. The Luma component is Y4Y5Y6Y7 in the 8-bit Split Receive Mode with SKIPEN = 0 and SWAPEN = 1 table.

## Configuring 10/12/14/16-Bit Split Receive Mode with SPLTWRD=0

For 16-bit split receive mode, the EPPI\_CTL.PACKEN bit is not valid. The EPPI always packs two 16-bit words into one 32-bit word. For 10, 12, or 14-bit split receive modes, the EPPI first either sign-extends or zero-fills the incoming data into a 16-bit word. The EPPI then packs two of these words into one 32-bit word to send to the DMA.

Table 18-25: 16-bit Split Receive Mode with SPLTWRD = 0, SKIPEN = 0 and SWAPEN = 0

| Pin Data (16 bits)   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   |
|----------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|
|                      | DMACFG=1                                           | DMACFG=1                                           | DMACFG=0                                           | DMACFG=1                                           | DMACFG=1                                           | DMACFG=0                                           |
|                      | Primary DMA Channel                                | Secondary DMA Channel                              | Primary DMA Channel                                | Primary DMA Channel                                | Secondary DMA Channel                              | Primary DMA Channel                                |
| V 0                  |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| Y 0                  |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| U 0                  |                                                    | U 0 V 0                                            | U 0 V 0                                            |                                                    |                                                    |                                                    |
| Y 1                  | Y 1 Y 0                                            |                                                    | Y 1 Y 0                                            | Y 1 Y 0                                            |                                                    | Y 1 Y 0                                            |
| V 1                  |                                                    |                                                    |                                                    |                                                    | V 1 V 0                                            | V 1 V 0                                            |
| Y 2                  |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| U 1                  |                                                    | U 1 V 1                                            | U 1 V 1                                            |                                                    | U 1 U 0                                            |                                                    |
| Y 3                  | Y 3 Y 2                                            |                                                    | Y 3 Y 2                                            | Y 3 Y 2                                            |                                                    | Y 3 Y 2                                            |
| V 2                  |                                                    |                                                    |                                                    |                                                    |                                                    | U 1 U 0                                            |

Table 18-26: 16-bit Split Receive Mode with SPLTWRD = 0, SKIPEN = 0 and SWAPEN = 1

| Pin Data (16 bits)   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=1 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=1 SWAPEN=1 SKIPEN=0 SKIPEO=X   |
|----------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|
|                      | DMACFG=1                                           | DMACFG=1                                           | DMACFG=0                                           | DMACFG=1                                           | DMACFG=1                                           | DMACFG=0                                           |
|                      | PRIMARY DMA CHANNEL                                | SECONDARY DMA CHANNEL                              | PRIMARY DMA CHANNEL                                | PRIMARY DMA CHANNEL                                | SECONDARY DMA CHANNEL                              | PRIMARY DMA CHANNEL                                |
| V 0                  |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| Y 0                  |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| U 0                  |                                                    | V 0 U 0                                            | V 0 U 0                                            |                                                    |                                                    |                                                    |
| Y 1                  | Y 0 Y 1                                            |                                                    | Y 0 Y 1                                            | Y 0 Y 1                                            |                                                    | Y 0 Y 1                                            |
| V 1                  |                                                    |                                                    |                                                    |                                                    | V 0 V 1                                            | V 0 V 1                                            |
| Y 2                  |                                                    |                                                    |                                                    |                                                    |                                                    |                                                    |
| U 1                  |                                                    | V 1 U 1                                            | V 1 U 1                                            |                                                    | U 0 U 1                                            |                                                    |
| Y 3                  | Y 2 Y 3                                            |                                                    | Y 2 Y 3                                            | Y 2 Y 3                                            |                                                    | Y 2 Y 3                                            |
| V 2                  |                                                    |                                                    |                                                    |                                                    |                                                    | U 0 U 1                                            |

## Configuring 16-Bit Split Receive Mode with SPLTWRD=1

For 16-bit split receive mode, the EPPI\_CTL.PACKEN bit is not valid. The EPPI always packs two 16-bit words into one 32-bit word. The EPPI\_CTL.SPLTWRD bit is only valid when the EPPI\_CTL.DLEN bit =16 bits.

Table 18-27: 16-bit Split Receive Mode with SPLTWRD = 1, SKIPEN = 0 and SWAPEN = 0

| Pin Data (16 bits)   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLT_EVEN_ODD=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLT_EVEN_ODD=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLT_EVEN_ODD=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   |
|----------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|-----------------------------------------------------------|-----------------------------------------------------------|-----------------------------------------------------------|
|                      | DMACFG=1                                           | DMACFG=1                                           | DMACFG=0                                           | DMACFG=1                                                  | DMACFG=1                                                  | DMACFG=0                                                  |
|                      | Primary DMA Channel                                | Secondary DMA Channel                              | Primary DMA Channel                                | Primary DMA Channel                                       | Secondary DMA Channel                                     | Primary DMA Channel                                       |
| V 0 Y 0              |                                                    |                                                    |                                                    |                                                           |                                                           |                                                           |

Table 18-27: 16-bit Split Receive Mode with SPLTWRD = 1, SKIPEN = 0 and SWAPEN = 0 (Continued)

| Pin Data (16 bits)   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLTEO=1 SUBSPLTODD=0 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLT_EVEN_ODD=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLT_EVEN_ODD=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   | SPLT_EVEN_ODD=1 SUBSPLTODD=1 SWAPEN=0 SKIPEN=0 SKIPEO=X   |
|----------------------|----------------------------------------------------|----------------------------------------------------|----------------------------------------------------|-----------------------------------------------------------|-----------------------------------------------------------|-----------------------------------------------------------|
|                      | DMACFG=1                                           | DMACFG=1                                           | DMACFG=0                                           | DMACFG=1                                                  | DMACFG=1                                                  | DMACFG=0                                                  |
|                      | Primary DMA Channel                                | Secondary DMA Channel                              | Primary DMA Channel                                | Primary DMA Channel                                       | Secondary DMA Channel                                     | Primary DMA Channel                                       |
| U 0 Y 1              |                                                    |                                                    |                                                    |                                                           |                                                           |                                                           |
| V 1 Y 2              |                                                    |                                                    |                                                    |                                                           |                                                           |                                                           |
| U 1 Y 3              | Y 3 Y 2 Y 1 Y 0                                    | U 1 V 1 U 0 V 0                                    | Y 3 Y 2 Y 1 Y 0                                    | Y 3 Y 2 Y 1 Y 0                                           |                                                           | Y 3 Y 2 Y 1 Y 0                                           |
| V 2 Y 4              |                                                    |                                                    | U 1 V 1 U 0 V 0                                    |                                                           |                                                           |                                                           |
| U 2 Y 5              |                                                    |                                                    |                                                    |                                                           |                                                           |                                                           |
| V 3 Y 6              |                                                    |                                                    |                                                    |                                                           | V 3 V 2 V 1 V 0                                           | V 3 V 2 V 1 V 0                                           |
| U 3 Y 7              | Y 7 Y 6 Y 5 Y 4                                    | U 3 V 3 U 2 V 2                                    | Y 7 Y 6 Y 5 Y 4                                    | Y 7 Y 6 Y 5 Y 4                                           | U 3 U 2 U 1 U 0                                           | Y 7 Y 6 Y 5 Y 4                                           |
| V 4 Y 8              |                                                    |                                                    | U 3 V 3 U 2 V 2                                    |                                                           |                                                           | U 3 U 2 U 1 U 0                                           |

## Configuring 8-Bit Transmit Mode

For 8-bit non-split transmit mode, if the EPPI\_CTL.PACKEN bit =1, the DMA is a 32-bit DMA and the EPPI unpacks the 32-bit word from memory into 4 bytes to transmit. The EPPI transmits either the MSBs or the LSBs as the first data, depending on the EPPI\_CTL.SWAPEN bit setting. If EPPI\_CTL.PACKEN =0, the DMA is a 16bit DMA and the EPPI transmits the lower 8 bits. The EPPI\_CTL.SWAPEN bit has no effect when EPPI\_CTL.PACKEN =0.

Table 18-28: 8-bit Transmit Mode with Packing Enabled

| DMAData (32 bits)   | Pin Data when SWAPEN=0   | Pin Data when SWAPEN=1   |
|---------------------|--------------------------|--------------------------|
| 0x11223344          | 0x44                     | 0x11                     |
| 0x55667788          | 0x33                     | 0x22                     |
|                     | 0x22                     | 0x33                     |
|                     | 0x11                     | 0x44                     |
|                     | 0x88                     | 0x55                     |
|                     | 0x77                     | 0x66                     |
|                     | 0x66                     | 0x77                     |

Table 18-28: 8-bit Transmit Mode with Packing Enabled (Continued)

| DMAData (32 bits)   | Pin Data when SWAPEN=0   | Pin Data when SWAPEN=1   |
|---------------------|--------------------------|--------------------------|
|                     | 0x55                     | 0x88                     |

Table 18-29: Data Sent in 8-bit Transmit Mode with Packing Disabled

| DMAData (16 bits)   | Pin Data SWAPEN=X   |
|---------------------|---------------------|
| 0x1234              | 0x34                |
| 0x2345              | 0x45                |
| 0x3456              | 0x56                |

## Configuring 10/12/14-Bit Transmit Modes

For 10, 12, or 14-bit non-split transmit modes, if the EPPI\_CTL.PACKEN bit =1, the DMA is a 32-bit DMA. The EPPI unpacks the 32-bit word from memory into two 16-bit data words. The EPPI then transmits the required LSBs from each data word. The EPPI transmits either the most significant word or the least significant word as the first data, depending on the EPPI\_CTL.SWAPEN bit setting. If EPPI\_CTL.PACKEN =0, the DMA is a 16-bit DMA and the EPPI transmits the required LSBs. The EPPI\_CTL.SWAPEN bit has no effect when the EPPI\_CTL.PACKEN bit =0.

Table 18-30: 10-bit Transmit Mode with Packing Enabled

| DMAData (32 bits)   | Pin Data when SWAPEN=0   | Pin Data when SWAPEN=1   |
|---------------------|--------------------------|--------------------------|
| 0x1111 2222         | 0x222                    | 0x111                    |
| 0x3333 4444         | 0x111                    | 0x222                    |
|                     | 0x044                    | 0x333                    |
|                     | 0x333                    | 0x044                    |

Table 18-31: 10-bit Transmit Mode with Packing Disabled

| DMAData (16 bits)   | Pin Data SWAPEN=X   |
|---------------------|---------------------|
| 0x1234              | 0x234               |
| 0x2345              | 0x345               |
| 0x3456              | 0x056               |
| 0x4567              | 0x167               |

## Configuring 16-Bit Transmit Mode

For 16-bit non-split transmit mode, if the EPPI\_CTL.PACKEN bit =1, the DMA is a 32-bit DMA. The EPPI unpacks the 32-bit word from memory into two 16-bit data words to transmit. The EPPI transmits either the MSBs or the LSBs as the first data, depending on the EPPI\_CTL.SWAPEN bit setting. If the EPPI\_CTL.PACKEN bit =0, the DMA is a 16-bit DMA and the EPPI transmits the data as is. The EPPI\_CTL.SWAPEN has no effect when EPPI\_CTL.PACKEN bit =0.

Table 18-32: 16-bit Transmit Mode with Packing Enabled

| DMAData (32 bits)   | Pin Data when SWAPEN=0   | Pin Data when SWAPEN=1   |
|---------------------|--------------------------|--------------------------|
| 0x1111 2222         | 0x2222                   | 0x1111                   |
| 0x3333 4444         | 0x1111                   | 0x2222                   |
|                     | 0x4444                   | 0x3333                   |
|                     | 0x3333                   | 0x4444                   |

Table 18-33: 16-bit Transmit Mode with Packing Disabled

| DMAData (16 bits)   | Pin Data SWAPEN=X   |
|---------------------|---------------------|
| 0x1234              | 0x1234              |
| 0x2345              | 0x2345              |
| 0x3456              | 0x3456              |

## Configuring 18-Bit Transmit Mode

For 18-bit transmit mode, if the EPPI\_CTL.PACKEN bit =1, the DMA is a 32-bit DMA and the EPPI unpacks the 32-bit word from memory. In this mode, when EPPI\_CTL.RGBFMTEN is set, the least significant 2 bits of R, G, and B are dropped.

Table 18-34: 18-bit Transmit Mode with Packing Enabled

| DMAData     | Pin Data (18-bits)   | Pin Data (18-bits)   |
|-------------|----------------------|----------------------|
|             | RGBFMTEN=0           | RGBFMTEN=1           |
| 0x0123 4567 | 0x3 4567             | 0x0 8459             |
| 0x89AB CDEF | 0x1 EF01             | 0x3 3EC0             |
| 0x0123 4567 | 0x3 89AB             | 0x1 98AA             |
|             | 0x1 2345             | 0x0 0211             |

Table 18-35: 18-bit Transmit Mode with Packing Disabled

| DMAData     | Pin Data (18-bits)   | Pin Data (18-bits)   |
|-------------|----------------------|----------------------|
|             | RGBFMTEN=0           | RGBFMTEN=1           |
| 0x0123 4567 | 0x3 4567             | 0x0 8459             |
| 0x89AB CDEF | 0x3 CDEF             | 0x2 ACFB             |
| 0x0123 4567 | 0x3 4567             | 0x0 8459             |

## Configuring 8-Bit Split Transmit Mode

For 8-bit split transmit mode, the EPPI\_CTL.PACKEN bit is not valid. The EPPI always unpacks the 32-bit DMA data into 4 bytes to transmit.

Table 18-36: 8-bit Split T ransmit Mode with SPLTEO=1, SUBSPLTODD=0 and SWAPEN=0

| DMACFG=1            | DMACFG=1            | DMACFG=1          | DMACFG=0            | DMACFG=0          |
|---------------------|---------------------|-------------------|---------------------|-------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (8 bits) | DMA0 DATA (32 bits) | Pin Data (8 bits) |
| Y 3 Y 2 Y 1 Y 0     | U 1 V 1 U 0 V 0     | V 0               | U 1 V 1 U 0 V 0     | V 0               |
| Y 7 Y 6 Y 5 Y 4     | U 3 V 3 U 2 V 2     | Y 0               | Y 3 Y 2 Y 1 Y 0     | Y 0               |
|                     |                     | U 0               | U 3 V 3 U 2 V 2     | U 0               |
|                     |                     | Y 1               | Y 7 Y 6 Y 5 Y 4     | Y 1               |
|                     |                     | V 1               |                     | V 1               |
|                     |                     | Y 2               |                     | Y 2               |
|                     |                     | U 1               |                     | U 1               |
|                     |                     | Y 3               |                     | Y 3               |
|                     |                     | V 2               |                     | V 2               |
|                     |                     | Y 4               |                     | Y 4               |
|                     |                     | U 2               |                     | U 2               |
|                     |                     | Y 5               |                     | Y 5               |
|                     |                     | V 3               |                     | V 3               |
|                     |                     | Y 6               |                     | Y 6               |
|                     |                     | U 3               |                     | U 3               |
|                     |                     | Y 7               |                     | Y 7               |

Table 18-37: 8-bit Split T ransmit Mode with SPLTEO=1, SUBSPLTODD=1 and SWAPEN=0

| DMACFG=1            | DMACFG=1            | DMACFG=1          | DMACFG=0            | DMACFG=0          |
|---------------------|---------------------|-------------------|---------------------|-------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (8 bits) | DMA0 DATA (32 bits) | Pin Data (8 bits) |
| Y 3 Y 2 Y 1 Y 0     | V 3 V 2 V 1 V 0     | V 0               | V 3 V 2 V 1 V 0     | V 0               |
| Y 7 Y 6 Y 5 Y 4     | U 3 U 2 U 1 U 0     | Y 0               | Y 3 Y 2 Y 1 Y 0     | Y 0               |
|                     | V 7 V 6 V 5 V 4     | U 0               | U 3 U 2 U 1 U 0     | U 0               |
|                     | U 7 U 6 U 5 U 4     | Y 1               | Y 7 Y 6 Y 5 Y 4     | Y 1               |
|                     |                     | V 1               |                     | V 1               |
|                     |                     | Y 2               |                     | Y 2               |
|                     |                     | U 1               |                     | U 1               |
|                     |                     | Y 3               |                     | Y 3               |
|                     |                     | V 2               |                     | V 2               |

Table 18-37: 8-bit Split T ransmit Mode with SPLTEO=1, SUBSPLTODD=1 and SWAPEN=0 (Continued)

| DMACFG=1            | DMACFG=1            | DMACFG=1          | DMACFG=0            | DMACFG=0          |
|---------------------|---------------------|-------------------|---------------------|-------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (8 bits) | DMA0 DATA (32 bits) | Pin Data (8 bits) |
|                     |                     | Y 4               |                     | Y 4               |
|                     |                     | U 2               |                     | U 2               |
|                     |                     | Y 5               |                     | Y 5               |
|                     |                     | V 3               |                     | V 3               |
|                     |                     | Y 6               |                     | Y 6               |
|                     |                     | U 3               |                     | U 3               |
|                     |                     | Y 7               |                     | Y 7               |

Table 18-38: 8-bit Split T ransmit Mode with SPLTEO=1, SUBSPLTODD=0 and SWAPEN=1

| DMACFG=1            | DMACFG=1            | DMACFG=1          | DMACFG=0            | DMACFG=0          |
|---------------------|---------------------|-------------------|---------------------|-------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (8 bits) | DMA0 DATA (32 bits) | Pin Data (8 bits) |
| Y 3 Y 2 Y 1 Y 0     | U 1 V 1 U 0 V 0     | U 1               | U 1 V 1 U 0 V 0     | U 1               |
| Y 7 Y 6 Y 5 Y 4     | U 3 V 3 U 2 V 2     | Y 3               | Y 3 Y 2 Y 1 Y 0     | Y 3               |
|                     |                     | V 1               | U 3 V 3 U 2 V 2     | V 1               |
|                     |                     | Y 2               | Y 7 Y 6 Y 5 Y 4     | Y 2               |
|                     |                     | U 0               |                     | U 0               |
|                     |                     | Y 1               |                     | Y 1               |
|                     |                     | V 0               |                     | V 0               |
|                     |                     | Y 0               |                     | Y 0               |
|                     |                     | U 3               |                     | U 3               |
|                     |                     | Y 7               |                     | Y 7               |
|                     |                     | V 3               |                     | V 3               |
|                     |                     | Y 6               |                     | Y 6               |
|                     |                     | U 2               |                     | U 2               |
|                     |                     | Y 5               |                     | Y 5               |
|                     |                     | V 2               |                     | V 3               |
|                     |                     | Y 4               |                     | Y 4               |

Table 18-39: 8-bit Split T ransmit Mode with SPLTEO=1, SUBSPLTODD=1, and SWAPEN=1

| DMACFG=1            | DMACFG=1            | DMACFG=1          | DMACFG=0            | DMACFG=0          |
|---------------------|---------------------|-------------------|---------------------|-------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (8 bits) | DMA0 DATA (32 bits) | Pin Data (8 bits) |
| Y 3 Y 2 Y 1 Y 0     | V 3 V 2 V 1 V 0     | V 3               | V 3 V 2 V 1 V 0     | V 3               |
| Y 7 Y 6 Y 5 Y 4     | U 3 U 2 U 1 U 0     | Y 3               | Y 3 Y 2 Y 1 Y 0     | Y 3               |
|                     | V 7 V 6 V 5 V 4     | U 3               | U 3 V 3 U 2 V 2     | U 3               |
|                     | U 7 U 6 U 5 U 4     | Y 2               | Y 7 Y 6 Y 5 Y 4     | Y 2               |
|                     |                     | V 2               |                     | V 2               |
|                     |                     | Y 1               |                     | Y 1               |
|                     |                     | U 2               |                     | U 2               |
|                     |                     | Y 0               |                     | Y 0               |
|                     |                     | V 1               |                     | V 1               |
|                     |                     | Y 7               |                     | Y 7               |
|                     |                     | U 1               |                     | U 1               |
|                     |                     | Y 6               |                     | Y 6               |
|                     |                     | V 0               |                     | V 0               |
|                     |                     | Y 5               |                     | Y 5               |
|                     |                     | U 0               |                     | U 0               |
|                     |                     | Y 4               |                     | Y 4               |

## Configuring 10/12/14/16-Bit Transmit Mode with SPLTWRD=0

For 16-bit split transmit mode, the EPPI\_CTL.PACKEN bit is not valid. The EPPI always unpacks the 32-bit DMA data into two 16-bit words to transmit. For 10, 12, or 14-bit split transmit modes, the EPPI first unpacks the data in the same way as for 16-bit transmit mode. But, the EPPI transmits only the required number of LSBs.

Table 18-40: 16-bit Split T ransmit Mode with SPLTEO = 1, SUBSPLTODD = 0, and SWAPEN = 0

| DMACFG = 1          | DMACFG = 1          | DMACFG = 1         | DMACFG = 0          | DMACFG = 0         |
|---------------------|---------------------|--------------------|---------------------|--------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (16 bits) | DMA0 DATA (32 bits) | Pin Data (16 bits) |
| Y 1 Y 0             | U 0 V 0             | V 0                | U 0 V 0             | V 0                |
| Y 3 Y 2             | U 1 V 1             | Y 0                | Y 1 Y 0             | Y 0                |
|                     |                     | U 0                | U 1 V 1             | U 0                |
|                     |                     | Y 1                | Y 3 Y 2             | Y 1                |
|                     |                     | V 1                |                     | V 1                |

Table 18-40: 16-bit Split T ransmit Mode with SPLTEO = 1, SUBSPLTODD = 0, and SWAPEN = 0 (Continued)

| DMACFG = 1          | DMACFG = 1          | DMACFG = 1         | DMACFG = 0          | DMACFG = 0         |
|---------------------|---------------------|--------------------|---------------------|--------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (16 bits) | DMA0 DATA (32 bits) | Pin Data (16 bits) |
|                     |                     | Y 2                |                     | Y 2                |
|                     |                     | U 1                |                     | U 1                |
|                     |                     | Y 3                |                     | Y 3                |

Table 18-41: 16-bit Split T ransmit Mode with SPLTEO = 1, SUBSPLTODD = 1, and SWAPEN = 0

| DMACFG = 1          | DMACFG = 1          | DMACFG = 1         | DMACFG = 0          | DMACFG = 0         |
|---------------------|---------------------|--------------------|---------------------|--------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (16 bits) | DMA0 DATA (32 bits) | Pin Data (16 bits) |
| Y 1 Y 0             | V 1 V 0             | V 0                | V 1 V 0             | V 0                |
| Y 3 Y 2             | U 1 U 0             | Y 0                | Y 1 Y 0             | Y 0                |
|                     | V 3 V 2             | U 0                | U 1 U 0             | U 0                |
|                     | U 3 U 2             | Y 1                | Y 3 Y 2             | Y 1                |
|                     |                     | V 1                |                     | V 1                |
|                     |                     | Y 2                |                     | Y 2                |
|                     |                     | U 1                |                     | U 1                |
|                     |                     | Y 3                |                     | Y 3                |

Table 18-42: 16-bit Split T ransmit Mode with SPLTEO = 1, SUBSPLTODD = 0, and SWAPEN = 1

| DMACFG = 1          | DMACFG = 1          | DMACFG = 1         | DMACFG = 0          | DMACFG = 0         |
|---------------------|---------------------|--------------------|---------------------|--------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (16 bits) | DMA0 DATA (32 bits) | Pin Data (16 bits) |
| Y 1 Y 0             | V 0 U 0             | V 0                | V 0 U 0             | V 0                |
| Y 3 Y 2             | V 1 U 1             | Y 1                | Y 1 Y 0             | Y 1                |
|                     |                     | U 0                | V 1 U 1             | U 0                |
|                     |                     | Y 0                | Y 3 Y 2             | Y 0                |
|                     |                     | V 1                |                     | V 1                |
|                     |                     | Y 3                |                     | Y 3                |
|                     |                     | U 1                |                     | U 1                |
|                     |                     | Y 2                |                     | Y 2                |

Table 18-43: 16-bit Split T ransmit Mode with SPLTEO = 1, SUBSPLTODD = 1, and SWAPEN = 1

| DMACFG = 1          | DMACFG = 1          | DMACFG = 1         | DMACFG = 0          | DMACFG = 0         |
|---------------------|---------------------|--------------------|---------------------|--------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (16 bits) | DMA0 DATA (32 bits) | Pin Data (16 bits) |
| Y 1 Y 0             | V 1 V 0             | V 1                | V 1 V 0             | V 1                |
| Y 3 Y 2             | U 1 U 0             | Y 1                | Y 1 Y 0             | Y 1                |
|                     | V 3 V 2             | U 1                | U 1 U 0             | U 1                |
|                     | U 3 U 2             | Y 0                |                     | Y 0                |
|                     |                     | V 0                |                     | V 0                |
|                     |                     | Y 3                |                     | Y 1                |
|                     |                     | U 0                |                     | U 0                |
|                     |                     | Y 2                |                     | Y 2                |

## Configuring 16-Bit Split Transmit Mode with SPLTWRD=1

For 16-bit split transmit mode, the EPPI\_CTL.PACKEN bit is not valid. The EPPI always unpacks the 32-bit DMA data into two 16-bit words to transmit. The EPPI\_CTL.SPLTWRD bit is only valid when the EPPI\_CTL.DLEN bit =16 bits.

Table 18-44: 16-bit Split T ransmit Mode with SPLTWRD = 1, SUBSPLTODD = 0, and SWAPEN = 0

| DMACFG = 1          | DMACFG = 1          | DMACFG = 1         | DMACFG = 0          | DMACFG = 0         |
|---------------------|---------------------|--------------------|---------------------|--------------------|
| DMA0 DATA (32 bits) | DMA1 DATA (32 bits) | Pin Data (16 bits) | DMA0 DATA (32 bits) | Pin Data (16 bits) |
| Y 3 Y 2 Y 1 Y 0     | U 1 V 1 U 0 V 0     | V 0 Y 0            | U 1 V 1 U 0 V 0     | V 0 Y 0            |
| Y 7 Y 6 Y 5 Y 4     | U 3 V 3 U 2 V 2     | U 0 Y 1            | Y 3 Y 2 Y 1 Y 0     | U 0 Y 1            |
|                     |                     | V 1 Y 2            | U 3 V 3 U 2 V 2     | V 1 Y 2            |
|                     |                     | U 1 Y 3            | Y 7 Y 6 Y 5 Y 4     | U 1 Y 3            |
|                     |                     | V 2 Y 4            |                     | V 2 Y 4            |
|                     |                     | U 2 Y 5            |                     | U 2 Y 5            |
|                     |                     | V 3 Y 6            |                     | V 3 Y 6            |
|                     |                     | U 3 Y 7            |                     | U 3 Y 7            |

Table 18-45: 16-bit Split T ransmit Mode with SPLTWRD = 1, SUBSPLTODD = 1, and SWAPEN = 0

| DMACFG = 1                | DMACFG = 1                  | DMACFG = 1         | DMACFG = 0          | DMACFG = 0         |
|---------------------------|-----------------------------|--------------------|---------------------|--------------------|
| PRIMARY DMADATA (32 bits) | SECONDARYDMA DATA (32 bits) | Pin Data (16 bits) | DMA0 DATA (32 bits) | Pin Data (16 bits) |
| Y 3 Y 2 Y 1 Y 0           | V 3 V 2 V 1 V 0             | V 0 Y 0            | V 3 V 2 V 1 V 0     | V 0 Y 0            |

Table 18-45: 16-bit Split T ransmit Mode with SPLTWRD = 1, SUBSPLTODD = 1, and SWAPEN = 0 (Continued)

| DMACFG = 1                | DMACFG = 1                  | DMACFG = 1         | DMACFG = 0          | DMACFG = 0         |
|---------------------------|-----------------------------|--------------------|---------------------|--------------------|
| PRIMARY DMADATA (32 bits) | SECONDARYDMA DATA (32 bits) | Pin Data (16 bits) | DMA0 DATA (32 bits) | Pin Data (16 bits) |
| Y 7 Y 6 Y 5 Y 4           | U 3 U 2 U 1 U 0             | U 0 Y 1            | Y 3 Y 2 Y 1 Y 0     | U 0 Y 1            |
|                           | V 7 V 6 V 5 V 4             | V 1 Y 2            | U 3 U 2 U 1 U 0     | V 1 Y 2            |
|                           | U 7 U 6 U 5 U 4             | U 1 Y 3            | Y 7 Y 6 Y 5 Y 4     | U 1 Y 3            |
|                           |                             | V 2 Y 4            |                     | V 2 Y 4            |
|                           |                             | U 2 Y 5            |                     | U 2 Y 5            |
|                           |                             | V 3 Y 6            |                     | V 3 Y 6            |
|                           |                             | U 3 Y 7            |                     | U 3 Y 7            |

## EPPI Programming Concepts

This section provides information on SMPTE programming.

## SMPTE Modes Programming

The programming model of SMPTE modes is similar to ITU Modes. All programming modes pertaining to ITU modes like XFRTYPE, FSCFG, FLDSEL, and BLANKGEN hold true for SMPTE modes as well. The only difference is that since ITU modes use Y-Cr/Cb interleaved data and SMPTE use parallel Y-Cr/Cb data, SPLTWRD could be set while operating in SMPTE modes. The Programming Modes for SMPTE Formats table describes the programming modes for different SMPTE formats.

Table 18-46: Programming Modes for SMPTE Formats

| SMPTE Format   | SMPTE Channel Width   | EPPI Input Bit Width           | EPPI Mode                                 | Remarks                                     |
|----------------|-----------------------|--------------------------------|-------------------------------------------|---------------------------------------------|
| 296M           | 8 8                   | 16 Cr/Cb - [15:8] Y - [7:0] 16 | DLEN = 16 bits SPLTWRD = 1 DLEN = 16 bits | SIGNEXT not supported SIGNEXT not supported |

## ADSP-SC58x EPPI Register Descriptions

Enhanced Parallel Peripheral Interface (EPPI) contains the following registers.

Table 18-47: ADSP-SC58x EPPI Register List

| Name           | Description                                                             |
|----------------|-------------------------------------------------------------------------|
| EPPI_CLKDIV    | Clock Divide Register                                                   |
| EPPI_CTL       | Control Register                                                        |
| EPPI_CTL2      | Control Register 2 Register                                             |
| EPPI_EVENCLIP  | Clipping Register for EVEN (Luma) Data Register                         |
| EPPI_FRAME     | Lines Per Frame Register                                                |
| EPPI_FS1_DLY   | Frame Sync 1 Delay Value Register                                       |
| EPPI_FS1_PASPL | FS1 Period Register / EPPI Active Samples Per Line Register             |
| EPPI_FS1_WLHB  | FS1 Width Register / EPPI Horizontal Blanking Samples Per Line Register |
| EPPI_FS2_DLY   | Frame Sync 2 Delay Value Register                                       |
| EPPI_FS2_PALPF | FS2 Period Register / EPPI Active Lines Per Field Register              |
| EPPI_FS2_WLVB  | FS2 Width Register / EPPI Lines Of Vertical Blanking Register           |
| EPPI_HCNT      | Horizontal Transfer Count Register                                      |
| EPPI_HDLY      | Horizontal Delay Count Register                                         |
| EPPI_IMSK      | Interrupt Mask Register                                                 |
| EPPI_LINE      | Samples Per Line Register                                               |
| EPPI_ODDCLIP   | Clipping Register for ODD(Chroma) Data Register                         |
| EPPI_STAT      | Status Register                                                         |
| EPPI_VCNT      | Vertical Transfer Count Register                                        |
| EPPI_VDLY      | Vertical Delay Count Register                                           |

## Clock Divide Register

The EPPI\_CLKDIV register provides the divisor for EPPI internal clock generation. The generated clock frequency is given by following formula:

EPPI\_CLK = (SCLK1\_0) / ( EPPI\_CLKDIV + 1)

Note that a value of 0xFFFF is invalid for the EPPI\_CLKDIV register.

Figure 18-14: EPPI\_CLKDIV Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000012_3e72b983b3a09a6c4313ca0e0550ef7c241341cdbd6c20860b77e8d9418311c7.png)

Table 18-48: EPPI\_CLKDIV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | VALUE      | Internal Clock Divider.   |

## Control Register

The EPPI\_CTL register configures the EPPI for operating mode, control signal polarities, and data width of the port.

Figure 18-15: EPPI\_CTL Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000013_e01200591cedd83d537326856620f8aa293c801deb3542efc1a0b217b69850e9.png)

Table 18-49: EPPI\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKGATEN   | Clock Gating Enable. The EPPI_CTL.CLKGATEN bit enables using the EPPI_FS3 pin as a clock gating pin. When EPPI_CTL.CLKGATEN is set, the EPPI_FS3 pin acts as a clock gating signal, and both the internal and external clock are gated. Note that the EPPI_FS3 pin gating signal is active low, and the EPPI_CTL.CLKGATEN selection is ignored if EPPI_CTL.MUXSEL is set or EPPI_CTL.FSCFG equals 0x3. |
| 30 (R/W)           | MUXSEL     | MUXSelect. The EPPI_CTL.MUXSEL bit enables multiplexing of a primary and alternate camera using the EPPI main data and clock lines. For more information on this feature, see the EPPI functional description.                                                                                                                                                                                         |
| 29 (R/W)           | DMAFINEN   | 1 Multiplexed Operation DMAFinish Enable. The EPPI_CTL.DMAFINEN bit selects whether or not the EPPI sends a finish com- mand (010) through the DDE COMMANDline soon after a frame/line is received completely.                                                                                                                                                                                         |
| 28 (R/W)           | DMACFG     | 0 No Finish Command 1 Enable Send Finish Command                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | One or Two DMAChannels Mode. The EPPI_CTL.DMACFG bit is valid only if EPPI_CTL.SPLTEO is set. If EPPI_CTL.DMACFG is set, the EPPI uses two DMAchannels. And, if EPPI_CTL.DMACFG is cleared, the EPPI uses only one DMAchannel. 0 PPI Uses One DMAChannel                                                                                                                                               |
| 27 (R/W)           | RGBFMTEN   | RGB Formatting Enable. For 16- or 18-bit transmit modes only, the EPPI_CTL.RGBFMTEN bit enables con- version of RGB888 from memory into RGB666 output data (18-bit transmit) or ena- bles conversion of RGB888 from memory into RGB565 output data (16-bit transmit). Note that EPPI_CTL.SPLTEO and EPPI_CTL.RGBFMTEN should never be set simultaneously.                                              |
|                    | 0          | Disable RGB Formatted Output                                                                                                                                                                                                                                                                                                                                                                           |

Table 18-49: EPPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | SPLTWRD    | Split Word. The EPPI_CTL.SPLTWRD bit selects split word data placement when the data length ( EPPI_CTL.DLEN ) selects 16-, 20-, or 24-bit data words. For all other EPPI_CTL.SPLTWRD values, the set or clear selections for EPPI_CTL.SPLTWRD produce the same result (act as though EPPI_CTL.SPLTWRD is cleared). For EPPI_CTL.SPLTWRD set, the EPPI_CTL.DLEN values below result in the follow- ing combinations of split words: DLEN Cr/Cb data Y data 16 PPI_DATA[15:8] PPI_DATA[7:0] 20 PPI_DATA[19:10] PPI_DATA[9:0] 24 PPI_DATA[23:12] PPI_DATA[11:0] 0 PPI_DATA has (DLEN-1) bits of Y or Cr or Cb |
| 25 (R/W)           | SUBSPLTODD | Sub-Split Odd Samples. The EPPI_CTL.SUBSPLTODD bit is valid only if EPPI_CTL.SPLTEO is set. If EPPI_CTL.SUBSPLTODD is set, the EPPI sub-splits the odd sub-stream, and packs them separately.                                                                                                                                                                                                                                                                                                                                                                                                              |
| 24 (R/W)           | SPLTEO     | Split Even and Odd Data Samples. If EPPI_CTL.SPLTEO is set, the EPPI splits the incoming data stream into two sub-streams, an even stream and an odd stream, and packs them separately.                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 23 (R/W)           | SWAPEN     | Swap Enable. The EPPI_CTL.SWAPEN selects whether or not to swap the order of the first data (most-significant bits versus least-significant bits) of the DMAword. For receive modes, the EPPI puts the first data in the most significant bits (if set) or puts the first data in the least significant bits (if cleared) of the DMAword. For transmit modes, the EPPI transmits the most significant bits in the DMAword as the first data (if set) or transmits the least significant bits in the DMAword as the first data (if cleared). 0 Disable                                                      |

Table 18-49: EPPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/W)           | PACKEN     | Pack/Unpack Enable. The EPPI_CTL.PACKEN select whether or not packing is enabled (for receive modes) and unpacking is enabled (for transmit modes). When this bit is set, EPPI transfer DMAis 32-bits wide. When this bit is cleared and the EPPI_CTL.DLEN is less than or equal to 16 bits, EPPI transfer DMAis 16-bits wide. For receive modes, if this bit is set, then the EPPI packs the incoming data into 32-bit words. If this bit is cleared, then the EPPI does not do any packing. For transmit modes, if this bit is set, then the EPPI always unpacks the 32-bit data | Pack/Unpack Enable. The EPPI_CTL.PACKEN select whether or not packing is enabled (for receive modes) and unpacking is enabled (for transmit modes). When this bit is set, EPPI transfer DMAis 32-bits wide. When this bit is cleared and the EPPI_CTL.DLEN is less than or equal to 16 bits, EPPI transfer DMAis 16-bits wide. For receive modes, if this bit is set, then the EPPI packs the incoming data into 32-bit words. If this bit is cleared, then the EPPI does not do any packing. For transmit modes, if this bit is set, then the EPPI always unpacks the 32-bit data | Pack/Unpack Enable. The EPPI_CTL.PACKEN select whether or not packing is enabled (for receive modes) and unpacking is enabled (for transmit modes). When this bit is set, EPPI transfer DMAis 32-bits wide. When this bit is cleared and the EPPI_CTL.DLEN is less than or equal to 16 bits, EPPI transfer DMAis 16-bits wide. For receive modes, if this bit is set, then the EPPI packs the incoming data into 32-bit words. If this bit is cleared, then the EPPI does not do any packing. For transmit modes, if this bit is set, then the EPPI always unpacks the 32-bit data |
| 22 (R/W)           | PACKEN     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 22 (R/W)           | PACKEN     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 21 (R/W)           | SKIPEO     | Skip Even or Odd. The EPPI_CTL.SKIPEO bit selects whether even (if set) or odd (if cleared) samples are skipped if sample skipping is enabled ( EPPI_CTL.SKIPEN is set). This feature only is useful for receive modes.                                                                                                                                                                                                                                                                                                                                                            | Skip Even or Odd. The EPPI_CTL.SKIPEO bit selects whether even (if set) or odd (if cleared) samples are skipped if sample skipping is enabled ( EPPI_CTL.SKIPEN is set). This feature only is useful for receive modes.                                                                                                                                                                                                                                                                                                                                                            | Skip Even or Odd. The EPPI_CTL.SKIPEO bit selects whether even (if set) or odd (if cleared) samples are skipped if sample skipping is enabled ( EPPI_CTL.SKIPEN is set). This feature only is useful for receive modes.                                                                                                                                                                                                                                                                                                                                                            |
| 21 (R/W)           | SKIPEO     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Skip Odd Samples                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 21 (R/W)           | SKIPEO     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Skip Even Samples                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 20 (R/W)           | SKIPEN     | Skip Enable. The EPPI_CTL.SKIPEN bit enables skipping alternate samples. This feature only is                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Skip Enable. The EPPI_CTL.SKIPEN bit enables skipping alternate samples. This feature only is                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Skip Enable. The EPPI_CTL.SKIPEN bit enables skipping alternate samples. This feature only is                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 20 (R/W)           | SKIPEN     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | No Samples Skipping                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 20 (R/W)           | SKIPEN     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Skip Alternate Samples                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 19 (R/W)           | DMIRR      | Data Mirroring. The EPPI_CTL.DMIRR field enables mirroring (bit reversing) of the data coming in or going out on the EPPI data pins.                                                                                                                                                                                                                                                                                                                                                                                                                                               | Data Mirroring. The EPPI_CTL.DMIRR field enables mirroring (bit reversing) of the data coming in or going out on the EPPI data pins.                                                                                                                                                                                                                                                                                                                                                                                                                                               | Data Mirroring. The EPPI_CTL.DMIRR field enables mirroring (bit reversing) of the data coming in or going out on the EPPI data pins.                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 19 (R/W)           | DMIRR      | Pin Data                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | PPI Data (DAT_MRR=0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | PPI Data (DAT_MRR=1)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 19 (R/W)           | DMIRR      | 15 14 .... 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | ------------------------------ 15 14 .... 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 0 1 ....                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 19 (R/W)           | DMIRR      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | 14 15                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 19 (R/W)           | DMIRR      |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | No Data Mirroring                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 19 (R/W)           | DMIRR      |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Data Mirroring                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 18:16 (R/W)        | DLEN       | Data Length. The EPPI_CTL.DLEN bits select the data length for the EPPI. Note that the 20 bits                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Data Length. The EPPI_CTL.DLEN bits select the data length for the EPPI. Note that the 20 bits                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Data Length. The EPPI_CTL.DLEN bits select the data length for the EPPI. Note that the 20 bits                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 18-49: EPPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                        | Description/Enumeration                                                                                                                                                                               |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 0                                                                                                                      | 8 bits                                                                                                                                                                                                |
|                    |            | 1                                                                                                                      | 10 bits                                                                                                                                                                                               |
|                    |            | 2                                                                                                                      | 12 bits                                                                                                                                                                                               |
|                    |            | 3                                                                                                                      | 14 bits                                                                                                                                                                                               |
|                    |            | 4                                                                                                                      | 16 bits                                                                                                                                                                                               |
|                    |            | 5                                                                                                                      | 18 bits                                                                                                                                                                                               |
|                    |            | 6                                                                                                                      | 20 bits                                                                                                                                                                                               |
|                    |            | 7                                                                                                                      | 24 bits                                                                                                                                                                                               |
| 15:14 (R/W)        | POLS       | Frame Sync Polarity. The EPPI_CTL.POLS selects whether the frame syncs' polarity is active low versus active high.     | Frame Sync Polarity. The EPPI_CTL.POLS selects whether the frame syncs' polarity is active low versus active high.                                                                                    |
| 15:14 (R/W)        | POLS       | 0                                                                                                                      | FS1 and FS2 are active high                                                                                                                                                                           |
| 15:14 (R/W)        | POLS       | 1                                                                                                                      | FS1 is active low. FS2 is active high                                                                                                                                                                 |
| 15:14 (R/W)        | POLS       | 2                                                                                                                      | FS1 is active high. FS2 is active low                                                                                                                                                                 |
| 15:14 (R/W)        | POLS       | 3                                                                                                                      | FS1 and FS2 are active low                                                                                                                                                                            |
| 13:12 (R/W)        | POLC       | Clock Polarity. The EPPI_CTL.POLC selects the rising versus falling edge for sampling data and sampling/driving syncs. | Clock Polarity. The EPPI_CTL.POLC selects the rising versus falling edge for sampling data and sampling/driving syncs.                                                                                |
|                    |            | 0                                                                                                                      | Clock/Sync Polarity Mode 0. For receive mode: Sample data on falling edge and sample/drive syncs on falling edge. For transmit mode: Drive data on rising edge and sample/drive syncs on rising edge. |
|                    |            | 1                                                                                                                      | Clock/Sync Polarity Mode 1. For receive mode: Sample data on falling edge and sample/drive syncs on rising edge. For transmit mode: Drive data on rising edge and sample/drive syncs on falling edge. |
|                    |            | 2                                                                                                                      | Clock/Sync Polarity Mode 2. For receive mode: Sample data on rising edge and sample/drive syncs on falling edge. For transmit mode: Drive data on falling edge and sample/drive syncs on rising edge. |
|                    |            | 3                                                                                                                      | Clock/Sync Polarity Mode 3. For receive mode: Sample data on rising edge and sample/drive syncs on rising edge. For transmit mode: Drive data on falling edge and sample/drive syncs on falling edge. |

Table 18-49: EPPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | SIGNEXT    | Sign Extension. The EPPI_CTL.SIGNEXT select whether (for receive modes when EPPI_CTL.DLEN selecting 16 bit data length) the data is sign extended or zero fil- led. Not that EPPI_CTL.SPLTWRD is removed from this shared bit.                                                                                                                                                        |
| 11 (R/W)           | SIGNEXT    | 0 Zero Filled                                                                                                                                                                                                                                                                                                                                                                         |
| 10 (R/W)           | IFSGEN     | Internal Frame Sync Generation. The EPPI_CTL.IFSGEN bit selects whether the frame syncs are generated internally or are supplied by an external device.                                                                                                                                                                                                                               |
| 10 (R/W)           | IFSGEN     | 0 External Frame Sync                                                                                                                                                                                                                                                                                                                                                                 |
| 10 (R/W)           | IFSGEN     | 1 Internal Frame Sync                                                                                                                                                                                                                                                                                                                                                                 |
| 9 (R/W)            | ICLKGEN    | Internal Clock Generation. The EPPI_CTL.ICLKGEN bit selects whether the EPPI_CLK is generated inter- nally or is supplied by an external device.                                                                                                                                                                                                                                      |
| 9 (R/W)            | ICLKGEN    | 0 External Clock                                                                                                                                                                                                                                                                                                                                                                      |
| 9 (R/W)            | ICLKGEN    | 1 Internal Clock                                                                                                                                                                                                                                                                                                                                                                      |
| 8 (R/W)            | BLANKGEN   | king Generation (ITU Output Mode). The EPPI_CTL.BLANKGEN enables ITU output with internal blanking. In GP 8, 10 transmit bit modes (when EPPI_CTL.SPLTWRD is cleared) and 16-, 20-, and 24-bit transmit modes (when EPPI_CTL.SPLTWRD is set), EPPI_CTL.BLANKGEN selects whether or not the EPPI generates blanking and generates preamble and insertion with active data from memory. |
| 8 (R/W)            | BLANKGEN   | 0 Disable                                                                                                                                                                                                                                                                                                                                                                             |
| 8 (R/W)            | BLANKGEN   | 1 Enable                                                                                                                                                                                                                                                                                                                                                                              |
| 7 (R/W)            | ITUTYPE    | ITU Interlace or Progressive. The EPPI_CTL.ITUTYPE selects interlaced or progressive operation for ITU656 mode. This selection is valid for both TX and RX modes.                                                                                                                                                                                                                     |
| 7 (R/W)            | ITUTYPE    | 0 Interlaced                                                                                                                                                                                                                                                                                                                                                                          |
| 7 (R/W)            | ITUTYPE    | 1 Progressive                                                                                                                                                                                                                                                                                                                                                                         |

Table 18-49: EPPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | FLDSEL     | Field Select/Trigger. The EPPI_CTL.FLDSEL bits configure the EPPI field and trigger selection. These are valid for GP modes ( EPPI_CTL.XFRTYPE =0x3) and ITU656 active video mode ( EPPI_CTL.XFRTYPE cleared).                                                                                               | Field Select/Trigger. The EPPI_CTL.FLDSEL bits configure the EPPI field and trigger selection. These are valid for GP modes ( EPPI_CTL.XFRTYPE =0x3) and ITU656 active video mode ( EPPI_CTL.XFRTYPE cleared).                                                                                               |
| 6 (R/W)            | FLDSEL     |                                                                                                                                                                                                                                                                                                              | 0 Field Mode 0. Read field 1 (for ITU656 active video mode). Set internal trigger (for GP RX mode). FS3 is toggled on FS2 assertion followed by FS1 assertion (when the EPPI_CTL.FSCFG bit selects sync mode 3 and the EPPI_CTL.IFSGEN bit selects internal frame sync).                                     |
| 6 (R/W)            | FLDSEL     |                                                                                                                                                                                                                                                                                                              | 1 Field Mode 1 Read field 1 and field 2 (ITU656 active video mode). Set external trigger (GP RX mode). FS3 is toggled on FS2 assertion (when the EPPI_CTL.FSCFG bit selects sync mode 3 and the EPPI_CTL.IFSGEN bit selects internal frame sync).                                                            |
| 5:4 (R/W)          | FSCFG      | Frame Sync Configuration. The EPPI_CTL.FSCFG bits configure the EPPI frame syncs. These are valid only for GP modes ( EPPI_CTL.XFRTYPE =0x3). The output of the frames syncs also depends on whether the EPPI transfer direction is transmit and the EPPI is in ITU output mode ( EPPI_CTL.BLANKGEN is set). | Frame Sync Configuration. The EPPI_CTL.FSCFG bits configure the EPPI frame syncs. These are valid only for GP modes ( EPPI_CTL.XFRTYPE =0x3). The output of the frames syncs also depends on whether the EPPI transfer direction is transmit and the EPPI is in ITU output mode ( EPPI_CTL.BLANKGEN is set). |
| 5:4 (R/W)          | FSCFG      |                                                                                                                                                                                                                                                                                                              | 0 Sync Mode 0. FS0 driven in GP mode. FS0 not driven in ITU output mode.                                                                                                                                                                                                                                     |
| 5:4 (R/W)          | FSCFG      |                                                                                                                                                                                                                                                                                                              | 1 Sync Mode 1. FS1 driven in GP mode. HSYNC driven on FS1 in ITU output mode.                                                                                                                                                                                                                                |
| 5:4 (R/W)          | FSCFG      |                                                                                                                                                                                                                                                                                                              | 2 Sync Mode 2. FS2 driven in GP mode. HSYNC driven on FS1 and VSYNC driven on FS1 in ITU output mode.                                                                                                                                                                                                        |
| 5:4 (R/W)          | FSCFG      |                                                                                                                                                                                                                                                                                                              | 3 Sync Mode 3. FS3 driven in GP mode. HSYNC driven on FS1, VSYNC driven on FS2, and FIELD driven on FS3 in ITU output mode.                                                                                                                                                                                  |

Table 18-49: EPPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:2 (R/W)          | XFRTYPE    | Transfer Type ( Operating Mode). The EPPI_CTL.XFRTYPE bits select the EPPI operating mode. In receive mode ( EPPI_CTL.DIR cleared), the EPPI modes include ITU656 active video only mode, ITU656 entire field mode, ITU656 vertical blanking only mode, and non-ITU656 mode (GP mode). For transmit mode ( EPPI_CTL.DIR set), the EPPI_CTL.XFRTYPE bits have no effect, and the EPPI (in transmit mode) is al- ways in GP mode. | Transfer Type ( Operating Mode). The EPPI_CTL.XFRTYPE bits select the EPPI operating mode. In receive mode ( EPPI_CTL.DIR cleared), the EPPI modes include ITU656 active video only mode, ITU656 entire field mode, ITU656 vertical blanking only mode, and non-ITU656 mode (GP mode). For transmit mode ( EPPI_CTL.DIR set), the EPPI_CTL.XFRTYPE bits have no effect, and the EPPI (in transmit mode) is al- ways in GP mode. |
| 3:2 (R/W)          | XFRTYPE    | 0                                                                                                                                                                                                                                                                                                                                                                                                                               | ITU656 Active Video Only Mode                                                                                                                                                                                                                                                                                                                                                                                                   |
| 3:2 (R/W)          | XFRTYPE    | 1                                                                                                                                                                                                                                                                                                                                                                                                                               | ITU656 Entire Field Mode                                                                                                                                                                                                                                                                                                                                                                                                        |
| 3:2 (R/W)          | XFRTYPE    | 2                                                                                                                                                                                                                                                                                                                                                                                                                               | ITU656 Vertical Blanking Only Mode                                                                                                                                                                                                                                                                                                                                                                                              |
| 3:2 (R/W)          | XFRTYPE    | 3                                                                                                                                                                                                                                                                                                                                                                                                                               | Non-ITU656 Mode (GP Mode)                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1 (R/W)            | DIR        | PPI Direction. The EPPI_CTL.DIR bit selects whether the EPPI is in receive mode (if cleared) or in transmit mode (if set).                                                                                                                                                                                                                                                                                                      | PPI Direction. The EPPI_CTL.DIR bit selects whether the EPPI is in receive mode (if cleared) or in transmit mode (if set).                                                                                                                                                                                                                                                                                                      |
| 1 (R/W)            | DIR        | 0                                                                                                                                                                                                                                                                                                                                                                                                                               | Receive Mode                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 1 (R/W)            | DIR        | 1                                                                                                                                                                                                                                                                                                                                                                                                                               | Transmit Mode                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | EN         | PPI Enable. The EPPI_CTL.EN bit enables or disables the EPPI.                                                                                                                                                                                                                                                                                                                                                                   | PPI Enable. The EPPI_CTL.EN bit enables or disables the EPPI.                                                                                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | EN         | 0                                                                                                                                                                                                                                                                                                                                                                                                                               | Disable                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 0 (R/W)            | EN         | 1                                                                                                                                                                                                                                                                                                                                                                                                                               | Enable                                                                                                                                                                                                                                                                                                                                                                                                                          |

## Control Register 2 Register

The EPPI\_CTL2 register controls HSYNC finish signal generation.

Figure 18-16: EPPI\_CTL2 Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000014_c4001225dd7d06e7e0fcba5e58ee68839118e49fc70cf47c52252b8ce35fd01f.png)

Table 18-50: EPPI\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | FS1FINEN   | HSYNC Finish Enable. The EPPI_CTL2.FS1FINEN bit selects whether (if set) the EPPI sends a finish command (010) through the DDE COMMANDline soon after a LINE is received completely or (if cleared) the EPPI sends a finish command (010) through the DDE COMMANDline soon after a FRAME is received completely. Note that the EPPI_CTL.DMAFINEN bit must be set for the EPPI to generate either | HSYNC Finish Enable. The EPPI_CTL2.FS1FINEN bit selects whether (if set) the EPPI sends a finish command (010) through the DDE COMMANDline soon after a LINE is received completely or (if cleared) the EPPI sends a finish command (010) through the DDE COMMANDline soon after a FRAME is received completely. Note that the EPPI_CTL.DMAFINEN bit must be set for the EPPI to generate either |
| 1 (R/W)            | FS1FINEN   | 0                                                                                                                                                                                                                                                                                                                                                                                                | Finish sent after frame RX done. PPI sends a finish command (010) through the DDE COMMANDline soon after a FRAME is received completely                                                                                                                                                                                                                                                          |
| 1 (R/W)            | FS1FINEN   | 1                                                                                                                                                                                                                                                                                                                                                                                                | Finish sent after frame/line RX done. PPI sends a finish command (010) through the DDE COMMANDline soon after a frame/line is received completely.                                                                                                                                                                                                                                               |

## Clipping Register for EVEN (Luma) Data Register

The EPPI\_EVENCLIP register selects the clipping threshold for luma data, which provides clipping of individual video components.

The high even and low even spaces in EPPI\_EVENCLIP are 16-bits wide and (depending on the EPPI\_CTL.DLEN bit selection) only the corresponding video component bits are considered for clipping.

For example, if the EPPI is programmed in 10-bit mode, bits [9:0] and bits [25:16] constitute the clipping thresholds. The higher bits are (in this case) ignored.

Using the this method, 8-, 10-, 12- and 16-bit clipping thresholds can be set.

Note that when the EPPI is programmed in 16-, 20-, or 24-bit mode with the EPPI\_CTL.SPLTWRD bit set, the luma data gets the clipping threshold levels of the EPPI\_EVENCLIP register, and the chroma data gets the clipping threshold levels of the EPPI\_ODDCLIP register.

Also, note that the EPPI\_EVENCLIP and EPPI\_ODDCLIP registers are ignored when the EPPI\_CTL.RGBFMTEN bit is set.

Figure 18-17: EPPI\_EVENCLIP Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000015_dbd996c9f9c22954bdcb224d966d3de956b61f1077fa4d403e8a997ed03d7714.png)

Table 18-51: EPPI\_EVENCLIP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | HIGHEVEN   | High Even Clipping Threshold (Luma Data). The EPPI_EVENCLIP.HIGHEVEN bit field selects the clipping threshold for luma data. The high even spaces are 16-bits wide and (depending on the EPPI_CTL.DLEN selection) only the corresponding video component bits are con- sidered for clipping. |
| 15:0 (R/W)         | LOWEVEN    | Low Even Clipping Threshold (Luma Data). The EPPI_EVENCLIP.LOWEVEN bit field selects the clipping threshold for luma data. The low even spaces are 16-bits wide and (depending on the EPPI_CTL.DLEN selection) only the corresponding video component bits are considered for clipping.      |

## Lines Per Frame Register

The EPPI\_FRAME register tracks the frame track overflow and underflow errors. This register should be programmed with the number of lines expected per frame. Any write to the EPPI\_FRAME register will also write the same value to the EPPI\_VCNT register. But, any write to the EPPI\_VCNT register does not affect the EPPI\_FRAME register value. So the EPPI\_FRAME register should be programmed before the EPPI\_VCNT register.

Figure 18-18: EPPI\_FRAME Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000016_fa08f3aa1d77fd71fd6e0e177fbefc1a6bd6113e056a2425e3e777bb360305d6.png)

Table 18-52: EPPI\_FRAME Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 15:0               | VALUE      | Lines Per Frame.                                                           |
| (R/W)              |            | The EPPI_FRAME.VALUE holds the number of lines expected per frame of data. |

## Frame Sync 1 Delay Value Register

The EPPI\_FS1\_DLY register selects the delay count (based on the period of the EPPI\_CLK clock) between the first rising edge of EPPI\_CLK after the EPPI is enabled and the first active edge of the associated frame sync when the internal frame sync is used.

Note that if the EPPI\_FS1\_DLY or EPPI\_FS2\_DLY registers are programmed with value 0, the EPPI operates as though 0 value is 1, and the first frame sync transition occurs after the completion of one period value of the respective counters.

Figure 18-19: EPPI\_FS1\_DLY Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000017_711e97dbea4b43a1190ac1687c04a31fc912f7516aa7d40c8183358cfea56fdb.png)

Table 18-53: EPPI\_FS1\_DLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                     |
|--------------------|------------|-------------------------------------------------------------|
| 31:0               | FS1_DLY    | Frame Sync 1 Delay Count.                                   |
| (R/W)              |            | The EPPI_FS1_DLY.FS1_DLY bit field selects the delay count. |

## FS1 Period Register / EPPI Active Samples Per Line Register

The EPPI\_FS1\_PASPL register content varies depending on whether the EPPI is in GP1/2/3 FS modes or in GP transmit mode.

In GP 1, 2, or 3 FS modes, the EPPI\_FS1\_PASPL register is used for the generation of frame sync 1. The register contains the period required for EPPI\_FS1 based on the EPPI\_CLK clock.

In GP transmit mode with the EPPI\_CTL.BLANKGEN bit set, this register contains the number of samples of active video or vertical blanking samples per line. When used for blanking generation, only the lower 16 bits are valid.

Note that a value of 0 for this register is illegal. If programmed as 0, the EPPI regards the EPPI\_FS1\_PASPL register as containing 1.

Frame Sync Period or Samples Number

Figure 18-20: EPPI\_FS1\_PASPL Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000018_742c894360a453244ae48231150114ab0173e94239554b24e2d2d41005829b18.png)

Table 18-54: EPPI\_FS1\_PASPL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Frame Sync Period or Samples Number. In GP 1, 2, or 3 FS modes, the EPPI_FS1_PASPL.VALUE bit field is used for the generation of frame sync 1 and contains the period required for EPPI_FS1 based on the EPPI_CLK clock. In GP transmit mode with the EPPI_CTL.BLANKGEN bit set, this bit field contains the number of samples of active video or vertical blanking samples per line. |

## FS1 Width Register / EPPI Horizontal Blanking Samples Per Line Register

The EPPI\_FS1\_WLHB register's content varies depending on whether the EPPI is in GP1/2/3 FS modes or in GP transmit mode.

In GP 1, 2 or 3 FS modes, EPPI\_FS1\_WLHB is used for the generation of frame sync 1. The register contains the width required for EPPI\_FS1 based on the EPPI\_CLK clock.

In GP transmit mode with the EPPI\_CTL.BLANKGEN bit set, this register contains the number of samples of horizontal blanking per line. When used for blanking generation, only the lower 16 bits are valid.

Note that a value of 0 for the EPPI\_FS1\_WLHB register is illegal. If programmed as 0, the EPPI regards the EPPI\_FS1\_WLHB register as containing 1.

Figure 18-21: EPPI\_FS1\_WLHB Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000019_6ce6501757ee642c6aa7d21572588b8049a80f4836db16e9a76910be5cc36bfa.png)

Table 18-55: EPPI\_FS1\_WLHB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Frame Sync Width or Blanking Samples Number. The EPPI_FS1_WLHB.VALUE bit field content varies depending on whether the EPPI is in GP1/2/3 FS modes or in GP transmit mode. In GP 1, 2 or 3 FS modes, the EPPI_FS1_WLHB.VALUE bit field is used for the generation of frame sync 1. The register contains the width required for EPPI_FS1 based on the EPPI_CLK clock. In GP transmit mode with EPPI_CTL.BLANKGEN set, this bit field contains the number of samples of horizontal blanking per line. |

## Frame Sync 2 Delay Value Register

The EPPI\_FS2\_DLY register selects the delay count (based on the period of the EPPI\_CLK clock) between the first rising edge of EPPI\_CLK after EPPI enabled and the first active edge of the associated frame sync when the internal frame sync is used.

Note that if the EPPI\_FS1\_DLY or EPPI\_FS2\_DLY registers are programmed with the value 0, the EPPI operates as though 0 value is 1, and the first frame sync transition occurs after the completion of one period value of the respective counters.

Figure 18-22: EPPI\_FS2\_DLY Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000020_31ec5221a96d40340c06a05ee45dbe27a6dd419eef472d7d09f46dd1a6f30380.png)

Table 18-56: EPPI\_FS2\_DLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                     |
|--------------------|------------|-------------------------------------------------------------|
| 31:0               | FS2_DLY    | Frame Sync 2 Delay Count.                                   |
| (R/W)              |            | The EPPI_FS2_DLY.FS2_DLY bit field selects the delay count. |

## FS2 Period Register / EPPI Active Lines Per Field Register

The EPPI\_FS2\_PALPF register content varies depending on whether the EPPI is in GP2/3 FS modes or in GP transmit mode.

In GP 2 or 3 FS modes, EPPI\_FS2\_PALPF is used for the generation of frame sync 2. This register contains the period required for EPPI\_FS2 based on the EPPI\_CLK clock.

In GP transmit mode with the EPPI\_CTL.BLANKGEN bit set, this register contains the number of lines of active video per field.

Note that a value of 0 for the EPPI\_FS2\_PALPF.F1ACT or EPPI\_FS2\_PALPF.F2ACT bits is illegal. If either is programmed as 0, the EPPI regards the 0 value fields as containing 1.

Also, note that for progressive video, the EPPI\_FS2\_PALPF.F2ACT bit is ignored.

Figure 18-23: EPPI\_FS2\_PALPF Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000021_cbae92b6f79eef3136833af7e0f8d8b678803077bd56770326e7bbc6e7173671.png)

Table 18-57: EPPI\_FS2\_PALPF Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | F2ACT      | Field 2 Active. The EPPI_FS2_PALPF.F2ACT bit field contains the number of lines of active da- ta in field 2. |
| 15:0 (R/W)         | F1ACT      | Field 1 Active. The EPPI_FS2_PALPF.F1ACT bit field contains the number of lines of active da- ta in field 1. |

## FS2 Width Register / EPPI Lines Of Vertical Blanking Register

The EPPI\_FS2\_WLVB register content varies depending on whether the EPPI is in GP2/3 FS modes or in GP transmit mode.

In GP 2 or 3 FS modes, the EPPI\_FS2\_WLVB register is used for the generation of frame sync 2. The register contains the width required for EPPI\_FS2 based on the EPPI\_CLK clock.

In GP transmit mode with the EPPI\_CTL.BLANKGEN bit set, this register contains the number or lines of vertical blanking.

Note that for progressive video, the EPPI\_FS2\_WLVB.F2VBBD and EPPI\_FS2\_WLVB.F2VBAD bits are ignored.

Figure 18-24: EPPI\_FS2\_WLVB Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000022_8310ee3b075c2e54b022398b7f03f6d035861f93ef514194a6f013a5c13bade2.png)

Table 18-58: EPPI\_FS2\_WLVB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | F2VBAD     | Field 2 Vertical Blanking After Data. The EPPI_FS2_WLVB.F2VBAD bit field contains the number of lines of vertical blanking after field 2.   |
| 23:16 (R/W)        | F2VBBD     | Field 2 Vertical Blanking Before Data. The EPPI_FS2_WLVB.F2VBBD bit field contains the number of lines of vertical blanking before field 2. |
| 15:8 (R/W)         | F1VBAD     | Field 1 Vertical Blanking After Data. The EPPI_FS2_WLVB.F1VBAD bit field contains the number of lines of vertical blanking after field 1.   |
| 7:0 (R/W)          | F1VBBD     | Field 1 Vertical Blanking Before Data. The EPPI_FS2_WLVB.F1VBBD bit field contains the number of lines of vertical blanking before field 1. |

## Horizontal Transfer Count Register

The EPPI\_HCNT register holds the number of samples to read in or write out per line, after EPPI\_HDLY number of cycles have expired since the assertion of EPPI\_FS1 . Any write to the EPPI\_LINE register modifies the EPPI\_HCNT register. But, any write to the EPPI\_HCNT register does not affect the EPPI\_LINE register value. So the EPPI\_HCNT register should be programmed after the EPPI\_LINE register.

Figure 18-25: EPPI\_HCNT Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000023_c20aec4c80ea0ee1cf3638d05d67c982428bc763f159b9816c967503d753ac56.png)

Table 18-59: EPPI\_HCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Horizontal Transfer Count. The EPPI_HCNT.VALUE holds the number of samples to read in or write out per line, after EPPI_HDLY number of cycles have expired since the last assertion of EPPI_FS1 . |

## Horizontal Delay Count Register

The EPPI\_HDLY register contains the number of clock cycles to delay after the assertion of EPPI\_FS1 is detected before starting to read or write data.

Figure 18-26: EPPI\_HDLY Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000024_300d8e613cbc802cba711562b99177658841939a72d5bc20565475dceb15490d.png)

Table 18-60: EPPI\_HDLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0               | VALUE      | Horizontal Delay Count. The EPPI_HDLY.VALUE holds the number of EPPI_CLK cycles to delay after as- sertion of EPPI_FS1 before starting to read or write data. |
| (R/W)              |            |                                                                                                                                                               |

## Interrupt Mask Register

The EPPI\_IMSK register permits the masking (if associated bit is set) of EPPI error interrupts for YFIFO underflow or overflow, CFIFO underflow or overflow, line track overflow error, line track underflow error, frame track overflow error, frame track underflow error, and ERR\_NCOR (ITU preamble error not corrected. These conditions are flagged in the EPPI\_STAT register and cleared by write-1-to-clear.

Figure 18-27: EPPI\_IMSK Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000025_79d66f10d7226e05f83b3995db48fccf4986bc15919f294f8fd6ae30afef456d.png)

Table 18-61: EPPI\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                             |
|--------------------|------------|---------------------------------------------------------------------|
| 7                  | PXPERR     | PxP Ready Error Interrupt Mask.                                     |
| 6 (R/W)            | ERRNCOR    | ITU Preamble Error Not Corrected Interrupt Mask. 0 Unmask Interrupt |
| 5 (R/W)            | FTERRUNDR  | Frame Track Underflow Error Interrupt Mask. 0 Unmask Interrupt      |
| 4                  | FTERROVR   | 1 Mask Interrupt Frame Track                                        |
| (R/W)              |            | Overflow Error Interrupt Mask. 0 Unmask Interrupt 1 Mask Interrupt  |

Table 18-61: EPPI\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------|
| 3                  | LTERRUNDR  | Line Track Underflow Error Interrupt Mask.                                            |
| 2 (R/W)            | LTERROVR   | Line Track Overflow Error Interrupt Mask. 0 Unmask Interrupt                          |
| 1 (R/W)            | YFIFOERR   | YFIFO Underflow or Overflow Error Interrupt Mask. 0 Unmask Interrupt                  |
| 0                  | CFIFOERR   | 1 Mask Interrupt                                                                      |
| (R/W)              |            | CFIFO Underflow or Overflow Error Interrupt Mask. 0 Unmask Interrupt 1 Mask Interrupt |

## Samples Per Line Register

The EPPI\_LINE register tracks the line track overflow and underflow errors. This register should be programmed with the number of samples expected per line. Any write to the EPPI\_LINE register will also write the same value to the EPPI\_HCNT register. However, any write to the EPPI\_HCNT register does not affect the EPPI\_LINE register value. So the EPPI\_LINE register should be programmed before the EPPI\_HCNT register.

Figure 18-28: EPPI\_LINE Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000026_de2c6a42ca994653062302d179f3a8e34f66dcab16c35527bb79849d4ce3f167.png)

Table 18-62: EPPI\_LINE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                            |
|--------------------|------------|--------------------------------------------------------------------|
| 15:0               | VALUE      | Samples Per Line.                                                  |
| (R/W)              |            | The EPPI_LINE.VALUE holds the number of samples expected per line. |

## Clipping Register for ODD (Chroma) Data Register

The EPPI\_ODDCLIP register selects the clipping threshold for chroma data, which provides clipping of individual video components.

The high odd and low odd spaces in EPPI\_ODDCLIP are 16-bits wide and (depending on the EPPI\_CTL.DLEN bit selection) only the corresponding video component bits are considered for clipping.

For example, if the EPPI is programmed in 10-bit mode, bits [9:0] and bits [25:16] constitute the clipping thresholds. The higher bits are (in this case) ignored.

Using the this method, 8-, 10-, 12- and 16-bit clipping thresholds can be set.

Note that when the EPPI is programmed in 16-, 20-, or 24-bit mode with the EPPI\_CTL.SPLTWRD bit set, the luma data gets the clipping threshold levels of the EPPI\_EVENCLIP register, and the chroma data gets the clipping threshold levels of the EPPI\_ODDCLIP register.

Also, note that the EPPI\_EVENCLIP and EPPI\_ODDCLIP registers are ignored when the EPPI\_CTL.RGBFMTEN bit is set.

Figure 18-29: EPPI\_ODDCLIP Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000027_91b48cb687aa5993b6fcfcec31d25440c451e8939d7c09863b634830d4d64f42.png)

Table 18-63: EPPI\_ODDCLIP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | HIGHODD    | High Odd Clipping Threshold (Chroma Data). The EPPI_ODDCLIP.HIGHODD bit field selects the clipping threshold for luma da- ta. The high odd spaces are 16-bits wide and (depending on the EPPI_CTL.DLEN selection) only the corresponding video component bits are considered for clipping. |
| 15:0 (R/W)         | LOWODD     | Low Odd Clipping Threshold (Chroma Data). The EPPI_ODDCLIP.LOWODD bit field selects the clipping threshold for luma data. The low add spaces are 16-bits wide and (depending on the EPPI_CTL.DLEN selec- tion) only the corresponding video component bits are considered for clipping.    |

## Status Register

The EPPI\_STAT register contains bits that provide information about the current operating state of the EPPI.

Figure 18-30: EPPI\_STAT Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000028_c2a1a6ab319e02f554555b72ff42d55560fba0c5b1eee117b45ff100cb247b8c.png)

Table 18-64: EPPI\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | FLD        | Current Field Received by EPPI. The EPPI_STAT.FLD bit indicates whether the current field being received by the PPI is field 1 (if clear) or field 2 (if set). 0 Field 1                                                                                |
| 14 (R/W1C)         | ERRDET     | Preamble Error Detected. The EPPI_STAT.ERRDET bit is useful only in ITU receive modes and indicates if an error has been detected in the status word of EAV or SAV sequences (if set) or not (if clear). 0 No Preamble Error Detected                   |
| 7 (R/W1C)          | PXPERR     | PxP Ready Error. The EPPI_STAT.PXPERR bit is valid only in the RX mode. This bit indicates whether the incoming PPI data overflows the PxP interface (if set) or not (if clear). This bit is sticky and must be cleared by software by writing 1 to it. |

Table 18-64: EPPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1C)          | ERRNCOR    | Preamble Error Not Corrected. The EPPI_STAT.ERRNCOR bit is useful only in the ITU receive modes and indi- cates if an error in the status word of EAV or SAV sequences can not be cleared (if set) or not (if clear). This bit is sticky and must be cleared by software by writing 1 to it. |
| 6 (R/W1C)          | ERRNCOR    | 0 No uncorrected preamble error has occurred                                                                                                                                                                                                                                                 |
| 6 (R/W1C)          | ERRNCOR    | 1 Preamble error detected but not corrected                                                                                                                                                                                                                                                  |
| 5 (R/W1C)          | FTERRUNDR  | Frame Track Underflow. The EPPI_STAT.FTERRUNDR bit indicates whether a frame track underflow error has occurred (if set) or not (if clear). This bit is sticky and must be cleared by software by writing 1 to it.                                                                           |
| 5 (R/W1C)          | FTERRUNDR  | 0 No Error Detected                                                                                                                                                                                                                                                                          |
| 5 (R/W1C)          | FTERRUNDR  | 1 Error Occurred                                                                                                                                                                                                                                                                             |
| 4 (R/W1C)          | FTERROVR   | Frame Track Overflow. The EPPI_STAT.FTERROVR bit indicates whether a frame track overflow error has occurred (if set) or not (if clear). This bit is sticky and must be cleared by software by writing 1 to it.                                                                              |
| 4 (R/W1C)          | FTERROVR   | 0 No Error Detected                                                                                                                                                                                                                                                                          |
| 4 (R/W1C)          | FTERROVR   | 1 Error Occurred                                                                                                                                                                                                                                                                             |
| 3 (R/W1C)          | LTERRUNDR  | Line Track Underflow. The EPPI_STAT.LTERRUNDR bit indicates whether a line track underflow error has occurred (if set) or not (if clear). This bit is sticky and must be cleared by software by writing 1 to it.                                                                             |
| 3 (R/W1C)          | LTERRUNDR  | 0 No Error Detected                                                                                                                                                                                                                                                                          |
| 3 (R/W1C)          | LTERRUNDR  | 1 Error Occurred                                                                                                                                                                                                                                                                             |
| 2 (R/W1C)          | LTERROVR   | Line Track Overflow. The EPPI_STAT.LTERROVR bit indicates whether a line track overflow error has occurred (if set) or not (if clear). This bit is sticky and must be cleared by software by writing 1 to it.                                                                                |
| 2 (R/W1C)          | LTERROVR   | 0 No Error Detected                                                                                                                                                                                                                                                                          |
| 2 (R/W1C)          | LTERROVR   | 1 Error Occurred                                                                                                                                                                                                                                                                             |

Table 18-64: EPPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | YFIFOERR   | Luma FIFO Error. For RX modes, the EPPI_STAT.YFIFOERR bit indicates whether the Luma FIFO has overflowed (if set) or not (if clear). For TX modes, this bit indicates whether the Luma FIFO has underflowed (if set) or not (if clear). This bit is sticky and must be cleared by software by writing 1 to it. | Luma FIFO Error. For RX modes, the EPPI_STAT.YFIFOERR bit indicates whether the Luma FIFO has overflowed (if set) or not (if clear). For TX modes, this bit indicates whether the Luma FIFO has underflowed (if set) or not (if clear). This bit is sticky and must be cleared by software by writing 1 to it. |
| 1 (R/W1C)          | YFIFOERR   | 0                                                                                                                                                                                                                                                                                                              | No Error Detected                                                                                                                                                                                                                                                                                              |
| 0 (R/W1C)          | CFIFOERR   | Chroma FIFO Error. For RX modes, the EPPI_STAT.CFIFOERR bit indicates whether the chroma FIFO has overflowed (if set) or not (if clear). For TX modes, this bit indicates whether the chroma FIFO has underflowed (if set) or not (if clear). This bit is sticky and must                                      | Chroma FIFO Error. For RX modes, the EPPI_STAT.CFIFOERR bit indicates whether the chroma FIFO has overflowed (if set) or not (if clear). For TX modes, this bit indicates whether the chroma FIFO has underflowed (if set) or not (if clear). This bit is sticky and must                                      |
| 0 (R/W1C)          | CFIFOERR   | 0                                                                                                                                                                                                                                                                                                              | No Error Detected                                                                                                                                                                                                                                                                                              |
| 0 (R/W1C)          | CFIFOERR   | 1                                                                                                                                                                                                                                                                                                              | Error Occurred                                                                                                                                                                                                                                                                                                 |

## Vertical Transfer Count Register

The EPPI\_VCNT register holds the number of lines to read in or write out, after EPPI\_VDLY number of lines from the start of frame. Any write to the EPPI\_FRAME register modifies the EPPI\_VCNT register. However, any write to the EPPI\_VCNT register does not affect the EPPI\_FRAME register value. So the EPPI\_VCNT register should be programmed after the EPPI\_FRAME register.

Figure 18-31: EPPI\_VCNT Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000029_e0c1877c2f488219462f9c313953da7e164a51644aa3fbb1929215a38114cf11.png)

Table 18-65: EPPI\_VCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Vertical Transfer Count. The EPPI_VCNT.VALUE holds the number of lines to read in or write out, after EPPI_VDLY number of lines from the start of frame. |

## Vertical Delay Count Register

The EPPI\_VDLY register contains the number of lines to wait after the start of a new frame before starting to read/ transmit data.

Figure 18-32: EPPI\_VDLY Register Diagram

![Image](21_Enhanced_Parallel_Peripheral_Interface_(EPPI)_artifacts/image_000030_00a42fe90166956a532571c4a6a49f2ba08f76c48ee57c028cfc8e550ade9dc7.png)

Table 18-66: EPPI\_VDLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Vertical Delay Count. The EPPI_VDLY.VALUE holds the number of lines to wait after the start of a new frame before starting to read/transmit data. |