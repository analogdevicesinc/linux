# Media Local Bus (MLB)

<!-- source: 029_Media_Local_Bus_MLB.pdf | original pages 1538–1601 -->

## 27   Media Local Bus (MLB)

Media Local Bus (MediaLB ® ) is an on-PCB or inter-chip communication bus, which allows an application to access the MOST network data. Media Local Bus supports all the MOST network data transport methods including synchronous stream data, asynchronous packet data, control message data and isochronous data. The MLB topology supports communication among the MLB controller and MLB devices, where the MLB controller is the interface between the MLB devices and the MOST network.

The MLB module serves as an interface between the MediaLB and the processor, implementing the requirements of the physical layer and the link layer outlined in the MediaLB specification. It supports up to 64 logical channels with up to 468 bytes of data per MediaLB frame. Transmit and receive data can be transferred between MediaLB and on-chip memory with DMA block transfers.

The MLB supports the MOST25, MOST50 and MOST150 standards and this document assumes familiarity with these standards. For more information, refer to the Media Local Bus specification version 4.2.

- Copyright 1998-2016 Microchip Technology Inc. All rights reserved. Portions of this chapter are included with permission from Microchip Technology, Inc.

## Features

The objective of MLB is to map all the MOST Network data types (transport methods) into a single low-cost, scalable, and standardized hardware interface between a MediaLB controller and at least one other MediaLB Device. The adoption of MediaLB simplifies the hardware interface, reduces the pin count, and facilitates the design of modular reusable hardware. From a software development perspective, the use of MediaLB relieves the system developer from the complexity of the MOST network, which simplifies software development and enables the design of reusable software for different applications. This simplified, standardized interface shortens time to market and makes software maintenance effortless.

- Compliant to Media Local Bus specification version 4.2
- Support MOST25, MOST50, MOST150 standards
- Support 3-pin and 6-pin mode
- 6-pin mode supports a data rate of 2048 × FS
- 3-pin mode support various data rates of 256 × FS, 512 × FS, 1024 × FS

- Support 64 logic channels
- Dedicated PLL for clock recovery and phase alignment
- Dedicated pins for 6-pin mode (LVDS)
- Shared pins for 3-pin mode
- Dedicated internal RAM for data buffering and channel table
- Recovered clock (after division) available out on a shared pin for DAI

NOTE: Only one MLB interface (3-pin mode or 6-pin mode) is active at any given time.

## MLB Definitions

The following are standard MLB and MOST terms that are used in this chapter.

## AGU

Address Generation Units. To access a particular HBI channel the HC must first configure one of two HBI address generation units. AGUs can be configured by writing the HBI command registers, HCMD0 and HCMD1.

## CAT

Channel Allocation Table. The Channel Allocation Table (CAT) is comprised of 16 CTR entries. Each 16-bit CAT entry represents a logical connection to or from a transmit/receive device (for example MediaLB or HBI channel).

## CDT

Channel Descriptor Table.

## FCE

Flow Control Enable bit. The FCE bit is used by MediaLB isochronous Rx channels only.

## HBI

Host Bus Interface. The HBI block provides 16-bit parallel completer access to all MOST channels and data types for the external Host Controller (HC). The HBI supports up to 64 independent channels.

## HC

Host Controller (external).

## HCMDx

HBI Command registers.

## HSTSx

HBI status registers.

## MFE

Multi-Frame per sub-buffer enable bit. The MFE bit is used by MediaLB synchronous channels only.

## PML

Packet Message Length.

## Clocking

The MLB controller provides an external clock pin-the media local bus clock. The MLB controller generates the clock. It is synchronized to the MOST network and provides the timing for the entire MLB interface at FS = 48 kHz.

## Functional Description

The MediaLB Block Diagram figure shows the MLB high-level architecture. The MLB core serves as an interface between the MediaLB and the processor, implementing the requirements of the physical layer and the link layer outlined in the MediaLB specification. The MLB core has the following responsibilities.

- Transmit commands and data when functioning as the transmitting device associated with a Channel Address
- Receive data and transmit Rx status responses when functioning as the receiving device associated Channel Address
- MLB lock detection
- System channel command handling

The MediaLB device can function as either a MediaLB 3-pin interface (single-ended) or MediaLB 6-pin interface (differential) but only one interface can be active at a time. The MediaLB interfaces are capable of exchanging data at speeds up to 1024 × Fs in 3-pin mode or 2048 × Fs in 6-pin mode.

A set of physical channels for exchanging data over the MediaLB bus is supported. These physical channels (4 bytes in length, or a quadlet) can be grouped into logical channels, where each logical channel is referenced using a channel address and represents a uni-directional datapath between a specific MediaLB device transmitting the data and the MediaLB device(s) receiving the data. The MediaLB 6-pin interface provides support for up to 468 bytes of data per frame. The logical channels, configured by system software, can be any combination of channel types (synchronous, asynchronous, isochronous, or control) and direction (transmit or receive).

## ADSP-2159x\_SC592\_SC594 MLB Register List

The MediaLB Device Interface Macro 2 (MediaLB DIM 2), also referred to as OS62420, implements the required functionality of a Media Local Bus (MediaLB) device. This logic serves as an interface between the inter-chip MediaLB bus and a customer IC, implementing the physical- and link-layer requirements outlined in the MediaLB Specification.

Table 27-1: ADSP-2159x\_SC592\_SC594 MLB Register List

| Name      | Description                                           |
|-----------|-------------------------------------------------------|
| MLB_ACMR0 | Peripheral Channel Mask 0 Register                    |
| MLB_ACMR1 | Peripheral Channel Mask 1 Register                    |
| MLB_ACSR0 | Peripheral Channel Status 0 Register                  |
| MLB_ACSR1 | Peripheral Channel Status 1 Register                  |
| MLB_ACTL  | Bus Control Register                                  |
| MLB_CTL0  | MediaLB Control 0 Register                            |
| MLB_CTL1  | Control 1 Register                                    |
| MLB_GCTL  | MLB Global Control Register                           |
| MLB_HCBR0 | HBI Channel Busy 0 Register                           |
| MLB_HCBR1 | HBI Channel Busy 1 Register                           |
| MLB_HCER0 | HBI Channel Error 0 Register                          |
| MLB_HCER1 | HBI Channel Error 1 Register                          |
| MLB_HCMR0 | HBI Channel Mask 0 Register                           |
| MLB_HCMR1 | HBI Channel Mask 1 Register                           |
| MLB_HCTL  | HBI Control Register                                  |
| MLB_MADR  | Memory Interface Address Register                     |
| MLB_MCTL  | Memory Interface Control Register                     |
| MLB_MDAT0 | Memory Interface Control Data 0 Register              |
| MLB_MDAT1 | Memory Interface Control Data 1 Register              |
| MLB_MDAT2 | Memory Interface Control Data 2 Register              |
| MLB_MDAT3 | Memory Interface Control Data 3 Register              |
| MLB_MDWE0 | Memory Interface Control Data Write Enable 0 Register |
| MLB_MDWE1 | Memory Interface Control Data Write Enable 1 Register |
| MLB_MDWE2 | Memory Interface Control Data Write Enable 2 Register |
| MLB_MDWE3 | Memory Interface Control Data Write Enable 3 Register |
| MLB_MIEN  | Interrupt Enable Register                             |

Table 27-1: ADSP-2159x\_SC592\_SC594 MLB Register List (Continued)

| Name      | Description                      |
|-----------|----------------------------------|
| MLB_MS0   | Channel Status 0 Register        |
| MLB_MS1   | Channel Status 1 Register        |
| MLB_MSD   | System Data Register             |
| MLB_MSS   | System Status Register           |
| MLB_PCTL0 | MediaLB 6-pin Control 0 Register |

## ADSP-2159x\_SC592\_SC594 MLB Interrupt List

Table 27-2: ADSP-2159x\_SC592\_SC594 MLB Interrupt List

|   Interrupt ID | Name      | Description      | Sensitivity   | DMA Channel   |
|----------------|-----------|------------------|---------------|---------------|
|            187 | MLB0_INT0 | MLB0 Interrupt 0 |               |               |
|            188 | MLB0_INT1 | MLB0 Interrupt 1 |               |               |
|            189 | MLB0_STAT | MLB0 Status      |               |               |

## MediaLB Protocol

The MediaLB topology supports communication among all MediaLB devices, including the MediaLB controller. The bus interface consists of a uni-directional line for clock (MLBC), a bidirectional line for signal information (MLBS), and a bidirectional line for data transfer (MLBD). The MediaLB topology supports one controller connected to one or more devices, where the controller is the interface between the MediaLB devices and the MOST network.

The MediaLB controller includes MediaLB device functionality, and also generates the MediaLB clock (MLBC) that is synchronized to the MOST Network. This generated clock provides the timing for the entire MediaLB interface. The MLBS line is a multiplexed signal which carries channel addresses generated by the MediaLB controller, as well as command and RxStatus bytes from MediaLB devices. The MLBD line is driven by the transmitting MediaLB device and is received by all other MediaLB devices, including the MediaLB controller. The MLBD line carries the actual data (synchronous, asynchronous, control, or isochronous).

Once per MOST network frame, the MLB controller generates a unique frame sync pattern on the MLB\_SIG line. The end of the frame sync pattern defines the byte boundary and the channel boundary for the MLB\_SIG and MLB\_DAT lines of all MLB devices.

The MLB controller manages the arbitration for all the channels on the MLB and grants bandwidth for all the MLB devices. An MLB physical channel is defined as four bytes wide, or a quadlet. Physical channels can be grouped into multiple quadlets (which do not have to be consecutive) to form an MLB logical channel , which is defined by a unique channel address.

As shown in MLB Data Structure , the MLB controller initiates communication by sending out a channel address on the MLB\_SIG line for each physical channel. The channel address indicates which MLB device is transmitting and which MLB devices are receiving in the following physical channel. Therefore, four bytes after the controller outputs the channel address on the MLB\_SIG line, the transmitting device outputs a command byte command on the MLB\_SIG line and outputs the respective data on the MLB\_DAT line, concurrently. The MLB command byte contains the type of data currently being transmitted (for example synchronous, asynchronous or control).

The MLB device receiving the channel data outputs a status byte, RxStatus, on the MLB\_SIG line immediately after the transmitting device outputs the command byte. The status response can indicate that the receiving device is busy and cannot receive the data at present, or the device is ready to receive the data. Since synchronous stream data is sent in a broadcast fashion, receiving devices cannot return a busy status and should not drive RxStatus onto the MLB\_SIG line.

Figure 27-1: MLB Data Structure

<!-- image -->

## MLB Architectural Concepts

The following sections provide information about the MLB architecture.

## MediaLB Block Diagram

The MediaLB Block Diagram shows the various blocks within the interface its connections to the processor.

Figure 27-2: MediaLB Block Diagram

<!-- image -->

## MediaLB Interface

The Media Local Bus (MediaLB) block supports both a MediaLB 3-pin interface and MediaLB 6-pin interface; however, only one MediaLB interface can be active at any given time. Both MediaLB interfaces provide real-time access to all network data types - synchronous, asynchronous, control, and isochronous data.

- MediaLB 3-pin interface-supports the MediaLB protocol for single-ended 3-pin mode, with a maximum data rate of 1024 FS (49.152 MHz at FS = 48 kHz).
- MediaLB 6-pin interface-supports the MediaLB protocol for high-speed differential 6-pin mode, with a maximum data rate of 2048 FS (98.304 MHz at FS = 48 kHz).

## MediaLB Channel Address to Logical Channel Mapping

The MediaLB channel addresses are mapped to the logical channels as shown in the MediaLB Channel Address to Logical Channel Mapping table.

Table 27-3: MediaLB Channel Address to Logical Channel Mapping

| Channel Address   | Logical Address                                             |
|-------------------|-------------------------------------------------------------|
| 0x0002            | 1                                                           |
| 0x0004            | 2                                                           |
| 0x0006            | 3                                                           |
| ...               | ...                                                         |
| 0x007C            | 62                                                          |
| 0x007E            | 63                                                          |
| 0x01FE            | 0 (Logical channel 0 is the system channel and is reserved) |

## Routing Fabric

The Routing Fabric (RF) block manages the flow of data between the MediaLB port and the HBI port. It manages accessing the Channel Table RAM (CTR) and Data Buffer RAM (DBR), which are explained in the following subsections. The routing fabric uses channel descriptors (stored in the CTR) to manage access to dynamic buffers in the DBR.

## Data Buffer RAM

The Data Buffer RAM (DBR) is an 8-bit x 16k entriy buffer, single-port synchronous SRAM, and provides dynamic circular buffering between the transmit and receive devices. The size and location of each data buffer is defined by software in the Channel Descriptor Table (CDT), which is in the Channel Table RAM (CTR), described in following sections.

## Channel Table RAM

The Channel Table RAM (CTR) is a 128-bit x 144-entry table that allows system software to dynamically configure channel routing and allocate data buffers in the DBR. The CTR is logically divided into three tables:

- Channel Descriptor Tables
- AHB Descriptor Table (ADT)
- Channel Allocation Table

## Address Mapping

The CTR Address Mapping table shows the address mapping for the CTR.

Table 27-4: CTR Address Mapping

| Label                          | Address                        | Bits [127:96]                  | Bits [63:32]                   | Bits [31:0]                    |                                |
|--------------------------------|--------------------------------|--------------------------------|--------------------------------|--------------------------------|--------------------------------|
| Channel Descriptor Table (CDT) | Channel Descriptor Table (CDT) | Channel Descriptor Table (CDT) | Channel Descriptor Table (CDT) | Channel Descriptor Table (CDT) | Channel Descriptor Table (CDT) |
| CDT                            | 0x00                           |                                | CDT0[127:0], CL = 0            |                                |                                |
| CDT                            | 0x01                           |                                | CDT1[127:0], CL = 1            |                                |                                |
| CDT                            | 0x02                           |                                | CDT2[127:0], CL = 2            |                                |                                |
| CDT                            | ...                            |                                |                                |                                |                                |
| CDT                            | 0x3D                           |                                | CDT61[127:0], CL = 61          |                                |                                |
| CDT                            | 0x3E                           |                                | CDT62[127:0], CL = 62          |                                |                                |
| CDT                            | 0x3F                           |                                | CDT63[127:0], CL = 63          |                                |                                |
| AHB Descriptor Table (ADT)     | AHB Descriptor Table (ADT)     | AHB Descriptor Table (ADT)     | AHB Descriptor Table (ADT)     | AHB Descriptor Table (ADT)     | AHB Descriptor Table (ADT)     |
| ADT                            | 0x40                           |                                | ADT0[127:0], CL = 0            |                                |                                |
| ADT                            | 0x41                           |                                | ADT1[127:0], CL = 1            |                                |                                |
| ADT                            | 0x42                           |                                | ADT2[127:0], CL = 2            |                                |                                |
| ADT                            | ...                            |                                |                                |                                |                                |

Table 27-4: CTR Address Mapping (Continued)

| Label                          | Address                        | Bits [127:96]                  | Bits [127:96]                  | Bits [95:64]                   | Bits [95:64]                   | Bits [63:32]                   | Bits [63:32]                   | Bits [31:0]                    | Bits [31:0]                    |
|--------------------------------|--------------------------------|--------------------------------|--------------------------------|--------------------------------|--------------------------------|--------------------------------|--------------------------------|--------------------------------|--------------------------------|
|                                | 0x7D                           | ADT61[127:0], CL = 61          | ADT61[127:0], CL = 61          | ADT61[127:0], CL = 61          | ADT61[127:0], CL = 61          | ADT61[127:0], CL = 61          | ADT61[127:0], CL = 61          | ADT61[127:0], CL = 61          | ADT61[127:0], CL = 61          |
|                                | 0x7E                           | ADT62[127:0], CL = 62          | ADT62[127:0], CL = 62          | ADT62[127:0], CL = 62          | ADT62[127:0], CL = 62          | ADT62[127:0], CL = 62          | ADT62[127:0], CL = 62          | ADT62[127:0], CL = 62          | ADT62[127:0], CL = 62          |
|                                | 0x7F                           | ADT63[127:0], CL = 63          | ADT63[127:0], CL = 63          | ADT63[127:0], CL = 63          | ADT63[127:0], CL = 63          | ADT63[127:0], CL = 63          | ADT63[127:0], CL = 63          | ADT63[127:0], CL = 63          | ADT63[127:0], CL = 63          |
| Channel Allocation Table (CAT) | Channel Allocation Table (CAT) | Channel Allocation Table (CAT) | Channel Allocation Table (CAT) | Channel Allocation Table (CAT) | Channel Allocation Table (CAT) | Channel Allocation Table (CAT) | Channel Allocation Table (CAT) | Channel Allocation Table (CAT) | Channel Allocation Table (CAT) |
| CAT for Medi- aLB              | 0x80                           | CAT7                           | CAT6                           | CAT5                           | CAT4                           | CAT3                           | CAT2                           | CAT1                           | CAT0                           |
|                                | ...                            | ...                            | ...                            | ...                            | ...                            | ...                            | ...                            | ...                            | ...                            |
|                                | 0x87                           | CAT63                          | CAT62                          | CAT61                          | CAT60                          | CAT59                          | CAT58                          | CAT57                          | CAT56                          |
| CAT for HBI *1                 | 0x88                           | CAT71                          | CAT70                          | CAT69                          | CAT68                          | CAT67                          | CAT66                          | CAT65                          | CAT64                          |
|                                | ...                            | ...                            | ...                            | ...                            | ...                            | ...                            | ...                            | ...                            | ...                            |
|                                | 0x8F                           | CAT127                         | CAT126                         | CAT125                         | CAT124                         | CAT123                         | CAT122                         | CAT121                         | CAT120                         |

## Channel Allocation Table

The Channel Allocation Table (CAT) table is comprised of 16 CTR entries (addresses 0x80 0x8F) as shown in the CTR Entry Map table. Each 16-bit CAT entry represents a logical connection to or from a transmit or receive device. (for example, MediaLB channel). All entries are indexed according to a fixed physical address assigned to every RX/TX channel as shown in the CAT Entry Formats table. The value stored in a CAT entry includes a 6-bit connection label, which provides a pointer to the CDT. To complete a logical channel and form a routing connection, system software must assign the same connection label to both the RX and TX channels.

Table 27-5: CAT Entry Map

| Peripheral   | TX Channels   | RX Channels    |   CAT Start Index |   CAT End Index |   Entries |
|--------------|---------------|----------------|-------------------|-----------------|-----------|
| MediaLB      | 0 to 64       | 64 TX Channels |                 0 |              63 |        64 |
| HBI          | 0 to 64       | 64 TX Channels |                64 |             127 |        64 |

The format of a full CAT entry is shown in the CAT Entry Formats table, with field descriptions described in the CAT Field Definitions table. All reserved bits of a CAT entry field should be written as zero.

Table 27-6: CAT Entry Formats

| Channel Type   | 15   | 14   | 13   | 12   | 11   | 10            | 9             | 8             | 7    | 6    | 5       | 4       | 3       | 2       | 1       | 0       |
|----------------|------|------|------|------|------|---------------|---------------|---------------|------|------|---------|---------|---------|---------|---------|---------|
| Isochronous    | rsvd | FCE  | rsvd | RNW  | CE   | CT[2:0] = 011 | CT[2:0] = 011 | CT[2:0] = 011 | rsvd | rsvd | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] |
| Asynchronous   | rsvd | rsvd | MT   | RNW  | CE   | CT[2:0] = 010 | CT[2:0] = 010 | CT[2:0] = 010 | rsvd | rsvd | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] |
| Control        | rsvd | rsvd | MT   | RNW  | CE   | CT[2:0] = 001 | CT[2:0] = 001 | CT[2:0] = 001 | rsvd | rsvd | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] |

Table 27-6: CAT Entry Formats (Continued)

| Channel Type   | 15   | 14   | 13   | 12   | 11   | 10            | 9             | 8             | 7    | 6   | 5   | 4       | 3       | 2       | 1       | 0       |
|----------------|------|------|------|------|------|---------------|---------------|---------------|------|-----|-----|---------|---------|---------|---------|---------|
| Synchronous    | rsvd | MFE  | MT   | RNW  | CE   | CT[2:0] = 000 | CT[2:0] = 000 | CT[2:0] = 000 | rsvd |     |     | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] | CL[5:0] |

Table 27-7: CAT Field Definitions

| Field   | Description                                                                                                                                                                                                                                                                                                                                 |
|---------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CL[5:0] | Connection Label (offset into CDT)                                                                                                                                                                                                                                                                                                          |
| CT[2:0] | Channel Type (others) 111 = Reserved 110 = Reserved 101 = Reserved 100 = Reserved 011 = Isochronous 010 = Asynchronous 001 = Control 000 = Synchronous                                                                                                                                                                                      |
| CE      | Channel Enable. 0 = Disabled, 1 = Enabled                                                                                                                                                                                                                                                                                                   |
| RNW     | Read Not Write. 0 = Write, 1 = Read                                                                                                                                                                                                                                                                                                         |
| MT      | Mute Enable. 0 = Disabled When set for synchronous channels, the MTbit forces RX channels to write zeros into the channel data buf- fer, and TX channels to output zeros on the physical interface. When set for asynchronous and control chan- nels, the MTbit causes DMAto halt at a packet boundary. Not valid for isochronous channels. |
| FCE     | Flow Control Enable. 0 = Disabled, 1 = Enabled The FCE bit is used by MediaLB isochronous RX channels only.                                                                                                                                                                                                                                 |
| MFE     | Multi-Frame per sub-buffer enable. 0 = Disabled, 1 = Enabled The MFE bit is used by MediaLB synchronous channels only.                                                                                                                                                                                                                      |
| rsvd    | Reserved. Software writes a 0 to all reserved bits when the entry is initialized. These bits are read-only after initialization.                                                                                                                                                                                                            |

## Channel Set Up

Data direction is in reference to the DBR. The data direction of CAT entries corresponding to the same channel is reversed for the HBI CAT and the MediaLB CAT.

- For a Tx channel (from the HC to the MediaLB interface):
- HBI CAT entry: RNW = 0 (write)
- MediaLB CAT entry: RNW = 1 (read)
- Conversely, for an Rx channel (data from MediaLB to HC):

- HBI CAT entry: RNW = 1 (read)
- MediaLB CAT entry: RNW = 0 (write)

Figure 27-3: DBR Directional Relationship

<!-- image -->

## Channel Descriptor Tables

The Channel Descriptor Table (CDT) is comprised of 64 CTR entries (addresses 0x00 - 0x3F), as shown in the Table 27-4 CTR Address Mapping table. Each 128-bit CDT entry (also referred to as a channel descriptor) is referenced by a connection label and contains information about a data buffer in the DBR (for example buffer size, address pointers). The format of each CDT entry is dependent on the channel type (synchronous, isochronous, asynchronous, or control).

NOTE: All reserved channel descriptor bits must be written to 0 by software when initialized.

## Synchronous Channel Descriptors

The format and field definitions for a synchronous CDT entry are shown in the Synchronous CDT Entry Format and Synchronous CDT Entry Field Definitions tables.

Table 27-8: Synchronous CDT Entry Format

|   Bit Offset | 15                   | 14                   | 13                   | 12                   | 11                   | 10                   | 9                    | 8                    | 7                    | 6                    | 5                    | 4                    | 3                    | 2                    | 1                    | 0                    |
|--------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|
|            0 | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   | WSBC[1:0] Reserved   |
|           16 | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   | RSBC[1:0] Reserved   |
|           32 | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             |
|           48 | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             | Reserved             |
|           64 | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] | WSTS[3:0] WPTR[11:0] |
|           80 | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] | RSTS[3:0] RPTR[11:0] |
|           96 | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    | Reserved BD[11:0]    |
|          112 | Reserved             | Reserved             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             | BA[13:0]             |

Table 27-9: Synchronous CDT Entry Field Definitions

| Field   | Description         | Details                              | Access   |
|---------|---------------------|--------------------------------------|----------|
|         | Buffer Base Address | Can start at any byte in the 16k DBR | RW       |

Table 27-9: Synchronous CDT Entry Field Definitions (Continued)

| Field                                                                                                                     | Description                                                                                                               | Details                                                                                                                                                              | Access   |
|---------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------|
| BA                                                                                                                        |                                                                                                                           |                                                                                                                                                                      |          |
| BD                                                                                                                        | Buffer Depth                                                                                                              | BD = size of buffer in bytes 1. Buffer end address = BA + BD. BD = 4 mbpf 1 where: m=frames per sub-buffer (for MFE = 0, m=1) bpf = bytes per frame.                 | RW       |
| RPTR                                                                                                                      | Read Pointer                                                                                                              | Software initializes to 0, hardware updates. Counts the read address offset within a buffer. DMAread address = BA + RPTR.                                            | RWU      |
| WPTR                                                                                                                      | Write Pointer                                                                                                             | Software initializes to 0, hardware updates. Counts the write address offset within a buffer. DMAread address = BA + WPTR.                                           | RWU      |
| RSBC                                                                                                                      | Read Sub-buffer Counter                                                                                                   | Software initializes to 0, hardware updates. Counts the read sub-buffer offset. DMAuses for pointer management.                                                      | RWU      |
| WSBC                                                                                                                      | Write Sub-buffer Counter                                                                                                  | Software initializes to 0, hardware updates. Counts the write sub-buffer offset. DMAuses for pointer management.                                                     | RWU      |
| RSTS                                                                                                                      | Read Status                                                                                                               | Software initializes to 0, hardware updates. RSTS States: xxx0 = normal operation (no mute) xxx1 = normal operation (mute) xx0x = idle                               | RWU      |
| WSTS                                                                                                                      | Write Status                                                                                                              | Software initializes to 0, hardware updates. WSTS States: xxx0 = normal operation (no mute) xxx1 = normal operation (mute) xx0x = idle 1xxx = command protocol error | RWU      |
| Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization. | Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization. | Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization.                                            | RWU      |

## Isochronous Channel Descriptors

The format and field definitions for a synchronous CDT entry are shown in the Isochronous Entry Formats and Isochronous CDT Entry Field Definitions tables.

Table 27-10: Isochronous Entry Formats

|   Bit Offset | 15               | 14               | 13               | 12               | 11               | 10               | 9                | 8                | 7                | 6                | 5                | 4                | 3                | 2                | 1                | 0                |
|--------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|
|            0 | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         |
|           16 | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         |
|           32 | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] | Reserved BS[8:0] |
|           48 | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         | Reserved         |
|           64 | WSTS[3:0]        | WSTS[3:0]        | WSTS[3:0]        | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       | WPTR[12:0]       |
|           80 | RSTS[3:0]        | RSTS[3:0]        | RSTS[3:0]        | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       | RPTR[12:0]       |
|           96 | Reserved         | Reserved         | Reserved         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         | BD[12:0]         |
|          112 | BF               | Rsvd             | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         | BA[13:0]         |

Table 27-11: Isochronous CDT Entry Field Definitions

| Field   | Description         | Details                                                                                                                                                                                                                                                                                             | Access   |
|---------|---------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------|
| BA      | Buffer Base Address | Can start at any byte in the 16k DBR.                                                                                                                                                                                                                                                               | RW       |
| BD      | Buffer Depth        | BD = size of buffer in bytes 1. Buffer end address = BA + BD. Isochronous buffers must be large enough to hold at least 3 blocks (packets) of data. BD Must be an integer multiple of blocks.                                                                                                       | RW       |
| BF      | Buffer Full         | Software initializes to 0, hardware updates. DMAwrite hardware sets BF when the buffer is full. DMAread hardware clears BF when the buffer is empty. BF is valid only when buffer is full or empty, otherwise ig- nore.                                                                             | RWU      |
| BS      | Block Size          | BS defines when to begin the DMAto the data buffer. BS = buffer block size in bytes 1. For RX channels, the DMAwrites start when the number of empty bytes in the data buffer the block size. For TX channels, the DMAreads start when the number of valid bytes in the data buffer the block size. | RWU      |
| RPTR    | Read Pointer        | Software initializes to 0, hardware updates. Counts the read address offset within a buffer. DMAread address = BA + RPTR.                                                                                                                                                                           | RWU      |
| WPTR    | Write Pointer       | Software initializes to 0, hardware updates. Counts the write address offset within a buffer. DMAwrite address = BA + WPTR.                                                                                                                                                                         | RWU      |

Table 27-11: Isochronous CDT Entry Field Definitions (Continued)

| Field                                                                                                                     | Description                                                                                                               | Details                                                                                                                                                | Access   |
|---------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|----------|
| RSTS                                                                                                                      | Read Status                                                                                                               | Software initializes to 0, hardware updates. RSTS States: xx1 = active xx0 = idle                                                                      | RWU      |
| WSTS                                                                                                                      | Write Status                                                                                                              | Software initializes to 0, hardware updates. WSTS States: xxx0 =active xxx1 = idle xx0x = command protocol error 1xxx = buffer overflow (FCE = 0 only) | RWU      |
| Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization. | Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization. | Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization.                              | RWU      |

## Asynchronous/Control Channel Descriptors

The format and field definitions for an Asynchronous/Control CDT entry are shown in the Asynchronous/Control CDT Entry Format and Asynchronous/Control CDT Entry Field Definitions tables.

Table 27-12: Asynchronous/Control CDT Entry Format

|   Bit Offset | 15        | 14        | 13        | 12        | 11         | 10         | 9          | 8          | 7          | 6          | 5          | 4          | 3          | 2          | 1          | 0          |
|--------------|-----------|-----------|-----------|-----------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|
|            0 | WPC[4:0]  | WPC[4:0]  | WPC[4:0]  | WPC[4:0]  | WPC[4:0]   | WPC[4:0]   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           16 | RPC[4:0]  | RPC[4:0]  | RPC[4:0]  | RPC[4:0]  | RPC[4:0]   | RPC[4:0]   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           32 | Rsvd      | WPC[7:5]  | WPC[7:5]  | WPC[7:5]  | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           48 | Rsvd      | RPC[7:5]  | RPC[7:5]  | RPC[7:5]  | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           64 | WSTS[3:0] | WSTS[3:0] | WSTS[3:0] | WSTS[3:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] | WPTR[11:0] |
|           80 | RSTS[3:0] | RSTS[3:0] | RSTS[3:0] | RSTS[3:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] | RPTR[11:0] |
|           96 | RSTS[4]   | WSTS[4 ]  | Rsvd      |           | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   | BD[11:0]   |
|          112 | Rsvd      | Rsvd      | BA[13:0]  | BA[13:0]  | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   | BA[13:0]   |

Table 27-13: Asynchronous/Control CDT Entry Field Definitions

| Field   | Description         | Details                                                                                 | Access   |
|---------|---------------------|-----------------------------------------------------------------------------------------|----------|
| BA      | Buffer Base Address | Can start at any byte in the 16k DBR.                                                   | RW       |
| BD      | Buffer Depth        | BD = size of buffer in bytes 1. Buffer end address = BA + BD. BD = max packet length 1. | RW       |

Table 27-13: Asynchronous/Control CDT Entry Field Definitions (Continued)

| Field                                                                                                                     | Description                                                                                                               | Details                                                                                                                                                                                                                                                            | Access   |
|---------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------|
| RPC                                                                                                                       | Read Packet Count                                                                                                         | Software initializes to 0, hardware updates. Used with RPC, RPTR and WPTR to determine if the buf- fer is empty or full.                                                                                                                                           | RWU      |
| WPC                                                                                                                       | Write Packet Count                                                                                                        | Software initializes to 0, hardware updates. Used with RPC, RPTR and WPTR to determine if the buf- fer is empty or full.                                                                                                                                           | RWU      |
| RPTR                                                                                                                      | Read Pointer                                                                                                              | Software initializes to 0, hardware updates. Counts the read address offset within a buffer. DMAread address = BA + RPTR.                                                                                                                                          | RWU      |
| WPTR                                                                                                                      | Write Pointer                                                                                                             | Software initializes to 0, hardware updates. Counts the write address offset within a buffer. DMAread address = BA + WPTR.                                                                                                                                         | RWU      |
| RSTS                                                                                                                      | Read Status                                                                                                               | Software initializes to 0, hardware updates. RSTS States: x0x00 = idle xx1xx = ReceiverProtocolError response received from RX de- vice 1xxxx = ReceiverBreak command received from RX device                                                                      | RWU      |
| WSTS                                                                                                                      | Write Status                                                                                                              | Software initializes to 0, hardware updates. Status States (only valid for DMApointers associated with the MLB block, not HBI block pointers): xxx0 = idle xxx1 = command protocol error detected xx0x = AsyncBreak / ControlBreak command received from TX device | RWU      |
| Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization. | Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization. | Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization.                                                                                                                                          | RWU      |

## AHB Descriptor Table (ADT)

The AHB block manages data exchange between local channel data buffers within MLB module and the system memory buffer. To support system memory buffering, a ping-pong memory structure is implemented on a per channel basis using 128-bit descriptors for AHB Descriptor Table (ADT) entries. The Table 27-4 CTR Address Mapping table provides a complete address map of the CTR, including the location of the ADT.

Each logical channel is assigned a separate 128-bit descriptor, defining the data buffers in the system memory used by the DMA interface for that channel. The descriptors are stored at fixed addresses in the CTR as described in previous section. The ADT Field Definitions table provides an overview of field definitions for ADT entries.

Table 27-14: ADT Field Definitions

| Field    | No. of Bits   | Description                                                                                                                                                                                       | Access                       |
|----------|---------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------|
| CE       | 1             | Channel Enable. 0 = Disabled                                                                                                                                                                      | RW, U                        |
| LE       | 1             | Endianness Select. 0 = Big Endian, 1 = Little Endian                                                                                                                                              | RW                           |
| PG       | 1             | Page pointer. Software initializes to 0, hardware writes thereafter. 0 = Ping buffer, 1 = Pong buffer                                                                                             | RW, U                        |
| RDY1     | 1             | Buffer Ready bit for ping buffer page. 0 = Not ready, 1 = Ready                                                                                                                                   | RW                           |
| RDY2     | 1             | Buffer Ready bit for pong buffer page. 0 = Not ready, 1 = Ready                                                                                                                                   | RW                           |
| DNE1     | 1             | Buffer Done bit for ping buffer page. 0 = Not done, 1 = Done                                                                                                                                      | R, U, c0                     |
| DNE2     | 1             | Buffer Done bit for pong buffer page. 0 = Not done, 1 = Done                                                                                                                                      | R, U, c0                     |
| ERR1     | 1             | Error Response detected for ping buffer page. 0 = No error, 1 = Error                                                                                                                             | R, U, c0                     |
| ERR2     | 1             | Error Response detected for pong buffer page. 0 = No error, 1 = Error                                                                                                                             | R, U, c0                     |
| PS1      | 1             | Packet Start bit for ping buffer page. 0 = No packet start, 1 = Packet start Reserved for synchronous and isochronous channels.                                                                   | RW, U both TX and RX         |
| PS2      | 1             | Packet Start bit for pong buffer page. 0 = No packet start, 1 = Packet start Reserved for synchronous and isochronous channels.                                                                   | RW, U both TX and RX         |
| MEP1     | 1             | Most Ethernet Packet indicator for ping buffer page. 0 = Not MEP, 1 = MEP. MEP1 only valid for the first page of a segmented buffer. Reserved for con- trol synchronous and isochronous channels. | Rsvd for TX, R, U, c0 for RX |
| MEP2     | 1             | Most Ethernet Packet indicator for pong buffer page. 0 = Not MEP, 1 = MEP. MEP2 only valid for the first page of a segmented buffer. Reserved for con- trol synchronous and isochronous channels. | Rsvd for TX, R, U, c0 for RX |
| BD1      | 11 to 13      | Buffer Depth for ping buffer page. 11 or 12 bits for asynchronous and con- trol channels. 13 bits for synchronous and isochronous channels.                                                       | RW                           |
| BD2      | 11 to 13      | Buffer Depth for pong buffer page. 11 or 12 bits for asynchronous and control channels. 13 bits for synchronous and isochronous channels.                                                         | RW                           |
| BA1      | 32            | Buffer Base Address for ping buffer page                                                                                                                                                          | RW                           |
| BA2      | 32            | Buffer Base Address for pong buffer page.                                                                                                                                                         | RW                           |
| Reserved | varies        | Reserved. Software writes a 0 to all these bits when the entry is initialized. Reserved bits are RO after initialization.                                                                         | RW, U                        |

The Ping-Pong System Memory Structure figure shows that this system memory structure is similar for all channel types and shows the relationship between the BAn, BDn, and PG descriptor fields.

Figure 27-4: Ping-Pong System Memory Structure

<!-- image -->

Each ADT entry (also referred to as a Channel Descriptor) holds a 32-bit BAn field which defines the start of each ping or pong buffer within system memory. The BDn field is used to indicate the size for the respective ping or pong page. The maximum size is 2k entries for asynchronous and control channels and 8k entries for isochronous and synchronous channels.

## Synchronous Channel Descriptors

The synchronous buffering scheme allows each ping or pong buffer to contain a single frame or a multiple number of frames. For this reason, the synchronous buffer depth (BDn) must be defined in terms of an integer number (n), frames per sub-buffer (m) and bytes per frame (bpf) of data (for example BDn = n m bpf 1). The Synchronous ADT Entry Format table shows the format for a synchronous ADT entry. The field definitions are defined in the ADT Field Definitions table. Each synchronous channel buffer can be up to 8k-bytes deep.

Table 27-15: Synchronous ADT Entry Format

|   Bit Offset | 15         | 14         | 13         | 12         | 11         | 10         | 9          | 8          | 7          | 6          | 5          | 4          | 3          | 2          | 1          | 0          |
|--------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|
|            0 | CE         | LE         | PG         | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           16 | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           32 | RDY1       | DNE1       | ERR1       | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  |
|           48 | RDY2       | DNE2       | ERR2       | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  |
|           64 | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  |
|           80 | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] |
|           96 | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  |
|          112 | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] |

## Isochronous Channel Descriptors

The isochronous buffering scheme allows each ping or pong buffer to contain a single block or a multiple number of blocks. For this reason, the isochronous buffer depth (BDn) must be defined in terms of an integer number (n) and block size (BS) (for example BDn = n (BS + 1) 1).

The Isochronous ADT Entry Format table shows the format for an isochronous ADT entry. The field definitions are defined in the ADT Field Definitions table. Each isochronous channel buffer can be up to 8k-bytes deep.

Table 27-16: Isochronous ADT Entry Format

|   Bit Offset | 15         | 14         | 13         | 12         | 11         | 10         | 9          | 8          | 7          | 6          | 5          | 4          | 3          | 2          | 1          | 0          |
|--------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|
|            0 | CE         | LE         | PG         | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           16 | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           32 | RDY1       | DNE1       | ERR1       | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  | BD1[12:0]  |
|           48 | RDY2       | DNE2       | ERR2       | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  | BD2[12:0]  |
|           64 | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  |
|           80 | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] |
|           96 | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  |
|          112 | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] |

## Asynchronous and Control Channel Descriptors

Every asynchronous and control packet adheres to the Port Message Protocol (PMP), which designates the first two bytes of each packet as the packet length (PML). Each packet must be no more than 2048-bytes (PML 2048).

Software must set the buffer ready bit (RDYn) for each buffer as it programs the DMA. As hardware processes each buffer, it sets the done bit (DNEn) and generates an interrupt to inform HC. When hardware finishes processing a buffer, it can begin processing another buffer if RDYn is set. The application is responsible for setting up and configuring the channel buffer descriptor prior to every DMA access on the channel.

Two packet buffering modes are supported by hardware for programming the DMA, single-packet mode ( MLB\_ACTL.MPB =0) and multiple-packet mode ( MLB\_ACTL.MPB =1). The MPB is written prior to enabling the channel DMA.

Single Packet Mode . The single-packet mode asynchronous and control buffering scheme supports a maximum of one packet per buffer (for example, ping or pong). Both non-segmented and segmented data packets are allowed while using single-packet mode. Non-segmented packets are exchanged when only one buffer (for example, ping or pong) is needed for packet transfer. Segmented packets are exchanged when a single packet is too long for one buffer and the packet must span multiple buffers.

The Single-Packet Asynchronous and Control Entry Format table shows the format for single-packet mode asynchronous and control ADT entries. The field definitions are defined in the ADT Field Definitions table.

Table 27-17: Single-Packet Asynchronous and Control Entry Format

|   Bit Offset | 15         | 14         | 13         | 12         | 11         | 10         | 9          | 8          | 7          | 6          | 5          | 4          | 3          | 2          | 1          | 0          |
|--------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|
|            0 | CE         | LE         | PG         | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           16 | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           32 | RDY1       | DNE1       | ERR1       | PS1        | MEP1       | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  | BD1[10:0]  |
|           48 | RDY2       | DNE2       | ERR2       | PS2        | MEP2       | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  | BD2[10:0]  |
|           64 | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  |
|           80 | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] |
|           96 | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  |
|          112 | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] |

Multiple Packet Mode . The multiple-packet mode asynchronous and control buffering scheme supports more than one packet per system memory buffer, as shown in the Asynchronous/Control CDT Entry Format table. Multiplepacket mode reduces the interrupt rate for packet channels at the cost of increasing buffering and latency.

For TX packet channels in multiple-packet mode, software sets the packet start bit (PSn) for every buffer. Setting PSn informs hardware that the first two bytes of the buffer contains the port message length (PML) of the first packet. After the first packet, hardware keeps track of where packets start and end within the current buffer. Software should not write to PSn while the buffer is active (RDYn = 1 and DNEn = 0). For TX packet channels, the buffer is done (DNEn = 1) when the last byte of the last packet in the buffer is read from system memory. Software should set the buffer depth to contain the exact number of complete packets for that buffer. Segmented buffers are not supported for TX packet channels in multiple-packet mode.

NOTE: The PS1 and PS2 bits are only valid for TX channels. Set PS1 and PS2 = 1 at the start of the buffer.

Table 27-18: Multiple-Packet Asynchronous and Control Entry Format

|   Bit Offset | 15         | 14         | 13         | 12         | 11         | 10         | 9          | 8          | 7          | 6          | 5          | 4          | 3          | 2          | 1          | 0          |
|--------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|------------|
|            0 | CE         | LE         | PG         | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           16 | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   | Reserved   |
|           32 | RDY1       | DNE1       | ERR1       | PS1        | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  | BD1[11:0]  |
|           48 | RDY2       | DNE2       | ERR2       | PS2        | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  | BD2[11:0]  |
|           64 | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  | BA1[15:0]  |
|           80 | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] | BA1[31:16] |
|           96 | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  | BA2[15:0]  |
|          112 | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] | BA2[31:16] |

## Interrupt Interface Block

The interrupt interface raises an interrupt when specific changes to HBI channel descriptors occur, including:

- For asynchronous and control read/write channels:
- a packet is available to read in the channel buffer, or
- sufficient empty space is available in the channel buffer to accept a requested packet write
- For isochronous read/write channels:
- the number of valid bytes in the channel buffer exceeds the block size, or
- the number of empty bytes in the channel buffer exceeds the block size

## Operating Modes

The following sections describe the operating modes of the MLB interface. The channel type selection enables the logical channels to operate in synchronous, asynchronous, isochronous, or control channels.

NOTE: The logical channels can be any combination of channel type (for example synchronous, asynchronous, or control) and direction (transmit or receive).

## Isochronous Data Exchange

An isochronous HBI channel is initially opened and synchronized with HCMD0.CMD[2:0]= 010. For isochronous channels, no further synchronization is required from the HBI perspective; however, an optional resynchronization command is available for HC flexibility. Setting HCMD0.CMD[2:0]= 011 reinitializes the address pointer within the data buffer, ensuring that subsequent data exchange with the channel is aligned at an isochronous packet boundary. When the HC must close an isochronous channel before it has read or written an entire data packet, setting HCMD0.CMD[2:0]= 000 reopens the channel without synchronizing the address pointer in the buffer. This action allows reading and writing to continue where the HC previously stopped.

For isochronous data transmission, the Exchanging Isochronous Data on an HBI Channel figure shows the flow control.

Figure 27-5: Exchanging Isochronous Data on an HBI Channel

<!-- image -->

## Asynchronous and Control Data Exchange

An asynchronous or control HBI channel is initially opened using HCMD0.CMD[2:0]= 010. Occasionally, the HC may need to close a packet channel before it has completed writing or reading the current packet of data (for example, the HC needs to use this AGU to service another HBI channel). In this case, the HC can reopen the previous channel with HCMD0.CMD[2:0]= 000. This situation allows the HC to continue writing or reading a packet from

the point it left off. In this situation, the PML is already known, and the packet length is not reread or rewritten by the HC.

During the reading of a packet, if the HC sets HCMD0.CMD[2:0]= 010 for the channel before the last byte of the packet is read, an internal hardware pointer is reset to the beginning of the packet buffer. This situation requires that the HC reread the PML (from HSTS1) and reread the packet data (from HDATn) from the beginning. In the same manner, if the HC resets HCMD0.CMD[2:0]= 010 for the channel before the last byte of a packet is written, an internal hardware pointer is reset to the beginning of the packet buffer. This situation requires that the HC rewrite the PML (to HCMD1) and rewrite the packet data (to HDATn) from the beginning. Any previous packet data in the buffer is overwritten.

Frame synchronization is not supported for asynchronous channels.

## Synchronous Data Exchange

The MLB core provides two modes of operation; standard and multi-frame per sub-buffer which provide flexibility for implementing synchronous channels. Channels configured for standard mode require less buffer space but have higher interrupt rates and more stringent latency requirements. Channels setup for each multi-frame per sub-buffer mode requires more buffer space, but has lower interrupt rates and less stringent latency requirements.

To set up a channel in multi-frame per sub-buffer mode:

1. Program the MLB\_CTL0.FCNT bit field to select the number of frames per sub-buffer.
2. Program the CAT to enable multi-frame sub-buffering (MFE= 1) for each channel.
3. Set the buffer depth in the CDT: BD= 4 × m × bpf 1 where: m = frames per sub-buffer, bpf = bytes per frame.
4. Repeat for additional synchronous channels

A sample synchronous data buffer is shown in the Synchronous Data Buffer Structure figure. Each data buffer contains four sub-buffers and each sub-buffer contains space for 1 to 64 frames of data, determined by the MLB\_CTL0.FCNT bits.

Figure 27-6: Synchronous Data Buffer Structure

<!-- image -->

## Data Transfer

Two modes of operation are supported for transferring channel data between the MLB and internal memory. DMA allows the multi-channel DMA engine to manage data transfers without core intervention. Core driven mode (I/O mode) allows software to manage the transfer of data between MLB and internal memory.

NOTE: All hardware channels must use the same data transfer method. Mixed mode operation where hardware channels operate in both I/O mode and DMA mode is not supported.

## DMA

The processor supports DMA mode which uses only INCR8, INCR4 and SINGLE beat bursts. Program the MLB\_ACTL.DMAMODE = 1 to use DMA mode.

## Programming Model

This section provides general guidelines for programming the MediaLB interface.

## Channel Initialization

The software flow required to initialize a channel must be performed to ensure proper operation.

## Configure the Hardware

1. Initialize the CTR and registers. a. Set all the CTR (CAT, CDT, and ADT) bits to 0. b. Set all bits of all registers to 0. 2. Configure the MediaLB interface. a. Select 3-pin or 6-pin MediaLB operation: MLB\_CTL0.PEN =0 (3-pin), MLB\_CTL0.PEN =1 (6-pin). b. Select MediaLB clock speed via MLB\_CTL0.CLK . c. Set MediaLB enable via MLB\_CTL0.EN . 3. Configure the HBI interface. a. Set MLB\_HCMR0 and MLB\_HCMR1 = 0xFFFFFFFF to activate all channels.
- b. Set the HBI enable bit: MLB\_HCTL.EN =1.

## Program the CAT and the CDT

```
1. Initialize all bits of the CAT to 0. 2. Select a logical channel: N = 1 - 63. 3. Program the CDT for channel N. a. Set the 14-bit base address (BA) b. Set the 12-bit or 13-bit buffer depth (BD): BD = buffer depth in bytes 1 · For synchronous channels: (BD + 1) = 4 frames per sub-buffer (m) bytes-per-frame (bpf) · For isochronous channels: (BD + 1) mod (BS + 1) = 0
```

- For asynchronous channels: (BD + 1) max packet length (1024 for a MOST Data Packet (MDP)
- 1536 for a MOST Ethernet Packet (MEP)
- For control channels: (BD + 1) max packet length (64)
4. Program the CAT for the inbound DMA.
- a. For TX channels (to MediaLB) HBI is the inbound DMA
- b. For RX channels (from MediaLB) MediaLB is the inbound DMA
- c. Set the channel direction: RNW = 0
- d. Set the channel type: CT[2:0] = 010 (asynchronous), 001 (control), 011 (isochronous), or 000 (synchronous)
- e. Set the connection label: CL[5:0] = N
- f. If CT[2:0] = 000 (synchronous), set the mute bit (MT = 1)
- g. Set the channel enable: CE = 1
- h. Set all other bits of the CAT to 0
5. Program the CAT for the outbound DMA.
- a. For TX channels (to MediaLB) HBI is the outbound DMA
- b. For RX channels (from MediaLB) MediaLB is the outbound DMA
- c. Set the channel direction: RNW = 1
- d. Set the channel type: CT[2:0] = 010 (asynchronous), 001 (control), 011 (isochronous), or 000 (synchronous)
- e. Set the channel label: CL[5:0] = N
- f. If CT[2:0] = 000 (synchronous), set the mute bit (MT = 1)
- g. Set the channel enable: CE = 1
- h. Set all other bits of the CAT to 0
6. Repeat steps 2 through 5 to initialize all logical channels.

## Program the ADT

1. Initialize all bits of the ADT to 0
2. Select a logical channel: N = 1 - 63
3. Program the AMBA AHB block ping page for channel N
- a. Set the 32-bit base address (BA1)

- b. Set the 11-bit buffer depth (BD1): BD1 = buffer depth in bytes - 1
- For synchronous channels: (BD1 + 1) = n frames per sub-buffer (m) bytes-per-frame (bpf)
- For isochronous channels: (BD1 + 1) mod (BS + 1) = 0
- For asynchronous channels: 5 (BD1 + 1) 4096 (max packet length)
- For control channels: 5 (BD1 + 1) 4096 (max packet length)
- c. For asynchronous and control Tx channels set the packet start bit (PS1) when the page contains the start of the packet
- d. Clear the page done bit (DNE1)
- e. Clear the error bit (ERR1)
- f. Set the page ready bit (RDY1)
4. Program the AMBA AHB block pong page for channel N
- a. Set the 32-bit base address (BA2)
- b. Set the 11-bit buffer depth (BD2): BD2 = buffer depth in bytes - 1
- For synchronous channels: (BD2 + 1) = n frames per sub-buffer (m) bytes-per-frame (bpf)
- For isochronous channels: (BD2 + 1) mod (BS + 1) = 0
- For asynchronous channels: 5 (BD2 + 1) 4096 (max packet length)
- For control channels: 5 (BD2 + 1) 4096 (max packet length)
- c. For asynchronous and control TX channels set the packet start bit (PS2) when the page contains the start of the packet
- d. Clear the page done bit (DNE2)
- e. Clear the error bit (ERR2)
- f. Set the page ready bit (RDY2)
5. Select Big Endian (LE = 0) or Little Endian (LE = 1)
6. Select the active page: PG = 0 (ping), PG = 1 (pong)
7. Set the channel enable (CE) bit for all active logical channels
8. Repeat steps 2 through 7 for all active logical channels.

## Service

After initialization, each channel will require periodic servicing. Use the procedures in the following sections to service DMA and MLB interrupts and to poll for MLB system commands.

## Servicing the DMA Channel Interrupts

1. Program the MLB\_ACMR0 / MLB\_ACMR1 registers to enable interrupts from all active DMA channels.
2. Select the status clear method. MLB\_ACTL.SCE =0 (hardware clears on read), MLB\_ACTL.SCE = 1 (software writes a 1 to clear).
3. Select 1 or 2 interrupt signals. Configure the MLB\_ACTL.SMX bit =0 (one interrupt for channels 0 through 31 and another interrupt for channels 32 through 63 on). Configure the MLB\_ACTL.SMX bit =1 (single interrupt for all channels).
4. Wait for an interrupt.
5. Read the MLB\_ACSR0 / MLB\_ACSR1 registers to determine which channel or channels are causing the interrupt.
6. If the MLB\_ACTL.SCE bit =1, write the results of step 5 back to the MLB\_ACSR0 and MLB\_ACSR1 registers to clear the interrupt.
7. Select a logical channel (N = 0 - 63) with an interrupt to service.
8. Read the ADT entry for channel N to:
- a. Determine the active page (ping or pong) via the PG bit
- b. Determine which page(s) are done via the DNEn bits
- c. Determine which channels encountered an AHB error via the ERRn bit
- d. Determine which asynchronous and control Rx channel pages contain a packet start via the PSn bit (extract the PML)
9. Repeat steps 6 through 8 for all channels with pending interrupts
10. Repeat steps 4 through 9 while there are active channels.

## Servicing the MediaLB Status Interrupts

1. Select the MediaLB channel status register ( MLB\_MS0 / MLB\_MS1 ) to be cleared by software, writing a 0 to the appropriate bits.
2. Program the MLB\_MIEN register to enable protocol error interrupts for all active MediaLB channels. ( MLB\_MIEN.CTXPE =1, MLB\_MIEN.CRXPE =1, MLB\_MIEN.ATXPE =1, MLB\_MIEN.ARXPE =1, MLB\_MIEN.SYNCPE =1, and MLB\_MIEN.ISOCPE =1).
3. Wait for an interrupt on the MLB\_INT0 / MLB\_INT1 signal.
4. Read the MLB\_MS0 / MLB\_MS1 registers to determine which channel(s) are causing the interrupt.
5. Read the RSTS/WSTS of the appropriate CDT(s) to determine the interrupt type.

6. Clear the RSTS/WSTS errors to ensure that the status of channel operations is reflected in the register:

| Option                                   | Description                 |
|------------------------------------------|-----------------------------|
| For synchronous RX channels              | WSTS[3] = 0                 |
| For synchronous TX channels              | RSTS[3] = 0                 |
| For isochronous RX channels              | WSTS[2:1] = 00              |
| For isochronous TX channels              | RSTS[2:1] = 00              |
| For asynchronous and control RX channels | WSTS[4] = 0 and WSTS[2] = 0 |
| For asynchronous and control TX channels | RSTS[4] = 0 and RSTS[2] = 0 |

## Polling for MediaLB System Commands

The MediaLB System status ( MLB\_MSS ) register is used to detect a system command received from the MediaLB controller. The processor's peripheral automatically sends the appropriate system response to the MediaLB controller. The procedure for the application is:

1. The application periodically polls the MLB\_MSS register.
2. Clear by writing a 0 to the appropriate bit in the MLB\_MSS register after the application finishes the service.
3. If MLB\_MSS.SWSYSCMD = 1, read the MLB\_MSD register to receive the system data sent from MediaLB controller.

## ADSP-2159x\_SC592\_SC594 MLB Register Descriptions

MediaLB Device Interface Macro 2 (MLB) contains the following registers.

Table 27-19: ADSP-2159x\_SC592\_SC594 MLB Register List

| Name      | Description                          |
|-----------|--------------------------------------|
| MLB_ACMR0 | Peripheral Channel Mask 0 Register   |
| MLB_ACMR1 | Peripheral Channel Mask 1 Register   |
| MLB_ACSR0 | Peripheral Channel Status 0 Register |
| MLB_ACSR1 | Peripheral Channel Status 1 Register |
| MLB_ACTL  | Bus Control Register                 |
| MLB_CTL0  | MediaLB Control 0 Register           |
| MLB_CTL1  | Control 1 Register                   |
| MLB_GCTL  | MLB Global Control Register          |
| MLB_HCBR0 | HBI Channel Busy 0 Register          |
| MLB_HCBR1 | HBI Channel Busy 1 Register          |

Table 27-19: ADSP-2159x\_SC592\_SC594 MLB Register List (Continued)

| Name      | Description                                           |
|-----------|-------------------------------------------------------|
| MLB_HCER0 | HBI Channel Error 0 Register                          |
| MLB_HCER1 | HBI Channel Error 1 Register                          |
| MLB_HCMR0 | HBI Channel Mask 0 Register                           |
| MLB_HCMR1 | HBI Channel Mask 1 Register                           |
| MLB_HCTL  | HBI Control Register                                  |
| MLB_MADR  | Memory Interface Address Register                     |
| MLB_MCTL  | Memory Interface Control Register                     |
| MLB_MDAT0 | Memory Interface Control Data 0 Register              |
| MLB_MDAT1 | Memory Interface Control Data 1 Register              |
| MLB_MDAT2 | Memory Interface Control Data 2 Register              |
| MLB_MDAT3 | Memory Interface Control Data 3 Register              |
| MLB_MDWE0 | Memory Interface Control Data Write Enable 0 Register |
| MLB_MDWE1 | Memory Interface Control Data Write Enable 1 Register |
| MLB_MDWE2 | Memory Interface Control Data Write Enable 2 Register |
| MLB_MDWE3 | Memory Interface Control Data Write Enable 3 Register |
| MLB_MIEN  | Interrupt Enable Register                             |
| MLB_MS0   | Channel Status 0 Register                             |
| MLB_MS1   | Channel Status 1 Register                             |
| MLB_MSD   | System Data Register                                  |
| MLB_MSS   | System Status Register                                |
| MLB_PCTL0 | MediaLB 6-pin Control 0 Register                      |

## Peripheral Channel Mask 0 Register

The MLB\_ACMR0 register allows control over which channel(s) generate interrupts on MLB\_INT[1:0]. All of the bits in this register default to 0 (masked) so the HC must initially write to the MLB\_ACMR0 register to enable interrupts. Each bit of this register is read/write accessible.

Figure 27-7: MLB\_ACMR0 Register Diagram

<!-- image -->

Table 27-20: MLB\_ACMR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------|
| 31:0 (R/W)         | CHM        | Bitwise Channel Mask. The MLB_ACMR0.CHM bit field masks or unmasks channels 31-0. |
| 31:0 (R/W)         | CHM        | 0 Mask interrupt for Channel 31:0 (bitwise; all channels shown)                   |
| 31:0 (R/W)         | CHM        | 4294967295 Unmask interrupt for Channel 31:0 (bitwise; all chan- nels shown)      |

## Peripheral Channel Mask 1 Register

The MLB\_ACMR1 register allows control over which channel(s) generate interrupts on MLB\_INT[1:0]. All of the bits in this register default to 0 (masked) so the HC must initially write to the MLB\_ACMR1 register to enable interrupts. Each bit of this register is read/write accessible.

Figure 27-8: MLB\_ACMR1 Register Diagram

<!-- image -->

Table 27-21: MLB\_ACMR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                            |
|--------------------|------------|------------------------------------------------------------------------------------|
| 31:0 (R/W)         | CHM        | Bitwise Channel Mask. The MLB_ACMR1.CHM bit field masks or unmasks channels 63-32. |
| 31:0 (R/W)         | CHM        | 0 Mask interrupt for Channel 63:32 (bitwise; all channels shown)                   |
| 31:0 (R/W)         | CHM        | 4294967295 Unmask interrupt for Channel 63:32 (bitwise; all chan- nels shown)      |

## Peripheral Channel Status 0 Register

The MLB\_ACSR0 register contains interrupt bits for each of the 64 physical channels. When a bit in this register is set, it indicates that the corresponding physical channel has an interrupt pending.

A peripheral interrupt is triggered when either DNEn or ERRn is set within the Bus Channel Descriptor. The HC is notified of the channel interrupt via MLB\_INT[1:0]. When an interrupt occurs in ACCUSER (for channels 31 to 0) MLB\_INT[0] is set. When an interrupt occurs in MLB\_ACSR1 (for channels 63 to 32) MLB\_INT[1] is set.

Interrupts in the MLB\_ACSR0 and MLB\_ACSR1 registers can be optionally multiplexed onto a single interrupt signal, MLB\_INT[0], if the MLB\_ACTL.SMX bit = 1.

If the MLB\_ACTL.SCE bit =0, hardware automatically clears the interrupt bit(s) after the HC reads the peripheral channel status registers. Alternatively, if the MLB\_ACTL.SCE bit =1, software must write a 1 to the appropriate bit(s) of the peripheral channel status registers to clear the interrupt(s).

Figure 27-9: MLB\_ACSR0 Register Diagram

<!-- image -->

Table 27-22: MLB\_ACSR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------|
| 31:0 (R/W1C)       | CHS        | Channel Status. The MLB_ACSR0.CHS bit field indicates channel status for channels 31-0. |
| 31:0 (R/W1C)       |            | 0 No interrupt on Channel 31:0 (bitwise; all channels shown)                            |
| 31:0 (R/W1C)       |            | 4294967295 Interrupt on Channel 31:0 (bitwise; all channels shown)                      |

## Peripheral Channel Status 1 Register

The MLB\_ACSR1 register contains interrupt bits for each of the 64 physical channels. When a bit in this register is set, it indicates that the corresponding physical channel has an interrupt pending.

A peripheral interrupt is triggered when either DNEn or ERRn is set within the Bus Channel Descriptor. The HC is notified of the channel interrupt via MLB\_INT[1:0]. When an interrupt occurs in ACCUSER (for channels 31 to 0) MLB\_INT[0] is set. When an interrupt occurs in MLB\_ACSR1 (for channels 63 to 32) MLB\_INT[1] is set.

Interrupts in the MLB\_ACSR1 and MLB\_ACSR0 registers can be optionally multiplexed onto a single interrupt signal, MLB\_INT[0], if the MLB\_ACTL.SMX bit = 1.

If the MLB\_ACTL.SCE bit =0, hardware automatically clears the interrupt bit(s) after the HC reads the peripheral channel status registers. Alternatively, if the MLB\_ACTL.SCE bit =1, software must write a 1 to the appropriate bit(s) of peripheral channel status registers to clear the interrupt(s).

Figure 27-10: MLB\_ACSR1 Register Diagram

<!-- image -->

Table 27-23: MLB\_ACSR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------|
| 31:0 (R/W1C)       | CHS        | Channel Status. The MLB_ACSR1.CHS bit field indicates channel status for channels 63-32. |
| 31:0 (R/W1C)       | CHS        | 0 No interrupt on Channel 63:32 (bitwise; all channels shown)                            |
| 31:0 (R/W1C)       | CHS        | 4294967295 Interrupt on Channel 63:32 (bitwise; all channels shown)                      |

## Bus Control Register

The MLB\_ACTL register is written by the HC to configure the block for channel interrupts. This register contains bits to select the packet buffering mode and the DMA mode. This register also contains bits that are used to multiplex channel interrupts onto a single interrupt signal and to select the method of clearing channel interrupts.

Figure 27-11: MLB\_ACTL Register Diagram

<!-- image -->

Table 27-24: MLB\_ACTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | MPB        | Packet Buffering Mode. The MLB_ACTL.MPB bit selects whether the buffering mode is single-packet or mul- tiple-packet. 0 single-packet mode                                                                                                                                                                                                                                                                                                                            |
| 2 (R/W)            | DMAMODE    | DMAMode. The MLB_ACTL.DMAMODE bit selects between DMAmode 1 and 0. DMAMode 0 uses incrementing bursts of an unspecified length. This allows the block to perform single beat transfers as well as an incrementing (INCR) burst of unspecified length up to the maximum specified burst length (8 beats). DMAMode 1 uses only INCR8, INCR4, and SINGLE beat bursts. The hburst[2:0] signal encodes the burst transfer used for a DMAoperation. 0 DMAMode 0 1 DMAMode 1 |

Table 27-24: MLB\_ACTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | SMX        | Interrupt Multiplex Enable. The MLB_ACTL.SMX bit selects whether ACSR0 generates an interrupt on MLB_INT[0] and ACSR1 generates an interrupt on MLB_INT[1] or ACSR0 and ACSR1 generate an interrupt on MLB_INT[0] only. | Interrupt Multiplex Enable. The MLB_ACTL.SMX bit selects whether ACSR0 generates an interrupt on MLB_INT[0] and ACSR1 generates an interrupt on MLB_INT[1] or ACSR0 and ACSR1 generate an interrupt on MLB_INT[0] only. |
|                    |            | 0                                                                                                                                                                                                                       | ACSR0 generates an interrupt on MLB_INT[0]; ACSR1 generates an interrupt on MLB_INT[1]                                                                                                                                  |
| 0 (R/W)            | SCE        | Software Clear Enable. The MLB_ACTL.SCE bit selects whether hardware clears the interrupt after a ACSRn                                                                                                                 | Software Clear Enable. The MLB_ACTL.SCE bit selects whether hardware clears the interrupt after a ACSRn                                                                                                                 |
|                    |            | 0                                                                                                                                                                                                                       | Hardware clears interrupt after a ACSRn register read                                                                                                                                                                   |
|                    |            | 1                                                                                                                                                                                                                       | Software clears interrupt                                                                                                                                                                                               |

## MediaLB Control 0 Register

The MLB\_CTL0 register contains bit that enable the module and provide module status. Note that the maximum speed for ML6-pin mode is 2048 x FS.

Figure 27-12: MLB\_CTL0 Register Diagram

<!-- image -->

Table 27-25: MLB\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17:15 (R/W)        | FCNT       | Frames Per Sub-buffer. The MLB_CTL0.FCNT bit field configures the frames per sub-buffer on synchronous channels.                                                                                                                                                            |
| 17:15 (R/W)        | FCNT       | 0 1 Frame per sub-buffer (Operation is the same as Stand- ard mode)                                                                                                                                                                                                         |
| 17:15 (R/W)        | FCNT       | 1 2 frames per sub-buffer                                                                                                                                                                                                                                                   |
| 17:15 (R/W)        | FCNT       | 2 4 frames per sub-buffer                                                                                                                                                                                                                                                   |
| 17:15 (R/W)        | FCNT       | 3 8 frames per sub-buffer                                                                                                                                                                                                                                                   |
| 17:15 (R/W)        | FCNT       | 4 16 frames per sub-buffer                                                                                                                                                                                                                                                  |
| 17:15 (R/W)        | FCNT       | 5 32 frames per sub-buffer                                                                                                                                                                                                                                                  |
| 17:15 (R/W)        | FCNT       | 6 64 frames per sub-buffer                                                                                                                                                                                                                                                  |
| 14 (R/W)           | CTLRETRY   | Control Tx Packet Retry. When the MLB_CTL0.CTLRETRY bit is set, a control packet that is flagged with a Break or Protocol error by the receiver is retransmitted. When cleared, a control packet that is flagged with a Break or Protocol error by the receiver is skipped. |

Table 27-25: MLB\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | ASYRETRY   | Asynchronous Tx Packet Retry. When the MLB_CTL0.ASYRETRY bit is set, an asynchronous packet that is flagged with a Break or Protocol error by the receiver is retransmitted. When cleared, an asyn- chronous packet that is flagged with a Break or ProtocolError by the receiver is skip-                                                                                                                                                                                                                                                         | Asynchronous Tx Packet Retry. When the MLB_CTL0.ASYRETRY bit is set, an asynchronous packet that is flagged with a Break or Protocol error by the receiver is retransmitted. When cleared, an asyn- chronous packet that is flagged with a Break or ProtocolError by the receiver is skip-                                                                                                                                                                                                                                                         |
| 7 (R/NW)           | LKSTAT     | ped. Lock Status. When the MLB_CTL0.LKSTAT bit is set (=0), the MediaLB block is synchronized to the incoming MediaLB frame with the following conditions. • If MLB_CTL1.LOCK =0 (unlocked), MLB_CTL1.LOCK is set after a FRAME- SYNC is detected at the same position for three consecutive frames. • If MLB_CTL1.LOCK =1 (locked), MLB_CTL1.LOCK is cleared after not receiv- ing a FRAMESYNC at the expected time for two consecutive frames. In this case FRAMESYNC patterns occurring at locations other than the expected one are ig- nored. | ped. Lock Status. When the MLB_CTL0.LKSTAT bit is set (=0), the MediaLB block is synchronized to the incoming MediaLB frame with the following conditions. • If MLB_CTL1.LOCK =0 (unlocked), MLB_CTL1.LOCK is set after a FRAME- SYNC is detected at the same position for three consecutive frames. • If MLB_CTL1.LOCK =1 (locked), MLB_CTL1.LOCK is cleared after not receiv- ing a FRAMESYNC at the expected time for two consecutive frames. In this case FRAMESYNC patterns occurring at locations other than the expected one are ig- nored. |
| 5 (R/W)            | PEN        | 6-pin Enable. The MLB_CTL0.PEN bit configures the MLB for 6-pin or 3-pin mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 6-pin Enable. The MLB_CTL0.PEN bit configures the MLB for 6-pin or 3-pin mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 5 (R/W)            | PEN        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | 3-pin interface enabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 4:2 (R/W)          | CLK        | Clock Speed Select. The MLB_CTL0.CLK bit field sets the clock speed.                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Clock Speed Select. The MLB_CTL0.CLK bit field sets the clock speed.                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 4:2 (R/W)          | CLK        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | 256 x Fs (MLB_CTL.PEN = 0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 4:2 (R/W)          | CLK        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | 512 x Fs (MLB_CTL.PEN = 0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 4:2 (R/W)          | CLK        | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | 1024 x Fs (MLB_CTL.PEN = 0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 4:2 (R/W)          | CLK        | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | 2048 x Fs (MLB_CTL.PEN = 1)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 4:2 (R/W)          | CLK        | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 4:2 (R/W)          | CLK        | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 4:2 (R/W)          | CLK        | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 4:2 (R/W)          | CLK        | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 0 (R/W)            | EN         | Enable. When the MLB_CTL0.EN bit is set (=1), MediaLB clock, signal, and data are re- ceived and transmitted on the appropriate MediaLB pins.                                                                                                                                                                                                                                                                                                                                                                                                      | Enable. When the MLB_CTL0.EN bit is set (=1), MediaLB clock, signal, and data are re- ceived and transmitted on the appropriate MediaLB pins.                                                                                                                                                                                                                                                                                                                                                                                                      |

## Control 1 Register

The MLB\_CTL1 register contains bits that provide lock status and control system commands.

Figure 27-13: MLB\_CTL1 Register Diagram

<!-- image -->

Table 27-26: MLB\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | NDA        | Node Device Address. The MLB_CTL1.NDA bit field is used for system commands directed to individual MediaLB nodes.                     |
| 7 (R/W0C)          | CLKM       | Lock Missing Status. The MLB_CTL1.CLKM bit is set when the MediaLB clock is not toggling at the pin. This bit is cleared by software. |
| 6 (R/W0C)          | LOCK       | Lock Error Status. The MLB_CTL1.LOCK bit is set when the MediaLB is unlocked. This bit is cleared by software.                        |

## MLB Global Control Register

The MLB\_GCTL register contains bits that manage the MLB clock.

Figure 27-14: MLB\_GCTL Register Diagram

<!-- image -->

Table 27-27: MLB\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | CLKOUTSEL  | CLKOUT Select. The MLB_GCTL.CLKOUTSEL bit selects either the MLB 3 pin clock or the 6 pin clock as MLBCLKOUT. |
| 2 (R/W)            | CLKOUTSEL  | 0 MLB 3 pin clock is selected for MLBCLKOUT                                                                   |
| 2 (R/W)            | CLKOUTSEL  | 1 MLB 6 pin clock is selected for MLBCLKOUT                                                                   |
| 1 (R/W)            | CLKOUTEN   | CLKOUT Enable.                                                                                                |

## HBI Channel Busy 0 Register

The HC can determine which channel(s) are busy by reading the MLB\_HCBR0 register. An HBI channel is busy if:

- it is currently loaded into one of the two AGUs
- the channel is enabled, CE = 1 from the Channel Allocation Table
- the DMA is active

When an HBI channel is busy, hardware may write back its local copy of the channel descriptor at any time. System software should not write a CDT descriptor for a channel that is busy. Only two HBI channels can be busy at any given time. Each bit of this register is read-only.

Figure 27-15: MLB\_HCBR0 Register Diagram

<!-- image -->

Table 27-28: MLB\_HCBR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | CHB        | Channel Busy. The MLB_HCBR0.CHB bit field contains the bitwise channel busy bit for channels 31:0. When a bit is cleared (=0) the channel is idle. When a bit is set (=1) the channel is busy. |

## HBI Channel Busy 1 Register

The HC can determine which channel(s) are busy by reading the MLB\_HCBR1 register. An HBI channel is busy if:

- it is currently loaded into one of the two AGUs
- the channel is enabled, CE = 1 from the Channel Allocation Table
- the DMA is active

When an HBI channel is busy, hardware may write back its local copy of the channel descriptor at any time. System software should not write a CDT descriptor for a channel that is busy. Only two HBI channels can be busy at any given time. Each bit of this register is read-only.

Figure 27-16: MLB\_HCBR1 Register Diagram

<!-- image -->

Table 27-29: MLB\_HCBR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | CHB        | Channel Busy. The MLB_HCBR1.CHB bit field contains the bitwise channel busy bit for channels 63:32. When a bit is cleared (=0) the channel is idle. When a bit is set (=1) the channel is busy. |

## HBI Channel Error 0 Register

The MLB\_HCER0 register indicates which channels (channels 31:0) have encountered fatal errors.

Figure 27-17: MLB\_HCER0 Register Diagram

<!-- image -->

Table 27-30: MLB\_HCER0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:0               | CERR       | Channel Error.                                                         |
| (R/W0C)            |            | The MLB_HCER0.CERR bit field reports bitwise errors for channels 31:0. |

## HBI Channel Error 1 Register

The MLB\_HCER1 register indicates which channel(s) have encountered fatal errors.

Figure 27-18: MLB\_HCER1 Register Diagram

<!-- image -->

Table 27-31: MLB\_HCER1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 31:0               | CERR       | Channel Error.                                                           |
| (R/W0C)            |            | The MLB_HCER1.CERR bite field reports bitwise errors for channels 63:32. |

## HBI Channel Mask 0 Register

The MLB\_HCMR0 register controls which channels (for channels 31:0) are able to generate an HBI interrupt.

Figure 27-19: MLB\_HCMR0 Register Diagram

<!-- image -->

Table 27-32: MLB\_HCMR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | CHM        | Channel Mask. The MLB_HCMR0.CHM bit field contains the bitwise channel mask bit for channels 31:0. When a bit is cleared (=0) the channel is masked. When set (=1) the channel is unmasked. |

## HBI Channel Mask 1 Register

The MLB\_HCMR1 register controls which channels (for channels 63:32) are able to generate an HBI interrupt.

Figure 27-20: MLB\_HCMR1 Register Diagram

<!-- image -->

Table 27-33: MLB\_HCMR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | CHM        | Bitwise channel mask bit for channels 63:32. The MLB_HCMR1.CHM bit field contains the bitwise channel mask bit for channels 63:32. When a bit is cleared (=0) the channel is masked. When set (=1) the channel is unmasked. |

## HBI Control Register

The MLB\_HCTL register controls and monitors general operation of the HBI block through the Address Generation Units) by reading and writing the register through the I/O interface. Each bit of this register is read/write.

Figure 27-21: MLB\_HCTL Register Diagram

<!-- image -->

Table 27-34: MLB\_HCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                         |
|--------------------|------------|-----------------------------------------------------------------|
| 15 (R/W)           | EN         | HBI Enable. Setting the MLB_HCTL.EN bit enables the HBI.        |
| 1 (R/W)            | RST1       | AGU1 Software Reset. Setting the MLB_HCTL.RST1 bit resets AGU1. |
| 0 (R/W)            | RST0       | AGU0 Software Reset. Setting the MLB_HCTL.RST0 bit resets AGU0. |

## Memory Interface Address Register

The MLB\_MADR register contains bit fields that contain the Channel Table RAM (CTR) or Data Buffer RAM (DBR) addresses. It also contains bits that set target location and read-not-write parameters.

Figure 27-22: MLB\_MADR Register Diagram

<!-- image -->

Table 27-35: MLB\_MADR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | WNR        | Write-Not-Read. The MLB_MADR.WNR bit selects Write-Not-Read. 0 Read                                                                                |
| 30 (R/W)           | TB         | Target Bit. The MLB_MADR.TB bit sets the target location. 0 Channel Table RAM (CTR)                                                                |
| 15:8 (R/W)         | ADDRH      | Address Higher Bits. The MLB_MADR.ADDRH bit field contains the DBR address of the 8-bit entry (bits [13:8]).                                       |
| 7:0 (R/W)          | ADDRL      | Address Lower Bits. The MLB_MADR.ADDRL bit field contains the CTR address of the 128-bit entry or the DBR address of the 8-bit entry (bits [7:0]). |

## Memory Interface Control Register

The MLB\_MCTL register contains a bit that indicates that the data transfer is complete.

Figure 27-23: MLB\_MCTL Register Diagram

<!-- image -->

Table 27-36: MLB\_MCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                             |
|--------------------|------------|---------------------------------------------------------------------|
| 0                  | XCMP       | Transfer Complete (Write 0 to clear).                               |
| (R/W0C)            |            | The MLB_MCTL.XCMP bit indicates that the data transfer is complete. |

## Memory Interface Control Data 0 Register

The MLB\_MDAT0 register contains Channel Table RAM (CTR) data (bits [31:0] of 128-bit entry) or Data Buffer RAM (DBR) data (bits [7:0] of 8-bit entry).

Figure 27-24: MLB\_MDAT0 Register Diagram

<!-- image -->

Table 27-37: MLB\_MDAT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                     |
|--------------------|------------|-------------------------------------------------------------|
| 31:0               | DATA       | DATA.                                                       |
| (R/W)              |            | The MLB_MDAT0.DATA bit field contains CTR data or DBR data. |

## Memory Interface Control Data 1 Register

The MLB\_MDAT1 register contains Channel Table RAM (CTR) data (bits [63:32] of 128-bit entry).

Figure 27-25: MLB\_MDAT1 Register Diagram

<!-- image -->

Table 27-38: MLB\_MDAT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                         |
|--------------------|------------|-------------------------------------------------|
| 31:0               | DATA       | DATA.                                           |
| (R/W)              |            | The MLB_MDAT1.DATA bit field contains CTR data. |

## Memory Interface Control Data 2 Register

The MLB\_MDAT2 register contains Channel Table RAM (CTR) data (bits [95:64] of 128-bit entry).

Figure 27-26: MLB\_MDAT2 Register Diagram

<!-- image -->

Table 27-39: MLB\_MDAT2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                         |
|--------------------|------------|-------------------------------------------------|
| 31:0               | DATA       | DATA.                                           |
| (R/W)              |            | The MLB_MDAT2.DATA bit field contains CTR data. |

## Memory Interface Control Data 3 Register

The MLB\_MDAT3 register contains Channel Table RAM (CTR) data (bits [127:96] of 128-bit entry).

Figure 27-27: MLB\_MDAT3 Register Diagram

<!-- image -->

Table 27-40: MLB\_MDAT3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                         |
|--------------------|------------|-------------------------------------------------|
| 31:0               | DATA       | DATA.                                           |
| (R/W)              |            | The MLB_MDAT3.DATA bit field contains CTR data. |

## Memory Interface Control Data Write Enable 0 Register

The MLB\_MDWE0 register contains the bitwise write enable for Channel Table RAM (CTR) data bits [31:0]. When cleared (=0) the bit is disabled, when set (=1) the bit is enabled.

Figure 27-28: MLB\_MDWE0 Register Diagram

<!-- image -->

Table 27-41: MLB\_MDWE0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                     |
|--------------------|------------|-----------------------------------------------------------------------------|
| 31:0               | MSK        | Write Enable.                                                               |
| (R/W)              |            | The MLB_MDWE0.MSK bit field contains the bitwise write enable for CTR data. |

## Memory Interface Control Data Write Enable 1 Register

The MLB\_MDWE1 register contains the bitwise write enable for Channel Table RAM (CTR) data bits [63:32]. When cleared (=0), the bit is disabled. When set (=1), the bit is enabled.

Figure 27-29: MLB\_MDWE1 Register Diagram

<!-- image -->

Table 27-42: MLB\_MDWE1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                     |
|--------------------|------------|-----------------------------------------------------------------------------|
| 31:0               | MSK        | Bitwise write enable for CTR data - bits[63:32].                            |
| (R/W)              |            | The MLB_MDWE1.MSK bit field contains the bitwise write enable for CTR data. |

## Memory Interface Control Data Write Enable 2 Register

The MLB\_MDWE2 register contains the bitwise write enable for Channel Table RAM (CTR) data bits [95:64]. When cleared (=0), the bit is disabled. When set (=1), the bit is enabled.

Figure 27-30: MLB\_MDWE2 Register Diagram

<!-- image -->

Table 27-43: MLB\_MDWE2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                     |
|--------------------|------------|-----------------------------------------------------------------------------|
| 31:0               | MSK        | Bitwise write enable for CTR data - bits[95:64].                            |
| (R/W)              |            | The MLB_MDWE2.MSK bit field contains the bitwise write enable for CTR data. |

## Memory Interface Control Data Write Enable 3 Register

The MLB\_MDWE3 register contains the bitwise write enable for Channel Table RAM (CTR) data bits [127:96]. When cleared (=0), the bit is disabled. When set (=1), the bit is enabled.

Figure 27-31: MLB\_MDWE3 Register Diagram

<!-- image -->

Table 27-44: MLB\_MDWE3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                     |
|--------------------|------------|-----------------------------------------------------------------------------|
| 31:0               | MSK        | Bitwise write enable for CTR data - bits[127:96].                           |
| (R/W)              |            | The MLB_MDWE3.MSK bit field contains the bitwise write enable for CTR data. |

## Interrupt Enable Register

The MLB\_MIEN register is used to enable various interrupt conditions.

Figure 27-32: MLB\_MIEN Register Diagram

<!-- image -->

Table 27-45: MLB\_MIEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | CTXBREAK   | Control Transmit Break Enable. When the MLB_MIEN.CTXBREAK bit is set, a ReceiverBreak response received from the receiver on a control Tx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set. |
| 28 (R/W)           | CTXPE      | Control Transmit Protocol Error Enable. When the MLB_MIEN.CTXPE bit is set, a ProtocolError generated by the receiver on a control Tx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.     |
| 27 (R/W)           | CTXDONE    | Control Transmit Packet Done Enable. When the MLB_MIEN.CTXDONE bit is set, a packet transmitted with no errors on a control Tx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.            |

Table 27-45: MLB\_MIEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | CRXBREAK   | Control Receive Break Enable. When the MLB_MIEN.CRXBREAK bit is set, a ControlBreak command received from the transmitter on a control Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.            |
| 25 (R/W)           | CRXPE      | Control Receive Protocol Error Enable. When the MLB_MIEN.CRXPE bit is set, a ProtocolError detected on a control Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.                                  |
| 24 (R/W)           | CRXDONE    | Control Receive Packet Done Enable. When the MLB_MIEN.CRXDONE bit is set, a packet received with no errors on a control Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.                           |
| 22 (R/W)           | ATXBREAK   | Asynchronous Transmit Break Enable. When the MLB_MIEN.ATXBREAK bit is set, a ReceiverBreak response received from the receiver on an asynchronous Tx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set. |
| 21 (R/W)           | ATXPE      | Asynchronous Transmit Protocol Error Enable. When the MLB_MIEN.ATXPE bit is set, a ProtocolError generated by the receiver on an asynchronous Tx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.     |
| 20 (R/W)           | ATXDONE    | Asynchronous Transmit Done Enable. When the MLB_MIEN.ATXDONE bit is set, a packet transmitted with no errors on an asynchronous Tx channel channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.           |
| 19 (R/W)           | ARXBREAK   | Asynchronous Receive Break Enable. When the MLB_MIEN.ARXBREAK bit is set, a AsyncBreak command received from the transmitter on an asynchronous Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.   |
| 18 (R/W)           | ARXPE      | Asynchronous Receive Protocol Error Enable. When the MLB_MIEN.ARXPE bit is set, a protocol error detected on an asynchro- nous Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.                    |
| 17 (R/W)           | ARXDONE    | Asynchronous Receive Packet Done Enable. When the MLB_MIEN.ARXDONE bit is set, a packet received with no errors on an asynchronous Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.                |

Table 27-45: MLB\_MIEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | SYNCPE     | Synchronous Protocol Error Enable. When the MLB_MIEN.SYNCPE bit is set, a protocol error detected on a synchronous Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set. This occurs only when isochronous flow control is disabled.     |
| 1 (R/W)            | ISOCBUFO   | Isochronous Receive Buffer Overflow Enable. When the MLB_MIEN.ISOCBUFO bit is set, a buffer overflow on an isochronous Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set. This occurs only when isochronous flow control is disabled. |
| 0 (R/W)            | ISOCPE     | Isochronous Receive Protocol Error Enable. When the MLB_MIEN.ISOCPE bit is set, a protocol error detected on an isochro- nous Rx channel causes the appropriate channel bit in the MLB_MS0 or MLB_MS1 registers to be set.                                                      |

## Channel Status 0 Register

The MLB\_MS0 register indicates the channel status for MediaLB channels 31 to 0. Channel status bits are set by hardware and cleared by software. Status is only set if the appropriate bits in the MLB\_MIEN register are set.

Figure 27-33: MLB\_MS0 Register Diagram

<!-- image -->

Table 27-46: MLB\_MS0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W0C)       | MCS        | MediaLB Channel Status. The MLB_MS0.MCS bit field indicates the MediaLB channel status for channels 31 to 0. |

## Channel Status 1 Register

The MLB\_MS1 register indicates the channel status for MediaLB channels 32 to 63. Channel status bits are set by hardware and cleared by software. Status is only set if the appropriate bits in the MLB\_MIEN register are set.

Figure 27-34: MLB\_MS1 Register Diagram

<!-- image -->

Table 27-47: MLB\_MS1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W0C)       | MCS        | MediaLB Channel Status. The MLB_MS1.MCS bit field indicates the MediaLB channel status for channels 63 to 32. |

## System Data Register

The MLB\_MSD register allows system software to receive control information from the MLB controller. The MLB\_MSD register is updated once per frame by the hardware during the MLB system channel.

The MLB\_MSD register is loaded with the data from the MLBDAT\_IN signal during the system channel quadlet. System software must read this register before the start of the next MLB frame to prevent the current data from being lost.

Figure 27-35: MLB\_MSD Register Diagram

<!-- image -->

Table 27-48: MLB\_MSD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | SD3        | System Data Byte 3 (MSB). The MLB_MSD.SD3 bits are updated with MediaLB Data [31:24] when a MediaLB software system command is received in the system quadlet. If the MLB_MSS.SWSYSCMD bit is already set, then SD3 is not updated. |
| 23:16 (R/NW)       | SD2        | System Data Byte 2. The MLB_MSD.SD2 bits are updated with MediaLB Data [23:16] when a MediaLB software system command is received in the system quadlet. If the MLB_MSS.SWSYSCMD bit is already set, then SD2 is not updated.       |
| 15:8 (R/NW)        | SD1        | System Data Byte 1. The MLB_MSD.SD1 bits are updated with MediaLB Data [15:8] when a MediaLB software system command is received in the system quadlet. If the MLB_MSS.SWSYSCMD bit is already set, then SD1 is not updated.        |
| 7:0 (R/NW)         | SD0        | System Data Byte 0 (LSB). The MLB_MSD.SD0 bits are updated with MediaLB Data [7:0] when a MediaLB software system command is received in the system quadlet. If the MLB_MSS.SWSYSCMD bit is already set, then SD0 is not updated.   |

## System Status Register

The MLB\_MSS register allows system software to monitor and control the status of the MLB network. The register is updated once per frame by hardware during the MLB system channel. The bits of this register are not valid until the processor is locked to the MLB interface (except for the bits associated with MLB lock and unlock, SDMU and SDML). System software must service events before the start of the next MLB frame to prevent the current frame status from being lost.

Figure 27-36: MLB\_MSS Register Diagram

<!-- image -->

Table 27-49: MLB\_MSS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | SERVREQ    | Service Request Enable. When the MLB_MSS.SERVREQ bit set, the MediaLB block responds with a device present, request service system response if a matching channel scan system command is detected. When cleared, the MediaLB block responds with a device present system re- sponse.                                                                                                                                                            |
| 4 (R/W0C)          | SWSYSCMD   | Software System Command Detected. The MLB_MSS.SWSYSCMD bit indicates that a software system command was de- tected (in the system quadlet). The MLB_MSS.SWSYSCMD bit is set by hardware, cleared by software. Data is stored in the MLB_MSD register for this command.                                                                                                                                                                          |
| 3 (R/W0C)          | CSSYSCMD   | Channel Scan System Command Detected. The MLB_MSS.CSSYSCMD bit indicates that a channel scan system command was detected (in the system quadlet). The MLB_MSS.CSSYSCMD bit is set by hardware, cleared by software. If the node address specified in the data quadlet matches the value in the MLB_CTL1.NDA bit field, the device responds either device present or device present, request service system response in the next system quadlet. |

Table 27-49: MLB\_MSS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W0C)          | ULKSYSCMD  | Network Unlock System Command Detected. The MLB_MSS.ULKSYSCMD bit indicates a network unlock system command was detected (in the system quadlet). The MLB_MSS.ULKSYSCMD bit is by hardware, cleared by software.    |
| 1 (R/W0C)          | LKSYSCMD   | Network Lock System Command Detected. The MLB_MSS.LKSYSCMD bit indicates a network lock system command was de- tected (in the system quadlet). The MLB_MSS.LKSYSCMD bit is set by hardware and cleared by software. |
| 0 (R/W0C)          | RSTSYSCMD  | Reset System Command Detected. The MLB_MSS.RSTSYSCMD bit indicates a reset system command was detected (in the system quadlet). The MLB_MSS.RSTSYSCMD bit is set by hardware, cleared by software.                  |

## MediaLB 6-pin Control 0 Register

Figure 27-37: MLB\_PCTL0 Register Diagram

<!-- image -->

Table 27-50: MLB\_PCTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                    |
|--------------------|------------|--------------------------------------------|
| 1                  | CMRES      | MediaLB 6-pin common mode resistor enable. |
| (R/W)              |            | This signal is no longer used.             |