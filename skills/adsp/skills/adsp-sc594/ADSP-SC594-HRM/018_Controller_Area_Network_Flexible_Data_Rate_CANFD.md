# Controller Area Network Flexible Data Rate (CANFD)

<!-- source: 018_Controller_Area_Network_Flexible_Data_Rate_CANFD.pdf | original pages 713–917 -->

## 16   Controller Area Network Flexible Data Rate (CANFD)

The controller area network (CAN) protocol is primarily for use as a vehicle serial data bus. It meets the specific requirements of this field, including real-time processing, reliable operation in the EMI environment of a vehicle, cost-effectiveness, and required bandwidth.

The CANFD module is a full implementation of the CAN protocol specification, the CAN with flexible data rate (CAN FD) protocol, and the CAN 2.0 Part B protocol. It supports both standard and extended message frames and long payloads up to 64 bytes, transferred at rates up to 8 Mbps. The message buffers are stored in an embedded RAM dedicated to the CANFD module.

NOTE: This document assumes familiarity with the CAN standard. For more information, refer to Version 2.0 of the CAN specification from Robert Bosch GmbH.

## CANFD Features

Key features of the CANFD module include:

- Full implementation of the CAN FD protocol and the CAN specification 2.0 (Part B) including:
- Standard data frames
- Extended data frames
- Zero to sixty-four bytes data length
- Programmable bit rate
- Content-related addressing
- ISO 11898-1 standard compliance
- Flexible mailboxes
- Configurable data lengths from 0 to 64 bytes
- Configurable as receive or transmit, supporting standard and extended messages
- Individual Rx mask registers per mailbox

- Full-featured Rx FIFO
- Storage capacity for up to six frames
- Automatic internal pointer handling with DMA support
- Transmission abort capability
- 28 message buffers of 8 bytes data length each, configurable as Rx or Tx
- Listen-Only mode capability
- Programmable Loop-Back mode supporting self-test operation
- Programmable transmission priority scheme: lowest ID, lowest buffer number, or highest priority
- Time stamp based on 16-bit free-running timer
- Global network time, synchronized by a specific message
- Independence from the transmission medium (an external transceiver is required)
- Low-power modes, with programmable wakeup on bus activity or matching with received frames (Pretended Networking)
- Transceiver Delay Compensation feature when transmitting CAN FD messages at faster data rates
- Remote request frames may be handled automatically or by software
- Identifier Acceptance Filter Hit Indicator (IDHIT) register for received frames
- Status of synchronization with CAN bus
- CRC status for transmitted message
- Rx FIFO Global Mask register
- Selectable priority between mailboxes and Rx FIFO during matching process
- Rx FIFO ID filtering, matching incoming IDs against 128 extended, 256 standard, or 512 partial (8 bit) IDs
- Up to 32 ID filter table elements
- Detection and correction of memory read access errors, with five parity bits for each byte of CANFD memory
- Pretended networking functionality in low-power doze mode

## CANFD Functional Description

The CANFD module is a CAN protocol engine with a very flexible mailbox system for transmitting and receiving CAN frames. The mailbox system consists of a set of message buffers (MBs) that store configuration and control data, time stamp, message ID, and data. The memory corresponding to the first 38 MBs is configurable to support a FIFO reception scheme with a powerful ID filtering mechanism. The ID filtering mechanism is capable of checking

incoming frames against a table of IDs (up to 128 extended IDs, 256 standard IDs, or 512 8-bit ID slices), with an individual mask register for up to 32 ID filter table elements.

Classical CAN frames support simultaneous reception through a FIFO and mailboxes. CAN FD frames support reception only through mailboxes. For mailbox reception, there is a matching algorithm to store received frames into the MBs that have the same ID programmed in the ID field. There is a masking scheme to match the ID programmed on the MB with a range of IDs on received CAN frames. For transmission, an arbitration algorithm decides the prioritization of MBs to be transmitted based on the message ID (optionally augmented by 3 local priority bits) or the MB ordering.

The CANFD module can also receive and transmit messages in CAN FD format. The MBs are sized to store the quantity of data bytes selected in the MBDSRn fields of the CANFD\_FD\_CTL register.

The following sections provides listings of the CANFD registers, interrupts, and triggers. It section also provides information on the architectural concepts and functional operation of the CANFD module.

## ADSP-2159x\_SC592\_SC594 CANFD Register List

CANFD module is a communication controller implementing the CAN protocol according to the CAN 2.0B protocol specification. Features : Flexible DataRate, PretendedNetwork, DMA support, ECC for RAM.

Table 16-1: ADSP-2159x\_SC592\_SC594 CANFD Register List

| Name            | Description                             |
|-----------------|-----------------------------------------|
| CANFD_TIMING    | Can Bit Timing Register                 |
| CANFD_CRC       | CRC Register                            |
| CANFD_CTL1      | Control 1 Register                      |
| CANFD_PN_CTL1   | Pretended Networking Control1 Register  |
| CANFD_CTL2      | Control 2 Register                      |
| CANFD_PN_CTL2   | Pretended Networking Control2 Register  |
| CANFD_ECR       | Error Counter Register                  |
| CANFD_ERR_IADDR | Error Injection Address Register        |
| CANFD_ERR_IDP   | Error Injection Data Pattern Register   |
| CANFD_ERR_IPP   | Error Injection Parity Pattern Register |
| CANFD_ERR_STAT  | Error Status Register                   |
| CANFD_ESR1      | Error and Status 1 Register             |
| CANFD_ESR2      | Error and Status 2 Register             |
| CANFD_FD_TIMING | CANFD Bit Timing Register               |
| CANFD_FD_CRC    | CANFD CRC Register                      |
| CANFD_FD_CTL    | CANFD Control Register                  |

Table 16-1: ADSP-2159x\_SC592\_SC594 CANFD Register List (Continued)

| Name                     | Description                                                                            |
|--------------------------|----------------------------------------------------------------------------------------|
| CANFD_FLTR_DLC           | Pretended Networking DLC Filter Register                                               |
| CANFD_FLTR_ID1           | Pretended Networking ID Filter1 Register                                               |
| CANFD_FLTR_ID2_IDMSK     | Pretended Networking ID Filter2 / IDMask Register                                      |
| CANFD_IFLG1              | Mailbox Interrupt Flag 1 Register                                                      |
| CANFD_IFLG2              | Mailbox Interrupt Flag 2 Register                                                      |
| CANFD_IMSK1              | Mailbox Interrupt Mask 1 Register                                                      |
| CANFD_IMSK2              | Mailbox Interrupt Mask 2 Register                                                      |
| CANFD_CFG                | Module Configuration Register                                                          |
| CANFD_MEC                | Memory Error Control Register                                                          |
| CANFD_FLTR_DATA1_HI      | Pretended Networking Payload Low Filter2 Register                                      |
| CANFD_FLTR_DATA1_LO      | Pretended Networking Payload Low Filter1 Register                                      |
| CANFD_FLTR_DATA2_DMSK_HI | Pretended Networking Payload High Filter2 High Order Bits / Payload High Mask Register |
| CANFD_FLTR_DATA2_DMSK_LO | Pretended Networking Payload Low Filter2 / Payload Low Mask Register                   |
| CANFD_ERR_RADDR          | Error Report Address Register                                                          |
| CANFD_ERR_RDAT           | Error Report Data Register                                                             |
| CANFD_ERR_RSYN           | Error Report Syndrome Register                                                         |
| CANFD_RX_14_MSK          | Receive Mailbox14 Mask Register                                                        |
| CANFD_RX_15_MSK          | Receive Mailbox15 Mask Register                                                        |
| CANFD_RX_FIFO_GMSK       | Receive FIFO Global Mask Register                                                      |
| CANFD_RX_FIFO            | Receive FIFO Information Register                                                      |
| CANFD_RX_IMSK[n]         | Receive Individual Mask Register                                                       |
| CANFD_RX_MB_GMSK         | Receive Mailbox Global Mask Register                                                   |
| CANFD_TMR                | Free Running Timer Register                                                            |
| CANFD_WMB[n]_DATA_HI     | Wakeup Message Buffer Data 4-7 Register                                                |
| CANFD_WMB[n]_DATA_LO     | Wakeup Message Buffer Data 0-3 Register                                                |
| CANFD_WMB[n]_ID          | Wakeup Message ID Buffer Register                                                      |
| CANFD_WMB[n]_STAT        | Wakeup Message Buffer Control/Status Register                                          |
| CANFD_WUM                | Pretended Networking Wakeup Match Register                                             |

## ADSP-2159x\_SC592\_SC594 CANFD Interrupt List

Table 16-2: ADSP-2159x\_SC592\_SC594 CANFD Interrupt List

|   Interrupt ID | Name       | Description                                      | Sensitivity   | DMA Channel   |
|----------------|------------|--------------------------------------------------|---------------|---------------|
|            231 | CANFD0_WU  | CANFD0 CAN0 Wakeup Interrupt                     | None          |               |
|            233 | CANFD0_IRQ | CANFD0 CAN0 Interrupt                            | None          |               |
|            235 | CANFD0_MSG | CANFD0 CAN0 Message Receive/Trans- mit Interrupt | None          |               |
|            236 | CANFD1_WU  | CANFD1 CAN1 Wakeup Interrupt                     | None          |               |
|            238 | CANFD1_IRQ | CANFD1 CAN1 Interrupt                            | None          |               |
|            240 | CANFD1_MSG | CANFD1 CAN1 Message Receive/Trans- mit Interrupt | None          |               |

## ADSP-2159x\_SC592\_SC594 CANFD Trigger List

Table 16-3: ADSP-2159x\_SC592\_SC594 CANFD Trigger List Generators

|   Trigger ID | Name           | Description                      | Sensitivity   |
|--------------|----------------|----------------------------------|---------------|
|          175 | CANFD0_IPD_REQ | CANFD0 CAN0 DMArequest interrupt | None          |
|          176 | CANFD1_IPD_REQ | CANFD1 CAN1 DMArequest interrupt | None          |

Table 16-4: ADSP-2159x\_SC592\_SC594 CANFD Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## CANFD Architectural Concepts

The following sections provide information about the CANFD architecture.

## Block Diagram

The CANFD Block Diagram figure shows a block diagram of the CANFD module.

Figure 16-1: CANFD Controller Block Diagram

<!-- image -->

The CANFD module has the following three submodules and message buffers in an embedded RAM:

## Protocol Engine (PE)

The PE submodule manages the serial communication on the CAN bus. It requests RAM access for receiving and transmitting message frames, validates received messages, preforms error handling, and detects CAN FD messages.

## Controller Host Interface (CHI)

The CHI submodule handles message buffer selection for reception and transmission, including arbitration, and ID matching algorithms for both flexible data rate (FD) and non-FD message formats.

## Bus Interface Unit (BIU)

The BIU submodule controls the access to and from the internal interface bus to establish connection to the CPU and to other blocks. Clocks, address and data buses, interrupt outputs, DMA, and test signals are accessed through the BIU.

## Message Buffer (MB)

The CANFD module uses a message buffer structure that represents both extended (29-bit identifier) and standard (11-bit identifier) frames from the CAN specification (Version 2.0, Part B).

Each individual message buffer is 16, 24, 40, or 72 bytes, depending on the quantity of data bytes allocated for the message payload: 8, 16, 32, or 64 data bytes, respectively.

The memory area for mailboxes is from address offset 0x80 to 0x47F . When CAN FD is enabled, the exact address for each memory byte depends on the size of the payload.

The Message Buffer Structure 64-Byte Payload figure shows an example of the message buffer structure with a 64-byte payload.

Figure 16-2: Message Buffer Structure 64-Byte Payload

<!-- image -->

| 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16   | 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0   | 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0   | 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0   | 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0   | 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0   |
|---------------------------------------------------|-----------------------------------------|-----------------------------------------|-----------------------------------------|-----------------------------------------|-----------------------------------------|
| CODE 0x00                                         | E D L B R S E S I                       | S R R I D E R T R                       | DLC                                     | TIME STAMP                              | TIME STAMP                              |
| 0x04                                              | PRIO ID STANDARD/ ID EXTENDED MSB       | PRIO ID STANDARD/ ID EXTENDED MSB       | PRIO ID STANDARD/ ID EXTENDED MSB       | ID EXTENDED LSB                         | ID EXTENDED LSB                         |
| 0x08 DATA BYTE 0                                  | 0x08 DATA BYTE 0                        | DATA BYTE 1                             | DATA BYTE 1                             | DATA BYTE 2                             | DATA BYTE 3                             |
| 0x0C DATA BYTE 4                                  | 0x0C DATA BYTE 4                        | DATA BYTE 5                             | DATA BYTE 5                             | DATA BYTE 6                             | DATA BYTE 7                             |
| 0x10 DATA BYTE 8                                  | 0x10 DATA BYTE 8                        | DATA BYTE 9                             | DATA BYTE 9                             | DATA BYTE 10                            | DATA BYTE 11                            |
| 0x14 DATA BYTE 8                                  | 0x14 DATA BYTE 8                        | DATA BYTE 9                             | DATA BYTE 9                             | DATA BYTE 10                            | DATA BYTE 15                            |
| 0x18 DATA BYTE 16                                 | 0x18 DATA BYTE 16                       | DATA BYTE 17                            | DATA BYTE 17                            | DATA BYTE 18                            | DATA BYTE 19                            |
| 0x1C DATA BYTE 20                                 | 0x1C DATA BYTE 20                       | DATA BYTE 21                            | DATA BYTE 21                            | DATA BYTE 22                            | DATA BYTE 23                            |
| 0x20 DATA BYTE 24                                 | 0x20 DATA BYTE 24                       | DATA BYTE 25                            | DATA BYTE 25                            | DATA BYTE 26                            | DATA BYTE 27                            |
| 0x24 DATA BYTE 28                                 | 0x24 DATA BYTE 28                       | DATA BYTE 29                            | DATA BYTE 29                            | DATA BYTE 30                            | DATA BYTE 31                            |
| 0x28 DATA BYTE 32                                 | 0x28 DATA BYTE 32                       | DATA BYTE 33                            | DATA BYTE 33                            | DATA BYTE 34                            | DATA BYTE 35                            |
| 0x2C DATA BYTE 36                                 | 0x2C DATA BYTE 36                       | DATA BYTE 37                            | DATA BYTE 37                            | DATA BYTE 38                            | DATA BYTE 39                            |
| 0x30 DATA BYTE 40                                 | 0x30 DATA BYTE 40                       | DATA BYTE 41                            | DATA BYTE 41                            | DATA BYTE 42                            | DATA BYTE 43                            |
| 0x34 DATA BYTE 44                                 | 0x34 DATA BYTE 44                       | DATA BYTE 45                            | DATA BYTE 45                            | DATA BYTE 46                            | DATA BYTE 47                            |
| 0x38 DATA BYTE 48                                 | 0x38 DATA BYTE 48                       | DATA BYTE 49                            | DATA BYTE 49                            | DATA BYTE 50                            | DATA BYTE 51                            |
| 0x3C DATA BYTE 52                                 | 0x3C DATA BYTE 52                       | DATA BYTE 53                            | DATA BYTE 53                            | DATA BYTE 54                            | DATA BYTE 55                            |
| 0x40 DATA BYTE 56                                 | 0x40 DATA BYTE 56                       | DATA BYTE 57                            | DATA BYTE 57                            | DATA BYTE 58                            | DATA BYTE 59                            |
| 0x44 DATA BYTE 60                                 | 0x44 DATA BYTE 60                       | DATA BYTE 61                            | DATA BYTE 61                            | DATA BYTE 62                            | DATA BYTE 63                            |

In the example, the first eight bytes of the message buffer is control information associated with the data, followed by the bytes of data in the payload. The control information is defined in the following sections:

## Extended Data Length (EDL)

The EDL bit distinguishes between CAN and CAN FD format frames. Do not set EDL when message buffers are configured to RANSWER with code field 0b1010.

## Bit Rate Switch (BRS)

The BRS bit defines whether the bit rate is switched inside a CAN FD format frame.

## Error State Indicator (ESI)

The ESI bit indicates if the transmitting node is error active or error passive.

## Message Buffer Code (CODE)

The CODE field is accessible (read or write) to the processor and the CANFD module, as part of the message buffer matching and arbitration process. The Message Buffer Code - Rx Buffer and Message Buffer Code - Tx Buffer tables show the encoding.

Table 16-5: Message Buffer Code - Rx Buffer

| CODE Descrip- tion                                             | Rx Code Before New Frame   | SRV *1   | Rx Code After Successful Re- ception *2   | RRS *3   | Comment                                                                                                                                                                                                                                      |
|----------------------------------------------------------------|----------------------------|----------|-------------------------------------------|----------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0000: INAC- TIVE - MBis not active.                            | INACTIVE                   | N/A      | N/A                                       | N/A      | MBdoes not participate in the matching process.                                                                                                                                                                                              |
| 0100: EMPTY - MBis active and empty                            | EMPTY                      | N/A      | FULL                                      | N/A      | When a frame is received successfully, the CODE field is automatically updated to FULL.                                                                                                                                                      |
| 0010: FULL - MBis full.                                        | FULL                       | Yes      | FULL                                      | N/A      | The act of reading the C/S word followed by unlock- ing the MB(SRV) does not make the code return to EMPTY. It remains FULL. If a new frame is moved to the MBafter the MBwas serviced, the code still re- mains FULL. See Matching Process. |
| 0010: FULL - MBis full.                                        | FULL                       | No       | OVERRUN                                   | N/A      | If the MBis FULL and a new frame is moved to this MBbefore the CPU services it, the CODE field is au- tomatically updated to OVERRUN. See Matching Process.                                                                                  |
| 0110: OVER- RUN - MBis being overwrit- ten into a full buffer. | OVERRUN                    | Yes      | FULL                                      | N/A      | If the CODE field indicates OVERRUN and CPU has serviced the MB, when a new frame is moved to the MB, the code returns to FULL.                                                                                                              |
| 0110: OVER- RUN - MBis being overwrit- ten into a full buffer. | OVERRUN                    | No       | OVERRUN                                   | N/A      | If the CODE field already indicates OVERRUN, and another new frame must be moved, the MBwill be overwritten again, and the code will remain OVER- RUN. See Matching Process.                                                                 |

Table 16-5: Message Buffer Code - Rx Buffer (Continued)

| CODE Descrip- tion                                                                                                         | Rx Code Before New Frame   | SRV *1   | Rx Code After Successful Re- ception *2   | RRS *3   | Comment                                                                                                                                                                                                                                                                                                                                         |
|----------------------------------------------------------------------------------------------------------------------------|----------------------------|----------|-------------------------------------------|----------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1010: RANSW- ER *4 - A frame was configured to recognize a remote request frame and trans- mit a response frame in return. | RANSWER                    | N/A      | TANSW- ER(1110)                           | 0        | A Remote Answer was configured to recognize a re- mote request frame received. After that, an MBis set to transmit a response frame. The code is automatical- ly changed to TANSWER (1110). See Matching Process. If the CANFD_CTL2.RRS bit is negated, transmit a response frame whenever a remote request frame with the same ID is received. |
| 1010: RANSW- ER *4 - A frame was configured to recognize a remote request frame and trans- mit a response frame in return. | RANSWER                    | N/A      | N/A                                       | 1        | This code is ignored during matching and arbitration process.                                                                                                                                                                                                                                                                                   |
| CODE[0]=1; BUSY - CANFD is up- dating the con- tents of the MB. The CPU must not access the MB.                            | BUSY *5                    | N/A      | FULL                                      | N/A      | Indicates that the MBis being updated. It will be ne- gated automatically and does not interfere with the next CODE.                                                                                                                                                                                                                            |
| CODE[0]=1; BUSY - CANFD is up- dating the con- tents of the MB. The CPU must not access the MB.                            | BUSY *5                    | N/A      | OVERRUN                                   | N/A      | Indicates that the MBis being updated. It will be ne- gated automatically and does not interfere with the next CODE.                                                                                                                                                                                                                            |

- *1 Serviced MB. The MB has been read and unlocked by reading the CANFD\_TMR register or other MB.
- *2 A frame has a successful reception after it is moved to an MB. See Move Process.
- *3 Remote Request Stored bit, see the CANFD\_CTL2 register.
- *4 Code 1010 is not considered Tx and an MB with this code should not be aborted.
- *5 6. For Tx MBs, the BUSY bit is ignored upon read, except when the CANFD\_CFG.ABORTEN bit is set. If this bit is asserted, the corresponding MB does not participate in the matching process.

Table 16-6: Buffer Message Code - Tx Buffer

| CODE Description                                      | Tx Code Before Tx Frame   | MBRTR   | Tx Code After Successful Trans- mission   | Comment                                                                                                            |
|-------------------------------------------------------|---------------------------|---------|-------------------------------------------|--------------------------------------------------------------------------------------------------------------------|
| 1000: INACTIVE - MBis not active.                     | INACTIVE                  | N/A     | N/A                                       | MBdoes not participate in arbitration process.                                                                     |
| 1001: ABORT - MBis aborted.                           | ABORT                     | N/A     | N/A                                       | MBdoes not participate in arbitration process.                                                                     |
| 1100: DATA - MBis a Tx Data Frame (MB RTR must be 0). | DATA                      | 0       | INACTIVE                                  | Transmit data frame unconditionally once. After transmission, the MBau- tomatically returns to the INACTIVE state. |

Table 16-6: Buffer Message Code - Tx Buffer (Continued)

| CODE Description                                                                    | Tx Code Before Tx Frame   | MBRTR   | Tx Code After Successful Trans- mission   | Comment                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|-------------------------------------------------------------------------------------|---------------------------|---------|-------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1100: REMOTE - MBis a Tx Re- mote Request Frame (MB RTR must be 1).                 | REMOTE                    | 1       | EMPTY                                     | Transmit remote request frame un- conditionally once. After transmis- sion, the MBautomatically becomes an Rx Empty MBwith the same ID.                                                                                                                                                                                                                                                                                                                                                                     |
| 1110: TANSWER - MBis a Tx Re- sponse Frame from an incoming Re- mote Request Frame. | TANSWER                   | N/A     | RANSWER                                   | This is an intermediate code that is automatically written to the MBby the CHI as a result of a match to a re- mote request frame. The remote re- sponse frame will be transmitted un- conditionally once, and then the code will automatically return to RANSW- ER (1010). The CPU can also write this code with the same effect. The remote re- sponse frame can be either a data frame or another remote request frame depending on the RTR bit val- ue. See Matching Process and Arbitra- tion Process. |

## Substitute Remote Request (SRR)

SRR is a fixed recessive bit, only for the extended format. The SRR bit is set to one for transmission (Tx buffers) and is stored with the value received on the CAN bus for Rx receiving buffers. It is received as either recessive or dominant. If the CANFD module receives this bit as dominant, then it is interpreted as an arbitration loss.

When the bit is set, the recessive value is compulsory for transmission in extended format frames. When it is clear, dominant is not a valid value for transmission in extended format frames.

## ID Extended Bit (IDE)

IDE identifies whether the frame format is standard or extended.

When the bit is set, the frame format is extended. When the bit is clear, the frame format is standard.

## Remote Transmission Request (RTR)

The RTR bit affects the behavior of remote frames and is part of the reception filter:

When the bit is set, the current MB may have a remote request frame to be transmitted if MB is Tx. If the MB is Rx, then incoming remote request frames may be stored. When the RTR bit is clear, the current MB has a data frame to be transmitted. In an Rx MB, it may be considered in matching processes.

See the Message Buffer Code - Rx Buffer table, the Message Buffer Code - Tx Buffer table, and the description of the CANFD\_CTL2.RRS bit for additional details.

If the CANFD module transmits RTR as 1 (recessive) and receives it as 0 (dominant), it is interpreted as an arbitration loss. If RTR is transmitted as 0 (dominant) and received as 1 (recessive), the CANFD treats it as a bit error. If the value received matches the value transmitted, it is considered a successful bit transmission.

NOTE: When configuring CAN FD frames, the RTR bit must be negated.

## Length of Data in Bytes (DLC)

The 4-bit DLC field is the length (in bytes) of the Rx or Tx data, which is located in offset 0x8 through 0xF of the MB space. In reception, DLC is written by the CANFD module, copied from the DLC (Data Length Code) field of the received frame. In transmission, DLC is written by the processor and corresponds to the DLC field value of the frame to be transmitted. When RTR = 1, the frame to be transmitted is a remote frame and does not include the data field, regardless of the DLC field. See the Data Byte Validity table.

## Free-Running Counter Time Stamp (TIME STAMP)

The 16-bit TIME STAMP field is a copy of the free-running timer, captured for Tx and Rx frames when the beginning of the identifier field appears on the CAN bus.

## Local Priority (PRIO)

The 3-bit PRIO field is used only when the CANFD\_CFG.LPRIO\_EN bit is set and only makes sense for Tx mailboxes. The PRIO field is not transmitted. It is appended to the regular ID to define the transmission priority. See the Arbitration Process section.

## Frame Identifier (ID)

In standard frame format, only the 11 most significant bits (28 to 18) are used for frame identification for both receive and transmit, and the 18 least significant bits are ignored. In extended frame format, all bits are used for frame identification for both receive and transmit.

## Data Field (DATA BYTE 0 to 63)

Up to 64 bytes can be used for a data frame, depending on the size of payload selected for the Message Buffers.

For Rx frames, the data is stored as it is received from the CAN bus. DATA BYTE(n) is valid only if n is less than DLC as shown in the Data Byte Validity table.

Table 16-7: Data Byte Validity

|   DLC | Valid Data Bytes   |
|-------|--------------------|
|     0 | None               |
|     1 | Data Byte 0        |

Table 16-7: Data Byte Validity (Continued)

|   DLC | Valid Data Bytes   |
|-------|--------------------|
|     2 | Data Byte 0 to 1   |
|     3 | Data Byte 0 to 2   |
|     4 | Data Byte 0 to 3   |
|     5 | Data Byte 0 to 4   |
|     6 | Data Byte 0 to 5   |
|     7 | Data Byte 0 to 6   |
|     8 | Data Byte 0 to 7   |
|     9 | Data Byte 0 to 11  |
|    10 | Data Byte 0 to 15  |
|    11 | Data Byte 0 to 19  |
|    12 | Data Byte 0 to 23  |
|    13 | Data Byte 0 to 31  |
|    14 | Data Byte 0 to 47  |
|    15 | Data Byte 0 to 63  |

## Message Buffer Memory Map

The CANFD memory buffers are allocated in memory according to the 8-Byte Message Buffers , 16-Byte Message Buffers , 32-Byte Message Buffers , and 64-Byte Message Buffers tables.

Table 16-8: 8-Byte Message Buffers

| Address Offset   | MBDSR = 00 (8-Byte Payload)   |
|------------------|-------------------------------|
| 0x0080           | MB0                           |
| 0x0090           | MB1                           |
| 0x00A0           | MB2                           |
| 0x00B0           | MB3                           |
| 0x00C0           | MB4                           |
| 0x00D0           | MB5                           |
| 0x00E0           | MB6                           |
| 0x00F0           | MB7                           |
| 0x0100           | MB8                           |
| 0x0110           | MB9                           |
| 0x0120           | MB10                          |

Table 16-8: 8-Byte Message Buffers (Continued)

| Address Offset   | MBDSR = 00 (8-Byte Payload)   |
|------------------|-------------------------------|
| 0x0130           | MB11                          |
| 0x0140           | MB12                          |
| 0x0150           | MB13                          |
| 0x0160           | MB14                          |
| 0x0170           | MB15                          |
| 0x0180           | MB16                          |
| 0x0190           | MB17                          |
| 0x01A0           | MB18                          |
| 0x01B0           | MB19                          |
| 0x10C0           | MB20                          |
| 0x10D0           | MB21                          |
| 0x10E0           | MB22                          |
| 0x10F0           | MB23                          |
| 0x0200           | MB24                          |
| 0x0210           | MB25                          |
| 0x0220           | MB26                          |
| 0x0230           | MB27                          |
| 0x0240           | MB28                          |
| 0x0250           | MB29                          |
| 0x0260           | MB30                          |
| 0x0270           | MB31                          |
| 0x0280           | MB32                          |
| 0x0290           | MB33                          |
| 0x02A0           | MB34                          |
| 0x02B0           | MB35                          |
| 0x02C0           | MB36                          |
| 0x02D0           | MB37                          |
| 0x02E0           | MB38                          |
| 0x02F0           | MB39                          |
| 0x0300           | MB40                          |
| 0x0310           | MB41                          |

Table 16-8: 8-Byte Message Buffers (Continued)

| Address Offset   | MBDSR = 00 (8-Byte Payload)   |
|------------------|-------------------------------|
| 0x0320           | MB42                          |
| 0x0330           | MB43                          |
| 0x0340           | MB44                          |
| 0x0350           | MB45                          |
| 0x0360           | MB46                          |
| 0x0370           | MB47                          |
| 0x0380           | MB48                          |
| 0x0390           | MB49                          |
| 0x03A0           | MB50                          |
| 0x03B0           | MB51                          |
| 0x03C0           | MB52                          |
| 0x03D0           | MB53                          |
| 0x03E0           | MB54                          |
| 0x03F0           | MB55                          |
| 0x0400           | MB56                          |
| 0x0410           | MB57                          |
| 0x0420           | MB58                          |
| 0x0430           | MB59                          |
| 0x0440           | MB60                          |
| 0x0450           | MB61                          |
| 0x0460           | MB62                          |
| 0x0470           | MB63                          |

Table 16-9: 16-Byte Message Buffers

| Address Offset   | MBSR = 01 (16-Byte Payload)   |
|------------------|-------------------------------|
| 0x0080           | MB0                           |
| 0x0098           | MB1                           |
| 0x00B0           | MB2                           |
| 0x00C8           | MB3                           |
| 0x00E0           | MB4                           |
| 0x00F8           | MB5                           |
| 0x0100           | MB6                           |

Table 16-9: 16-Byte Message Buffers (Continued)

| Address Offset   | MBSR = 01 (16-Byte Payload)   |
|------------------|-------------------------------|
| 0x0128           | MB7                           |
| 0x0140           | MB8                           |
| 0x0158           | MB9                           |
| 0x0170           | MB10                          |
| 0x0188           | MB11                          |
| 0x01A0           | MB12                          |
| 0x01B8           | MB13                          |
| 0x01D0           | MB14                          |
| 0x01E8           | MB15                          |
| 0x0200           | MB16                          |
| 0x0218           | MB17                          |
| 0x0230           | MB18                          |
| 0x0248           | MB19                          |
| 0x0260           | MB20                          |
| 0x0280           | MB21                          |
| 0x0298           | MB22                          |
| 0x02B0           | MB23                          |
| 0x02C8           | MB24                          |
| 0x02E0           | MB25                          |
| 0x02F8           | MB26                          |
| 0x0310           | MB27                          |
| 0x0328           | MB28                          |
| 0x0340           | MB29                          |
| 0x0358           | MB30                          |
| 0x0370           | MB31                          |
| 0x0388           | MB32                          |
| 0x03A0           | MB33                          |
| 0x03B8           | MB34                          |
| 0x03D0           | MB35                          |
| 0x03E8           | MB36                          |
| 0x0400           | MB37                          |

Table 16-9: 16-Byte Message Buffers (Continued)

| Address Offset   | MBSR = 01 (16-Byte Payload)   |
|------------------|-------------------------------|
| 0x0418           | MB38                          |
| 0x0430           | MB39                          |
| 0x0448           | MB40                          |
| 0x0460           | MB41                          |

Table 16-10: 32-Byte Message Buffers

| Address Offset   | MBDSR = 10 (32-Byte Payload)   |
|------------------|--------------------------------|
| 0x0080           | MB0                            |
| 0x00A8           | MB1                            |
| 0x00D0           | MB2                            |
| 0x00F8           | MB3                            |
| 0x0120           | MB4                            |
| 0x0248           | MB5                            |
| 0x0170           | MB6                            |
| 0x0198           | MB7                            |
| 0x01C0           | MB8                            |
| 0x01E8           | MB9                            |
| 0x0210           | MB10                           |
| 0x0238           | MB11                           |
| 0x0280           | MB12                           |
| 0x02A8           | MB13                           |
| 0x02D0           | MB14                           |
| 0x02F8           | MB15                           |
| 0x0320           | MB16                           |
| 0x0348           | MB17                           |
| 0x0270           | MB18                           |
| 0x0298           | MB19                           |
| 0x03C0           | MB20                           |
| 0x03E8           | MB21                           |
| 0x0410           | MB22                           |
| 0x0438           | MB23                           |

Table 16-11: 64-Byte Message Buffers

| Address Offset   | MBSR = 11 (64-Byte Payload)   |
|------------------|-------------------------------|
| 0x0080           | MB0                           |
| 0x0C8            | MB1                           |
| 0x0110           | MB2                           |
| 0x0158           | MB3                           |
| 0x01A0           | MB4                           |
| 0x01E8           | MB5                           |
| 0x0230           | MB6                           |
| 0x0280           | MB7                           |
| 0x02C8           | MB8                           |
| 0x0310           | MB9                           |
| 0x0258           | MB10                          |
| 0x03A0           | MB11                          |
| 0x03E8           | MB12                          |
| 0x0430           | MB13                          |

## Memory Partition

When CAN FD is enabled, the CANFD RAM can be partitioned in two blocks of 512 bytes. Each block can accommodate a number of message buffers. This depends on the configuration of the size control bit fields in the CANFD\_FD\_CTL register as shown in the RAM Partition table.

Table 16-12: RAM Partition

|   RAM Block | Number of MBs with 8 Bytes *1   | Size Control Bit Field   | Number of MBs of Different Sizes, per Block                                                                                                                            |
|-------------|---------------------------------|--------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|           0 | 0 to 31                         | CANFD_FD_CTL.MBDSIZR0    | MBDSIZR0=00, 32 MBs with 8 bytes payload MBDSIZR0=01, 21 MBs with 16 bytes payload MBDSIZR0=10, 12 MBs with 32 bytes payload, MBDSIZR0=11, 7 MBs with 64 bytes payload |
|           1 | 32 to 63                        | CANFD_FD_CTL.MBDSIZR1    | MBDSIZR1=00, 32 MBs with 8 bytes payload MBDSIZR1=01, 21 MBs with 16 bytes payload MBDSIZR1=10, 12 MBs with 32 bytes payload MBDSIZR1=11, 7 MBs with 64 bytes payload  |

## *1 default range

When payload sizes of 16, 32, or 64 bytes are configured in some or all RAM blocks, the total number of MBs, and the respective number order may differ from the default configuration of 8 bytes. For example, if Block 0 is configured to 8 bytes payload, Block 1 to 16 bytes, the following table indicates how the message buffers are arranged into RAM.

Table 16-13: RAM Partition Example

|   RAM Block | Payload Size                                 |   Number of MBs in the RAM Block | MBRange   |
|-------------|----------------------------------------------|----------------------------------|-----------|
|           0 | CANFD_FD_CTL[MBDSIZR0 ]=00, 8 bytes payload  |                               32 | 0 to 31   |
|           1 | CANFD_FD_CTL[MBDSIZR1 ]=01, 16 bytes payload |                               21 | 32 to 52  |

## Rx FIFO Structure

When the CANFD\_CFG.RFEN bit is set, the memory area from 0x80 to 0xDC (which is normally occupied by MBs 0-5) is used by the Rx FIFO engine.

The region 0x80 to 0x8C contains the output of the FIFO, which must be read by the processor as a message buffer. The FIFO output contains the oldest message that has been received but not yet read. The region 0x90-0xDC is reserved for internal use of the FIFO engine.

An additional memory area, which starts at 0xE0 and may extend up to 0x2DC (normally occupied by MBs 6 to 37) depending on the CANFD\_CTL2.RFFNUM bit field setting, contains the ID filter table (configurable from 8 to 128 table elements), which specifies filtering criteria for accepting frames into the FIFO. Out of reset, the ID filter table flexible memory area defaults to 0xE0 and extends only to 0xFC, which corresponds to MBs 6 to 7 for the CANFD\_CTL2.RFFNUM bit field setting equal to zero.

The Rx FIFO Structure figure shows the Rx FIFO data structure.

Figure 16-3: Rx FIFO Structure

Each ID filter table element occupies an entire 32-bit word and can be compounded by one, two, or four Identifier Acceptance Filters (IDAF) depending on the CANFD\_CFG.IDAM bit field setting.

The ID Filter T able Structure figure shows the three different formats of the ID table elements. All elements of the table must have the same format, as selected by the CANFD\_CFG.IDAM bit field.

<!-- image -->

| FORMAT A   | 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 R T R I D E R RXIDA (STD= 29-19, EXT = 29-1)   |
|------------|------------------------------------------------------------------------------------------------------------------------------------|
| B          | T R I D E R T R I D E RXIDB_0 (STD= 29-19, EXT = 29-16) RXIDB_1 (STD= 13-3, EXT = 13-0)                                            |
| C          | RXIDC_1 (STD/EXT = 23-16) RXIDC_0 (STD/EXT = 31-24) RXIDC_2 (STD/EXT = 15-8) RXIDC_3 (STD/EXT = 7-0)                               |

Figure 16-4: ID Filter Table Structure

The Rx FIFO Filters table shows the number of Rx FIFO filters configured with the CANFD\_CTL2.RFFNUM bit field setting.

| CANFD_CTL2. RFFNUM   |   Number of Rx FIFO Filter Elements | Message Buffers Oc- cupied by Rx FIFO and ID filter table   | Remaining Mailbox- es   | Rx FIFO ID Filter Table Elements Af- fected by Rx Individ- ual Masks   | Rx FIFO ID filter table elements affect- ed by Rx FIFO glob- al mask   |
|----------------------|-------------------------------------|-------------------------------------------------------------|-------------------------|------------------------------------------------------------------------|------------------------------------------------------------------------|
| 0x0                  |                                   8 | MB0 to 7                                                    | MB8 to 63               | Elements 0 to 7                                                        | None                                                                   |
| 0x1                  |                                  16 | MB0 to 9                                                    | MB10 to 63              | Elements 0 to 9                                                        | Elements 10 to 15                                                      |
| 0x2                  |                                  24 | MB0 to 11                                                   | MB12 to 63              | Elements 0 to 11                                                       | Elements 12 to 23                                                      |

Table 16-14: Rx FIFO Filters

<!-- image -->

Table 16-14: Rx FIFO Filters (Continued)

| CANFD_CTL2. RFFNUM   |   Number of Rx FIFO Filter Elements | Message Buffers Oc- cupied by Rx FIFO and ID filter table   | Remaining Mailbox- es   | Rx FIFO ID Filter Table Elements Af- fected by Rx Individ- ual Masks   | Rx FIFO ID filter table elements affect- ed by Rx FIFO glob- al mask   |
|----------------------|-------------------------------------|-------------------------------------------------------------|-------------------------|------------------------------------------------------------------------|------------------------------------------------------------------------|
| 0x3                  |                                  32 | MB0 to 13                                                   | MB14 to 63              | Elements 0 to 13                                                       | Elements 14 to 31                                                      |
| 0x4                  |                                  40 | MB0 to 15                                                   | MB16 to 63              | Elements 0 to 15                                                       | Elements 16 to 39                                                      |
| 0x5                  |                                  48 | MB0 to 17                                                   | MB18 to 63              | Elements 0 to 17                                                       | Elements 18 to 47                                                      |
| 0x6                  |                                  56 | MB0 to 19                                                   | MB20 to 63              | Elements 0 to 19                                                       | Elements 20 to 55                                                      |
| 0x7                  |                                  64 | MB0 to 21                                                   | MB22 to 63              | Elements 0 to 21                                                       | Elements 22 to 63                                                      |
| 0x8                  |                                  72 | MB0 to 23                                                   | MB24 to 63              | Elements 0 to 23                                                       | Elements 24 to 71                                                      |
| 0x9                  |                                  80 | MB0 to 25                                                   | MB26 to 63              | Elements 0 to 25                                                       | Elements 26 to 79                                                      |
| 0xA                  |                                  88 | MB0 to 27                                                   | MB28 to 63              | Elements 0 to 27                                                       | Elements 28 to 87                                                      |
| 0xB                  |                                  96 | MB0 to 29                                                   | MB30 to 63              | Elements 0 to 29                                                       | Elements 30 to 95                                                      |
| 0xC                  |                                 104 | MB0 to 31                                                   | MB32 to 63              | Elements 0 to 31                                                       | Elements 32 to 103                                                     |
| 0xD                  |                                 112 | MB0 to 33                                                   | MB34 to 63              | Elements 0 to 31                                                       | Elements 32 to 111                                                     |
| 0xE                  |                                 120 | MB0 to 35                                                   | MB36 to 63              | Elements 0 to 31                                                       | Elements 32 to 119                                                     |
| 0xF                  |                                 128 | MB0 to 37                                                   | MB38 to 63              | Elements 0 to 31                                                       | Elements 32 to 127                                                     |

## Remote Frame (RTR)

The RTR bit specifies if remote frames are accepted into the FIFO if they match the target ID. If the bit is set, remote frames are accepted and data frames are rejected. If the bit is clear, remote frames are rejected and data frames are accepted.

## Extended Frame (IDE)

The IDE bit specifies whether extended or standard frames are accepted into the FIFO if they match the target ID. If the bit is set, extended frames are accepted and standard frames are rejected. If the bit is clear, extended frames are rejected and standard frames are accepted.

## Rx Frame Identifier Format A (RXIDA)

Specifies the ID for as acceptance criteria for the FIFO. In the standard frame format, only the 11 most significant bits (29 to 19) are used for frame identification. In the extended frame format, all bits are used.

## Rx Frame Identifier Format B (RXIDB\_0, RXIDB\_1)

Specifies an ID to be used as acceptance criteria for the FIFO. In the standard frame format, the 11 most significant bits (a full standard ID) (29 to 19 and 13 to 3) are used for frame identification. In the extended frame format, all 14 bits of the field are compared to the 14 most significant bits of the received ID.

## Rx Frame Identifier Format C (RXIDC\_1, RXIDC\_2, RXIDC\_3)

Specifies an ID to be used as acceptance criteria for the FIFO. In both standard and extended frame formats, all 8 bits of the field are compared to the 8 most significant bits of the received ID.

## Identifier Acceptance Filter Hit Indicator (IDHIT)

The 9-bit IDHIT field indicates which identifier acceptance filter is hit by the received message that is in the output of the Rx FIFO. See the Rx FIFO section for more information.

## CANFD Processes

The following sections describe the how to transmit and receive CAN and CAN FD frames using the mailbox system.

## Transmit Process

To transmit a CAN frame, the processor must prepare a message buffer for transmission by executing the following procedure:

1. Check if the respective interrupt bit is set and clear it.
2. If the MB is active (transmission pending), write the ABORT code (1001) to the CODE field of the control and status (C/S) word to request an abortion of the transmission. Wait for the corresponding IFLAG bit to be asserted by polling the CANFD\_IFLG1 or CANFD\_IFLG2 register or by the interrupt request if enabled in the respective IMASK bit in the CANFD\_IMSK1 or CANFD\_IMSK2 register. Then, read back the CODE field to check if the transmission was aborted or transmitted (see the T ransmission Abort Mechanism section).
3. Write the ID word.
4. Write the data bytes.
5. Write the DLC, Control, and CODE fields of the C/S word to activate the MB. When CANFD\_CFG.FDEN is set, also write the EDL, BRS and ESI bits.

When the MB is activated, it participates in the arbitration process and is eventually transmitted according to its priority. When the DLC value stored in the MB selected for transmission is larger than the respective MB payload size, the CANFD adds the necessary number of bytes with constant 0xCC pattern to complete the expected DLC.

At the end of the successful transmission, the value of the CANFD\_TMR register is written into the TIME STAMP field, the CODE field in the C/S word is updated, both CANFD\_CRC and CANFD\_FD\_CRC registers are updated, a status flag is set in the CANFD\_IFLG1 or CANFD\_IFLG2 register, and an interrupt is generated if allowed by the corresponding CANFD\_IFLAGn bit. The new CODE field after transmission depends on the code that was used to activate the MB. See the Message Buffer Code - Rx Buffer table.

When the Abort feature is enabled ( CANFD\_CFG.ABORTEN is asserted), after the interrupt flag is asserted for a mailbox configured as transmit buffer, the mailbox is blocked. Therefore, the processor is not able to update it until the interrupt flag is disabled by the processor. This means that the processor must clear the corresponding CANFD\_IFLAGn bit before starting to prepare this MB for a new transmission or reception.

## Arbitration Process

The arbitration process scans the mailboxes, searching for the Tx mailbox that holds the message to be sent in the next opportunity. This mailbox is called the arbitration winner.

The scan starts from the lowest number mailbox and runs toward the higher ones. The arbitration process is triggered in the following events:

- From the CRC field of the CAN frame. The start point depends on the CANFD\_CTL2.TXASDLY field value.
- During the error delimiter field of a CAN frame.
- During the overload delimiter field of a CAN frame.
- When the winner is inactivated, and the CAN bus has still not reached the first bit of the intermission field.
- When there is a processor write to the C/S word of a winner MB and the CAN bus has still not reached the first bit of the intermission field.
- When CHI is in idle state and the processor writes to the C/S word of any MB.
- When CANFD module exits the bus off state.
- Upon leaving freeze mode or low power mode.

If the arbitration process does not manage to evaluate all mailboxes before the CAN bus has reached the first bit of the intermission field, the temporary arbitration winner is invalidated and the CANFD module does not compete for the CAN bus in the next opportunity.

The arbitration process selects the winner among the active Tx mailboxes at the end of the scan according to the CANFD\_CTL1.LBUF and CANFD\_CFG.LPRIOEN bit settings.

## Lowest Number Mailbox First

If the CANFD\_CTL1.LBUF bit is enabled, the first (lowest number) active Tx mailbox found is the arbitration winner. The CANFD\_CFG.LPRIOEN bit has no effect when the CANFD\_CTL1.LBUF bit is enabled.

## Highest-Priority Mailbox First

If the CANFD\_CTL1.LBUF bit is disabled, then the arbitration process searches the active Tx mailbox with the highest priority. This mailbox frame with the highest priority has a higher probability to win the arbitration on CAN bus when multiple external nodes compete for the bus at the same time.

The sequence of bits considered for this arbitration is called the arbitration value of the mailbox. The highest-priority Tx mailbox is the one that has the lowest arbitration value among all Tx mailboxes.

If two or more mailboxes have equivalent arbitration values, the mailbox with the lowest number is the arbitration winner.

The composition of the arbitration value depends on the CANFD\_CFG.LPRIOEN bit.

## Local Priority Disabled

When the CANFD\_CFG.LPRIOEN is disabled, local priority is disabled, and the arbitration value is built in the exact sequence of bits transmitted in a CAN frame as shown in the Arbitration Value Priority Disabled table.

Table 16-15: Arbitration Value Priority Disabled

| Format             | Mailbox Arbitration Value (32 bits)   | Mailbox Arbitration Value (32 bits)   | Mailbox Arbitration Value (32 bits)   | Mailbox Arbitration Value (32 bits)   | Mailbox Arbitration Value (32 bits)   |
|--------------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|
| Standard (IDE = 0) | Standard ID (11 bits)                 | RTR (1bit)                            | IDE (1 bit)                           | - (18 bits)                           | - (1 bit)                             |
| Extended (IDE = 1) | Extended ID[28:18] (11 bits)          | SRR (1bit)                            | IDE (1 bit)                           | Extended ID[17:0] (11 bits)           | RTR (1bit)                            |

## Local Priority Enabled

If the CANFD\_CFG.LPRIOEN is enabled, local priority is enabled. In this case, the mailbox PRIO field is included at the very left of the arbitration value as shown in the Arbitration Value Priority Enabled table.

Table 16-16: Arbitration Value Priority Enabled

| Format             | Mailbox Arbitration Value (35 bits)   | Mailbox Arbitration Value (35 bits)   | Mailbox Arbitration Value (35 bits)   | Mailbox Arbitration Value (35 bits)   | Mailbox Arbitration Value (35 bits)   | Mailbox Arbitration Value (35 bits)   |
|--------------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|
| Standard (IDE = 0) | PRIO (3 bits)                         | Standard ID (11 bits)                 | RTR (1bit)                            | IDE (1 bit)                           | - (18 bits)                           | - (1 bit)                             |
| Extended (IDE = 1) | PRIO (3 bits)                         | Extended ID[28:18] (11 bits)          | SRR (1bit)                            | IDE (1 bit)                           | Extended ID[17:0] (11 bits)           | RTR (1bit)                            |

As the PRIO field is the most significant part of the arbitration value, mailboxes with low PRIO values have higher priority than mailboxes with high PRIO values, regardless of the rest of their arbitration values.

NOTE: The PRIO field is not part of the frame on the CAN bus and only affects the internal arbitration process.

## Arbitration Completion

After the arbitration winner is found, its content is copied to a hidden auxiliary MB called Tx Serial Message Buffer (Tx SMB), which has the same structure as a normal MB but is not user accessible. This operation is called moveout and, after it is done, write access to the C/S word of the corresponding MB is blocked (if CANFD\_CFG.ABORTEN is asserted). Write access is restored in the following events:

- After the MB is transmitted and the corresponding IFLAG bit is cleared by the processor.
- The CANFD module enters freeze mode or bus off.
- The CANFD module loses the bus arbitration or there is an error during the transmission.

At the first opportunity window on the CAN bus, the message on the Tx SMB is transmitted according to the CAN protocol rules.

## Arbitration Start and Stop Conditions

The arbitration process is triggered in the following situations:

- During Rx and Tx frames from CAN CRC field to end of frame. The CANFD\_CTL2.TXASDLY value may be changed to optimize the arbitration start point.
- During CAN bus off state from TX\_ERR\_CNT = 124 to 128. The CANFD\_CTL2.TXASDLY value may be changed to optimize the arbitration start point.
- During C/S write by the processor in bus idle mode. The first C/S write starts the arbitration process and a second C/S write during this same arbitration restarts the process. If other C/S writes are performed, the Tx arbitration process is pending. If there is no arbitration winner after the arbitration process has finished, the TX arbitration machine begins a new arbitration process. If there is a pending arbitration and the bus idle state starts, an arbitration process is triggered. In this case, the first and second C/S write in the bus idle state do not restart the arbitration process. It is possible that there is not enough time to finish arbitration in the wait for bus idle state and the next state is idle. In this case, the scan is not interrupted, and it is completed during the bus idle state. During this arbitration, a C/S write does not cause an arbitration restart.
- Arbitration winner deactivation during a valid arbitration window.
- Upon exiting freeze mode (first bit of the wait for bus idle state). If there is a re- synchronization during wait for bus idle state, the arbitration process restarts.

Arbitration process stops in the following situations:

- All mailboxes were scanned.
- A Tx active mailbox is found when the lowest buffer feature is enabled.
- Arbitration winner inactivation or abort during any arbitration process.
- There was not enough time to finish the Tx arbitration process (for example, when a deactivation was performed near the end of frame). In this case, the arbitration process is pending.
- There is an error or overload flag in the bus.
- There is a low power or freeze mode request in the idle state.

Arbitration is considered pending as described below:

- It was not possible to finish the arbitration process in time.
- C/S write during arbitration if write is performed in a MB whose number is lower than the Tx arbitration pointer.
- Any C/S write if there is no Tx arbitration process in progress.
- Rx Match has just updated an Rx code to Tx code
- Entering the bus off state.

C/S write during arbitration has the following effect:

- If the C/S write is performed in the arbitration winner, a new process is restarted immediately.
- If the C/S write is performed in an MB whose number is higher than the Tx arbitration pointer, the ongoing arbitration process will scan this MB as normal.

## Receive Process

To receive CAN frames into a mailbox, the processor must prepare the mailbox for reception by executing the following steps:

1. If the mailbox is active (either Tx or Rx), inactivate the mailbox (See Mailbox Inactivation), preferably with a safe inactivation (See T ransmission Abort Mechanism).
2. Write the ID word.
3. Write the EMPTY code (0b0100) to the CODE field of the C/S word to activate the mailbox. No setup is required for the EDL, BRS, and ESI bits; they are overwritten by the respective bit fields in the received message.

After the MB is activated, it will be able to receive frames that match the programmed filter. At the end of a successful reception, the mailbox is updated by the move-in process (See Move-In) as follows:

1. The received data field (8 bytes at most for classical CAN message format and up to 64 bytes for the CAN FD message format) is stored.
2. The received ID field is stored.
3. The value of the CANFD\_TMR register at the time of the second bit of the frame identifier field is written into the mailbox TIME STAMP field.
4. The received SRR, IDE, RTR, EDL, BRS, ESI, and DLC fields are stored.
5. The CODE field in the C/S word is updated according to the Message Buffer Code - Rx Buffer and Message Buffer Code - Tx Buffer tables.
6. A status flag is set in the CANFD\_IFLG1 or CANFD\_IFLG2 register and an interrupt is generated if allowed by the corresponding CANFD\_IMSK1 or CANFD\_IMSK2 register bit.

The recommended procedure for the processor to service (read) the frame received in a mailbox is:

The processor polls for frame reception by checking the status flag bit for the specific mailbox in the respective CANFD\_IFLAGn register and not by polling the CODE field of that mailbox. Polling the CODE field does not work because, once a frame is received and the processor services the mailbox (by reading the C/S word followed by unlocking the mailbox), the CODE field does not return to EMPTY. It remains FULL, as explained in the Message Buffer Code - Rx Buffer table. If the processor tries to work around this behavior by writing to the C/S word to force an EMPTY code after reading the mailbox without a prior safe inactivation, a newly received frame matching the filter of that mailbox may be lost.

NOTE: Never poll by reading the C/S word directly from the mailboxes. Always read the CANFD\_IFLG1 or CANFD\_IFLG2 register.

The identifier field of the receive frame is always stored in the matching mailbox. Therefore, the contents of the ID field in a mailbox may change if the match was due to masking.

When the CANFD\_CFG.SRXDIS bit is enabled, the CANFD module does not store frames transmitted by itself in any MB, even if it contains a matching Rx mailbox, and no interrupt flag or interrupt signal is generated. Otherwise, when the CANFD\_CFG.SRXDIS bit is disabled, the CANFD module can receive frames transmitted by itself if there is a matching Rx mailbox.

To be able to receive CAN frames through the Rx FIFO, the processor must enable and configure the Rx FIFO during freeze mode (See Rx FIFO). Upon receiving the frames available in Rx FIFO interrupt (see the CANFD\_IFLG1.MB05 bit description), the processor services the received frame using the following procedure:

1. Read the C/S word (optional: needed only if a mask was used for the IDE and RTR bits).
2. Read the ID field (optional: needed only if a mask was used).
3. Read the Data field.
4. Read the CANFD\_RX\_FIFO register (optional).
5. Clear the frames available in Rx FIFO interrupt by writing a one to the CANFD\_IFLG1.MB05 bit to release the MB and allow the processor to read the next Rx FIFO entry.

When the CANFD\_CFG.DMAEN bit is enabled, upon receiving a frame in the Rx FIFO, the CANFD\_IFLG1.MB05 bit generates a DMA request and does not generate a processor interrupt (See Rx FIFO Under DMA Operation). The CANFD\_IMSK1 bits in the Rx FIFO region are not used.

The DMA controller must service the received frame using the following procedure:

1. Read the C/S word (read 0x80 address, optional).
2. Read the ID field (read 0x84 address, optional).
3. Read all data bytes (start read at 0x88 address, optional).
4. Read the last data bytes (read 0x8C address is mandatory).

## Matching Process

The matching process scans the MB memory looking for Rx MBs programmed with the same ID as the one received from the CAN bus. If the FIFO is enabled, the priority of scanning can be selected between mailboxes and FIFO filters. The matching starts from the lowest number MB toward the higher ones. When no match is found within the first structure, then the other is scanned subsequently. When the FIFO is full, the matching algorithm always looks for a matching MB outside the FIFO region.

As the frame is being received, it is stored in a hidden auxiliary MB called Rx Serial Message Buffer (Rx SMB).

The matching process start point depends on the following conditions:

- When the received frame is a remote frame, the start point is the CRC field of the frame.

- When the received frame is a data frame with DLC field equal to zero, the start point is the CRC field of the frame.
- When the received frame is a data frame with DLC field different than zero, the start point is the DATA field of the frame.

When a matching ID is found in the FIFO table or in one of the mailboxes, the contents of the Rx SMB are transferred to the FIFO or to the matched mailbox by the move-in process. When any CAN protocol error is detected, then no match results are transferred to the FIFO or to the matched mailbox at the end of reception.

The matching process scans all matching elements of both Rx FIFO (if enabled) and the active Rx mailboxes (CODE is EMPTY, FULL, OVERRUN, or RANSWER) in search of a successful comparison with the matching elements of the Rx SMB that is receiving the frame on the CAN bus. The Rx SMB has the same structure as a mailbox. The reception structures (Rx FIFO or mailboxes) associated with the matching elements that had a successful comparison are the matched structures. The matching winner is selected at the end of the scan among those matched structures and depends on conditions described below.

Table 16-17: Matching Architecture

| Structure   | SMB.RTR   | CTRL2.RSS   | CTRL2.EA- CEN   | MB.IDE   | MB.RTR    | MB.ID *1   | MB.CODE   |
|-------------|-----------|-------------|-----------------|----------|-----------|------------|-----------|
| Mailbox     | 0         | N/A         | 0               | cmp *2   | no_cmp *3 | cmp_msk *4 |           |
| Mailbox     | 0         | N/A         | 1               | cmp_msk  | cmp_msk   | cmp_msk    |           |
| Mailbox     | 1         | 0           | N/A             | cmp      | no_cmp    | cmp        |           |
| Mailbox     | 1         | 1           | 0               | cmp      | no_cmp    | cmp_msk    |           |
| Mailbox     | 1         | 1           | 1               | cmp_msk  | cmp_msk   | cmp_msk    |           |
| FIFO *5     | N/A       | N/A         | N/A             | cmp_msk  | cmp_msk   | cmp_msk    |           |

- *1 1. For mailbox structure, If SMB.IDE is asserted, the ID is 29 bits (ID Standard + ID Extended). If SMB.IDE is negated, the ID is only 11 bits (ID Standard). For FIFO structure, the ID depends on CANFD\_CFG.IDAM.
- *2 cmp: Compares the Rx SMB contents with the MB contents regardless the masks.
- *3 no\_cmp: The Rx SMB contents are not compared with the MB contents.
- *4 cmp\_msk: Compares the Rx SMB contents with MB contents taking into account the masks.
- *5 SMB.IDE and SMB.RTR. are not considered when CANFD\_CFG.IDAM is format C.

A reception structure is free-to-receive when any of the following conditions is satisfied:

- The CODE field of the mailbox is EMPTY.
- The CODE field of the mailbox is either FULL or OVERRUN and it has already been serviced (the C/S word was read by the processor and unlocked as described in the Mailbox Lock Mechanism section).
- The CODE field of the mailbox is either FULL or OVERRUN and an inactivation is performed. See the Mailbox Inactivation section.
- The Rx FIFO is not full.

The scan order for mailboxes and the Rx FIFO is from the matching element with lowest number to the higher ones.

The matching winner search for mailboxes is affected by the CANFD\_CFG.IRMQEN bit. If the CANFD\_CFG.IRMQEN bit is disabled, the matching winner is the first matched mailbox regardless if it is free-toreceive or not. If the CANFD\_CFG.IRMQEN bit is enabled, the matching winner is selected according to the priority below:

1. The first free-to-receive matched mailbox.
2. The last non free-to-receive matched mailbox.

It is possible to select the priority of scan between mailboxes and the Rx FIFO by using the CANFD\_CTL2.MRPRIO bit.

## If the selected priority is Rx FIFO first:

- If the Rx FIFO is a matched structure and is free-to-receive, then the Rx FIFO is the matching winner regardless of the scan for mailboxes.
- Otherwise (the Rx FIFO is not a matched structure or is not free-to-receive), then the matching winner is searched for among mailboxes as described above.

## If the selected priority is mailboxes first:

- If a free-to-receive matched mailbox is found, it is the matching winner regardless of the scan for the Rx FIFO.
- If no matched mailbox is found, then the matching winner is searched for in the scan for the Rx FIFO.

If both conditions above are not satisfied and a non free-to-receive matched mailbox is found, then the matching winner determination is conditioned by the CANFD\_CFG.IRMQEN bit:

- If the CANFD\_CFG.IRMQEN bit is disabled, the matching winner is the first matched mailbox.
- If the CANFD\_CFG.IRMQEN bit is enabled, the matching winner is the Rx FIFO if it is a free-to- receive matched structure; otherwise, the matching winner is the last non free-to- receive matched mailbox.

See the Matching Possibilities and Resulting Reception Structures table for a summary of matching possibilities.

Table 16-18: Matching Possibilities and Resulting Reception Structures

| RFEN                                       | IRMQEN                                     | MRPRIO                                     | Matched inMB *1                            | Matched in FIFO *2                         | Reception Struc- ture                      | Description                                |
|--------------------------------------------|--------------------------------------------|--------------------------------------------|--------------------------------------------|--------------------------------------------|--------------------------------------------|--------------------------------------------|
| No FIFO, only MB, match is always MBfirst: | No FIFO, only MB, match is always MBfirst: | No FIFO, only MB, match is always MBfirst: | No FIFO, only MB, match is always MBfirst: | No FIFO, only MB, match is always MBfirst: | No FIFO, only MB, match is always MBfirst: | No FIFO, only MB, match is always MBfirst: |
| 0                                          | 0                                          | X *3                                       | None                                       | - *4                                       | None                                       | Frame lost by no match                     |
| 0                                          | 0                                          | X                                          | Free                                       | -                                          | FirstMB                                    |                                            |
| 0                                          | 1                                          | X                                          | None                                       | -                                          | None                                       | Frame lost by no match                     |

Table 16-18: Matching Possibilities and Resulting Reception Structures (Continued)

| RFEN                                                        | IRMQEN                                                      | MRPRIO                                                      | Matched inMB *1                                             | Matched in FIFO *2                                          | Reception Struc- ture                                       | Description                                                 |
|-------------------------------------------------------------|-------------------------------------------------------------|-------------------------------------------------------------|-------------------------------------------------------------|-------------------------------------------------------------|-------------------------------------------------------------|-------------------------------------------------------------|
| 0                                                           | 1                                                           | X                                                           | Free                                                        | -                                                           | FirstMB                                                     |                                                             |
| 0                                                           | 1                                                           | X                                                           | Not Free                                                    | -                                                           | LastMB                                                      | Overrun                                                     |
| FIFO enabled, no match in FIFO (as if FIFO does not exist): | FIFO enabled, no match in FIFO (as if FIFO does not exist): | FIFO enabled, no match in FIFO (as if FIFO does not exist): | FIFO enabled, no match in FIFO (as if FIFO does not exist): | FIFO enabled, no match in FIFO (as if FIFO does not exist): | FIFO enabled, no match in FIFO (as if FIFO does not exist): | FIFO enabled, no match in FIFO (as if FIFO does not exist): |
| 1                                                           | 0                                                           | X                                                           | None                                                        | None                                                        | None                                                        | Frame lost by no match                                      |
| 1                                                           | 0                                                           | X                                                           | Free                                                        | None                                                        | FirstMB                                                     |                                                             |
| 1                                                           | 1                                                           | X                                                           | None                                                        | None                                                        | None                                                        | Frame lost by no match                                      |
| 1                                                           | 1                                                           | X                                                           | Free                                                        | None                                                        | FirstMB                                                     |                                                             |
| 1                                                           | 1                                                           | X                                                           | Not Free                                                    | None                                                        | LastMB                                                      | Overrun                                                     |
| FIFO enabled, queue disabled:                               | FIFO enabled, queue disabled:                               | FIFO enabled, queue disabled:                               | FIFO enabled, queue disabled:                               | FIFO enabled, queue disabled:                               | FIFO enabled, queue disabled:                               | FIFO enabled, queue disabled:                               |
| 1                                                           | 0                                                           | 0                                                           | X                                                           | NotFull                                                     | FIFO                                                        |                                                             |
| 1                                                           | 0                                                           | 0                                                           | None                                                        | Full                                                        | None                                                        | Frame lost by FIFO full (FIFO overflow)                     |
| 1                                                           | 0                                                           | 0                                                           | Free                                                        | Full                                                        | FirstMB                                                     |                                                             |
| 1                                                           | 0                                                           | 0                                                           | Not Free                                                    | Full                                                        | FirstMB                                                     |                                                             |
| 1                                                           | 0                                                           | 1                                                           | None                                                        | NotFull                                                     | FIFO                                                        |                                                             |
| 1                                                           | 0                                                           | 1                                                           | None                                                        | Full                                                        | None                                                        | Frame lost by FIFO full (FIFO overflow)                     |
| 1                                                           | 0                                                           | 1                                                           | Free                                                        | X                                                           | FirstMB                                                     |                                                             |
| 1                                                           | 0                                                           | 1                                                           | Not Free                                                    | X                                                           | FirstMB                                                     | Overrun                                                     |
| FIFO enabled, queue enabled:                                | FIFO enabled, queue enabled:                                | FIFO enabled, queue enabled:                                | FIFO enabled, queue enabled:                                | FIFO enabled, queue enabled:                                | FIFO enabled, queue enabled:                                | FIFO enabled, queue enabled:                                |
| 1                                                           | 1                                                           | 0                                                           | X                                                           | NotFull                                                     | FIFO                                                        |                                                             |
| 1                                                           | 1                                                           | 0                                                           | None                                                        | Full                                                        | None                                                        | Frame lost by FIFO full (FIFO overflow)                     |
| 1                                                           | 1                                                           | 0                                                           | Free                                                        | Full                                                        | FirstMB                                                     |                                                             |
| 1                                                           | 1                                                           | 0                                                           | Not Free                                                    | Full                                                        | LastMB                                                      | Overrun                                                     |
| 1                                                           | 1                                                           | 1                                                           | None                                                        | NotFull                                                     | FIFO                                                        |                                                             |
| 1                                                           | 1                                                           | 1                                                           | Free                                                        | X                                                           | FirstMB                                                     |                                                             |
| 1                                                           | 1                                                           | 1                                                           | Not Free                                                    | NotFull                                                     | FIFO                                                        |                                                             |

Table 16-18: Matching Possibilities and Resulting Reception Structures (Continued)

|   RFEN |   IRMQEN |   MRPRIO | Matched inMB *1   | Matched in FIFO *2   | Reception Struc- ture   | Description   |
|--------|----------|----------|-------------------|----------------------|-------------------------|---------------|
|      1 |        1 |        1 | Not Free          | Full                 | LastMB                  | Overrun       |

- *1 In the Matched in MB column, the term "None" means the frame has not matched an MB (free-to-receive or non-free-to receive). The term "Free" means the frame matched at least one MB free-to-receive regardless of whether it has matched MBs non-free-toreceive.
- *2 In the Matched in FIFO column, the term "None" means the frame has not matched any filter for the Rx FIFO. It is as if the Rx FIFO does not exist (CANFD\_CTL2.RFEN = 0). The term "NotFull" means that the frame has matched a FIFO filter and has empty slots to receive it. The term "Full" means that the frame has matched a FIFO filter, but the FIFO couldn't store it because the FIFO has no empty slots to receive it.
- *3 The X denotes a don't-care condition.
- *4 The - denotes a forbidden condition.

When a non-safe mailbox inactivation (See Mailbox Inactivation) occurs during the matching process and the mailbox inactivated is the temporary matching winner, then the temporary matching winner is invalidated. The matching elements scan is not stopped nor restarted; it continues normally. The consequence is that the current matching process works as if the matching elements compared before the inactivation did not exist, therefore a message may be lost.

Suppose, for example, that the Rx FIFO is disabled, the CANFD\_CFG.IRMQEN is enabled, there are two MBs with the same ID, and the CANFD starts receiving messages with that ID. If the two MBs are the second and the fifth in the array, then when the first message arrives, the matching algorithm finds the first match in MB number 2. The code of this MB is EMPTY, so the message is stored there. When the second message arrives, the matching algorithm finds MB number 2 again, but it is not 'free-to-receive,' so it keeps looking, finds MB number 5 and stores the message there. If yet another message with the same ID arrives, the matching algorithm finds out that there are no matching MBs that are 'free-to-receive,' so it decides to overwrite the last matched MB, which is number 5. In doing so, it sets the CODE field of the MB to indicate OVERRUN.

The ability to match the same ID in more than one MB can be exploited to implement a reception queue (in addition to the full-featured FIFO) to allow more time for the processor to service the MBs. By programming more than one MB with the same ID, received messages are queued into the MBs. The CPU can examine the TIME STAMP field of the MBs to determine the order in which the messages arrived.

Matching to a range of IDs is possible by using ID acceptance masks. The CANFD module supports individual masking per MB. During the matching algorithm, if a mask bit is asserted, then the corresponding ID bit is compared. If the mask bit is negated, the corresponding ID bit is a 'don't care.' The CANFD\_IMSK1 and CANFD\_IMSK2 registers are implemented in RAM, so they are not initialized out of reset. Also, they can only be programmed while the CANFD module is in freeze mode; otherwise, they are blocked by hardware.

The CANFD also supports an alternate masking scheme with only four mask registers ( CANFD\_RX\_FIFO\_GMSK , CANFD\_RX\_MB\_GMSK , CANFD\_RX\_14\_MSK , and CANFD\_RX\_15\_MSK ) for backward compatibility with legacy applications. This alternate masking scheme is enabled when CANFD\_CFG.IRMQEN is enabled.

## Receive Process Pretending Network Mode

Pretended networking mode adds specific wake up functionality in low power modes (doze mode). When pretended network (PN) mode is enabled by asserting the CANFD\_CFG.PNETEN bit, the CANFD module continues processing Rx CAN messages under low power mode, able to detect specific wake up messages by filtering them against ID and payload target values using preselected matching criteria. Wake up functionality is not available for messages in CAN FD format. While in pretended networking mode, CAN FD format messages are ignored.

PN registers are in the 0x0B00-0x0B7C address range and can only be written only in freeze mode. These registers are used for writing PN configuration (both control and target values) prior entering into pretended networking mode, and for reading wake up flags and the received message ID and data when returning to normal mode after waking. The processor waits for the CANFD\_CFG.LPMACK bit to be disabled before performing any access to CANFD PN registers.

PN control registers are CANFD\_PN\_CTL1 and CANFD\_PN\_CTL2 . The control bit fields that configure the filtering criteria are:

- CANFD\_PN\_CTL1.PLFSEL : Payload filtering selection.
- CANFD\_PN\_CTL1.IDFSEL : ID filtering selection.
- CANFD\_PN\_CTL1.FCSEL : Filtering combination selection.

## PN target values are:

- CANFD\_FLTR\_ID1.IDE : IDE target value used to filter the incoming message by its format (standard or extended).
- CANFD\_FLTR\_ID1.RTR : RTR target value used to filter the incoming message by its type (data or remote frame).
- CANFD\_FLTR\_DLC.HI and CANFD\_FLTR\_DLC.LO : Target DLC range used to filter the size of payload part of an incoming message.
- CANFD\_FLTR\_ID1.VALUE : ID target value used to filter the incoming message ID (equal to, smaller than or equal, greater than or equal, or the lower limit value in an ID range).
- CANFD\_FLTR\_ID2\_IDMSK : ID target value used as the upper limit in an ID range.
- PL1: Payload target value used to filter the incoming message payload (equal to, smaller than or equal, greater than or equal, or the lower limit value in a payload range).
- PL2: Payload target value used as the upper limit in a payload range.

The IDE, RTR, ID, and payload filters have their respective masks. These masks determine which bits are considered in equality comparisons (1 in certain mask positions) and which ones are don't care (0 in other mask positions). ID and payload masks are used only for exact ID and/or exact payload comparisons.

The ID of Rx incoming messages can be filtered based on the following criteria:

- A match with the exact ID value by detecting the equality between the ID field of the incoming message and the content of target CANFD\_FLTR\_ID1 register. The ID mask is used.
- A match with the maximum range of ID; in other words, any message with ID value smaller than or equal to the content of target CANFD\_FLTR\_ID1 1 register is accepted. The ID mask is not used.
- A match with the minimum range of ID; in other words, any message with ID value greater than or equal to the content of target CANFD\_FLTR\_ID1 register is accepted. The ID mask is not used.
- A match inside a range of IDs; in other words, any message with an ID value that is greater than or equal to the content of target CANFD\_FLTR\_ID1 register and smaller than or equal to the content of target CANFD\_FLTR\_ID2\_IDMSK register is accepted. The ID mask is not used.

The above criteria for ID filtering must be coherent with CANFD\_FLTR\_ID1.IDE and CANFD\_FLTR\_ID1.RTR target values. Only Rx frames that match the respective IDE and RTR bits to the contents of FLT\_IDE and FLT\_RTR bit fields are compared. When range of IDs is selected ( CANFD\_PN\_CTL1.IDFSEL = 11), both the CANFD\_FLTR\_ID1.VALUE field and the CANFD\_FLTR\_ID2\_IDMSK register are referred to the same CANFD\_FLTR\_ID1.IDE and CANFD\_FLTR\_ID1.RTR bits.

The ID mask is applied only to the exact ID comparison filtering option ( CANFD\_PN\_CTL1.IDFSEL = 00) to determine which bits are considered in the comparison. For the exact match option, the mask can select any bit within the ID field. For maximum range, minimum range and inside range comparisons, the ID mask is not considered.

The IDE and RTR masks are applied in both exact and range ID comparison filtering options to determine which bits are considered in comparison.

Similarly to the ID criteria, 64-bit data or payloads (PL) of Rx incoming messages can be filtered based on the following criteria:

- A match with the exact payload value by detecting the equality between the payload field of the incoming message and the content of PL1 register. The payload mask is used.
- A match with the maximum range of payload-in other words, any message with payload value smaller than or equal to the content of PL1 register is accepted. The payload mask is not used.
- A match with the minimum range of payload-in other words, any message with payload value greater than or equal to the content of PL1 register is accepted. The payload mask is not used.
- A match inside a range of payloads-in other words, any message with a payload value that is greater than or equal to the content of PL1 register and smaller than or equal to the content of PL2 register is accepted. The payload mask is not used.

The above criteria for payload filtering must be coherent with CANFD\_FLTR\_DLC.HI and CANFD\_FLTR\_DLC.LO limit values. The payload of a Rx incoming message is filtered in accordance with the selected criteria only when the DLC value of the Rx incoming message is inside a DLC range:

- Greater than or equal to the CANFD\_FLTR\_DLC.LO (lower limit) and

- Lower than or equal to the CANFD\_FLTR\_DLC.HI (upper limit)

Conversely, a DLC value out of the specified range results in mismatch. By making CANFD\_FLTR\_DLC.LO = CANFD\_FLTR\_DLC.HI , only payloads of specified quantity of bytes will be filtered. DLC is not maskable.

When the inside range of payloads option is selected ( CANFD\_PN\_CTL1.PLFSEL = 11), both PL1 and PL2 are considered with the 8-byte data length. All the data bytes excluded by the DLC of the received message are considered with value zero.

Payload mask is only used in the exact match option ( CANFD\_PN\_CTL1.PLFSEL = 00) to select which bits or bytes in the 8-byte data field of both Rx incoming message and the contents of PL1 register are selected for matching. Mask length must be in accordance with the expected range of DLC values. For maximum range, minimum range, and inside range comparisons, the payload mask is not considered.

When a remote frame is received by the CANFD module and the CANFD\_PN\_CTL1.FCSEL bit is configured to select the payload comparison, the payload filtering is not considered and the comparison results in a mismatch.

Rx incoming messages can also be filtered based upon the quantity and rate of message reception, specifically:

- Several messages that match the filtering criteria for ID or payload a predefined quantity of times. This quantity can be configured in the 1 to 255 range. See the CANFD\_CTL1 register.
- No message matching the filtering criteria for ID or payload up to a timeout trigger.

That is, non-reception of a matching message for a defined quantity of time. See the CANFD\_CTL2 register.

The CANFD module can generate a wakeup timeout event from an internal timer with associated comparator circuitry capable of generating a timeout flag when the counting reaches the predefined timeout value, as specified in the CANFD\_PN\_CTL2.MATCHTO bit field.

The above filtering criteria can be used together as follows:

- Message ID filtering only.
- Message ID filtering and payload filtering.
- Message ID filtering only occurring N times.
- Message ID filtering and payload filtering occurring N times.

The timeout counter runs concurrently with the reception filtering process. Both engines, timeout counter and message filtering, are independent. If an incoming message matches the selected filter criteria, the timeout counter keeps counting until the processor wakes up.

Conversely, if the timeout counter reaches the target value, then the message filtering process continue to filter incoming messages until the processor wakes up. The CANFD\_WUM.MCNT field reports the number of matched messages that occurred under pretended networking mode up to the moment the processor wakes up.

Under pretended networking mode, a wakeup event that occurs sets the respective wake up flag (see the CANFD\_WUM register.

- In case of a successful match with the selected filtering criteria, the CANFD\_WUM.WUMF bit field.

- In case of a timeout trigger, the CANFD\_WUM.WTOF bit field.

Either of these flags generates an interrupt to the processor if the respective mask bit is asserted ( CANFD\_PN\_CTL1.WUMFMSK or CANFD\_PN\_CTL1.WTOFMSK ).

There are four wake up message buffers (WMBs) for storing incoming messages in pretended networking mode. Up to four messages can be stored per the CANFD\_WMB[n]\_ID register. When the CANFD\_PN\_CTL1.MATCHCNT value is one, just one message is received if matching the filtering criteria, and this message is stored in the CANFD\_WMB0 register. If the CANFD\_PN\_CTL1.MATCHCNT value is between two and four, CANFD\_WMB1, CANFD\_WMB2, and CANFD\_WMB3 are used to store the second, third, and fourth matching messages, respectively. If CANFD\_PN\_CTL1.MATCHCNT is greater than four, the last four matching messages are stored in the WMBs; respecting the WMB index to indicate the arrival order, the latest is stored in CANFD\_WMB3. Only the valid data bytes of the incoming match message is stored in data field of WMBs. The non-valid data bytes are read as zero. In case the of DLC = 0 and RTR = 1, the data field is filled with zero. In any of the above cases, the wake up interrupt is generated just when the filtering criteria is completed and the CANFD\_PN\_CTL1.WUMFMSK bit is set.

When a non-match wakeup event occurs (timeout or external) and the CANFD\_WUM.MCNT value is equal or greater than four, the message stored in WMB0 does not have a valid content. The CANFD\_WMB0 is used as the buffer for the current message on the CAN bus. Messages received during pretended networking mode do not have timestamps and respective field in the WMB structure must be ignored.

Under low-power mode (doze or stop), all processes are shut down except for the PN functionality in the PE submodule, which is kept clocked by the oscillator clock. The CANFD continues to receive Rx incoming messages and just compares them against the predefined target values and according to the selected filtering criteria. The matching, arbitration, move-in, and move-out processes, normally available in normal mode, are not performed under pretended networking mode.

Under pretended networking, the CANFD reacts to messages on the CAN bus in the same manner as in normal mode (generates acknowledge bits, detects and counts errors, etc.).

## Move Process

There are two types of move process: move-in and move-out.

## Move-In

The move-in process is the copy of a message received by an Rx SMB to an Rx mailbox or FIFO that has matched it. If the move destination is the Rx FIFO, attributes of the message are also copied to the CANFD\_RX\_FIFO register. Each Rx SMB has its own move-in process, but only one is performed at a given time as described below. The move-in starts only when the message held by the Rx SMB has a corresponding matching winner (See Matching Process) and all of the following conditions are true:

- The CAN bus has reached or let past either of the following:
- The second bit of the Intermission field next to the frame that carried the message that is in the Rx SMB
- The first bit of an overload frame next to the frame that carried the message that is in the Rx SMB

- There is no ongoing matching process.
- The destination mailbox is not locked by the processor.
- There is no ongoing move-in process from another Rx SMB. If more than one move-in processes are to be started at the same time, both are performed and the newest substitutes the oldest.

The term pending move-in is used throughout the documentation and stands for a move-to- be that still does not satisfy all the previously mentioned conditions.

The move-in is canceled and the Rx SMB can receive another message when any of the following conditions is satisfied:

- The destination mailbox is inactivated after the CAN bus has reached the first bit of intermission field next to the frame that carried the message and its matching process has finished.
- There is a previous pending move-in to the same destination mailbox.
- The Rx SMB is receiving a frame transmitted by the CANFD itself and the self-reception is disabled ( CANFD\_CFG.SRXDIS bit is enabled)
- Any CAN protocol error is detected.

Note that the pending move-in is not canceled if the CANFD module enters freeze or low-power mode. The pending move-in stays on hold waiting for exit from Freeze or low-power mode and to be unlocked. If an MB is unlocked during freeze mode, the move-in happens immediately.

The move-in process is the execution by the CANFD module of the following steps:

1. Push CANFD\_RX\_FIFO.IDHIT into the CANFD\_RX\_FIFO FIFO if the message is destined to the Rx FIFO.
2. Read all data words from the Rx SMB according to the selected payload size for the Rx storage element.
3. Write all data words to the Rx mailbox according to the selected payload size for the Rx storage element. If the data size of the storage element is smaller than the original payload size described in the message DLC field, the payload is truncated and the high order bytes that do not fit the destination size are lost.
4. Read the C/S and ID words from the Rx SMB.
5. Write C/S and ID words to the Rx mailbox and update the CODE field.

The move-in process is not atomic, in such a way that it is immediately canceled by the inactivation of the destination mailbox (See Mailbox Inactivation). In this case, the mailbox may be left partially updated, thus incoherent. The exception is if the move-in destination is an Rx FIFO message buffer, in which case the process cannot be canceled.

The BUSY bit (least significant bit of the CODE field) of the destination message buffer is asserted while the movein is being performed to alert the processor that the message buffer content is temporarily incoherent.

## Move-Out

The move-out process is the copy of the content from a Tx mailbox to the Tx SMB when a message for transmission is available (See Arbitration Process). The move-out occurs in the following conditions:

- The first bit of intermission field.
- During the bus off state when TX error counter is in the 124 to 128 range.
- During bus idle state.
- During the wait for bus idle state.

The move-out process is not atomic. Only the processor has priority to access the memory concurrently out of bus idle state. In bus idle, the move-out has the lowest priority to the concurrent memory accesses.

## Data Coherence

In order to maintain data coherency and proper operation of the CANFD module, the processor must obey the rules described in the T ransmit Process and Receive Process sections.

## Transmission Abort Mechanism

The abort mechanism provides a safe way to request the abortion of a pending transmission. A feedback mechanism is provided to inform the processor if the transmission was aborted or if the frame could not be aborted and was transmitted instead.

Two primary conditions must be fulfilled in order to abort a transmission:

- The CANFD\_CFG.ABORTEN bit is enabled.
- The first processor action must be the writing of abort code (1001) into the CODE field of the C/S word.

Active MBs configured for transmission must be aborted first before they can be updated. If the abort code is written to a mailbox that is currently being transmitted or to a mailbox that was already loaded into the Tx SMB for transmission, the write operation is blocked, and the transmission is not disturbed. However, the abort request is captured and kept pending until one of the following conditions is satisfied:

- The CANFD module loses the bus arbitration.
- There is an error during the transmission.
- The CANFD module is put into freeze mode.
- The CANFD enters the bus off state.
- There is an overload frame.

When none of the conditions above are reached, the MB is transmitted correctly, the interrupt flag is set in the respective CANFD\_IFLG1 or CANFD\_IFLG2 register, and an interrupt to the processor is generated (if enabled). The abort request is automatically cleared when the interrupt flag is set. On the other hand, when one of the above conditions is reached, the frame is not transmitted; therefore, the abort code is written into the CODE field, the

interrupt flag is set in the respective CANFD\_IFLG1 or CANFD\_IFLG2 register, and an interrupt is (optionally) generated.

When the processor writes the abort code before the transmission begins internally, then the write operation is not blocked; therefore, the MB is updated, and the interrupt flag is set. The processor just needs to read the abort code to make sure the active MB was safely inactivated. Although the CANFD\_CFG.ABORTEN bit is asserted and the processor wrote the abort code, in this case the MB is inactivated and not aborted, because the transmission did not start yet. One mailbox is only aborted when the abort request is captured and kept pending until one of the previous conditions are satisfied.

The abort procedure can be summarized as follows:

- The processor checks the corresponding IFLAG and clears it, if asserted.
- The processor writes 1001 into the CODE field of the C/S word.
- The processor waits for the corresponding IFLAG indicating that the frame was either transmitted or aborted.
- The processor reads the CODE field to check if the frame was either transmitted (CODE = 1000) or aborted (CODE = 1001).
- It is necessary to clear the corresponding IFLAG to allow the MB to be reconfigured.
- It is necessary to reconfigure the EDL, BRS, and ESI fields of the aborted MB before transmitting it again.

## Mailbox Inactivation

Inactivation is a mechanism provided to protect the mailbox against updates by the CANFD internal processes, thus allowing the processor to rely on mailbox data coherence after having updated it, even in normal mode.

Inactivation of transmission mailboxes must be performed only when the CANFD\_CFG.ABORTEN bit is deasserted.

If a mailbox is inactivated, it participates in neither the arbitration process nor the matching process until it is reactivated.

To inactivate a mailbox, the processor must update its CODE field to INACTIVE (either 0000 or 1000).

Because the processor is not able to synchronize the CODE field update with the CANFD internal processes, an inactivation has the following consequences:

- A frame on the bus that matches the filtering of the inactivated Rx mailbox may be lost without notice, even if there are other mailboxes with the same filter.
- A frame containing the message within the inactivated Tx mailbox may be transmitted without setting the respective IFLAG.

To perform a safe inactivation and avoid the above consequences for Tx mailboxes, the processor must use the transmission abort mechanism .

The inactivation automatically unlocks the mailbox (See Mailbox Lock Mechanism).

- NOTE: Message buffers that are part of the Rx FIFO cannot be inactivated. There is no write protection on the FIFO region. The processor must maintain data coherency in the FIFO region when the CANFD\_CFG.RFEN bit is enabled.

## Mailbox Lock Mechanism

Other than mailbox inactivation, the CANFD module has another data coherence mechanism for the receive process. When the processor reads the C/S word of an Rx MB with codes FULL or OVERRUN, the CANFD assumes that the processor wants to read the whole MB in an atomic operation and therefore it sets an internal lock flag for that MB. The lock is released when the processor reads the CANFD\_TIMING register (global unlock operation), or when it reads the C/S word of another MB regardless of its code. A processor write into the C/S word also unlocks the MB, but this procedure is not recommended for normal unlock use because it cancels a pending move and potentially may lose a received message. The MB locking prevents a new frame from being written into the MB while the processor is reading it.

- NOTE: The locking mechanism applies only to Rx MBs that are not part of the FIFO and have a code other than INACTIVE (0000) or EMPTY (0100). Tx MBs can not be locked. When the CANFD\_CFG.IRMQEN bit is disabled, reading the C/S word locked the MB even if it was EMPTY.

When the FIFO is disabled and the second and the fifth MBs of the array are programmed with the same ID, and the CANFD module has already received and stored messages into these two MBs. If the processor then attempts to read MB number five and at the same time another message with the same ID is arriving, then when the processor reads the C/S word of MB number five, this MB is locked. The new message arrives, and the matching algorithm finds out that there are no 'free-to-receive' MBs, it decides to override MB number five. However, this MB is locked, so the new message cannot be written there. It remains in the Rx SMB waiting for the MB to be unlocked, and only then will be written to the MB.

When the MB is not unlocked in time and yet another new message with the same ID arrives, the new message overwrites the message in the Rx SMB and there is no indication of lost messages either in the CODE field of the MB or in the Error and Status Register.

While the message is being moved-in from the Rx SMB to the MB, the BUSY bit on the CODE field is enabled. If the processor reads the C/S word and finds out that the BUSY bit is set, it should defer accessing the MB until the BUSY bit is cleared.

NOTE: If the BUSY bit is enabled or if the MB is empty, then reading the C/S word does not lock the MB.

Inactivation takes precedence over locking. If the processor inactivates a locked Rx MB, then the lock status is negated and the MB is marked as invalid for the current matching round. Any pending message on the Rx SMB is transferred to the MB. An MB is unlocked when the processor reads the CANFD\_TMR register, or the C/S word of another MB.

Lock and unlock mechanisms have the same functionality in both normal and freeze modes.

An unlock during normal or freeze mode results in the move-in of the pending message. However, the move-in is postponed if an unlock occurs during a low power mode, and it takes place only when the module resumes to normal or freeze mode.

## Rx FIFO

The Rx FIFO is receive-only and is enabled by asserting the CANFD\_CFG.RFEN bit. The reset value of this bit is zero to maintain software backward compatibility with previous versions of the CANFD module that did not have the FIFO feature.

NOTE: The Rx FIFO must not be enabled when the CAN FD feature is enabled.

The Rx FIFO is 6 messages deep. The memory region occupied by the FIFO structure (both message buffers and FIFO engine) is described in the Rx FIFO Structure section. The processor can read the received messages sequentially, in the order they were received, by repeatedly reading a message buffer structure at the output of the FIFO.

The frames available bit ( CANFD\_IFLG1.MB05 )is enabled when there is at least one frame available to be read from the Rx FIFO. An interrupt is generated if it is enabled by the corresponding mask bit. Upon receiving the interrupt, the processor reads the message (accessing the output of the FIFO as a message buffer) and the CANFD\_RX\_FIFO register, then clears the interrupt. If there are more messages in the Rx FIFO, the act of clearing the interrupt updates the output of the FIFO with the next message and updates the CANFD\_RX\_FIFO register with the attributes of that message, reissuing the interrupt to the processor. Otherwise, the flag remains negated. The output of the Rx FIFO is only valid while the CANFD\_IFLG1.MB05 bit enabled.

The Rx FIFO warning bit ( CANFD\_IFLG1.MB06 ) is enabled when the number of unread messages in the Rx FIFO is increased to five from four due to the reception of a new message, meaning that the Rx FIFO is almost full. The flag remains enabled until the processor clears it.

The Rx FIFO overflow bit ( CANFD\_IFLG1.MB07 ) is enabled when an incoming message is lost because the Rx FIFO is full. The CANFD\_IFLG1.MB07 bit is not set when the Rx FIFO is full, and the message is captured by a mailbox. The CANFD\_IFLG1.MB07 remains set until the processor clears it.

Clearing one of those three flags does not affect the state of the other two.

An interrupt is generated when an IFLAG bit asserts and the corresponding mask bit is enabled.

A powerful filtering scheme is provided to accept only frames intended for the target application, reducing the interrupt servicing workload. The filtering criteria is specified by programming a table of up to 128 32-bit registers, according to the CANFD\_CTL2.RFFNUM bit setting, which is configurable to one of the following formats:

- Format A: 128 IDAFs (extended or standard IDs including IDE and RTR)
- Format B: 256 IDAFs (standard IDs or extended 14-bit ID slices including IDE and RTR)
- Format C: 512 IDAFs (standard or extended 8-bit ID slices)

NOTE: A chosen format is applied to all entries of the filter table. It is not possible to mix formats within the table.

Every frame available in the FIFO has a corresponding identifier acceptance filter hit indicator (IDHIT) that can read in the IDHIT field from C/S word. Another way the processor can obtain this information is by accessing the CANFD\_RX\_FIFO register. The CANFD\_RX\_FIFO.IDHIT field refers to the message at the output of the FIFO

and is valid while the CANFD\_IFLG1.MB05 flag is enabled. The CANFD\_RX\_FIFO register must be read only before clearing the flag, to guarantee that the information refers to the correct frame within the FIFO.

Up to 32 elements of the filter table are individually affected by the individual mask registers ( CANFD\_IMSK1 and CANFD\_IMSK2 ), according to the setting of the CANFD\_CTL2.RFFNUM bit, allowing very powerful filtering criteria to be defined. If the CANFD\_CFG.IRMQEN bit is disabled, then the FIFO filter table is affected by the CANFD\_RX\_FIFO\_GMSK register value.

## Rx FIFO Under DMA Operation

The Rx FIFO can support DMA. Using the Rx FIFO with the DMA feature is enabled by enabling both CANFD\_CFG.RFEN and CANFD\_CFG.DMAEN bits. The DMA controller reads the received message by reading a message buffer structure at the FIFO output port at the 0x80-0x8C address range.

The reset value of the CANFD\_CFG.DMAEN bit is zero. When the CANFD\_CFG.DMAEN is enabled, the processor must not access the FIFO output port address range. Before enabling the CANFD\_CFG.DMAEN bit, the processor must service the IFLAGs asserted in the Rx FIFO region. Otherwise, these IFLAGs may show that the FIFO has data to be serviced, and mistakenly generate a DMA request. Before disabling the CANFD\_CFG.DMAEN , the processor must perform a clear FIFO operation.

The Frames available in Rx FIFO bit ( CANFD\_IFLG1.MB05 ) is asserted when there is at least one frame available to be read from the FIFO; consequently, a DMA request is generated simultaneously. Upon receiving the request, the DMA controller reads the message (accessing the output of the FIFO as a message buffer). The DMA reading process must end by reading address 0x8C, which clears the CANFD\_IFLG1.MB05 bit and updates both the FIFO output with the next message (if FIFO is not empty) and the CANFD\_RX\_FIFO register with the attributes of the new message. If there are more messages stored in the FIFO, the CANFD\_IFLG1.MB05 bit is re-enabled, and another DMA request is issued.

NOTE: The CANFD\_RX\_FIFO register contents cannot be read after the DMA completes the FIFO read. The IDHIT information is also available in the C/S word at address 0x080.

The CANFD\_IFLG1.MB06 and CANFD\_IFLG1.MB07 flags are not used when the DMA feature is enabled.

When the CANFD module is working with DMA, the processor does not receive any Rx FIFO interrupts and must not clear the related IFLAGs. In addition, the related IMASKs are not used to mask the generation of DMA requests.

There is no dedicated DMA channel for reading the CANFD FIFO, however, the MDMA along with the TRU can read from the FIFO RAM. This works by configuring the CANFD module to generate a trigger whenever the CANFD\_IFLG1.MB05 bit is set. The MDMA is configured to wait for the trigger from the CANFD module and on receiving the trigger, read from the CANFD FIFO start address at 0x80 offset. Once 16 bytes of FIFO data are read, the CANFD\_IFLG1.MB05 bit is cleared.

Only MDMA2 and MDMA6 streams can access the CANFD FIFO.

## Clear FIFO Operation

When the CANFD\_CFG.RFEN is enabled, the clear FIFO operation empties the FIFO contents. With the CANFD\_CFG.RFEN bit enabled, the clear FIFO occurs when the processor writes a one to the

CANFD\_IFLG1.MB01 bit. This operation only can only occur in freeze mode and is blocked by hardware in other modes. This operation does not clear the FIFO IFLAGs; consequently, the processor must service all FIFO IFLAGs before executing the clear FIFO task.

When the Rx FIFO is working with DMA, the clear FIFO operation clears the CANFD\_IFLG1.MB05 flag and the DMA request is canceled.

NOTE: The clear FIFO operation does not clear IFLAGs, except when the CANFD\_CFG.DMAEN bit is enabled. When the CANFD\_CFG.DMAEN bit is enabled, only the CANFD\_IFLG1.MB05 flag is cleared.

## CAN Protocol Features

This section describes the CAN protocol related features.

## CAN FD Frames

The ISO 11898-1 standard specifies the classical Frame format compliant to ISO 11898-1 (2003) and introduces the CAN flexible data rate frame format. The classical frame format allows bit rates up to 1 Mbit/s and payloads up to 8 bytes per frame. The flexible data rate frame format allows bit rates higher than 1 Mbit/s and payloads longer than 8 bytes per frame. The CANFD module can receive and transmit CAN FD messages interleaved with classical CAN messages.

There are three additional control bits in the CAN FD frame:

- The extended data length (EDL) bit enables a longer data payload with different data length coding.
- The bit rate switch (BRS) bit decides whether the bit rate is switched inside a CAN FD format frame.
- The error state indicator (ESI) flag is transmitted dominant by error active nodes, and recessive by error passive nodes.

There are no remote frames in the CAN FD format. A message configured to transmit a remote frame is always sent out in the classical CAN format. When a FD frame is received and matches a mailbox, the RTR bit in the receiving message buffer is cleared. The RTR bit is considered in only in classical CAN frames.

CAN FD messages can be formatted as long frames, in which the data field exceeds 8 bytes and may range from 12 up to 64 bytes. CAN FD messages can also be configured to support bit rate switching, where the control field, the data field, and the CRC field of a CAN frame are transmitted with a higher bit rate than the beginning and the end of the frame.

Messages in the classical CAN format are limited to transport a maximum payload of 8 bytes at nominal rate. The CAN Message Format figure illustrates the message formats for classical and FD frames with either a standard or extended ID.

Figure 16-5: CAN Message Format

<!-- image -->

The ability to receive and transmit CAN FD messages is enabled by the CANFD\_CFG.FDEN bit. Either a recessive R0 bit in CAN frames with 11-bit identifiers or a recessive R1 bit in CAN frames with 29-bit identifiers are decoded as an EDL bit (not a reserved one). A CAN FD frame is recognized by a recessive EDL bit, while a classical CAN frame is recognized by a dominant EDL bit. The BRS bit specifies whether this frame switches the bit rate in its data phase. A long frame is decoded according to the DLC field value.

CANFD messages can be transmitted with two different bit rates. The first part of a CAN FD frame, from the start of frame (SOF) bit until the bit rate switch (BRS) bit, also called the arbitration phase, is transmitted with the nominal bit rate based on a set of nominal CAN bit timing configuration values. The second part, from the BRS bit until the CRC delimiter bit, also named the data phase, is transmitted with the data bit rate defined by a second set of CAN data bit timing configuration values. Finally, from the CRC delimiter until the intermission bits, the transmission resumes to nominal bit rate. In CAN FD frames with bit rate switching, the bit timing is changed inside the frame at the sample point of the BRS bit if this bit is recessive. Before the BRS bit, in the CAN FD arbitration

phase, the nominal CAN bit timing defined by the CANFD\_TIMING register and by the CANFD\_CTL1 register for backward compatibility. Upon detecting a recessive BRS bit, the CAN data bit timing is used as defined by the CANFD\_FD\_TIMING register.

NOTE: If the length of the time quantum in the nominal bit timing and the length of the time quantum in the data bit timing are not identical, a quantization error of up to one time quantum of the arbitration phase may be present as a phase error. This situation can occur after the switch from arbitration to data phase and lasts until the next synchronization event. Thus, the length of the time quantum should be the same in nominal and data bit timing to minimize the chance of error frames on the CAN bus, and to optimize the clock tolerance in networks that use CAN FD frames.

The CANFD\_FD\_CTL.BRSEN bit enables the transmission of all frames with bit rate switching if the BRS bit in the selected Tx MB is set. When the bit is cleared, the transmission is performed at nominal rate regardless of the BRS bit value. The CANFD\_FD\_CTL.BRSEN bit can be written any time but takes effect only for the next message transmitted or received.

The nominal bit timing is resumed at either the sample point of the CRC delimiter bit or when an error is detected, whichever occurs first. The CAN FD Message Bit Rate Switching figure describes the mechanism for entering and leaving the data phase when the BRS bit is recessive.

Figure 16-6: CAN FD Message Bit Rate Switching

<!-- image -->

NOTE: In classical CAN frames, the CRC delimiter is one single recessive bit. In CAN FD frames, the CRC delimiter may consist of one or two recessive bits. The CANFD module sends only one recessive bit as the CRC delimiter, but it accepts two recessive bits before the edge from recessive to dominant that starts the acknowledge slot. As a receiver, the CANFD module sends its acknowledge bit after the first CRC delimiter bit. In CAN FD frames, the CANFD module accepts a two-bit dominant ACK slot as a valid ACK to compensate for phase shifts between the receivers.

The value of the ESI bit is determined either by the error state of the transmitter at the start of the transmission, if the frame is originated in the CANFD node, or by the original transmitting node if the CANFD module is acting as a gateway for the message. If the transmitter is error passive, the ESI is transmitted recessive; otherwise, it is transmitted dominant.

There are different CRC polynomials for different CAN frame formats:

- The first polynomial, CRC\_15, is for all frames in classical CAN format.
- The second, CRC\_17, is for frames in CAN FD format with a data field up to 16 bytes long.
- The third, CRC\_21, is for frames in CAN FD format with a data field longer than 16 bytes.

Each polynomial results in a hamming distance of 6. At the start of the frame, all three CRC polynomials are calculated concurrently. The CRC sequence to be transmitted is selected by the values of the EDL bit and the DLC bit field. When receiving a message, the CANFD module decodes the EDL and DLC bits to select the proper CRC polynomial to check for a CRC error.

In CAN FD format frames, stuff bits are included in the bit stream for CRC calculation. In classical CAN format frames, stuff bits are not included. After the transmission of the last bit relevant to the CRC calculation, the CANFD\_FD\_CRC register stores the calculated CRC for the transmitted message, with the adequate length for the type of message, for both CAN FD and non-FD messages. The CANFD\_CRC register reports a valid CRC for classical CAN messages only.

In CAN FD format frames, the CAN bit stuffing method is changed for the CRC sequence so that the stuff bits are inserted at fixed positions. When the CANFD module is transmitting a CAN FD frame, a fixed stuff bit is inserted just before the first bit of the CRC sequence, even if the last bits of the preceding field do not fulfill the CAN stuff condition. Additional stuff bits are inserted after each fourth bit of the CRC sequence. The value of any fixed stuff bit is the inverse value of its preceding bit. When the CANFD module is receiving a CAN FD frame, it discards the fixed stuff bits from the bit stream for the CRC check. A stuff error is detected if the fixed stuff bit has the same value as its preceding bit.

The CANFD module detects errors in CAN FD frames the same way as in Classical CAN frames. The error counters CANFD\_ECR.RXERRCNT and CANFD\_ECR.TXERRCNT in the CANFD\_ECR register accumulate the counts of Rx and Tx errors, respectively, for both FD and non-FD frames indistinctly. There are two extra error counters ( CANFD\_ECR.RXERRCNTF and CANFD\_ECR.TXERRCNTF ) that accumulate Rx and Tx errors occurring in the data phase of CAN FD frames with the BRS bit set. The rules for updating the error counters are the same for both CAN FD and non-FD frames.

The error flags CANFD\_ESR1.B1ERR , CANFD\_ESR1.B0ERR , CANFD\_ESR1.ACKERR , CANFD\_ESR1.CRCERR , CANFD\_ESR1.FRMERR , and CANFD\_ESR1.STFERR report errors in both CAN FD and non-FD frames. They also generate the ERRINT interrupt if the CANFD\_CTL1.ERRMSK bit is enabled. The CANFD\_ESR1 register has additional error flags ( CANFD\_ESR1.B1ERRF , CANFD\_ESR1.B0ERRF , CANFD\_ESR1.CRCERRF , CANFD\_ESR1.FRMERRF , and CANFD\_ESR1.STFERRF ) to individually indicate the occurrence of errors in the data phase of CAN FD frames with the BRS bit set. There is no ACKERR detected in the data phase of a CAN FD frame. Fault confinement status reported with the CANFD\_ESR1.FLTCONF bit is the same for both CAN FD and classical CAN frames, and is based on CANFD\_ECR.RXERRCNT and

CANFD\_ECR.TXERRCNT error counters only. Information contained in CANFD\_ECR.RXERRCNTF and CANFD\_ECR.TXERRCNTF counters may be considered as status to help detect the error nature related to the bit rate value. The CANFD State Related Bit Encoding table shows how the CANFD state is related to the CANFD\_ESR1.SYNC , CANFD\_ESR1.IDLE , CANFD\_ESR1.TXINPROG , and CANFD\_ESR1.RXINPROG bits.

Table 16-19: CANFD State Related Bit Encoding

|   SYNC |   IDLE1 | TXINPROG   | RXINPROG   | CANFD State                 |
|--------|---------|------------|------------|-----------------------------|
|      0 |       0 | 0          | 0          | Not synchronized to CAN bus |
|      1 |       1 | x          | x          | Idle                        |
|      1 |       0 | 1          | 0          | Transmitting                |
|      1 |       0 | 0          | 1          | Receiving                   |

When the CANFD is in the data phase, either transmitting or receiving a CAN FD message, and detects an error, it immediately switches back to the arbitration phase and to the nominal rate to start an error flag

Resynchronization and hard synchronization occur in CAN FD frames in the same way as in classical CAN frames. A hard synchronization is also performed at the recessive-to-dominant edge from EDL to R0 in CAN FD format frames. The CANFD does not resynchronize while transmitting in the CAN FD data phase.

## Transceiver Delay Compensation

The CAN FD protocol allows the transmission and reception of data at a higher bit rate than the nominal rate used in the arbitration phase when the BRS bit of the message is set. This feature enables the use of rates up to 8 Mbps.

During the data phase of a CAN FD frame, the transmitter detects a bit error if it cannot receive its own latest transmitted bit at the sample point of that bit. When bit rate switching is enabled, the length of the CAN bit time in the data phase can become shorter than the transceiver's loop delay, thus impeding the correct comparison between the transmitted bit and the received bit within the current CAN bit time interval.

The CANFD module supports an optional transceiver delay compensation (TDC) mechanism that defines a secondary sample point where the transmitted bit is correctly compared with the received bit to check for bit errors.

The TDC mechanism is enabled by the CANFD\_FD\_CTL.TDCOMPEN bit and is effective only during the data phase of CAN FD frames having the BRS bit set. It has no effect either on non-FD frames, or on FD frames transmitted at normal bit rate. The TDC mode is active from the sample point of the BRS bit until the sample point of the CRC delimiter bit, provided the respective message under transmission has the BRS bit set. When it is active, a comparison is done between the real received bit and the delayed transmitted bit, where the delay is calculated based on the measured transceiver loop delay.

NOTE: The actual value of the CRC delimiter bit is disregarded by transmitters using the TDC mechanism. A global error at the end of the CRC field will cause the receivers to send error frames that the transmitter will detect during an acknowledge or end of frame.

For every transmitted FD frame having the BRS bit set, the delay measurement is triggered by the transition from the recessive EDL bit to the dominant R0 bit, as shown in the Transceiver Loop Delay Measurement figure.

Figure 16-7: Transceiver Loop Delay Measurement

<!-- image -->

The loop delay is measured in PE clock periods (CANCLK) , from the transmitted EDL-R0 edge to the received EDL-R0 edge. The position of the secondary sample point is defined by the measured loop delay time added to an offset value specified with the CANFD\_FD\_CTL.TDCOMPOFF bit. The CANFD\_FD\_CTL.TDCOMPVAL bit field stores the result of this calculation. The CANFD\_FD\_CTL.TDCOMPVAL value saturates at the maximum value of 15 CANCLK when the delay measurement is too long.

The measured loop delay is not enough to be used to define the secondary sample point because it relates to the CAN bit edges. The transceiver delay compensation offset ( CANFD\_FD\_CTL.TDCOMPOFF ) shifts the secondary sample point from the edge to an intermediate point inside the bit time (for example, half of the bit time in the data phase), far away from its edges. Therefore, the CANFD\_FD\_CTL.TDCOMPOFF value cannot be larger than the CAN bit duration in the data phase.

During the data phase of CAN FD frames with bit rate switching enabled, at the onset of every Tx CAN bit, the transmitted Tx bit value is temporarily stored in a buffer and a time countdown based on the CANFD\_FD\_CTL.TDCOMPVAL value is started which ends with the comparison of the received Rx bit (delayed by the external loop delay plus the specified offset) with the stored Tx bit. If a bit error is detected at the secondary sample point, the CANFD module issues an error flag to the CAN bus at the next sample point.

During the arbitration phase, the delay compensation is always disabled. The maximum delay that can be compensated by the transceiver delay compensation during the data phase is 3 CAN bit times - 2 Tq. Beyond this limit, the CANFD\_FD\_CTL.TDCOMPFAIL flag is set to indicate when the TDC mechanism is out of range and unable to compensate the transceiver loop delay.

## Remote Frames

A remote frame is a special kind of frame. The processor can program a mailbox to be a remote request frame by configuring the mailbox to transmit with the RTR bit set to one. After the remote request frame is transmitted successfully, the mailbox becomes a receive message buffer, with the same ID as before.

When a remote request frame is received by the CANFD module, it is treated in one of the following ways, depending on the CANFD\_CTL2.RRS and CANFD\_CFG.RFEN bits:

- When RRS is disabled, the frame ID is compared to the IDs of the transmit message buffers with the CODE field 1010. If there is a matching ID, then this mailbox frame is transmitted. Note that if the matching mailbox has the RTR bit set, then the CANFD module transmits a remote frame as a response. The received remote request frame is not stored in a receive buffer. It is only used to trigger a transmission of a frame in response. The mask registers are not used in remote frame matching, and all ID bits (except RTR) of the incoming received frame match. In the case that a remote request frame is received and matches a mailbox, this message buffer immediately enters the internal arbitration process, but is considered a normal Tx mailbox, with no higher priority. The data length of this frame is independent of the DLC field in the remote frame that initiated its transmission.
- When RRS is enabled, the frame ID is compared to the IDs of the receive mailboxes with the CODE field 0100, 0010, or 0110. If there is a matching ID, then this mailbox stores the remote frame in the same fashion of a data frame. No automatic remote response frame is generated. The mask registers are used in the matching process.
- When RFEN is asserted, the CANFD does not generate an automatic response for remote request frames that match the FIFO filtering criteria. If the remote frame matches one of the target IDs, it is stored in the FIFO and presented to the processor. Note that for filtering formats A and B, it is possible to select whether remote frames are accepted or not. For format C, remote frames are always accepted (if they match the ID). Remote request frames are considered normal frames,and generate a FIFO overflow when a successful reception occurs, and the FIFO is already full.

NOTE: There is no remote frame in the CAN FD format. The RTR bit is replaced by a fixed dominant RRS bit. The CANFD receives and transmits remote frames in the classical CAN format.

## Overload Frames

The CANFD module transmits overload frames due to the detection of the following CAN bus conditions:

- Detection of a dominant bit in the first/second bit of intermission.
- Detection of a dominant bit at the 7th bit (last) of end of frame field (Rx frames).
- Detection of a dominant bit at the 8th bit (last) of error frame delimiter or overload frame delimiter.

## Time Stamp

The value of the CANFD\_TMR register is sampled at the beginning of the identifier field on the CAN bus and stored at the end of the move-in process in the TIME STAMP field, providing network behavior with respect to time. The free running timer is clocked by the CANFD bit-clock, which defines the baud rate on the CAN bus. During a message transmission or reception, it increments by one for each bit that is received or transmitted. When there is no message on the bus, it counts using the previously programmed baud rate.

The CANFD\_TMR register is not incremented during module disable, doze, stop, and freeze modes. It can be reset upon a specific frame reception, enabling network time synchronization.

## Protocol Timing

The CAN Engine Clocking Scheme figure shows the structure of the clock generation circuitry that prescales the CAND module clock to generate the Sclock.

<!-- image -->

Figure 16-8: CAN Engine Clocking Scheme

The CANFD module supports a variety of means to set up bit timing parameters that are required by the CAN protocol. The CANFD\_CTL1 register has fields to control bit timing parameters: CANFD\_CTL1.PRESDIV , CANFD\_CTL1.PROPSEG , CANFD\_CTL1.PSEG1 , CANFD\_CTL1.PSEG2 , and CANFD\_CTL1.RJW .

The CANFD\_TIMING register extends the range of the CAN bit timing variables in the CANFD\_CTL1 register. The CANFD\_TIMING register provides a second set of CAN bit timing variables to be applied at the data phase of CAN FD frames with the BRS bit set.

NOTE: When the CAN FD feature is enabled, always set the CANFD\_TIMING.BTF bit and configure the CAN bit timing variables in the CANFD\_TIMING register.

The CANFD\_CTL1.PRESDIV bit field (as well as the extended range CANFD\_TIMING.EPRESDIV and CANFD\_FD\_TIMING.FPRESDIV for the data phase bits of CAN FD messages) defines the prescaler value that generates Sclock. The Sclock period defines the time quantum used to compose the CAN waveform. A time quantum (Tq) is the atomic unit of time handled by the CAN engine.

Tq = (PRESDIV + 1) / f CANCLK

The bit rate, which defines the rate at which the CAN message is either received or transmitted, is given by the formula:

CAN Bit Time = (Number of Time Quanta in 1 bit time) * Tq

Bit Rate = 1/CAN Bit Time

A bit time is subdivided into three segments:

- SYNC\_SEG - Has a fixed length of one time quantum. Signal edges are expected to occur within this segment.
- Time segment 1 - Includes the propagation segment and the phase segment 1 of the CAN standard. Time segment 1 is programmed by setting the CANFD\_CTL1.PROPSEG and the CANFD\_CTL1.PSEG1 fields so that their sum (plus 2) is in the range of 2 to 16 time quanta. When the CANFD\_TIMING.BTF bit is enabled, the CANFD module uses the CANFD\_TIMING.EPROPSEG and CANFD\_TIMING.EPSEG1 bit fields so that their sum (plus 2) is in the range of 2 to 96 time quanta. For messages in CAN FD format with the BRS bit set, the CANFD uses CANFD\_FD\_TIMING.FPROPSEG and CANFD\_FD\_TIMING.FPSEG1 bit fields, so that their sum (plus 1) is in the range of 2 to 39 time quanta.
- Time Segment 2 - Represents the phase segment 2 of the CAN standard. It can be programmed by setting the CANFD\_CTL1.PSEG2 bit field (plus 1) to be 2 to 8 time quanta long. When the CANFD\_TIMING.BTF

bit is enabled, the CANFD module configures the CANFD\_TIMING.EPSEG2 bit field with a value (plus 1) is in the range of 2 to 32 time quanta. For messages in CAN FD format with the BRS bit set, the CANFD uses the CANFD\_FD\_TIMING.FPSEG2 bit field instead, so that the value (plus 1) is in the range of 2 to 8 time quanta. The time segment 2 cannot be smaller than the information processing time (IPT), which is 2 time quanta.

See the Bit Time Segment Example One figure for an example using CANFD\_CTL1 register bit timing variables for classical can format. See the Bit Time Segment Example Two figure for an example using CANFD\_TIMING and CANFD\_FD\_TIMING register bit timing variables for CAN FD format. See the Time Segment Syntax table for syntax descriptions.

NOTE: For further explanation of the underlying concepts, see the ISO 11898-1 standard and the CAN 2.0A/B protocol specification for bit timing.

The bit time defined by the above time segments must not be smaller than 5 time quanta. For bit time calculations, use an IPT of 2, which is the value implemented in the CANFD module.

Figure 16-9: Bit Time Segment Example One

<!-- image -->

Figure 16-10: Bit Time Segment Example Two

<!-- image -->

Table 16-20: Time Segment Syntax

| Syntax         | Description                                                                                                                                    |
|----------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| SYNC_SEG       | System expects transitions to occur on the bus during this period.                                                                             |
| TSEG1          | Corresponds to the sum of PROPSEG and PSEG1.                                                                                                   |
| TSEG2          | Corresponds to the PSEG2 value.                                                                                                                |
| Transmit Point | A node in transmit mode transfers a new value to the CAN bus at this point.                                                                    |
| Sample Point   | A node samples the bus at this point. If the three samples per bit option is selected, then this point marks the position of the third sample. |

The BOSCH CAN 2.0B Standard Compliant Bit Time Segment Settings table gives some examples of the CAN compliant segment settings for classical CAN format (Bosch CAN 2.0B) (non-FD) messages.

Table 16-21: BOSCH CAN 2.0B Standard Compliant Bit Time Segment Settings

| Time Segment 1   |   Time Segment 2 | Resynchronization Jump Width   |
|------------------|------------------|--------------------------------|
|                  |                2 | 1 .. 2                         |

Table 16-21: BOSCH CAN 2.0B Standard Compliant Bit Time Segment Settings (Continued)

| Time Segment 1   |   Time Segment 2 | Resynchronization Jump Width   |
|------------------|------------------|--------------------------------|
| 5 .. 10          |                  |                                |
| 4 .. 11          |                3 | 1 .. 3                         |
| 5 .. 12          |                4 | 1 .. 4                         |
| 6 .. 13          |                5 | 1 .. 4                         |
| 7 .. 14          |                6 | 1 .. 4                         |
| 8 .. 15          |                7 | 1 .. 4                         |
| 9 .. 16          |                8 | 1 .. 4                         |

NOTE: Ensure the bit time settings comply with the CAN Protocol standard (ISO 11898-1).

Whenever a CAN bit is used as a measure of time duration (for example, estimating the occurrence of a CAN bit event in a message), the number of peripheral clocks in one CAN bit (NumClkBit) is calculated as NumClkBit = (fSYS /fCANCLK) x (PRESDIV + 1) x (PROPSEG + PSEG1 + PSEG2 + 4), where:

- NumClkBit is the number of peripheral clocks in one CAN bit.
- f CANCLK is the PE clock in Hz
- f SYS  is the frequency of operation of the system CHI clock in Hz.
- PSEG1 is the CANFD\_CTL1.PSEG1 value.
- PSEG2 is the CANFD\_CTL1.PSEG2 value.
- PROPSEG is the CANFD\_CTL1.PROPSEG value.
- PRESDIV is the CANFD\_CTL1.PRESDIV value.

The formula above is also applicable to the alternative CAN bit timing variables described in the CANFD\_TIMING register and the CANFD\_FD\_TIMING register. For example, 180 CAN bits = (180 x NumClkBit) peripheral clock periods.

## Arbitration and Matching Timing

During normal reception and transmission, the matching, arbitration, move-in and move-out processes are executed during certain time windows inside the CAN frame, as shown in the following figures.

Figure 16-11: Matching and Move-In Time Windows

<!-- image -->

Figure 16-12: Arbitration And Move-out Time Windows

<!-- image -->

Figure 16-13: Arbitration At The End Of Bus Off And Move-out Time Windows

<!-- image -->

NOTE: In the previous figures, the matching and arbitration timing does not consider the delay caused by the concurrent memory access due to the processor or other internal CANFD submodules.

## Tx Arbitration Start Delay

The Tx arbitration start delay ( CANFD\_CTL2.TXASDLY bit field) is a variable that indicates the number of CAN bits used by the CANFD to delay the Tx arbitration process start point from the first bit of the CRC field of the current frame.

The CANFD\_CTL2.TXASDLY bit field can only be written only in freeze mode and is blocked by hardware in other modes.

The transmission performance is impacted by the ability of the processor to reconfigure MBs for transmission after the end of the internal arbitration process, in which the CANFD module finds the winner MB for transmission according to the standard arbitration process. If the arbitration ends before the first bit of the intermission field, then there is a chance the processor reconfigures some Tx MBs and the winner MB is no longer the best candidate to be transmitted.

The CANFD\_CTL2.TXASDLY bit field is useful to optimize the transmission performance by defining the arbitration start point, as shown in the next figure, based on factors such as the CAN bit timing variables that determine the CAN bit rate and the number of MBs in use by the matching and arbitration processes.

Figure 16-14: Optimal Tx Arbitration Start Point

<!-- image -->

The duration of an arbitration process, in terms of CAN bits, is directly proportional to the number of available MBs and to the CAN bit rate, and inversely proportional to the peripheral clock frequency.

The optimal arbitration timing is that in which the last MB is scanned right before the first bit of the intermission field of a CAN frame. For instance, if there are few MBs and the peripheral/oscillator clock ratio is high and the CAN baud rate is low, then the arbitration is placed closer to the frame's end, adding more delay to its start point, and vice-versa.

When the CANFD\_CTL2.TXASDLY field is set to zero, the arbitration start is not delayed, and more time is reserved for arbitration. On the other hand, when the CANFD\_CTL2.TXASDLY value is close to 24, then the processor can configure a Tx MB later and less time is reserved for arbitration. When too little time is reserved for arbitration, the CANFD module may not be able to find a winner MB in time to be transmitted with the best chance to win the bus arbitration against external nodes on the CAN bus.

The optimal CANFD\_CTL2.TXASDLY value is calculated as follows:

For CAN FD frames and (MAXMB + 1) ≤ NMBEND, TXASDLY = 31 - ( (2*(MAXMB + 1) + 4 )/CPCBN )

For CAN FD frames and (MAXMB + 1) &gt; NMBEND, TXASDLY = 22 - ( (2*(MAXMB + 1) - NMBEND )/ CPCBF )

For non-FD frames, TXASDLY = 25 - ( ( (2*(MAXMB + 1) + 4 ) ) / CPCB ) where:

NMBEND = (( 9 * CPCBN ) - 4) / 2

BITRATEN= fCANCLK / ([1 + (EPSEG1 + 1) + (EPSEG2 + 1) + (EPROPSEG + 1) ] x (EPRESDIV + 1))

BITRATEF= fCANCLK / ([1 + (FPSEG1 + 1) + (FPSEG2 + 1) + FPROPSEG] x (FPRESDIV + 1))

CPCBN = fSYS / BITRATEN

CPCBF = fSYS / BITRATEF

CPCB = CPCBN

- MAXMB is the CANFD\_CFG.MAXMB value.

- NMBEND is the number of MBs that can be scanned by the arbitration process during the 9 last CAN bits at the end of a frame.
- BITRATEN is the CAN bit rate in bits per second calculated by the nominal CAN bit time variables.
- BITRATEF is the CAN bit rate in bits per second calculated by the data CAN bit time variables.
- CPCBN is the number of peripheral clocks per CAN bit in nominal bit rate for CAN FD frames.
- CPCBF is the number of peripheral clocks per CAN bit in data bit rate for CAN FD frames.
- CPCB is the number of peripheral clocks per CAN bit for non-FD frames.
- f CANCLK is the oscillator clock, in Hz.
- f SYS  is the peripheral clock, in Hz.
- EPSEG1 is the CANFD\_TIMING.EPSEG1 or CANFD\_CTL1.PSEG1 value.
- EPSEG2 is the CANFD\_TIMING.EPSEG2 or CANFD\_CTL1.PSEG2 value.
- EPROPSEG is the CANFD\_TIMING.EPROPSEG or CANFD\_CTL1.PROPSEG value.
- EPRESDIV is the CANFD\_TIMING.EPRESDIV or CANFD\_CTL1.PRESDIV value.
- FPSEG1 is the CANFD\_FD\_TIMING.FPSEG1 value.
- FPSEG2 is the CANFD\_FD\_TIMING.FPSEG2 value.
- FPROPSEG is the CANFD\_FD\_TIMING.FPROPSEG value.
- FPRESDIV is the CANFD\_FD\_TIMING.FPRESDIV value.

NOTE: The f SYS  and fCANCLK is same clocked from CDU CLKO4.

The following table give the TXASDLY value calculated for some configuration cases.

- fCANCLK = 40 MHz
- Bit rate in arbitration phase = 1 Mbaud

Table 16-22: TXASDLY Values For Case Example

|   Number of MBs |   TXASDLY Value | Maximum Bit Rate in Data Phase (Mbaud)   |
|-----------------|-----------------|------------------------------------------|
|              16 |              24 | Invalid                                  |
|              32 |              23 | 6.67                                     |
|              54 |              22 | 5.0                                      |
|              64 |              21 | 3.33                                     |
|              96 |              20 | 1.6                                      |

## CANFD Clock Domains and Restrictions

When doing matching and arbitration, the CANFD module needs to scan the entire MB memory during the time slot of one CAN frame, comprising a number of CAN bits. To have sufficient time to do that, there must be a minimum number of peripheral clocks per CAN bit, as specified in the CAN Bit Minimum Peripheral Clock table.

Table 16-23: CAN Bit Minimum Peripheral Clock

|   Number of Mailboxes |   CANFD_CFG.RFEN Bit Value |   Minimum Peripheral Clocks per CAN Bit |
|-----------------------|----------------------------|-----------------------------------------|
|                    16 |                          0 |                                      16 |
|                    32 |                          0 |                                      16 |
|                    64 |                          0 |                                      25 |
|                    96 |                          0 |                                      37 |
|                   128 |                          0 |                                      49 |
|                    16 |                          1 |                                      16 |
|                    32 |                          1 |                                      17 |
|                    64 |                          1 |                                      30 |

For the classical CAN frame format, the minimum number of peripheral clocks per CAN bit specified in the TXASDLY Values For Case Example table determines the minimum peripheral clock frequency for a given number of mailboxes and for an expected CAN bit rate. The CAN bit rate depends on the number of time quanta in a CAN bit, that can be defined by adjusting one or more of the bit timing values contained in either the CANFD\_CTL1 register or the CANFD\_TIMING register. The time quantum (Tq) is defined in the Protocol Timing section. The minimum number of time quanta per CAN bit must be 8, so the CAN clock frequency is at least 8 times the CAN bit rate.

For the CAN FD frame format, the number of peripheral clocks per CAN bit in nominal bit rate (NumClkNomBit) is calculated with the following equation:

NumClkNomBit =(fSYS/fCANCLK) X (PRESDIV + 1) X (PROPSEG + PSEG1 + PSEG2 + 4)

## = fSYS/NomBitRate

where PRESDIV, PSEG1 and PSEG2 are CAN bit time values in the CANFD\_CTL1 register. Alternatively, the EPRESDIV, EPSEG1 and EPSEG2 values in the CANFD\_TIMING register can be used instead. NumClkNomBit is also calculated as a function of the expected nominal bit rate used in the arbitration phase (NomBitRate).

The number of CAN bits in the data phase of a FD frame with the BRS bit set depends on the number of data bytes in the payload. The number of fast CAN bits (NumOfFastBits) is determined as shown in the CAN FD Frame Fast CAN Bits table. The less the number of data bytes, the less the number of fast CAN bits, and less time is available for the CANFD module to scan the whole MB memory during the internal matching and arbitration processes.

Table 16-24: CAN FD Frame Fast CAN Bits

|   Minimum Data Bytes | DLC Field   |   Number of Fast Bits |
|----------------------|-------------|-----------------------|
|                    0 | 0x0         |                    21 |
|                    1 | 0x1         |                    29 |
|                    2 | 0x2         |                    37 |
|                    3 | 0x3         |                    45 |
|                    4 | 0x4         |                    53 |
|                    5 | 0x5         |                    61 |
|                    6 | 0x6         |                    69 |
|                    7 | 0x7         |                    77 |
|                    8 | 0x8         |                    85 |
|                   12 | 0x9         |                   117 |
|                   16 | 0xA         |                   149 |
|                   20 | 0xB         |                   186 |
|                   24 | 0xC         |                   218 |
|                   32 | 0xD         |                   282 |
|                   48 | 0xE         |                   410 |
|                   64 | 0xF         |                   538 |

The critical part of a CAN FD frame is during the data phase, where the CAN bit rate is faster than in the arbitration phase. The minimum number of peripheral clocks per fast CAN bit (MinNumClkFastBit) is calculated so that enough time is available for the CANFD module to scan the MB memory during reception and transmission. The equation below calculates this constraint:

MinNumClkFastBit = ( ( 8.5 x MaxNumOfMb ) + 64 - ( 9 x NumClkNomBit ) ) / NumOfFastBits where MaxNumOfMb is the maximum number of available mailboxes defined in the CANFD\_CFG.MAXMB bit field.

The clock domain crossing circuit between the CHI and PE submodules also imposes a minimum number of peripheral clocks per fast CAN bit for the handshake mechanism to work properly without losing status information through the interface, as shown in the following equation:

MinNumClkFastBit = 3 x ( 1 + ( f SYS /f CANCLK ))

Therefore, the minimum number of peripheral clocks per fast CAN bit (MinNumClkFastBit) is determined by the larger of the two values calculated above:

MinNumClkFastBit = Maximum ( MinNumClkFastBit , MinNumClkFastBit )

Then, the maximum CAN bit rate in the data phase of CAN FD frames (DataBitRateMAX) is calculated as follows:

DataBitRate = f CANCLK  / ( ROUNDUP ( ( MinNumClkFastBit x f CANCLK  ) / f SYS

The peripheral and oscillator clock frequencies, the maximum number of mailboxes, and the expected nominal bit rate affect the maximum data bit rate attainable by the CANFD module in CAN FD mode. In addition, the data bit rate depends on the minimum payload size of FD frames used in each application.

To illustrate how the CAN FD bit rate is affected by the configuration of CANFD variables, an consider an application example with the peripheral and oscillator clock frequencies set to 50 MHz.

1. Considering the nominal bit rate as 1 Mbps, the number of peripheral clocks per CAN bit in nominal bit rate is calculated as follows: NumClkNomBit = ( 50 x 10 6  )/( 1 x 10 6  ) = 50
2. The number of fast CAN bits (NumOfFastBits) is determined as shown in the CAN FD Frames Maximum CAN Bit Rate In Data Phase table. For example, if the minimum payload in FD frames is 8 bytes, then there are 85 CAN bits in the data phase.
3. Assuming the maximum number of mailboxes is 96, the minimum number of peripheral clocks per fast CAN bit (MinNumClkFastBit) can be calculated as follows:

MinNumClkFastBit = ( (8.5 X 96) + 64 - (9 X 50) ) / 85 = 5.06

MinNumClkFastBit = 3 x (1 + 50/50 ) = 6.0

MinNumClkFastBit = Maximum ( 5.06 , 6.0 ) = 6.0

As demonstrated in this example, even though the oscillator clock frequency (50 MHz) is adequate to generate a data rate of 8 Mbps in CAN FD mode, the specific CANFD configuration limits this rate to 6.667 Mbps. This limitation is due to the low peripheral clock frequency that imposes the MinNumClkFastBit B  limit.

The CAN FD Frames Maximum CAN Bit Rate In Data Phase table shows the maximum data rate for CAN FD according to the clock frequencies, payload size, and number of available mailboxes. As shown in the CAN FD Frames Maximum CAN Bit Rate In Data Phase table, for some cases, if the number of available mailboxes is reduced, the CANFD module achieves a data rate up to 8 Mbps.

Table 16-25: CAN FD Frames Maximum CAN Bit Rate In Data Phase

|   Peripheral Clock Frequency (MHz) | Payload Size   |   Number of Available Mailboxes |   Maximum Data Rate (Mbps) |
|------------------------------------|----------------|---------------------------------|----------------------------|
|                                 40 | 8              |                              94 |                      6.667 |
|                                 40 | 8              |                             114 |                      5     |
|                                 40 | 12             |                             117 |                      6.667 |
|                                 40 | 12             |                             128 |                      5.714 |
|                                 50 | 12 to 64       |                             128 |                      6.667 |
|                                 60 | 8              |                             128 |                      8     |
|                                 60 | 12             |                             128 |                      8     |

Table 16-25: CAN FD Frames Maximum CAN Bit Rate In Data Phase (Continued)

|   Peripheral Clock Frequency (MHz) |   Payload Size |   Number of Available Mailboxes |   Maximum Data Rate (Mbps) |
|------------------------------------|----------------|---------------------------------|----------------------------|
|                                 67 |              6 |                             128 |                          8 |
|                                 80 |              3 |                             128 |                          8 |
|                                100 |              0 |                             128 |                          8 |

## CANFD Operating Modes

The CANFD module has functional modes and low-power modes.

CAUTION: The CANFD does not support the permanent dominant failure on the CAN bus line. If a low-power request or freeze mode request is done during a permanent dominant, the corresponding acknowledge can never be asserted.

The CANDFD module has the following main functional modes:

- Normal Mode
- Freeze Mode
- Loop-Back Mode
- Listen-Only Mode
- CAN FD Active Mode

For low-power operation, the CANDFD module has the following modes:

- Module Disable Mode
- Doze Mode
- Stop Mode
- Pretended Network Mode

NOTE: The CANFD does not support the permanent dominant failure on the CAN bus line. If there is a lowpower request or freeze mode request during a permanent dominant, the corresponding acknowledge will not be asserted.

## Normal Mode

In normal mode, the CANFD module operates by receiving and transmitting message frames. In this mode, errors are handled normally, and all CAN protocol functions are enabled.

## Freeze Mode

In freeze mode, there is no transmission or reception of frames and synchronicity to the CAN bus is lost. Freeze mode is enabled by setting the CANFD\_CFG.FRZ bit.

Freeze mode is requested either by the processor by enabling the CANFD\_CFG.HALT bit or when it is put into debug mode. In both cases, when the CANFD\_CFG.FRZ bit is enabled, the CANFD module must not be in a lowpower mode. Freeze mode is also requested through the automatic assertion of both the CANFD\_CFG.HALT and CANFD\_CFG.FRZACK bits when the CANFD\_MEC.NCERRFRZEN bit is set and a non-correctable error is detected in a memory read access performed by CANFD internal processes. The acknowledgement is obtained through the assertion by the CANFD\_CFG.FRZACK bit. The CANFD module is only in freeze mode when both the request and acknowledge conditions are satisfied.

In response to a freeze mode request, the CANFD module performs the following operations:

- Waits to be in either the intermission, passive error, bus off, or idle state.
- Waits for all internal activities like arbitration, matching, move-in, and move-out to finish. A pending move-in does not prevent going to freeze mode.
- Ignores the Rx input pin and drives the Tx pin as recessive.
- Stops the prescaler, thus halting all CAN protocol activities.
- Grants write access to the CANFD\_ECR register, which is read-only in other modes.
- Sets the CANFD\_CFG.NOTRDY and CANFD\_CFG.FRZACK bits.

After requesting freeze mode, wait for the CANFD\_CFG.FRZACK bit to be set before executing any other action, or the CANFD module will not operate properly.

To exit freeze mode, disable the CANFD\_CFG.FRZ bit, remove the processor from debug mode, or disable the CANFD\_CFG.HALT bit.

The CANFD\_CFG.FRZACK bit is disabled after the PE recognizes the negation of the freeze request. Once out of freeze mode, the CANFD module tries to re-synchronize to the CAN bus by waiting for 11 consecutive recessive bits.

## Loop-Back Mode

The CANFD module enters loop-back mode when the CANFD\_CTL1.LBEN is set. In loop-back mode, the CANFD module performs an internal loop back for self-test operation. The bit stream output of the transmitter is internally fed back to the receiver input. The Rx CAN input pin is ignored and the Tx CAN output goes to the recessive state (logic '1'). The CANFD module behaves as it normally does when transmitting and treats its own transmitted message as a message received from a remote node. In loop-back mode, the CANFD ignores the bit sent during the ACK slot in the CAN frame acknowledge field to ensure proper reception of its own message. Both transmit and receive interrupts are generated.

## Listen-Only Mode

The CANFD module enters listen-only mode when the CANFD\_CTL1.LOMEN bit is enabled. In listen-only mode, transmission is disabled, all error counters are frozen, and the CANFD module operates in a CAN error passive mode. The module only receives messages acknowledged by another CAN station. If the CANFD detects a message that has not been acknowledged, it flags a BIT0 error (without changing the receive error counter), as if it were trying to acknowledge the message.

## CAN FD Active Mode

In CAN FD active mode, the CANFD module can transmit and receive all messages formatted according to the CAN FD protocol and CAN 2.0 (protocol 2.0) in an interleaved fashion. The CANFD module is put into CAN FD active mode by setting the CANFD\_CFG.FDEN bit when in freeze mode.

## Module Disable Mode

The module disable mode is a low-power mode normally used to temporarily disable a complete CANFD block, with no power consumption. To enter module disable mode, enable the CANFD\_CFG.DIS bit. Once the processor is in module disable mode, the CANFD\_CFG.LPMACK bit is set.

Do not use module disable mode under pretended networking mode. Disable the CANFD\_CFG.DIS bit and wait for CANFD\_CFG.LPMACK bit to clear before enabling the CANFD\_CFG.PNETEN bit.

If the CNAFD module is disabled during transmission or reception, it will perform the following operations:

- Waits to be in either the idle or bus off state, or else waits for the third bit of intermission and then checks it to be recessive.
- Waits for all internal activities like arbitration, matching, move-in, and move-out to finish. A pending move-in is not considered.
- Ignores its Rx input pin and drives its Tx pin as recessive.
- Shuts down the clocks to the PE and CHI submodules.
- Sets the CANFD\_CFG.NOTRDY and CANFD\_CFG.LPMACK bits.

The bus interface unit continues to operate, enabling the processor to access memory mapped registers, except the CANFD\_RX\_FIFO\_GMSK , CANFD\_RX\_14\_MSK , CANFD\_RX\_15\_MSK , and CANFD\_RX\_FIFO\_GMSK registers. The CANFD\_RX\_FIFO , the message buffers, the Rx individual mask registers, and the reserved words within RAM are not accessible when the CANFD module is in the module disable mode.

To exit module disable mode, disable the CANFD\_CFG.DIS bit. If the CANFD module is disabled during freeze mode, it sends a requests to disable the clocks to the PE and CHI submodules, sets the CANFD\_CFG.LPMACK bit, and clears the CANFD\_CFG.FRZACK bit.

## Doze Mode

The doze mode is a system low-power mode in which the processor bus is kept alive, and a global doze mode request is sent to all peripherals asking them to enter low-power mode. Before sending a global doze mode request, set the CANFD\_CFG.DOZEN bit. The global doze request is set with the MISCREG\_CAN\_SYSCTL.CAN1\_IPG\_DOZE or MISCREG\_CAN\_SYSCTL.CAN0\_IPG\_DOZE bit. Once the processor requests the doze mode, the CANFD\_CFG.LPMACK bit is set.

To exit doze mode, disable the CANFD\_CFG.DOZEN bit or clear the IPG\_DOZE bits in the MISCREG\_CAN\_SYSCTL register. The CANFD module also exits doze mode when activity is detected on the CAN bus and the self-wake up mechanism is enabled using the CANFD\_CFG.SLFWAKEN bit.

If the doze mode is triggered during freeze mode, the CANFD module requests to disable the clocks to the PE and CHI submodules, sets the CANFD\_CFG.LPMACK bit, and clears the CANFD\_CFG.FRZACK bit.

If doze mode is triggered during transmission or reception, the CANFD module performs the following operations:

- Waits to be in either idle or bus off state, or else waits for the third bit of intermission and checks it to be recessive.
- Waits for all internal activities like arbitration, matching, move-in, and move-out to finish. A pending move-in is not considered.
- Ignores its Rx input pin and drives its Tx pin as recessive.
- Shuts down the clocks to the PE and CHI submodules.
- Sets the CANFD\_CFG.NOTRDY and CANFD\_CFG.LPMACK bits.

The bus interface unit continues to operate, enabling the processor to access memory mapped registers, except the CANFD\_RX\_FIFO\_GMSK , CANFD\_RX\_14\_MSK , CANFD\_RX\_15\_MSK , and CANFD\_RX\_FIFO\_GMSK registers. The CANFD\_RX\_FIFO , the message buffers, the Rx individual mask registers, and the reserved words within RAM are not accessible when the CANFD module is in the doze mode.

Exiting Doze mode is done in one of the following ways:

- Disabling the doze mode request in the MISCREG\_CAN\_SYSCTL register.
- Disabling the CANFD\_CFG.DOZEN bit.
- When the CANFD module detects activity on the CAN bus and the self-wake up mechanism is enabled using the CANFD\_CFG.SLFWAKEN bit.

When the self-wake up mechanism is enabled at the time that the CANFD module entered doze mode, then upon detection of a recessive-to-dominant transition on the CAN bus, the CANFD module clears the CANFD\_CFG.DOZEN bit, requests to resume clocks, and disables the CANFD\_CFG.LPMACK bit after the CAN PE recognizes the doze mode is disabled.

The CANFD module also sets the CANFD\_ESR1.WAKINT bit, so if the CANFD\_CFG.WAKMSK bit is enabled, it generates a wake up interrupt to the processor. The CANFD module then waits for 11 consecutive recessive bits to

synchronize to the CAN bus. Consequently, the CANFD module does not receive the frame that woke it up. The Wake Up From Doze Mode table details the effect of the CANFD\_CFG.SLFWAKEN and the CANFD\_CFG.WAKMSK bits upon waking from the doze mode.

Table 16-26: Wake Up From Doze Mode

|   SLFWAKEN | WAKINT   | WAKMSK   | CANFD Clocks Enabled   | Wakeup Interrupt Gener- ated   |
|------------|----------|----------|------------------------|--------------------------------|
|          0 | N/A      | N/A      | No                     | No                             |
|          0 | N/A      | N/A      | No                     | No                             |
|          1 | 0        | 0        | No                     | No                             |
|          1 | 0        | 1        | No                     | No                             |
|          1 | 1        | 0        | Yes                    | No                             |
|          1 | 1        | 1        | Yes                    | Yes                            |

Applying a low-pass filter function to the Rx CAN input line while in doze mode will modify sensitivity to CAN bus activity as described in the CANFD\_CFG.WSFLTREN bit description. This feature also protects the CANFD module from waking up due to short glitches on the CAN bus lines that can result from electromagnetic interference in noisy environments.

## Stop Mode

The stop mode is a system low-power mode in which all clocks are stopped for maximum power savings. A global stop request is set with the MISCREG\_CAN\_SYSCTL.CAN0\_IPG\_STOP or MISCREG\_CAN\_SYSCTL.CAN1\_IPG\_STOP bit. Once the processor is in the stop mode, the CANFD\_CFG.LPMACK bit is set.

If the CANFD module receives a global stop mode request during freeze mode, it sets the CANFD\_CFG.LPMACK bit, clears the CANFD\_CFG.FRZACK bit, and then sends the stop acknowledge signal to the processor to shut down the clocks globally.

If stop mode is requested during transmission or reception, the CANFD module performs the following operations:

- Waits to be in either the idle or bus off state, or else waits for the third bit of intermission and checks it to be recessive.
- Waits for all internal activities like arbitration, matching, move-in, and move-out to finish. A pending move-in is not considered.
- Ignores its Rx input pin and drives its Tx pin as recessive.
- Sets the CANFD\_CFG.NOTRDY and CANFD\_CFG.LPMACK bits.
- Sends a stop acknowledge signal to the processor to shut down the clocks globally.

Stop mode is exited when the processor resumes the clocks and removes the stop mode request, or when activity is detected on the CAN bus and the self-wake up mechanism is enabled using the CANFD\_CFG.SLFWAKEN bit.

When the self-wake up mechanism is enabled at the time that the CANFD module entered stop mode, then upon detection of a recessive-to-dominant transition on the CAN bus, the CANFD module sets the CANFD\_ESR1.WAKINT bit, so if the CANFD\_CFG.WAKMSK bit is enabled, it generates a wake up interrupt to the processor. Upon receiving the interrupt, the processor resumes the clocks and removes the stop mode request. The CANFD module then waits for 11 consecutive recessive bits to synchronize to the CAN bus. Consequently, the CANFD module does not receive the frame that woke it up. The Wakeup From Stop Mode table details the effect of the CANFD\_CFG.SLFWAKEN and the CANFD\_CFG.WAKMSK bits upon wakeup from the stop mode. Note that wakeup from the stop mode only works when both bits are asserted.

Table 16-27: Wakeup From Stop Mode

|   SLFWAKEN | WAKINT   | WAKMSK   | Chip Clocks Enabled   | Wakeup Interrupt Gener- ated   |
|------------|----------|----------|-----------------------|--------------------------------|
|          0 | N/A      | N/A      | No                    | No                             |
|          0 | N/A      | N/A      | No                    | No                             |
|          1 | 0        | 0        | No                    | No                             |
|          1 | 0        | 1        | No                    | No                             |
|          1 | 1        | 0        | No                    | No                             |
|          1 | 1        | 1        | Yes                   | Yes                            |

Applying a low-pass filter function to the Rx CAN input line while in stop mode will modify sensitivity to CAN bus activity as described in the CANFD\_CFG.WSFLTREN bit description. This feature also protects the CANFD module from waking up due to short glitches on the CAN bus lines that can result from electromagnetic interference in noisy environments.

## Pretended Network Mode

Pretended networking is a special low-power mode used to receive wake up messages with low power consumption. Pretended networking mode can be selected to operate together with doze mode. Before entering low-power mode, the CANFD\_CFG.PNETEN bit must be asserted. Once in low-power mode, the CHI submodule clocks are shut down and the PE submodule is kept clocked, so that the receive process is still active to filter incoming messages as defined by the configuration.

Upon detecting a wakeup event, a wake up interrupt is issued to the system. When the CANFD\_CFG.PNETEN bit is enabled, the processor disables the self-wake up feature by disabling the CANFD\_CFG.SLFWAKEN bit. Wake up from matched message or time-out in pretended networking modes happens only when pretended networking mode is used along with doze mode. Wakeup is not supported in stop mode.

To enter pretended networking mode, the CANFD must be in normal mode and not in freeze or module disable mode. Under pretended networking mode, the CANFD module stays synchronized with the CAN bus in doze mode. When doze is requested, the CANFD module performs the following operations:

- Waits to be in the idle state, or else waits for the third bit of intermission, and then checks it to be recessive.
- Sets the CANFD\_CFG.LPMACK bit.

- Requests the shutdown of the CHI submodule clock, while keeping the PE submodule clock active.

The CANFD module exits pretended networking mode in the following ways:

- The processor removes the doze mode request.
- The processor disables the CANFD\_CFG.DOZEN bit.

The CANFD will wait until bus idle or the third bit of the intermission state to disable the CANFD\_CFG.LPMACK bit.

The above exit methods are triggered either by the CANFD module detecting a wake up event and issuing the respective interrupt, or by the processor itself upon being woken up. The CANFD will wait until the bus idle state or until the third bit of the intermission state to disable the CANFD\_CFG.LPMACK bit and resume normal mode. This procedure ensures that the CANFD module is synchronized to the CAN bus after exiting pretended networking mode. The processor must wait for the CANFD\_CFG.LPMACK bit to be disabled before performing any access to the CANFD.

## CANFD Event Control

The following section describe how the CANFD module generates and controls events.

## Interrupts

The CANFD module has many interrupt sources: interrupts due to MBs and interrupts due to the OR'd interrupts from MBs, bus off, bus off done, error, error fast (errors detected in the data phase of CAN FD format messages with the CANFD\_FD\_CTL.BRSEN bit set), wake up, wake up match, wake up timeout, Tx warning, and Rx warning states.

Each one of the MBs can be an interrupt source if the corresponding IMASK bit is set. There is no distinction between Tx and Rx interrupts for a particular buffer, under the assumption that the buffer is initialized for either transmission or reception. Each of the buffers has an assigned flag bit in the CANFD\_IFLG1 and CANFD\_IFLG2 registers. The bit is set when the corresponding buffer completes a successful transfer and is cleared when the processor writes it to 1 (unless another interrupt is generated at the same time).

NOTE: The processor must only clear the bit causing the current interrupt, so bit manipulation instructions (BSET) must not be used to clear interrupt flags. These instructions may cause the accidental clearing of interrupt flags which are set after entering the current interrupt service routine.

If the Rx FIFO is enabled ( CANFD\_CFG.RFEN = 1) and DMA is disabled ( CANFD\_CFG.DMAEN = 0), the interrupts corresponding to MBs 0 to 7 have different meanings:

- Bit 7 of the CANFD\_IFLG1 register becomes the 'FIFO Overflow' flag.
- Bit 6 of the CANFD\_IFLG1 register becomes the 'FIFO Warning' flag.
- Bit 5 of the CANFD\_IFLG1 register becomes the 'Frames Available in FIFO' flag.

- Bit 4-0 of the CANFD\_IFLG1 register are unused.

If both Rx FIFO and DMA are enabled ( CANFD\_CFG.RFEN =1 and CANFD\_CFG.DMAEN = 1), the CANFD does not generate any FIFO interrupt. Bit 5 of the CANFD\_IFLG1 register still indicates 'Frames Available in FIFO' and generates a DMA request. Bits 7, 6, and 4-0 are unused.

NOTE: The Rx FIFO cannot be enabled when the CAN FD feature is enabled.

For a combined interrupt where multiple MB interrupt sources are OR'd together, the interrupt is generated when any of the associated MBs (or FIFO, if applicable) generates an interrupt. In this case, the processor must read the CANFD\_IFLG1 and CANFD\_IFLG2 registers to determine which MB or FIFO source caused the interrupt.

The interrupt sources for bus off, bus off done, error, error fast, wake up, Tx warning ,and Rx warning generate interrupts like the MB interrupt sources, and are read from the CANFD\_ESR1 register. The bus off, error, Tx warning, and Rx warning interrupt mask bits are located in the CANFD\_CTL1 register; the wake up interrupt mask bit is located in CANFD\_CFG register.

The interrupt sources for pretended networking (wake up by match flag and wake up by timeout flag) are read in the CANFD\_WUM register and the respective interrupts masks bits are located in CANFD\_PN\_CTL1 register.

## Bus Interface

The processor accesses to CANFD registers are subject to the following rules:

- Read and write accesses to reserved address space result in access errors.
- Write accesses to positions with bits that are all currently read-only results in access errors.

If at least one of the bits is not read-only then no access error is issued. Write permission to positions or some of their bits can change depending on the mode of operation or transitory state. Refer to the register and bit descriptions for details.

- Read and write accesses to unimplemented address space results in access errors.
- Read and write accessed to RAM located positions during low-power modes results in access errors.
- If CANFD\_CFG.MAXMB is programmed with a value smaller than the available number of MBs, then the unused memory space can be used as general purpose RAM space. Reserved words within RAM cannot be used.

For example, suppose the CANFD RAM can support up to 16 MBs, the CANFD\_CTL2.RFFNUM bit field is 0x0, and the CANFD\_CFG.MAXMB bit field is programmed with zero. The maximum number of MBs in this case becomes one. The RAM starts at 0x0080, and the space from 0x0080 to 0x008F is used by the one MB. The memory space from 0x0090 to 0x017F is available. The space between 0x0180 and 0x087F is reserved. The space from 0x0880 to 0x0883 is used by the one individual mask and the available memory in the mask registers space is from 0x0884 to 0x08BF . From 0x08C0 through 0x09DF , there are reserved words for internal use which cannot be used as general purpose RAM. Generally, free memory space for general-purpose usage depends only on the CANFD\_CFG.MAXMB bit field.

## Detection and Correction of Memory Errors

The CANFD module supports detection and correction of errors in memory read accesses. Each byte of CANFD memory is associated with five parity bits, which ensures a Hamming distance of four. The error correction mechanism ensures that in this 13-bit word, errors in one bit can be corrected (correctable errors), and errors in 2 bits can be detected but not corrected (non-correctable errors). Errors in more than 2 bits may not be detected. In case of non-correctable errors, the corrupted data is not changed by the error correction logic. When a read access is performed, the parity bits are used to calculate a syndrome, which indicates the error in each byte.

The CANFD module detects a non-correctable error in the event either an all-zeros or an all-ones read occurs and updates the CANFD\_ERR\_RSYN register. The CANFD\_ERR\_RSYN register holds the syndrome detected in a memory read with an error. It reports the bytes that are read in the 32-bit read transaction.

The Syndrome Definition table shows the SYNDn field in the CANFD\_ERR\_RSYN register, indicates the type of error, and the bit in byte (n) affected by the error. Each BEn field in the CANFD\_ERR\_RSYN register indicates which byte in the 32-bit word reported was effectively read. The syndrome bits are calculated for all bytes, even for the non-read ones. Errors detected in non-read bytes are indicated and reported as described in the Error Indication and Error Reporting sections.

Table 16-28: Syndrome Definition

| SYNDn      | Type   | Bit Affected                    |
|------------|--------|---------------------------------|
| 0x00       | N/A    | None (no error)                 |
| 0x01       | Code   | 0                               |
| 0x02       | Code   | 1                               |
| 0x04       | Code   | 2                               |
| 0x07       | Data   | 5                               |
| 0x08       | Code   | 3                               |
| 0x0E       | Data   | 7                               |
| 0x10       | Code   | 4                               |
| 0x13       | Data   | 2                               |
| 0x15       | Data   | 6                               |
| 0x16       | Data   | 1                               |
| 0x19       | Data   | 3                               |
| 0x1A       | Data   | 4                               |
| 0x1C       | Data   | 0                               |
| 0x06       | N/A    | All-zeros non-correctable error |
| 0x1F       | N/A    | All-ones non-correctable error  |
| All others | N/A    | Non-correctable error           |

Memory errors are indicated to the host through the CANFD\_ERR\_STAT register and bus transfer errors, and reported through the CANFD\_ERR\_RADDR , CANFD\_ERR\_RDAT , and CANFD\_ERR\_RSYN registers.

The error detection and correction mechanism is activated by the CANFD\_MEC.ECCDIS bit. When error detection and correction is disabled, updates on indications and reporting registers are stopped, but the parity bits are still calculated and written along with data in memory write operations to ensure that memory has consistent parity bits associated to the data.

NOTE: All CANFD memory must be initialized before any operation for the parity bits in memory to properly update. The CANFD\_CTL2.WRMFRZEN bit grants write access to all memory positions that require initialization, ranging from 0x080 to 0xADF (and from 0xF28 to 0xFFF when the CAN FD feature is enabled). The CANFD\_RX\_MB\_GMSK , CANFD\_RX\_14\_MSK , CANFD\_RX\_15\_MSK , and CANFD\_RX\_FIFO\_GMSK registers also need to be initialized. The CANFD\_CFG.RFEN bit must not be set during memory initialization. The Identifier Acceptance Filter Fields for Global Bits table shows the configurations for the CANFD\_RX\_FIFO\_GMSK register.

Table 16-29: Identifier Acceptance Filter Fields for Global Mask Bits

| Rx FIFO ID Fil- ter Table Element Format CANFD_CFG. IDAM   | Identifier Acceptance Filter Fields CANFD_RX_FIFO_GMSK.VALUE   | Identifier Acceptance Filter Fields CANFD_RX_FIFO_GMSK.VALUE   | Identifier Acceptance Filter Fields CANFD_RX_FIFO_GMSK.VALUE   | Identifier Acceptance Filter Fields CANFD_RX_FIFO_GMSK.VALUE   | Identifier Acceptance Filter Fields CANFD_RX_FIFO_GMSK.VALUE   | Identifier Acceptance Filter Fields CANFD_RX_FIFO_GMSK.VALUE   |
|------------------------------------------------------------|----------------------------------------------------------------|----------------------------------------------------------------|----------------------------------------------------------------|----------------------------------------------------------------|----------------------------------------------------------------|----------------------------------------------------------------|
| RTR                                                        | IDE                                                            | RXIDA                                                          | RXIDB                                                          | RXIDC                                                          | Reserved                                                       |                                                                |
| A                                                          | VALUE[31]                                                      | VALUE[30]                                                      | VALUE[29:1]                                                    | N/A                                                            | N/A                                                            | VALUE[0]                                                       |
| B                                                          | VALUE[31], VALUE[15]                                           | VALUE[30], VALUE[14]                                           | N/A                                                            | VALUE[29:16], VALUE[13:0]                                      | N/A                                                            | N/A                                                            |
| C                                                          | N/A                                                            | N/A                                                            | N/A                                                            | N/A                                                            | VALUE[31:24], VALUE[23:16], VALUE[15:8], VALUE[7:0]            | N/A                                                            |

To avoid accidentally changing the critical error correction configuration, use the following protocol to enable the CANFD\_MEC register update:

1. By default, the CANFD\_CTL2.ECRWREN bit is zero and the CANFD\_MEC.ECRWRDIS bit is one.
2. Set the CANFD\_CTL2.ECRWREN bit.
3. Clear the CANFD\_MEC.ECRWRDIS bit.
4. All writes to the CANFD\_MEC register must keep the CANFD\_MEC.ECRWRDIS bit cleared.
5. After configuration is done, lock the CANFD\_MEC register by either setting the CANFD\_MEC.ECRWRDIS bit or clearing the CANFD\_CTL2.ECRWREN bit.

## Sources of Memory Access

The CANFD memory can be accessed by two major sources (or requesters):

- Host (processor): The largest word accessed is 32 bits wide.
- CANFD internals processes (Rx matching, Tx arbitration, move-in on reception, move-out on transmission): The largest word accessed is 64 bits wide.

The way that non-correctable errors are indicated and reported depends on the source of access.

## Error Indication

Memory errors are indicated by the CANFD\_ERR\_STAT.HNCEI , CANFD\_ERR\_STAT.INCEI , and CANFD\_ERR\_STAT.CEI flag bits. Non-correctable errors detected in memory reads requested by the host are indicated separately from the errors detected in requests by CANFD internal processes. The CANFD makes no distinction of the source of the access when correctable errors are detected. There are three independent flags for these three cases, and each flag raises an interrupt unless it is masked in the CANFD\_MEC register. If both non-correctable and correctable errors are found in different bytes of the same read operation, both flags are set.

A non-correctable error detected in host access is also indicated as a bus transfer error. A bus wait request asserts to extend the memory transaction to the moment the report registers are updated. This indication cannot be masked. If the CANFD\_ERR\_STAT.HNCEI bit is not masked, the same non-correctable error will raise a bus transfer error and an interrupt request.

Each indication flag has one overrun flag in the CANFD\_ERR\_STAT register. The overrun flags do not request interrupts. Overrun flags for non-correctable errors indicate that other errors of the same nature were detected after current error being treated, while overrun flags for correctable errors indicate that other errors of the same nature were detected before the current error being treated. The recommended handling sequence for error indication is as follows:

1. Get the error report information from the report registers CANFD\_ERR\_RADDR , CANFD\_ERR\_RDAT , and CANFD\_ERR\_RSYN .
2. Use the error report information to take proper measures in the application.
3. Clear the CANFD\_ERR\_STAT.HNCEI , CANFD\_ERR\_STAT.INCEI , CANFD\_ERR\_STAT.CEI flag bits.
4. If the overrun flag is active:
- a. Alert that application that at least one error could not be handled.
- b. Clear the overrun flag bit.

The CANFD internal processes can access memory in transactions larger than 32 bits. For the indication, this kind of access is considered a consecutive sequence of 32-bit accesses. If errors are found in 2 or more 32-bit words, the interrupt and overrun flags are set simultaneously.

## Error Reporting

The report registers CANFD\_ERR\_RADDR , CANFD\_ERR\_RDAT , and CANFD\_ERR\_RSYN provide detailed information about the address read, raw data, and syndrome read with error and indicated by the flags.

The address, data, and syndrome registers are updated simultaneously along with the error flags, according to these rules:

1. If any of the two non-correctable error flags is currently set, the report registers are not updated (the previous non-correctable error reporting is preserved).
2. Otherwise (either no error flag is currently set or only the correctable error flag is currently set), the report registers are updated according to the new error, or according to the most severe of new errors if non-correctable and correctable errors are simultaneously detected.

Reporting of errors detected in accesses larger than 32 bits follows the rules described in the CANFD\_ERR\_RADDR register.

The address reported in the CANFD\_ERR\_RADDR register and defined in the CANFD\_ERR\_IADDR register are not the same as those listed in the module memory map. The relation between the reported addresses and the respective ones in the module memory map is shown in the Error Injection section.

Addresses reported when reading memory portions organized as FIFOs, such as the CANFD\_RX\_FIFO register, refer to the address of the specific entry accessed in the FIFO, not to the FIFO base address.

To ensure coherence of the error report registers, turn off the report update by setting the CANFD\_MEC.RERDIS bit before reading the report registers.

## Response to Errors

Correctable errors have no effect on CANFD operation because affected data is corrected before use by the host or CANFD internal processes.

For host-initiated reads, a non-correctable error affects the host, but does not affect CANFD operation.

Non-correctable errors detected on memory reads requested by the CANFD internal processes result in incorrect operation depending on the state of the CANFD\_MEC.NCERRFRZEN bit, as follows:

- During reception (either matching or move-in processes), when a non-correctable error occurs, an incorrect destination is selected to store the incoming frame, a corrupted frame is stored in the correct destination, or both. If the CANFD\_MEC.NCERRFRZEN is set, the CANFD module stops operation automatically and enters freeze mode to prevent corrupted data from being treated as valid by CANFD internal processes. If the CANFD\_MEC.NCERRFRZEN bit is cleared, the CANFD continues working and a corrupted frame is received.
- During the arbitration process, when a non-correctable error occurs, either a non-highest priority Tx message buffer is mistakenly selected for transmission or the data is corrupted. If the CANFD\_MEC.NCERRFRZEN bit is set, the CANFD module stops operation automatically and enters freeze mode before starting the move-out. If the CANFD\_MEC.NCERRFRZEN bit is cleared, the CANFD proceeds to move-out with a corrupted frame that will transmit on the CAN bus.

- During the move-out process, when a non-correctable error occurs, a corrupted frame is copied from the selected Tx MB that won the arbitration to the Tx SMB for transmission. If the CANFD\_MEC.NCERRFRZEN bit is set, the CANFD module stops operation automatically and enters freeze mode before starting the transmission. If the CANFD\_MEC.NCERRFRZEN bit is cleared, the corrupted frame is transferred from the Tx SMB to the PE submodule and transmits on the CAN bus.
- A non-correctable error can be detected beyond the move-out process, when Tx data is read from the Tx SMB (buffer located in RAM) to be transferred to the PE submodule for transmission. In this case, a frame with corrupted ID and/or data is transmitted on the CAN bus. To prevent the frame from being successfully received by the external nodes, the CANFD module inverts all bits in the CRC field (CRC sequence plus CRC Delimiter) and transmits an error flag just after CRC delimiter because of a self-detecting bit1 error and a form error due to the CRC field inversion. When the CANFD\_MEC.NCERRFRZEN bit is set, the CANFD module stops operation automatically and enters freeze mode just after the error frame. If the CANFD\_MEC.NCERRFRZEN bit is negated, the CANFD module attempts to re-transmit the same frame, when no other higher priority Tx MB is subsequently configured for transmission. In the event the non-correctable error persists, the CANFD module eventually reaches the bus off state because of consecutive error detections. The CANFD\_ECR.TXERRCNT field is updated every time the CANFD module inverts the CRC field causing errors as described above.

When the CANFD\_MEC.NCERRFRZEN bit is set and the CANFD module enters freeze mode, only the host processor can cause then CANFD to exit freeze mode and resume normal mode. Assertion of the CANFD\_MEC.NCERRFRZEN bit is the only way to prevent corrupted frames from transmitting on the CAN bus up to the move-out internal process.

The error report registers provide information to the application for customized handling of these situations.

## Error Injection

The error injection registers CANFD\_ERR\_IADDR , CANFD\_ERR\_IDP , and CANFD\_ERR\_IPP are used to inject errors in memory reads in order to force errors and consequently update the indication and reporting registers.

The Error Injection Address Mapping table shows the relationship between the error injection addresses and the ones in the module memory map. Use this table to convert from the memory map address to the correct location in the physical CANFD RAM.

NOTE: Where pairs of values are provided in the table, the first is the address when the CANFD\_CFG.FDEN bit is disabled and the second is when the CANFD\_CFG.FDEN bit is enabled.

Table 16-30: Error Injection Address Mapping

| RAM Contents    | Injection Address   | Memory Map   |
|-----------------|---------------------|--------------|
| CANFD registers | Not mapped          | N/A          |
| MBs             | 0x0000              | 0x0080       |
| Reserved        | N/A                 | 0x0480       |
| RXIMRs          | 0x0400              | 0x0880       |

Table 16-30: Error Injection Address Mapping (Continued)

| RAM Contents                   | Injection Address   | Memory Map      |
|--------------------------------|---------------------|-----------------|
| Reserved                       | N/A                 | 0x0980          |
| RXFIR_0                        | 0x0500              | 0x0A80          |
| RXFIR_1                        | 0x0504              | 0x0A84          |
| RXFIR_2                        | 0x0508              | 0x0A88          |
| RXFIR_3                        | 0x050C              | 0x0A8C          |
| RXFIR_4                        | 0x0510              | 0x0A90          |
| RXFIR_5                        | 0x0514              | 0x0A94          |
| Reserved                       | N/A                 | 0x0A98          |
| CANFD_RX_MB_GMSK               | 0x0520              | 0x0010          |
| CANFD_RX_FIFO_GMSK             | 0x0524              | 0x0048          |
| CANFD_RX_14_MSK                | 0x0528              | 0x0014          |
| CANFD_RX_15_MSK                | 0x052C              | 0x0018          |
| Tx_SMB                         | 0x0530              | 0x0AB0 / 0x0F28 |
| Rx_SMB0                        | 0x0540 / 0x0578     | 0x0AC0 / 0x0F70 |
| Rx_SMB1                        | 0x0550 / 0x05C0     | 0x0AD0 / 0x0FB8 |
| ECC registers                  | Not mapped          | 0x0AE0          |
| Pretended networking registers | Not mapped          | 0x0B00          |
| CAN FD registers               | Not mapped          | 0x0C00          |
| Reserved                       | N/A                 | 0x0C0C          |

The injection is done by flipping the data and parity bits corresponding to the bits set to 1 in the CANFD\_ERR\_IDP and CANFD\_ERR\_IPP registers. Injection can be selected specifically for memory accesses requested by host or by CANFD internal processes.

In case of accesses larger than 32 bits, the CANFD\_MEC.EXTERRIEN bit extends the injection pattern, replicating it in 32-bit words to fill the width of the access.

NOTE: It is very unlikely, but error injection may correct a bit with an error. This will not raise the error flags and reports as expected.

To ensure coherence among error injection registers and avoid spurious error injections, the CANFD\_MEC.HAERRIEN and CANFD\_MEC.IAERRIEN bits must be cleared while configuring the memory injection.

## CANFD Programming Model

Following sections provide some guidelines for CANFD programming.

## Initialization

The CANFD module is reset in one of the following ways:

- A chip level hard reset that resets all memory mapped registers asynchronously.
- Setting the CANFD\_CFG.SOFTRST bit , which resets some of the memory mapped registers synchronously. The Register Reset States table shows the registers affected by a soft reset.
- Chip level soft reset, which has the same effect as setting the CANFD\_CFG.SOFTRST bit.

A soft reset is synchronous and must follow an internal request and acknowledge procedure across clock domains and takes time to fully propagate the effects. The CANFD\_CFG.SOFTRST bit remains set while soft reset is pending, so software can poll this bit to know when the reset has completed. Also, soft reset cannot be applied while clocks are shut down in a low-power mode. Exit the low-power mode and resume the clocks before applying a soft reset.

Table 16-31: Register Reset States

| Register             | Affected by Hard Reset   | Affected by Soft Reset   |
|----------------------|--------------------------|--------------------------|
| CANFD_TIMING         | Yes                      | No                       |
| CANFD_CRC            | Yes                      | Yes                      |
| CANFD_CTL1           | Yes                      | No                       |
| CANFD_PN_CTL1        | Yes                      | Yes                      |
| CANFD_CTL2           | Yes                      | No                       |
| CANFD_PN_CTL2        | Yes                      | Yes                      |
| CANFD_ECR            | Yes                      | Yes                      |
| CANFD_ERR_IADDR      | Yes                      | Yes                      |
| CANFD_ERR_IDP        | Yes                      | Yes                      |
| CANFD_ERR_IPP        | Yes                      | Yes                      |
| CANFD_ERR_STAT       | Yes                      | Yes                      |
| CANFD_ESR1           | Yes                      | Yes                      |
| CANFD_ESR2           | Yes                      | Yes                      |
| CANFD_FD_TIMING      | Yes                      | No                       |
| CANFD_FD_CRC         | Yes                      | Yes                      |
| CANFD_FD_CTL         | Yes                      | No                       |
| CANFD_FLTR_DLC       | Yes                      | Yes                      |
| CANFD_FLTR_ID1       | Yes                      | Yes                      |
| CANFD_FLTR_ID2_IDMSK | Yes                      | Yes                      |

Table 16-31: Register Reset States (Continued)

| Register                 | Affected by Hard Reset   | Affected by Soft Reset   |
|--------------------------|--------------------------|--------------------------|
| CANFD_IFLG1              | Yes                      | Yes                      |
| CANFD_IFLG2              | Yes                      | Yes                      |
| CANFD_IMSK1              | Yes                      | Yes                      |
| CANFD_IMSK2              | Yes                      | Yes                      |
| CANFD_CFG                | Yes                      | Yes                      |
| CANFD_MEC                | Yes                      | Yes                      |
| CANFD_FLTR_DATA1_HI      | Yes                      | Yes                      |
| CANFD_FLTR_DATA1_LO      | Yes                      | Yes                      |
| CANFD_FLTR_DATA2_DMSK_HI | Yes                      | Yes                      |
| CANFD_FLTR_DATA2_DMSK_LO | Yes                      | Yes                      |
| CANFD_ERR_RADDR          | Yes                      | Yes                      |
| CANFD_ERR_RDAT           | Yes                      | Yes                      |
| CANFD_ERR_RSYN           | Yes                      | Yes                      |
| CANFD_RX_14_MSK          | No                       | No                       |
| CANFD_RX_15_MSK          | No                       | No                       |
| CANFD_RX_FIFO_GMSK       | No                       | No                       |
| CANFD_RX_FIFO            | No                       | No                       |
| CANFD_RX_IMSK[n]         | No                       | No                       |
| CANFD_RX_MB_GMSK         | No                       | No                       |
| CANFD_TMR                | Yes                      | Yes                      |
| CANFD_WMB[n]_DATA_HI     | Yes                      | No                       |
| CANFD_WMB[n]_DATA_LO     | Yes                      | No                       |
| CANFD_WMB[n]_ID          | Yes                      | No                       |
| CANFD_WMB[n]_STAT        | Yes                      | No                       |
| CANFD_WUM                | Yes                      | Yes                      |

Note that the MBs and the Rx individual mask registers are not affected by reset, so they are not automatically initialized.

## CANFD Initialization Sequence

For any configuration change or initialization, the CANFD must be is put into freeze mode. The following steps are a generic initialization sequence applicable to the CANFD module:

1. Initialize the Module Configuration Register ( CANFD\_CFG ):

- Enable the individual filtering per MB and reception queue features by setting the CANFD\_CFG.IRMQEN bit.
- Enable the warning interrupts by setting the CANFD\_CFG.WRNEN bit.
- Optionally disable frame self reception by setting the CANFD\_CFG.SRXDIS bit.
- Enable the Rx FIFO by setting the CANFD\_CFG.RFEN bit.
- If Rx FIFO is enabled and DMA is required, set the CANFD\_CFG.DMAEN bit.
- If Pretended Networking mode is required, set the CANFD\_CFG.PNETEN bit.
- Enable the abort mechanism by setting the CANFD\_CFG.ABORTEN bit.
- Enable the local priority feature by setting the CANFD\_CFG.LPRIOEN bit.
2. Initialize the Control 1 Register ( CANFD\_CTL1 ) and optionally the CAN Bit Timing Register ( CANFD\_TIMING ). Also, initialize the CAN FD CAN Bit Timing Register ( CANFD\_FD\_TIMING ):
- Determine the bit timing parameters: CANFD\_CTL1.PROPSEG , CANFD\_CTL1.PSEG1 , CANFD\_CTL1.PSEG2 , CANFD\_CTL1.RJW .
- Optionally determine the extended bit timing parameters: CANFD\_TIMING.EPROPSEG , CANFD\_TIMING.EPSEG1 , CANFD\_TIMING.EPSEG2 , CANFD\_TIMING.ERJW .
- Determine the CAN FD bit timing parameters: CANFD\_FD\_TIMING.FPROPSEG , CANFD\_FD\_TIMING.FPSEG1 , CANFD\_FD\_TIMING.FPSEG2 , CANFD\_FD\_TIMING.FRJW .
- Determine the bit rate by programming the CANFD\_CTL1.PRESDIV field and optionally the CANFD\_TIMING.EPRESDIV field.
- Determine the CAN FD bit rate by programming the CANFD\_FD\_TIMING.FPRESDIV field. Determine the internal arbitration mode ( CANFD\_CTL1.LBUF bit).
3. Initialize the Message Buffers:
- Initialize the C/S word of all MBs.
- If the Rx FIFO is enabled, initialize the ID filter table.
- Initialize other entries in each MB.
4. Initialize the individual masks in the CANFD\_IMSK1 and CANFD\_IMSK2 registers.
5. Set required interrupt mask bits in the CANFD\_IMSK1 and CANFD\_IMSK2 registers (for all MB interrupts), in the CANFD\_CFG register for the wake up interrupt, and in the CANFD\_CTL1 and CANFD\_CTL2 registers for the bus off and error interrupts.
6. If pretended networking mode is enabled, configure the necessary registers for selective wake up.
7. Disable the CANFD\_CFG.HALT bit.

After the last step listed, the CANFD module attempts to synchronize to the CAN bus.

## Pretended Network - Doze Mode

Complete the following sequence to enter the pretended networking mode in doze mode:

1. Set the CANFD\_CFG.PNETEN bit to enable the controller to enter the pretended networking mode, when one of the low power modes is requested.
2. Clear the CANFD\_PN\_CTL1.WTOFMSK and CANFD\_PN\_CTL1.WUMFMSK bits.
3. Request the CANFD to enter Doze mode.
4. Wait for the CANFD to set the CANFD\_CFG.LPMACK bit.
5. Clear the CANFD\_WUM.WTOF and/or CANFD\_WUM.WUMF bits (if either bit is set).
6. Set the CANFD\_PN\_CTL1.WTOFMSK and CANFD\_PN\_CTL1.WUMFMSK bits.

## Pretended Network - Stop Mode

Complete the following sequence to enter the pretended networking mode in stop mode:

1. Set the CANFD\_CFG.PNETEN bit to enable the controller to enter pretended networking mode, when one of the low power modes is requested.
2. Clear the CANFD\_PN\_CTL1.WTOFMSK and CANFD\_PN\_CTL1.WUMFMSK bits.
3. Request the CANFD to enter Stop mode. Keep CAN0\_IPG\_STOP\_CLK\_EXT/ CAN1\_IPG\_STOP\_CLK\_EXT bits set to keep the clock to register block running.
4. Wait for the CANFD to set the CANFD\_CFG.LPMACK bit.
5. Clear the CANFD\_WUM.WTOF and/or CANFD\_WUM.WUMF bits (if either bit is set).
6. Set the CANFD\_PN\_CTL1.WTOFMSK and CANFD\_PN\_CTL1.WUMFMSK bits.
7. Disable CANFD register clocks by clearing bits CAN0\_IPG\_STOP\_CLK\_EXT/ CAN1\_IPG\_STOP\_CLK\_EXT for CANFD0/CANFD1 respectively.

## ADSP-2159x\_SC592\_SC594 CANFD Register Descriptions

Controller Access Network with Flexible Data Rate (CANFD) (CANFD) contains the following registers.

Table 16-32: ADSP-2159x\_SC592\_SC594 CANFD Register List

| Name          | Description                            |
|---------------|----------------------------------------|
| CANFD_TIMING  | Can Bit Timing Register                |
| CANFD_CRC     | CRC Register                           |
| CANFD_CTL1    | Control 1 Register                     |
| CANFD_PN_CTL1 | Pretended Networking Control1 Register |

Table 16-32: ADSP-2159x\_SC592\_SC594 CANFD Register List (Continued)

| Name                     | Description                                                                            |
|--------------------------|----------------------------------------------------------------------------------------|
| CANFD_CTL2               | Control 2 Register                                                                     |
| CANFD_PN_CTL2            | Pretended Networking Control2 Register                                                 |
| CANFD_ECR                | Error Counter Register                                                                 |
| CANFD_ERR_IADDR          | Error Injection Address Register                                                       |
| CANFD_ERR_IDP            | Error Injection Data Pattern Register                                                  |
| CANFD_ERR_IPP            | Error Injection Parity Pattern Register                                                |
| CANFD_ERR_STAT           | Error Status Register                                                                  |
| CANFD_ESR1               | Error and Status 1 Register                                                            |
| CANFD_ESR2               | Error and Status 2 Register                                                            |
| CANFD_FD_TIMING          | CANFD Bit Timing Register                                                              |
| CANFD_FD_CRC             | CANFD CRC Register                                                                     |
| CANFD_FD_CTL             | CANFD Control Register                                                                 |
| CANFD_FLTR_DLC           | Pretended Networking DLC Filter Register                                               |
| CANFD_FLTR_ID1           | Pretended Networking ID Filter1 Register                                               |
| CANFD_FLTR_ID2_IDMSK     | Pretended Networking ID Filter2 / IDMask Register                                      |
| CANFD_IFLG1              | Mailbox Interrupt Flag 1 Register                                                      |
| CANFD_IFLG2              | Mailbox Interrupt Flag 2 Register                                                      |
| CANFD_IMSK1              | Mailbox Interrupt Mask 1 Register                                                      |
| CANFD_IMSK2              | Mailbox Interrupt Mask 2 Register                                                      |
| CANFD_CFG                | Module Configuration Register                                                          |
| CANFD_MEC                | Memory Error Control Register                                                          |
| CANFD_FLTR_DATA1_HI      | Pretended Networking Payload Low Filter2 Register                                      |
| CANFD_FLTR_DATA1_LO      | Pretended Networking Payload Low Filter1 Register                                      |
| CANFD_FLTR_DATA2_DMSK_HI | Pretended Networking Payload High Filter2 High Order Bits / Payload High Mask Register |
| CANFD_FLTR_DATA2_DMSK_LO | Pretended Networking Payload Low Filter2 / Payload Low Mask Register                   |
| CANFD_ERR_RADDR          | Error Report Address Register                                                          |
| CANFD_ERR_RDAT           | Error Report Data Register                                                             |
| CANFD_ERR_RSYN           | Error Report Syndrome Register                                                         |
| CANFD_RX_14_MSK          | Receive Mailbox14 Mask Register                                                        |
| CANFD_RX_15_MSK          | Receive Mailbox15 Mask Register                                                        |
| CANFD_RX_FIFO_GMSK       | Receive FIFO Global Mask Register                                                      |

Table 16-32: ADSP-2159x\_SC592\_SC594 CANFD Register List (Continued)

| Name                 | Description                                   |
|----------------------|-----------------------------------------------|
| CANFD_RX_FIFO        | Receive FIFO Information Register             |
| CANFD_RX_IMSK[n]     | Receive Individual Mask Register              |
| CANFD_RX_MB_GMSK     | Receive Mailbox Global Mask Register          |
| CANFD_TMR            | Free Running Timer Register                   |
| CANFD_WMB[n]_DATA_HI | Wakeup Message Buffer Data 4-7 Register       |
| CANFD_WMB[n]_DATA_LO | Wakeup Message Buffer Data 0-3 Register       |
| CANFD_WMB[n]_ID      | Wakeup Message ID Buffer Register             |
| CANFD_WMB[n]_STAT    | Wakeup Message Buffer Control/Status Register |
| CANFD_WUM            | Pretended Networking Wakeup Match Register    |

## Can Bit Timing Register

The CANFD\_TIMING register is an alternative way to store the CAN bit timing variables described in the CANFD\_CTL1 register. The CANFD\_TIMING.EPRESDIV , CANFD\_TIMING.EPROPSEG , CANFD\_TIMING.EPSEG1 , CANFD\_TIMING.EPSEG2 , and CANFD\_TIMING.ERJW bit fields are extended versions of the CANFD\_CTL1 register CANFD\_CTL1.PRESDIV , CANFD\_CTL1.PROPSEG , CANFD\_CTL1.PSEG1 , CANFD\_CTL1.PSEG2 and CANFD\_CTL1.RJW bit fields respectively.

The CANFD\_TIMING.BTF bit selects the use of the timing variables defined in the CANFD\_TIMING register. The contents of the CANFD\_TIMING register are not affected by a soft reset.

NOTE: The CAN bit variables in the CANFD\_CTL1 and CANFD\_TIMING registers are stored in the same register.

NOTE: When the CAN FD feature is enabled ( CANFD\_CFG.FDEN is set), always set CANFD\_TIMING.BTF .

Figure 16-15: CANFD\_TIMING Register Diagram

<!-- image -->

Table 16-33: CANFD\_TIMING Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | BTF        | Bit Timing Format Enable. The CANFD_TIMING.BTF bit enables the use of the extended CAN bit timing fields in the CANFD_TIMING register to replace the CAN bit time variables defined in the CANFD_CTL1 register. The CANFD_TIMING.BTF bit can only be written in freeze mode. Disable |
| 31 (R/W)           | BTF        | 0                                                                                                                                                                                                                                                                                    |
| 31 (R/W)           | BTF        | 1 Enable                                                                                                                                                                                                                                                                             |

Table 16-33: CANFD\_TIMING Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:21 (R/W)        | EPRESDIV   | Extended Prescaler Division Factor. When the CANFD_TIMING.BTF bit is enabled, the CANFD_TIMING.EPRESDIV bit field defines the ratio between the PE clock fre- quency and the serial clock (Sclock) frequency. The CANFD_TIMING.EPRESDIV bit field extends the CANFD_CTL1.PRESDIV value range. Sclock frequency = PE clock frequency / (EPRESDIV + 1) The Sclock period defines the time quantum of the CAN protocol. For the reset value, the Sclock frequency is equal to the PE clock frequency. When the CANFD_TIMING.BTF bit is disabled, the CANFD_TIMING.EPRESDIV bit field has no effect. The CANFD_TIMING.EPRESDIV bit field can only be written in freeze mode and is blocked by hardware in other modes. |
| 20:16 (R/W)        | ERJW       | Extended Resync Jump Width. When the CANFD_TIMING.BTF bit is enabled, the CANFD_TIMING.ERJW bit field defines the maximum number of time quanta that a bit time can be changed by one resynchronization. The CANFD_TIMING.ERJW bit field extends the CANFD_CTL1.RJW value range. Resync Jump Width = ERJW + 1 One time quantum is equal to the Sclock period. When the CANFD_TIMING.BTF bit is disabled, the CANFD_TIMING.ERJW bit field has no effect. The CANFD_TIMING.ERJW bit field can only be written in freeze mode and is blocked by hardware in other modes.                                                                                                                                              |
| 15:10 (R/W)        | EPROPSEG   | Extended Propagation Segment. When the CANFD_TIMING.BTF bit is enabled, the CANFD_TIMING.EPROPSEG bit field defines the length of the propagation seg- ment in the bit time. The CANFD_TIMING.EPROPSEG bit field extends the CANFD_CTL1.PROPSEG value range. Propagation Segment Time = (EPROPSEG + 1) * Time-Quanta One time quantum is equal to the Sclock period. When the CANFD_TIMING.BTF bit is disabled, the CANFD_TIMING.EPROPSEG bit field has no effect. The CANFD_TIMING.EPROPSEG bit field can only be written in freeze mode and is blocked by hardware in other modes.                                                                                                                               |

Table 16-33: CANFD\_TIMING Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:5 (R/W)          | EPSEG1     | Extended Phase Segment 1. When the CANFD_TIMING.BTF bit is enabled, the CANFD_TIMING.EPSEG1 bit field defines the length of phase segment 1 in the bit time. The CANFD_TIMING.EPSEG1 bit field extends the CANFD_CTL1.PSEG1 value range. Phase Buffer Segment 1 = (EPSEG1 + 1) * Time-Quanta One time quantum is equal to the Sclock period. When the CANFD_TIMING.BTF bit is disabled, the CANFD_TIMING.EPSEG1 bit field has no effect. The CANFD_TIMING.EPSEG1 bit field can only be written in freeze mode and is blocked by hardware in other modes. |
| 4:0 (R/W)          | EPSEG2     | Extended Phase Segment 2. When the CANFD_TIMING.BTF bit is enabled, the CANFD_TIMING.EPSEG2 bit field defines the length of phase segment 2 in the bit time. The CANFD_TIMING.EPSEG2 bit field extends the CANFD_CTL1.PSEG2 value range. Phase Buffer Segment 1 = (EPSEG1 + 1) * Time-Quanta One time quantum is equal to the Sclock period. When the CANFD_TIMING.BTF bit is disabled, the CANFD_TIMING.EPSEG2 bit field has no effect. The CANFD_TIMING.EPSEG2 bit field can only be written in freeze mode and is blocked by hardware in other modes. |

## CRC Register

The CANFD\_CRC register provides information about the CRC of transmitted messages for non-FD messages. For messages in CAN FD format that require either 17 or 21 bits, the CANFD\_CRC register only reports the 15 low order bits of CRC calculations. For CAN FD format frames, the CANFD\_FD\_CRC register must be used. The CANFD\_CRC register updates at the same time the Tx Interrupt Flag is set.

Figure 16-16: CANFD\_CRC Register Diagram

<!-- image -->

Table 16-34: CANFD\_CRC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22:16 (R/NW)       | MB         | CRC Mailbox. The CANFD_CRC.MB bit field indicates the number of the mailbox corresponding to the value in the CANFD_CRC.TX field.                                                                         |
| 14:0 (R/NW)        | TX         | Transmitted CRC Value. The CANFD_CRC.TX bit field indicates the CRC value of the last transmitted mes- sage for non-FD frames. For CAN FD frames, the CRC value is reported in the CANFD_FD_CRC register. |

## Control 1 Register

The CANFD\_CTL1 register is defined for specific CANFD control features related to the CAN bus, such as bit-rate, programmable sampling point within an Rx bit, loop-back mode, listen-only mode, bus off recovery behavior, and interrupt enabling (bus off, error, warning). It also determines the division factor for the clock prescaler.

The CAN bit timing variables (PRESDIV, PROPSEG, PSEG1, PSEG2, and RJW) are also configurable in the CANFD\_TIMING register, which extends the range of all these variables. If the CANFD\_TIMING.BTF bit is set, the PRESDIV, PROPSEG, PSEG1, PSEG2, and RJW fields of the CANFD\_CTL1 register become read-only.

NOTE: When the CAN FD feature is enabled, do not use the CANFD\_CTL1.PRESDIV , CANFD\_CTL1.RJW , CANFD\_CTL1.PSEG1 , CANFD\_CTL1.PSEG2 , and CANFD\_CTL1.PROPSEG fields of the CANFD\_CTL1 register for CAN bit timing. Instead, use the CANFD\_TIMING.EPRESDIV , CANFD\_TIMING.ERJW , CANFD\_TIMING.EPSEG1 , CANFD\_TIMING.EPSEG2 , and CANFD\_TIMING.EPROPSEG fields of the CANFD\_TIMING register.

The contents of the CANFD\_CTL1 register are not affected by a soft reset.

Figure 16-17: CANFD\_CTL1 Register Diagram

<!-- image -->

Table 16-35: CANFD\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | PRESDIV    | Prescaler Division Factor. The CANFD_CTL1.PRESDIV bit field defines the ratio between the PE clock fre- quency and the serial clock (SCLOCK) frequency. The SCLOCK period defines the time quantum of the CAN protocol. For the reset value, the SCLOCK frequency is equal to the PE clock frequency. The maximum val- ue of PRESDIV is 0xFF, which gives a minimum SCLOCK frequency equal to the PE clock frequency divided by 256. The CANFD_CTL1.PRESDIV bit can only be written in freeze mode and is blocked by hardware in other modes. SCLOCK Frequency = PE Clock Frequency / (PRESDIV + 1) |
| 23:22 (R/W)        | RJW        | Resync Jump Width. The CANFD_CTL1.RJW bit field defines the maximum number of time quanta that a bit time can change with one re-synchronization. One time quantum is equal to the SCLOCK period. The valid programmable values for the CANFD_CTL1.RJW bits are 0-3. This bit field can only be written in freeze mode and is blocked by hardware in other modes. Resync Jump Width = RJW + 1                                                                                                                                                                                                       |
| 21:19 (R/W)        | PSEG1      | Phase Segment 1. The CANFD_CTL1.PSEG1 bit field defines the length of the phase segment 1 of the bit time. The valid programmable values are 0-7. This bit field can only be written in freeze mode and is blocked by hardware in other modes. Phase Buffer Segment 1 = (PSEG1 + 1) x Time-Quanta                                                                                                                                                                                                                                                                                                   |
| 18:16 (R/W)        | PSEG2      | Phase Segment 2. The CANFD_CTL1.PSEG2 bit field defines the length of phase segment 2 of the bit time. The valid programmable values are 1-7. This bit field can only be written in freeze mode and is blocked by hardware in other modes. Phase Buffer Segment 2 = (PSEG2 + 1) x Time-Quanta                                                                                                                                                                                                                                                                                                       |
| 15 (R/W)           | BOFFMSK    | Bus-Off Interrupt Mask. The CANFD_CTL1.BOFFMSK bit provides a mask for the bus off interrupt ( CANFD_ESR1.BOFFINT ). 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 15 (R/W)           | BOFFMSK    | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 16-35: CANFD\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | ERRMSK     | Error Interrupt Mask. The CANFD_CTL1.ERRMSK bit provides a mask for the error interrupt ( CANFD_ESR1.ERRINT ).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Error Interrupt Mask. The CANFD_CTL1.ERRMSK bit provides a mask for the error interrupt ( CANFD_ESR1.ERRINT ).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 14 (R/W)           | ERRMSK     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 14 (R/W)           | ERRMSK     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 12 (R/W)           | LBEN       | Loop-Back Mode. The CANFD_CTL1.LBEN bit configures the CANFD module to operate in loop- back mode. In loop-back mode, the CANFD modules performs an internal loop back to use for self-test operation. The bit stream output of the transmitter is fed back internally to the receiver input. The Rx CAN input pin is ignored and the Tx CAN output goes to the recessive state (logic 1). The CANFD module behaves as it normally does when transmitting, and treats its own transmitted message as a message received from a re- mote node. In loop-back mode, the CANFD module ignores the bit sent during the ACK slot in the CAN frame acknowledge field, generating an internal acknowledge bit to ensure proper reception of its own message. Both transmit and receive interrupts are generat- ed. The CANFD_CTL1.LBEN bit can only be written in freeze mode and is blocked by hardware in other modes. Note that in loop-back mode, the CANFD_CFG.SRXDIS bit cannot be asserted, be- cause it impedes the self reception of a transmitted message. Also, note that the CANFD_FD_CTL.TDCOMPEN bit must be 0 (transceiver delay | Loop-Back Mode. The CANFD_CTL1.LBEN bit configures the CANFD module to operate in loop- back mode. In loop-back mode, the CANFD modules performs an internal loop back to use for self-test operation. The bit stream output of the transmitter is fed back internally to the receiver input. The Rx CAN input pin is ignored and the Tx CAN output goes to the recessive state (logic 1). The CANFD module behaves as it normally does when transmitting, and treats its own transmitted message as a message received from a re- mote node. In loop-back mode, the CANFD module ignores the bit sent during the ACK slot in the CAN frame acknowledge field, generating an internal acknowledge bit to ensure proper reception of its own message. Both transmit and receive interrupts are generat- ed. The CANFD_CTL1.LBEN bit can only be written in freeze mode and is blocked by hardware in other modes. Note that in loop-back mode, the CANFD_CFG.SRXDIS bit cannot be asserted, be- cause it impedes the self reception of a transmitted message. Also, note that the CANFD_FD_CTL.TDCOMPEN bit must be 0 (transceiver delay |
| 11 (R/W)           | TWRNMSK    | compensation feature disabled) when the CANFD_CTL1.LBEN bit is asserted. Tx Warning Interrupt Mask. The CANFD_CTL1.TWRNMSK bit provides a mask for the Tx warning interrupt ( CANFD_ESR1.TXWRNINT ) The CANFD_CTL1.TWRNMSK bit is read as zero when CAN_MCR.WRNEN is ne- gated. The CANFD_CTL1.TWRNMSK bit can only be written when the CANFD_CFG.WRNEN bit is enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | compensation feature disabled) when the CANFD_CTL1.LBEN bit is asserted. Tx Warning Interrupt Mask. The CANFD_CTL1.TWRNMSK bit provides a mask for the Tx warning interrupt ( CANFD_ESR1.TXWRNINT ) The CANFD_CTL1.TWRNMSK bit is read as zero when CAN_MCR.WRNEN is ne- gated. The CANFD_CTL1.TWRNMSK bit can only be written when the CANFD_CFG.WRNEN bit is enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 11 (R/W)           | TWRNMSK    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Tx Warning Interrupt Disabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 11 (R/W)           | TWRNMSK    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Tx Warning Interrupt Enabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 16-35: CANFD\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | RWRNMSK    | Rx Warning Interrupt Mask. The CANFD_CTL1.RWRNMSK bit provides a mask for the Rx warning interrupt ( CANFD_ESR1.RXWRNINT ) The CANFD_CTL1.RWRNMSK bit is read as zero when the CANFD_CFG.WRNEN bit is negated. It can only be written only if the CANFD_CFG.WRNEN bit is enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 7 (R/W)            | SMP        | 1 Rx Warning Interrupt Enabled CAN Bit Sampling. The CANFD_CTL1.SMP bit defines the sampling mode of CAN bits at the Rx input. When the CANFD_CTL1.SMP bit is disabled, one sample determines the bit value. When it is enabled three samples determine the value of the received bit: the regular sample (sample point) and the two preceding samples; a majority rule is used. The CANFD_CTL1.SMP bit can only be written in freeze mode and is blocked by hardware in other modes. Note that for proper operation, when enabling CAN bit sampling, guarantee a mini- mum value of 2 time quanta in the CANFD_CTL1.PSEG1 (or CANFD_TIMING.EPSEG1 ) bit field. SMP cannot be asserted when CAN FD is en- abled ( CANFD_CFG.FDEN = 1). 0 Disable Enable |
| 7 (R/W)            |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 7 (R/W)            |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 16-35: CANFD\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | BOFFREC    | Bus-Off Recovery. The CANFD_CTL1.BOFFREC bit defines how the CANFD module recovers from bus off state. When the CANFD_CTL1.BOFFREC bit is enabled, automatic recovering from the bus off state occurs according to the CAN Specification 2.0B. When the CANFD_CTL1.BOFFREC bit is disabled, automatic recovering from the bus off state is disabled and the CANFD module remains in the bus off state until the CANFD_CTL1.BOFFREC bit is enabled by software. If this occurs before 128 se- quences of 11 recessive bits are detected on the CAN bus, then the bus off recovery happens as if the CANFD_CTL1.BOFFREC bit was never disabled. If the bit is ena- bled after 128 sequences of 11 recessive bits occurred, then the CANFD module will re-synchronize to the bus by waiting for 11 recessive bits before joining the bus. After being enabled, the CANFD_CTL1.BOFFREC bit can be disabled again during the bus off state, but it will be effective only the next time the CANFD module enters the bus off state. If CANFD_CTL1.BOFFREC bit was enabled when the CANFD module entered the bus off state, disabling the CANFD_CTL1.BOFFREC bit during the bus off state will not be effective for the current bus off recovery. | Bus-Off Recovery. The CANFD_CTL1.BOFFREC bit defines how the CANFD module recovers from bus off state. When the CANFD_CTL1.BOFFREC bit is enabled, automatic recovering from the bus off state occurs according to the CAN Specification 2.0B. When the CANFD_CTL1.BOFFREC bit is disabled, automatic recovering from the bus off state is disabled and the CANFD module remains in the bus off state until the CANFD_CTL1.BOFFREC bit is enabled by software. If this occurs before 128 se- quences of 11 recessive bits are detected on the CAN bus, then the bus off recovery happens as if the CANFD_CTL1.BOFFREC bit was never disabled. If the bit is ena- bled after 128 sequences of 11 recessive bits occurred, then the CANFD module will re-synchronize to the bus by waiting for 11 recessive bits before joining the bus. After being enabled, the CANFD_CTL1.BOFFREC bit can be disabled again during the bus off state, but it will be effective only the next time the CANFD module enters the bus off state. If CANFD_CTL1.BOFFREC bit was enabled when the CANFD module entered the bus off state, disabling the CANFD_CTL1.BOFFREC bit during the bus off state will not be effective for the current bus off recovery. |
| 5 (R/W)            | TSYNEN     | Timer Sync. The CANFD_CTL1.TSYNEN bit enables a mechanism that resets the free-running timer each time a message is received in message buffer 0. The timer sync feature provides a means to synchronize multiple CANFD stations with a special "SYNC" message (global network time). If the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the first available mailbox, according to the CANFD_CTL2.RFFNUM setting, is used for timer synchronization instead of MB0. The CANFD_CTL1.TSYNEN bit can only be written in freeze mode and is blocked by hardware in other modes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Timer Sync. The CANFD_CTL1.TSYNEN bit enables a mechanism that resets the free-running timer each time a message is received in message buffer 0. The timer sync feature provides a means to synchronize multiple CANFD stations with a special "SYNC" message (global network time). If the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the first available mailbox, according to the CANFD_CTL2.RFFNUM setting, is used for timer synchronization instead of MB0. The CANFD_CTL1.TSYNEN bit can only be written in freeze mode and is blocked by hardware in other modes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 5 (R/W)            | TSYNEN     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 5 (R/W)            | TSYNEN     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 16-35: CANFD\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | LBUF Lowest Buffer Transmitted First. The CANFD_CTL1.LBUF bit defines the ordering mechanism for message buffer transmission. When the CANFD_CTL1.LBUF bit is enabled, the lowest number buffer is transmit- ted first and when it is disabled the buffer with highest priority is transmitted first. When the CANFD_CTL1.LBUF bit is enabled, the CANFD_CFG.LPRIOEN bit                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | LBUF Lowest Buffer Transmitted First. The CANFD_CTL1.LBUF bit defines the ordering mechanism for message buffer transmission. When the CANFD_CTL1.LBUF bit is enabled, the lowest number buffer is transmit- ted first and when it is disabled the buffer with highest priority is transmitted first. When the CANFD_CTL1.LBUF bit is enabled, the CANFD_CFG.LPRIOEN bit                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 4 (R/W)            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 3 (R/W)            | Listen-Only Mode. The CANFD_CTL1.LOMEN bit configures the CANFD module to operate in listen- only mode. When the CANFD_CTL1.LOMEN is enabled to be in listen-only mode, transmission is disabled, all error counters in the CANFD_ECR register are frozen, and the CANFD module operates in the CAN error passive mode. Only messages acknowledged by an- other CAN station will be received. If the CANFD module detects a message that has not been acknowledged, it flags a BIT0 error without changing the receive error coun- ter ( CANFD_ECR.RXERRCNT ), as if it were trying to acknowledge the message. Lis- ten-Only mode is acknowledged by the state of the CANFD_ESR1.FLTCONF bit field indicating a passive error. There can be some delay between the listen-only mode request and acknowledge. The CANFD_CTL1.LOMEN bit can only be written in freeze mode and is blocked by hardware in other modes. | Listen-Only Mode. The CANFD_CTL1.LOMEN bit configures the CANFD module to operate in listen- only mode. When the CANFD_CTL1.LOMEN is enabled to be in listen-only mode, transmission is disabled, all error counters in the CANFD_ECR register are frozen, and the CANFD module operates in the CAN error passive mode. Only messages acknowledged by an- other CAN station will be received. If the CANFD module detects a message that has not been acknowledged, it flags a BIT0 error without changing the receive error coun- ter ( CANFD_ECR.RXERRCNT ), as if it were trying to acknowledge the message. Lis- ten-Only mode is acknowledged by the state of the CANFD_ESR1.FLTCONF bit field indicating a passive error. There can be some delay between the listen-only mode request and acknowledge. The CANFD_CTL1.LOMEN bit can only be written in freeze mode and is blocked by hardware in other modes. |
| 3 (R/W)            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 3 (R/W)            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 2:0 (R/W)          | PROPSEG Propagation Segment. The CANFD_CTL1.PROPSEG bit field defines the length of the propagation seg- ment in the bit time. The valid programmable values are 0-7. The CANFD_CTL1.PROPSEG bit can only be written in freeze mode and is blocked by hardware in other modes. Propagation Segment Time = (PROPSEG + 1) x Time-Quanta.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | PROPSEG Propagation Segment. The CANFD_CTL1.PROPSEG bit field defines the length of the propagation seg- ment in the bit time. The valid programmable values are 0-7. The CANFD_CTL1.PROPSEG bit can only be written in freeze mode and is blocked by hardware in other modes. Propagation Segment Time = (PROPSEG + 1) x Time-Quanta.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

## Pretended Networking Control1 Register

The CANFD\_PN\_CTL1 register contains control bits for pretended networking mode filtering selection. Configure this register with the filter criteria to receive wakeup messages. Bits other than CANFD\_PN\_CTL1.WTOFMSK and CANFD\_PN\_CTL1.WUMFMSK can only be written in freeze mode.

Figure 16-18: CANFD\_PN\_CTL1 Register Diagram

<!-- image -->

Table 16-36: CANFD\_PN\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | WTOFMSK    | Wake Up by Timeout Flag Mask. The CANFD_PN_CTL1.WTOFMSK bit masks the generation of a wakeup event ori- ginated by a timeout: | Wake Up by Timeout Flag Mask. The CANFD_PN_CTL1.WTOFMSK bit masks the generation of a wakeup event ori- ginated by a timeout: |
|                    |            | 0                                                                                                                             | Timeout Event Disabled                                                                                                        |
| 16 (R/W)           | WUMFMSK    | Wake Up by Match Flag Mask. The CANFD_PN_CTL1.WUMFMSK bit masks the generation of a wakeup event ori-                         | Wake Up by Match Flag Mask. The CANFD_PN_CTL1.WUMFMSK bit masks the generation of a wakeup event ori-                         |
|                    |            | 0                                                                                                                             | Wakeup match disabled                                                                                                         |
|                    |            | 1                                                                                                                             | Wakeup match enabled                                                                                                          |

Table 16-36: CANFD\_PN\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | MATCHCNT   | Number of Messages Matching Same Filtering Criteria. The CANFD_PN_CTL1.MATCHCNT bit field defines the number of times a given message must match the predefined filtering criteria for ID and/or PL before generat- ing a wakeup event. This quantity is configured in the 1 to 255 range. For example, if the received message must match the predefined filtering criteria for ID and/or PL once before generating a wakeup event, the CANFD_PN_CTL1.MATCHCNT bit field value is 0x01; if it must | Number of Messages Matching Same Filtering Criteria. The CANFD_PN_CTL1.MATCHCNT bit field defines the number of times a given message must match the predefined filtering criteria for ID and/or PL before generat- ing a wakeup event. This quantity is configured in the 1 to 255 range. For example, if the received message must match the predefined filtering criteria for ID and/or PL once before generating a wakeup event, the CANFD_PN_CTL1.MATCHCNT bit field value is 0x01; if it must |
| 5:4 (R/W)          | PLFSEL     | Payload Filtering Selection. The CANFD_PN_CTL1.PLFSEL bit field selects the level of payload filtering to be applied when the CANFD module is under pretended networking mode. Filtering does not accept remote messages (RTR=1) when payload filtering is active.                                                                                                                                                                                                                                  | Payload Filtering Selection. The CANFD_PN_CTL1.PLFSEL bit field selects the level of payload filtering to be applied when the CANFD module is under pretended networking mode. Filtering does not accept remote messages (RTR=1) when payload filtering is active.                                                                                                                                                                                                                                  |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Match the payload contents against an exact target value                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Match if a payload value is less than or equal to a specif- ic target value                                                                                                                                                                                                                                                                                                                                                                                                                         |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Match if a payload value is inside a range, greater than or equal to a specified lower limit and smaller than or equal a specified upper limit                                                                                                                                                                                                                                                                                                                                                      |
| 3:2 (R/W)          | IDFSEL     | ID Filtering Selection. The CANFD_PN_CTL1.IDFSEL bit field selects the level of ID filtering to be ap- plied when the CANFD module is under pretended networking mode. In ID filtering, if the CANFD_FLTR_ID2_IDMSK.IDE and CANFD_FLTR_ID2_IDMSK.RTR bits are set, the IDE and RTR bits are also con- sidered part of the reception filter.                                                                                                                                                         | ID Filtering Selection. The CANFD_PN_CTL1.IDFSEL bit field selects the level of ID filtering to be ap- plied when the CANFD module is under pretended networking mode. In ID filtering, if the CANFD_FLTR_ID2_IDMSK.IDE and CANFD_FLTR_ID2_IDMSK.RTR bits are set, the IDE and RTR bits are also con- sidered part of the reception filter.                                                                                                                                                         |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Match the ID contents against an exact target value                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Match if the ID value is greater than or equal to a spe- cific target value                                                                                                                                                                                                                                                                                                                                                                                                                         |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Match if the ID value is less than or equal to a specific target value                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Match if the ID value is inside a range, greater than or equal to a specified lower limit and smaller than or equal a specified upper limit                                                                                                                                                                                                                                                                                                                                                         |

Table 16-36: CANFD\_PN\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                         | Description/Enumeration                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1:0 (R/W)          | FCSEL      | Filtering Combination Selection. The CANFD_PN_CTL1.FCSEL bit field selects elects the filtering criteria to be ap- plied when the CANFD module is in pretended networking mode. | Filtering Combination Selection. The CANFD_PN_CTL1.FCSEL bit field selects elects the filtering criteria to be ap- plied when the CANFD module is in pretended networking mode. |
|                    |            | 0                                                                                                                                                                               | Message ID filtering only                                                                                                                                                       |
|                    |            | 1                                                                                                                                                                               | Message ID filtering and payload filtering                                                                                                                                      |
|                    |            | 2                                                                                                                                                                               | Message ID filtering occurring a specified number of times                                                                                                                      |
|                    |            | 3                                                                                                                                                                               | Message ID filtering and payload filtering a specified number of times                                                                                                          |

## Control 2 Register

The CANFD\_CTL2 register complements CANFD\_CTL1 , providing control bits for memory write accesses in freeze mode, for extending FIFO filter quantity, and for adjusting the operation of internal CANFD processes like matching and arbitration.

The contents of the CANFD\_CTL2 register are not affected by soft reset.

Figure 16-19: CANFD\_CTL2 Register Diagram

<!-- image -->

Table 16-37: CANFD\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | ERRMSKF    | Error Interrupt Mask Fast Frames. The CANFD_CTL2.ERRMSKF bit is the error interrupt mask for errors detected in the data phase of fast CAN FD frames. It provides a mask for the CANFD_ESR1.ERRINTF interrupt. When the CANFD_CTL2.ERRMSKF bit is set, the error interrupt is enabled. When the bit is cleared the error interrupt is disabled. |

Table 16-37: CANFD\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|--------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | BOFFDONEIMSK | Bus Off Done Interrupt Mask. The CANFD_CTL2.BOFFDONEIMSK bit provides a mask for the bus off done inter- rupt in the CANFD_ESR1 register. When the CANFD_CTL2.BOFFDONEIMSK bit is set, the bus off done interrupt is enabled. When the bit is cleared the bus off done interrupt is disabled.                                                                                                                                                                                                    |
| 29 (R/W)           | ECRWREN      | Error-Correction Configuration Register Write Enable. The CANFD_CTL2.ECRWREN bit enables the MECR register to be updated. Setting the CANFD_CTL2.ECRWREN bit is set, enables updates. Clearing the bit disables up- dates. The CANFD_CTL2.ECRWREN bit is automatically set to zero if the protocol descri- bed in the "Detection and Correction of Memory Errors" section of the CANFD chapter is not followed.                                                                                  |
| 28 (R/W)           | WRMFRZEN     | Write-Access To Memory In Freeze Mode. The CANFD_CTL2.WRMFRZEN bit enables unrestricted write access to the CANFD memory in freeze mode. Clear the CANFD_CTL2.WRMFRZEN bit to maintain the write access restrictions. Set the CANFD_CTL2.WRMFRZEN bit to enable unrestrict- ed write access to the CANFD memory. The CANFD_CTL2.WRMFRZEN bit can only be written in freeze mode and has no effect out of freeze mode. The CANFD_CFG.RFEN bit must not be set during CANFD memory initialization. |

Table 16-37: CANFD\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:24 (R/W)        | RFFNUM     | Number Of Rx FIFO Filters. The CANFD_CTL2.RFFNUM bit field defines the number of Rx FIFO filters. For more details, see the Number of Rx FIFO Filters table in the CANFD chapter. The maximum selectable number of filters is a function of configuration parameter NUMBER_OF_MB. Do not program the CANFD_CTL2.RFFNUM bit field with values that make the number of message buffers occupied by the Rx FIFO and ID Fil- ter exceed the number of mailboxes present, defined by the CANFD_CFG.MAXMB bit field. Each group of eight filters occupies a memory space equivalent to two message buffers, which means that as more filters are implemented fewer mailboxes are available. Con- sidering that the Rx FIFO occupies the memory space originally reserved for MB0-5, program the CANFD_CTL2.RFFNUM bit with a value corresponding to a number of filters not greater than the number of available memory words. Calculate this as fol- lows: (SETUP_MB - 6) x 4 where SETUP_MB is the lower value between parameter NUMBER_OF_MB and register field CANFD_CFG.MAXMB . The number of remaining mailboxes available is (SETUP_MB - 8) - (RFFN x 2) If the number of Rx FIFO filters programmed through the CANFD_CTL2.RFFNUM bit field exceeds the SETUP_MB value (memory space available), the exceeding ones are not functional. Note that the number of the last remaining available mailbox is defined by the least value between (NUMBER_OF_MB - 1) and the CANFD_CFG.MAXMB field. If the individual Rx mask registers are not enabled, all Rx FIFO filters are affected by the Rx FIFO global mask. The CANFD_CTL2.RFFNUM bit field can only be written in freeze mode as it is |
| 23:19 (R/W)        | TXASDLY    | Tx Arbitration Start Delay. The CANFD_CTL2.TXASDLY bit field indicates how many CAN bits the Tx arbi- tration process start point can be delayed from the first bit of CRC field on the CAN bus. The CANFD_CTL2.TXASDLY bit field can only be written in freeze mode and it is blocked by hardware in other modes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 18 (R/W)           | MRPRIO     | Mailboxes Reception Priority. The CANFD_CTL2.MRPRIO bit defines the mailboxes reception priority. If the CANFD_CTL2.MRPRIO bit is set, the matching process starts from the mail- boxes and if no match occurs the matching continues on the Rx FIFO. If the bit is cleared, matching starts from Rx FIFO and continues on mailboxes. The CANFD_CTL2.MRPRIO bit can only be written only in freeze mode and it is blocked by hardware in other modes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 16-37: CANFD\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | RRS        | Remote Request Storing. If the CANFD_CTL2.RRS bit is set, a remote request frame is submitted to a match- ing process and stored in the corresponding message buffer in the same fashion of a data frame. No automatic remote response frame is generated. If the CANFD_CTL2.RRS bit is cleared, the remote request frame is submitted to a matching process and an automatic remote response frame is generated if a message buffer with CODE=0b1010 is found with the same ID. The CANFD_CTL2.RRS bit can only be written in freeze mode and it is blocked by hardware in other modes.                                                                                                                                                                            |
| 16 (R/W)           | EACEN      | Entire Frame Arbitration Field Comparison Enable For Rx Mailboxes. The CANFD_CTL2.EACEN bit controls the comparison of IDE and RTR bits within Rx mailboxes filters with their corresponding bits in the incoming frame by the match- ing process. The CANFD_CTL2.EACEN bit does not affect matching for the Rx FIFO. When the CANFD_CTL2.EACEN bit is set it enables the comparison of both the IDE and RTR bits of an Rx mailbox filter with the corresponding bits of the incoming frame. Mask bits do apply. When the CANFD_CTL2.EACEN bit is cleared the IDE bit of the Rx mailbox filter is always compared and RTR is never compared, regardless of mask bits. EACEN can be only written only in freeze mode and it is blocked by hardware in oth- er modes. |
| 15 (R/W)           | TMRSRC_SEL | Timer Source. The CANFD_CTL2.TMRSRC_SEL bit selects the time tick source used for incre- menting the free running timer counter. When the CANFD_CTL2.TMRSRC_SEL bit is set, the free running timer is clocked by an external time tick. The period can be either adjusted to be equal to the baud rate on the CAN bus or a different value as required. When the CANFD_CTL2.TMRSRC_SEL bit is cleared, the free running timer is clocked by the CAN bit clock, which defines the baud rate on the CAN bus. The CANFD_CTL2.TMRSRC_SEL bit can only be written in freeze mode.                                                                                                                                                                                        |
| 14 (R/W)           | PREXEN     | Protocol Exception Enable. The CANFD_CTL2.PREXEN bit enables the protocol exception feature. The CANFD_CTL2.PREXEN bit can only be written in freeze mode. 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 16-37: CANFD\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | ISOFDEN    | ISO CAN FD Enable. The CANFD_CTL2.ISOFDEN bit controls ISO CAN FD compliant operation. Setting the bit enables ISO CAN FD compliant operation by enabling the following features, which are part of the ISO 11898 standard and not included in the original (Bosch) CAN FD protocol specification: The count of variable stuff bits inserted from the start of frame bit to the last bit of data field. Also, the modulo 8 count of variable stuff bits plus the respective parity bit (even parity calculated over the 3-bit modulo 8 count) are combined as the 4-bit stuff count field and inserted before the CRC sequence field. CRC calculation extends beyond the end of data field and takes the stuff count field bits into account. Clearing the CANFD_CTL2.ISOFDEN bit disables ISO CAN FD specific features (non-ISO CAN FD operation). The CANFD_CTL2.ISOFDEN bit is only writable in freeze mode. |
| 11 (R/W)           | EDFLTDIS   | Edge Filter Disable. The CANFD_CTL2.EDFLTDIS bit controls the edge filter used during the bus inte- gration state. When the edge filter is enabled, two consecutive nominal time quanta with dominant bus state are required to detect an edge that causes synchronization. When synchroni- zation occurs, the counting of the sequence of eleven consecutive recessive bits is re- started. The edge filter prevents the dominant pulses that are shorter than a nominal bit time (present during the data phase of an FD frame) from being mistaken for an idle condition. The CANFD_CTL2.EDFLTDIS bit is only writable in freeze mode. 0 Enable Disable                                                                                                                                                                                                                                                      |
| 11 (R/W)           | EDFLTDIS   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 11 (R/W)           | EDFLTDIS   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

## Pretended Networking Control2 Register

The CANFD\_PN\_CTL2 register contains configuration bits for the timeout value under pretended networking mode. Only write to this register in freeze mode.

Figure 16-20: CANFD\_PN\_CTL2 Register Diagram

<!-- image -->

Table 16-38: CANFD\_PN\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | MATCHTO    | Timeout for No Message Matching the Filtering Criteria. The CANFD_PN_CTL2.MATCHTO bit field defines a timeout value that generates a wakeup event when the CANFD_CFG.PNETEN bit is enabled. If the timeout counter reaches the target value when the CANFD module is in pretended networking mode, a wakeup event is generated. The timeout limit value is configured from 1 to 65535 to control an internal 16-bit up-count timer to produce a trigger upon reaching this configured value. The internal timer is incremented based on periodic time ticks, with a period 64 times the CAN bit time unit. When the CANFD_PN_CTL2.MATCHTO bit field is 0x0000, the timeout is disabled. |

## Error Counter Register

The CANFD\_ECR register has four 8-bit fields reflecting the value of the CANFD error counters:

- Transmit error counter ( CANFD\_ECR.TXERRCNT field)
- Receive error counter ( CANFD\_ECR.RXERRCNT field)
- Transmit error counter for errors detected in the data phase of CAN FD messages with the BRS bit set ( CANFD\_ECR.TXERRCNTF field)
- Receive error counter for errors detected in the data phase of CAN FD messages with the BRS bit set ( CANFD\_ECR.RXERRCNTF field)

The TXERRCNT and RXERRCNT counters take into account all errors in both CAN FD and non-FD message formats. The TXERRCNTF and RXERRCNT\_FAST counters only count the errors that occur in the data phase of CAN FD frames with the BRS bit set.

The fault confinement state ( CANFD\_ESR1.FLTCONF ) is only updated based on the TXERRCNT and RXERRCNT counters. The TXERRCNT and RXERRCNT counters can only be written in freeze mode. The TXERRCNTF and RXERRCNT\_FAST counters are read-only except in freeze mode, where the processor can write a zero value. The rules for incrementing and decrementing these counters are described in the CAN protocol and are completely implemented in the CANFD module.

The basic rules for the CANFD bus state transitions are as follows:

- If the value of TXERRCNT or RXERRCNT increases to be greater than or equal to 128, the CANFD\_ESR1.FLTCONF field updates to reflect the error passive state.
- If the CANFD state is error passive, and either TXERRCNT or RXERRCNT decrements to a value less than or equal to 127 while the other already satisfies this condition, the CANFD\_ESR1.FLTCONF field updates to reflect the error active state.
- If the value of TXERRCNT increases to be greater than 255, the CANFD\_ESR1.FLTCONF field updates to reflect the bus off state and an interrupt may be issued. The value of TXERRCNT is then reset to zero.
- If the CANFD is in the bus off state, then TXERRCNT is cascaded together with another internal counter to count the 128 occurrences of 11 consecutive recessive bits on the bus. Hence, TXERRCNT is reset to zero and counts in a manner where the internal counter counts 11 such bits and then wraps around while incrementing TXERRCNT. When TXERRCNT reaches the value of 128, the CANFD\_ESR1.FLTCONF field is updated to be error active and both error counters are reset to zero. At any instance of dominant bit following a stream of less than 11 consecutive recessive bits, the internal counter resets itself to zero without affecting the TXERRCNT value. The TXERRCNTF counter is frozen during bus off.
- If during system start-up, only one node is operating, then its TXERRCNT increases in each message it is trying to transmit, as a result of acknowledge errors (indicated by the CANFD\_ESR1.ACKERR bit). After the transition to the error passive state, TXERRCNT is no longer incremented by acknowledge errors. Therefore, the device never goes to the bus off state.

- If RXERRCNT increases to a value greater than 127, it is not incremented further, even if more errors are detected while being a receiver. At the next successful message reception, the counter is set to a value between 119 and 127 to resume to the error active state.
- The TXERRCNTF and RXERRCNT\_FAST error counters values increment and decrement based on errors detected only in the data phase of CAN FD frames with the BRS bit set, following the same increment and decrement rules as the TXERRCNT and RXERRCNT counters. These counters do not wrap around and get stuck at their maximum value (255). They stop counting and keep their values frozen while the CANFD is in the bus off state. They are reset when the CANFD leaves the bus off state and restart counting once the CANFD resumes to the error active state.

· When the CANFD module is in pretended networking mode, RXERRCNT and RXERRCNT\_FAST keep counting errors and error flags are stored. TXERRCNT and TXERRCNTF preserve their values and do not change since no transmission occurs under pretended networking mode. Error counters and error flags that changed values while in pretended networking mode are updated in the CANFD\_ECR and CANFD\_ESR1 registers when the CANFD module resumes normal mode. The FAST error flags in the CANFD\_ESR1 register are not set if the CANFD is in pretended networking mode.

Figure 16-21: CANFD\_ECR Register Diagram

<!-- image -->

Table 16-39: CANFD\_ECR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | RXERRCNTF  | Receive Error Counter for Fast Bits. The CANFD_ECR.RXERRCNTF bit field is for errors detected in the data phase of received CAN FD messages with the BRS bit set. The CANFD_ECR.RXERRCNTF bit field is read-only except in freeze mode. In freeze mode the processor can only write a 8-bit zero value.  |
| 23:16 (R/W)        | TXERRCNTF  | Transmit Error Counter for Fast Bits. The CANFD_ECR.TXERRCNTF bit field is for errors detected in the data phase of received CAN FD messages with the BRS bit set. The CANFD_ECR.TXERRCNTF bit field is read-only except in freeze mode. In freeze mode the processor can only write a 8-bit zero value. |

Table 16-39: CANFD\_ECR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | RXERRCNT   | Receive Error Counter. The CANFD_ECR.RXERRCNT bit field is for all errors detected in received messages. The CANFD_ECR.RXERRCNT bit field is read-only except in freeze mode, where the processor can write it.       |
| 7:0 (R/W)          | TXERRCNT   | Transmit Error Counter. The CANFD_ECR.TXERRCNT bit field is for all errors detected in transmitted mes- sages. The CANFD_ECR.TXERRCNT bit field is read-only except in freeze mode, where the processor can write it. |

## Error Injection Address Register

The CANFD\_ERR\_IADDR register holds the address where a memory read error is injected.

Figure 16-22: CANFD\_ERR\_IADDR Register Diagram

<!-- image -->

Table 16-40: CANFD\_ERR\_IADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:0 (R/W)         | VALUE      | Error Injection Address. The CANFD_ERR_IADDR.VALUE bit field defines the physical RAM address where a memory read error is to be injected. See the Error Injection Address Mapping table in the CANFD chapter for more details. Note that the two least significant bits are always read as zero. |

## Error Injection Data Pattern Register

The CANFD\_ERR\_IDP register holds the error pattern injected in the data word that is read from memory.

Figure 16-23: CANFD\_ERR\_IDP Register Diagram

<!-- image -->

Table 16-41: CANFD\_ERR\_IDP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DFLIP      | Data Flip Pattern. The CANFD_ERR_IDP.DFLIP bit field determines the data flip pattern. Bits set to one in the flip pattern cause the corresponding data bit in the word read from memory to invert. |

## Error Injection Parity Pattern Register

The CANFD\_ERR\_IPP register holds the error pattern that is injected in the parity bits read from memory along with data word. Bits set to one in the flip pattern cause the corresponding parity bit, in the word read from memory, to invert.

Figure 16-24: CANFD\_ERR\_IPP Register Diagram

<!-- image -->

Table 16-42: CANFD\_ERR\_IPP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 28:24 (R/W)        | PFLIP3     | Parity Flip Pattern Byte 3. The CANFD_ERR_IPP.PFLIP3 bit field is for the parity flip pattern for byte 3 (most significant).  |
| 20:16 (R/W)        | PFLIP2     | Parity Flip Pattern Byte 2. The CANFD_ERR_IPP.PFLIP2 bit field is for the parity flip pattern for byte 2.                     |
| 12:8 (R/W)         | PFLIP1     | Parity Flip Pattern Byte 1. The CANFD_ERR_IPP.PFLIP1 bit field is for the parity flip pattern for byte 1.                     |
| 4:0 (R/W)          | PFLIP0     | Parity Flip Pattern Byte 0. The CANFD_ERR_IPP.PFLIP0 bit field is for the parity flip pattern for byte 0 (least significant). |

## Error Status Register

The CANFD\_ERR\_STAT register holds the status bits of the error correction and detection operations. Clear these flags by writing them with a one. Writing a zero to the flags has no effect.

Figure 16-25: CANFD\_ERR\_STAT Register Diagram

<!-- image -->

Table 16-43: CANFD\_ERR\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W1C)         | HNCEI      | Host Access With Non-Correctable Error Interrupt Flag. The CANFD_ERR_STAT.HNCEI bit indicates if a non-correctable error is detected in a host access. If the CANFD_ERR_STAT.HNCEI bit is set, a non-correctable error was detected in a memory read initiated by the host. A bus transfer error is asserted for that access. If the CANFD_MEC.HNCIMSK bit is set, the interrupt is asserted. If the CANFD_ERR_STAT.HNCEI bit is cleared, no non-correctable errors were de- tected in host accesses so far. |
|                    |            | 0 No Non-Correctable Error Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 1 Non-Correctable Error Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 16-43: CANFD\_ERR\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W1C)         | INCEI      | Internal CAN Access With Non-Correctable Error Interrupt Flag. The CANFD_ERR_STAT.INCEI bit indicates if a non-correctable error is detected in a CAN access. If the CANFD_ERR_STAT.INCEI bit is set, a non-correctable error was detected in a memory read initiated by a CANFD internal process. A bus transfer error is asserted for that access. If the CANFD_MEC.INCEMSK bit is set, the interrupt is asserted. If the CANFD_ERR_STAT.INCEI bit is cleared, no non-correctable errors were de- tected in CAN accesses so far. | Internal CAN Access With Non-Correctable Error Interrupt Flag. The CANFD_ERR_STAT.INCEI bit indicates if a non-correctable error is detected in a CAN access. If the CANFD_ERR_STAT.INCEI bit is set, a non-correctable error was detected in a memory read initiated by a CANFD internal process. A bus transfer error is asserted for that access. If the CANFD_MEC.INCEMSK bit is set, the interrupt is asserted. If the CANFD_ERR_STAT.INCEI bit is cleared, no non-correctable errors were de- tected in CAN accesses so far. |
| 18 (R/W1C)         | INCEI      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | No Non-Correctable Error Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 18 (R/W1C)         | INCEI      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Non-Correctable Error Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 16 (R/W1C)         | CEI        | Correctable Error Interrupt Flag. The CANFD_ERR_STAT.CEI bit indicates if a correctable error is detected. If the CANFD_ERR_STAT.CEI bit is set, a correctable error was detected in a mem- ory read. If the CANFD_MEC.CEIMSK bit is set, the interrupt is asserted. If the CANFD_ERR_STAT.CEI bit is cleared, no correctable errors were detected so far.                                                                                                                                                                         | Correctable Error Interrupt Flag. The CANFD_ERR_STAT.CEI bit indicates if a correctable error is detected. If the CANFD_ERR_STAT.CEI bit is set, a correctable error was detected in a mem- ory read. If the CANFD_MEC.CEIMSK bit is set, the interrupt is asserted. If the CANFD_ERR_STAT.CEI bit is cleared, no correctable errors were detected so far.                                                                                                                                                                         |
| 16 (R/W1C)         | CEI        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | No Correctable Error Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 16 (R/W1C)         | CEI        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Correctable Error Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 3 (R/W1C)          | HNCEIOV    | Host Access With Non-Correctable Error Interrupt Overrun Flag. The CANFD_ERR_STAT.HNCEIOV bit indicates if an overrun on non-correctable error is detected in a host access. If the CANFD_ERR_STAT.HNCEIOV bit is set, a non-correctable error was detected in a memory read initiated by the host when the CANFD_ERR_STAT.HNCEI bit is set. No interrupt is associated with this flag. If the CANFD_ERR_STAT.HNCEIOV bit is cleared, no overrun on non-correctable errors were detected in a host access.                         | Host Access With Non-Correctable Error Interrupt Overrun Flag. The CANFD_ERR_STAT.HNCEIOV bit indicates if an overrun on non-correctable error is detected in a host access. If the CANFD_ERR_STAT.HNCEIOV bit is set, a non-correctable error was detected in a memory read initiated by the host when the CANFD_ERR_STAT.HNCEI bit is set. No interrupt is associated with this flag. If the CANFD_ERR_STAT.HNCEIOV bit is cleared, no overrun on non-correctable errors were detected in a host access.                         |
| 3 (R/W1C)          | HNCEIOV    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | No Overrun Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 3 (R/W1C)          | HNCEIOV    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Overrun Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 16-43: CANFD\_ERR\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | INCEIOV    | Internal CAN Access With Non-Correctable Error Interrupt Overrun Flag. The CANFD_ERR_STAT.INCEIOV bit indicates if an overrun on non-correctable error is detected in a CAN access. If the CANFD_ERR_STAT.INCEIOV bit is set, a non-correctable error was detected in a memory read initiated by a CANFD internal process when the CANFD_ERR_STAT.INCEI bit is set. No interrupt is associated with this flag. If the CANFD_ERR_STAT.INCEIOV bit is cleared, no overrun on non-correctable errors were detected in a CAN access. | Internal CAN Access With Non-Correctable Error Interrupt Overrun Flag. The CANFD_ERR_STAT.INCEIOV bit indicates if an overrun on non-correctable error is detected in a CAN access. If the CANFD_ERR_STAT.INCEIOV bit is set, a non-correctable error was detected in a memory read initiated by a CANFD internal process when the CANFD_ERR_STAT.INCEI bit is set. No interrupt is associated with this flag. If the CANFD_ERR_STAT.INCEIOV bit is cleared, no overrun on non-correctable errors were detected in a CAN access. |
| 0                  | CEIOV      | 1 Overrun Detected Correctable Error Interrupt Overrun Flag.                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | 1 Overrun Detected Correctable Error Interrupt Overrun Flag.                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 0                  | CEIOV      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | No Overrun Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 0                  | CEIOV      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Overrun Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

## Error and Status 1 Register

The CANFD\_ESR1 register reports various error conditions detected in the reception and transmission of a CAN frame, some general status of the device, and is the source of some interrupts to the processor.

The reported error conditions are:

- CANFD\_ESR1.B1ERR , CANFD\_ESR1.B0ERR , CANFD\_ESR1.ACKERR , CANFD\_ESR1.CRCERR , CANFD\_ESR1.FRMERR , and CANFD\_ESR1.STFERR , for errors detected in CAN frames of any format.
- CANFD\_ESR1.B1ERRF , CANFD\_ESR1.B0ERRF , CANFD\_ESR1.CRCERRF , CANFD\_ESR1.FRMERRF , and CANFD\_ESR1.STFERRF for errors detected in the data phase of CAN FD frames with the BRS bit set.

An error detected in a single CAN frame is reported by one or more error flags. Error reporting is cumulative in case additional error events occur in subsequent frames before the processor reads the CANFD\_ESR1 register.

CANFD\_ESR1.TXWRN , CANFD\_ESR1.RXWRN , CANFD\_ESR1.IDLE , CANFD\_ESR1.TXINPROG , CANFD\_ESR1.FLTCONF , CANFD\_ESR1.RXINPROG , and CANFD\_ESR1.SYNC are status bits.

CANFD\_ESR1.BOFFINT , CANFD\_ESR1.BOFFDONEINT , CANFD\_ESR1.ERRINT ,

CANFD\_ESR1.ERRINTF , CANFD\_ESR1.WAKINT , CANFD\_ESR1.TXWRNINT , and

CANFD\_ESR1.RXWRNINT are interrupt bits. When servicing interrupt requests generated by these bits use the following procedure:

- Read the CANFD\_ESR1 register to capture all error condition and status bits. This action clears the respective bits set since the last read access.
- Write 1 to clear the interrupt bit that triggered the interrupt request.
- Write 1 to clear the CANFD\_ESR1.ERROVR bit if it is set.

Starting from all error flags cleared, a first error event sets either the CANFD\_ESR1.ERRINT or the CANFD\_ESR1.ERRINTF bits (provided the corresponding mask bit is asserted). If other error events in subsequent frames occur before the processor services the interrupt request, the CANFD\_ESR1.ERROVR bit is set to indicate that errors from different frames have accumulated.

Figure 16-26: CANFD\_ESR1 Register Diagram

<!-- image -->

Table 16-44: CANFD\_ESR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | B1ERRF     | Bit 1 Error BRS. The CANFD_ESR1.B1ERRF bit indicates when an inconsistency occurs between the transmitted and the received bit in the data phase of CAN FD frames with the BRS bit set. When the CANFD_ESR1.B1ERRF bit is enabled, at least one bit sent as recessive is received as dominant. When the bit is disabled, there is no such occurrence.                                                        |
| 30 (R/NW)          | B0ERRF     | 1 Enable Bit 0 Error BRS. The CANFD_ESR1.B0ERRF bit indicates when an inconsistency occurs between the transmitted and the received bit in the data phase of CAN FD frames with the BRS bit set. When the CANFD_ESR1.B0ERRF bit is enabled, at least one bit sent as recessive is received as dominant. When the bit is disabled, there is no such occurrence. 0 Disable                                     |
| 28 (R/NW)          | CRCERRF    | 1 Enable Cyclic Redundancy Check Error BRS. The CANFD_ESR1.CRCERRF bit indicates if a CRC error is detected by the receiver node in the CRC field of CAN FD frames with the BRS bit set. The calculated CRC is different from the received CRC. When the CANFD_ESR1.CRCERRF bit is enabled, a CRC error occurred since the last read of the register. When the bit is disabled, there is no such occurrence. |
| 27 (R/NW)          | FRMERRF    | 0 Disable 1 Enable Form Error BRS. The CANFD_ESR1.FRMERRF bit indicates that a form error is detected by the re-                                                                                                                                                                                                                                                                                             |

Table 16-44: CANFD\_ESR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/NW)          | STFERRF    | Stuffing Error BRS. The CANFD_ESR1.STFERRF bit indicates that a stuffing error is detected in the data phase of CAN FD frames with the BRS bit set. When the CANFD_ESR1.STFERRF bit is enabled, a stuffing error occurred since the last read of the register. When the bit is disabled, there is no such occurrence.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 21 (R/W1C)         | ERROVR     | Error Overrun. The CANFD_ESR1.ERROVR bit indicates that an error condition occurred when any error flag is already set. When the CANFD_ESR1.ERROVR bit is enabled an overrun has occurred. When it is disabled, an overrun has not occurred. Clear the CANFD_ESR1.ERROVR bit by writing a one to it. 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 20 (R/W1C)         | ERRINTF    | 1 Enable Error Interrupt BRS. The CANFD_ESR1.ERRINTF bit indicates an error interrupt is detected in the data phase of CAN FD frames with the BRS bit set. When the CANFD_ESR1.ERRINTF bit is enabled it indicates setting the of any er- ror bit detected in the data phase of CAN FD frames with the BRS bit set. When it is disabled, there is no such occurrence. The CANFD_ESR1.ERRINTF bit indicates that at least one of the error bits detect- ed in the data phase of CAN FD frames with the BRS bit set ( CANFD_ESR1.B1ERRF , CANFD_ESR1.B0ERRF , CANFD_ESR1.CRCERRF , CANFD_ESR1.FRMERRF , or CANFD_ESR1.STFERRF ) is set. If the CANFD_CTL2.ERRMSKF is set, an interrupt is generated to the processor. Clear the CANFD_ESR1.ERRINTF bit by writing a one to it. Writing a zero has no effect. 0 Disable |

Table 16-44: CANFD\_ESR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W1C)         | BOFFDONEINT | Bus Off Done Interrupt. When the CANFD_ESR1.BOFFDONEINT bit is enabled it indicates the CANFD module has completed the bus off process. When it is disabled, there is no such occur- rence. The CANFD_ESR1.BOFFDONEINT bit is set when the Tx Error Counter ( CANFD_ECR.TXERRCNT ) finishes counting 128 occurrences of 11 consecutive re- cessive bits on the CAN bus and is ready to leave bus off mode. If the CANFD_CTL2.BOFFDONEIMSK is set, an interrupt is generated to the processor. Clear the CANFD_ESR1.BOFFDONEINT bit by writing a one to it. Writing a zero has no effect. | Bus Off Done Interrupt. When the CANFD_ESR1.BOFFDONEINT bit is enabled it indicates the CANFD module has completed the bus off process. When it is disabled, there is no such occur- rence. The CANFD_ESR1.BOFFDONEINT bit is set when the Tx Error Counter ( CANFD_ECR.TXERRCNT ) finishes counting 128 occurrences of 11 consecutive re- cessive bits on the CAN bus and is ready to leave bus off mode. If the CANFD_CTL2.BOFFDONEIMSK is set, an interrupt is generated to the processor. Clear the CANFD_ESR1.BOFFDONEINT bit by writing a one to it. Writing a zero has no effect. |
| 19 (R/W1C)         | BOFFDONEINT | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 18 (R/NW)          | SYNC        | CAN Synchronization Status. The CANFD_ESR1.SYNC bit is a read-only flag that indicates whether the CANFD module is synchronized to the CAN bus and able to participate in the communication process. It is set and cleared by the CANFD module. When the CANFD_ESR1.SYNC bit is enabled, the CANFD module is synchronized to the CAN bus. When it is disabled, the CANFD module is not synchronized to the CAN bus.                                                                                                                                                                      | CAN Synchronization Status. The CANFD_ESR1.SYNC bit is a read-only flag that indicates whether the CANFD module is synchronized to the CAN bus and able to participate in the communication process. It is set and cleared by the CANFD module. When the CANFD_ESR1.SYNC bit is enabled, the CANFD module is synchronized to the CAN bus. When it is disabled, the CANFD module is not synchronized to the CAN bus.                                                                                                                                                                      |
| 18 (R/NW)          | SYNC        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 18 (R/NW)          | SYNC        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 16-44: CANFD\_ESR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | TXWRNINT   | Tx Warning Interrupt Flag. When the CANFD_ESR1.TXWRNINT bit is enabled, the Tx error counter transi- tioned from less than 96 to greater than or equal to 96. When the bit is disabled, there is no such occurrence. If the CANFD_CFG.WRNEN bit is enabled, the CANFD_ESR1.TXWRNINT bit is set when the CANFD_ESR1.TXWRN bit transitions from 0 to 1, meaning that the Tx error counter reached 96. If the CANFD_CTL1.TWRNMSK bit is set, an interrupt is generated to the processor. The CANFD_ESR1.TXWRNINT bit is cleared by writing a one to it. Writing a zero has no effect. When the CANFD_CFG.WRNEN bit is disabled, the CANFD_ESR1.TXWRNINT flag is masked. The processor must clear the CANFD_ESR1.TXWRNINT flag before disabling the CANFD_CFG.WRNEN bit or the CANFD_ESR1.TXWRNINT bit will be set when the CANFD_CFG.WRNEN bit is enabled again. The CANFD_ESR1.TXWRNINT bit is not generated during the bus off state and is not updated during freeze mode. When the CANFD module returns to normal mode from pretended networking mode, the CANFD_ESR1.TXWRNINT bit does not up-                      | Tx Warning Interrupt Flag. When the CANFD_ESR1.TXWRNINT bit is enabled, the Tx error counter transi- tioned from less than 96 to greater than or equal to 96. When the bit is disabled, there is no such occurrence. If the CANFD_CFG.WRNEN bit is enabled, the CANFD_ESR1.TXWRNINT bit is set when the CANFD_ESR1.TXWRN bit transitions from 0 to 1, meaning that the Tx error counter reached 96. If the CANFD_CTL1.TWRNMSK bit is set, an interrupt is generated to the processor. The CANFD_ESR1.TXWRNINT bit is cleared by writing a one to it. Writing a zero has no effect. When the CANFD_CFG.WRNEN bit is disabled, the CANFD_ESR1.TXWRNINT flag is masked. The processor must clear the CANFD_ESR1.TXWRNINT flag before disabling the CANFD_CFG.WRNEN bit or the CANFD_ESR1.TXWRNINT bit will be set when the CANFD_CFG.WRNEN bit is enabled again. The CANFD_ESR1.TXWRNINT bit is not generated during the bus off state and is not updated during freeze mode. When the CANFD module returns to normal mode from pretended networking mode, the CANFD_ESR1.TXWRNINT bit does not up-                      |
| 17 (R/W1C)         | TXWRNINT   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 16 (R/W1C)         | RXWRNINT   | 1 Enable Rx Warning Interrupt Flag. When the CANFD_ESR1.RXWRNINT bit is enabled, it indicates that the Rx error counter transitioned from less than 96 to greater than or equal to 96. When the bit is disabled, there is no such occurrence. If the CANFD_CFG.WRNEN bit is enabled, the CANFD_ESR1.RXWRNINT bit is set when the CANFD_ESR1.RXWRN bit transitions from 0 to 1, meaning that the Rx error counter reached 96. If the CANFD_CTL1.RWRNMSK bit is set, an interrupt is generated to the processor. The CANFD_ESR1.RXWRNINT bit is cleared by writing a one to it. Writing a zero has no effect. When the CANFD_CFG.WRNEN bit is disabled, the CANFD_ESR1.RXWRNINT flag is masked. The processor must clear the CANFD_ESR1.RXWRNINT flag before disabling the CANFD_CFG.WRNEN bit or the CANFD_ESR1.RXWRNINT bit will be set when the CANFD_CFG.WRNEN bit is enabled again. The CANFD_ESR1.RXWRNINT bit is not updated during freeze mode. When the CANFD module returns to normal mode from pretended networking mode, the CANFD_ESR1.RXWRNINT bit does not update to reflect the Rx error counter state. | 1 Enable Rx Warning Interrupt Flag. When the CANFD_ESR1.RXWRNINT bit is enabled, it indicates that the Rx error counter transitioned from less than 96 to greater than or equal to 96. When the bit is disabled, there is no such occurrence. If the CANFD_CFG.WRNEN bit is enabled, the CANFD_ESR1.RXWRNINT bit is set when the CANFD_ESR1.RXWRN bit transitions from 0 to 1, meaning that the Rx error counter reached 96. If the CANFD_CTL1.RWRNMSK bit is set, an interrupt is generated to the processor. The CANFD_ESR1.RXWRNINT bit is cleared by writing a one to it. Writing a zero has no effect. When the CANFD_CFG.WRNEN bit is disabled, the CANFD_ESR1.RXWRNINT flag is masked. The processor must clear the CANFD_ESR1.RXWRNINT flag before disabling the CANFD_CFG.WRNEN bit or the CANFD_ESR1.RXWRNINT bit will be set when the CANFD_CFG.WRNEN bit is enabled again. The CANFD_ESR1.RXWRNINT bit is not updated during freeze mode. When the CANFD module returns to normal mode from pretended networking mode, the CANFD_ESR1.RXWRNINT bit does not update to reflect the Rx error counter state. |
| 16 (R/W1C)         | RXWRNINT   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 16 (R/W1C)         | RXWRNINT   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 16-44: CANFD\_ESR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | B1ERR      | Bit 1 Error. The CANFD_ESR1.B1ERR bit indicates when an inconsistency occurs between the transmitted and the received bit in a non-CAN FD message or in the arbitration or data phase of a CAN FD message. When the CANFD_ESR1.B1ERR bit is enabled, at least one bit sent as recessive is received as dominant. When the bit is disabled, there is no such occurrence. The CANFD_ESR1.B1ERR bit updates when the CANFD module returns to nor- mal mode from pretended networking mode. Note that the CANFD_ESR1.B1ERR bit is not set by a transmitter in case of arbitra- tion field or ACK slot, or in case of a node sending a passive error flag that detects dominant bits. 0 Disable |
| 14 (R/NW)          | B0ERR      | Bit 0 Error. The CANFD_ESR1.B0ERR bit indicates when an inconsistency occurs between the transmitted and the received bit in a non-CAN FD message or in the arbitration or data phase of a CAN FD message. When the CANFD_ESR1.B0ERR bit is enabled, at least one bit sent as recessive is received as dominant. When the bit is disabled, there is no such occurrence. The CANFD_ESR1.B0ERR bit updates when the CANFD module returns to nor- mal mode from pretended networking mode. 0 Disable 1 Enable                                                                                                                                                                                 |
| 13 (R/NW)          | ACKERR     | Acknowledge Error. The CANFD_ESR1.ACKERR bit indicates an acknowledge error is detected by the transmitter node (a dominant bit detected during the ACK SLOT) When the CANFD_ESR1.ACKERR bit is enabled an ACK error occurred since the last read of this register.When the bit is disabled, there is no such occurrence. The CANFD_ESR1.ACKERR bit updates when the CANFD module returns to nor- mal mode from pretended networking mode. 0 Disable 1 Enable                                                                                                                                                                                                                              |

Table 16-44: CANFD\_ESR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/NW)          | CRCERR     | Cyclic Redundancy Check Error. The CANFD_ESR1.CRCERR bit indicates if a CRC error is detected by the receiver node in a non-FD message or in the arbitration or data phase of a frame in CAN FD format (the calculated CRC is different from the received CRC) When the CANFD_ESR1.CRCERR bit is enabled, a CRC error occurred since the last read of the register. When the bit is disabled, there is no such occurrence. The CANFD_ESR1.CRCERR bit updates when the CANFD module returns to nor- mal mode from pretended networking mode. Disable |
| 12 (R/NW)          | CRCERR     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 11 (R/NW)          | FRMERR     | Form Error. The CANFD_ESR1.FRMERR bit indicates that a form error is detected by the receiv- er node in a non-FD message or in a CAN FD message in the arbitration or data phase. A fixed-form bit field contains at least one illegal bit. When the CANFD_ESR1.FRMERR bit is enabled, a form error occurred since the last read of the register. When the bit is disabled, there is no such occurrence. The CANFD_ESR1.FRMERR bit updates when the CANFD module returns to nor- mal mode from pretended networking mode.                           |
| 11 (R/NW)          | FRMERR     | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 11 (R/NW)          | FRMERR     | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 10 (R/NW)          | STFERR     | Stuffing Error. The CANFD_ESR1.STFERR bit indicates that a stuffing error is detected by the re- ceiver node in a non-FD message or in a CAN FD message in the arbitration or data phase. When the CANFD_ESR1.STFERR bit is enabled, a stuffing error occurred since the last read of the register. When the bit is disabled, there is no such occurrence. The CANFD_ESR1.STFERR bit updates when the CANFD module returns to nor- mal mode from pretended networking mode. 0 Disable Enable                                                        |
| 10 (R/NW)          | STFERR     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 10 (R/NW)          | STFERR     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 16-44: CANFD\_ESR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/NW)           | TXWRN      | TX Error Warning. The CANFD_ESR1.TXWRN bit indicates when repetitive errors are occurring during message transmission. When the CANFD_ESR1.TXWRN bit is enabled, the CANFD_ECR.TXERRCNT val- ue is greater than or equal to 96. When it is disabled, there is no such occurrence. The CANFD_ESR1.TXWRN bit is only affected by the CANFD_ECR.TXERRCNT value. It does not update during freeze mode. 0 Disable                                                                                 |
| 8 (R/NW)           | RXWRN      | 1 Enable Rx Error Warning. The CANFD_ESR1.RXWRN bit indicates when repetitive errors are occurring during message transmission. When the CANFD_ESR1.RXWRN bit is enabled, the CANFD_ECR.TXERRCNT val- ue is greater than or equal to 96. When it is disabled, there is no such occurrence. The CANFD_ESR1.RXWRN bit is only affected by the CANFD_ECR.RXERRCNT value. It does not update during freeze mode. It updates when the CANFD returns to normal mode from pretended networking mode. |
| 7 (R/NW)           | IDLE       | 1 Enable IDLE. The CANFD_ESR1.IDLE bit indicates when the CAN bus is in the IDLE state. When the CANFD_ESR1.IDLE bit is enabled, the CAN bus is IDLE. When it is disabled, there is no such occurrence.                                                                                                                                                                                                                                                                                       |
| 6 (R/NW)           | TXINPROG   | CANFD in Transmission. The CANFD_ESR1.TXINPROG bit indicates if the CANFD module is transmitting a message. When the CANFD_ESR1.TXINPROG bit is enabled, the CANFD module is trans- mitting a message. When it is disabled, the CANFD module is not transmitting a mes- sage.                                                                                                                                                                                                                 |
| 6 (R/NW)           |            | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

Table 16-44: CANFD\_ESR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:4 (R/NW)         | FLTCONF    | Fault Confinement State. The CANFD_ESR1.FLTCONF bit field indicates the fault confinement state of the CANFD module. If the CANFD_CTL1.LOMEN bit is enabled, after some delay that depends on the CAN bit timing, the CANFD_ESR1.FLTCONF bit field will indicate the error pas- sive state. The same delay affects how the CANFD_ESR1.FLTCONF bit field reflects an update to the CANFD_ECR register by the processor. It may take up to one CAN bit time to get them coherent again. The CANFD_ESR1.FLTCONF bit field is affected by soft reset. However, if the CANFD_CTL1.LOMEN bit is enabled, the CANFD_ESR1.FLTCONF reset value lasts just one CAN bit. After this time, the CANFD_ESR1.FLTCONF bit field indi- cates the error passive state. 0 Error Active |
| 3 (R/NW)           | RXINPROG   | 3 Bus Off CANFD in Reception. The CANFD_ESR1.RXINPROG bit indicates if the CANFD module is receiving a message. When the CANFD_ESR1.RXINPROG bit is enabled the CANFD module is receiving a message and when it is disabled the CANFD module is not receiving a                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 2 (R/W1C)          | BOFFINT    | Bus Off Interrupt. The CANFD_ESR1.BOFFINT bit is enabled when the CANFD module enters the bus off state. If the corresponding mask bit (CAN_CTRL1.BOFFMSK) is set, an in- terrupt is generated to the processor. When the CANFD_ESR1.BOFFINT bit is disa- bled, there is no such occurrence. The CANFD_ESR1.BOFFINT bit is cleared by writing one to it. Writing zero has no effect. 0 Disable 1 Enable                                                                                                                                                                                                                                                                                                                                                             |

Table 16-44: CANFD\_ESR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | ERRINT     | Error Interrupt. The CANFD_ESR1.ERRINT bit indicates that at least one of the error bits ( CANFD_ESR1.B1ERR , CANFD_ESR1.B0ERR , CANFD_ESR1.CRCERR , CANFD_ESR1.FRMERR , or CANFD_ESR1.STFERR ) is set. If the corresponding mask bit ( CANFD_CTL1.ERRMSK ) is set, an interrupt is generated to the processor. The CANFD_ESR1.ERRINT bit is cleared by writing a one to it. Writing a zero has no effect. 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 0 (R/W1C)          | WAKINT     | Wake Up Interrupt. The CANFD_ESR1.WAKINT bit applies when the CANFD module is in low-power mode under a self wake up mechanism (doze or stop mode). When the CANFD_ESR1.WAKINT bit is enabled it indicates a recessive-to-dominant transition was received on the CAN bus. When a recessive-to-dominant transition is detected on the CAN bus and the CANFD_CFG.WAKMSK bit is set, an interrupt is generated to the processor. This bit is cleared by writing it to one. When the bit is disabled, there is no such occurrence. When the CANFD_CFG.SLFWAKEN bit is disabled, the CANFD_ESR1.WAKINT flag is masked. The processor must clear the CANFD_ESR1.WAKINT flag before disabling the CANFD_CFG.SLFWAKEN bit. Otherwise, he CANFD_ESR1.WAKINT is set when the CANFD_CFG.SLFWAKEN bit is set again. Writing zero has no effect. The CANFD_ESR1.WAKINT bit is cleared by writing one to it. Writing zero has no effect. 0 Disable Enable |
| 0 (R/W1C)          |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 0 (R/W1C)          |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

## Error and Status 2 Register

The CANFD\_ESR2 register reports general status information.

Figure 16-27: CANFD\_ESR2 Register Diagram

<!-- image -->

Table 16-45: CANFD\_ESR2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22:16 (R/NW)       | LPTXMB     | Lowest Priority Tx Mailbox. If the CANFD_ESR2.VPS bit is set, the CANFD_ESR2.LPTXMB bit field indicates the lowest number inactive mailbox, per the CANFD_ESR2.IMB bit. If there is no inactive mailbox, then the mailbox indicated depends on the CANFD_CTL1.LBUF bit value. If the CANFD_CTL1.LBUF bit bit is cleared, the mailbox indicated is the one that has the greatest arbitration value. If the CANFD_CTL1.LBUF bit bit is set, the mailbox indicated is the highest number active Tx mailbox. If a Tx mailbox is being transmitted, it is not considered in the CANFD_ESR2.LPTXMB bit field calculation. If the CANFD_ESR2.IMB bit is not set and a frame is transmitted successfully, the CANFD_ESR2.LPTXMB bit field is updated with the mailbox number. |

Table 16-45: CANFD\_ESR2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/NW)          | VPS        | Valid Priority Status. The CANFD_ESR2.VPS bit indicates whether the CANFD_ESR2.IMB and CANFD_ESR2.LPTXMB contents are currently valid. If the CANFD_ESR2.VPS bit is set, the contents of the CANFD_ESR2.IMB and CANFD_ESR2.LPTXMB bits are valid. If the bit is cleared, the contents of the CANFD_ESR2.IMB and CANFD_ESR2.LPTXMB bits are invalid. The CANFD_ESR2.VPS bit is set upon every complete Tx arbitration process unless the processor writes to the C/S word of a mailbox that has already been scanned (is behind Tx Arbitration Pointer during the Tx arbitration process). If there is no inac- tive mailbox and only one Tx mailbox that is being transmitted, then the CANFD_ESR2.VPS bit is not set. The CANFD_ESR2.VPS bit is cleared upon the start of every Tx arbitration process or upon a write to the C/S word of any mailbox. Note that the CANFD_ESR2.VPS bit is not affected by any processor write into the C/S of a MBthat is blocked by the abort mechanism. When the CANFD_CFG.ABORTEN bit is set, the abort code write into the C/S of an MBthat is being transmitted (pending abort), or any write attempt into a Tx MBwith the inter- rupt flag set is blocked. |
| 13 (R/NW)          | IMB        | Inactive Mailbox. If the CANFD_ESR2.VPS bit is set, the CANFD_ESR2.IMB bit indicates whether there is any inactive mailbox (CODE field is either 0b1000 or 0b0000). If the CANFD_ESR2.IMB bit is set and the CANFD_ESR2.VPS bit is set and there is at least one inactive mailbox. The CANFD_ESR2.LPTXMB bit field content is the num- ber of the first one. If the CANFD_ESR2.IMB bit is cleared and the CANFD_ESR2.VPS bit is set, the CANFD_ESR2.LPTXMB bit field is not an inac- tive mailbox. The CANFD_ESR2.IMB bit is set during arbitration, if a CANFD_ESR2.LPTXMB value is found and it is inactive. The CANFD_ESR2.IMB bit is also set if CAN_ESR2.MB is not set and a frame is transmitted successfully. The CANFD_ESR2.IMB bit is cleared at the start of arbitration. Note that if an MBis successfully transmitted and The CANFD_ESR2.IMB bit is zero (no inactive mailbox), then the CANFD_ESR2.VPS and CANFD_ESR2.IMB bits are set and the index related to the MBjust transmitted is loaded into the CANFD_ESR2.LPTXMB bit field.                                                                                                                                                |

## CANFD Bit Timing Register

The CANFD\_FD\_TIMING register stores the CAN bit timing variables for use in the data phase of CAN FD messages when the CANFD\_FD\_CTL.BRSEN bit is set, compatible with CAN FD specification. The CANFD\_FD\_TIMING.FPRESDIV , CANFD\_FD\_TIMING.FPROPSEG , CANFD\_FD\_TIMING.FPSEG1 , CANFD\_FD\_TIMING.FPSEG2 , and CANFD\_FD\_TIMING.FRJW bit fields define the time quantum duration, the number of time quanta per CAN bit, and the sample point position for the data bit rate portion of a CAN FD message with the BRS bit set.

The contents of the CANFD\_FD\_TIMING register are not affected by soft reset.

Note that the sum of values in the CANFD\_FD\_TIMING.FPROPSEG and CANFD\_FD\_TIMING.FPSEG1 bit fields must be at least two time quanta.

Figure 16-28: CANFD\_FD\_TIMING Register Diagram

<!-- image -->

Table 16-46: CANFD\_FD\_TIMING Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:20 (R/W)        | FPRESDIV   | Fast Prescaler Division Factor. The CANFD_FD_TIMING.FPRESDIV bit field defines the ratio between the PE clock frequency and the Serial Clock (Sclock) frequency. This is in the data bit rate portion of a CAN FD message with the BRS bit set. The Sclock period defines the time quantum of the CAN FD protocol for the data bit rate. Sclock frequency = PE clock frequency / (FPRESDIV + 1). To minimize errors when processing FD frames, use the same value for the CANFD_FD_TIMING.FPRESDIV and CANFD_CTL1.PRESDIV bit fields The CANFD_FD_TIMING.FPRESDIV bit field can only be written in freeze mode and is blocked by hardware in other modes. |

Table 16-46: CANFD\_FD\_TIMING Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18:16 (R/W)        | FRJW       | Fast Resync Jump Width. The CANFD_FD_TIMING.FRJW bit field defines the maximum number of time quanta that a bit time can be changed by one re-synchronization. This is in the data bit rate portion of a CAN FD message with the BRS bit set. Resync Jump Width = FSJW + 1. One time quantum is equal to the Sclock period. The CANFD_FD_TIMING.FRJW bit field can only be written in freeze mode and is blocked by hardware in other modes. |
| 14:10 (R/W)        | FPROPSEG   | Fast Propagation Segment. The CANFD_FD_TIMING.FPROPSEG bit field defines the length of the propaga- tion segment in the bit time. This is in the data bit rate portion of a CAN FD message with the BRS bit set. Propagation Segment Time = FPROPSEG x Time-Quanta One time quantum is equal to the Sclock period. The CANFD_FD_TIMING.FPROPSEG bit field can only be written in freeze mode and is blocked by hardware in other modes.      |
| 7:5 (R/W)          | FPSEG1     | Fast Phase Segment 1. The CANFD_FD_TIMING.FPSEG1 bit field defines the length of phase segment 1 in the bit time. This is in the data bit rate portion of a CAN FD message with the BRS bit set. Phase Segment 1 = (FPSEG1 + 1) x Time-Quanta One time quantum is equal to the Sclock period. FPSEG1 can be written only in Freeze mode because it is blocked by hardware in other modes.                                                    |
| 2:0 (R/W)          | FPSEG2     | Fast Phase Segment 2. The CANFD_FD_TIMING.FPSEG2 bit field defines the length of phase segment 2. This is in the data bit rate portion of a CAN FD message with the BRS bit set. Phase Segment 2 = (FPSEG2 + 1) x Time-Quanta One time quantum is equal to the Sclock period. FPSEG1 can be written only in Freeze mode because it is blocked by hardware in other modes.                                                                    |

## CANFD CRC Register

The CANFD\_FD\_CRC register provides information about the CRC of transmitted messages. The CANFD module uses different CRC polynomials for different frame formats as follows:

- The CRC\_15 polynomial is used for all frames in CAN format.
- The CRC\_17 polynomial is used for frames in CAN FD format with a data field up to sixteen bytes.
- The CRC\_21 polynomial is used for frames in CAN FD format with a data field longer than sixteen bytes.

Each polynomial shown below results in a Hamming distance of 6. The CANFD\_FD\_CRC register updates when the Tx interrupt flag is asserted.

<!-- formula-not-decoded -->

Figure 16-29: CANFD\_FD\_CRC Register Diagram

<!-- image -->

Table 16-47: CANFD\_FD\_CRC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:24 (R/W)        | MB         | CRC Mailbox Number for FD_TXCRC. The CANFD_FD_CRC.MB bit field indicates the number of the mailbox correspond- ing to the value in the FD_TXCRC field, for both FD and non-FD frames. |

Table 16-47: CANFD\_FD\_CRC Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20:0 (R/W)         | TX         | Extended Transmitted CRC value. The CANFD_FD_CRC.TX bit field contains the CRC value calculated over the most recent transmitted message. There are different CRC polynomials for different frame formats. All frames in CAN format use a 15-bit polynomial, CRC_15. CRC_17 is for frames in CAN FD format with a data field up to sixteen bytes long. The 21-bit poly- nomial, CRC_21, is for frames in CAN FD format with a data field longer than six- teen bytes. For CRC_15 and CRC_17, the six most significant bits and the four most significant bits are reported as zeros, respectively. For CRC_15, this register has the same content as the CANFD_CRC register. |

## CANFD Control Register

The CANFD\_FD\_CTL register contains control bits for CAN FD operation. It also defines the data size of message buffers allocated in different partitions of RAM (memory blocks).

When 8 bytes of payload is selected, block R0 allocates MB0 to MB31 and block R1 allocates MB32 to MB63.

When more than 8 bytes of payload is selected, the maximum number of MBs in a block is limited as described as follows:

Payload Size =&gt; Maximum Number of Message Buffers per RAM Block

8 bytes =&gt; 32

16 bytes =&gt; 21

32 bytes =&gt; 12

64 bytes =&gt; 7

Note that one memory block fits exactly 32 MBs with 8 bytes payload. For the other options of payload sizes, empty memory may exist between last MB in a block and the beginning of the next block. This empty memory corresponds to less than one MB, and must not be used.

The contents of CANFD\_FD\_CTL register are not affected by soft reset.

Figure 16-30: CANFD\_FD\_CTL Register Diagram

<!-- image -->

Table 16-48: CANFD\_FD\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | BRSEN      | Bit Rate Switch Enable. The CANFD_FD_CTL.BRSEN bit enables the effect of the bit rate switch (BRS) bit during the data phase of Tx messages. When the CANFD_FD_CTL.BRSEN bit is set, if the BRS bit in the Tx MBis recessive, the CANFD module transmits a frame with bit rate switching. When the CANFD_FD_CTL.BRSEN bit is clear, the BRS bit in the Tx MBhas no effect and the CANFD module transmits a frame with the nominal rate. The processor can write the CANFD_FD_CTL.BRSEN bit any time, however, the ef- fect only becomes active when the CAN bus is in the wait for bus idle, bus idle, bus off state, or when the current frame under reception or transmission reaches the inter- frame space. By negating the CANFD_FD_CTL.BRSEN bit, the processor can force all bits in CAN FD messages to be transmitted with a nominal bit rate, despite the value in the BRS bit of the Tx MBs. | Bit Rate Switch Enable. The CANFD_FD_CTL.BRSEN bit enables the effect of the bit rate switch (BRS) bit during the data phase of Tx messages. When the CANFD_FD_CTL.BRSEN bit is set, if the BRS bit in the Tx MBis recessive, the CANFD module transmits a frame with bit rate switching. When the CANFD_FD_CTL.BRSEN bit is clear, the BRS bit in the Tx MBhas no effect and the CANFD module transmits a frame with the nominal rate. The processor can write the CANFD_FD_CTL.BRSEN bit any time, however, the ef- fect only becomes active when the CAN bus is in the wait for bus idle, bus idle, bus off state, or when the current frame under reception or transmission reaches the inter- frame space. By negating the CANFD_FD_CTL.BRSEN bit, the processor can force all bits in CAN FD messages to be transmitted with a nominal bit rate, despite the value in the BRS bit of the Tx MBs. |
| 31 (R/W)           | BRSEN      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Nominal Rate                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 26:25 (R/W)        | MBDSIZR3   | Message Buffer Data Size for Region 3. The CANFD_FD_CTL.MBDSIZR3 field selects the data size for the region R3 of message buffers allocated in RAM. The CANFD_FD_CTL.MBDSIZR3 bit field can only be written in freeze mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Message Buffer Data Size for Region 3. The CANFD_FD_CTL.MBDSIZR3 field selects the data size for the region R3 of message buffers allocated in RAM. The CANFD_FD_CTL.MBDSIZR3 bit field can only be written in freeze mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 26:25 (R/W)        | MBDSIZR3   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 8 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 26:25 (R/W)        | MBDSIZR3   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 16 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 26:25 (R/W)        | MBDSIZR3   | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 32 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 26:25 (R/W)        | MBDSIZR3   | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 64 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 23:22 (R/W)        | MBDSIZR2   | Message Buffer Data Size for Region 2. The CANFD_FD_CTL.MBDSIZR2 field selects the data size for the region R2 of message buffers allocated in RAM. The CANFD_FD_CTL.MBDSIZR2 bit field can only be written in freeze mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Message Buffer Data Size for Region 2. The CANFD_FD_CTL.MBDSIZR2 field selects the data size for the region R2 of message buffers allocated in RAM. The CANFD_FD_CTL.MBDSIZR2 bit field can only be written in freeze mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 23:22 (R/W)        | MBDSIZR2   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 8 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 23:22 (R/W)        | MBDSIZR2   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 16 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 23:22 (R/W)        | MBDSIZR2   | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 32 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 23:22 (R/W)        | MBDSIZR2   | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 64 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 16-48: CANFD\_FD\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20:19 (R/W)        | MBDSIZR1   | Message Buffer Data Size for Region 1. The CANFD_FD_CTL.MBDSIZR1 bit field selects the data size for the region R1 of message buffers allocated in RAM. The CANFD_FD_CTL.MBDSIZR1 bit field can only be written in freeze mode.                                                                                                                                                                                                                                                                               |
| 20:19 (R/W)        | MBDSIZR1   | 0 8 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 20:19 (R/W)        | MBDSIZR1   | 1 16 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 20:19 (R/W)        | MBDSIZR1   | 2 32 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 20:19 (R/W)        | MBDSIZR1   | 3 64 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 17:16 (R/W)        | MBDSIZR0   | Message Buffer Data Size Region 0. The CANFD_FD_CTL.MBDSIZR0 bit field selects the data size for the region R0 of message buffers allocated in RAM. The CANFD_FD_CTL.MBDSIZR0 bit field can only be written in freeze mode.                                                                                                                                                                                                                                                                                   |
| 17:16 (R/W)        | MBDSIZR0   | 0 8 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 17:16 (R/W)        | MBDSIZR0   | 1 16 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 17:16 (R/W)        | MBDSIZR0   | 2 32 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 17:16 (R/W)        | MBDSIZR0   | 3 64 bytes per message buffer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 15 (R/W)           | TDCOMPEN   | Transceiver Delay Compensation Enable. The CANFD_FD_CTL.TDCOMPEN bit enables or disables the TDC feature. The CANFD_FD_CTL.TDCOMPEN bit can only be written in freeze mode. Note that when loop-back mode is enabled ( CANFD_CTL1.LBEN ), the CANFD_FD_CTL.TDCOMPEN bit must be disabled.                                                                                                                                                                                                                     |
| 15 (R/W)           | TDCOMPEN   | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 15 (R/W)           | TDCOMPEN   | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 14 (R/W1C)         | TDCOMPFAIL | Transceiver Delay Compensation Fail. The CANFD_FD_CTL.TDCOMPFAIL bit indicates when the transceiver delay com- pensation (TDC) mechanism is out of range. When the TDC is out of range it is un- able to compensate for the loop delay of the transceiver and successfully compare the delayed received bits to the transmitted ones. The CANFD_FD_CTL.TDCOMPFAIL bit is set the first time the CANFD module detects the out of range condition. To clear the CANFD_FD_CTL.TDCOMPFAIL bit, write a one to it. |
| 14 (R/W1C)         | TDCOMPFAIL | 0 Measured loop delay is in range                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 14 (R/W1C)         | TDCOMPFAIL | 1 Measured loop delay is out of range                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 16-48: CANFD\_FD\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:8 (R/W)         | TDCOMPOFF  | Transceiver Delay Compensation Offset. The CANFD_FD_CTL.TDCOMPOFF bit field contains the offset value that must be added to the loop delay of the measured transceiver. This defines the position of the delayed comparison point when bit rate switching is active. The CANFD_FD_CTL.TDCOMPOFF bit field can only be written in freeze mode. The bit field value is defined in PE clock periods and must be smaller than the CAN bit duration in the data bit rate. Do not set the CANFD_FD_CTL.TDCOMPOFF bit field to zero. |
| 5:0 (R/NW)         | TDCOMPVAL  | Transceiver Delay Compensation Value. The CANFD_FD_CTL.TDCOMPVAL bit field contains the value of the transceiver loop delay measured from the transmitted EDL to R0 transition edge to the respective received one added to the value in the CANFD_FD_CTL.TDCOMPOFF bit field. This value is an integer multiple of the PE clock period.                                                                                                                                                                                      |

## Pretended Networking DLC Filter Register

The CANFD\_FLTR\_DLC register contains the DLC inside range target values ( CANFD\_FLTR\_DLC.HI and CANFD\_FLTR\_DLC.LO ) for filtering incoming messages. The DLC range is only for payload filtering. The CANFD\_FLTR\_DLC register can only be written in freeze mode.

Note that when a fixed quantity of data bytes is required, both the CANFD\_FLTR\_DLC.HI and CANFD\_FLTR\_DLC.LO bit fields must have the same value; otherwise, a range of DLC is considered for filtering.

Figure 16-31: CANFD\_FLTR\_DLC Register Diagram

<!-- image -->

Table 16-49: CANFD\_FLTR\_DLC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/W)        | LO         | Lower Limit for Length of Data Bytes Filter. The CANFD_FLTR_DLC.LO bit field specifies the lower limit on the number of data bytes considered valid for payload comparison. The CANFD_FLTR_DLC.LO bit field is part of payload reception filter.   |
| 3:0 (R/W)          | HI         | Upper Limit for Length of Data Bytes Filter. The CANFD_FLTR_DLC.HI bit field specifies the upper limit on the number of da- ta bytes considered valid for payload comparison. The CANFD_FLTR_DLC.HI bit field is part of payload reception filter. |

## Pretended Networking ID Filter1 Register

The CANFD\_FLTR\_ID1 register contains the FLT\_ID1 target value and the IDE and RTR target values for filtering the incoming message ID. This register is for equal to, smaller than, greater than comparisons, or as the lower limit value in an ID range detection. The CANFD\_FLTR\_ID1 register is only written in freeze mode.

Figure 16-32: CANFD\_FLTR\_ID1 Register Diagram

<!-- image -->

Table 16-50: CANFD\_FLTR\_ID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | IDE        | ID Extended Filter. The CANFD_FLTR_ID1.IDE bit identifies if the frame format is standard or ex- tended. It is part of the ID reception filter. 0 Accept standard frame format 1 Accept extended frame format                                       |
| 29 (R/W)           | RTR        | Remote Transmission Request Filter. The CANFD_FLTR_ID1.RTR bit identifies if the frame is remote. It is part of the ID reception filter.                                                                                                            |
| 28:0 (R/W)         | VALUE      | ID Filter 1 for Pretended Networking Filtering. The CANFD_FLTR_ID1.VALUE bit field defines either the 29 bits of a extended frame format, considering all bits, or the 11 bits of a standard frame format, consider- ing just the 11 leftmost bits. |

## Pretended Networking ID Filter2 / IDMask Register

The CANFD\_FLTR\_ID2\_IDMSK register contains the FLT\_ID2 target value for use as the upper limit value in ID range detection. When exact ID filtering criteria is selected, the CANFD\_FLTR\_ID2\_IDMSK register is also for storing the ID mask. The CANFD\_FLTR\_ID2\_IDMSK.IDE and CANFD\_FLTR\_ID2\_IDMSK.RTR bits are for ID filtering (exact and range) as part of the ID reception filter.

The CANFD\_FLTR\_ID2\_IDMSK register can only be written in freeze mode.

Figure 16-33: CANFD\_FLTR\_ID2\_IDMSK Register Diagram

<!-- image -->

Table 16-51: CANFD\_FLTR\_ID2\_IDMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | IDE        | ID Extended Mask Bit. The CANFD_FLTR_ID2_IDMSK.IDE bit indicates whether the frame format (standard/extended) is used as part of the ID reception filter. If the CANFD_FLTR_ID2_IDMSK.IDE bit is set, the corresponding bit in the filter is checked. If it is cleared, the corresponding bit in the filter is a don't care. | ID Extended Mask Bit. The CANFD_FLTR_ID2_IDMSK.IDE bit indicates whether the frame format (standard/extended) is used as part of the ID reception filter. If the CANFD_FLTR_ID2_IDMSK.IDE bit is set, the corresponding bit in the filter is checked. If it is cleared, the corresponding bit in the filter is a don't care. |
| 30 (R/W)           | IDE        | 0                                                                                                                                                                                                                                                                                                                            | Don't Care                                                                                                                                                                                                                                                                                                                   |
| 29 (R/W)           | RTR        | Remote Transmission Request Mask Bit. The CANFD_FLTR_ID2_IDMSK.RTR bit indicates if the frame type (data/remote)                                                                                                                                                                                                             | Remote Transmission Request Mask Bit. The CANFD_FLTR_ID2_IDMSK.RTR bit indicates if the frame type (data/remote)                                                                                                                                                                                                             |
| 29 (R/W)           | RTR        | 0                                                                                                                                                                                                                                                                                                                            | Don't Care                                                                                                                                                                                                                                                                                                                   |
| 29 (R/W)           | RTR        | 1                                                                                                                                                                                                                                                                                                                            | Checked                                                                                                                                                                                                                                                                                                                      |

Table 16-51: CANFD\_FLTR\_ID2\_IDMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28:0 (R/W)         | VALUE      | ID Filter 2 / ID Mask Bits for Pretended Networking Filtering. The CANFD_FLTR_ID2_IDMSK.VALUE bit field defines the ID filter value (FLT_ID2). In extended frame format, use all 29 bits (FLT_ID2[28:0]) in the field. In standard frame format, use the upper 11 bits (FLT_ID2[28:18]) and the lower 18 bits (FLT_ID2[17:0]) have no meaning. The CANFD_FLTR_ID2_IDMSK.VALUE bit field can also be IDMASK in exact ID filtering to define the mask value. In extended frame format, use all 29 bits (ID- MASK[28:0]) in the field. In standard frame format, use the upper 11 bits (ID- MASK[28:18]) and the lower 18 bits (IDMASK[17:0]) have no meaning. |

## Mailbox Interrupt Flag 1 Register

The CANFD\_IFLG1 register defines the flags for the 32 message buffer interrupts for MB31 to MB0. It contains one interrupt flag bit per buffer. Each successful transmission or reception sets the corresponding bit in the CANFD\_IFLG1 register. If the corresponding bit in the CANFD\_IMSK1 register is set, an interrupt is generated. The interrupt flag must be cleared by writing a 1 to it. Writing a zero has no effect.

Before updating the CANFD\_CFG.MAXMB field, the processor must service the CANFD\_IFLG1 bits whose MB value is greater than the CANFD\_CFG.MAXMB to be updated; otherwise, they will remain set and be inconsistent with the number of MBs available.

There is an exception is when DMA for Rx FIFO is enabled, as described below.

## Rx FIFO:

The BUF7I to BUF5I flags are also used to represent FIFO interrupts when the Rx FIFO is enabled. When the CANFD\_CFG.RFEN bit is set and the CANFD\_CFG.DMAEN bit is cleared, the function of the 8 least significant interrupt flags changes as follows:

- BUF7I, BUF6I, and BUF5I indicate operating conditions of the FIFO.
- BUF0I is used to empty the FIFO.
- BUF4I to BUF1I are reserved.

Before setting the CANFD\_CFG.RFEN bit, the processor must service the IFLAG bits asserted in the Rx FIFO region or these IFLAG bits will mistakenly show the related MBs now belonging to the FIFO as having contents to be serviced. When the CANFD\_CFG.RFEN bit is negated, the FIFO flags must be cleared. The same care must be taken when a CANFD\_CTL2.RFFNUM value is selected extending Rx FIFO filters beyond MB7. For example, when the CANFD\_CTL2.RFFNUM value is 0x8, the MB0-23 range is occupied by Rx FIFO filters and related IFLAG bits must be cleared.

The Rx FIFO must be disabled when the CANFD\_CFG.FDEN bit is enabled.

## Rx FIFO with DMA:

When both the CANFD\_CFG.RFEN bit and the CANFD\_CFG.DMAEN bits are set (DMA feature for Rx FIFO enabled), the function of the 8 least significant interrupt flags (BUF7I-BUF0I) are changed to support the DMA operation:

- BUF7I, BUF6I, and BUF4I-BUF1I are not used.
- BUF5I indicates operating condition of FIFO.
- BUF5I does not generate a CPU interrupt, but generates a DMA request.
- BUF0I is used to empty FIFO.

The CANFD\_IMSK1 register bits in the Rx FIFO region are not considered when the CANFD\_CFG.DMAEN bit is asserted. In addition, the processor must not clear the flag BUF5I when DMA is enabled. Before setting the

CANFD\_CFG.DMAEN bit, the processor must service the IFLAGs asserted in the Rx FIFO region. When the CANFD\_CFG.DMAEN bit is cleared, the FIFO must be empty.

Figure 16-34: CANFD\_IFLG1 Register Diagram

<!-- image -->

Table 16-52: CANFD\_IFLG1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | MB31       | Message Buffer 31 Interrupt Flag. The CANFD_IFLG1.MB31 bit flags the CANFD module message buffer interrupt for MB31. When the CANFD_IFLG1.MB31 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 30 (R/W1C)         | MB30       | Message Buffer 30 Interrupt Flag. The CANFD_IFLG1.MB30 bit flags the CANFD module message buffer interrupt for MB30. When the CANFD_IFLG1.MB30 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 29 (R/W1C)         | MB29       | Message Buffer 29 Interrupt Flag. The CANFD_IFLG1.MB29 bit flags the CANFD module message buffer interrupt for MB29. When the CANFD_IFLG1.MB29 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 28 (R/W1C)         | MB28       | Message Buffer 28 Interrupt Flag. The CANFD_IFLG1.MB28 bit flags the CANFD module message buffer interrupt for MB28 When the CANFD_IFLG1.MB28 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16.  |
| 27 (R/W1C)         | MB27       | Message Buffer 27 Interrupt Flag. The CANFD_IFLG1.MB27 bit flags the CANFD module message buffer interrupt for MB27. When the CANFD_IFLG1.MB27 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |

Table 16-52: CANFD\_IFLG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W1C)         | MB26       | Message Buffer 26 Interrupt Flag. The CANFD_IFLG1.MB26 bit flags the CANFD module message buffer interrupt for MB26. When the CANFD_IFLG1.MB26 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 25 (R/W1C)         | MB25       | Message Buffer 25 Interrupt Flag. The CANFD_IFLG1.MB25 bit flags the CANFD module message buffer interrupt for MB25. When the CANFD_IFLG1.MB25 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 24 (R/W1C)         | MB24       | Message Buffer 24 Interrupt Flag. The CANFD_IFLG1.MB24 bit flags the CANFD module message buffer interrupt for MB24. When the CANFD_IFLG1.MB24 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 23 (R/W1C)         | MB23       | Message Buffer 23 Interrupt Flag. The CANFD_IFLG1.MB23 bit flags the CANFD module message buffer interrupt for MB23. When the CANFD_IFLG1.MB23 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 22 (R/W1C)         | MB22       | Message Buffer 22 Interrupt Flag. The CANFD_IFLG1.MB22 bit flags the CANFD module message buffer interrupt for MB22. When the CANFD_IFLG1.MB22 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |

Table 16-52: CANFD\_IFLG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W1C)         | MB21       | Message Buffer 21 Interrupt Flag. The CANFD_IFLG1.MB21 bit flags the CANFD module message buffer interrupt for MB21. When the CANFD_IFLG1.MB21 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 20 (R/W1C)         | MB20       | Message Buffer 20 Interrupt Flag. The CANFD_IFLG1.MB20 bit flags the CANFD module message buffer interrupt for MB20. When the CANFD_IFLG1.MB20 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 19 (R/W1C)         | MB19       | Message Buffer 19 Interrupt Flag. The CANFD_IFLG1.MB19 bit flags the CANFD module message buffer interrupt for MB19. When the CANFD_IFLG1.MB19 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 18 (R/W1C)         | MB18       | Message Buffer 18 Interrupt Flag. The CANFD_IFLG1.MB18 bit flags the CANFD module message buffer interrupt for MB18. When the CANFD_IFLG1.MB18 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 17 (R/W1C)         | MB17       | Message Buffer 17 Interrupt Flag. The CANFD_IFLG1.MB17 bit flags the CANFD module message buffer interrupt for MB17. When the CANFD_IFLG1.MB17 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |

Table 16-52: CANFD\_IFLG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W1C)         | MB16       | Message Buffer 16 Interrupt Flag. The CANFD_IFLG1.MB16 bit flags the CANFD module message buffer interrupt for MB16. When the CANFD_IFLG1.MB16 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 15 (R/W1C)         | MB15       | Message Buffer 15 Interrupt Flag. The CANFD_IFLG1.MB15 bit flags the CANFD module message buffer interrupt for MB15. When the CANFD_IFLG1.MB15 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 14 (R/W1C)         | MB14       | Message Buffer 14 Interrupt Flag. The CANFD_IFLG1.MB14 bit flags the CANFD module message buffer interrupt for MB14. When the CANFD_IFLG1.MB14 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 13 (R/W1C)         | MB13       | Message Buffer 13 Interrupt Flag. The CANFD_IFLG1.MB13 bit flags the CANFD module message buffer interrupt for MB13. When the CANFD_IFLG1.MB13 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 12 (R/W1C)         | MB12       | Message Buffer 12 Interrupt Flag. The CANFD_IFLG1.MB12 bit flags the CANFD module message buffer interrupt for MB12. When the CANFD_IFLG1.MB12 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |

Table 16-52: CANFD\_IFLG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W1C)         | MB11       | Message Buffer 11 Interrupt Flag. The CANFD_IFLG1.MB11 bit flags the CANFD module message buffer interrupt for MB11. When the CANFD_IFLG1.MB11 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 10 (R/W1C)         | MB10       | Message Buffer 10 Interrupt Flag. The CANFD_IFLG1.MB10 bit flags the CANFD module message buffer interrupt for MB10. When the CANFD_IFLG1.MB10 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16. |
| 9 (R/W1C)          | MB09       | Message Buffer 9 Interrupt Flag. The CANFD_IFLG1.MB09 bit flags the CANFD module message buffer interrupt for MB09. When the CANFD_IFLG1.MB09 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16.  |
| 8 (R/W1C)          | MB08       | Message Buffer 8 Interrupt Flag. The CANFD_IFLG1.MB08 bit flags the CANFD module message buffer interrupt for MB08. When the CANFD_IFLG1.MB08 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. Only the lower 8 bits are available if parameter NUMBER_OF_MB is 16.  |

Table 16-52: CANFD\_IFLG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | MB07       | Message Buffer 7 Interrupt Flag or Rx FIFO Overflow. The CANFD_IFLG1.MB07 bit flags the CANFD module message buffer interrupt for MB7 or Rx FIFO overflow. When the CANFD_IFLG1.MB07 bit is set and CANFD_CFG.RFEN is disabled, MB7 completed transmission or reception. When the CANFD_IFLG1.MB07 bit is set and CANFD_CFG.RFEN is enabled, there is an Rx FIFO overflow. When the CANFD_IFLG1.MB07 bit is cleared and CANFD_CFG.RFEN is disabled, the corre- sponding buffer has no occurrence of successfully completed transmission or recep- tion.When the CANFD_IFLG1.MB07 bit is cleared and CANFD_CFG.RFEN is en- abled, there is no occurrence of Rx FIFO overflow. When the CANFD_CFG.RFEN bit is cleared (Rx FIFO disabled), the CANFD_IFLG1.MB07 bit flags the interrupt for MB7. When the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the CANFD_IFLG1.MB07 bit represents the Rx FIFO overflow. In this case, the CANFD_IFLG1.MB07 bit indicates that a message was lost because the Rx FIFO is full. Note that the CANFD_IFLG1.MB07 bit is not asserted when the Rx FIFO is full and the message was captured by a mailbox. Note that the CANFD_IFLG1.MB07 bit is cleared by the CANFD module whenever the CANFD_CFG.RFEN bit is changed by a processor write.                                                                                                                                                                             |
| 6 (R/W1C)          | MB06       | Message Buffer 6 Interrupt Flag or Rx FIFO Warning. The CANFD_IFLG1.MB06 bit flags the CANFD module message buffer interrupt for MB6 or Rx FIFO warning. When the CANFD_IFLG1.MB06 bit is set and CANFD_CFG.RFEN is disabled, MB6 completed transmission or reception. When the CANFD_IFLG1.MB06 bit is set and CANFD_CFG.RFEN is enabled, the Rx FIFO is almost full. When the CANFD_IFLG1.MB06 bit is cleared and CANFD_CFG.RFEN is disabled, the corre- sponding buffer has no occurrence of successfully completed transmission or recep- tion.When the CANFD_IFLG1.MB06 bit is cleared and CANFD_CFG.RFEN is en- abled, the Rx FIFO is not almost full. When the CANFD_CFG.RFEN bit is cleared (Rx FIFO disabled), the CANFD_IFLG1.MB06 bit flags the interrupt for MB6. When the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the CANFD_IFLG1.MB06 bit represents the Rx FIFO warning. In this case, BUF6I in- dicates when the number of unread messages in the Rx FIFO increases from 4 to 5 due to the reception of a new message, meaning that the Rx FIFO is almost full. Note that if the CANFD_IFLG1.MB06 bit is cleared while the number of unread messages is greater than 4, the CANFD_IFLG1.MB06 bit is not asserted again until the number of unread messages in the Rx FIFO decreases to 4 or less. Note that the CANFD_IFLG1.MB06 bit is cleared by the CANFD module whenever the CANFD_CFG.RFEN bit is changed by a processor write. |

Table 16-52: CANFD\_IFLG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W1C)          | MB05       | Message Buffer 5 Interrupt Flag or Frames Available in Rx FIFO. The CANFD_IFLG1.MB05 bit flags the CANFD module message buffer interrupt for MB5 or Rx FIFO frames available. When the CANFD_IFLG1.MB05 bit is set and CANFD_CFG.RFEN is disabled, MB5 completed transmission or reception. When the CANFD_IFLG1.MB05 bit is set and CANFD_CFG.RFEN is enabled, the Rx FIFO has frames available. BUF5I generates a DMArequest when both the CANFD_CFG.RFEN and CANFD_CFG.DMAEN bits are set. When the CANFD_IFLG1.MB05 bit is cleared and CANFD_CFG.RFEN is disabled, the corresponding buffer has no occurrence of successfully completed transmission or reception. When the CANFD_IFLG1.MB05 bit is cleared and CANFD_CFG.RFEN is enabled, no frames are available in the Rx FIFO. When the CANFD_CFG.RFEN bit is cleared (Rx FIFO disabled), CANFD_IFLG1.MB05 bit flags the interrupt for MB5. When the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the CANFD_IFLG1.MB05 bit represents the frames available in the Rx FIFO and indi- cates that at least one frame is available to be read from the Rx FIFO. When the CANFD_CFG.DMAEN bit is set, the CANFD_IFLG1.MB05 bit generates a DMAre- quest and the processor must not clear the CANFD_IFLG1.MB05 bit by writing one to it. Note that the CANFD_IFLG1.MB05 bit is cleared by the CANFD module whenever the CANFD_CFG.RFEN bit is changed by a processor write. |
| 4 (R/W1C)          | MB04       | Message Buffer 4 Interrupt Flag or Reserved. The CANFD_IFLG1.MB04 bit flags the interrupt for MB4, when the CANFD_CFG.RFEN bit is cleared (Rx FIFO disabled). When the CANFD_IFLG1.MB04 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. When the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the CANFD_IFLG1.MB04 bit is reserved. Note that the CANFD_IFLG1.MB04 bit is cleared by the CANFD module whenever the CANFD_CFG.RFEN bit is changed by a processor write.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 16-52: CANFD\_IFLG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W1C)          | MB03       | Message Buffer 3 Interrupt Flag or Reserved. The CANFD_IFLG1.MB03 bit flags the interrupt for MB3, when the CANFD_CFG.RFEN bit is cleared (Rx FIFO disabled). When the CANFD_IFLG1.MB03 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. When the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the CANFD_IFLG1.MB03 bit is reserved. Note that the CANFD_IFLG1.MB03 bit is cleared by the CANFD module whenever the CANFD_CFG.RFEN bit is changed by a processor write. |
| 2 (R/W1C)          | MB02       | Message Buffer 2 Interrupt Flag or Reserved. The CANFD_IFLG1.MB02 bit flags the interrupt for MB2, when the CANFD_CFG.RFEN bit is cleared (Rx FIFO disabled). When the CANFD_IFLG1.MB02 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. When the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the CANFD_IFLG1.MB02 bit is reserved. Note that the CANFD_IFLG1.MB02 bit is cleared by the CANFD module whenever the CANFD_CFG.RFEN bit is changed by a processor write. |
| 1 (R/W1C)          | MB01       | Message Buffer 1 Interrupt Flag or Reserved. The CANFD_IFLG1.MB01 bit flags the interrupt for MB1, when the CANFD_CFG.RFEN bit is cleared (Rx FIFO disabled). When the CANFD_IFLG1.MB01 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. When the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the CANFD_IFLG1.MB01 bit is reserved. Note that the CANFD_IFLG1.MB01 bit is cleared by the CANFD module whenever the CANFD_CFG.RFEN bit is changed by a processor write. |

Table 16-52: CANFD\_IFLG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W1C)          | MB00       | Message Buffer 0 Interrupt Flag or Clear FIFO. The CANFD_IFLG1.MB00 bit flags the interrupt for MB0, when the CANFD_CFG.RFEN bit is cleared (Rx FIFO disabled). When the CANFD_IFLG1.MB00 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. When the CANFD_CFG.RFEN bit is set (Rx FIFO enabled), the CANFD_IFLG1.MB00 bit triggers the clear FIFO operation. This operation empties FIFO contents. Before performing this operation, the processor must service all FIFO related IFLAGs. When the CANFD_CFG.DMAEN bit is set (DMA is enabled), this operation also clears the CANFD_IFLG1.MB05 flag and consequently aborts the DMArequest. The clear FIFO operation occurs when the processor writes a one to the CANFD_IFLG1.MB00 bit, which is only allowed in freeze mode and is blocked by hardware in other conditions. |

## Mailbox Interrupt Flag 2 Register

The CANFD\_IFLG2 register defines the flags for the 32 message buffer interrupts for MB63 to MB32. It contains one interrupt flag bit per buffer. Each successful transmission or reception sets the respective bit in the CANFD\_IFLG2 bit. If the corresponding bit is set, an interrupt is generated. The interrupt flag must be cleared by writing 1 to it. Writing 0 has no effect.

Before updating the CANFD\_CFG.MAXMB field, the processor must service the CANFD\_IFLG2 bits whose MB value is greater than the CANFD\_CFG.MAXMB to be updated; otherwise, they will remain set and be inconsistent with the number of MBs available.

Figure 16-35: CANFD\_IFLG2 Register Diagram

<!-- image -->

Table 16-53: CANFD\_IFLG2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | MB63       | Message Buffer 63 Interrupt Flag. The CANFD_IFLG2.MB63 bit flags the CANFD module message buffer interrupt for MB63. When the CANFD_IFLG2.MB63 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 30 (R/W1C)         | MB62       | Message Buffer 62 Interrupt Flag. The CANFD_IFLG2.MB62 bit flags the CANFD module message buffer interrupt for MB62. When the CANFD_IFLG2.MB62 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 29 (R/W1C)         | MB61       | Message Buffer 61 Interrupt Flag. The CANFD_IFLG2.MB61 bit flags the CANFD module message buffer interrupt for MB61. When the CANFD_IFLG2.MB61 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 28 (R/W1C)         | MB60       | Message Buffer 60 Interrupt Flag. The CANFD_IFLG2.MB60 bit flags the CANFD module message buffer interrupt for MB60. When the CANFD_IFLG2.MB60 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 27 (R/W1C)         | MB59       | Message Buffer 59 Interrupt Flag. The CANFD_IFLG2.MB59 bit flags the CANFD module message buffer interrupt for MB59. When the CANFD_IFLG2.MB59 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 26 (R/W1C)         | MB58       | Message Buffer 58 Interrupt Flag. The CANFD_IFLG2.MB58 bit flags the CANFD module message buffer interrupt for MB58. When the CANFD_IFLG2.MB58 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |

Table 16-53: CANFD\_IFLG2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W1C)         | MB57       | Message Buffer 57 Interrupt Flag. The CANFD_IFLG2.MB57 bit flags the CANFD module message buffer interrupt for MB57. When the CANFD_IFLG2.MB57 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 24 (R/W1C)         | MB56       | Message Buffer 56 Interrupt Flag. The CANFD_IFLG2.MB56 bit flags the CANFD module message buffer interrupt for MB56. When the CANFD_IFLG2.MB56 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 23 (R/W1C)         | MB55       | Message Buffer 55 Interrupt Flag. The CANFD_IFLG2.MB55 bit flags the CANFD module message buffer interrupt for MB55. When the CANFD_IFLG2.MB55 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 22 (R/W1C)         | MB54       | Message Buffer 54 Interrupt Flag. The CANFD_IFLG2.MB54 bit flags the CANFD module message buffer interrupt for MB54. When the CANFD_IFLG2.MB54 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 21 (R/W1C)         | MB53       | Message Buffer 53 Interrupt Flag. The CANFD_IFLG2.MB53 bit flags the CANFD module message buffer interrupt for MB53. When the CANFD_IFLG2.MB53 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 20 (R/W1C)         | MB52       | Message Buffer 52 Interrupt Flag. The CANFD_IFLG2.MB52 bit flags the CANFD module message buffer interrupt for MB52. When the CANFD_IFLG2.MB52 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |

Table 16-53: CANFD\_IFLG2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W1C)         | MB51       | Message Buffer 51 Interrupt Flag. The CANFD_IFLG2.MB51 bit flags the CANFD module message buffer interrupt for MB51. When the CANFD_IFLG2.MB51 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 18 (R/W1C)         | MB50       | Message Buffer 50 Interrupt Flag. The CANFD_IFLG2.MB50 bit flags the CANFD module message buffer interrupt for MB50. When the CANFD_IFLG2.MB50 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 17 (R/W1C)         | MB49       | Message Buffer 49 Interrupt Flag. The CANFD_IFLG2.MB49 bit flags the CANFD module message buffer interrupt for MB49. When the CANFD_IFLG2.MB49 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 16 (R/W1C)         | MB48       | Message Buffer 48 Interrupt Flag. The CANFD_IFLG2.MB48 bit flags the CANFD module message buffer interrupt for MB48. When the CANFD_IFLG2.MB48 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 15 (R/W1C)         | MB47       | Message Buffer 47 Interrupt Flag. The CANFD_IFLG2.MB47 bit flags the CANFD module message buffer interrupt for MB47. When the CANFD_IFLG2.MB47 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 14 (R/W1C)         | MB46       | Message Buffer 46 Interrupt Flag. The CANFD_IFLG2.MB46 bit flags the CANFD module message buffer interrupt for MB46. When the CANFD_IFLG2.MB46 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |

Table 16-53: CANFD\_IFLG2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1C)         | MB45       | Message Buffer 45 Interrupt Flag. The CANFD_IFLG2.MB45 bit flags the CANFD module message buffer interrupt for MB45. When the CANFD_IFLG2.MB45 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 12 (R/W1C)         | MB44       | Message Buffer 44 Interrupt Flag. The CANFD_IFLG2.MB44 bit flags the CANFD module message buffer interrupt for MB44. When the CANFD_IFLG2.MB44 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 11 (R/W1C)         | MB43       | Message Buffer 43 Interrupt Flag. The CANFD_IFLG2.MB43 bit flags the CANFD module message buffer interrupt for MB43. When the CANFD_IFLG2.MB43 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 10 (R/W1C)         | MB42       | Message Buffer 42 Interrupt Flag. The CANFD_IFLG2.MB42 bit flags the CANFD module message buffer interrupt for MB42. When the CANFD_IFLG2.MB42 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 9 (R/W1C)          | MB41       | Message Buffer 41 Interrupt Flag. The CANFD_IFLG2.MB41 bit flags the CANFD module message buffer interrupt for MB41. When the CANFD_IFLG2.MB41 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 8 (R/W1C)          | MB40       | Message Buffer 40 Interrupt Flag. The CANFD_IFLG2.MB40 bit flags the CANFD module message buffer interrupt for MB40. When the CANFD_IFLG2.MB40 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |

Table 16-53: CANFD\_IFLG2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | MB39       | Message Buffer 39 Interrupt Flag. The CANFD_IFLG2.MB39 bit flags the CANFD module message buffer interrupt for MB39. When the CANFD_IFLG2.MB39 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 6 (R/W1C)          | MB38       | Message Buffer 38 Interrupt Flag. The CANFD_IFLG2.MB38 bit flags the CANFD module message buffer interrupt for MB38. When the CANFD_IFLG2.MB38 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 5 (R/W1C)          | MB37       | Message Buffer 37 Interrupt Flag. The CANFD_IFLG2.MB37 bit flags the CANFD module message buffer interrupt for MB37. When the CANFD_IFLG2.MB37 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 4 (R/W1C)          | MB36       | Message Buffer 36 Interrupt Flag. The CANFD_IFLG2.MB36 bit flags the CANFD module message buffer interrupt for MB36. When the CANFD_IFLG2.MB36 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 3 (R/W1C)          | MB35       | Message Buffer 35 Interrupt Flag. The CANFD_IFLG2.MB35 bit flags the CANFD module message buffer interrupt for MB35. When the CANFD_IFLG2.MB35 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 2 (R/W1C)          | MB34       | Message Buffer 34 Interrupt Flag. The CANFD_IFLG2.MB34 bit flags the CANFD module message buffer interrupt for MB34. When the CANFD_IFLG2.MB34 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |

Table 16-53: CANFD\_IFLG2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | MB33       | Message Buffer 33 Interrupt Flag. The CANFD_IFLG2.MB33 bit flags the CANFD module message buffer interrupt for MB33. When the CANFD_IFLG2.MB33 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |
| 0 (R/W1C)          | MB32       | Message Buffer 32 Interrupt Flag. The CANFD_IFLG2.MB32 bit flags the CANFD module message buffer interrupt for MB32. When the CANFD_IFLG2.MB32 bit is set, the corresponding buffer successfully completed transmission or reception. When the bit is cleared, the corresponding buffer has no occurrence of successfully completed transmission or reception. |

## Mailbox Interrupt Mask 1 Register

The CANFD\_IMSK1 register allows any number of the 32 message buffer interrupts to be enabled or disabled for MB31 to MB. The CANFD\_IMSK1 register contains one interrupt mask bit per buffer, enabling the CPU to determine which buffer generates an interrupt after a successful transmission or reception (as indicated by the corresponding bit in the register).

Figure 16-36: CANFD\_IMSK1 Register Diagram

<!-- image -->

Table 16-54: CANFD\_IMSK1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | MB31       | Message Buffer 31 Interrupt Mask. The CANFD_IMSK1.MB31 bit is the CANFD module message buffer interrupt for MB31. When the CANFD_IMSK1.MB31 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 30 (R/W)           | MB30       | Message Buffer 30 Interrupt Mask. The CANFD_IMSK1.MB30 bit is the CANFD module message buffer interrupt for MB30. When the CANFD_IMSK1.MB30 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 29 (R/W)           | MB29       | Message Buffer 29 Interrupt Mask. The CANFD_IMSK1.MB29 bit is the CANFD module message buffer interrupt for MB29. When the CANFD_IMSK1.MB29 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 28 (R/W)           | MB28       | Message Buffer 28 Interrupt Mask. The CANFD_IMSK1.MB28 bit is the CANFD module message buffer interrupt for MB28. When the CANFD_IMSK1.MB28 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 27 (R/W)           | MB27       | Message Buffer 27 Interrupt Mask. The CANFD_IMSK1.MB27 bit is the CANFD module message buffer interrupt for MB27. When the CANFD_IMSK1.MB27 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |

Table 16-54: CANFD\_IMSK1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | MB26       | Message Buffer 26 Interrupt Mask. The CANFD_IMSK1.MB26 bit is the CANFD module message buffer interrupt for MB26. When the CANFD_IMSK1.MB26 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 25 (R/W)           | MB25       | Message Buffer 25 Interrupt Mask. The CANFD_IMSK1.MB25 bit is the CANFD module message buffer interrupt for MB25. When the CANFD_IMSK1.MB25 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 24 (R/W)           | MB24       | Message Buffer 24 Interrupt Mask. The CANFD_IMSK1.MB24 bit is the CANFD module message buffer interrupt for MB24. When the CANFD_IMSK1.MB24 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 23 (R/W)           | MB23       | Message Buffer 23 Interrupt Mask. The CANFD_IMSK1.MB23 bit is the CANFD module message buffer interrupt for MB23. When the CANFD_IMSK1.MB23 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 22 (R/W)           | MB22       | Message Buffer 22 Interrupt Mask. The CANFD_IMSK1.MB22 bit is the CANFD module message buffer interrupt for MB22. When the CANFD_IMSK1.MB22 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |

Table 16-54: CANFD\_IMSK1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | MB21       | Message Buffer 21 Interrupt Mask. The CANFD_IMSK1.MB21 bit is the CANFD module message buffer interrupt for MB21. When the CANFD_IMSK1.MB21 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 20 (R/W)           | MB20       | Message Buffer 20 Interrupt Mask. The CANFD_IMSK1.MB20 bit is the CANFD module message buffer interrupt for MB20. When the CANFD_IMSK1.MB20 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 19 (R/W)           | MB19       | Message Buffer 19 Interrupt Mask. The CANFD_IMSK1.MB19 bit is the CANFD module message buffer interrupt for MB19. When the CANFD_IMSK1.MB19 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 18 (R/W)           | MB18       | Message Buffer 18 Interrupt Mask. The CANFD_IMSK1.MB18 bit is the CANFD module message buffer interrupt for MB18. When the CANFD_IMSK1.MB18 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 17 (R/W)           | MB17       | Message Buffer 17 Interrupt Mask. The CANFD_IMSK1.MB17 bit is the CANFD module message buffer interrupt for MB17. When the CANFD_IMSK1.MB17 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |

Table 16-54: CANFD\_IMSK1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | MB16       | Message Buffer 16 Interrupt Mask. The CANFD_IMSK1.MB16 bit is the CANFD module message buffer interrupt for MB16. When the CANFD_IMSK1.MB16 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 15 (R/W)           | MB15       | Message Buffer 15 Interrupt Mask. The CANFD_IMSK1.MB15 bit is the CANFD module message buffer interrupt for MB15. When the CANFD_IMSK1.MB15 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 14 (R/W)           | MB14       | Message Buffer 14 Interrupt Mask. The CANFD_IMSK1.MB14 bit is the CANFD module message buffer interrupt for MB14. When the CANFD_IMSK1.MB14 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 13 (R/W)           | MB13       | Message Buffer 13 Interrupt Mask. The CANFD_IMSK1.MB13 bit is the CANFD module message buffer interrupt for MB13. When the CANFD_IMSK1.MB13 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 12 (R/W)           | MB12       | Message Buffer 12 Interrupt Mask. The CANFD_IMSK1.MB12 bit is the CANFD module message buffer interrupt for MB12. When the CANFD_IMSK1.MB12 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |

Table 16-54: CANFD\_IMSK1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | MB11       | Message Buffer 11 Interrupt Mask. The CANFD_IMSK1.MB11 bit is the CANFD module message buffer interrupt for MB11. When the CANFD_IMSK1.MB11 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 10 (R/W)           | MB10       | Message Buffer 10 Interrupt Mask. The CANFD_IMSK1.MB10 bit is the CANFD module message buffer interrupt for MB10. When the CANFD_IMSK1.MB10 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 9 (R/W)            | MB09       | Message Buffer 09 Interrupt Mask. The CANFD_IMSK1.MB09 bit is the CANFD module message buffer interrupt for MB09. When the CANFD_IMSK1.MB09 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 8 (R/W)            | MB08       | Message Buffer 08 Interrupt Mask. The CANFD_IMSK1.MB08 bit is the CANFD module message buffer interrupt for MB08. When the CANFD_IMSK1.MB08 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 7 (R/W)            | MB07       | Message Buffer 07 Interrupt Mask. The CANFD_IMSK1.MB07 bit is the CANFD module message buffer interrupt for MB07. When the CANFD_IMSK1.MB07 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |

Table 16-54: CANFD\_IMSK1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | MB06       | Message Buffer 06 Interrupt Mask. The CANFD_IMSK1.MB06 bit is the CANFD module message buffer interrupt for MB06. When the CANFD_IMSK1.MB06 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 5 (R/W)            | MB05       | Message Buffer 05 Interrupt Mask. The CANFD_IMSK1.MB05 bit is the CANFD module message buffer interrupt for MB05. When the CANFD_IMSK1.MB05 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 4 (R/W)            | MB04       | Message Buffer 04 Interrupt Mask. The CANFD_IMSK1.MB04 bit is the CANFD module message buffer interrupt for MB04. When the CANFD_IMSK1.MB04 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 3 (R/W)            | MB03       | Message Buffer 03 Interrupt Mask. The CANFD_IMSK1.MB03 bit is the CANFD module message buffer interrupt for MB03. When the CANFD_IMSK1.MB03 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 2 (R/W)            | MB02       | Message Buffer 02 Interrupt Mask. The CANFD_IMSK1.MB02 bit is the CANFD module message buffer interrupt for MB02. When the CANFD_IMSK1.MB02 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |

Table 16-54: CANFD\_IMSK1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | MB01       | Message Buffer 01 Interrupt Mask. The CANFD_IMSK1.MB01 bit is the CANFD module message buffer interrupt for MB01. When the CANFD_IMSK1.MB01 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |
| 0 (R/W)            | MB00       | Message Buffer 00 Interrupt Mask. The CANFD_IMSK1.MB00 bit is the CANFD module message buffer interrupt for MB00. When the CANFD_IMSK1.MB00 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK1 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG1 register is set. |

## Mailbox Interrupt Mask 2 Register

The CANFD\_IMSK2 register allows any number of the 32 message buffer interrupts to be enabled or disabled for MB63 to MB32. The CANFD\_IMSK2 register contains one interrupt mask bit per buffer, enabling the processor to determine which buffer generates an interrupt after a successful transmission or reception (as indicated by the corresponding bit in the register).

Figure 16-37: CANFD\_IMSK2 Register Diagram

<!-- image -->

Table 16-55: CANFD\_IMSK2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | MB63       | Message Buffer 63 Interrupt Mask. The CANFD_IMSK2.MB63 bit is the CANFD module message buffer interrupt for MB63. When the CANFD_IMSK2.MB63 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 30 (R/W)           | MB62       | Message Buffer 62 Interrupt Mask. The CANFD_IMSK2.MB62 bit is the CANFD module message buffer interrupt for MB62. When the CANFD_IMSK2.MB62 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 29 (R/W)           | MB61       | Message Buffer 61 Interrupt Mask. The CANFD_IMSK2.MB61 bit is the CANFD module message buffer interrupt for MB61. When the CANFD_IMSK2.MB61 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 28 (R/W)           | MB60       | Message Buffer 60 Interrupt Mask. The CANFD_IMSK2.MB60 bit is the CANFD module message buffer interrupt for MB60. When the CANFD_IMSK2.MB60 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 27 (R/W)           | MB59       | Message Buffer 59 Interrupt Mask. The CANFD_IMSK2.MB59 bit is the CANFD module message buffer interrupt for MB59. When the CANFD_IMSK2.MB59 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |

Table 16-55: CANFD\_IMSK2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | MB58       | Message Buffer 58 Interrupt Mask. The CANFD_IMSK2.MB58 bit is the CANFD module message buffer interrupt for MB58. When the CANFD_IMSK2.MB58 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 25 (R/W)           | MB57       | Message Buffer 57 Interrupt Mask. The CANFD_IMSK2.MB57 bit is the CANFD module message buffer interrupt for MB57. When the CANFD_IMSK2.MB57 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 24 (R/W)           | MB56       | Message Buffer 56 Interrupt Mask. The CANFD_IMSK2.MB56 bit is the CANFD module message buffer interrupt for MB56. When the CANFD_IMSK2.MB56 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 23 (R/W)           | MB55       | Message Buffer 55 Interrupt Mask. The CANFD_IMSK2.MB55 bit is the CANFD module message buffer interrupt for MB55. When the CANFD_IMSK2.MB55 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 22 (R/W)           | MB54       | Message Buffer 54 Interrupt Mask. The CANFD_IMSK2.MB54 bit is the CANFD module message buffer interrupt for MB54. When the CANFD_IMSK2.MB54 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |

Table 16-55: CANFD\_IMSK2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | MB53       | Message Buffer 53 Interrupt Mask. The CANFD_IMSK2.MB53 bit is the CANFD module message buffer interrupt for MB53. When the CANFD_IMSK2.MB53 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 20 (R/W)           | MB52       | Message Buffer 52 Interrupt Mask. The CANFD_IMSK2.MB52 bit is the CANFD module message buffer interrupt for MB52. When the CANFD_IMSK2.MB52 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 19 (R/W)           | MB51       | Message Buffer 51 Interrupt Mask. The CANFD_IMSK2.MB51 bit is the CANFD module message buffer interrupt for MB51. When the CANFD_IMSK2.MB51 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 18 (R/W)           | MB50       | Message Buffer 50 Interrupt Mask. The CANFD_IMSK2.MB50 bit is the CANFD module message buffer interrupt for MB50. When the CANFD_IMSK2.MB50 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 17 (R/W)           | MB49       | Message Buffer 49 Interrupt Mask. The CANFD_IMSK2.MB49 bit is the CANFD module message buffer interrupt for MB49. When the CANFD_IMSK2.MB49 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |

Table 16-55: CANFD\_IMSK2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | MB48       | Message Buffer 48 Interrupt Mask. The CANFD_IMSK2.MB48 bit is the CANFD module message buffer interrupt for MB48. When the CANFD_IMSK2.MB48 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 15 (R/W)           | MB47       | Message Buffer 47 Interrupt Mask. The CANFD_IMSK2.MB47 bit is the CANFD module message buffer interrupt for MB47. When the CANFD_IMSK2.MB47 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 14 (R/W)           | MB46       | Message Buffer 46 Interrupt Mask. The CANFD_IMSK2.MB46 bit is the CANFD module message buffer interrupt for MB46. When the CANFD_IMSK2.MB46 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 13 (R/W)           | MB45       | Message Buffer 45 Interrupt Mask. The CANFD_IMSK2.MB45 bit is the CANFD module message buffer interrupt for MB45. When the CANFD_IMSK2.MB45 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 12 (R/W)           | MB44       | Message Buffer 44 Interrupt Mask. The CANFD_IMSK2.MB44 bit is the CANFD module message buffer interrupt for MB44. When the CANFD_IMSK2.MB44 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |

Table 16-55: CANFD\_IMSK2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | MB43       | Message Buffer 43 Interrupt Mask. The CANFD_IMSK2.MB43 bit is the CANFD module message buffer interrupt for MB43. When the CANFD_IMSK2.MB43 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 10 (R/W)           | MB42       | Message Buffer 42 Interrupt Mask. The CANFD_IMSK2.MB42 bit is the CANFD module message buffer interrupt for MB42. When the CANFD_IMSK2.MB42 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 9 (R/W)            | MB41       | Message Buffer 41 Interrupt Mask. The CANFD_IMSK2.MB41 bit is the CANFD module message buffer interrupt for MB41. When the CANFD_IMSK2.MB41 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 8 (R/W)            | MB40       | Message Buffer 40 Interrupt Mask. The CANFD_IMSK2.MB40 bit is the CANFD module message buffer interrupt for MB40. When the CANFD_IMSK2.MB40 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 7 (R/W)            | MB39       | Message Buffer 39 Interrupt Mask. The CANFD_IMSK2.MB39 bit is the CANFD module message buffer interrupt for MB39. When the CANFD_IMSK2.MB39 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |

Table 16-55: CANFD\_IMSK2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | MB38       | Message Buffer 38 Interrupt Mask. The CANFD_IMSK2.MB38 bit is the CANFD module message buffer interrupt for MB39. When the CANFD_IMSK2.MB38 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 5 (R/W)            | MB37       | Message Buffer 37 Interrupt Mask. The CANFD_IMSK2.MB37 bit is the CANFD module message buffer interrupt for MB37. When the CANFD_IMSK2.MB37 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 4 (R/W)            | MB36       | Message Buffer 36 Interrupt Mask. The CANFD_IMSK2.MB36 bit is the CANFD module message buffer interrupt for MB36. When the CANFD_IMSK2.MB36 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 3 (R/W)            | MB35       | Message Buffer 35 Interrupt Mask. The CANFD_IMSK2.MB35 bit is the CANFD module message buffer interrupt for MB35. When the CANFD_IMSK2.MB35 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 2 (R/W)            | MB34       | Message Buffer 34 Interrupt Mask. The CANFD_IMSK2.MB34 bit is the CANFD module message buffer interrupt for MB34. When the CANFD_IMSK2.MB34 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |

Table 16-55: CANFD\_IMSK2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | MB33       | Message Buffer 33 Interrupt Mask. The CANFD_IMSK2.MB33 bit is the CANFD module message buffer interrupt for MB33. When the CANFD_IMSK2.MB33 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |
| 0 (R/W)            | MB32       | Message Buffer 32 Interrupt Mask. The CANFD_IMSK2.MB32 bit is the CANFD module message buffer interrupt for MB32. When the CANFD_IMSK2.MB32 bit is set, the corresponding buffer interrupt is en- abled. When the bit is cleared, the corresponding buffer interrupt is disabled. Note that setting or clearing a bit in the CANFD_IMSK2 register enables or disables an interrupt request if the corresponding bit in the CANFD_IFLG2 register is set. |

## Module Configuration Register

The CANFD\_CFG register configures the operation modes and settings for the CANFD module. This register defines global system configurations, such as the module operation modes and the maximum message buffer configuration.

Figure 16-38: CANFD\_CFG Register Diagram

<!-- image -->

Table 16-56: CANFD\_CFG Register Fields

| Bit No. (Access)   | Bit Name                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration   |
|--------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------|
| 31 (R/W)           | DIS Module Disable. The CANFD_CFG.DIS bit disables or enables the CANFD module. When the CANFD_CFG.DIS bit is set, the CANFD module disables the clocks to the PE and CHI submodules. This bit is not affected by a soft reset.                                                                                                                                                                                                                                                                                                                                                                                                                      |                           |
|                    | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 0                         |
|                    |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 1                         |
| 30 (R/W)           | Disable FRZ Freeze Mode Enable. The CANFD_CFG.FRZ bit specifies the CANFD module behavior when the CANFD_CFG.HALT bit is set or when there is a debug mode request at the chip level. When the CANFD_CFG.FRZ bit is set, the CANFD module enters freeze mode. Negation of the CANFD_CFG.FRZ bit causes the CANFD module to exit from freeze mode. The CANFD_CFG.FRZ bit is set by hardware when a non-correctable error is detected and the CANFD_MEC.NCERRFRZEN bit is set. Disable                                                                                                                                                                 |                           |
|                    |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 0                         |
|                    |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 1                         |
| 29 (R/W)           | Enable RFEN Rx FIFO Enable. The CANFD_CFG.RFEN bit enables the Rx FIFO. When the CANFD_CFG.RFEN bit is set, do not use MBs 0-5 for normal reception and transmission. The corresponding memory region (0x80-0xDC) is used by the FIFO engine and additional MBs (up to 32, depending on the CANFD_CTL2.RFFNUM bit setting) are used as Rx FIFO ID filter table elements. The CANFD_CFG.RFEN bit also impacts the minimum number of peripheral clocks per CAN bit. The CANFD_CFG.RFEN bit can only be written in freeze mode because it is blocked by hardware in other modes. Note that the CANFD_CFG.RFEN bit cannot be set when the CANFD_CFG.FDEN |                           |
|                    | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |                           |
|                    |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | 1 Enable                  |

Table 16-56: CANFD\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | HALT       | Freeze Mode Enable. Setting the CANFD_CFG.HALT bit puts the CANFD module into freeze mode. The processor clears the CANFD_CFG.HALT bit after initializing the message buffers, the CANFD_CTL1 register, and the CANFD_CTL2 register. The CANFD module does not receive or transmit data before the CANFD_CFG.HALT bit is cleared. The CANFD module cannot enter freeze mode while it is in a low-power mode. The CANFD_CFG.HALT bit is set by hardware when a non-correctable error is detected and the CANFD_MEC.NCERRFRZEN bit is set.                                                                                                                                                                                                                                                                                                                                                     |
| 27 (R/NW)          | NOTRDY     | CANFD Not Ready. The CANFD_CFG.NOTRDY bit is a read-only bit that indicates the CANFD module is either in module disable, doze, stop, or freeze mode. The CANFD_CFG.NOTRDY bit is negated once the CANFD module exits these modes. Soft reset does not affect the CANFD_CFG.NOTRDY bit.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 26 (R/W)           | WAKMSK     | Wakeup Interrupt Mask. The CANFD_CFG.WAKMSK bit enables the wakeup interrupt using the self wakeup mechanism.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 25 (R/W)           | SOFTRST    | Soft Reset. When the CANFD_CFG.SOFTRST bit is set, the internal state machines and some of the memory mapped registers of the CANFD module reset. For the processor to di- rectly assert a soft reset, write to the CANFD_CFG.SOFTRST bit. A soft reset can also occur when a global soft reset is requested at the chip level. Soft reset is synchronous and has to follow a request/acknowledge procedure across clock domains, so it takes time to fully propagate the effect. The CANFD_CFG.SOFTRST bit remains set while reset is pending and is automatically negated when the reset completes. Poll the CANFD_CFG.SOFTRST bit to know when the soft reset has completed. Soft reset cannot be applied while clocks are shut down in a low-power mode. Remove the CANFD module from low-power mode before applying soft reset. A soft reset does not affect the CANFD_CFG.SOFTRST bit. |

Table 16-56: CANFD\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/NW)          | FRZACK     | Freeze Mode Acknowledge. The CANFD_CFG.FRZACK bit is a read-only bit that indicates the CANFD module is in freeze mode and the prescaler is stopped. A freeze mode request is only granted after the current transmission or reception processes completes. Poll the CANFD_CFG.FRZACK bit in software to know when the CANFD module enters freeze mode. If a freeze mode request is negated, the CANFD_CFG.FRZACK bit is negated after the CANFD module prescaler is running again. If freeze mode is requested while the CANFD module is in a low-power mode, the CANFD_CFG.FRZACK bit sets only when the module exits the low-power mode. Soft reset does not affect the CANFD_CFG.FRZACK bit. Note that the CANFD_CFG.FRZACK bit is set within 178 CAN bits of the freeze mode request. It is negated within 2 CAN bits after a freeze mode request removal. 0 No Freeze Mode Prescaler is running.                                                                      | Freeze Mode Acknowledge. The CANFD_CFG.FRZACK bit is a read-only bit that indicates the CANFD module is in freeze mode and the prescaler is stopped. A freeze mode request is only granted after the current transmission or reception processes completes. Poll the CANFD_CFG.FRZACK bit in software to know when the CANFD module enters freeze mode. If a freeze mode request is negated, the CANFD_CFG.FRZACK bit is negated after the CANFD module prescaler is running again. If freeze mode is requested while the CANFD module is in a low-power mode, the CANFD_CFG.FRZACK bit sets only when the module exits the low-power mode. Soft reset does not affect the CANFD_CFG.FRZACK bit. Note that the CANFD_CFG.FRZACK bit is set within 178 CAN bits of the freeze mode request. It is negated within 2 CAN bits after a freeze mode request removal. 0 No Freeze Mode Prescaler is running.                                                                      |
| 22 (R/W)           | SLFWAKEN   | Self Wakeup Enable. The CANFD_CFG.SLFWAKEN bit enables the self wakeup feature when the CANFD module is in a low-power mode other than module disable. When the self wakeup feature is enabled, the CANFD module monitors the bus for a wakeup event (a recessive-to-dominant transition). If a wake up event is detected dur- ing doze mode, the CANFD module requests resumption of the clocks and when the CANFD_CFG.SLFWAKEN bit is enabled generates a wakeup interrupt to the process- or. If a wakeup event is detected during stop mode, the CANFD module, when the CANFD_CFG.SLFWAKEN bit is enabled, generates a wake up interrupt to the pro- cessor so that it can exit stop mode globally and the CANFD module can request to resume the clocks. When the CANFD module is in a low-power mode other than module disable, the CANFD_CFG.SLFWAKEN bit cannot be written as it is blocked by hardware. Disable self wakeup when pretended networking mode is set. | Self Wakeup Enable. The CANFD_CFG.SLFWAKEN bit enables the self wakeup feature when the CANFD module is in a low-power mode other than module disable. When the self wakeup feature is enabled, the CANFD module monitors the bus for a wakeup event (a recessive-to-dominant transition). If a wake up event is detected dur- ing doze mode, the CANFD module requests resumption of the clocks and when the CANFD_CFG.SLFWAKEN bit is enabled generates a wakeup interrupt to the process- or. If a wakeup event is detected during stop mode, the CANFD module, when the CANFD_CFG.SLFWAKEN bit is enabled, generates a wake up interrupt to the pro- cessor so that it can exit stop mode globally and the CANFD module can request to resume the clocks. When the CANFD module is in a low-power mode other than module disable, the CANFD_CFG.SLFWAKEN bit cannot be written as it is blocked by hardware. Disable self wakeup when pretended networking mode is set. |
| 22 (R/W)           |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 22 (R/W)           |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

Table 16-56: CANFD\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | WRNEN      | Warning Interrupt Enable. The CANFD_CFG.WRNEN bit enables the generation of the TWRNINT and RWRNINT flags in the CANFD_ESR1 register. If the CANFD_CFG.WRNEN bit is negated, the TWRNINT and RWRNINT flags will always be zero, independent of the values of the error counters. No warning inter- rupt is generated. The CANFD_CFG.WRNEN bit can only be written in freeze mode and is blocked by hardware in other modes. 0 Disable                                                                                                                                                                                                                                                                                                                           |
| 20 (R/NW)          | LPMACK     | 1 Enable Low Power Mode Acknowledge. The CANFD_CFG.LPMACK bit is a read-only bit that indicates that the CANFD module is in a low-power mode (module disable, doze, or stop). The module can only enter a low-power mode when all current transmission or reception processes have fin- ished. Poll the CANFD_CFG.LPMACK bit to know when the CANFD module is in low-power mode. The CANFD_CFG.LPMACK bit is not affected by soft reset. Note that the CANFD_CFG.LPMACK bit is set within 180 CAN bits of the low-pow- er mode request by the processor. It is negated within 2 CAN bits after removing the low-power mode request. When the CANFD module is in pretended networking mode, the CANFD_CFG.LPMACK bit is negated within 180 CAN bits from the re- |
| 19 (R/W)           | WSFLTREN   | 1 Enable In low-power mode Wakeup Source. The CANFD_CFG.WSFLTREN bit controls whether the integrated low-pass filter is applied to protect the Rx CAN input from spurious wakeup. When enabled, the CANFD module uses the filtered Rx input to detect recessive-to- dominant edges on the CAN bus. When disabled the CANFD module uses the unfil- tered Rx input to detect recessive-to-dominant edges on the CAN bus. The CANFD_CFG.WSFLTREN bit can only be written in freeze mode and is blocked by hardware in other modes. 0 Disable                                                                                                                                                                                                                       |

Table 16-56: CANFD\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | DOZEN      | Doze Mode Enable. The CANFD_CFG.DOZEN bit controls whether the CANFD module can enter low- power mode when doze mode is requested at the chip level. When the CANFD_CFG.DOZEN bit is set the CANFD module can enter low-power mode and when it is cleared the module cannot. The CANFD_CFG.DOZEN bit is automatically reset when the CANFD module wakes up from doze mode upon detecting activity on the CAN bus ( CANFD_CFG.SLFWAKEN enabled). 0 Disable                                                              |
| 17 (R/W)           | SRXDIS     | Self Reception Disable. The CANFD_CFG.SRXDIS bit controls whether the CANFD module can receive frames transmitted from itself. When the CANFD_CFG.SRXDIS bit is set, frames transmitted by the CANFD mod- ule are not stored in any MB, regardless of whether the MBis programmed with an ID that matches the transmitted frame. No interrupt flag or interrupt signal is generated due to the frame reception. The CANFD_CFG.SRXDIS bit can only be written in freeze mode and is blocked by hardware in other modes. |
| 16 (R/W)           | IRMQEN     | Rx Masking and Queue Enable. The CANFD_CFG.IRMQEN bit controls whether the Rx matching process is based on individual masking and queue or on a masking scheme with the CANFD_RX_MB_GMSK , CANFD_RX_14_MSK , CANFD_RX_15_MSK , and CANFD_RX_FIFO_GMSK registers. The CANFD_CFG.IRMQEN bit can only be written in freeze mode and is blocked by hardware in other modes. For backward compatibility, when CANFD_CFG.IRMQEN is disabled, reading a C/S word locks the MBeven when it is empty. 0 Disable                 |

Table 16-56: CANFD\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | DMAEN      | DMAEnable. The CANFD_CFG.DMAEN bit enables or disables the DMAfeature. The DMAfea- ture is only for use with the Rx FIFO and the CANFD_CFG.RFEN bit must be set. When both the CANFD_CFG.DMAEN and CANFD_CFG.RFEN bits are set, CANFD_IFLG1.MB05 generates the DMArequest and no Rx FIFO interrupt is generated. The CANFD_CFG.DMAEN bit can only be written in freeze mode and is blocked by hardware in other modes. 0 Disable                                                                                                                          |
| 14 (R/W)           | PNETEN     | 1 Enable Pretended Networking Enable. The CANFD_CFG.PNETEN bit enables the pretended networking mode. When pre- tended networking is enabled, the PE submodule is kept operational in doze mode and stop mode. It is able to process Rx message filtering as defined by the pretended net- working configuration registers. The CANFD_CFG.PNETEN bit can be only be written in freeze mode. 0 Disable                                                                                                                                                     |
| 13 (R/W)           | LPRIOEN    | Local Priority Enable. The CANFD_CFG.LPRIOEN bit controls whether the local priority feature is ena- bled and is provided for backward compatibility with legacy applications. Local priority expands the arbitration ID. With this expanded ID concept, the arbitra- tion process is based on the full 32-bit word, but the actual transmitted ID still has 11 bits for standard frames and 29 bits for extended frames. The CANFD_CFG.LPRIOEN bit can only be written only in freeze mode and is blocked by hardware in other modes. 0 Disable 1 Enable |

Table 16-56: CANFD\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | ABORTEN    | Abort Enable. The CANFD_CFG.ABORTEN bit enables the Tx abort mechanism. The Tx abort mechanism ensures a safe procedure for aborting a pending transmis- sion, so that no frame is sent on the CAN bus without notification. The CANFD_CFG.ABORTEN bit can only be written only in freeze mode and is blocked                                                                                                                                                                  | Abort Enable. The CANFD_CFG.ABORTEN bit enables the Tx abort mechanism. The Tx abort mechanism ensures a safe procedure for aborting a pending transmis- sion, so that no frame is sent on the CAN bus without notification. The CANFD_CFG.ABORTEN bit can only be written only in freeze mode and is blocked                                                                                                                                                                  |
| 12 (R/W)           | ABORTEN    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 11 (R/W)           | FDEN       | CANFD Enable. The CANFD_CFG.FDEN bit enables CAN with Flexible Data rate (CAN FD) opera- tion. When the CANFD_CFG.FDEN bit is enabled, the CANFD module can receive and transmit messages in both CAN FD and CAN 2.0 formats. When it is disabled, the CANFD module can only receive and transmit messages in CAN 2.0 format. The CANFD_CFG.FDEN bit can only be written in freeze mode only. Note that the CANFD_CFG.RFEN bit cannot be set if the CANFD_CFG.FDEN bit is set. | CANFD Enable. The CANFD_CFG.FDEN bit enables CAN with Flexible Data rate (CAN FD) opera- tion. When the CANFD_CFG.FDEN bit is enabled, the CANFD module can receive and transmit messages in both CAN FD and CAN 2.0 formats. When it is disabled, the CANFD module can only receive and transmit messages in CAN 2.0 format. The CANFD_CFG.FDEN bit can only be written in freeze mode only. Note that the CANFD_CFG.RFEN bit cannot be set if the CANFD_CFG.FDEN bit is set. |
| 11 (R/W)           | FDEN       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 11 (R/W)           | FDEN       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 9:8 (R/W)          | IDAM       | ID Acceptance Mode. The CANFD_CFG.IDAM bit field identifies the format of the Rx FIFO ID filter table elements (ID acceptance mode). All elements of the table are configured at the same time by the CANFD_CFG.IDAM bit field (they are all the same format). The CANFD_CFG.IDAM bit field can only be written in freeze mode and is blocked by hardware in other modes.                                                                                                      | ID Acceptance Mode. The CANFD_CFG.IDAM bit field identifies the format of the Rx FIFO ID filter table elements (ID acceptance mode). All elements of the table are configured at the same time by the CANFD_CFG.IDAM bit field (they are all the same format). The CANFD_CFG.IDAM bit field can only be written in freeze mode and is blocked by hardware in other modes.                                                                                                      |
| 9:8 (R/W)          | IDAM       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Format A. There is one full ID (standard and extended) per ID filter table element.                                                                                                                                                                                                                                                                                                                                                                                            |
| 9:8 (R/W)          | IDAM       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Format B. There are two full standard IDs or two partial 14-bit (standard and extended) IDs per ID filter table element.                                                                                                                                                                                                                                                                                                                                                       |
| 9:8 (R/W)          | IDAM       | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Format C. There are four partial 8-bit standard IDs per ID filter table element.                                                                                                                                                                                                                                                                                                                                                                                               |
| 9:8 (R/W)          | IDAM       | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Format D. All frames are rejected.                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 16-56: CANFD\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | MAXMB      | Number of the Last Message Buffer. The CANFD_CFG.MAXMB bit field defines the number of the last message buffer that will take part in the matching and arbitration processes. The reset value (0x0F) is equivalent to a 16 message buffer configuration. The CANFD_CFG.MAXMB bit field can only be written in freeze mode and is blocked by hardware in other modes. Note that the CANFD_CFG.MAXMB bit field must be programmed with a value smaller than or equal to the number of available message buffers. Additionally, the def- inition of the CANFD_CFG.MAXMB bit value must take into account the region of MBs occupied by Rx FIFO and the ID filters table space defined by the CANFD_CTL2.RFFNUM bit. The CANFD_CFG.MAXMB bit field impacts the mini- mum number of peripheral clocks per CAN bit. |

## Memory Error Control Register

The CANFD\_MEC register contains control bits for memory error detection and correction (ECC).

Note that when the CANFD\_CTL2.ECRWREN bit is disabled, writes to the CANFD\_MEC register are blocked, with the exception of the CANFD\_MEC.ECRWRDIS bit.

Figure 16-39: CANFD\_MEC Register Diagram

<!-- image -->

Table 16-57: CANFD\_MEC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | ECRWRDIS   | Error Configuration Register Write Disable. The CANFD_MEC.ECRWRDIS bit disables writes to the CANFD_MEC register. The CANFD_MEC.ECRWRDIS bit is automatically set to 1 (disabled) when the CANFD_CTL2.ECRWREN bit is enabled. See the protocol described in the Detection and Correction of Memory Errors section of the CANFD chapter. |
| 19 (R/W)           | HNCIMSK    | Host Access With Non-Correctable Errors Interrupt Mask. The CANFD_MEC.HNCIMSK bit enables the interrupt in case of non-correctable er- rors detected in memory reads issued by the host. 0 Disable                                                                                                                                      |
| 19 (R/W)           | HNCIMSK    | 1 Enable                                                                                                                                                                                                                                                                                                                                |
| 19 (R/W)           | HNCIMSK    |                                                                                                                                                                                                                                                                                                                                         |

Table 16-57: CANFD\_MEC Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | INCEMSK    | Internal CAN Access With Non-Correctable Errors Interrupt Mask. The CANFD_MEC.INCEMSK bit enables the interrupt in case of non-correctable er- rors detected in memory reads issued by the CAN internal processes.                                                                                                                                                                                                                                                                                                                                           |
| 18 (R/W)           | INCEMSK    | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 18 (R/W)           | INCEMSK    | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 16 (R/W)           | CEIMSK     | Correctable Errors Interrupt Mask. The CANFD_MEC.CEIMSK bit enables the interrupt in case of correctable errors de- tected in memory reads issued by the host or CANFD module internal processes.                                                                                                                                                                                                                                                                                                                                                            |
| 16 (R/W)           | CEIMSK     | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 16 (R/W)           | CEIMSK     | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 15 (R/W)           | HAERRIEN   | Host Access Error Injection Enable. The CANFD_MEC.HAERRIEN bit enables the injection of errors for memory reads issued by the host.                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 15 (R/W)           | HAERRIEN   | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 15 (R/W)           | HAERRIEN   | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 14 (R/W)           | IAERRIEN   | Internal CAN Access Error Injection Enable. The CANFD_MEC.IAERRIEN bit enables the injection of errors for memory reads issued by the CAN internal processes.                                                                                                                                                                                                                                                                                                                                                                                                |
| 14 (R/W)           | IAERRIEN   | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 14 (R/W)           | IAERRIEN   | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 13 (R/W)           | EXTERRIEN  | Extended Error Injection Enable. Memory accesses performed by internal CANFD processes are 64-bit accesses. The CANFD_MEC.EXTERRIEN bit extends the error injection on 32-bit memory access- es to the complementary 32-bit word using the same 32-bit error injection data and parity words. When the CANFD_MEC.EXTERRIEN bit is enabled, the error injection is applied to the 64-bit word. When the bit is disabled, error injection is only applied to the 32-bit word. See the descriptions of the CANFD_ERR_IDP and CANFD_ERR_IPP registers. 0 Disable |
| 13 (R/W)           | EXTERRIEN  | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 13 (R/W)           | EXTERRIEN  |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 16-57: CANFD\_MEC Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | RERDIS     | Error Report Disable. The CANFD_MEC.RERDIS bit disables the update of the error report registers. When the error report registers are disabled, the update of error-related flags and the generation of bus transfer errors are still active. Note that when reading the report registers, the CANFD_MEC.RERDIS bit must be set to ensure coherence on the consecutive register reads. 0 Enable                               |
| 8 (R/W)            | ECCDIS     | 1 Disable Error Correction Disable. The CANFD_MEC.ECCDIS bit disables the memory detection and correction mech- anism. In addition to disabling the error report mechanism, when the CANFD_MEC.ECCDIS bit is set, the update of the error-related flags and generation of bus transfer errors also stops. The parity bits continue to be calculated and written into memory on write transactions. 0 Enable                   |
| 7 (R/W)            | NCERRFRZEN | Non-Correctable Errors Freeze Mode. The CANFD_MEC.NCERRFRZEN bit determines the response when a non-correcta- ble error is detected in a memory read performed by CANFD module internal process- es. Enabling the CANFD_MEC.NCERRFRZEN bit puts the CANFD module in freeze mode, preventing corrupted data from being treated as valid by CANFD internal proc- esses. Disabling the bit maintains normal operation. 0 Disable |
| 7 (R/W)            | NCERRFRZEN |                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 7 (R/W)            | NCERRFRZEN |                                                                                                                                                                                                                                                                                                                                                                                                                               |

## Pretended Networking Payload Low Filter2 Register

The CANFD\_FLTR\_DATA1\_HI register contain Payload Filter 1 high order bits of the target value. This is for filtering the incoming message payload in pretended networking mode. It is for =, ≤ , or ≥ to comparisons; or as the lower limit value in payload range detection.

The CANFD\_FLTR\_DATA1\_HI register can only be written in freeze mode.

Figure 16-40: CANFD\_FLTR\_DATA1\_HI Register Diagram

<!-- image -->

Table 16-58: CANFD\_FLTR\_DATA1\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | PDB4       | Payload Filter 1 Byte 4. The CANFD_FLTR_DATA1_HI.PDB4 bit field contains the payload filter 1 high order bits for pretended networking payload filtering corresponding to data byte 4. |
| 23:16 (R/W)        | PDB5       | Payload Filter 1 Byte 5. The CANFD_FLTR_DATA1_HI.PDB5 bit field contains the payload filter 1 high order bits for pretended networking payload filtering corresponding to data byte 5. |
| 15:8 (R/W)         | PDB6       | Payload Filter 1 Byte 6. The CANFD_FLTR_DATA1_HI.PDB6 bit field contains the payload filter 1 high order bits for pretended networking payload filtering corresponding to data byte 6. |
| 7:0 (R/W)          | PDB7       | Payload Filter 1 Byte 7. The CANFD_FLTR_DATA1_HI.PDB7 bit field contains the payload filter 1 high order bits for pretended networking payload filtering corresponding to data byte 7. |

## Pretended Networking Payload Low Filter1 Register

The CANFD\_FLTR\_DATA1\_LO register contain Payload Filter 1 low order bits of the target value. This is for filtering the incoming message payload in pretended networking mode. It is for =, ≤ , or ≥ to comparisons; or as the lower limit value in payload range detection.

The CANFD\_FLTR\_DATA1\_LO register can only be written in freeze mode.

Figure 16-41: CANFD\_FLTR\_DATA1\_LO Register Diagram

<!-- image -->

Table 16-59: CANFD\_FLTR\_DATA1\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | PDB0       | Payload Filter 1 Byte 0. The CANFD_FLTR_DATA1_LO.PDB0 bit field contains the payload filter 1 low or- der bits for pretended networking payload filtering corresponding to data byte 0. |
| 23:16 (R/W)        | PDB1       | Payload Filter 1 Byte 1. The CANFD_FLTR_DATA1_LO.PDB1 bit field contains the payload filter 1 low or- der bits for pretended networking payload filtering corresponding to data byte 1. |
| 15:8 (R/W)         | PDB2       | Payload Filter 1 Byte 2. The CANFD_FLTR_DATA1_LO.PDB2 bit field contains the payload filter 1 low or- der bits for pretended networking payload filtering corresponding to data byte 2. |
| 7:0 (R/W)          | PDB3       | Payload Filter 1 Byte 3. The CANFD_FLTR_DATA1_LO.PDB3 bit field contains the payload filter 1 low or- der bits for pretended networking payload filtering corresponding to data byte 3. |

## Pretended Networking Payload High Filter2 High Order Bits / Payload High Mask Register

The CANFD\_FLTR\_DATA2\_DMSK\_HI register has two functions:

It contains the high order bits for the Payload Filter 2 value for use as the upper limit in payload range detection.

When exact payload filtering criteria is selected, the CANFD\_FLTR\_DATA2\_DMSK\_HI register is the payload mask.

Otherwise, the CANFD\_FLTR\_DATA2\_DMSK\_HI register is not used. It can only be written in freeze mode.

Figure 16-42: CANFD\_FLTR\_DATA2\_DMSK\_HI Register Diagram

<!-- image -->

Table 16-60: CANFD\_FLTR\_DATA2\_DMSK\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | PDB4       | Payload Filter 2 or Payload Mask Byte 4. The CANFD_FLTR_DATA2_DMSK_HI.PDB4 bit field contains the payload filter 2 high order bits or payload mask high order bits for pretended networking payload fil- tering corresponding to data byte 4. |
| 23:16 (R/W)        | PDB5       | Payload Filter 2 or Payload Mask Byte 5. The CANFD_FLTR_DATA2_DMSK_HI.PDB5 bit field contains the payload filter 2 high order bits or payload mask high order bits for pretended networking payload fil- tering corresponding to data byte 5. |
| 15:8 (R/W)         | PDB6       | Payload Filter 2 or Payload Mask Byte 6. The CANFD_FLTR_DATA2_DMSK_HI.PDB6 bit field contains the payload filter 2 high order bits or payload mask high order bits for pretended networking payload fil- tering corresponding to data byte 6. |
| 7:0 (R/W)          | PDB7       | Payload Filter 2 or Payload Mask Byte 7. The CANFD_FLTR_DATA2_DMSK_HI.PDB7 bit field contains the payload filter 2 high order bits or payload mask high order bits for pretended networking payload fil- tering corresponding to data byte 7. |

## Pretended Networking Payload Low Filter2 / Payload Low Mask Register

The CANFD\_FLTR\_DATA2\_DMSK\_LO register has two functions:

It contains the low order bits for the Payload Filter 2 value for use as the upper limit in payload range detection.

When exact payload filtering criteria is selected, the CANFD\_FLTR\_DATA2\_DMSK\_LO register is the payload mask.

Otherwise, the CANFD\_FLTR\_DATA2\_DMSK\_LO register is not used. It can only be written in freeze mode.

Figure 16-43: CANFD\_FLTR\_DATA2\_DMSK\_LO Register Diagram

<!-- image -->

Table 16-61: CANFD\_FLTR\_DATA2\_DMSK\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | PDB0       | Payload Filter 2 or Payload Mask Byte 0. The CANFD_FLTR_DATA2_DMSK_LO.PDB0 bit field contains the payload filter 2 low order bits or payload mask low order bits for pretended networking payload filter- ing corresponding to data byte 0. |
| 23:16 (R/W)        | PDB1       | Payload Filter 2 or Payload Mask Byte 1. The CANFD_FLTR_DATA2_DMSK_LO.PDB1 bit field contains the payload filter 2 low order bits or payload mask low order bits for pretended networking payload filter- ing corresponding to data byte 1. |
| 15:8 (R/W)         | PDB2       | Payload Filter 2 or Payload Mask Byte 2. The CANFD_FLTR_DATA2_DMSK_LO.PDB2 bit field contains the payload filter 2 low order bits or payload mask low order bits for pretended networking payload filter- ing corresponding to data byte 2. |
| 7:0 (R/W)          | PDB3       | Payload Filter 2 or Payload Mask Byte 3. The CANFD_FLTR_DATA2_DMSK_LO.PDB3 bit field contains the payload filter 2 low order bits or payload mask low order bits for pretended networking payload filter- ing corresponding to data byte 3. |

## Error Report Address Register

The CANFD\_ERR\_RADDR register reports the address of an access with an error (correctable or non-correctable) and reports the identification of the source of that access.

The address is 32-bit aligned. Non-aligned accesses, where the CANFD\_ERRADDR[1:0] bits are non-zero, are reported with the address aligned and the data in the CANFD\_ERR\_RDAT register shifted accordingly. If there is an error in an access that is larger than 32 bits (as performed by CANFD internal processes), the address of the 32-bit word with the error is reported. In case of errors detected in more than one 32-bit word, only the least significant address is reported.

Figure 16-44: CANFD\_ERR\_RADDR Register Diagram

<!-- image -->

Table 16-62: CANFD\_ERR\_RADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/NW)          | NCERR      | Non-Correctable Error. The CANFD_ERR_RADDR.NCERR bit indicates that a report is due to an non-cor- rectable error. When the CANFD_ERR_RADDR.NCERR bit is set, it is reporting a non-correctable error. When it is cleared, it is reporting a correctable-error. | Non-Correctable Error. The CANFD_ERR_RADDR.NCERR bit indicates that a report is due to an non-cor- rectable error. When the CANFD_ERR_RADDR.NCERR bit is set, it is reporting a non-correctable error. When it is cleared, it is reporting a correctable-error. |
| 18:16 (R/NW)       | RDREQID    | Read Request ID. The CANFD_ERR_RADDR.RDREQID bit field identifies the details of a memory read request.                                                                                                                                                         | Read Request ID. The CANFD_ERR_RADDR.RDREQID bit field identifies the details of a memory read request.                                                                                                                                                         |
|                    |            | 0                                                                                                                                                                                                                                                               | Move-Out CANFD Access                                                                                                                                                                                                                                           |
|                    |            | 1                                                                                                                                                                                                                                                               | Move-In CANFD Access                                                                                                                                                                                                                                            |
|                    |            | 2                                                                                                                                                                                                                                                               | TX Arbitration                                                                                                                                                                                                                                                  |
|                    |            | 3                                                                                                                                                                                                                                                               | Rx Matching                                                                                                                                                                                                                                                     |
|                    |            | 4                                                                                                                                                                                                                                                               | Move-Out Host Access                                                                                                                                                                                                                                            |
|                    |            | 5                                                                                                                                                                                                                                                               | Reserved                                                                                                                                                                                                                                                        |
|                    |            | 6                                                                                                                                                                                                                                                               | Reserved                                                                                                                                                                                                                                                        |
|                    |            | 7                                                                                                                                                                                                                                                               | Reserved                                                                                                                                                                                                                                                        |

Table 16-62: CANFD\_ERR\_RADDR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:0 (R/NW)        | VALUE      | Error Detection Address. The CANFD_ERR_RADDR.VALUE bit field contains the address where the error was detected. See the description of the Error Injection Address Register ( CANFD_ERR_IADDR ). |

## Error Report Data Register

The CANFD\_ERR\_RDAT register reports the raw data (unmodified by the ECC logic correction) read from the memory with the error. The value reported does not represent the transient values of the BUSY bit when reading a message buffer.

Figure 16-45: CANFD\_ERR\_RDAT Register Diagram

<!-- image -->

Table 16-63: CANFD\_ERR\_RDAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Raw Data Word with Error. The CANFD_ERR_RDAT.VALUE bit field contains the raw data word read from memory with error. This data word is unmodified by the correction performed by ECC logic and does not represent the transient values of the BUSY bit when reading the message buffer. |

## Error Report Syndrome Register

The CANFD\_ERR\_RSYN register holds the syndrome detected in a memory read with an error. It reports the bytes that are read in the 32-bit read transaction.

Each SYNDn field indicates the type of error and which bit in byte (n) is affected by the error. The CANFD\_ERR\_RSYN.SYND3 bit field corresponds to the most significant byte in the data word read from memory and the CANFD\_ERR\_RSYN.SYND0 bit field corresponds to the least significant.

Each BEn field indicates which byte in the 32-bit word reported was effectively read. The syndrome bits are calculated for all bytes, even for the non-read ones. Errors detected in non-read bytes are indicated and reported.

See the Detection and Correction of Memory Errors section of the CANFD chapter for more details.

Figure 16-46: CANFD\_ERR\_RSYN Register Diagram

<!-- image -->

Table 16-64: CANFD\_ERR\_RSYN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | BEN3       | Byte 3 Enabled. The CANFD_ERR_RSYN.BEN3 bit indicates if byte 3 (most significant) was read. If the CANFD_ERR_RSYN.BEN3 bit is enabled the byte was read, and if it is disabled then it was not read. 0 Disable |
| 28:24 (R/NW)       | SYND3      | Byte 3 Error Syndrome. The CANFD_ERR_RSYN.SYND3 bit field contains the error syndrome for byte 3. The Syndrome Definition table in the CANFD chapter defines the values for the CANFD_ERR_RSYN.SYND3 bit field. |

Table 16-64: CANFD\_ERR\_RSYN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23 (R/NW)          | BEN2       | Byte 2 Enabled. The CANFD_ERR_RSYN.BEN2 bit indicates if byte 2 was read. If the CANFD_ERR_RSYN.BEN2 bit is enabled the byte was read, and if it is disabled then it was not read.                                                  |
| 23 (R/NW)          | BEN2       | 0 Disable                                                                                                                                                                                                                           |
| 23 (R/NW)          | BEN2       | 1 Enable                                                                                                                                                                                                                            |
| 20:16 (R/NW)       | SYND2      | Byte 2 Error Syndrome. The CANFD_ERR_RSYN.SYND2 bit field contains the error syndrome for byte 2. The Syndrome Definition table in the CANFD chapter defines the values for the CANFD_ERR_RSYN.SYND2 bit field.                     |
| 15 (R/NW)          | BEN1       | Byte 1 Enabled. The CANFD_ERR_RSYN.BEN1 bit indicates if byte 1 was read. If the CANFD_ERR_RSYN.BEN1 bit is enabled the byte was read, and if it is disabled then it was not read.                                                  |
| 15 (R/NW)          | BEN1       | 0 Disable                                                                                                                                                                                                                           |
| 12:8 (R/NW)        | SYND1      | 1 Enable Byte 1 Error Syndrome. The CANFD_ERR_RSYN.SYND1 bit field contains the error syndrome for byte 1. The Syndrome Definition table in the CANFD chapter defines the values for the                                            |
| 7 (R/NW)           | BEN0       | Byte 0 Enabled. The CANFD_ERR_RSYN.BEN0 bit indicates if byte 2 (least significant) was read. If the CANFD_ERR_RSYN.BEN0 bit is enabled the byte was read, and if it is disabled then it was not read. Disable                      |
| 7 (R/NW)           | BEN0       | 0                                                                                                                                                                                                                                   |
| 7 (R/NW)           | BEN0       | 1 Enable                                                                                                                                                                                                                            |
| 4:0 (R/NW)         | SYND0      | Byte 0 Error Syndrome. The CANFD_ERR_RSYN.SYND0 bit field contains the error syndrome for byte 0 (least significant). The Syndrome Definition table in the CANFD chapter defines the values for the CANFD_ERR_RSYN.SYND0 bit field. |

## Receive Mailbox14 Mask Register

The CANFD\_RX\_14\_MSK register is located in RAM.

When the CANFD\_CFG.IRMQEN bit is asserted, the CANFD\_RX\_14\_MSK register has no effect.

The CANFD\_RX\_14\_MSK register is used to mask the filter fields of message buffer 14.

This register can only be programmed while the CANFD module is in freeze mode as it is blocked by hardware in other modes.

Figure 16-47: CANFD\_RX\_14\_MSK Register Diagram

<!-- image -->

Table 16-65: CANFD\_RX\_14\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | MSKVAL     | RX14 Mask Value. In the CANFD_RX_14_MSK.MSKVAL bit field, if a bit is a zero the corresponding bit in the filter is a don't care. If it is a one, the corresponding bit in the filter is checked. Each mask bit masks the corresponding mailbox 14 filter field in the same way that the CANFD_RX_MB_GMSK register masks other mailbox filters. |

## Receive Mailbox15 Mask Register

The CANFD\_RX\_15\_MSK register is located in RAM.

When the CANFD\_CFG.IRMQEN bit is asserted, the CANFD\_RX\_15\_MSK register has no effect.

The CANFD\_RX\_15\_MSK register is used to mask the filter fields of message buffer 15.

This register can only be programmed while the CANFD module is in freeze mode as it is blocked by hardware in other modes.

Figure 16-48: CANFD\_RX\_15\_MSK Register Diagram

<!-- image -->

Table 16-66: CANFD\_RX\_15\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | MSKVAL     | RX15 Mask Value. In the CANFD_RX_15_MSK.MSKVAL bit field, if a bit is a zero the corresponding bit in the filter is a don't care. If it is a one, the corresponding bit in the filter is checked. Each mask bit masks the corresponding mailbox 15 filter field in the same way that the CANFD_RX_MB_GMSK register masks other mailbox filters. |

## Receive FIFO Global Mask Register

The CANFD\_RX\_FIFO\_GMSK register is located in RAM.

If the Rx FIFO is enabled, the CANFD\_RX\_FIFO\_GMSK register is used to mask the Rx FIFO ID filter table elements that do not have a corresponding CANFD\_RX\_IMSK according to the CANFD\_CTL2.RFFNUM bit field setting.

The CANFD\_RX\_FIFO\_GMSK register can only be written in freeze mode as it is blocked by hardware in other modes.

Figure 16-49: CANFD\_RX\_FIFO\_GMSK Register Diagram

<!-- image -->

Table 16-67: CANFD\_RX\_FIFO\_GMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Rx FIFO Global Mask Bits. The CANFD_RX_FIFO_GMSK.VALUE bits mask the ID filter table elements bits in a perfect alignment. If the CANFD_RX_FIFO_GMSK.VALUE bit is set, the corre- sponding bit in the filter is checked. If it is clear, the corresponding bit in the filter is a dont care. See the Identifier Acceptance Filter Fields for Global Mask Bits table in the CANFD chapter. |

## Receive FIFO Information Register

The CANFD\_RX\_FIFO register provides information on the Rx FIFO.

The CANFD\_RX\_FIFO register is the port through which the processor accesses the output of the RXFIR FIFO located in RAM. The CANFD\_RX\_FIFO register is written by the CANFD module whenever a new message is moved into the Rx FIFO and the register output is updated whenever the output of the Rx FIFO is updated with the next message.

Figure 16-50: CANFD\_RX\_FIFO Register Diagram

<!-- image -->

Table 16-68: CANFD\_RX\_FIFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8:0 (R/NW)         | IDHIT      | Identifier Acceptance Filter Hit Indicator. The CANFD_RX_FIFO.IDHIT bit field indicates which identifier acceptance filter was hit by the received message that is in the output of the Rx FIFO. If multiple filters match the incoming message ID, the first matching IDAF found (lowest number) by the matching process is indicated. The CANFD_RX_FIFO.IDHIT bit field is valid only while the CANFD_IFLG1.MB05 bit is set. |

## Receive Individual Mask Register

The CANFD\_RX\_IMSK[n] registers are used to store the acceptance masks for ID filtering in Rx MBs and the Rx FIFO.

When the Rx FIFO is disabled ( CANFD\_CFG.RFEN is disabled), an individual mask is provided for each available Rx mailbox on a one-to-one correspondence. When the Rx FIFO is enabled ( CANFD\_CFG.RFEN is enabled), an individual mask is provided for each Rx FIFO ID Filter table element on a one-to-one correspondence depending on the setting of the CANFD\_CTL2.RFFNUM bit.

The CANFD\_RX\_IMSK0 register stores the individual mask associated with either MB0 or ID Filter Table Element 0, CANFD\_RX\_IMSK1 stores the individual mask associated with either MB1 or ID Filter Table Element 1, and so on.

The CANFD\_RX\_IMSK[n] registers are only be accessed by the processor while the CANFD is in freeze mode and are otherwise blocked by hardware.

The CANFD\_RX\_IMSK[n] registers are not affected by reset. They are located in RAM and must be explicitly initialized prior to any reception.

Figure 16-51: CANFD\_RX\_IMSK[n] Register Diagram

<!-- image -->

Table 16-69: CANFD\_RX\_IMSK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | IM         | Individual Mask Bits. The CANFD_RX_IMSK[n].IM bits masks the corresponding bit in both the mail- box filter and Rx FIFO ID filter table element in distinct ways. For mailbox filters, see the CANFD_RX_MB_GMSK register description. For Rx FIFO ID filter table ele- ments, see the CANFD_RX_FIFO_GMSK register description. If a bit in the CANFD_RX_IMSK[n].IM field is set, the corresponding bit in the filter is checked. If it is cleared, the corresponding bit in the filter is a don't care. |

## Receive Mailbox Global Mask Register

The CANFD\_RX\_MB\_GMSK register is located in RAM.

When the CANFD\_CFG.IRMQEN bit is disabled, the CANFD\_RX\_MB\_GMSK register is always in effect (the bits in the MG field will mask the mailbox filter bits).

When the CANFD\_CFG.IRMQEN bit is enabled, the CANFD\_RX\_MB\_GMSK register has no effect (the bits in the MG field will not mask the mailbox filter bits).

The CANFD\_RX\_MB\_GMSK register is used to mask the filter fields of all Rx MBs, excluding MBs 14-15, which have individual mask registers.

The CANFD\_RX\_MB\_GMSK register can only be written in freeze mode and is blocked by hardware in other modes.

Figure 16-52: CANFD\_RX\_MB\_GMSK Register Diagram

<!-- image -->

Table 16-70: CANFD\_RX\_MB\_GMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | MB[nn]     | Rx Mailboxes Global Mask Value. For all CANFD_RX_MB_GMSK.MB[nn] bits, write =0 for stop, and write =1 for start. Read =1 when timer is running. The bits in the CANFD_RX_MB_GMSK.MB[nn] bit field mask the mailbox filter bits. If a bit in the field is a zero, the corresponding bit in the filter is a don't care. If the bit is a one, the corresponding bit in the filter is checked. Note that the alignment with the ID word of the mailbox is not perfect as the two most significant bits affect the fields RTR and IDE, which are located in the control and status (C/S) word of the mailbox. For more details on this bit field, see the Mail- box Global Filter Field Mask table in the CANFD chapter. Care |
| 31:0 (R/W)         | MB[nn]     | 0 Don't                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 31:0 (R/W)         | MB[nn]     | 1 Checked                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

## Free Running Timer Register

The CANFD\_TMR register is a 16-bit free running counter. The timer starts from 0x0 after reset, counts linearly to 0xFFFF, and wraps around.

The timer is incremented by the CAN bit clock, which defines the baud rate on the CAN bus. During a message transmission/reception, the timer increments by one for each bit that is received or transmitted. When there is no message on the bus, the timer counts using the previously programmed baud rate. The timer is not incremented during module disable, doze, stop, pretended networking, and freeze modes.

The timer value is captured when the second bit of the identifier field of any frame is on the CAN bus. This captured value is written into the time stamp entry in a message buffer after the successful reception or transmission of a message.

If the CANFD\_CTL1.TSYNEN bit is enabled, the timer is reset whenever a message is received in the first available mailbox, according to the CANFD\_CTL2.RFFNUM setting.

The processor can write to this register anytime. However, if the write occurs at the same time that the timer is being reset by a reception in the first mailbox, then the write value is discarded.

Reading this register affects the mailbox unlocking procedure.

Figure 16-53: CANFD\_TMR Register Diagram

<!-- image -->

Table 16-71: CANFD\_TMR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 15:0               | VALUE      | Timer Value.                                                           |
| (R/W)              |            | The CANFD_TMR.VALUE bit field contains the free-running counter value. |

## Wakeup Message Buffer Data 4-7 Register

The CANFD\_WMB[n]\_DATA\_HI registers are the wakeup message buffer data 4-7 registers. Each of the four wake up message buffers contains a register to store the data bytes 4 to 7 of the payload information of an incoming Rx message. This register content is cleared when the incoming matched message is either a remote frame (RTR=1) or a data frame with DLC=0.

Note that the data 4-7 registers are located at 0xB4C for WMB0, 0xB5C for WMB1, 0xB6C for WMB2, and 0xB7C for WMB3.

Figure 16-54: CANFD\_WMB[n]\_DATA\_HI Register Diagram

<!-- image -->

Table 16-72: CANFD\_WMB[n]\_DATA\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | PDB4       | Payload Data Byte 4. The CANFD_WMB[n]_DATA_HI.PDB4 bit field contains the received payload cor- responding to data byte 4 in pretended networking mode. |
| 23:16 (R/NW)       | PDB5       | Payload Data Byte 5. The CANFD_WMB[n]_DATA_HI.PDB5 bit field contains the received payload cor- responding to data byte 5 in pretended networking mode. |
| 15:8 (R/NW)        | PDB6       | Payload Data Byte 6. The CANFD_WMB[n]_DATA_HI.PDB6 bit field contains the received payload cor- responding to data byte 6 in pretended networking mode. |
| 7:0 (R/NW)         | PDB7       | Payload Data Byte 7. The CANFD_WMB[n]_DATA_HI.PDB7 bit field contains the received payload cor- responding to data byte 7 in pretended networking mode. |

## Wakeup Message Buffer Data 0-3 Register

The CANFD\_WMB[n]\_DATA\_LO registers are the wakeup message buffer data 0-3 registers. Each of the four wake up message buffers contains a register to store the data bytes 0 to 3 of the payload information of an incoming Rx message. This register content is cleared when the incoming matched message is either a remote frame (RTR=1) or a data frame with DLC=0.

Note that the data 0-3 registers are located at 0xB48 for WMB0, 0xB58 for WMB1, 0xB68 for WMB2, and 0xB78 for WMB3.

Figure 16-55: CANFD\_WMB[n]\_DATA\_LO Register Diagram

<!-- image -->

Table 16-73: CANFD\_WMB[n]\_DATA\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | PDB0       | Payload Data Byte 0. The CANFD_WMB[n]_DATA_LO.PDB0 bit field contains the received payload cor- responding to data byte 0 in pretended networking mode. |
| 23:16 (R/NW)       | PDB1       | Payload Data Byte 1. The CANFD_WMB[n]_DATA_LO.PDB1 bit field contains the received payload cor- responding to data byte 1 in pretended networking mode. |
| 15:8 (R/NW)        | PDB2       | Payload Data Byte 2. The CANFD_WMB[n]_DATA_LO.PDB2 bit field contains the received payload cor- responding to data byte 2 in pretended networking mode. |
| 7:0 (R/NW)         | PDB3       | Payload Data Byte 3. The CANFD_WMB[n]_DATA_LO.PDB3 bit field contains the received payload cor- responding to data byte 3 in pretended networking mode. |

## Wakeup Message ID Buffer Register

The CANFD\_WMB[n]\_ID registers are the wakeup message ID buffers. Each of the four wake up message buffers contains a register to store the ID information of an incoming Rx message.

Note that the ID registers are located at 0xB44 for WMB0, 0xB54 for WMB1, 0xB64 for WMB2, and 0xB74 for WMB3.

Figure 16-56: CANFD\_WMB[n]\_ID Register Diagram

<!-- image -->

Table 16-74: CANFD\_WMB[n]\_ID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28:0 (R/W)         | RCVID      | Received ID Pretended Networking Mode. The CANFD_WMB[n]_ID.RCVID bit field stores the received ID as either, the 29 bits of the extended frame format (considering the ID[28:0] field) or the 11 bits of the standard frame format (considering the ID[28:18] field only. In the standard frame format, the remaining bits in the ID[17:0] range have no meaning. |

## Wakeup Message Buffer Control/Status Register

The CANFD\_WMB[n]\_STAT registers are the for wakeup message buffer control and status information. Each of the four wake up message buffers contains a register to store the control status information (IDE, RTR and DLC fields) of an incoming Rx message.

Note that the control status registers are located at 0xB40 for WMB0, 0xB50 for WMB1, 0xB60 for WMB2, and 0xB70 for WMB3.

Figure 16-57: CANFD\_WMB[n]\_STAT Register Diagram

<!-- image -->

Table 16-75: CANFD\_WMB[n]\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/NW)          | SRR        | Substitute Remote Request. The CANFD_WMB[n]_STAT.SRR bit is received as either recessive or dominant.                                                                                                                                                                                                                                                                    |
| 21 (R/NW)          | IDE        | ID Extended Bit. The CANFD_WMB[n]_STAT.IDE bit identifies is the frame format is standard or extended. 0 Standard Frame Format                                                                                                                                                                                                                                           |
| 20 (R/NW)          | RTR        | Remote Transmission Request Bit. The CANFD_WMB[n]_STAT.RTR bit identifies if the frame is remote. 0 Data Frame (Not Remote)                                                                                                                                                                                                                                              |
| 19:16 (R/NW)       | DLC        | Length of Data in Bytes. The CANFD_WMB[n]_STAT.DLC bit field represents the length (in bytes) of the Rx data received when the CANFD module is in pretended networking mode. The CANFD_WMB[n]_STAT.DLC bit field is written by the CANFD module, copied from the data length code (DLC) field of the received frame. The DLC field indicates which data bytes are valid. |

## Pretended Networking Wakeup Match Register

The CANFD\_WUM register contains wake up information related to the matching processes performed while the CANFD module receives frames under pretended networking mode.

Figure 16-58: CANFD\_WUM Register Diagram

<!-- image -->

Table 16-76: CANFD\_WUM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | WTOF       | Wake Up by Timeout Flag Bit. The CANFD_WUM.WTOF bit indicates if the CANFD module detects a timeout event during a time interval defined by the CANFD_PN_CTL2.MATCHTO bit field. WTOF generates a wakeup event if the CANFD_PN_CTL1.WTOFMSK enabled.                              | Wake Up by Timeout Flag Bit. The CANFD_WUM.WTOF bit indicates if the CANFD module detects a timeout event during a time interval defined by the CANFD_PN_CTL2.MATCHTO bit field. WTOF generates a wakeup event if the CANFD_PN_CTL1.WTOFMSK enabled.                              |
| 17 (R/W1C)         | WTOF       | 0                                                                                                                                                                                                                                                                                 | No event detected                                                                                                                                                                                                                                                                 |
| 17 (R/W1C)         | WTOF       | 1                                                                                                                                                                                                                                                                                 | Wakeup event detected                                                                                                                                                                                                                                                             |
| 16 (R/W1C)         | WUMF       | Wake Up by Match Flag Bit. The CANFD_WUM.WUMF bit indicates whether the CANFD module detects a match- ing Rx incoming message passing the filtering criteria specified in the CANFD_PN_CTL1 register. WUMFgenerates a wakeup event if the CANFD_PN_CTL1.WUMFMSK bit is ena- bled. | Wake Up by Match Flag Bit. The CANFD_WUM.WUMF bit indicates whether the CANFD module detects a match- ing Rx incoming message passing the filtering criteria specified in the CANFD_PN_CTL1 register. WUMFgenerates a wakeup event if the CANFD_PN_CTL1.WUMFMSK bit is ena- bled. |
| 16 (R/W1C)         | WUMF       | 0                                                                                                                                                                                                                                                                                 | No event detected                                                                                                                                                                                                                                                                 |
| 16 (R/W1C)         | WUMF       | 1                                                                                                                                                                                                                                                                                 | Wakeup event detected                                                                                                                                                                                                                                                             |
| 15:8 (R/NW)        | MCNT       | Number of Matches in Pretended Networking. The CANFD_WUM.MCNT bit field reports the number of times a given message                                                                                                                                                               | Number of Matches in Pretended Networking. The CANFD_WUM.MCNT bit field reports the number of times a given message                                                                                                                                                               |

## ADSP-2159x\_SC592\_SC594 MISCREG Register Descriptions

Misc registers for module are for integration purpose (MISCREG) contains the following registers.

Table 16-77: ADSP-2159x\_SC592\_SC594 MISCREG Register List

| Name                         | Description                                   |
|------------------------------|-----------------------------------------------|
| MISCREG_CAN_SYSCTL           | CANFD Low Power Selection Mode                |
| MISCREG_ECO_REG9             | Error Register                                |
| MISCREG_SH0_PFB_RANGE_SELECT | SH0 Prefetch Range Selection Register         |
| MISCREG_SH1_PFB_RANGE_SELECT | SH1 Prefetch Range Selection Register         |
| MISCREG_SHARC_BRIDGE_REMAP   | SPORT Direct Interrupt Enable for SH0 and SH1 |

## CANFD Low Power Selection Mode

The MISCREG\_CAN\_SYSCTL register contains the control bits for the different low power modes supported in the CANFD module.

Figure 16-59: MISCREG\_CAN\_SYSCTL Register Diagram

<!-- image -->

Table 16-78: MISCREG\_CAN\_SYSCTL Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                         |
|--------------------|----------------|-----------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | CAN1_IPG_SRST  | CAN1 Soft Reset. When set (=1), the MISCREG_CAN_SYSCTL.CAN1_IPG_SRST bit enables a soft reset request to CAN1.  |
| 6 (R/W)            | CAN1_IPG_DEBUG | CAN1 Debug Mode. When set (=1), the MISCREG_CAN_SYSCTL.CAN1_IPG_DEBUG bit enables a debug mode request to CAN1. |
| 5 (R/W)            | CAN1_IPG_STOP  | CAN1 Stop Mode. When set (=1), the MISCREG_CAN_SYSCTL.CAN1_IPG_STOP bit enables a stop mode request to CAN1.    |
| 4 (R/W)            | CAN1_IPG_DOZE  | CAN1 Doze Mode. When set (=1), the MISCREG_CAN_SYSCTL.CAN1_IPG_DOZE bit enables a doze mode request to CAN1.    |
| 3 (R/W)            | CAN0_IPG_SRST  | CAN0 Soft Reset. When set (=1), the MISCREG_CAN_SYSCTL.CAN0_IPG_SRST bit enables a soft reset request to CAN0.  |
| 2 (R/W)            | CAN0_IPG_DEBUG | CAN0 Debug Mode. When set (=1), the MISCREG_CAN_SYSCTL.CAN0_IPG_DEBUG bit enables a debug mode request to CAN0. |

Table 16-78: MISCREG\_CAN\_SYSCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                      |
|--------------------|---------------|--------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | CAN0_IPG_STOP | CAN0 Stop Mode. When set (=1), the MISCREG_CAN_SYSCTL.CAN0_IPG_STOP bit enables a stop mode request to CAN0. |
| 0 (R/W)            | CAN0_IPG_DOZE | CAN0 Doze Mode. When set (=1), the MISCREG_CAN_SYSCTL.CAN0_IPG_DOZE bit enables a doze mode request to CAN0. |

## Error Register

The MISCREG\_ECO\_REG9 register contains eight Tx channels, eight Rx channels, and DMA errors for both EMAC0 and EMAC1.

Figure 16-60: MISCREG\_ECO\_REG9 Register Diagram

<!-- image -->

Table 16-79: MISCREG\_ECO\_REG9 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | ECO_VALUE9 | .                         |

## SH0 Prefetch Range Selection Register

In the MISCREG\_SH0\_PFB\_RANGE\_SELECT register, each bit field controls if the corresponding address range will be prefetched. When an address range is set it is prefetched and when it is zero there is no prefetch.

The MISCREG\_SH0\_PFB\_RANGE\_SELECT [31:16] bits control the range selection for the instruction cache, and MISCREG\_SH0\_PFB\_RANGE\_SELECT [15:0] bits control the range selection for the data cache.

Figure 16-61: MISCREG\_SH0\_PFB\_RANGE\_SELECT Register Diagram

<!-- image -->

Table 16-80: MISCREG\_SH0\_PFB\_RANGE\_SELECT Register Fields

| Bit No. (Access)   | Bit Name             | Description/Enumeration                                                                                                                      |
|--------------------|----------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | IPORT_RANGE_SELECT   | IPORT Prefetch Range Select. The MISCREG_SH0_PFB_RANGE_SELECT.IPORT_RANGE_SELECT bit field indicates the prefetch range selection for IPORT. |
| 15:0 (R/W)         | DPORT_RANGE_SE- LECT | DPORT Prefetch Range Select. The MISCREG_SH0_PFB_RANGE_SELECT.DPORT_RANGE_SELECT bit field indicates the prefetch range selection for DPORT. |

## SH1 Prefetch Range Selection Register

Each bit field controls if the corresponding address range will be prefetched. When an address range is set it is prefetched an when it is zero there is no prefetch.

The MISCREG\_SH1\_PFB\_RANGE\_SELECT [31:16] bits control the range selection for the instruction cache, and MISCREG\_SH1\_PFB\_RANGE\_SELECT [15:0] bits control the range selection for the data cache.

Figure 16-62: MISCREG\_SH1\_PFB\_RANGE\_SELECT Register Diagram

<!-- image -->

Table 16-81: MISCREG\_SH1\_PFB\_RANGE\_SELECT Register Fields

| Bit No. (Access)   | Bit Name                 | Description/Enumeration                                                                                                                              |
|--------------------|--------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | SH1_IPORT_RANGE_SE- LECT | SH1 IPORT Prefetch Range Select. The MISCREG_SH1_PFB_RANGE_SELECT.SH1_IPORT_RANGE_SELECT bit field indicates the prefetch range selection for IPORT. |
| 15:0 (R/W)         | SH1_DPORT_RANGE_SE LECT  | SH1 DPORT Prefetch Range Select. The MISCREG_SH1_PFB_RANGE_SELECT.SH1_DPORT_RANGE_SELECT bit field indicates the prefetch range selection for DPORT. |

## SPORT Direct Interrupt Enable for SH0 and SH1

The bits in the MISCREG\_SHARC\_BRIDGE\_REMAP register control the direct connection of SPORT interrupts to the SHARC core.

Figure 16-63: MISCREG\_SHARC\_BRIDGE\_REMAP Register Diagram

<!-- image -->

Table 16-82: MISCREG\_SHARC\_BRIDGE\_REMAP Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                            |
|--------------------|-----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | SH1_DRT_INTR_EN | SPORT Direct Interrupt Enable for SH1. If the MISCREG_SHARC_BRIDGE_REMAP.SH1_DRT_INTR_EN bit is set a di- rect connection is enabled for SH1. Do not set the MISCREG_SHARC_BRIDGE_REMAP.SH1_DRT_INTR_EN bit when IIR1 of the SH1 is in use. If this bit is zero the direct connection is disabled. |
| 16 (R/W)           | SH0_DRT_INTR_EN | SPORT Direct Interrupt Enable for SH0. If the MISCREG_SHARC_BRIDGE_REMAP.SH0_DRT_INTR_EN bit is set a di- rect connection is enabled for SH0. Do not set the MISCREG_SHARC_BRIDGE_REMAP.SH0_DRT_INTR_EN bit when IIR1 of the SH0 is in use. If this bit is zero the direct connection is disabled. |