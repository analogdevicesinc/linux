## 25   Controller Area Network (CAN)

The processor contains a Controller Area Network (CAN) module based on the CAN 2.0B (active) protocol. This protocol is an asynchronous communications protocol used in both industrial and automotive control systems. The CAN protocol is compatible with the control applications. It can communicate reliably over a network and incorporates CRC checking, message error tracking, and fault node confinement.

NOTE: This document assumes familiarity with the CAN standard. For more information, refer to Version 2.0 of the CAN specification from Robert Bosch GmbH.

## CAN Features

Key features of the CAN module include:

- Conformity to the CAN 2.0B (active) standard
- Dedicated acceptance mask for each mailbox
- Support for data rates of up to 1M bit/s
- Support for standard (11-bit) and extended (29-bit) identifiers
- 32 mailboxes (8 transmit, 8 receive, 16 configurable)
- Data filtering (first 2 bytes) for acceptance filtering (DeviceNet TM  mode)
- Error status and warning registers
- Universal counter-module
- Readable receive and transmit pin values
- Support for remote frames
- Active or passive network support
- Interrupts, including transmit or receive complete, error, and global
- Clock derived from CDU0\_CLKO4 through a programmable divider, eliminating the need for an extra crystal

## CAN Functional Description

The following sections provide information on the functional operation of the CAN module. This section also provides listings of the CAN registers and interrupts.

## ADSP-SC58x CAN Register List

The controller area network (CAN) module implements the CAN 2.0B (active) protocol. This protocol is an asynchronous communications protocol used in both industrial and automotive control systems. A set of registers govern CAN operations. For more information on CAN functionality, see the CAN register descriptions.

Table 25-1: ADSP-SC58x CAN Register List

| Name             | Description                                |
|------------------|--------------------------------------------|
| CAN_AA1          | Abort Acknowledge 1 Register               |
| CAN_AA2          | Abort Acknowledge 2 Register               |
| CAN_AM[nn]H      | Acceptance Mask (H) Register               |
| CAN_AM[nn]L      | Acceptance Mask (L) Register               |
| CAN_CEC          | Error Counter Register                     |
| CAN_CLK          | Clock Register                             |
| CAN_CTL          | CAN Master Control Register                |
| CAN_DBG          | Debug Register                             |
| CAN_ESR          | Error Status Register                      |
| CAN_EWR          | Error Counter Warning Level Register       |
| CAN_GIF          | Global CAN Interrupt Flag Register         |
| CAN_GIM          | Global CAN Interrupt Mask Register         |
| CAN_GIS          | Global CAN Interrupt Status Register       |
| CAN_INT          | Interrupt Pending Register                 |
| CAN_MBIM1        | Mailbox Interrupt Mask 1 Register          |
| CAN_MBIM2        | Mailbox Interrupt Mask 2 Register          |
| CAN_MBRIF1       | Mailbox Receive Interrupt Flag 1 Register  |
| CAN_MBRIF2       | Mailbox Receive Interrupt Flag 2 Register  |
| CAN_MBTD         | Temporary Mailbox Disable Register         |
| CAN_MBTIF1       | Mailbox Transmit Interrupt Flag 1 Register |
| CAN_MBTIF2       | Mailbox Transmit Interrupt Flag 2 Register |
| CAN_MB[nn]_DATA0 | Mailbox Word 0 Register                    |
| CAN_MB[nn]_DATA1 | Mailbox Word 1 Register                    |

Table 25-1: ADSP-SC58x CAN Register List (Continued)

| Name                 | Description                                              |
|----------------------|----------------------------------------------------------|
| CAN_MB[nn]_DATA2     | Mailbox Word 2 Register                                  |
| CAN_MB[nn]_DATA3     | Mailbox Word 3 Register                                  |
| CAN_MB[nn]_ID0       | Mailbox ID 0 Register                                    |
| CAN_MB[nn]_ID1       | Mailbox ID 1 Register                                    |
| CAN_MB[nn]_LENGTH    | Mailbox Length Register                                  |
| CAN_MB[nn]_TIMESTAMP | Mailbox Time Stamp Register                              |
| CAN_MC1              | Mailbox Configuration 1 Register                         |
| CAN_MC2              | Mailbox Configuration 2 Register                         |
| CAN_MD1              | Mailbox Direction 1 Register                             |
| CAN_MD2              | Mailbox Direction 2 Register                             |
| CAN_OPSS1            | Overwrite Protection/Single Shot Transmission 1 Register |
| CAN_OPSS2            | Overwrite Protection/Single Shot Transmission 2 Register |
| CAN_RFH1             | Remote Frame Handling 1 Register                         |
| CAN_RFH2             | Remote Frame Handling 2 Register                         |
| CAN_RML1             | Receive Message Lost 1 Register                          |
| CAN_RML2             | Receive Message Lost 2 Register                          |
| CAN_RMP1             | Receive Message Pending 1 Register                       |
| CAN_RMP2             | Receive Message Pending 2 Register                       |
| CAN_STAT             | Status Register                                          |
| CAN_TA1              | Transmission Acknowledge 1 Register                      |
| CAN_TA2              | Transmission Acknowledge 2 Register                      |
| CAN_TIMING           | Timing Register                                          |
| CAN_TRR1             | Transmission Request Reset 1 Register                    |
| CAN_TRR2             | Transmission Request Reset 2 Register                    |
| CAN_TRS1             | Transmission Request Set 1 Register                      |
| CAN_TRS2             | Transmission Request Set 2 Register                      |
| CAN_UCCNF            | Universal Counter Configuration Mode Register            |
| CAN_UCCNT            | Universal Counter Register                               |
| CAN_UCRC             | Universal Counter Reload/Capture Register                |

## ADSP-SC58x CAN Interrupt List

Table 25-2: ADSP-SC58x CAN Interrupt List

|   Interrupt ID | Name      | Description   | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------|---------------|---------------|
|             84 | CAN0_RX   | CAN0 Receive  | Level         |               |
|             85 | CAN0_TX   | CAN0 Transmit | Level         |               |
|             86 | CAN0_STAT | CAN0 Status   | Level         |               |
|             87 | CAN1_RX   | CAN1 Receive  | Level         |               |
|             88 | CAN1_TX   | CAN1 Transmit | Level         |               |
|             89 | CAN1_STAT | CAN1 Status   | Level         |               |

## External Interface

The interface to the CAN bus is a simple two-wire line. The Representation of CAN Transceiver Interconnection shows a symbolic representation of the CAN transceiver interconnection. Typically, the CAN\_TX output and CAN\_RX input pins of the processor connect to an external CAN CAN\_TX and CAN\_RX pins (respectively) of the tranceiver. The CAN\_TX and CAN\_RX pins operate with TTL levels and are appropriate for operation with CAN bus transceivers according to ISO/DIS 11898.

Figure 25-1: Representation of CAN Transceiver Interconnection

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000000_8118ac9f702a15da468fe58147a2d6ecf6e5aa235c44435b9892dc20fa7f3921.png)

CAN data is either dominant (logic 0) or recessive (logic 1). The default state of the CAN\_TX output is recessive.

## Architectural Concepts

The full CAN controller features 32 message buffers called mailboxes. Eight mailboxes are dedicated for message transmission, eight are for reception, and 16 are programmable in direction.

The CAN module architecture is based around a 32-entry mailbox RAM. The CAN serial interface or the processor core accesses the mailbox sequentially. Each mailbox consists of eight 16-bit control and data registers and two optional 16-bit acceptance mask registers. Configure all of these registers before enabling the mailbox.

Since the mailbox area is implemented as RAM, the reset values of these registers are undefined. The CAN Mailbox Area figure shows the mailbox area. The data is divided into fields, which include a message identifier, a time stamp, a byte count, up to 8 bytes of data, and several control bits.

Figure 25-2: CAN Mailbox Area

The CAN mailbox identification register pair ( CAN\_MB[nn]\_ID0 / 1 ) includes:

- The 29-bit identifier (base part CAN\_AM[nn]H.BASEID plus the extended part CAN\_AM[nn]L.EXTID / CAN\_AM[nn]H.EXTID )
- The acceptance mask enable bit ( CAN\_MB[nn]\_ID1.AME )
- The remote transmission request bit ( CAN\_MB[nn]\_ID1.RTR )
- The identifier extension bit ( CAN\_MB[nn]\_ID1.IDE )

NOTE: Do not write to the identifier of a message object while the mailbox is enabled for the CAN module (the corresponding bit in CAN\_MC1 is set).

The other mailbox area registers and bits are:

- The data length code bit ( CAN\_MB[nn]\_LENGTH.DLC ). The upper 12 bits of this register for each mailbox are marked as reserved. Always, set these 12 bits to zero.
- The mailbox word registers ( CAN\_MB[nn]\_DATA0 / 1 / 2 / 3 ) supply up to 8 bytes for the data field. The data is sent MSB first based on the number of bytes defined in the CAN\_MB[nn]\_LENGTH.DLC bit. For example, if only one byte is transmitted or received ( CAN\_MB[nn]\_LENGTH.DLC =1), then it is stored in the most significant byte of the CAN\_MB[nn]\_DATA3 register.
- The time stamp value bits ( CAN\_MB[nn]\_TIMESTAMP.TSV )

The final registers in the mailbox area are the acceptance mask registers ( CAN\_AM[nn]H and CAN\_AM[nn]L ). The acceptance mask is enabled when the CAN\_MB[nn]\_ID1.AME bit is set.

Setting the CAN\_CTL.DNM and CAN\_AM[nn]H.FDF bits enables the filtering on data field option. When enabled, the CAN\_MB[nn]\_ID0.EXTID [15:0] bits are reused as acceptance code (DFC) for the data field filtering.

## Block Diagram

The CAN Controller Block Diagram figure shows a block diagram of the CAN module.

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000001_1f4cf9cef23508dde9440fd3e857ea1f718f06c370e676b46a63189a9deed111.png)

Figure 25-3: CAN Controller Block Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000002_cbcaa1fc0808dfdd8d712662d2850209fdd2dc6e7008d1d72aaa16712b0790b1.png)

## Mailbox Control

Mailbox control memory-mapped registers (MMRs) function as control and status registers for the 32 mailboxes. Each bit in these registers represents one specific mailbox. Since CAN MMRs are all 16 bits wide, pairs of registers manage certain functionality for all 32 individual mailboxes. Mailboxes 0-15 are configured or monitored in registers with a suffix of 1. Similarly, mailboxes 16-31 use the same named register with a suffix of 2. For example, the CAN mailbox direction registers ( CAN\_MD1 / CAN\_MD2 ) control mailboxes. See the CAN Mailbox Register Pair figure. The CAN Register List table shows the mailbox control registers.

Figure 25-4: CAN Mailbox Register Pair

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000003_0bfe22c7428a90a3b65bfade4129d28cd41c47cbee5be6d4e1cf0ef2c3e6406e.png)

Mailboxes 24-31 support transmit operation only and mailboxes 0-7 are receive-only mailboxes. Therefore, the lower 8 bits in the 1 registers and the upper 8 bits in the 2 registers are sometimes reserved or are restricted in their use.

## Protocol Fundamentals

Although the CAN\_RX and CAN\_TX pins are TTL-compliant signals, the CAN signals beyond the transceiver have asymmetric drivers. A low state on the CAN\_TX pin activates strong drivers while a high state activates weak drivers. So, the active low state is the dominant state and the active high state is the recessive state. If the CAN module is passive, the CAN\_TX pin is always high. If two CAN nodes transmit at the same time, dominant bits overwrite recessive bits.

The CAN protocol specifies that all nodes trying to send a message on the CAN bus attempt to send a frame once the bus is available. The Standard CAN Frame figure shows the frame. The Start of Frame (SOF) indicator signals the beginning of a new frame. Each CAN node then begins transmitting its message starting with the message ID.

Figure 25-5: Standard CAN Frame

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000004_0255fbf92815b2343ead375aeb24107110bb4e4f9f1b082e410e6dc6fb1149fb.png)

While transmitting, the CAN controller samples the CAN\_RX pin to verify that the driven logic level is the value it placed on the CAN\_TX pin. The names for the logic levels apply here. When a transmitting node places a recessive 1 on the CAN\_TX pin and detects a dominant 0 on the CAN\_RX pin, another node has placed a dominant bit on the bus. In this case, the dominant bit from from the other node has a higher priority.

Therefore, if the value sensed on the CAN\_RX pin is the value driven on the CAN\_TX pin, transmission continues. Otherwise, the CAN controller senses that it has lost arbitration. Module configuration determines the next course of action.

The Standard CAN Frame figure shows a basic 11-bit identifier frame. The CAN\_MB[nn]\_ID1.RTR bit follows the SOF and identifier. The CAN\_MB[nn]\_ID1.RTR bit indicates whether the frame contains data (data frame) or is a request for data associated with the message identifier in the frame sent (remote frame).

NOTE: In the CAN protocol, a dominant bit in the CAN\_MB[nn]\_ID1.RTR field wins arbitration against a remote frame request ( CAN\_MB[nn]\_ID1.RTR =1) for the same message ID. This functionality allows a remote request to be a lower priority than a data frame.

The next field of interest in the frame is the CAN\_MB[nn]\_ID1.IDE bit. When set, it indicates that the message is an extended frame with a 29-bit identifier instead of an 11-bit identifier. In an extended frame, the first part of the message resembles the Extended CAN Frame figure.

Figure 25-6: Extended CAN Frame

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000005_6b9dc4d5e85e217ea4ac31fb180b042e759a369f0da48b0c8521571234176a69.png)

For the CAN\_MB[nn]\_ID1.RTR field, a dominant bit in the CAN\_MB[nn]\_ID1.IDE field wins arbitration against an extended frame with the same lower 11 bits. Standard frames have a higher priority than extended frames.

The internal logic automatically generates the Substitute Remote Request (SRR), the reserved bits r0 and r1, and the checksum (CRC). (The SRR is always sent as recessive; reserved bits r0 and r1 are always sent as dominant).

## CAN Operating Modes

The CAN controller is in configuration mode when coming out of processor reset. Hardware behavior can be altered only when CAN is in configuration mode. Before initializing the mailboxes, configure the CAN bit timing to work on the CAN bus. The controller connect to the CAN bus.

## Data Transfer Modes

The following sections provide information on the data transfer modes supported by the CAN controller.

## Transmit Operations

The CAN Transmit Operation Flowchart shows the CAN transmit operation. Mailboxes 24-31 are dedicated transmitters. Configure mailboxes 8-23 as transmitters by writing 0 to the corresponding bit in the CAN\_MD1 or CAN\_MD2 registers. Enable mailbox n ( CAN\_MC1.MB =1). After writing the data and the identifier into the mailbox area, the message is sent. Then, the corresponding transmit request bit is set ( CAN\_TRS1.MB =1).

Figure 25-7: CAN Transmit Operation Flowchart

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000006_aef0eb568f7475616da50ab71aa1e39f33733f7331c1398a0ba2d2041e6871a9.png)

When a transmission completes, the corresponding bits in the CAN\_TRS1 or CAN\_TRS2 and CAN\_TRR1 or CAN\_TRR2 registers are cleared. If the transmission is successful, the corresponding bit in the CAN\_TA1 / CAN\_TA2 register is set. If the transmission aborts due to lost arbitration or a CAN error, the corresponding bit in

the CAN\_AA1 / CAN\_AA2 register is set. A requested transmission can also be manually aborted by setting the corresponding bit in the CAN\_TRR1 / CAN\_TRR2 register.

Software sets multiple CAN\_TRS1.MB bits simultaneously. These bits are reset after either a successful or an aborted transmission.

The CAN hardware sets these bits in the following cases:

- When using the auto-transmit mode of the universal counter
- When a message loses arbitration and the single-shot CAN\_OPSS1.MB bit is not set
- When a remote frame request occurs (only possible for receive or transmit mailboxes if the feature for automatic remote frame handling is enabled ( CAN\_RFH1.MB =1)).

NOTE: Manage the mailbox area when a CAN\_TRS1 or CAN\_TRS2 bit is set. Write access to the mailbox is permissible with a bit set. But, changing data in such a mailbox can lead to unexpected data during transmission.

Enabling and disabling mailboxes has an impact on transmit requests. Setting the CAN\_TRS1 or CAN\_TRS2 bit associated with a disabled mailbox can result in erroneous behavior. Similarly, disabling a mailbox before the associated CAN\_TRS1 or CAN\_TRS2 bit is reset by the internal logic can cause unpredictable results.

## Retransmission

Normally, the current message object is resent after the loss of arbitration or error frame detection on the CAN bus line. If there is more than one transmit message object pending, the message object with the highest mailbox transmits first. See the Transmit Flow figure. The currently aborted transmission restarts after any messages with higher priority are sent.

Figure 25-8: Transmit Flow

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000007_af2f53d8e819b51b7763fa664c4707e635de58016bc7c369da7e24b23f90d9fb.png)

A message written into the mailbox does not replace a message under preparation. The message under preparation is copied into the temporary transmit buffer when the internal transmit request for the CAN core module is set. The message in the buffer is not replaced until:

- The message is sent successfully
- The arbitration on the CAN bus line is lost
- There is an error frame on the CAN bus line

## Single-Shot Transmission

When using the single-shot transmission feature ( CAN\_OPSS1.MB =1), the corresponding CAN\_TRS1 bit is cleared after the message is successfully sent. The bit is cleared even if the transmission aborts due to a lost arbitration or an error frame on the CAN bus line. Therefore, there is no further attempt to transmit the message again when the initial try failed, and the abort error is reported ( CAN\_AA1.MB =1).

## Auto-Transmission

In auto-transmit mode, the message in mailbox 11 (MB11) can be sent periodically using the universal counter. This mode often broadcasts heartbeats to all CAN nodes. So, messages sent this way usually have a high priority.

The period value is written to the CAN\_UCRC register. Auto-transmission mode is enabled by setting the CAN\_UCCNF.UCCNF field to 0x03. When enabled, the counter CAN\_UCCNT is loaded with the value in the CAN\_UCRC register. The counter decrements to 0 at the CAN bit clock rate and is then reloaded from CAN\_UCRC . Each time the counter reaches a value of 0, internal logic automatically sends the CAN\_TRS1.MB bit. The corresponding message from mailbox 11 transfers.

For proper auto-transmit operation, configure mailbox 11 as a transmit mailbox. The mailbox must contain valid data (identifier, control bits, and data) before the counter expires and after this mode is enabled.

## Receive Operation

The CAN hardware autonomously receives messages and discards invalid messages. Once a valid message is successfully received, the receive logic interrogates all enabled receive mailboxes. The logic interrogates sequentially, from mailbox 23 down to mailbox 0, whether the message is of interest to the local node or not.

Each incoming data frame is compared to all identifiers stored in the active receive and transmit mailboxes with the feature for remote frame handling enabled (=1). The active receive mailboxes indices of CAN\_MD1 and CAN\_MC1 registers are set to 1. The message identifier of the received message, along with the identifier extension ( CAN\_MB[nn]\_ID1.IDE ) and remote transmission request ( CAN\_MB[nn]\_ID1.RTR ) bits, are compared with the register settings of each mailbox. In standard mode, the message is compared with the content of the CAN\_MB[nn]\_ID1 register. In extended mode, the content of the CAN\_MB[nn]\_ID0 register must also match.

If the acceptance mask enable CAN\_MB[nn]\_ID1.AME bit is not set, a match is signaled only if CAN\_MB[nn]\_ID1.IDE , CAN\_MB[nn]\_ID1.RTR , and all (11 or 29) identifier bits are exact. If, however, the CAN\_MB[nn]\_ID1.AME bit is set, the acceptance mask registers ( CAN\_AM[nn]H / L ) determine which of the CAN\_MB[nn]\_ID1.IDE and CAN\_MB[nn]\_ID1.RTR bits must match.

## The following logic applies:

[(Received Message ID) XNOR ( CAN\_MB[nn]\_ID0 /1)] OR [( CAN\_MB[nn]\_ID1.AME ) AND ( CAN\_AM[nn]H /L )].

This logic appears graphically in the CAN Message Receive Logic figure.

Figure 25-9: CAN Message Receive Logic

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000008_8951a31347e714a96049ac804c1c05894d5f4eea0578efa8ca39c3090f0d9b1e.png)

A one (1) at the respective bit position in the CAN\_AM[nn]H / CAN\_AM[nn]L mask registers means that the bit does not need to match when CAN\_MB[nn]\_ID1.AME =1. This way, a mailbox can accept a group of messages.

Table 25-3: Mailbox Used for Acceptance Filtering

|   MCn | MDn   | RFHn   | Mailbox n   | Comment                                                                              |
|-------|-------|--------|-------------|--------------------------------------------------------------------------------------|
|     0 | X     | X      | Ignored     | Mailbox n disabled                                                                   |
|     1 | 0     | 0      | Ignored     | Mailbox n enabled; Mailbox n configured for transmit; Remote frame handling disabled |
|     1 | 0     | 1      | Used        | Mailbox n enabled; Mailbox n configured for transmit; Remote frame handling enabled  |
|     1 | 1     | X      | Used        | Mailbox n enabled; Mailbox n configured for receive                                  |

If the acceptance filter finds a matching identifier, the content of the received data frame is stored in that mailbox. A received message is stored only once, even if multiple receive mailboxes match its identifier. If the current identifier does not match any mailbox, the message is not stored.

The CAN Receive Operation Flowchart illustrates the decision tree of the receive logic when processing the individual mailboxes.

If a message is received for a mailbox and that mailbox still contains unread data ( CAN\_RMP1.MB ), then the program decides whether to overwrite the old message. If the CAN\_OPSS1.MB bit is cleared, the corresponding CAN\_RML1.MB bit is set, and the stored message is overwritten. The receive message lost interrupt request occurs ( CAN\_GIS.RMLIS is set). If, however, the CAN\_OPSS1.MB bit is set, the next mailboxes are checked for another matching identifier. If no match is found, the message is discarded, and the next message is checked.

NOTE: If a receive mailbox is disabled, an ongoing receive message for that mailbox is lost even if a second mailbox is configured to receive the same identifier.

Figure 25-10: CAN Receive Operation Flowchart

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000009_ad078aaee053f3d979fb402add4d74e0c0f83578d8f2c3a1e019c6c1e44c29a0.png)

## Data Acceptance Filtering

If Device Net mode is enabled ( CAN\_CTL.DNM = 1) and the mailbox is set up for filtering on data field, the filtering occurs on the standard ID of the message and data fields. The data field filtering can be programmed for either the first byte only or the first 2 bytes, as shown the Data Field Filtering table.

If the CAN\_AM[nn]H.FDF bit is set, the corresponding CAN\_AM[nn]L register holds the data field mask (DFM bits 15:0]). If the CAN\_AM[nn]H.FDF bit is cleared, the corresponding CAN\_AM[nn]L register holds the extended identifier mask ( CAN\_AM[nn]H.EXTID bits 15:0).

Table 25-4: Data Field Filtering

|   FDF (Filter on Data Field) |   FMD (Full Mask Data Field) | Description                              |
|------------------------------|------------------------------|------------------------------------------|
|                            0 |                            0 | Do not allow filtering on the data field |
|                            0 |                            1 | Not allowed. FMF must be 0 when FDF is 0 |
|                            1 |                            0 | Filter on first data byte only           |
|                            1 |                            1 | Filter on first two data bytes           |

## Watchdog Mode

Watchdog mode ensures that messages are received periodically. It also observes whether a certain node on the network is alive and functioning properly. Watchdog mode detects and manages the failure cases, as needed.

Enable this mode by programming the universal counter to watchdog mode by setting the CAN\_UCCNF.UCCNF to 0x2. Once enabled, the CAN\_UCCNT register is loaded with the predefined value contained in CAN\_UCRC . This counter decrements at the CAN bit rate.

If the CAN\_UCCNF.UCCT and CAN\_UCCNF.UCRC bits are set and a message is received in mailbox 4 before the counter counts down to 0, the counter is reloaded with the CAN\_UCRC contents. If the counter has counted down to 0 without receiving a message in mailbox 4, then the CAN\_GIS.UCEIS bit is set. The counter reloads automatically with the contents of the CAN\_UCRC register. If an interrupt request is desired for this event, set the CAN\_GIM.UCEIM bit. With the mask bit set, when a watchdog interrupt request occurs, the CAN\_GIF.UCEIF bit is also set.

Write to the CAN\_UCCNF register to reload the counter with the contents of CAN\_UCRC or to disable the register.

The CAN\_UCRC register controls the time period it takes for the watchdog interrupt request to occur.

## Time Stamps

To get an indication of the time of the receive or transmit time for each message, program the CAN universal counter to time stamp mode. Enable this mode by setting the CAN\_UCCNF.UCCNF field to 0x01.

If enabled, the value of the 16-bit free-running counter ( CAN\_UCCNT ) is written into the CAN\_MB[nn]\_TIMESTAMP register of the corresponding mailbox. The operation occurs when a received message is stored or a message is transmitted.

The time stamp value is captured at the sample point of the Start of Frame (SOF) bit of each incoming or outgoing message. Afterwards, this time stamp value is copied to the CAN\_MB[nn]\_TIMESTAMP register of the corresponding mailbox.

If the mailbox is configured for automatic remote frame handling ( CAN\_RFH1.MB = 1), the time stamp value is written for transmission of a data frame or the reception of the requested data frame. The mailbox is configured for transmit or receive.

Clear the counter by setting the CAN\_UCCNF.UCRC bit to 1. Or, disable the counter by clearing the CAN\_UCCNF.UCE bit. Write to the CAN\_UCCNT register to load the counter with a value.

It is also possible to clear the counter ( CAN\_UCCNT ) by the reception of a message in mailbox number 4 (synchronization of all time stamp counters in the system). This operation is accomplished by setting the CAN\_UCCNF.UCCT bit.

The CAN\_GIS.UCEIS bit is set when the counter overflows. A global CAN interrupt request can optionally occur by unmasking the CAN\_GIM.UCEIM bit. If the interrupt source is unmasked, the CAN\_GIF.UCEIF bit is also set.

## Remote Frame Handling

Automatic handling of remote frames for a transmit mailbox is enabled by setting the corresponding CAN\_RFH1.MB bit.

Remote frames are data frames that have no data field and the CAN\_MB[nn]\_ID1.RTR bit is set. The data length code (DLC) of the requesting remote frame overrules the DLC of the responding data frame. A DLC can be programmed with values in the range of 0-15, but DLC values greater than 8 are considered as 8.

A remote frame contains:

- The identifier bits
- The control field CAN\_MB[nn]\_LENGTH.DLC (data length count)
- The remote transmission request ( CAN\_MB[nn]\_ID1.RTR ) bit

Only configurable mailboxes MB8-MB23 can process remote frames, but all mailboxes can receive and transmit remote frame requests. The CAN\_OPSS1 register has no effect when configured for automatic remote frame handling. All content of a mailbox is always overwritten by an incoming message.

NOTE: If a remote frame is received, the DLC of the corresponding mailbox is overwritten with the received value.

Erroneous behavior can result when the CAN\_RFH1.MB bit is changed while the corresponding mailbox is processing. To avoid the risk of inconsistent messages, programs must temporarily disable the mailbox while its data registers are updating.

## Temporarily Disabling CAN Mailbox

If a mailbox is enabled and configured to transmit, monitor the write accesses to the data field to avoid transmitting inconsistent messages. Be careful if the mailbox is transmitting (or attempting to transmit) repeatedly. Also, if this mailbox is used for automatic remote frame handling, the data field must be updated without losing an incoming remote request frame and without sending inconsistent data. Therefore, the CAN controller allows for temporarily disabling the mailbox using the mailbox temporary disable register ( CAN\_MBTD ).

The pointer to the requested mailbox to the CAN\_MBTD.TDPTR field is written, and the CAN\_MBTD.TDR bit is set. Internal logic then sets the corresponding CAN\_MBTD.TDA flag.

If a mailbox is configured as transmit ( CAN\_MD1 = 0) and the CAN\_MBTD.TDA bit is set, the content of the data field of that mailbox can be updated. If there is an incoming remote request frame while the mailbox is temporarily disabled,

- Internal logic sets the corresponding transmit request bit ( CAN\_TRS1.MB ), and
- The data length code (DLC) of the incoming message is written to the corresponding mailbox.

However, the requested message is not sent until the CAN\_MBTD.TDR bit is cleared. Similarly, all transmit requests for temporarily disabled mailboxes are ignored until the CAN\_MBTD.TDR bit is cleared. Additionally, transmission of a message immediately aborts when the mailbox is temporarily disabled and the corresponding transmission request reset ( CAN\_TRR1.MB ) bit for this mailbox is set.

If a mailbox is configured to receive ( CAN\_MD1 = 1), then after issuing a temporary disable request, the CAN\_MBTD.TDA flag is set. The mailbox is not processed. If there is an incoming message for a temporarily disabled mailbox, the internal logic waits until reception is complete or there is an error on the CAN bus before setting CAN\_MBTD.TDA . Once this flag is set, the mailbox can then be disabled ( CAN\_MC1 = 0) without the risk of losing an incoming frame. The CAN\_MBTD.TDR bit must then be reset as soon as possible.

When the CAN\_MBTD.TDA flag is set for a given mailbox, only the data field of that mailbox can be updated. Accesses to the control bits and the identifier are denied.

## Bit Timing

The CAN controller does not have a dedicated clock. Instead, the CAN clock is derived from the system clock based on a configurable number of time quanta. The time quantum (TQ) is derived from the formula:

TQ = (BRP + 1)/CDU0\_CLKO4

where BRP is the 10-bit bit rate prescaler field in the CAN\_CLK register.

Although the CAN\_CLK.BRP field can be set to any value, it is recommended that the value be greater than or equal to 4. Restrictions apply to the bit timing configuration when BRP is less than 4.

The CAN\_CLK register defines the TQ value, and multiple time quanta make up the duration of a CAN bit on the bus. The CAN\_TIMING register controls the nominal bit time and the sample point of the individual bits in the CAN protocol. The Three Phases of a CAN Bit figure shows the three phases of a CAN bit: the synchronization segment, the segment before the sample point, and the segment after the sample point.

Figure 25-11: Three Phases of a CAN Bit

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000010_ad45bd20c769155b84b3babe118fe4f958af53dd40ba5deb34b2a45bd24dc2d2.png)

The synchronization segment is fixed to one TQ. Synchronize the nodes on the bus. All signal edges are expected to occur within this segment.

The CAN\_TIMING.TSEG1 and CAN\_TIMING.TSEG2 fields control how many TQs the CAN bits consist of, resulting in the CAN bit rate. The following formula gives the nominal bit time.

<!-- formula-not-decoded -->

For safe receive operations on given physical networks, the sample point is programmable by the CAN\_TIMING.TSEG1 field. The CAN\_TIMING.TSEG2 field holds the number of TQs to complete the bit time. Often, best sample reliability is achieved with sample points in the high 80% range of the bit time. Never use sample points lower than 50%. Therefore, CAN\_TIMING.TSEG1 must always be greater than or equal to CAN\_TIMING.TSEG2 .

The CAN module does not distinguish between the propagation segment and the phase segment-1 as defined by the standard. The CAN\_TIMING.TSEG1 value is intended to cover both of them. The CAN\_TIMING.TSEG2 value represents the phase segment-2.

If the CAN module detects a recessive-to-dominant edge outside the synchronization segment, it can automatically move the sampling point such that the CAN bit is still handled properly. The synchronization jump width ( CAN\_TIMING.SJW ) field specifies the maximum number of TQs, ranging from 1 to 4 (SJW + 1), allowed for such a resynchronization attempt. The SJW value must not exceed CAN\_TIMING.TSEG2 or CAN\_TIMING.TSEG1 . Therefore, the fundamental rule for writing CAN\_TIMING is:

SJW ≤ TSEG2 ≤ TSEG1

In addition to this fundamental rule, CAN\_TIMING.TSEG2 must also be greater than or equal to the information processing time (IPT). IPT is the time required by the logic to sample the CAN\_RX input, which is 3 system clock cycles.

Therefore, restrictions apply to the minimal value of CAN\_TIMING.TSEG2 if CAN\_CLK.BRP is less than 2. If CAN\_CLK.BRP is set to 0, the CAN\_TIMING.TSEG2 field must be greater than or equal to 2. If CAN\_CLK.BRP is set to 1, the minimum CAN\_TIMING.TSEG2 value is 1.

NOTE: Use the same nominal bit rate for all nodes on a CAN bus.

With all the timing parameters set, the final consideration is sampling performance. The default behavior of the CAN controller is to sample the CAN bit once. The controller samples at the point described by the CAN\_TIMING register and controlled by the CAN\_TIMING.SAM bit. If this bit is set, however, the input signal is oversampled three times at the system clock rate. The resulting value is generated by a majority decision of the three sample values. Always keep the CAN\_TIMING.SAM bit cleared if the BRP value is less than 4.

Do not modify the CAN\_CLK and CAN\_TIMING registers during normal operation. Always enter configuration mode first. Writes to these registers have no effect when CAN is not in configuration or debug mode. If not coming out of processor reset, enter configuration mode by setting the CAN\_CTL.CCR bit and poll the CAN\_STAT register until CAN\_STAT.CCA is set.

NOTE: If the CAN\_TIMING.TSEG1 field is programmed to 0, the module does not leave the configuration mode.

During configuration mode, the module is not active on the CAN bus line. The CAN\_TX output pin remains recessive and the module does not receive or transmit messages or error frames. After leaving the configuration mode, all CAN internal core registers and the CAN error counters are set to their initial values.

A soft reset does not change the values of CAN\_CLK and CAN\_TIMING . Therefore, an ongoing transfer through the CAN bus cannot be corrupted by changing the bit timing parameter or initiating the soft reset (by setting the CAN\_CTL.SRS bit).

## CAN Low Power Features

The CAN module includes built-in sleep and suspend modes to save power.

The following sections describe the behavior of the CAN module in these modes.

## Built-In Suspend Mode

The most modest of power savings mode is the suspend mode. This mode is entered by setting the CAN\_CTL.CSR bit. The module enters the suspend mode after the current operation of the CAN bus finishes. Then, the internal logic sets the CAN\_STAT.CSA bit. Once CAN enters this mode, the module is no longer active on the CAN bus line, slightly reducing power consumption.

In suspend mode, the CAN\_TX output pin remains in a recessive state, and the module does not receive or transmit messages or error frames. The content of the CAN\_CEC register remains unchanged. Clear CAN\_CTL.CSR to exit suspend mode.

The only difference between suspend mode and configuration mode is that the CAN\_CTL and CAN\_STAT registers are not reset when exiting suspend mode.

## Built-In Sleep Mode

The next level of power savings can be realized by using the built-in sleep mode for the module. This mode is entered by setting the CAN\_CTL.SMR bit. The module enters the sleep mode after the current operation of the CAN bus finishes. Once this mode is entered, many of the internal CAN module clocks are shut off, reducing power consumption, and the CAN\_INT.SMACK bit is set.

When the CAN module is in sleep mode, all register reads return the contents of CAN\_INT instead of the usual contents. All register writes, except to CAN\_INT , are ignored in sleep mode. A small part of the module is clocked continuously to allow for waking up out of sleep mode.

A write to the CAN\_INT register ends sleep mode. If the CAN\_CTL.WBA bit is set before entering sleep mode, a dominant bit on the CAN\_RX pin also ends sleep mode. When software sets the CAN\_CTL.SMR bit, hardware sets the CAN\_CTL.CSR bit as well, making sleep mode a super set of suspend mode. When the controller wakes up from sleep mode, hardware automatically clears CAN\_CTL.SMR and CAN\_CTL.CSR . If, however, the controller never enters sleep mode because the wake-up condition was met before CAN\_INT.SMACK bit turns to 1, the CAN\_CTL.SMR and CAN\_CTL.CSR bits do not always automatically clear. Therefore, clear the two bits using software, when returning from sleep mode.

## Soft Reset

The CAN controller features a build-in reset mechanism called soft reset. Soft reset is entered immediately after software has set the CAN\_CTL.SRS bit. Soft reset brings all control registers to a defined state. Mailbox and error registers remain unaffected. Soft reset does not alter the CAN\_TIMING and CAN\_CLK registers and does not disturb the on-going transmission of a currently pending message, acknowledge bit or error frame. However, when recovering from soft reset, software can lose track of transmission or reception reports and interrupt requests.

## CAN Event Control

The following is a description of how CAN generates and controls events.

## CAN Interrupt Signals

The CAN module provides three independent interrupt requests: two mailbox interrupt requests (mailbox receive interrupt request (MBRIRQ) and mailbox transmit interrupt request (MBTIRQ)) and a global CAN status interrupt request (GIRQ). The values of these three interrupt requests can also be read through the CAN\_GIS register.

## Mailbox Interrupts

Each of the 32 mailboxes in the CAN module can generate a receive or transmit interrupt request, depending on the mailbox configuration. To enable a mailbox to generate an interrupt request, set the corresponding CAN\_MBIM1 bit.

If a mailbox is configured as a receive mailbox, the corresponding CAN\_MBRIF1 bit and CAN\_RMP1 bit are set after a received message is stored in mailbox n. When using the feature for automatic remote frame handling ( CAN\_RFH1 =1), the receive interrupt flag is set after the requested data frame is stored in the mailbox.

If any CAN\_MBRIF1 bits are set, the mailbox generates a CAN\_INT.MBRIRQ interrupt request. To clear the CAN\_INT.MBRIRQ interrupt request, software must clear all of the set CAN\_MBRIF1 bits by writing a 1 to those bit locations in CAN\_MBRIF1 . Prior to this operation, software must clear the corresponding CAN\_RMP1 bit.

If a mailbox is configured as a transmit mailbox, the corresponding CAN\_MBTIF1 bit in the transmit interrupt flag is set after the message in mailbox n is sent correctly. The corresponding CAN\_TA1 bit is also set. The CAN\_TA1 bits maintain their state even after the corresponding mailbox n is disabled ( CAN\_MC1 =0). When using the feature for automatic remote frame handling, the transmit interrupt flag is set after the requested data frame is sent from the mailbox.

If any CAN\_MBTIF1.MB bits are set, the MBTIRQ interrupt output is raised in the CAN\_INT register. T o clear the MBTIRQ interrupt request, software must clear all of the bits that are set in the CAN\_MBTIF1 register by writing a 1 to those bit locations. Additionally, software must clear the associated CAN\_TA1 bit or set the associated CAN\_TRS1 bit to clear the interrupt source that asserts the CAN\_MBTIF1 bit.

## Global Interrupt

The global CAN interrupt logic implements with three registers:

- The CAN\_GIM register, where each interrupt source can be enabled or disabled separately
- The CAN\_GIS register
- The CAN\_GIF register

The interrupt mask bits only affect the content of the CAN\_GIF register. If the mask bit is not set in the CAN\_GIM register, the corresponding flag bit is not set when the event occurs. The interrupt status bits in the CAN\_GIS register, however, are always set when the corresponding interrupt event occurs, independent of the mask bits. Thus, the interrupt status bits can be used to poll interrupt events.

The CAN\_INT.GIRQ bit is only asserted if a bit in the CAN\_GIF register is set. The read-only CAN\_INT.GIRQ bit remains set as long as at least 1 bit in CAN\_GIF is set. All bits in the interrupt status and interrupt flag registers remain set until cleared by software or a soft reset has occurred.

- NOTE: The CAN\_GIF register is read-only (RO). In the global CAN interrupt ISR, clear the interrupt latch by writing a 1 to the corresponding bit of the CAN\_GIS register. The operations clear the related bits of the CAN\_GIS and CAN\_GIF registers, as well as the CAN\_INT.GIRQ bit.

There are several interrupt events that can activate this GIRQ interrupt request:

- Access denied event interrupt ( CAN\_GIM.ADIM , CAN\_GIS.ADIS , CAN\_GIF.ADIF ): At least one access to the mailbox RAM occurred during a data update by internal logic.

- Universal counter exceeded event interrupt ( CAN\_GIM.UCEIM , CAN\_GIS.UCEIS , CAN\_GIF.UCEIF ): There is an overflow of the universal counter (in time stamp mode or event counter mode) or the counter has reached the value 0x0000 (in watchdog mode).
- Receive message lost event interrupt ( CAN\_GIM.RMLIM , CAN\_GIS.RMLIS , CAN\_GIF.RMLIF ): A message is received for a mailbox that currently contains unread data. At least 1 bit in the CAN\_RMLn register is set. If the bit in CAN\_GIS and CAN\_GIF registers is cleared and there is at least 1 bit in CAN\_RML1 still set, then the bit in the CAN\_GIS and CAN\_GIF registers is not set again. The internal interrupt source signal is only active if a new bit in CAN\_RML1 is set.
- Abort acknowledge event interrupt ( CAN\_GIM.AAIM , CAN\_GIS.AAIS , CAN\_GIF.AAIF ): At least 1 CAN\_AA1.MB bit in the CAN\_AA1 registers is set. If the bit in the CAN\_GIS and CAN\_GIF registers is cleared and there is at least 1 bit in CAN\_AA1 still set, then the bit in the CAN\_GIS and CAN\_GIF registers is not set again. The internal interrupt source signal is only active if a new bit in CAN\_AA1 is set. The CAN\_AA1.MB bits maintain state even after the corresponding mailbox n is disabled ( CAN\_MC1 = 0).
- Access to unimplemented address event interrupt ( CAN\_GIM.UIAIM , CAN\_GIS.UIAIS , CAN\_GIF.UIAIF ): There was a CPU access to an address which is not implemented in the controller module.
- Wake-up event interrupt ( CAN\_GIM.WUIM , CAN\_GIS.WUIS , CAN\_GIF.WUIF ): The CAN module has left the sleep mode because of detected activity on the CAN bus line.
- Bus-off event interrupt ( CAN\_GIM.BOIM , CAN\_GIS.BOIS , CAN\_GIF.BOIF ): The CAN module has entered the bus-off state. This interrupt source is active if the status of the CAN core changes from normal operation mode to the bus-off mode. If the bit in the CAN\_GIS and CAN\_GIF registers is cleared and the bus-off mode is still active, then this bit is not set again. If the module leaves the bus-off mode, the bit in the CAN\_GIS and CAN\_GIF registers remains set, if not explicitly cleared.
- Error-passive event interrupt ( CAN\_GIM.EPIM , CAN\_GIS.EPIS , CAN\_GIF.EPIF ): The CAN module has entered the error-passive state. This interrupt source is active if the status of the CAN module changes from the error-active mode to the error-passive mode. If the bit in the CAN\_GIS and CAN\_GIF registers is cleared and the error-passive mode is still active, then this bit is not set again. If the module leaves the error-passive mode, the bit in the CAN\_GIS and CAN\_GIF registers remains set, if not explicitly cleared.
- Error warning receive event interrupt ( CAN\_GIM.EWRIM , CAN\_GIS.EWRIS , CAN\_GIF.EWRIF ): The CAN receive error counter ( CAN\_CEC.RXECNT ) has reached the warning limit. If the bit in the CAN\_GIS and CAN\_GIF registers) is cleared and the error warning mode is still active, this bit is not set again. If the module leaves the error warning mode, the bit in the CAN\_GIS and CAN\_GIF registers remains set, if not explicitly cleared.
- Error warning transmit interrupt ( CAN\_GIM.EWTIM , CAN\_GIS.EWTIS , CAN\_GIF.EWTIF ): The CAN transmit error counter ( CAN\_CEC.TXECNT ) has reached the warning limit. If the bit in the CAN\_GIS and CAN\_GIF registers is cleared and the error warning mode is still active, this bit is not set again. If the module leaves the error warning mode, the bit in the CAN\_GIS and CAN\_GIF registers remains set, if not explicitly cleared.

## Event Counter

For diagnostic functions, it is possible to use the universal counter as an event counter. The counter can be programmed in the CAN\_UCCNF [3:0] field to increment on one of these conditions:

- 0x6 - CAN error frame. Counter increments if there is an error frame on the CAN bus line.
- 0x7 - CAN overload frame. Counter increments if there is an overload frame on the CAN bus line.
- 0x8 - Lost arbitration. Counter increments every time arbitration on the CAN line is lost during transmission.
- 0x9 - Transmission aborted. Counter increments every time arbitration is lost or a transmit request is canceled ( CAN\_AA1 is set).
- 0xA - Transmission succeeded. Counter increments every time a message sends without detected errors ( CAN\_TA1 is set).
- 0xB - Receive message rejected. Counter increments every time a message is received without detected errors but not stored in a mailbox because there is no matching identifier found.
- 0xC - Receive message lost. Counter increments every time a message is received without detected errors but not stored in a mailbox because the mailbox contains unread data ( CAN\_RML1 is set).
- 0xD - Message received. Counter increments every time a message is received without detected errors, whether the received message is rejected or stored in a mailbox.
- 0xE - Message stored. Counter increments every time a message is received without detected errors, has an identifier that matches an enabled receive mailbox, and is stored in the receive mailbox ( CAN\_RMP1 is set).
- 0xF - Valid message. Counter increments every time a valid transmit or receive message is detected on the CAN bus line.

## CAN Warnings and Errors

The processor controls CAN warnings and errors using the error counter ( CAN\_CEC ) register, the error status ( CAN\_ESR ) register, and the error counter warning level ( CAN\_EWR ) register. The following sections describe error handling.

## Programmable Warning Limits

Programs can set the warning level for CAN\_GIS.EWTIS and CAN\_GIS.EWRIS separately by writing to the CAN\_EWR.EWLREC and CAN\_EWR.EWLTEC fields. After power-on reset, the CAN\_EWR register is set to the default warning level of 96 for both error counters. After a soft reset, the contents of this register remain unchanged.

## Error Handling

Error management is a part of the CAN standard. Several different kinds of bus errors can occur during transmissions:

- Bit error - Only the transmitting node detects this error. Whenever a node transmits, it continuously monitors its receive pin ( CAN\_RX ) and compares the received bit with the transmitted bit. During the arbitration phase,

the node postpones the transmission if the received and transmitted bits do not match. However, after the arbitration phase, a bit error is signaled any time the value on CAN\_RX does not equal what is transmitted on the CAN\_TX pin. (The arbitration phase completes when the CAN\_MB[nn]\_ID1.RTR bit is sent successfully.)

- Form error. Occurs when a fixed-form bit position in the CAN frame contains one or more illegal bits. Occurs when a dominant bit is detected at a delimiter or end of frame bit position.
- Acknowledge error. Occurs whenever a message is sent and no receivers drive an acknowledge bit.
- CRC error. Occurs whenever a receiver calculates the CRC on the data it received and finds it different than the CRC that transmitted on the bus itself.
- Stuff error. The CAN specification requires the transmitter to insert an extra stuff bit of opposite value after 5 bits have transmitted with the same value. The receiver disregards the value of the stuff bits. However, it takes advantage of the signal edge to resynchronize itself. A stuff error occurs on receiving nodes whenever the sixth consecutive bit value is the same as the previous 5 bits.

Once the CAN module detects any of the errors, it updates the CAN\_ESR and CAN\_CEC registers. In addition to the standard errors, the CAN\_ESR.SAO flag signals when the CAN\_RX pin sticks at dominant level, indicating a possibility of shorted wires.

## Error Frames

It is important that all nodes on the CAN bus ignore data frames that any single node failed to receive. Every node sends an error frame as soon as it has detected an error as shown in the CAN Error Example figure.

A device that has detected an error still completes the ongoing bit. It initiates an error frame by sending six dominant and eight recessive bits to the bus. Since this activity is a violation of the bit stuffing rule, all nodes are signaled to discard the ongoing frame. (All receivers that did not detect the transmission error in the first instance now detect a stuff bit error.)

The transmitter can detect a normal bit error sooner. It aborts the transmission of the ongoing frame and tries resending it later.

When all nodes on the bus have detected the error, they also send six dominant and eight recessive bits to the bus. The resulting error frame consists of two different fields. The first field is the superposition of error flags contributed from the different stations, which are a sequence of 6-12 dominant bits. The second field is the error delimiter and consists of eight recessive bits indicating the end of frame.

Figure 25-12: CAN Error Example

For CRC errors, the error frame initiates at the end of the frame, rather than immediately after the failing bit.

After having received eight recessive bits, every node knows that the error condition is resolved and, if messages are pending, starts transmission. The transmitter that had to abort its operation must win the new arbitration again; otherwise its message is delayed as determined by priority.

Because the transmission of an error frame destroys the frame under transmission, a faulty node erroneously detecting an error can block the bus. So, there are two node states which determine a nodes right to signal an error-erroractive and error-passive.

- Error-active nodes have an error detection rate below a certain limit. These nodes drive an active error flag of six dominant bits.
- Error-passive nodes have a higher error detection rate and can have a local problem and therefore have a limited right to signal errors. These nodes drive a passive error flag consisting of six recessive bits. Therefore, an errorpassive transmitting node is still able to inform the other nodes about the aborting of a self-transmitted frame. But, it is no longer able to destroy correctly received frames of other nodes.

## Error Levels

The CAN specification requires each node in the system to operate at one of three levels. The CAN Error Level Description table describes the levels. This functionality prevents nodes with high error rates from blocking the entire network, as local hardware can cause the errors. The CAN module provides an error counter for transmit (TEC) and an error counter for receive (REC). The CAN\_CEC register contains each of these 8-bit counters.

After initialization, both the TEC and the REC counters are 0. Each time a bus error occurs, one of the counters increments by either 1 or 8, depending on the error situation. Refer to version 2.0 of the CAN specification. Successful transmit or receive operations decrement the respective counter by 1.

If either of the error counters exceeds 127, the CAN module goes into an error-passive state and the CAN\_STAT.EP bit is set. Once this state occurs, the module is not allowed to send any more active error frames. However, the module can still transmit messages and signal passive error frames in case the transmission fails due to bit errors.

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000011_1f6acc7a103fd983ded8afdd3dcfd0bcd6e8f30179028cf90af60df61413010f.png)

If one of the counters exceeds 255 (that is, when an 8-bit counter overflows), the CAN module disconnects from the bus and it goes into bus-off mode. In this mode, the CAN\_STAT.EBO bit is set. Software intervention is needed for recovery from this state, unless the CAN\_CTL.ABO bit is enabled. The bit puts the module into active mode after the bus-off recovery sequence.

Table 25-5: CAN Error Level Description

| Level         | Condition                                                               | Description                                                                                                                            |
|---------------|-------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| Error active  | Transmit and receive error counters <128                                | This level is the initial condition level. As long as errors stay be- low 128, the node drives active error flags during error frames. |
| Error passive | Transmit or receive error counter-value from 128 through 255, inclusive | Errors have accumulated to a level that requires the node to drive passive error flags during error frames                             |
| Bus off       | Transmit or receive error counters greater than 255                     | CAN module goes into bus-off mode                                                                                                      |

In addition to the three levels in the table, the CAN module also generates separate transmit and receive warnings (CAN specification enhancement). By default, when one of the error counters exceeds 96, it signals and reports a warning in the CAN\_STAT register. The CAN receive warning flag ( CAN\_STAT.WR ) bit is set when CAN\_CEC.RXECNT exceeds 96. The CAN transmit warning flag ( CAN\_STAT.WT ) bit is set when CAN\_CEC.TXECNT exceeds 96. The error warning level can be programmed using the error warning register ( CAN\_EWR ).

Additionally, interrupt requests can occur for all of these levels by unmasking them in the global CAN interrupt mask register ( CAN\_GIM ). These sources include: the bus-off interrupt ( CAN\_GIM.BOIM ), the error-passive interrupt ( CAN\_GIM.EPIM ), the error warning receive interrupt ( CAN\_GIM.EWRIM ), and the error warning transmit interrupt ( CAN\_GIM.EWTIM ).

During the bus-off recovery sequence, internal logic sets the configuration mode request CAN\_CTL.CCR bit. The CAN core module does not automatically come out of the bus-off mode. The CAN\_CTL.CCR bit cannot be reset until the bus-off recovery sequence completes.

NOTE: Set the CAN\_CTL.ABO bit to override this behavior. After exiting the bus-off or configuration modes, the CAN error counters are reset.

## CAN Debug and Test Modes

The CAN module contains test mode features that aid in the debugging of the CAN software and system.

NOTE: When using these features, the CAN module does not always comply to the CAN specification. Enable or disable all test modes only when the module is in configuration mode ( CAN\_STAT.CCA =1) or suspend mode ( CAN\_STAT.CSA =1).

The CAN\_DBG.CDE bit provides access to all of the debug features. Set this bit to enable the test mode. Write to the bit first before writing to the CAN\_DBG register. When the CAN\_DBG.CDE bit is cleared, all debug features are disabled.

When the CAN\_DBG.CDE bit is set, it enables writes to the other bits of the CAN\_DBG register. It also enables these features, which are not compliant to the CAN standard:

- Bit timing registers can be changed anytime, not only during configuration mode. The group includes the CAN\_CLK and CAN\_TIMING registers.
- Write access is allowed to the normally read-only CAN\_CEC register.

The following list describes other bits in the debug register.

- The CAN module uses the CAN\_DBG.MRB bit to enable the read back mode. In this mode, a message transmitted on the CAN bus (or through an internal loopback mode) is received back directly to the internal receive buffer. After a correct transmission, the internal logic treats this transfer as a normal receive message. This feature allows testing of most of the CAN features without an external device.
- The CAN\_DBG.MAA bit allows the CAN module to generate its own acknowledge during the ACK slot of the CAN frame. No external devices or connections are necessary to read back a transmit message. In this mode, the sent message is automatically stored in the internal receive buffer. In auto-acknowledge mode, the module itself transmits the acknowledge. This acknowledge can be programmed to appear on the CAN\_TX pin, if CAN\_DBG.DIL =1 and CAN\_DBG.DTO = 0. If the acknowledge is only used internally, then set these test mode bits to CAN\_DBG.DIL = 0 and CAN\_DBG.DTO =1.
- The CAN module uses the CAN\_DBG.DIL bit to internally enable the transmit output to be routed back to the receive input.
- The CAN module uses the CAN\_DBG.DTO bit to disable the CAN\_TX output pin. When this bit is set, the CAN\_TX pin continuously drives recessive bits.
- The CAN module uses the CAN\_DBG.DRI bit to disable the CAN\_RX input. When set, the internal logic receives recessive bits or receives the internally-generated transmit value in the case of the internal loop enabled ( CAN\_DBG.DIL = 0). In either case, the value on the CAN\_RX input pin is ignored.
- The CAN module uses the CAN\_DBG.DEC bit to disable the transmit and receive error counters in the CAN\_CEC register. When this bit is set, the CAN\_CEC holds its current contents and is not allowed to increment or decrement the error counters. This mode does not conform to the CAN specification.

NOTE: Write to the error counter registers in debug mode only. Write-access during reception can lead to undefined values. The maximum value which can be written into the error counters is 255. Therefore, the error counter value of 256, which forces the module into the bus off state, cannot be written into the error counter registers.

Table 25-6: Common CAN Test Mode Bit Combinations

| MRB   | MAA   | DIL   | DTO   | DRI   | CDE   | Functional Description          |
|-------|-------|-------|-------|-------|-------|---------------------------------|
| X     | X     | X     | X     | X     | 0     | Normal mode, not debug mode     |
| 0     | X     | X     | X     | X     | X     | No readback of transmit message |

Table 25-6: Common CAN Test Mode Bit Combinations (Continued)

|   MRB |   MAA |   DIL |   DTO |   DRI |   CDE | Functional Description                                                                                                                                                                                             |
|-------|-------|-------|-------|-------|-------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|     1 |     0 |     1 |     0 |     0 |     1 | Normal transmission on CAN bus line. Read back. External acknowledge from external device required.                                                                                                                |
|     1 |     1 |     1 |     0 |     0 |     1 | Normal transmission on CAN bus line. Read back. No external acknowledge required. Transmit message and acknowledge are transmitted on CAN bus line. CAN_RX input is enabled.                                       |
|     1 |     1 |     0 |     0 |     0 |     1 | Normal transmission on CAN bus line. Read back. No external acknowledge required. Transmit message and acknowledge transmit on CAN bus line. CAN_RX input and internal loop are enabled (internal OR of TX and RX) |
|     1 |     1 |     0 |     0 |     1 |     1 | Normal transmission on CAN bus line. Read back. No external acknowledge required. Transmit message and acknowledge are transmitted on CAN bus line. CAN_RX input is ignored. Internal loop is enabled.             |
|     1 |     1 |     0 |     1 |     1 |     1 | No transmission on CAN bus line. Read back. No external acknowledge required. Nether transmit message nor acknowledge are transmitted on CAN_TX . CAN_RX input is ignored. Internal loop is enabled.               |

## ADSP-SC58x CAN Register Descriptions

Controller Area Network (CAN) contains the following registers.

Table 25-7: ADSP-SC58x CAN Register List

| Name    | Description                  |
|---------|------------------------------|
| CAN_AA1 | Abort Acknowledge 1 Register |

Table 25-7: ADSP-SC58x CAN Register List (Continued)

| Name                 | Description                                |
|----------------------|--------------------------------------------|
| CAN_AA2              | Abort Acknowledge 2 Register               |
| CAN_AM[nn]H          | Acceptance Mask (H) Register               |
| CAN_AM[nn]L          | Acceptance Mask (L) Register               |
| CAN_CEC              | Error Counter Register                     |
| CAN_CLK              | Clock Register                             |
| CAN_CTL              | CAN Master Control Register                |
| CAN_DBG              | Debug Register                             |
| CAN_ESR              | Error Status Register                      |
| CAN_EWR              | Error Counter Warning Level Register       |
| CAN_GIF              | Global CAN Interrupt Flag Register         |
| CAN_GIM              | Global CAN Interrupt Mask Register         |
| CAN_GIS              | Global CAN Interrupt Status Register       |
| CAN_INT              | Interrupt Pending Register                 |
| CAN_MBIM1            | Mailbox Interrupt Mask 1 Register          |
| CAN_MBIM2            | Mailbox Interrupt Mask 2 Register          |
| CAN_MBRIF1           | Mailbox Receive Interrupt Flag 1 Register  |
| CAN_MBRIF2           | Mailbox Receive Interrupt Flag 2 Register  |
| CAN_MBTD             | Temporary Mailbox Disable Register         |
| CAN_MBTIF1           | Mailbox Transmit Interrupt Flag 1 Register |
| CAN_MBTIF2           | Mailbox Transmit Interrupt Flag 2 Register |
| CAN_MB[nn]_DATA0     | Mailbox Word 0 Register                    |
| CAN_MB[nn]_DATA1     | Mailbox Word 1 Register                    |
| CAN_MB[nn]_DATA2     | Mailbox Word 2 Register                    |
| CAN_MB[nn]_DATA3     | Mailbox Word 3 Register                    |
| CAN_MB[nn]_ID0       | Mailbox ID 0 Register                      |
| CAN_MB[nn]_ID1       | Mailbox ID 1 Register                      |
| CAN_MB[nn]_LENGTH    | Mailbox Length Register                    |
| CAN_MB[nn]_TIMESTAMP | Mailbox Time Stamp Register                |
| CAN_MC1              | Mailbox Configuration 1 Register           |
| CAN_MC2              | Mailbox Configuration 2 Register           |
| CAN_MD1              | Mailbox Direction 1 Register               |

Table 25-7: ADSP-SC58x CAN Register List (Continued)

| Name       | Description                                              |
|------------|----------------------------------------------------------|
| CAN_MD2    | Mailbox Direction 2 Register                             |
| CAN_OPSS1  | Overwrite Protection/Single Shot Transmission 1 Register |
| CAN_OPSS2  | Overwrite Protection/Single Shot Transmission 2 Register |
| CAN_RFH1   | Remote Frame Handling 1 Register                         |
| CAN_RFH2   | Remote Frame Handling 2 Register                         |
| CAN_RML1   | Receive Message Lost 1 Register                          |
| CAN_RML2   | Receive Message Lost 2 Register                          |
| CAN_RMP1   | Receive Message Pending 1 Register                       |
| CAN_RMP2   | Receive Message Pending 2 Register                       |
| CAN_STAT   | Status Register                                          |
| CAN_TA1    | Transmission Acknowledge 1 Register                      |
| CAN_TA2    | Transmission Acknowledge 2 Register                      |
| CAN_TIMING | Timing Register                                          |
| CAN_TRR1   | Transmission Request Reset 1 Register                    |
| CAN_TRR2   | Transmission Request Reset 2 Register                    |
| CAN_TRS1   | Transmission Request Set 1 Register                      |
| CAN_TRS2   | Transmission Request Set 2 Register                      |
| CAN_UCCNF  | Universal Counter Configuration Mode Register            |
| CAN_UCCNT  | Universal Counter Register                               |
| CAN_UCRC   | Universal Counter Reload/Capture Register                |

## Abort Acknowledge 1 Register

The CAN\_AA1 register indicates a transmission abort (due to lost arbitration or a CAN error) for mailboxes 8 through 15. Each bit in this register indicates a transmission abort for the corresponding mailbox when set (=1). Bits 0 through 7 are read-only, as the corresponding mailboxes are receive-only mailboxes.

Figure 25-13: CAN\_AA1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000012_23b19cae921efcc23a96519b7d4013b728b010e0ba55beef94acd7ccea81d930.png)

Table 25-8: CAN\_AA1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration      |
|--------------------|------------|------------------------------|
| 15:8               | MB         | Mailbox n Abort Acknowledge. |

## Abort Acknowledge 2 Register

The CAN\_AA2 register indicates a transmission abort (due to lost arbitration or a CAN error) for mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register indicates a transmission abort for the corresponding mailbox when set (=1).

Figure 25-14: CAN\_AA2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000013_83ea607a65c5729a845db05b3abfe20ac2c61863bb9cfc3709b623587109339e.png)

Table 25-9: CAN\_AA2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration      |
|--------------------|------------|------------------------------|
| 15:0               | MB         | Mailbox n Abort Acknowledge. |

## Acceptance Mask (H) Register

The CAN\_AM[nn]H register and CAN\_AM[nn]L register manage acceptance mask operation. For information about acceptance mask operation, see the Receive Operation section.

Figure 25-15: CAN\_AM[nn]H Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000014_d14c51c2c18dd85b6cb51bc95b935ab827babc7600d9587b6ea0ef72fec610b3.png)

Table 25-10: CAN\_AM[nn]H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | FDF        | Filter on Delay Field. The CAN_AM[nn]H.FDF bit selects the operation of the CAN_AM[nn]H register and CAN_AM[nn]L register when the CAN_CTL.DNM bit is enabled. If the CAN_AM[nn]H.FDF bit is set, the corresponding CAN_AM[nn]L.EXTID bits hold the data field mask. If the CAN_AM[nn]H.FDF bit is cleared, the corre- sponding CAN_AM[nn]L.EXTID bits hold the high bits of the extended identifier mask. |
| 14 (R/W)           | FMD        | Full Mask Data. The CAN_AM[nn]H.FMD bit works with the CAN_AM[nn]H.FDF bit to deter- mine data field filtering. For information about data field filtering, see the Receive Op- eration section.                                                                                                                                                                                                           |
| 13 (R/W)           | AMIDE      | Acceptance Mask Identifier Extension. The CAN_AM[nn]H.AMIDE bit enables the comparison of the received message ID to the value in the CAN_AM[nn]H.EXTID and CAN_AM[nn]L.EXTID bits.                                                                                                                                                                                                                        |
| 12:2 (R/W)         | BASEID     | Base Identifier. The CAN_AM[nn]H.BASEID bits hold the base ID for acceptance mask operations.                                                                                                                                                                                                                                                                                                              |
| 1:0 (R/W)          | EXTID      | Extended Identifier. The CAN_AM[nn]H.EXTID bits hold the extended ID (upper two bits) for accept- ance mask operations.                                                                                                                                                                                                                                                                                    |

## Acceptance Mask (L) Register

The CAN\_AM[nn]L register and CAN\_AM[nn]H register manage acceptance mask operation. For information about acceptance mask operation, see the Receive Operation section.

Figure 25-16: CAN\_AM[nn]L Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000015_a7621f9c30c5938a7201f2e9866e73da7e9493ca4a6715fa9e8f8d300e789128.png)

Table 25-11: CAN\_AM[nn]L Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | EXTID      | Extended Identifier/Data Field Mask. The CAN_AM[nn]L.EXTID bits hold the extended ID (lower 16 bits) for data field mask in acceptance mask operations. |

## Error Counter Register

The CAN\_CEC register, CAN\_ESR register, and CAN\_EWR register control CAN warnings and errors. For detailed information about error and warning operations, see the Event Control section.

The CAN\_CEC register holds an error counter for transmit ( CAN\_CEC.TXECNT ) and an error counter for receive ( CAN\_CEC.RXECNT ). After initialization, both counters are 0. Each time a bus error occurs, one of the counters is incremented by either 1 or 8, depending on the error situation (documented in Version 2.0 of CAN Specification). Successful transmit and receive operations decrement the respective counter by 1.

Figure 25-17: CAN\_CEC Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000016_a3dbfcfe0306871377a6ea80cb2ead60f504eb655e33219f712923d5fff42495.png)

Table 25-12: CAN\_CEC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | TXECNT     | Transmit Error Counter. The CAN_CEC.TXECNT bits hold the transmit error counter, which is incremented for errors (by either 1 or 8) and is decremented (by 1) for successful transmit opera- tions. |
| 7:0 (R/W)          | RXECNT     | Receive Error Counter. The CAN_CEC.RXECNT bits hold the receive error counter, which is incremented for errors (by either 1 or 8) and is decremented (by 1) for successful receive operations.      |

## Clock Register

The CAN\_CLK register selects the bit rate prescaler for calculating the time quantum (TQ), which is used to derive the CAN clock from the system clock (CDU0\_CLKO4). For more information about bit timing and clock operation, see the CAN Operating Modes section.

Figure 25-18: CAN\_CLK Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000017_eaafa50379861d2b466c6623072cfb1adfff4aded3a2fda7d3b7e122f738deac.png)

Table 25-13: CAN\_CLK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:0 (R/W)          | BRP        | Bit Rate Prescaler. The CAN_CLK.BRP bits select the bit rate prescaler value, which is used to calculate the time quantum for CAN bit timing. The formula using CAN_CLK.BRP to calcu- late the time quantum is: TQ = (BRP+1) / CDU0_CLKO4 Note that it is recommended that the CAN_CLK.BRP value be greater than or equal to 4. For more information about bit timing, see the Operating Modes section. |

## CAN Master Control Register

The CAN\_CTL register controls CAN mode requests, including soft reset.

Figure 25-19: CAN\_CTL Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000018_7dea054b29a4ec3233b28be76ea61bcfb8dd0a2a28fa338214fc0052607ae0db.png)

Table 25-14: CAN\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | CCR        | CAN Configuration Mode Request. The CAN_CTL.CCR bit requests that the CAN enter configuration mode. Note that the CAN should always be put in configuration mode before modifying the CAN_CLK or CAN_TIMING registers. 0 No Request (Exit Configuration Mode) |
| 6 (R/W)            | CSR        | CAN Suspend Mode Request. The CAN_CTL.CSR bit requests that the CAN enter suspend mode. The CAN enters suspend mode after the current operation of the CAN bus is finished.                                                                                   |
| 5 (R/W)            | SMR        | Sleep Mode Request. The CAN_CTL.SMR bit requests that the CAN enter sleep mode. The CAN enters sleep mode after the current operation of the CAN bus is finished. 0 No Request (Exit Sleep Mode)                                                              |

Table 25-14: CAN\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | WBA        | Wake Up on CAN Bus Activity. The CAN_CTL.WBA bit enables wake on CAN bus activity. When enabled, a domi- nant bit on the CAN_RX pin ends sleep mode (also, the default wake up condition of a write to the CAN_INT register). |
| 2 (R/W)            | ABO        | Auto Bus On. The CAN_CTL.ABO bit selects whether (if enabled) the CAN enters active mode af- ter the bus-off recovery sequence or (if disabled) the CAN enters configuration mode after the bus-off recovery sequence.        |
| 1 (R/W)            | DNM        | Device Net Mode. The CAN_CTL.DNM bit enables mailbox filtering on a data field. The filtering is done on the standard ID of the message and data fields. For more information, see the CAN_AM[nn]H.FDF bit description.       |
| 0 (R/W)            | SRS        | Software Reset. The CAN_CTL.SRS bit resets the CAN, bringing all control registers to a defined state. Soft reset is entered immediately after software has set the CAN_CTL.SRS bit. 0 No Action                              |

## Debug Register

The CAN\_DBG register controls CAN debug modes, including CAN\_TX and CAN\_RX pin enable and disable.

Figure 25-20: CAN\_DBG Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000019_d8188c48c28830b1ba339cd785cc9163e16d35beb6646b34d2bbb57c68fd5f23.png)

Table 25-15: CAN\_DBG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | CDE        | CAN Debug Mode Enable. The CAN_DBG.CDE bit enables debug mode. This bit must be written first before subsequent writes to the CAN_DBG register. When the CAN_DBG.CDE bit is cleared, all CAN debug features are disabled.                                                                                                                                                                        | CAN Debug Mode Enable. The CAN_DBG.CDE bit enables debug mode. This bit must be written first before subsequent writes to the CAN_DBG register. When the CAN_DBG.CDE bit is cleared, all CAN debug features are disabled.                                                                                                                                                                        |
| 15 (R/W)           | CDE        | 0                                                                                                                                                                                                                                                                                                                                                                                                | Disable Debug Mode                                                                                                                                                                                                                                                                                                                                                                               |
| 15 (R/W)           | CDE        | 1                                                                                                                                                                                                                                                                                                                                                                                                | Enable Debug Mode                                                                                                                                                                                                                                                                                                                                                                                |
| 5 (R/W)            | MRB        | Mode Read Back. The CAN_DBG.MRB bit enables read back mode. When enabled, a message transmit- ted on the CAN bus or through an internal loop back mode is received back directly to the internal receive buffer.                                                                                                                                                                                 | Mode Read Back. The CAN_DBG.MRB bit enables read back mode. When enabled, a message transmit- ted on the CAN bus or through an internal loop back mode is received back directly to the internal receive buffer.                                                                                                                                                                                 |
| 5 (R/W)            | MRB        | 0                                                                                                                                                                                                                                                                                                                                                                                                | Disable Read Back Mode                                                                                                                                                                                                                                                                                                                                                                           |
| 5 (R/W)            | MRB        | 1                                                                                                                                                                                                                                                                                                                                                                                                | Enable Read Back Mode                                                                                                                                                                                                                                                                                                                                                                            |
| 4 (R/W)            | MAA        | Mode Auto Acknowledge. The CAN_DBG.MAA bit enables auto acknowledge mode, allowing the CAN to gen- erate its own acknowledge during the ACK slot of the CAN frame. The CAN_DBG.MAA acknowledge appears on the CAN_TX pin if CAN_DBG.DIL =1 and CAN_DBG.DTO =0. If the acknowledge is only going to be used internally, these test mode bits should be set to CAN_DBG.DIL = 0 and CAN_DBG.DTO =1. | Mode Auto Acknowledge. The CAN_DBG.MAA bit enables auto acknowledge mode, allowing the CAN to gen- erate its own acknowledge during the ACK slot of the CAN frame. The CAN_DBG.MAA acknowledge appears on the CAN_TX pin if CAN_DBG.DIL =1 and CAN_DBG.DTO =0. If the acknowledge is only going to be used internally, these test mode bits should be set to CAN_DBG.DIL = 0 and CAN_DBG.DTO =1. |
| 4 (R/W)            | MAA        | 0                                                                                                                                                                                                                                                                                                                                                                                                | Disable Auto Acknowledge Mode                                                                                                                                                                                                                                                                                                                                                                    |
| 4 (R/W)            | MAA        | 1                                                                                                                                                                                                                                                                                                                                                                                                | Enable Auto Acknowledge Mode                                                                                                                                                                                                                                                                                                                                                                     |

Table 25-15: CAN\_DBG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | DIL        | Disable Internal Loop. The CAN_DBG.DIL bit disables internal loop mode, which routes the transmit out- put to the receive input.                                                                                                                                                                                                                                    |
| 2 (R/W)            | DTO        | Disable Tx Output Pin. The CAN_DBG.DTO bit disables the CAN_TX pin. 0 Enable Tx Output Pin                                                                                                                                                                                                                                                                          |
| 1 (R/W)            | DRI        | Disable Receive Input Pin. The CAN_DBG.DRI bit disables the CAN_RX pin. 0 Enable Rx Input Pin 1 Disable Rx Input Pin, Drive Recessive Internally                                                                                                                                                                                                                    |
| 0 (R/W)            | DEC        | Disable Transmit and Receive Error Counters. The CAN_DBG.DEC bit disables the transmit and receive error counters in the CAN_CEC register. When set, the CAN_CEC holds its current contents and is not al- lowed to increment or decrement the error counters. Note that this mode does not conform to the CAN specification. 0 Enable CEC Tx and Rx Error Counters |
| 0 (R/W)            |            | 1 Disable CEC Tx and Rx Error Counters                                                                                                                                                                                                                                                                                                                              |

## Error Status Register

The CAN\_ESR register, CAN\_CEC register, and CAN\_EWR register control CAN warnings and errors. All bits in the CAN\_ESR register are W1C. Note that the CAN updates the CAN\_CEC register when error status is detected in the CAN\_ESR register. For detailed information about error and warning operations, see the Operating Modes section.

Figure 25-21: CAN\_ESR Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000020_91b2f39742a3a85501044f8c06c7eec91a9b2c1c33b3bd62028c94f7ee0bd0b1.png)

Table 25-16: CAN\_ESR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | FER        | Form Error. The CAN_ESR.FER bit indicates when a form error occurs, indicating that a fixed- form bit position in the CAN frame contains one or more illegal bits. This occurs when a dominant bit is detected at a delimiter or end-of-frame bit position. 0 No Status 1 Form Error                                                                                                                                                                                                                                                                                                                                                                                              |
| 6 (R/W1C)          | BEF        | Bit Error Flag. The CAN_ESR.BEF bit indicates (detected by the transmitting node only) when the value on the CAN_RX pin does not equal what is being transmitted on the CAN_TX pin. When a node is transmitting, it continuously monitors its receive pin ( CAN_RX ) and compares the received data with the transmitted data. The node postpones the trans- mission (during the arbitration phase) if the received and transmitted data do not match. After the arbitration phase ( CAN_MB[nn]_ID1.RTR bit sent successfully), a bit error is signaled when the value on the CAN_RX pin does not equal what is being transmitted on the CAN_TX pin. 0 No Status 1 Bit Error Flag |

Table 25-16: CAN\_ESR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W1C)          | SAO        | Stuck at Dominant. The CAN_ESR.SAO bit indicates when the CAN_RX pin sticks at dominant level, in- dicating that shorted wires are likely.                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 5 (R/W1C)          | SAO        | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 4 (R/W1C)          | CRCE       | CRC Error. The CAN_ESR.CRCE bit indicates when a CRC error occurs. This error may occur when a receiver calculates the CRC on the data it received and finds the value different than the CRC that was transmitted on the bus.                                                                                                                                                                                                                                                                                                                                                  |
| 4 (R/W1C)          | CRCE       | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 4 (R/W1C)          | CRCE       | 1 CRC Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 3 (R/W1C)          | SER        | Stuff Bit Error. The CAN_ESR.SER bit indicates when a stuff bit error (stuffed 6th consecutive bit value is the same as the previous five bits) occurs. The CAN specification requires that the transmitter insert an extra stuff bit of opposite value after 5 bits have been transmitted with the same value. The receiver disregards the value of these stuff bits. The receiver takes advantage of the signal edge to re-syn- chronize itself. A stuff bit error occurs on receiving nodes when the 6th consecutive bit value is the same as the previous five bits. Status |
| 3 (R/W1C)          | SER        | 0 No                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 3 (R/W1C)          | SER        | 1 Stuff Bit Error Receive                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 2 (R/W1C)          | ACKE       | Acknowledge Error. The CAN_ESR.ACKE bit indicates when an acknowledge error occurs, indicating that a message is sent and no receivers drive an acknowledge bit.                                                                                                                                                                                                                                                                                                                                                                                                                |
| 2 (R/W1C)          | ACKE       | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 2 (R/W1C)          | ACKE       | 1 Acknowledge Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

## Error Counter Warning Level Register

The CAN\_EWR register, CAN\_CEC register, and CAN\_ESR register control CAN warnings and errors. For detailed information about error and warning operations, see the Operating Modes section.

Figure 25-22: CAN\_EWR Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000021_b6746b0fbae332a0e3372ef1dc80f812068f36163b2b61a49790d376672a4401.png)

Table 25-17: CAN\_EWR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | EWLTEC     | Transmit Error Warning Limit. The CAN_EWR.EWLTEC bits select the transmit error warning limit, which is used as a condition for the CAN_GIS.EWTIS interrupt. |
| 7:0 (R/W)          | EWLREC     | Receive Error Warning Limit. The CAN_EWR.EWLREC bits select the receive error warning limit, which is used as a condition for the CAN_GIS.EWRIS interrupt.   |

## Global CAN Interrupt Flag Register

The CAN\_GIF register, CAN\_GIF register, and CAN\_GIM register control CAN interrupt requests. For detailed information about interrupt operations, see the Event Control section.

The CAN\_GIF register holds the interrupt flag. The CAN\_INT.GIRQ bit is only asserted if a bit in the CAN\_GIF is set. The CAN\_INT.GIRQ bit remains set as long as at least one bit in the CAN\_GIF register is set. All bits in this register are W1C.

Figure 25-23: CAN\_GIF Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000022_a9239157ac2dd3f67700d8bbfd3721fbc1ff3ff67440622febd427cbd810d7d1.png)

Table 25-18: CAN\_GIF Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                         |                              |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------|
| 10 (R/NW)          | ADIF       | Access Denied Interrupt Flag. The CAN_GIF.ADIF bit indicates that the access denied interrupt flag is set (latch-                               |                              |
| 8 (R/NW)           | UCEIF      | Universal Counter Exceeded Interrupt Flag. The CAN_GIF.UCEIF bit indicates that the universal counter exceeded interrupt flag is set (latched). |                              |
|                    |            | 0                                                                                                                                               | No Interrupt Flag            |
|                    |            | 1                                                                                                                                               | Interrupt Flag Set (Latched) |
|                    |            | 1                                                                                                                                               | Interrupt Flag Set (Latched) |

Table 25-18: CAN\_GIF Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | RMLIF      | Receive Message Lost Interrupt Flag. The CAN_GIF.RMLIF bit indicates that the receive message lost interrupt flag is set (latched).   |
| 7 (R/NW)           | RMLIF      | 0 No Interrupt Flag                                                                                                                   |
| 7 (R/NW)           | RMLIF      | 1 Interrupt Flag Set (Latched)                                                                                                        |
| 6 (R/NW)           | AAIF       | Abort Acknowledge Interrupt Flag. The CAN_GIF.AAIF bit indicates that the abort acknowledge interrupt flag is set (latched).          |
| 6 (R/NW)           | AAIF       | 0 No Interrupt Flag                                                                                                                   |
| 6 (R/NW)           | AAIF       | 1 Interrupt Flag Set (Latched)                                                                                                        |
| 5 (R/NW)           | UIAIF      | Unimplemented Address Interrupt Flag. The CAN_GIF.UIAIF bit indicates that the unimplemented address interrupt flag is                |
| 5 (R/NW)           | UIAIF      | 0 No Interrupt Flag                                                                                                                   |
| 5 (R/NW)           | UIAIF      | 1 Interrupt Flag Set (Latched)                                                                                                        |
| 4 (R/NW)           | WUIF       | Wake Up Interrupt Flag. The CAN_GIF.WUIF bit indicates that the wake up interrupt flag is set (latched).                              |
| 4 (R/NW)           | WUIF       | 0 No Interrupt Flag                                                                                                                   |
| 3 (R/NW)           | BOIF       | Bus Off Interrupt Flag. The CAN_GIF.BOIF bit indicates that the bus off interrupt flag is set (latched).                              |
| 3 (R/NW)           | BOIF       | 0 No Interrupt Flag                                                                                                                   |
| 3 (R/NW)           | BOIF       | 1 Interrupt Flag Set (Latched)                                                                                                        |
| 2 (R/NW)           | EPIF       | Error Passive Interrupt Flag. The CAN_GIF.EPIF bit indicates that the error passive mode interrupt flag is set (latched).             |
| 2 (R/NW)           | EPIF       | 0 No Interrupt Flag                                                                                                                   |
| 2 (R/NW)           | EPIF       | 1 Interrupt Flag Set (Latched)                                                                                                        |
| 1 (R/NW)           | EWRIF      | Error Warning Receive Interrupt Flag. The CAN_GIF.EWRIF bit indicates that the error warning receive interrupt flag is set (latched). |
| 1 (R/NW)           | EWRIF      | 0 No Interrupt Flag                                                                                                                   |
| 1 (R/NW)           | EWRIF      | 1 Interrupt Flag Set (Latched)                                                                                                        |

Table 25-18: CAN\_GIF Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | EWTIF      | Error Warning Transmit Interrupt Flag. The CAN_GIF.EWTIF bit indicates that the error warning transmit interrupt flag is set (latched). |

## Global CAN Interrupt Mask Register

The CAN\_GIM register, CAN\_GIF register, and CAN\_GIF register control CAN interrupt requests. For detailed information about interrupt operations, see the Event Control section.

The CAN\_GIM register holds the interrupt mask. The interrupt mask bits only affect the content of the CAN\_GIF register. If the mask bit is not set (enabled/unmasked), the corresponding flag bit is not set when the event occurs.

Figure 25-24: CAN\_GIM Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000023_07f095bdda98be97e635e2666e91ab3d461660a5b36dd766ef856f9cda111940.png)

Table 25-19: CAN\_GIM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | ADIM       | Access Denied Interrupt Mask. The CAN_GIM.ADIM bit enables (unmasks) the access denied interrupt request.                                              |
| 8 (R/W)            | UCEIM      | Universal Counter Exceeded Interrupt Mask. The CAN_GIM.UCEIM bit enables (unmasks) the universal counter exceeded inter- rupt request.                 |
| 7 (R/W)            | RMLIM      | 1 Enable Interrupt (Unmask) Receive Message Lost Interrupt Mask. The CAN_GIM.RMLIM bit enables (unmasks) the receive message lost interrupt re- quest. |
| 7 (R/W)            |            | 0 Disable Interrupt (Mask)                                                                                                                             |

Table 25-19: CAN\_GIM Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | AAIM       | Abort Acknowledge Interrupt Mask. The CAN_GIM.AAIM bit enables (unmasks) the abort acknowledge interrupt request.                                                                   |
| 5 (R/W)            | UIAIM      | Unimplemented Address Interrupt Mask. The CAN_GIM.UIAIM bit enables (unmasks) the unimplemented address interrupt request.                                                          |
| 4 (R/W)            | WUIM       | Wake Up Interrupt Mask. The CAN_GIM.WUIM bit enables (unmasks) the wake up interrupt request. 0 Disable Interrupt (Mask)                                                            |
| 3 (R/W)            | BOIM       | Bus Off Interrupt Mask. The CAN_GIM.BOIM bit enables (unmasks) the bus off interrupt request. 0 Disable Interrupt (Mask)                                                            |
| 2 (R/W)            | EPIM       | 1 Enable Interrupt (Unmask)                                                                                                                                                         |
|                    |            | Error Passive Interrupt Mask. The CAN_GIM.EPIM bit enables (unmasks) the error passive mode interrupt request. 0 Disable Interrupt (Mask) 1 Enable Interrupt (Unmask)               |
| 1 (R/W)            | EWRIM      | Error Warning Receive Interrupt Mask. The CAN_GIM.EWRIM bit enables (unmasks) the error warning receive interrupt re- quest. 0 Disable Interrupt (Mask) 1 Enable Interrupt (Unmask) |
| 0 (R/W)            | EWTIM      | Error Warning Transmit Interrupt Mask. The CAN_GIM.EWTIM bit enables (unmasks) the error warning transmit interrupt request. 0 Disable Interrupt (Mask)                             |

## Global CAN Interrupt Status Register

The CAN\_GIS register, CAN\_GIF register, and CAN\_GIM register control CAN interrupt requests. For detailed information about interrupt operations, see the Event Control section.

The CAN\_GIS register holds the interrupt status. All bits in this register are W1C.

Figure 25-25: CAN\_GIS Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000024_9ea34abc4c138920ac2b6207cb46e7f1c8581108929ceb5a27d2383c932a03b4.png)

Table 25-20: CAN\_GIS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W1C)         | ADIS       | Access Denied Interrupt Status. The CAN_GIS.ADIS bit indicates when at least one access to the mailbox RAM oc- curred during a data update by internal logic. 0 No Interrupt Pending                                                                                                        |
| 8 (R/W1C)          | UCEIS      | Universal Counter Exceeded Interrupt Status. The CAN_GIS.UCEIS bit indicates when there has been an overflow of the universal counter (in time stamp mode or event counter mode) or the counter has reached the value 0x0000 (in watchdog mode). 0 No Interrupt Pending 1 Interrupt Pending |

Table 25-20: CAN\_GIS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | RMLIS      | Receive Message Lost Interrupt Status. The CAN_GIS.RMLIS bit indicates when a message is received for a mailbox that currently contains unread data. At least one bit in the receive message lost register ( CAN_RML1 or CAN_RML2 ) is set. If the bit in CAN_GIS (and CAN_GIF ) is reset and there is at least one bit in CAN_RML1 or CAN_RML2 still set, the bit in CAN_GIF (and CAN_GIF ) is not set again. The internal interrupt source signal is on- ly active if a new bit in CAN_RML1 or CAN_RML2 is set. |
| 6 (R/W1C)          | AAIS       | 1 Interrupt Pending Abort Acknowledge Interrupt Status. The CAN_GIS.AAIS bit indicates when At least one abort acknowledge bit is set in the CAN_AA1 or the CAN_AA2 registers. If the bit in CAN_GIS (and CAN_GIF ) is reset and there is at least one bit in CAN_AA1 or CAN_AA2 still set, the bit in CAN_GIS (and CAN_GIF ) is not set again. The internal interrupt source signal is on- ly active if a new bit in CAN_AA1 or CAN_AA2 is set. The abort acknowledge bits                                       |
| 5 (R/W1C)          | UIAIS      | Unimplemented Address Interrupt Status. The CAN_GIS.UIAIS bit indicates when there was a processor core access to an ad- dress that is not implemented in the CAN.                                                                                                                                                                                                                                                                                                                                                |
| 4 (R/W1C)          | WUIS       | 0 No Interrupt Pending 1 Interrupt Pending Wake Up Interrupt Status.                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | The CAN_GIS.WUIS bit indicates when the CAN has left the sleep mode because of detected activity on the CAN bus line. 0 No Interrupt Pending 1 Interrupt Pending                                                                                                                                                                                                                                                                                                                                                  |
| 3 (R/W1C)          | BOIS       | Bus Off Interrupt Status. The CAN_GIS.BOIS bit indicates when the CAN has entered the bus-off state. This interrupt source is active if the status of the CAN changes from normal operation mode to the bus-off mode. If the bit in CAN_GIS (and CAN_GIF ) is reset and the bus-off mode is still active, this bit is not set again. If the module leaves the bus-off mode, the bit in CAN_GIS (and CAN_GIF ) remains set. 0 No Interrupt Pending                                                                 |

Table 25-20: CAN\_GIS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | EPIS       | Error Passive Interrupt Status. The CAN_GIS.EPIS bit indicates when the CAN has entered the error passive state. This interrupt source is active if the status of the CAN changes from the error active mode to the error passive mode. If the bit in CAN_GIS (and CAN_GIF ) is reset and the error passive mode is still active, this bit is not set again. If the CAN leaves the error passive mode, the bit in CAN_GIS (and CAN_GIF ) remains set. |
| 1 (R/W1C)          | EWRIS      | Error Warning Receive Interrupt Status. The CAN_GIS.EWRIS bit indicates when the CAN_CEC.RXECNT has reached the warning limit. If the bit in CAN_GIS (and CAN_GIF ) is reset and the error warning mode is still active, this bit is not set again. If the CAN leaves the error warning mode, the bit in CAN_GIS (and CAN_GIF ) remains set.                                                                                                          |
| 0 (R/W1C)          | EWTIS      | Error Warning Transmit Interrupt Status. The CAN_GIS.EWTIS bit indicates when the CAN_CEC.TXECNT has reached the warning limit. If the bit in CAN_GIS (and CAN_GIF ) is reset and the error warning mode is still active, this bit is not set again. If the CAN leaves the error warning mode, the bit in CAN_GIS (and CAN_GIF ) remains set.                                                                                                         |
| 0 (R/W1C)          | EWTIS      | 0 No Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 0 (R/W1C)          | EWTIS      | 1 Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 0 (R/W1C)          | EWTIS      |                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

## Interrupt Pending Register

The CAN\_INT register indicates the status of pending CAN interrupts and indicates the state of the CAN\_RX and CAN\_TX pins. Though this register is read-only, a write is allowed to exit the built-in sleep mode of the module on processors supporting this feature.

Figure 25-26: CAN\_INT Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000025_2e38c05c0e8de648b57fcf4d5c7e02cae6997a3ce43ae848378879b775334d25.png)

Table 25-21: CAN\_INT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | CANRX      | Serial Input From Transceiver. The CAN_INT.CANRX bit indicates the logic value that the CAN detects on the CAN_RX pin. Note that the reset/default value for CAN_INT.CANRX is dependent on pin values. | Serial Input From Transceiver. The CAN_INT.CANRX bit indicates the logic value that the CAN detects on the CAN_RX pin. Note that the reset/default value for CAN_INT.CANRX is dependent on pin values. |
| 7 (R/NW)           | CANRX      | 0                                                                                                                                                                                                      | Dominant Value (Low Active)                                                                                                                                                                            |
| 7 (R/NW)           | CANRX      | 1                                                                                                                                                                                                      | Recessive Value (High Active)                                                                                                                                                                          |
| 6 (R/NW)           | CANTX      | Serial Input To Transceiver. The CAN_INT.CANTX bit indicates the logic value that the CAN detects on the CAN_TX pin. Note that the reset/default value for CAN_INT.CANTX is dependent on pin values.   | Serial Input To Transceiver. The CAN_INT.CANTX bit indicates the logic value that the CAN detects on the CAN_TX pin. Note that the reset/default value for CAN_INT.CANTX is dependent on pin values.   |
| 6 (R/NW)           | CANTX      | 0                                                                                                                                                                                                      | Dominant Value (Low Active)                                                                                                                                                                            |
| 6 (R/NW)           | CANTX      | 1                                                                                                                                                                                                      | Recessive Value (High Active)                                                                                                                                                                          |
| 3 (R/W)            | SMACK      | Sleep Mode Acknowledge. The CAN_INT.SMACK bit indicates when the CAN has entered sleep mode.                                                                                                           | Sleep Mode Acknowledge. The CAN_INT.SMACK bit indicates when the CAN has entered sleep mode.                                                                                                           |
| 3 (R/W)            | SMACK      | 0                                                                                                                                                                                                      | Not in Sleep Mode                                                                                                                                                                                      |
| 3 (R/W)            | SMACK      | 1                                                                                                                                                                                                      | Sleep Mode                                                                                                                                                                                             |

Table 25-21: CAN\_INT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | GIRQ       | Global CAN Interrupt Output. The CAN_INT.GIRQ bit indicates when at least one bit is set in the CAN_GIF regis- ter, indicating at least one unmasked CAN is flagged (latched). The CAN_INT.GIRQ bit remains set as long as at least one bit is set in the CAN_GIF register. 0 No CAN Global Interrupt Flag Set |
| 1 (R/W)            | MBTIRQ     | Mailbox Transmit Interrupt Output. The CAN_INT.MBTIRQ bit indicates when any bits are set in the CAN_MBTIF1 register or CAN_MBTIF2 register, indicating transmit.                                                                                                                                              |
| 0 (R/W)            | MBRIRQ     | Mailbox Receive Interrupt Output. The CAN_INT.MBRIRQ bit indicates when any bits are set in the CAN_MBRIF1 register or CAN_MBRIF2 register, indicating receive. 0 No CAN Receive Flags Set                                                                                                                     |
| 0 (R/W)            | MBRIRQ     | 1 CAN Receive Flags Set (1 or More)                                                                                                                                                                                                                                                                            |
| 0 (R/W)            | MBRIRQ     |                                                                                                                                                                                                                                                                                                                |
| 0 (R/W)            | MBRIRQ     |                                                                                                                                                                                                                                                                                                                |

## Mailbox Interrupt Mask 1 Register

The CAN\_MBIM1 register enables transmit and receive interrupt requests for mailboxes 0 through 15. Each bit in this register requests enables the transmit or receive interrupt request for the corresponding mailbox when set (=1).

|   15 |   14 |   13 |   12 |   11 |   10 |   9 |   8 |   7 |   6 |   5 |   4 |   3 |   2 |   1 |   0 |
|------|------|------|------|------|------|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
|    0 |    0 |    0 |    0 |    0 |    0 |   0 |   0 |   0 |   0 |   0 |   0 |   0 |   0 |   0 |   0 |

Figure 25-27: CAN\_MBIM1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000026_bed5dee392a373e57f363a525c04e6775b24180592f215b7f92bcf8145e0b1ad.png)

Table 25-22: CAN\_MBIM1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                  |
|--------------------|------------|----------------------------------------------------------|
| 15:0               | MB         | Mailbox n Transmit and Receive Interrupt Request Enable. |

## Mailbox Interrupt Mask 2 Register

The CAN\_MBIM2 register enables transmit and receive interrupt requests for mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register requests enables the transmit or receive interrupt request for the corresponding mailbox when set (=1).

Figure 25-28: CAN\_MBIM2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000027_f3eedf6461b65132a0d6473d195b489bc905b156e8829751b2d526158b94caaf.png)

Table 25-23: CAN\_MBIM2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                          |
|--------------------|------------|--------------------------------------------------|
| 15:0               | MB         | Mailbox n Transmit and Receive Interrupt Enable. |

## Mailbox Receive Interrupt Flag 1 Register

The CAN\_MBRIF1 register indicates when a receive interrupt request is pending---due to successful reception (corresponding CAN\_RMP1 bit set) and the interrupt is enabled (corresponding CAN\_MBIM1 bit set)---for mailboxes 0 through 15. Each bit in this register indicates the receive interrupt pending status for the corresponding mailbox when set (=1). When any bit in CAN\_MBRIF1 is set, the CAN receive interrupt request is raised ( CAN\_INT.MBRIRQ bit set). T o clear the interrupt request, all of the set bits in CAN\_RMP1 must be cleared by software, then the associated bits set in CAN\_MBRIF1 must be cleared (W1C).

Figure 25-29: CAN\_MBRIF1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000028_bb59c0b864577267d5f97d8ec71b961600e9eb4e99af94165dbd5add0964f013.png)

Table 25-24: CAN\_MBRIF1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                      |
|--------------------|------------|----------------------------------------------|
| 15:0               | MB         | Mailbox n Receive Interrupt Request Pending. |

## Mailbox Receive Interrupt Flag 2 Register

The CAN\_MBRIF2 register indicates when a receive interrupt request is pending---due to successful reception (corresponding CAN\_RMP2 bit set) and the interrupt is enabled (corresponding CAN\_MBIM2 bit set)---for mailboxes 16 (bit 0) through 23 (bit 7). Each bit in this register indicates the receive interrupt pending status for the corresponding mailbox when set (=1). When any bit in CAN\_MBRIF2 is set, the CAN receive interrupt request is raised ( CAN\_INT.MBRIRQ bit set). T o clear the interrupt request, all of the set bits in CAN\_RMP2 must be cleared by software, then the associated bits set in CAN\_MBRIF2 must be cleared (W1C). Bits 8 through 15 are reserved and read-only, as the corresponding mailboxes (24 through 31) are transmit-only mailboxes.

Figure 25-30: CAN\_MBRIF2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000029_9a120fad0071576b6ded032177e14afd4c35b712c290d8c42ad38aca07a5c398.png)

Table 25-25: CAN\_MBRIF2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 7:0                | MB         | Mailbox n Receive Interrupt Pending. |
| (R/W1C)            |            |                                      |

## Temporary Mailbox Disable Register

The CAN\_MBTD register supports temporarily and selectively disabling CAN mailboxes. For more information about this feature, see the Operating Modes section.

Figure 25-31: CAN\_MBTD Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000030_61cd08b68fafb922719d171f22078dee837b760e694ea9cfa3105c15e4b4cae9.png)

Table 25-26: CAN\_MBTD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | TDR        | Temporary Disable Request. The CAN_MBTD.TDR bit holds the pointer to mailbox, which is disabled when the CAN_MBTD.TDR bit is set. 0 No Request                                                                                                                                                  |
| 6 (R/NW)           | TDA        | Temporary Disable Acknowledge. The CAN_MBTD.TDA bit indicates when the mailbox (to which the CAN_MBTD.TDPTR bit points) is disabled. When this bit is set for a mailbox, only the data field of that mailbox may be updated. Accesses that mailboxs control bits and the identifier are denied. |
| 4:0 (R/W)          | TDPTR      | Temporary Disable Pointer. The CAN_MBTD.TDPTR bits hold the pointer to mailbox, which is disabled when the CAN_MBTD.TDR bit is set.                                                                                                                                                             |

## Mailbox Transmit Interrupt Flag 1 Register

The CAN\_MBTIF1 register indicates when a transmit interrupt request is pending---due to successful transmission (corresponding CAN\_TA1 bit is set) and the interrupt is enabled (corresponding CAN\_MBIM1 bit is set)---for mailboxes 8 through 15. Each bit in this register indicates the transmit interrupt pending status for the corresponding mailbox when set (=1). When any bit in CAN\_MBTIF1 is set, the CAN transmit interrupt request is raised ( CAN\_INT.MBTIRQ bit set). T o clear the interrupt request, all of the set bits in CAN\_MBTIF1 must be cleared by software (W1C). Also, software must clear the associated bits set in CAN\_TA1 or set the associated bits in CAN\_TRS1 bit to clear the interrupt source asserting the bits in CAN\_MBTIF1 . Bits 0 through 7 are read-only, as the corresponding mailboxes are receive-only mailboxes.

Figure 25-32: CAN\_MBTIF1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000031_ff54cdcce7ddb26f9fde38c171a2a29bd28413640d18f7f4c81931bf8d7163d5.png)

Table 25-27: CAN\_MBTIF1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                       |
|--------------------|------------|-----------------------------------------------|
| 15:8               | MB         | Mailbox n Transmit Interrupt Request Pending. |

## Mailbox Transmit Interrupt Flag 2 Register

The CAN\_MBTIF2 register indicates when a transmit interrupt request is pending---due to successful transmission (corresponding CAN\_TA2 bit is set) and the interrupt is enabled (corresponding CAN\_MBIM2 bit is set)---for mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register indicates the transmit interrupt pending status for the corresponding mailbox when set (=1). When any bit in CAN\_MBTIF2 is set, the CAN transmit interrupt request is raised ( CAN\_INT.MBTIRQ bit is set). T o clear the interrupt request, all of the set bits in CAN\_MBTIF2 must be cleared by software (W1C). Also, software must clear the associated bits set in CAN\_TA2 or set the associated bits in CAN\_TRS2 bit to clear the interrupt source asserting the bits in CAN\_MBTIF2 .

Figure 25-33: CAN\_MBTIF2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000032_839b42207cb508a3ca66e45e25ad6a41a798225f2536885fca2f9566dd8885cb.png)

Table 25-28: CAN\_MBTIF2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration               |
|--------------------|------------|---------------------------------------|
| 15:0               | MB         | Mailbox n Transmit Interrupt Pending. |

## Mailbox Word 0 Register

The CAN\_MB[nn]\_DATA0 register holds mailbox data bytes.

Figure 25-34: CAN\_MB[nn]\_DATA0 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000033_1698614c07f3015725a47ddfd59d60506d7461a2a381fafe38952ed3816eeca9.png)

Table 25-29: CAN\_MB[nn]\_DATA0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                           |
|--------------------|------------|---------------------------------------------------|
| 15:8               | DFB6       | Data Field Byte 6.                                |
| (R/W)              |            | The CAN_MB[nn]_DATA0.DFB6 bits hold mailbox data. |
| 7:0                | DFB7       | Data Field Byte 7.                                |
| (R/W)              |            | The CAN_MB[nn]_DATA0.DFB7 bits hold mailbox data. |

## Mailbox Word 1 Register

The CAN\_MB[nn]\_DATA1 register holds mailbox data bytes.

Figure 25-35: CAN\_MB[nn]\_DATA1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000034_f60dc64907a36cf86890467e50bbf57a6a5d55725428726b0df26ba3297080d4.png)

Table 25-30: CAN\_MB[nn]\_DATA1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                           |
|--------------------|------------|---------------------------------------------------|
| 15:8               | DFB4       | Data Field Byte 4.                                |
| (R/W)              |            | The CAN_MB[nn]_DATA1.DFB4 bits hold mailbox data. |
| 7:0                | DFB5       | Data Field Byte 5.                                |
| (R/W)              |            | The CAN_MB[nn]_DATA1.DFB5 bits hold mailbox data. |

## Mailbox Word 2 Register

The CAN\_MB[nn]\_DATA2 register holds mailbox data bytes.

Figure 25-36: CAN\_MB[nn]\_DATA2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000035_2e4d8e4a867c267e842114eb8d44d914eb204fb312024c2fa58a4eb6f08c1b02.png)

Table 25-31: CAN\_MB[nn]\_DATA2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                           |
|--------------------|------------|---------------------------------------------------|
| 15:8               | DFB2       | Data Field Byte 2.                                |
| (R/W)              |            | The CAN_MB[nn]_DATA2.DFB2 bits hold mailbox data. |
| 7:0                | DFB3       | Data Field Byte 3.                                |
| (R/W)              |            | The CAN_MB[nn]_DATA2.DFB3 bits hold mailbox data. |

## Mailbox Word 3 Register

The CAN\_MB[nn]\_DATA3 register holds mailbox data bytes.

Figure 25-37: CAN\_MB[nn]\_DATA3 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000036_d26302bc05329f577ac9703828e1a03da2422b8d94f1fceac0ed219ce4db06b3.png)

Table 25-32: CAN\_MB[nn]\_DATA3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                           |
|--------------------|------------|---------------------------------------------------|
| 15:8               | DFB0       | Data Field Byte 0.                                |
| (R/W)              |            | The CAN_MB[nn]_DATA3.DFB0 bits hold mailbox data. |
| 7:0                | DFB1       | Data Field Byte 1.                                |
| (R/W)              |            | The CAN_MB[nn]_DATA3.DFB1 bits hold mailbox data. |

## Mailbox ID 0 Register

The CAN\_MB[nn]\_ID0 register contains the lower 16 bits of the 18-bit extended identifier.

Figure 25-38: CAN\_MB[nn]\_ID0 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000037_79f65749e16087bd967bdf0da4ff5d970b60bb9efe844a779f08d3b1d5ead7c6.png)

Table 25-33: CAN\_MB[nn]\_ID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | EXTID      | Extended Identifier/Data Field Acceptance Code. The CAN_MB[nn]_ID0.EXTID bits hold the lower 16 bits of the 18-bit extended ID. |

## Mailbox ID 1 Register

The CAN\_MB[nn]\_ID1 register contains the identifier bits of mailbox. The 11-bit BASE\_ID is mapped to The CAN\_MB[nn]\_ID1.BASEID field. It also enables the extended identification and contains upper two bits of 18bit extended identifier.

Figure 25-39: CAN\_MB[nn]\_ID1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000038_133dd0bfb6e165c10c0475ae5d37e46e1138d52951231330fa78cba30fa74429.png)

Table 25-34: CAN\_MB[nn]\_ID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | AME        | Acceptance Mask Enable. The CAN_MB[nn]_ID1.AME bit enables acceptance mask operations if the mailbox is configured as receiver. When enabled (=1), only those bits that have the correspond- ing mask bit cleared are compared to the received message ID. A bit position that is set in the mask register does not need to match. This bit should be set to 0 when the mail- box is configured in transmit mode. |
| 14 (R/W)           | RTR        | Remote Transmission Request. The CAN_MB[nn]_ID1.RTR bit selects whether the frame contains data (data frame) or contains a request for data associated with the message identifier in the frame being sent (remote frame).                                                                                                                                                                                        |
| 13 (R/W)           | IDE        | Identifier Extension. The CAN_MB[nn]_ID1.IDE bit enables the comparison of the received message ID to the value in the CAN_MB[nn]_ID1.EXTID and CAN_MB[nn]_ID0.EXTID bits. When configured as transmitter, it sends the ex- tended identifier in addition to the base identifier.                                                                                                                                 |
| 12:2 (R/W)         | BASEID     | Base Identifier. The CAN_MB[nn]_ID1.BASEID bits hold the base identifier for acceptance mask operations.                                                                                                                                                                                                                                                                                                          |
| 1:0 (R/W)          | EXTID      | Extended Identifier. The CAN_MB[nn]_ID1.EXTID bits hold the upper two bits of 18-bit extended identifier.                                                                                                                                                                                                                                                                                                         |

## Mailbox Length Register

The CAN\_MB[nn]\_LENGTH register holds the data length code for the received remote frame. For more information about remote frames, see the Remote Frame Handling section.

Figure 25-40: CAN\_MB[nn]\_LENGTH Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000039_e1a9d081235e29cfec01cb55c636e36599afd3d6026375f53a4694e361135157.png)

Table 25-35: CAN\_MB[nn]\_LENGTH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | DLC        | Data Length Code. The CAN_MB[nn]_LENGTH.DLC bits hold the DLC value of the received remote frame. The received value overwrites any previous value. |

## Mailbox Time Stamp Register

The CAN\_MB[nn]\_TIMESTAMP register holds an indication of the time of reception or transmission for each message, when the universal counter is in time stamp mode ( CAN\_UCCNF.UCCNF =0x1). In this mode, the CAN writes the value of the counter ( CAN\_UCCNT ) to the CAN\_MB[nn]\_TIMESTAMP register when a received message is stored or a message is transmitted. For more information about time stamps, see the Time Stamps section.

Figure 25-41: CAN\_MB[nn]\_TIMESTAMP Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000040_334329ae777bcc5ca4ba7b34fa95d79048aac32f0446aca4207899d35b31bba0.png)

Table 25-36: CAN\_MB[nn]\_TIMESTAMP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                              |
|--------------------|------------|----------------------------------------------------------------------|
| 15:0               | TSV        | Time Stamp Value.                                                    |
| (R/W)              |            | The CAN_MB[nn]_TIMESTAMP.TSV bits hold the message time stamp value. |

## Mailbox Configuration 1 Register

The CAN\_MC1 register enables mailboxes 0 through 15. Each bit in this register enables or disables the corresponding mailbox. For all bits, set the bit (=1) to enable the mailbox, and clear the bit (=0) to disable the mailbox.

Enabling and disabling mailboxes has an impact on transmit requests. Setting the CAN\_TRS1 bit associated with a disabled mailbox may result in erroneous behavior. Similarly, disabling a mailbox before the associated CAN\_TRS1 bit is reset by the internal logic can cause unpredictable results.

Figure 25-42: CAN\_MC1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000041_ef5d503f12f7504ed52241977edb01c420b102b1f17307f9714795b6a67662eb.png)

Table 25-37: CAN\_MC1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | MB         | Mailbox n Enable/Disable. |

## Mailbox Configuration 2 Register

The CAN\_MC2 register enables mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register enables or disables the corresponding mailbox. For all bits, set the bit (=1) to enable the mailbox, and clear the bit (=0) to disable the mailbox.

Enabling and disabling mailboxes has an impact on transmit requests. Setting the CAN\_TRS2 bit associated with a disabled mailbox may result in erroneous behavior. Similarly, disabling a mailbox before the associated CAN\_TRS2 bit is reset by the internal logic can cause unpredictable results.

Figure 25-43: CAN\_MC2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000042_73a0af930f6f2125d1d54214648f56f706a1342b5beea6a89565ef6dcee9012d.png)

Table 25-38: CAN\_MC2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | MB         | Mailbox n Enable/Disable. |

## Mailbox Direction 1 Register

The CAN\_MD1 register selects the data transfer direction for mailboxes 0 through 15. Each bit in this register selects receive mode or transmit mode for the corresponding mailbox. For all bits, set the bit (=1) for receive mode from the mailbox, and clear the bit (=0) for transmit mode to the mailbox. Bits 0 through 7 are read-only, as the corresponding mailboxes are receive-only mailboxes.

Figure 25-44: CAN\_MD1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000043_d7f376d5efb3dcff5f401a1573db7be2c4d8ee3c8f5a7798000dc20d8908669d.png)

Table 25-39: CAN\_MD1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 15:8               | MB         | Mailbox n Transmit/Receive. |

## Mailbox Direction 2 Register

The CAN\_MD2 register selects the data transfer direction for mailboxes 16 (bit 0) through 23 (bit 7). Each bit in this register selects receive mode or transmit mode for the corresponding mailbox. For all bits, set the bit (=1) for receive mode from the mailbox, and clear the bit (=0) for transmit mode to the mailbox. Bits 8 through 15 are read-only, as the corresponding mailboxes (24 through 31) are transmit-only mailboxes.

Figure 25-45: CAN\_MD2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000044_abd99917ecd5da191f86ddfa10f413bafa40dc7af1d98b97de68ab0ef914d443.png)

Table 25-40: CAN\_MD2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 7:0                | MB         | Mailbox n Transmit/Receive. |

## Overwrite Protection/Single Shot Transmission 1 Register

The CAN\_OPSS1 register enables overwrite protection for mailboxes 0 through 15. Each bit in this register enables overwrite protection for the corresponding mailbox when set (=1). Note that enabling this bit affects transmit and receive operations for mailboxes. For more information about remote overwrite protection, see the detailed feature description in the CAN Functional Description section. For more information about how this feature affects transmit and receive operations, see the CAN Operating Modes sections, describing transmit and receive operations.

Figure 25-46: CAN\_OPSS1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000045_cafa1841275089792330a8406fbbb50b5b1d43619459cc065e0953d39dd366fe.png)

Table 25-41: CAN\_OPSS1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                |
|--------------------|------------|----------------------------------------|
| 15:0               | MB         | Mailbox n Overwrite Protection Enable. |

## Overwrite Protection/Single Shot Transmission 2 Register

The CAN\_OPSS2 register enables overwrite protection for mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register enables overwrite protection for the corresponding mailbox when set (=1). Note that enabling this bit affects transmit and receive operations for mailboxes. For more information about remote overwrite protection, see the detailed feature description in the CAN Functional Description section. For more information about how this feature affects transmit and receive operations, see the CAN Operating Modes sections, describing transmit and receive operations.

Figure 25-47: CAN\_OPSS2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000046_addfed07822f8f3aea266350d90dc62a164ed49f56ef040db1ce943b061742e0.png)

Table 25-42: CAN\_OPSS2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                |
|--------------------|------------|----------------------------------------|
| 15:0               | MB         | Mailbox n Overwrite Protection Enable. |

## Remote Frame Handling 1 Register

The CAN\_RFH1 register enables remote frame handling for mailboxes 8 through 15. Each bit in this register enables remote frame handling for the corresponding mailbox when set (=1). Note that enabling this bit affects transmit and receive operations for mailboxes. For more information about remote frame handling, see the CAN Operating Modes sections, describing transmit and receive operations. Bits 0 through 7 are read-only, as the corresponding mailboxes are receive-only mailboxes.

Figure 25-48: CAN\_RFH1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000047_663bf2199b331b0d68b2b30108beec2b8801eef420545e942048407612fb7634.png)

Table 25-43: CAN\_RFH1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                 |
|--------------------|------------|-----------------------------------------|
| 15:8               | MB         | Mailbox n Remote Frame Handling Enable. |

## Remote Frame Handling 2 Register

The CAN\_RFH2 register enables remote frame handling for mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register enables remote frame handling for the corresponding mailbox when set (=1). Note that enabling this bit affects transmit and receive operations for mailboxes. For more information about remote frame handling, see the CAN Operating Modes sections, describing transmit and receive operations.

Figure 25-49: CAN\_RFH2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000048_b7a79e748b2fbf3efb563f0f001afee3304eb48d9c34d21cb97f869df5be4aa6.png)

Table 25-44: CAN\_RFH2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                 |
|--------------------|------------|-----------------------------------------|
| 7:0                | MB         | Mailbox n Remote Frame Handling Enable. |
| (R/W)              |            |                                         |

## Receive Message Lost 1 Register

The CAN\_RML1 register indicates when a message is lost---due to a message coming while there is pending data (corresponding CAN\_RMP1 bit set) and overwrite protection is disabled ( CAN\_OPSS1 bit cleared)---for mailboxes 0 through 15. Each bit in this register indicates the message lost status for the corresponding mailbox when set (=1).

Figure 25-50: CAN\_RML1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000049_d28fa1ff52f85b96358e63c7daa7f2a2820813554ef464b3a091b681b2e5b736.png)

Table 25-45: CAN\_RML1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | MB         | Mailbox n Message Lost.   |
| (R/NW)             |            |                           |

## Receive Message Lost 2 Register

The CAN\_RML2 register indicates when a message is lost---due to a message coming while there is pending data (corresponding CAN\_RMP2 bit set) and overwrite protection is disabled ( CAN\_OPSS2 bit cleared)---for mailboxes 16 (bit 0) through 23 (bit 7). Each bit in this register indicates the message lost status for the corresponding mailbox when set (=1). Bits 8 through 15 are reserved, as the corresponding mailboxes (24 through 31) are transmit-only mailboxes.

Figure 25-51: CAN\_RML2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000050_2d0c0818b1c67d514987afad0b2b4841b7b76be9047cb119109f7a7917df254a.png)

Table 25-46: CAN\_RML2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | MB         | Mailbox n Message Lost.   |
| (R/NW)             |            |                           |

## Receive Message Pending 1 Register

The CAN\_RMP1 register indicates when a message is pending (unread data) for mailboxes 0 through 15. Each bit in this register indicates the message pending status for the corresponding mailbox when set (=1).

Figure 25-52: CAN\_RMP1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000051_60c4e5c971d1377a154467241f5c5ff8bfa082caee26e910bf8eb1416be49656.png)

Table 25-47: CAN\_RMP1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 15:0               | MB         | Mailbox n Message Pending. |

## Receive Message Pending 2 Register

The CAN\_RMP2 register indicates when a message is pending (unread data) for mailboxes 16 (bit 0) through 23 (bit 7). Each bit in this register indicates the message pending status for the corresponding mailbox when set (=1). Bits 8 through 15 are reserved, as the corresponding mailboxes (24 through 31) are transmit-only mailboxes.

Figure 25-53: CAN\_RMP2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000052_42bfbdb56169ed659966a0ad2bc3212ed8b4499c8873432ca3dd41ac2b5e167e.png)

Table 25-48: CAN\_RMP2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 7:0                | MB         | Mailbox n Message Pending. |

## Status Register

The CAN\_STAT register indicates status for CAN modes and error conditions.

Figure 25-54: CAN\_STAT Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000053_cea047bb00a920754793135af8afdbfb9dac325cf16e565a3f03ab1e999dd87e.png)

Table 25-49: CAN\_STAT Register Fields

| Bit No. (Access)   | Description/Enumeration                                                                                                                                              |
|--------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | Receive Mode. The CAN_STAT.REC bit indicates whether the CAN is in receive mode. 0 Not in Receive Mode                                                               |
| 14 (R/NW)          | Transmit Mode. The CAN_STAT.TRM bit indicates whether the CAN is in transmit mode.                                                                                   |
| 12:8 (R/NW)        | Mailbox Pointer. The CAN_STAT.MBPTR bits represent the mailbox number of the current transmit message. After a successful transmission, these bits remain unchanged. |
| 7 (R/NW)           | CAN Configuration Mode Acknowledge. The CAN_STAT.CCA bit indicates whether the CAN is in configuration mode. 0 Not in Configuration Mode 1 Configuration mode        |

Table 25-49: CAN\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/NW)           | CSA        | CAN Suspend Mode Acknowledge. The CAN_STAT.CSA bit indicates whether the CAN is in suspend mode. 0 Not in Suspend Mode                      |
| 3 (R/NW)           | EBO        | CAN Error Bus Off Mode. The CAN_STAT.EBO bit indicates whether the CAN is in error bus off mode. 0 TXECNT Below 256                         |
| 2 (R/NW)           | EP         | CAN Error Passive Mode. The CAN_STAT.EP bit indicates whether the CAN is in error passive mode. 0 TXECNT and RXECNT Below 128               |
| 1 (R/NW)           | WR         | CAN Receive Warning Flag. The CAN_STAT.WR bit indicates whether the CAN has detected a receive warning flag condition. 0 RXECNT Below Limit |
| 0 (R/NW)           | WT         | 1 RXECNT at Limit CAN Transmit Warning Flag. The CAN_STAT.WT bit indicates whether the CAN detected a transmit warning flag condition.      |
|                    |            | 0 TXECNT Below Limit                                                                                                                        |

## Transmission Acknowledge 1 Register

The CAN\_TA1 register indicates transmission success for mailboxes 8 through 15. Each bit in this register indicates transmission success for the corresponding mailbox when set (=1). Bits 0 through 7 are read-only, as the corresponding mailboxes are receive-only mailboxes.

Figure 25-55: CAN\_TA1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000054_b325bc8c896b3319ee65b7062e1c54abe3fb9c7d8666fdbdfa1811fe213e73b0.png)

Table 25-50: CAN\_TA1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 15:8               | MB         | Mailbox n Transmit Acknowledge. |

## Transmission Acknowledge 2 Register

The CAN\_TA2 register indicates transmission success for mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register indicates transmission success for the corresponding mailbox when set (=1).

Figure 25-56: CAN\_TA2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000055_19e38ac2faa41206b1e175939a76a780063b498dabb10ef5cd96bf0a83194301.png)

Table 25-51: CAN\_TA2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 15:0               | MB         | Mailbox n Transmit Acknowledge. |

## Timing Register

The CAN\_TIMING register select the time segments, sampling, and synchronization for CAN bit timing. For more information about bit timing and clock operation, see the CAN Operating Modes section.

Figure 25-57: CAN\_TIMING Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000056_07aa99c0decfc7183df1b2d8fb7742c75306b8a9cee46fd8ff5b32f5fb1489f9.png)

Table 25-52: CAN\_TIMING Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:8 (R/W)          | SJW        | Synchronization Jump Width. The CAN_TIMING.SJW bits select the maximum number of time quanta, ranging from 1 to 4(SJW + 1). This selection allows for a re-synchronization attempt when the CAN detects a recessive-to-dominant edge outside the synchronization segment. The re-synchronization automatically moves the sampling point such that the CAN bit is still handled properly. Note that the CAN_TIMING.SJW value should not exceed CAN_TIMING.TSEG2 or CAN_TIMING.TSEG1 . |
| 7 (R/W)            | SAM        | Sampling. The CAN_TIMING.SAM bit selects whether the CAN performs normal sampling (once at the sampling point described by the CAN_TIMING register) or performs over sampling. If CAN_TIMING.SAM is set, the CAN over samples the input signal at three times at the CDU0_CLKO4 rate. The resulting value is generated by a majority decision of the three sample values. Note that the CAN_TIMING.SAM bit should al- ways be cleared if the CAN_CLK.BRP value is less than 4.       |
| 6:4 (R/W)          | TSEG2      | Time Segment 2. The CAN_TIMING.TSEG2 bits and CAN_TIMING.TSEG1 bits control how many time quanta of which the CAN bits consist, resulting in the CAN bit rate. For more information about bit timing and clock operation, see the CAN Operating Modes sec- tion. Note that the CAN_TIMING.TSEG1 value should always be greater than or equal to the CAN_TIMING.TSEG2 value.                                                                                                          |
| 3:0 (R/W)          | TSEG1      | Time Segment 1. The CAN_TIMING.TSEG1 bits and CAN_TIMING.TSEG2 bits control how many time quanta of which the CAN bits consist, resulting in the CAN bit rate. For more information about bit timing and clock operation, see the CAN Operating Modes sec- tion. Note that the CAN_TIMING.TSEG1 value should always be greater than or equal to the CAN_TIMING.TSEG2 value.                                                                                                          |

## Transmission Request Reset 1 Register

The CAN\_TRR1 register requests transmit abort for mailboxes 8 through 15. Bits in this register request transmit abort for the corresponding mailbox when set (=1). When a transmission completes, the corresponding bits in the transmit request set register ( CAN\_TRS1 ) and in the CAN\_TRR1 are cleared. Bits 0 through 7 are read-only, as the corresponding mailboxes are receive-only mailboxes.

Figure 25-58: CAN\_TRR1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000057_8d421515b63bf234aca6503d4374670bf7a35cfe797c9302b78ba6c86d9d278a.png)

Table 25-53: CAN\_TRR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:8               | MB         | Mailbox n Transmit Abort. |

## Transmission Request Reset 2 Register

The CAN\_TRR2 register requests transmit abort for mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register requests transmit abort for the corresponding mailbox when set (=1). When a transmission completes, the corresponding bits in the transmit request set register ( CAN\_TRS2 ) and in the CAN\_TRR2 are cleared.

Figure 25-59: CAN\_TRR2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000058_a951d45e6cfe59e3940691319cb9f30f7804d2b298f77cb11fff83a05e5d0e05.png)

Table 25-54: CAN\_TRR2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | MB         | Mailbox n Transmit Abort. |

## Transmission Request Set 1 Register

The CAN\_TRS1 register requests transmit for mailboxes 8 through 15. Bits in this register request transmit for the corresponding mailbox when set (=1). After writing the data and the identifier into the mailbox area, the message is sent after mailbox n is enabled (with the corresponding bit in CAN\_MC1 = 1), and (subsequently) the corresponding transmit request bit is set (in CAN\_TRS1 ). When a transmission completes, the corresponding bits in CAN\_TRS1 and in the transmit request reset register ( CAN\_TRR1 ) are cleared. Bits 0 through 7 are read-only, as the corresponding mailboxes are receive-only mailboxes.

Figure 25-60: CAN\_TRS1 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000059_ed33543658bf9d6108e266f6a7b760891c19d253e1206f1cf6a1d5be8456899b.png)

Table 25-55: CAN\_TRS1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 15:8               | MB         | Mailbox n Transmit Request. |

## Transmission Request Set 2 Register

The CAN\_TRS2 register requests transmit for mailboxes 16 (bit 0) through 31 (bit 15). Each bit in this register requests transmit for the corresponding mailbox when set (=1). After writing the data and the identifier into the mailbox area, the message is sent after mailbox n is enabled (with the corresponding bit in CAN\_MC2 = 1), and (subsequently) the corresponding transmit request bit is set (in CAN\_TRS2 ). When a transmission completes, the corresponding bits in CAN\_TRS2 and in the transmit request reset register ( CAN\_TRR2 ) are cleared.

Figure 25-61: CAN\_TRS2 Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000060_5d4f3cc33ebe0f917b05e8503e2013a5d381bb4f20f718ce469a8a1042b83ab0.png)

Table 25-56: CAN\_TRS2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 15:0               | MB         | Mailbox n Transmit Request. |

## Universal Counter Configuration Mode Register

The CAN\_UCCNF register controls the operation of the universal counter, including counter enable and counter mode selection.

Figure 25-62: CAN\_UCCNF Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000061_3cbf22e4c94d5008bf35a2f3a55c150e55a2363abe5932977448065275455aa5.png)

Table 25-57: CAN\_UCCNF Register Fields

| Bit No. (Access)   | Bit Name                                          | Description/Enumeration                                                                                                                                                                                                                                                                                |
|--------------------|---------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | UCE                                               | Universal Counter Enable. The CAN_UCCNF.UCE bit enables universal counter operation in the mode selected by the CAN_UCCNF.UCCNF bits.                                                                                                                                                                  |
| 6 (R/W)            | UCCT                                              | Universal Counter CAN Trigger. The CAN_UCCNF.UCCT bit enables the universal counter trigger, directing the CAN to re-load the counter on mailbox 4 reception in watchdog mode and clear the counter on mailbox 4 reception in time stamp mode. This bit has no effect in all other modes.              |
| 5 (R/W)            | UCRC                                              | 1 Enable Trigger Universal Counter Reload/Clear. The CAN_UCCNF.UCRC bit re-loads or clears the universal counter, depending on the counter mode. In watchdog mode, setting this bit directs the CAN to re-load the counter. In all other modes, setting this bit directs the CAN to clear the counter. |
| 3:0 (R/W)          | Universal Counter Configuration.                  | 0 No Action 1 Re-load or Clear the Counter                                                                                                                                                                                                                                                             |
| 3:0 (R/W)          | UCCNF The CAN_UCCNF.UCCNF information about these | bits select the universal counter operating mode. For more modes, see the Operating Modes section. 0 Reserved                                                                                                                                                                                          |
|                    |                                                   | 1 Time Stamp Mode                                                                                                                                                                                                                                                                                      |
|                    |                                                   | 2 Watchdog Mode                                                                                                                                                                                                                                                                                        |

Table 25-57: CAN\_UCCNF Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration |                                 |
|-----------|------------|---------------------------|---------------------------------|
| (Access)  |            |                           |                                 |
|           |            |                         3 | Auto-transmit Mode              |
|           |            |                         4 | Reserved                        |
|           |            |                         5 | Reserved                        |
|           |            |                         6 | Count Error Frames              |
|           |            |                         7 | Count Overload Frames           |
|           |            |                         8 | Count Arbitration Lost          |
|           |            |                         9 | Count Aborted Transmissions     |
|           |            |                        10 | Count Successful Transmissions  |
|           |            |                        11 | Count Rejected Receive Messages |
|           |            |                        12 | Count Receive Message Lost      |
|           |            |                        13 | Count Successful Receptions     |
|           |            |                        14 | Count Stored Receptions         |
|           |            |                        15 | Count Valid Messages            |

## Universal Counter Register

The CAN\_UCCNT register holds the current universal count. This register is reloaded from the CAN\_UCRC register when counter the decrements to zero in auto-transmit mode.

Figure 25-63: CAN\_UCCNT Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000062_b0c5e54e6dbded8f15dec97c3926bd88a2752b17926289f771bc4398a6954ef6.png)

Table 25-58: CAN\_UCCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 15:0               | COUNT      | Count Value.                                                     |
| (R/W)              |            | The CAN_UCCNT.COUNT bits hold the current universal count value. |

## Universal Counter Reload/Capture Register

The CAN\_UCRC register holds the period value (universal count), which is used in auto-transmit mode as the period for sending the message in mailbox 11 (broadcast heartbeat) to all CAN nodes. Accordingly, messages sent this way usually have high priority.

The period value is written to the CAN\_UCRC register. When auto-transmit mode is enabled ( CAN\_UCCNF.UCCNF = 0x3), the CAN loads the counter with the value in CAN\_UCRC . The counter decrements to 0 at the CAN bit clock rate, then is reloaded. Each time the counter decrements to 0, the CAN sets the CAN\_TRS1.MB bit for mailbox 11 and sends the corresponding message from mailbox 11.

Note that for auto-transmit mode, mailbox 11 must be configured as a transmit mailbox and must contain valid data (identifier, control bits, and data). This setup must occur before the counter first expires after this mode is enabled.

Figure 25-64: CAN\_UCRC Register Diagram

![Image](28_Controller_Area_Network_(CAN)_artifacts/image_000063_2369d6b23ccb0e08ba796f5e0444d0e0af4d8396c6101fc5c972929cd6ffc080.png)

Table 25-59: CAN\_UCRC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | UCVAL      | Universal Counter Value. The CAN_UCRC.UCVAL bits hold the value for the universal count period, which is used in auto-transmit mode. |