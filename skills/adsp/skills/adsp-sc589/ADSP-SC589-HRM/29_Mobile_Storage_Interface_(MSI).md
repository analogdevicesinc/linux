## 26   Mobile Storage Interface (MSI)

The mobile storage interface (MSI) is a fast, synchronous controller that uses various protocols to communicate with MMC, SD, and SDIO cards. It addresses the growing storage need in embedded systems, handheld, and consumer electronics applications that require low power. The MSI is compatible with the following protocols.

- MMC (Multimedia Card) bus protocol
- SD (Secure Digital) bus protocol
- SDIO (Secure Digital Input Output) bus protocol

All of these storage solutions use similar interface protocols. The main difference between MMC and SD support is the initialization sequence. The main difference between SD and SDIO support is the use of interrupt and read wait signals for SDIO.

NOTE: The MSI does not support the SPI bus protocol.

## MSI Features

The MSI includes the following features.

- Supports Secure Digital memory protocol (version 3.0)
- Supports Secure Digital I/O protocol (version 3.0)
- Supports Multimedia Card protocol (MMC version 4.41, eMMC version 4.5)
- Support for a single SD or SDIO card
- Support for a single MMC device (removable or embedded)
- Support for 1-bit and 4-bit SD modes (SPI mode is not supported)
- Support for 1-, 4-, and 8-bit MMC modes (SPI mode is not supported)
- Supports MMC boot operation
- Supports SDIO interrupts
- Supports command completion signal and interrupt to host processor

- Supports CRC generation and error detection
- 1024-byte transmit/receive FIFO
- Integrated DMA controller
- Card detection capabilities
- Programmable clock frequency
- Supports power management and clock control.

## MSI Functional Description

This section provides information on the function of the MSI module.

## MMC booting

Normal boot operation is applicable to MMC4.3, MMC4.4, and MMC4.41 cards and is performed in pushpull mode. Also alternate Boot Operation; removable MMC4.3 card.

## CRC generation and error detection

Cyclic redundancy codes (CRC) are used to protect commands, responses, and data transfers from transmission errors.

## FIFO controller

Interfaces the internal FIFO to the host or DMA interface and the card controller unit. The FIFO depth is configured for 1024 bytes

## Integrated DMA controller

Contains a single transmit or receive engine, which transfers data from host memory to the device port and conversely. The controller uses a descriptor to move data efficiently from source to destination with minimal core intervention

## Power management and clock control

A low-power mode is available. A clock control block provides the clock frequencies required for SD/MMC cards

## ADSP-SC58x MSI Register List

The Mobile Storage Interface (MSI) supports access to mobile memory and devices. A set of registers governs MSI operations. For more information on MSI functionality, see the MSI register descriptions.

Table 26-1: ADSP-SC58x MSI Register List

| Name         | Description                                      |
|--------------|--------------------------------------------------|
| MSI_BLKSIZ   | Block Size Register                              |
| MSI_BUFADDR  | Current Buffer Descriptor Address Register       |
| MSI_BUSMODE  | Bus Mode Register                                |
| MSI_BYTCNT   | Byte Count Register                              |
| MSI_CDETECT  | Card Detect Register                             |
| MSI_CDTHRCTL | Card Threshold Control Register                  |
| MSI_CLKDIV   | Clock Divider Register                           |
| MSI_CLKEN    | Clock Enable Register                            |
| MSI_CMD      | Command Register                                 |
| MSI_CMDARG   | Command Argument Register                        |
| MSI_CTL      | Control Register                                 |
| MSI_CTYPE    | Card Type Register                               |
| MSI_DBADDR   | Descriptor List Base Address Register            |
| MSI_DEBNCE   | Debounce Count Register                          |
| MSI_DSCADDR  | Current Host Descriptor Address Register         |
| MSI_ENSHIFT  | Enable Phase Shift Register                      |
| MSI_FIFOTH   | FIFO Threshold Watermark Register                |
| MSI_IDINTEN  | Internal DMAInterrupt Enable Register            |
| MSI_IDSTS    | Internal DMAStatus Register                      |
| MSI_IMSK     | Interrupt Mask Register                          |
| MSI_ISTAT    | Raw Interrupt Status Register                    |
| MSI_MSKISTAT | Masked Interrupt Status Register                 |
| MSI_PLDMND   | Poll Demand Register                             |
| MSI_RESP0    | Response Register 0                              |
| MSI_RESP1    | Response Register 1                              |
| MSI_RESP2    | Response Register 2                              |
| MSI_RESP3    | Response Register 3                              |
| MSI_STAT     | Status Register                                  |
| MSI_TBBCNT   | Transferred Host to BIU-FIFO Byte Count Register |
| MSI_TCBCNT   | Transferred CIU Card Byte Count Register         |
| MSI_TMOUT    | Timeout Register                                 |

## ADSP-SC58x MSI Interrupt List

Table 26-2: ADSP-SC58x MSI Interrupt List

|   Interrupt ID | Name      | Description   | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------|---------------|---------------|
|            131 | MSI0_STAT | MSI0 Status   | Level         |               |

## ADSP-SC58x MSI Trigger List

Table 26-3: ADSP-SC58x MSI Trigger List Masters

|   Trigger ID | Name      | Description        | Sensitivity   |
|--------------|-----------|--------------------|---------------|
|          136 | MSI0_DONE | MSI0 Transfer Done | Level         |

Table 26-4: ADSP-SC58x MSI Trigger List Slaves

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## MSI Block Diagram

The MSI Block Diagram shows the functional blocks within the MSI. These blocks are described in more detail in the MSI Architectural Concepts section.

Figure 26-1: MSI Block Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000000_2ba27d2697bb4424cbbda7c35b0f8dc558b5e5425f3b76eb15287f7019f5c5e0.png)

NOTE: The card-detect and write-protect signals are from the SD/MMC card socket and not from the SD/MMC card.

## MSI Architectural Concepts

The following sections describe the functions and features of the MSI controller as well as the MMC, SD, and SDIO protocols.

Communication is through a master and slave configuration, where the MSI is the master device and the card is the slave device. The MSI communicates with the device using a message-based bus protocol in which the host sends commands serially using the MSI\_CMD signal. Some commands require that the card provide a response back to the host. This response is also sent serially on the MSI\_CMD signal.

Data transfers, both to and from the card, occur using the data signals. The number of data lines used for the data transfer can be configured to 1 ( MSI\_D0 ), 4 ( MSI\_D3 -MSI\_D0 ), or 8 ( MSI\_D7 -MSI\_D0 ). All MSI\_CMD and MSI\_D7 -MSI\_D0 transfers are synchronous with MSI\_CLK . Cyclic redundancy codes (CRC) are used to protect commands, responses, and data transfers from transmission errors. A CRC7 code is generated for every command sent by the host and for almost every response returned by the card on the MSI\_CMD signal.

The MSI architecture can be described in terms of its submodules. The primary parts of the architecture are:

- Bus Interface Unit (BIU). This unit provides the host interface to the registers through the Host Interface Unit (HIU). Additionally, it provides independent data FIFO access through a DMA interface.

- Internal Direct Memory Access Controller (IDMAC). This unit is responsible for exchanging data between the FIFO and the host memory. A set of IDMAC registers is accessible to the host for controlling the IDMAC operation.
- Card Interface Unit (CIU). This unit controls the card-specific protocols. Within the CIU, the command path control unit and datapath control unit interface the controller to the command and data ports of the SD/MMC/SDIO cards. The CIU also provides clock control.

## Bus Interface Unit (BIU)

The BIU provides the following functions:

- Host interface
- Interrupt control
- Register access
- FIFO access
- Power and pull-up control and card detection

## Host Interface Unit (HIU)

The Host Interface Unit (HIU) is a slave bus interface, which provides the interface between the MSI and the processor system bus. It supports the burst accesses to the data FIFO address region only. The register address region is accessed through the standard accesses.

## Interrupt Controller Unit

The interrupt controller unit generates an interrupt that depends on the controller interrupt status, the interruptmask register, and the global interrupt-enable register bit. Once an interrupt condition is detected, the software sets the corresponding interrupt bit in the interrupt status register. The interrupt status bit remains set until the software clears it by writing a 1 to the interrupt bit (a 0 leaves the bit unchanged).

NOTE: Before enabling the interrupt, programs must write 32'hFFFF\_FFFF to the interrupt status register ( MSI\_ISTAT ) in order to clear any pending unserviced interrupts. When clearing interrupts during normal operation, only clear the interrupt bits that are serviced.

## Register Unit

The register unit is part of the Bus Interface Unit (BIU). It provides read and write access to the registers.

All registers reside in the BIU clock domain. When a command is sent to a card by setting the MSI\_CMD.STARTCMD bit, all relevant registers needed for the CIU operation are transferred to the CIU block. (The MSI\_CMD.STARTCMD bit is bit[31] of the MSI\_CMD register). During this time, software must not write to the registers that are transferred from the BIU to the CIU. The software must wait for the hardware to clear the start bit before writing to these registers again. The register unit has a hardware locking feature to prevent illegal writes to registers.

Once a command start is issued by setting the MSI\_CMD.STARTCMD bit, the following registers cannot be reprogrammed until the Card Interface Unit (CIU) accepts the command:

- MSI\_CMD - Command
- MSI\_CMDARG - Command Argument
- MSI\_BYTCNT - Byte Count
- MSI\_BLKSIZ - Block Size
- MSI\_CLKDIV - Clock Divider
- MSI\_CLKEN - Clock Enable
- MSI\_TMOUT - Timeout
- MSI\_CTYPE - Card Type

The hardware resets the MSI\_CMD.STARTCMD bit once the CIU accepts the command. If a host writes to any of these registers during this locked time, then the write is ignored and the hardware lock error bit is set in the MSI\_ISTAT (status) register. Additionally, if the interrupt is enabled and not masked for a hardware lock error, then an interrupt is sent to the host.

Once a command is accepted, software can send another command to the CIU which has a one-deep command queue under the following conditions:

- If the previous command was not a data transfer command, the new command is sent to the card once the previous command completes.
- If the previous command is a data transfer command and if the MSI\_CMD.WTPRIVDATA bit is set for the new command, the new command is sent to the card only when the data transfer completes.
- If the MSI\_CMD.WTPRIVDATA is 0, then the new command is sent to the card as soon as the previous command is sent. Typically, software uses this option only to stop or abort a previous data transfer or query the card status in the middle of a data transfer.

## FIFO Controller Unit

The FIFO controller interfaces the internal FIFO to the host or DMA interface and the card controller unit. The FIFO depth is configured for 1024 bytes. A single shared FIFO is used for read and write operations because read and write transfers to the cards do not occur simultaneously.

Figure 26-2: Combined Transmit and Receive FIFO

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000001_a53fb4571ab66061320f9ec4cbbff5428046114a26fa0aa280e9a917c9dd9fa2.png)

## Power and Pull-up Control and Card Detection Unit

The card detection unit looks for any changes in the card-detect signal for card insertion or card removal. The unit filters out the debounce associated with mechanical insertion or removal, and generates one interrupt to the host. The debounce filter value is programmed through the MSI\_DEBNCE register.

On power-on, the controller reads in the MSI\_CDETECT register and stores the value in the memory. Upon receiving a card-detect interrupt, the controller again reads the MSI\_CDETECT register to decide whether card was removed or inserted. The Card-Detect Signals figure shows the timing for the card-detect signal.

Figure 26-3: Card-Detect Signals

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000002_b5684461aaab58a1d2905ec1bfa03249cfe37c37fbba085d71360337023302eb.png)

## Internal Direct Memory Access Controller (IDMAC)

The Internal Direct Memory Access Controller (IDMAC) contains a single transmit or receive engine, which transfers data from the host memory to the device port and conversely. The controller uses a descriptor to move data efficiently from source to destination with minimal core intervention. Programs can configure the controller to interrupt the core in situations such as data transmit and receive transfer completion from the card, as well as other normal or error conditions.

The IDMAC and the core communicate through a single data structure. The IDMAC transfers the data received from the card to the data buffer in the host memory, and it transfers transmit data from the data buffer in the host memory to the MSI FIFO. Descriptors that reside in the memory act as pointers to these buffers.

A data buffer resides in physical memory space of the processor and consists of complete data or partial data. Buffers contain only data, while buffer status is maintained in the descriptor. Data chaining refers to data that spans multiple data buffers. However, a single descriptor cannot span multiple data.

A single descriptor is used for both reception and transmission. The base address of the list is written into descriptor list base address register ( MSI\_DBADDR ). A descriptor list is forward-linked. The last descriptor can point back to the first entry creating a ring structure. The descriptor list resides in the physical memory address space of the host. Each descriptor can point to a maximum of two data buffers.

Programs can enable or disable the IDMAC using the MSI\_CTL.INTDMAC bit of the BIU.

## DMA Descriptors

The IDMAC uses two types of descriptor structures:

- Dual-Buffer Structure - The skip length value programmed in the MSI\_BUSMODE.DSL bit field determines the distance between two descriptors.
- Chain Structure - Each descriptor points to a unique buffer and the next descriptor.

The Dual-Buffer Descriptor Structure and Chain Descriptor Structure figures show these descriptor structures.

Figure 26-5: Chain Descriptor Structure

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000003_02c3164f7db1a6a610e9610cb1f1b95caa62c011a55559475aac4874600bb999.png)

The Descriptor Format figure illustrates the internal format of a descriptor. The descriptor addresses must align with the 32-bit bus width. Each descriptor contains 16 bytes of control and status information. DES0 is a notation used to denote the [31:0] bits, DES1 to denote [63:32] bits, DES2 to denote [95:64] bits, and DES3 to denote [127:96] bits in a descriptor.

Figure 26-6: Descriptor Format

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000004_89474adf89422ecac90e44163026ea25d0d864d7cf7c00305b7ca7b90d849696.png)

The following tables provide descriptor bit descriptions. Bits not shown are reserved. The DES0 descriptor in the IDMAC (described in the IDMAC DES0 Descriptor table) contains control and status information.

Table 26-5: IDMAC DES0 Descriptor

|   Bits | Name                                  | Description                                                                                                                                                                                                                                                                                                                                            |
|--------|---------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|     31 | OWN                                   | When set, this bit indicates that the IDMAC owns the descriptor. When this bit is reset, it indicates that the host owns the descriptor. The IDMAC clears this bit when it completes the data transfer.                                                                                                                                                |
|     30 | Card Error Summary (CES)              | This error bit indicates the status of the transaction to or from the card. This bit is also present in the MSI_ISTAT register and indicates the logical OR of the follow- ing bits: EBE: End bit error RTO: Response timeout RCRC: Response CRC SBE: Start bit error DRTO: Data Read timeout DCRC: Data CRC for receive RE: Response error            |
|      5 | End of Ring (ER)                      | When set, this bit indicates that the descriptor list reached its final descriptor. The IDMAC returns to the base address of the list, creating a descriptor ring. This feature is meaningful for only a dual-buffer descriptor structure.                                                                                                             |
|      4 | Second Address Chained (CH)           | When set, this bit indicates that the second address in the descriptor is the next de- scriptor address rather than the second buffer address. When this bit is set, BS2 (DES1[25:13]) is all zeros.                                                                                                                                                   |
|      3 | First Descriptor (FS)                 | When set, this bit indicates that this descriptor contains the first buffer of the data. If the size of the first buffer is 0, next descriptor contains the beginning of the data.                                                                                                                                                                     |
|      2 | Last Descriptor (LD)                  | This bit is associated with the last block of a DMAtransfer. When set, the bit indi- cates that the buffers pointed to by this descriptor are the last buffers of the data. Af- ter this descriptor is completed, the remaining byte count is 0. In other words, after the descriptor with the LD bit set is completed, the remaining byte count is 0. |
|      1 | Disable Interrupt on Completion (DIC) | When set, this bit prevents the setting of the TI/RI bit of the IDMAC Status Register (IDSTS) for the data that ends in the buffer pointed to by this descriptor.                                                                                                                                                                                      |

The DES1 descriptor (described in the IDMAC DES1 Descriptor table) contains the buffer size.

Table 26-6: IDMAC DES1 Descriptor

| Bits   | Name                | Description                                                                                                                                                                                                                                                                                                                                                                     |
|--------|---------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:13  | Buffer 2 Size (BS2) | These bits indicate the second data buffer byte size which must be a multiple of 4. In the case where the buffer size is not a multiple of 4, the resulting behavior is undefined. If this field is 0, the DMAignores this buffer and proceeds to the next buffer if there is a dual-buffer structure. This field is not valid for chain structure; that is, if DES0[4] is set. |
| 12:0   | Buffer 1 Size (BS1) | Indicates the data buffer byte size which must be a multiple of 4. In the case where the buffer size is not a multiple of 4, the resulting behavior is undefined. This field must not be zero. Note: If there is only one buffer to be programmed, use only the buffer 1, and not buffer 2.                                                                                     |

The DES2 descriptor (described in the IDMAC DES2 Descriptor table) is the address pointer to the data buffer.

Table 26-7: IDMAC DES2 Descriptor

| Bits   | Name                            | Description                                                                                                  |
|--------|---------------------------------|--------------------------------------------------------------------------------------------------------------|
| 31:0   | Buffer Address Pointer 1 (BAP1) | These bits indicate the physical address of the first data buffer. The IDMAC ignores DES2 [1:0], internally. |

The DES3 descriptor (described in the IDMAC DES3 Descriptor table) is the address pointer to the next descriptor when the present descriptor is not

- the last descriptor in a chained descriptor structure, or
- the second buffer address for a dual-buffer structure

Table 26-8: IDMAC DES3 Descriptor

| Bits   | Name                                                     | Description                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------|----------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0   | Buffer Address Pointer 2/ Next Descriptor Address (BAP2) | These bits indicate the physical address of the second buffer when the dual-buffer structure is used. If the Second Address Chained (DES0[4]) bit is set, then this address contains the pointer to the physical memory where the next descriptor is present. If this pointer is not the last descriptor, then the next descriptor address pointer must be bus- width aligned (DES3[1:0] = 0. Internally, the LSBs are ignored. |

## Initialization

IDMAC initialization occurs as follows.

1. Write to IDMAC bus mode register ( MSI\_BUSMODE ) to set host bus access parameters.
2. Write to IDMAC interrupt enable register ( MSI\_IDINTEN ) to mask unnecessary interrupt causes.
3. The software driver creates either the transmit or the receive descriptor list. Then it writes to IDMAC descriptor list base address register ( MSI\_DBADDR ), which provides the IDMAC the starting address of the list.
4. The IDMAC engine attempts to acquire descriptors from the descriptor lists.

## Host Bus Burst Access

The IDMAC executes fixed-length burst transfers on the bus interface when the MSI\_BUSMODE.FB bit =1. The maximum burst length is indicated and limited by the MSI\_BUSMODE.PBL bit field. The descriptors are always accessed in the maximum burst-size for the 16-bytes to be read (4-word burst).

The IDMAC initiates a data transfer only when sufficient space to accommodate the configured burst is available in the FIFO or the number of bytes to the end of data, when less than the configured burst-length. The IDMAC indicates the start address and the number of transfers required to the bus interface. When the bus interface is configured for fixed-length bursts, it transfers data using the best combination of INCR4/8/16 and SINGLE transactions. Otherwise, in no fixed-length bursts, it transfers data using INCR (undefined length) and SINGLE transactions.

The transmit and receive data buffers must be aligned to data bus width (32-bit).

NOTE: Due to the bus protocol limitation, the burst mode address should not cross the 1 KB boundary. Address [31:10] should not change during a burst.

## Buffer Size Calculations

The software knows the amount of data to transmit or receive. For transmitting to the card, the IDMAC transfers the exact number of bytes to the FIFO, indicated by the buffer size field of DES1.

If a descriptor is not marked as last (LS bit of DES0), then the corresponding buffers of the descriptor are full. Its buffer size field indicates the amount of valid data in a buffer. If a descriptor is marked as last, then the buffer cannot be full, as indicated by the buffer size in DES1. The software is aware of the number of locations that are valid in this case.

## Data Transmit/Receive

IDMAC transmission occurs as follows:

1. The software sets up the descriptor (DES0-DES3) for transmission and sets the OWN bit (DES0[31]). The software also prepares the data buffer.
2. The software programs the write data command in the MSI\_CMD register in the BIU.
3. The software also programs the required transmit threshold level ( MSI\_FIFOTH.TXWM bit field).
4. The IDMAC determines that a write data transfer must occur as a consequence of step 2.
5. The IDMAC engine fetches the descriptor and checks the OWN bit. If the OWN bit is not set, it means that the software owns the descriptor. In this case, the IDMAC enters the suspend state and asserts the MSI\_IDSTS.DU bit. The software must release the IDMAC by writing any value to the poll demand register.
6. It then waits for command done ( MSI\_ISTAT.CMDDONE ) bit and no errors from BIU which indicates that a transfer can occur.
7. The IDMAC engine waits for a DMA interface request from the BIU. This request is generated based on the programmed transmit threshold value. For the last bytes of data which cannot be accessed using a burst, SINGLE transfers are performed.
8. The IDMAC fetches the transmit data from the data buffer and transfers it to the FIFO for transmission to the card.
9. When data spans across multiple descriptors, the IDMAC fetches the next descriptor and continues with its operation with the next descriptor. The last descriptor bit in the descriptor indicates whether the data spans multiple descriptors or not.
10. When data transmission is complete, status information is updated in IDMAC status register ( MSI\_IDSTS ) by setting the transmit interrupt, when enabled. Also, the IDMAC performs a write transaction to DES0 to clear the OWN bit.

## IDMAC reception occurs as follows:

1. The software sets up the Descriptor (DES0-DES3) for reception, sets the OWN bit (DES0[31]).

2. The software programs the read data command in the MSI\_CMD register in the BIU.
3. The software programs the required receive threshold level ( MSI\_FIFOTH.RXWM bit field).
4. The IDMAC determines that a read data transfer must occur as a consequence of step 2.
5. The IDMAC engine fetches the descriptor and checks the OWN bit. If the OWN bit is not set, it means that the host owns the descriptor. In this case, the DMA enters suspend state and asserts the release the IDMAC by writing any value to the poll demand register.
6. It then waits for the command done ( MSI\_ISTAT.CMDDONE ) bit and no errors from BIU which indicates that a transfer can occur.
7. The IDMAC engine waits for a DMA interface request from the BIU. This request is generated based on the programmed receive threshold value. For the last bytes of data which cannot be accessed using a burst, SINGLE transfers are performed.
8. The IDMAC fetches the data from the FIFO and transfers to the memory.
9. When data spans across multiple descriptors, the IDMAC fetches the next descriptor and continues with its operation with the next descriptor. The last descriptor bit in the descriptor indicates whether the data spans multiple descriptors or not.
10. When data reception is complete, status information is updated in the IDMAC status register ( MSI\_IDSTS ) by setting the receive interrupt, when enabled. Also, the IDMAC performs a write transaction to DES0 to clear the OWN bit.

## Interrupts

Interrupts can be generated as a result of various DMAC events. The IDMAC status register ( MSI\_IDSTS ) contains all the bits that can cause an interrupt. The IDMAC interrupt enable register ( MSI\_IDINTEN ) contains an enable bit for each of the events that can cause an interrupt.

There are two groups of summary interrupts-normal and abnormal-as outlined in the MSI\_IDSTS register. Interrupts are cleared by writing a 1 to the corresponding bit position. When all the enabled interrupts within a group are cleared, the corresponding summary bit is cleared.

Interrupts are not queued and if the interrupt event occurs before the software has responded to it, no additional interrupts are generated. For example, the receive interrupt ( MSI\_IDSTS.RI ) indicates that one or more data was transferred to the buffer. An interrupt is generated only once for simultaneous, multiple events. The software must scan the MSI\_IDSTS register for the interrupt cause.

NOTE: The final interrupt signal from MSI is a logical OR of the interrupt from the BIU and the IDMAC.

## Finite State Machine (FSM)

The IDMAC finite state machine can be in any one of the states reflected in the MSI\_IDSTS.FSM bit field.

The FSM uses the following sequence.

1. IDMAC performs four accesses for fetching the descriptor.

2. Stores descriptors in internal register and also issues a FIFO reset when it is first descriptor.
3. Each bit is checked for correctness. In case of bit mismatches, appropriate error bit is set. If it is first descriptor, then issues a FIFO reset and wait until FIFO reset is complete. The error status indicates one of the following:
- Response timeout
- Response CRC error
- Data receive timeout
- Response error
4. The FSM waits in current state until DMA request is asserted, which implies that, FIFO:
- For DMA write request wait - Holds the number of data, indicated by FIFO RX watermark. In case of error due to response timeout or error, FSM goes to DESC\_CLOSE state to close descriptor.
- For DMA read request wait - Holds number of data, indicated by FIFO TX watermark. In case of error, FSM goes to DESC\_CLOSE state to close descriptor.
5. FSM performs the following:
- For DMA write - Requests a write to SCB. If number of beats in one transfer is greater than PBL, then one of the following occurs:

Burst count to SCB is PBL value

Single transfers are initiated

- For DMA read - Requests a read from SCB. If number of beats in one transfer is greater than PBL, then one of the following occurs:

Burst count to SCB is PBL value

Single transfers are initiated

6. After the programmed transfer count is accessed from the memory, the OWN bit in descriptor is closed. If a transfer spans more than one descriptor, the FSM fetches the next descriptor. If the transfer ends with the current descriptor, the FSM goes to the idle state after setting the receive or transmit interrupt. Depending on the descriptor structure-dual buffer or chained-the appropriate starting address of descriptor is loaded. If it is the second data buffer of the dual buffer, the descriptor is not fetched again.

## Abort Operation

The host issues CMD12 when a data transfer on the card data lines is in progress. The FSM closes the present descriptor after completing the transfer of data until a DTO interrupt is asserted. Once an abort command is issued, the DMAC performs single burst transfers.

1. For a card write, the FSM keeps pushing data to the FIFO after fetching it from the memory until a DTO interrupt is asserted. The controller asserts a busy clear interrupt after the DTO interrupt. It ensures that the card has completed driving the busy signal. This sequence occurs to keep the card clock running so that CMD12 is reliably sent to the card.

2. For a card read, the FSM keeps popping data from the FIFO and writes to the memory until a DTO interrupt is generated. This sequence is required since the DTO interrupt is not generated until and unless all the FIFO data is emptied.

NOTE: If an FBE occurs, the IDMAC FSM does not close the current descriptor and the remaining unread descriptors.

If a write abort occurs, the IDMAC FSM closes only the current descriptor during which the abort occurred. The IDMAC FSM does not close the current descriptor and the remaining unread descriptors.

If a read abort occurs, the IDMAC FSM pops the data out of the FIFO and writes it to the corresponding descriptor data buffers. The remaining unread descriptors are not closed.

## FIFO Overflow and Underflow

During normal data transfer conditions, FIFO overflow and underflow do not occur. If there is a programming error, then a FIFO overflow or underflow can result as shown in the following examples.

Transmit settings: MSI\_BUSMODE.PBL =4, MSI\_FIFOTH.TXWM =1

In this example, if the FIFO has only one location empty, it issues a DMA request to the IDMAC FSM. Because MSI\_BUSMODE.PBL =4, the IDMAC FSM performs 4 pushes into the FIFO which results in a FIFO overflow interrupt.

Receive settings: MSI\_BUSMODE.PBL =4, MSI\_FIFOTH.RXWM =1

In this example, if the FIFO has only one location filled, it issues a DMA request to the IDMAC FSM. Because MSI\_BUSMODE.PBL =4, the IDMAC FSM performs 4 pops to the FIFO which results in a FIFO underflow interrupt.

The software must ensure that the number of bytes to be transferred as indicated in the descriptor are a multiple of 4. For example, if the MSI\_BYTCNT register =13, the number of bytes indicated in the descriptor must be 16.

## Card Interface Unit

The Card Interface Unit (CIU) interfaces with the Bus Interface Unit (BIU) on one side and with the SD/MMC/ SDIO cards or devices on other side. The software writes command parameters to the MSI BIU control registers, and these parameters are then passed to the CIU. Depending on control register values, the CIU generates card command and data traffic on a selected card bus according to card protocol.

As shown in the MSI Block Diagram, the CIU consists of the following primary functional blocks:

- Command path
- Datapath
- SDIO interrupt control
- Clock control
- Multiplex or demultiplex unit

The following software restrictions must be met for proper CIU operation:

- Only one data transfer command can be issued at a time.
- During an open-ended card write operation, if the card clock is stopped because the FIFO is empty, the software must first fill the data into the FIFO and start the card clock. It can then issue only a stop or abort command to the card.
- During an SDIO card transfer, if the card function is suspended and the software wants to resume the suspended transfer, it must first reset the FIFO and start the resume command as if it were a new data transfer command.
- When issuing card reset commands (CMD0, CMD15 or CMD52) while a card data transfer is in progress, the software must set the MSI\_CMD.STPABORTCMD bit so that the MSI can stop the data transfer after issuing the card reset command.
- When the data end error bit ( MSI\_ISTAT.EBE ) is set, the MSI does not guarantee SDIO interrupts. The software must ignore the SDIO interrupts and issue the stop or abort command to the card, so that the card stops sending the read data.
- If the card clock is stopped because the FIFO is full during a card read, the software or DMA must read at least two FIFO locations to start the card clock.

## Command Path

The command path performs the following functions.

- Loads clock parameters
- Loads card command parameters
- Sends commands to card bus
- Receives responses from card bus
- Sends responses to BIU
- Drives the P-bit on command line

A new command is issued to the MSI by programming the BIU registers and setting the MSI\_CMD.STARTCMD bit. The command path loads this new command (command, command argument, timeout) and sends acknowledge to the BIU.

Once the new command is loaded, the command path state machine sends a command to the SD/MMC bus (this includes the internally generated CRC7). The command path state machine receives a response if there is any. The state machine then sends the received response and signals to the BIU that the command is done, and then waits for eight clock cycles before loading a new command.

## Load Command Parameters

One of the following commands or responses is loaded in the command path:

- New command from BIU when the MSI\_CMD.STARTCMD bit is set.

- Internally generated auto-stop command when the data path ends, the stop command request is loaded.
- IRQ response with RCA =0x000 when the command path is waiting for an IRQ response from the MMC card, then the MSI\_CTL.IRQRESP bit is set.

Loading a new command from the BIU in the command path depends on the following MSI\_CMD register bit settings:

- Update clock registers only using the MSI\_CMD.UCLKREGS bit. If the MSI\_CMD.UCLKREGS bit =1, the command path updates only the clock enable and clock divider registers. If MSI\_CMD.UCLKREGS =0, the command path loads the command, command argument, and timeout registers, then starts processing the new command.
- Wait for previous data to complete using the MSI\_CMD.WTPRIVDATA bit. If the MSI\_CMD.WTPRIVDATA bit =1, the command path loads the new command under one of the following conditions:
- Immediately, if the data path is free (that is, there is no data transfer in progress), or if an open-ended data transfer is in progress ( MSI\_BYTCNT =0).
- After completion of the current data transfer, if a predefined data transfer is in progress.

## Send Command and Receive Response

Once a new command is loaded in the command path with MSI\_CMD.UCLKREGS bit =0, the command path state machine sends out a command on the SD/MMC bus. The SD\_MMC Command Path State Machine figure illustrates the command path state machine.

Figure 26-7: SD\_MMC Command Path State Machine

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000005_a265ba000c6415af9a821705d781be56dc8c1a6d4ceda81290c8a0254cd84b0b.png)

The command path state machine performs the following functions, according to following command register bit values:

1. MSI\_CMD.SENDINIT (send initialization). Initialization sequence of 80 clocks is sent before sending the command.
2. MSI\_CMD.RXPECT (response expected). A response is expected for the command. After the command is sent out, the command path state machine receives a 48-bit or 136-bit response and sends it to the BIU. If the start

bit of the card response is not received within the number of clocks programmed in the timeout register, then the MSI\_ISTAT.RTO (response timeout) and MSI\_ISTAT.CD (command done) bits are set as a signal to the BIU. If the MSI\_CMD.RXPECT bit is not set, the command path sends out a command and signals a response done to the BIU, that is, the MSI\_ISTAT.CD bit is set.

3. MSI\_CMD.RLEN (response length). If =1, a 136-bit response is received; if =0, a 48-bit response is received.
4. MSI\_CMD.CHKRESPCRC (check response CRC). If =1, the command path compares CRC7 received in the response with the internally-generated CRC7. If the two do not match, the response CRC error is signaled to the BIU, that is, the response CRC error bit ( MSI\_ISTAT.RCRC ) is set.

## Send Response to BIU

If the MSI\_CMD.RXPECT bit =1, the received response is sent to the BIU. The MSI\_RESP0 register is updated for a short response, and the MSI\_RESP3 , MSI\_RESP2 , MSI\_RESP1 , and MSI\_RESP0 registers are updated on a long response, after which the MSI\_ISTAT.CMDDONE bit is set. If the CIU sends an auto-stop command as a response, the response is saved in the MSI\_RESP1 register, after which the MSI\_ISTAT.ACD (auto command done) bit is set.

Additionally, the command path checks for the following.

- Transmission bit =0
- Command index in response matches command index of the sent command
- End bit =1 in received card response

The command index is not checked for a 136-bit response or if the MSI\_CMD.CHKRESPCRC bit is cleared. For a 136-bit response and reserved CRC 48-bit responses, the command index is reserved (=111111).

## Driving a P-bit on CMD Line

The command path drives a P-bit =1 on the CMD line between two commands when a response is not expected. If a response is expected, the P-bit is driven after the response is received and before the start of the next command.

## Datapath

The datapath block pops the data FIFO and transmits data on MSI data lines during a write data transfer. Or, it receives data from data lines and pushes it into the FIFO during a read data transfer. Whenever a data transfer command is not in-progress, the datapath loads new data parameters;

- data expected
- read/write data transfer
- stream or block transfer
- block size
- byte count

- card type
- timeout registers

If the MSI\_CMD.DXPECT bit =1, the new command is a data transfer command and the datapath starts one of the following:

- Data transmit if the MSI\_CMD.RDWR (read/write) bit =1
- Data receive if MSI\_CMD.RDWR (read/write) bit 0

## Auto-Stop

The MSI internally generates a stop command and is loaded in the command path when the MSI\_CMD.SENDASTOP bit is set. The auto-stop command helps to send an exact number of data bytes using a stream read or write for the MMC, and a multiple-block read or write for the SD memory transfer for SD cards.

The software must set the MSI\_CMD.SENDASTOP bit according to details listed in the Auto-Stop Generation table.

Table 26-9: Auto-Stop Generation

| Card Type   | Transfer Type        | Byte Count   | MSI_CMD. SENDASTOP bit =1   | Comments                           |
|-------------|----------------------|--------------|-----------------------------|------------------------------------|
| MMC         | Stream read          | 0            | No                          | Open-ended stream                  |
| MMC         | Stream read          | >0           | Yes                         | Auto-stop after all bytes transfer |
| MMC         | Stream write         | 0            | No                          | Open-ended stream                  |
| MMC         | Stream write         | >0           | Yes                         | Auto-stop after all bytes transfer |
| MMC         | Single-block read    | >0           | No                          | Byte count = 0 is illegal          |
| MMC         | Single-block write   | >0           | No                          | Byte count = 0 is illegal          |
| MMC         | Multiple-block read  | 0            | No                          | Open-ended multiple block          |
| MMC         | Multiple-block read  | >0           | Yes *1                      | Pre-defined multiple block         |
| MMC         | Multiple-block write | 0            | No                          | Open-ended multiple block          |
| MMC         | Multiple-block write | >0           | Yes                         | Pre-defined multiple block         |
| SDMEM       | Single-block read    | >0           | No                          | Byte count = 0 is illegal          |
| SDMEM       | Single-block write   | >0           | No                          | Byte count = 0 is illegal          |
| SDMEM       | Multiple-block read  | 0            | No                          | Open-ended multiple block          |
| SDMEM       | Multiple-block read  | >0           | Yes                         | Auto-stop after all bytes transfer |
| SDMEM       | Multiple-block write | 0            | No                          | Open-ended multiple block          |
| SDMEM       | Multiple-block write | >0           | Yes                         | Auto-stop after all bytes transfer |
| SDIO        | Single-block read    | >0           | No                          | Byte count = 0 is illegal          |
| SDIO        | Single-block write   | >0           | No                          | Byte count = 0 is illegal          |

Table 26-9: Auto-Stop Generation (Continued)

| Card Type   | Transfer Type        | Byte Count   | MSI_CMD. SENDASTOP bit =1   | Comments                   |
|-------------|----------------------|--------------|-----------------------------|----------------------------|
|             | Multiple-block read  | 0            | No                          | Open-ended multiple block  |
|             | Multiple-block read  | >0           | No                          | Pre-defined multiple block |
|             | Multiple-block write | 0            | No                          | Open-ended multiple block  |
|             | Multiple-block write | >0           | No                          | Pre-defined multiple block |

- *1 The condition under which the transfer mode blocks transfer and byte\_count equals block size is treated as a single-block data transfer command for both MMC and SD cards. If byte\_count = n × block\_size (n = 2, 3, …), the condition is treated as a predefined multiple-block data transfer command. For an MMC card, the host software can perform a predefined data transfer in two ways:
1. Issue the CMD23 command before issuing CMD18/CMD25 commands to the card - in this case, issue CMD18/CMD25 commands without setting the MSI\_CMD.SENDASTOP bit.
2. Issue CMD18/CMD25 commands without issuing CMD23 command to the card, with the MSI\_CMD.SENDASTOP bit set. In this case, an internally generated auto-stop command after the programmed byte count terminates the multiple-block data transfer.

The following list explains different conditions for the auto-stop command.

- Stream read for MMC card with byte count greater than 0. The MSI generates an internal stop command and loads it into the command path. The end bit of the stop command is sent out when the last byte of data is read from the card and no extra data byte is received. If the byte count is less than 6 (48 bits), a few extra data bytes are received from the card before the end bit of the stop command is sent.
- Stream write for MMC card with byte count greater than 0. The MSI generates an internal stop command and loads it into the command path. The end bit of the stop command is sent when the last byte of data is transmitted on the card bus and no extra data byte is transmitted. If the byte count is less than 6 (48 bits), the datapath transmits the data last in order to meet the above condition.
- Multiple-block read memory for SD card with byte count greater than 0. If the block size is less than 4 (singlebit data bus), 16 (4-bit data bus), or 32 (8-bit data bus) bytes, the auto-stop command is loaded in the command path after all the bytes are read. Otherwise, the stop command is loaded in the command path so that the end bit of the stop command is sent after the last data block is received.
- Multiple-block write memory for SD card with byte count greater than 0. If the block size is less than 3 (singlebit data bus), 12 (4-bit data bus), or 24 (8-bit data bus), the auto-stop command is loaded in the command path after all data blocks are transmitted. Otherwise, the stop command is loaded in the command path so that the end bit of the stop command is sent after the end bit of the CRC status is received.
- Precaution for host software during auto-stop. Whenever an auto-stop command is issued, the host software must not issue a new command to the MSI until the MSI sends the auto-stop command and the data transfer completes. If the host issues a new command during a data transfer with the auto-stop in progress, an auto-stop command can be sent after the new command is sent and its response is received. This process can delay sending the stop command, which transfers extra data bytes. For a stream write, extra data bytes are erroneous data

that can corrupt the card data. If the host wants to terminate the data transfer before the data transfer is complete, it can issue a stop or abort command. In this case, the MSI does not generate an auto-stop command.

## Non-Data Transfer Commands that Use Datapath

Some non-data transfer commands (non-read/write commands) also use the datapath. The Non-Data Transfer Commands and Requirements table lists the commands and register programming requirements.

Table 26-10: Non-Data Transfer Commands and Requirements

|                                       | CMD27                                 | CMD30                                 | CMD42                                 | ACMD13                                | ACMD22                                | ACMD51                                |
|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------|
| Command Register Programming          | Command Register Programming          | Command Register Programming          | Command Register Programming          | Command Register Programming          | Command Register Programming          | Command Register Programming          |
| Cmd_index                             | 6'h1B                                 | 6'h1E                                 | 6'h2A                                 | 6'h0D                                 | 6'h16                                 | 6'h33                                 |
| Response_expect                       | 1                                     | 1                                     | 1                                     | 1                                     | 1                                     | 1                                     |
| Response_length                       | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     |
| Check_response_crc                    | 1                                     | 1                                     | 1                                     | 1                                     | 1                                     | 1                                     |
| Data_expected                         | 1                                     | 1                                     | 1                                     | 1                                     | 1                                     | 1                                     |
| Read/write                            | 1                                     | 0                                     | 1                                     | 0                                     | 0                                     | 0                                     |
| Transfer_mode                         | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     |
| Send_auto_stop                        | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     |
| Wait_prevdata_complete                | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     |
| Stop_abort_cmd                        | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     | 0                                     |
| Command Argument register programming | Command Argument register programming | Command Argument register programming | Command Argument register programming | Command Argument register programming | Command Argument register programming | Command Argument register programming |
|                                       | Stuff bits                            | 32-bit write pro- tect data address   | Stuff bits                            | Stuff bits                            | Stuff bits                            | Stuff bits                            |
| Block Size register programming       | Block Size register programming       | Block Size register programming       | Block Size register programming       | Block Size register programming       | Block Size register programming       | Block Size register programming       |
|                                       | 16                                    | 4                                     | Num_bytes *1                          | 64                                    | 4                                     | 8                                     |
| Byte Count register programming       | Byte Count register programming       | Byte Count register programming       | Byte Count register programming       | Byte Count register programming       | Byte Count register programming       | Byte Count register programming       |
|                                       | 16                                    | 4                                     | Num_bytes *1                          | 64                                    | 4                                     | 8                                     |

## SDIO Interrupt Control

Interrupts for SD cards are reported to the BIU by asserting an interrupt signal for two clock cycles. SDIO cards signal an interrupt by asserting MSI0\_D1 low during the interrupt period; The interrupt control state machine determines an interrupt period for the selected card. An interrupt period is always valid for cards in 1-bit data mode. An interrupt period for a wide-bus active or selected card is valid for the following conditions:

- Card is idle
- Non-data transfer command in progress

- Third clock after end bit of data block between two data blocks
- From two clocks after end bit of last data until end bit of next data transfer command

Bear in mind that, in the following situations, the MSI does not sample the SDIO interrupt of the selected card when the card data width is 4 bits. Since the SDIO interrupt is level-triggered, it is sampled in a further interrupt period and the host does not lose any SDIO interrupt from the card.

1. Read/write resume. The CIU treats the resume command as a normal data transfer command. SDIO interrupts during the resume command are handled similarly to other data commands. According to the SDIO specification, for the normal data command the interrupt period ends after the command end bit of the data command. For the resume command, it ends after the response end bit. For the resume command, the MSI stops the interrupt sampling period after the resume command end bit, instead of stopping after the response end bit of the resume command.
2. Suspend during read transfer. If the host suspends the read data transfer, the host sets the MSI\_CTL.RDABORT bit to reset the data state machine. In the CIU, the SDIO interrupts are handled such that the interrupt sampling starts after the host sets the MSI\_CTL.RDABORT bit. In this case, the MSI does not sample SDIO interrupts between the period from response of the suspend command.

## Clock Control

The clock control block provides the clock frequencies required for SD/MMC cards.

The source clock for the MSI block and the input clock for clock dividers of the clock control block is clocked by CLK09 clock from CDU module. Refer to CDU chapter for more details on CLK09 clock.

The source clock is used to generate the card clock frequencies. The card clock can have different clock frequencies, since the SD card can be a low-speed SD card or a full-speed SD card. The MSI allows the card to operate at different clock frequencies.

The clock frequency of a card depends on the following clock control registers:

- Clock divider register ( MSI\_CLKDIV ). An internal clock divider is used to generate different clock frequencies required for the card. The clock divider is an 8-bit value that provides a clock division factor from 1 to 510. A value of 0 represents a clock-divider bypass, a value of 1 represents a divide by 2, a value of 2 represents a divide by 4, and so on.
- Clock control register ( MSI\_CLKEN ). The MSI module can enable or disable the card clock under the following conditions:
- The clock for a card is enabled when the MSI\_CLKEN.EN0 bit for a card is programmed (= 1) or disabled (= 0).
- Setting the MSI\_CLKEN.LP0 =1. enables the low-power mode of a card. If low-power mode is enabled to save card power, the clock signal is disabled when the card is idle for at least 8 card clock cycles. It is enabled when a new command is loaded and the command path goes to a non-idle state.

Additionally, the clock of the card is disabled when:

- An internal FIFO is full on a card read (no more data can be received from card)
- The FIFO is empty on a card write (no data is available for transmission).

Disabling the clock in these situations helps to avoid FIFO overrun and underrun conditions.

Under the following conditions, the card clock is stopped or disabled for the card:

- The clock can be disabled by writing to the clock enable MSI\_CLKEN register.
- If low-power mode is selected and a card is idle, or not selected for 8 clocks.
- The FIFO is full and data path cannot accept more data from the card and data transfer is incomplete to avoid FIFO overrun.
- The FIFO is empty and data path cannot transmit more data to the card and data transfer is incomplete to avoid FIFO underrun.

NOTE: The host software must take care while changing the MSI\_CLKDIV register values. The card clock must be disabled through the MSI\_CLKEN register before changing the values of the MSI\_CLKDIV register.

## MSI Data Transfer Modes

The following sections provide information on data transfer using the MSI interface.

## Data Transmit

The data transmit state machine starts data transmission two clocks after a response for the data write command is received. The transmission occurs even if the command path detects a response error or response CRC error. See the Data Transmit State Machine figure. If a response is not received from the card because of a response timeout, data is not transmitted. Depending upon the value of the MSI\_CMD.XFRMODE bit, the data transmit state machine puts data on the card data bus in a stream or in blocks.

Figure 26-8: Data Transmit State Machine

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000006_497431aa8df970f3f16f53d853584740883d18b5168d5cf86b256e1367890c91.png)

## Stream Data Transmit

If the MSI\_CMD.XFRMODE bit =1, it is a stream-write data transfer. The data path pops the FIFO from the BIU and transmits in a stream to the card data bus. If the FIFO becomes empty, the card clock is stopped and restarted once data is available in the FIFO.

If the MSI\_BYTCNT register is programmed to 0, it is an open-ended stream-write data transfer. During this data transfer, the data path continuously transmits data in a stream until the host software issues a stop command. A stream data transfer is terminated when the end bit of the stop command and end bit of the data match over two clocks.

If the MSI\_BYTCNT register is programmed with a non-zero value and the MSI\_CMD.SENDASTOP bit is set, the stop command is internally generated. The command is loaded in the command path when the end bit of the stop command occurs after a match with the last byte of the stream-write transfer. This data transfer can also terminate if the software issues a stop command before all the data bytes transfer to the card bus.

## Single Block Data

If the MSI\_CMD.XFRMODE bit =0 and the MSI\_BYTCNT register value is equal to the value of the MSI\_BLKSIZ register, a single-block write-data transfer occurs. The data transmit state machine sends data in a single block, where the number of bytes equals the block size, including the internally-generated CRC16.

If the MSI\_CTYPE register bit is set for a 1-bit, 4-bit, or 8-bit data transfer, the data transmits on 1, 4, or 8 data lines, respectively. CRC16 is separately generated and transmitted for 1, 4, or 8 data lines, respectively.

After a single data block is transmitted, the data transmit state machine receives the CRC status from the card and signals a data transfer to the BIU. This operation happens when the MSI\_ISTAT.DTO bit =1. If a negative CRC status is received from the card, the data path signals a data CRC error to the BIU by setting the MSI\_ISTAT.DCRC bit.

Additionally, if the start bit of the CRC status is not received before two clocks after the end of the data block, a CRC status start bit error is signaled to the BIU. The MSI\_ISTAT.EBE bit is set.

## Multiple Block Data

A multiple-block write-data transfer occurs if the MSI\_CMD.XFRMODE bit =0 and the value in the MSI\_BYTCNT register is not equal to the value of the MSI\_BLKSIZ register. The data transmit state machine sends data in blocks, where the number of bytes in a block equals the block size, including the internally-generated CRC16.

If the MSI\_CTYPE register bit is set to 1-bit, 4-bit, or 8-bit data transfer, the data transmits on 1, 4, or 8 data lines, respectively. CRC16 is separately generated and transmitted on 1, 4, or 8 data lines, respectively.

After one data block is transmitted, the data transmit state machine receives the CRC status from the card. If the remaining MSI\_BYTCNT becomes 0, the data path signals to the BIU that the data transfer is complete. This operation happens when the MSI\_ISTAT.DTO bit =1. If the remaining data bytes are greater than 0, the data path state machine starts to transmit another data block.

If a negative CRC status is received from the card, the data path signals a data CRC error to the BIU by setting the MSI\_ISTAT.DCRC bit. The block continues further data transmission until all the bytes are sent. Additionally, if the CRC status start bit is not received within two clocks after the end of a data block, a CRC status start bit error is signaled to the BIU. The MSI\_ISTAT.EBE bit is set. Further data transfer is terminated.

If the MSI\_CMD.SENDASTOP is set, the stop command is internally generated during the transfer of the last data block. No extra bytes are transferred to the card. The end bit of the stop command does not always exactly match the end bit of the CRC status in the last data block.

If the block size is less than 4, 16, or 32 for card data widths of 1 bit, 4 bits, or 8 bits, respectively, the data transmit state machine terminates the data transfer when all the data has transferred. The internally generated stop command is loaded in the command path.

If the MSI\_BYTCNT register =0 (the block size must be greater than 0), it is an open-ended block transfer. The data transmit state machine for this type of data transfer continues the block-write data transfer until the host software issues a stop or abort command.

## Data Receive

The data-receive state machine receives data two clock cycles after the end bit of a data read command, even if the command path detects a response error or response CRC error. See the Data Receive State Machine figure. If a response is not received from the card because a timeout error occurs, the BIU does not receive the signal that the data transfer is complete. This error happens when the command sent by the MSI is an illegal operation for the card. The error prevents the card from starting a read data transfer.

If data is not received before the data timeout, the data path signals a data timeout to the BIU and an end to the data transfer done. Based on the value of the MSI\_CMD.XFRMODE bit, the data-receive state machine gets data from the card data bus in a stream or blocks.

Figure 26-9: Data Receive State Machine

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000007_519ce68e4ff5caba60ce7b583d4d75662bad8f9a3c46df6973022cabd151af57.png)

## Stream Data Read

A stream-read data transfer occurs if the MSI\_CMD.XFRMODE bit =1, at which time the data path receives data from the card and pushes it to the FIFO. If the FIFO becomes full, the card clock stops and restarts once the FIFO is no longer full.

An open-ended stream-read data transfer occurs when the MSI\_BYTCNT register equals 0. During this type of data transfer, the data path continuously receives data in a stream until the host software issues a stop command. A stream data transfer terminates two clock cycles after the end bit of the stop command.

If the MSI\_BYTCNT register contains a non-zero value and the MSI\_CMD.SENDASTOP bit =1, a stop command is internally generated and loaded into the command path. In the command path, the end bit of the stop command occurs after the last byte of the stream data transfer is received. This data transfer can terminate if the host issues a stop or abort command before all the data bytes are received from the card.

## Single Block Data Read

A single-block read-data transfer occurs if the MSI\_CMD.XFRMODE bit =0 and the value of the MSI\_BYTCNT register is equal to the value of the MSI\_BLKSIZ register. When a start bit is received before the data times out, data bytes equal to the block size and CRC16 are received and checked with the internally-generated CRC16.

If the MSI\_CTYPE register bit for the card is set to a 1-bit, 4-bit, or 8-bit data transfer, data is received from 1, 4, or 8 data lines, respectively. CRC16 is separately generated and checked for 1, 4, or 8 data lines, respectively. If there is a CRC16 mismatch, the data path signals a data CRC error to the BIU. If the received end bit is not 1, the BIU receives an end-bit error.

## Multiple Block Data Read

If the MSI\_CMD.XFRMODE =0 and the value of the MSI\_BYTCNT register is not equal to the value of the MSI\_BLKSIZ register, it is a multiple-block read-data transfer. The data-receive state machine receives data in blocks, where the number of bytes in a block is equal to the block size, including the internally-generated CRC16.

If the MSI\_CTYPE register bit for the card is set to a 1-bit, 4-bit, or 8-bit data transfer, data is received from 1, 4, or 8 data lines, respectively. CRC16 is separately generated and checked for 1, 4, or 8 data lines, respectively. After a data block is received, if the remaining MSI\_BYTCNT becomes 0, the data path signals a data transfer to the BIU.

If the remaining data bytes are greater than 0, the data path state machine causes another data block to be received. If CRC16 of a received data block does not match the internally-generated CRC16, a data CRC error to the BIU and data reception continue further data transmission until all bytes are transmitted. Additionally, if the end of a received data block is not 1, data on the data path signals terminate the bit error to the CIU. The data-receive state machine terminates data reception, waits for a data timeout, and signals to the BIU that the data transfer is complete.

If the MSI\_CMD.SENDASTOP bit =1, the stop command is internally generated when the last data block is transferred, where no extra bytes are transferred from the card. The end bit of the stop command does not always exactly match the end bit of the last data block.

If the requested block size for data transfers to cards is less than 4, 16, or 32 bytes for 1-bit, 4-bit, or 8-bit data transfer modes, respectively, the data-transmit state machine terminates the data transfer when all data transfers. The internally-generated stop command is loaded in the command path. The data path ignores any subsequent data received from the card.

If the MSI\_BYTCNT =0 (the block size must be greater than 0), it is an open-ended block transfer. For this type of data transfer, the data-receive state machine continues the block-read data transfer until the host software issues a stop or abort command.

## Data Transfer Commands

Data transfer commands transfer data between the memory card and the MSI. To send a data command, the MSI needs a command argument, total data size, and block size.

Prior to a data transfer command, software must confirm that the card is not busy and is in a transfer state. This confirmation uses the CMD13 and CMD7 commands, respectively. For the data transfer commands, it is important that the same bus width that is programmed in the card is set in the MSI\_CTYPE register.

The MSI generates an interrupt for different conditions during data transfer, which are reflected in the MSI\_ISTAT register.

NOTE: The DCRC, SBEBCI, EBE, and SBEBCI conditions indicate that the received data could have errors. If there was a response timeout, then no data transfer occurred. For more information, see the Register Descriptions section.

## Transmission and Reception with Internal DMAC (IDMAC)

The general sequence of events for transmit and receive is as follows.

1. Program the required programming in the bus mode register ( MSI\_BUSMODE ). If the MSI\_CTL.INTDMAC bit is disabled during the middle of an IDMAC transfer, it has no effect. It only takes effect for a new data transfer command. Issuing a software reset immediately terminates the transfer. It is recommended that the software issue a reset to the DMA interface by setting the MSI\_CTL.DMARST bit and then issuing an IDMAC software reset using the MSI\_CTL.CTLRST bit. Program the fixed burst bit ( MSI\_BUSMODE.FB ) appropriately for system performance.
2. When a descriptor unavailable interrupt is asserted, the software must form the descriptor, appropriately set its own bit and then write to poll the demand register for the IDMAC to refetch the descriptor.
3. It is always appropriate for the application to enable abnormal interrupts since any errors related to the transfer are reported to the application.

## MSI Event Control

The following sections describe MSI events.

## Dedicated Interrupt Pins

Interrupt lines are defined for only eSDIO devices, which are connected to the controller interrupt line in MSI. These interrupt lines can operate even when the card clock is switched off and can be used only during an asynchronous interrupt period.

## MSI Status and Error Signals

The following provides information on MSI errors.

## Error Detection

The MSI has an error detection mechanism which operates for any of the following situations.

## Response

- Response timeout - Response expected with the response start bit is not received within programmed number of clocks in timeout register.
- Response CRC error - Response is expected and check response CRC requested; response CRC7 does not match with the internally-generated CRC7.
- Response error - Response transmission bit is not 0, command index does not match with the command index of the send command, or response end bit is not 1.

## Data Transmit

- No CRC status - During a write data transfer, if the CRC status start bit is not received two clocks after the end bit of the data block is sent out, the datapath does the following:
- Signals no CRC status error to the BIU
- Terminates further data transfer
- Signals data transfer done to the BIU
- Negative CRC - If the CRC status received after the write data block is negative (that is, not 010), a data CRC error is signaled to the BIU and further data transfer is continued.
- Data starvation due to empty FIFO - If the FIFO becomes empty during a write data transmission, or if the card clock is stopped and the FIFO remains empty for data timeout clocks, then a data-starvation error is signaled to the BIU. The datapath continues to wait for data in the FIFO.

## Data Receive

- Data timeout - During a read-data transfer, if the data start bit is not received before the number of clocks that are programmed in the timeout register, the datapath does the following.
- Signals data-timeout error to the BIU
- Terminates further data transfer
- Signals data transfer done to BIU

- Data start bit error - During a 4-bit or 8-bit read-data transfer, when an SBE occurs, the application or driver must issue CMD12 for SD/MMC cards and CMD52 for a SDIO card to exit the error condition. After a CMD done is received, the application or driver must reset IDMAC and CIU (if required) to clear the condition. The FIFO must be reset before issuing any data transfer commands in general.
- Data CRC error - During a read-data-block transfer, if the received CRC16 does not match with the internally-generated CRC16, the datapath signals a data CRC error to the BIU and continues further data transfer.
- Data end-bit error - During a read-data transfer, if the end bit of the received data is not 1, the datapath signals an end-bit error to the BIU, terminates further data transfer, and signals to the BIU that the data transfer is done.
- Data starvation due to FIFO full - During a read data transmission and when the FIFO becomes full, the card clock is stopped. If the FIFO remains full for data timeout clocks, a data starvation error is signaled to the BIU. The datapath continues to wait for the FIFO to start to empty. (The data starvation by host timeout bit is set in the MSI\_ISTAT register.)
- NOTE: In an error situation, DTO generation depends on when the error has occurred, since the descriptors are closed after the error scenarios as follows.
- If the data remains in the FIFO when the DMA detects the error scenario, then a DTO is not generated.
- If the data is completely read out before the error occurs, then a DTO is generated. CMD12 ensures DTO generation in error situations; therefore, issuing CMD12 is recommended.

## Error Handling

The MSI implements error checking. Errors are reflected in the MSI\_ISTAT register and can be communicated to the software through an interrupt, or the software can poll these bits. On power-on, interrupts are disabled, and all the interrupts are masked (bits 31:0 of the MSI\_IMSK register are all 0). The MSI module captures the following errors:

- Response and data timeout errors. For response timeout, software can retry the command. For a data timeout error, the MSI has not received the data start bit - either for the first block or the intermediate block - within the timeout period. Software can either retry the whole data transfer again or retry from a specified block onwards. By reading the contents of the MSI\_TCBCNT register later, the software can decide how many bytes remain to be copied.
- Response errors. Set when an error is received during response reception. In this case, the response that copied in the response registers is invalid. Software can retry the command.
- Data errors. Set when error in data reception is observed. For example, this error can occur in the data CRC, when the start bit is not found, when the end bit is not found, and so on. These errors could be set for any block-first block, intermediate block, or last block. On receipt of an error, the software can issue a STOP or ABORT command and retry the command for either whole data or partial data.

- Hardware locked error. Set when the MSI cannot load a command issued by software. When software sets the MSI\_CMD.STARTCMD bit, the MSI tries to load the command. If the command buffer is already filled with a command, this error is generated. The software then has to reload the command.
- FIFO underrun or overrun error. If the FIFO is full and DMA tries to write data into the FIFO, then an overrun error is set. Conversely, if the FIFO is empty and the DMA tries to read data from the FIFO, an underrun error is set.
- Data starvation by host timeout. Raised when the MSI is waiting for software intervention to transfer the data to or from the FIFO, but the software does not transfer within the stipulated timeout period. Under this condition and when a read transfer is in progress, the software must read data from the FIFO and create space for further data reception. When a transmit operation is in process, the software must fill data in the FIFO to start transferring data to the card.
- CRC Error on command. If a CRC error is detected for a command.

NOTE: During a multiple-block data transfer, if a negative CRC status is received from the device, the datapath signals a data CRC error to the BIU by setting the data MSI\_ISTAT.DCRC register. It then continues further data transmission until all the bytes are transmitted.

## Fatal Bus Errors (FBE)

An FBE occurs due to an error response from SCB. This response is a system error, so the software driver must not perform any further programming of the MSI. The only recovery mechanism from such scenarios is to do one of the following.

- Issue a hard reset.
- Do a controller reset by writing to the MSI\_CTL.CTLRST bit.

## MSI Programming Model

This section provides procedures that allow programmers to use the MSI properly. These procedures include MSI initialization, single and multiple block reads and writes and many others.

## MSI Programming Concepts

Using the features, operating modes, and event control for the MSI to their greatest potential requires an understanding of the following MSI-related concepts.

## Software and Hardware Restrictions

Before issuing a new data transfer command, the software should ensure that the card is not busy due to any previous data transfer command. Before changing the card clock frequency, the software must ensure that there are no data or command transfers in progress.

To avoid glitches in the card clock outputs, use the following steps when changing the card clock frequency.

1. Before disabling the clocks, ensure that the card is not busy due to any previous data command. To determine this status, check for 0 in the MSI\_STAT.DBUSY bit.
2. Update the MSI\_CLKEN register to disable all clocks. T o ensure completion of any previous command before this update, send a command to the CIU to update the clock registers by setting the MSI\_CMD.STARTCMD bit, the MSI\_CMD.WTPRIVDATA bits, and the MSI\_CMD.UCLKREGS bit.
3. Set the MSI\_CMD.STARTCMD bit to update the clock divider register, and send a command to the CIU in order to update the clock registers. Wait for the CIU to take the command.
4. Set the MSI\_CMD.STARTCMD to update the MSI\_CLKEN register to enable the required clocks and send a command to the CIU to update the clock registers. Wait for the CIU to take the command.

If the software issues a controller reset command by setting the MSI\_CTL.CTLRST bit, all the CIU state machines are reset and the FIFO is not cleared. The DMA sends all remaining bytes to the host. In addition to a card-reset, if a FIFO reset is also issued ( MSI\_CTL.FIFORST ), then:

- Any pending DMA transfer on the bus completes correctly
- DMA data reads are ignored
- Write data is unknown (x)

Additionally, if a DMA reset is also issued ( MSI\_CTL.DMARST ), any pending DMA transfer is abruptly terminated.

If any of the previous data commands do not properly terminate, then the software must issue the FIFO reset. The reset removes any residual data, if any, in the FIFO. After asserting the FIFO reset, the program waits until this bit is cleared. One data transfer requirement between the FIFO and host is that the number of transfers must be a multiple of the FIFO data width. For example, if the FIFO data width = 32-bit and the program must write only 15 bytes to the card, the host must program the DMA to do 16-byte transfers to the card. (The program writes to the card using the MSI\_BYTCNT register.)

## Initializing the MSI

After the power-up, the MSI is reset. The reset initializes the registers, ports, FIFO-pointers, DMA interface controls, and state-machine. After power-on reset, the software performs the following steps which are reflected in the Data Transmit State Machine figure.

Figure 26-10: Data Transmit State Machine

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000008_0feb64974fe66cb1b82e3e04cce0854ae2beeaa43659db10a157ffe27599169d.png)

1. Configure the control register ( MSI\_CTL ).
2. Set masks for interrupts by clearing appropriate bits in the interrupt mask register. Set the global MSI\_CTL.INTEN bit. It is recommended that programs write 0xFFFF\_FFFF to the MSI\_ISTAT register to clear any pending interrupts before setting the MSI\_CTL.INTEN bit.
3. Enumerate the card stack. Each card is enumerated according to card type. For details, refer to Enumerating the Card Stack. For enumeration, restrict the clock frequency to 400 kHz in accordance with SD/MMC standards.
4. Set the card frequency using the MSI\_CLKDIV register.
5. Set other parameters, which normally do not need changing with every command, with a typical value such as response and data timeout values. Set the values according to the SD/MMC specifications and the FIFO threshold value.

## Enumerating the Card Stack

The card stack does the following:

- Enumerates the connected card
- Sets the RCA for the connected card
- Reads card-specific information
- Stores card-specific information locally

Each card is enumerated separately. The identification procedure depends on whether the card connected is SD, SDIO, or MMC.

## Identifying Card Types

The MSI can have an MMC, SD, or SDIO type of card connected to it. All types of SDIO cards are supported; that is, SDIO\_IO\_ONLY, SDIO\_MEM\_ONLY, and SDIO\_COMBO cards. Each card is enumerated separately. The enumeration sequence includes the following steps:

1. Check if the card is connected.
2. Clear the bits in the MSI\_CTYPE register to configure the controller in 1-bit mode
3. Identify the card type; that is, SD, MMC, SDIO, or COMBO card.
- a. Send CMD5 with argument 0.
- b. Read the response register ( MSI\_RESP0 ), which gives the voltage window that the card supports-the response to CMD5 gives the voltages that the card supports.
- c. Send CMD5 again with the desired voltage window set in the argument. This CMD5 is used to set the voltage window and move the card state m/c out of the initialization state.
- d. Check the response [27] bit. If this bit = 1, this response indicates that the memory is present and the SDIO card is a COMBO card.
- e. If the card is SDIO go to Step 4. If the card is combinational logic (COMBO) or if the response is not received, continue with the following steps.
- f. Send CMD8 with the following argument:
- g. If the response is received, the card supports high capacity SD2.0; send ACMD41 with the following argument:
- h. If the response is received for ACMD41, then the card is SD; otherwise the card is MMC.
- i. If the response is not received for initial CMD8, then the card does not support high capacity SD2.0. Issue CMD0 followed byMD41 with the following argument:

```
Bit[31:12] = 20'h0; // Reserved bits Bit[11:8] = 4'b0001 // VHS value Bit[7:0] = 8'b10101010 // Preferred Check Pattern by SD2.0
```

```
Bit[31] = 1'b0; // Reserved bits Bit[30] = 1'b1; // High Capacity Status Bit[29:25] = 6'h0; // Reserved bits Bit[24] = 1'b0; Bit[23:0] = Supported Voltage Range
```

```
Bit[31] = 1'b0; // Reserved bits Bit[30] = 1'b0; // High Capacity Status
```

```
Bit[29:24] = 6'h0; // Reserved bits Bit[23:0] = Supported Voltage Range
```

- j. If the response is received for ACMD41, then the card is SD. Otherwise, the card is MMC.
4. Enumerate the card according to the card type.
5. Use a card clock frequency = f OD  (that is, 400 kHz) and use the following enumeration command sequence:
- SD card - Send CMD0, CMD8, ACMD41, CMD2, CMD3.
- SDIO - Send CMD5; if the function count is valid, CMD3. For the SDIO memory section, follow the same commands as for the SD card.
- MMC - Send CMD0, CMD1, CMD2, CMD3.

The card clock frequency can be configured after enumeration.

## Programming Card Clocks

The MSI supports programming the desired operational frequency for the card. The clock for the card can also be enabled or disabled. Registers that support this feature are:

- MSI\_CLKDIV : Programs clock source frequency.
- MSI\_CLKEN : Enables or disables clock for the card and enables low-power mode, which automatically stops the clock to a card when the card is idle for more than 8 clocks.

The MSI loads each of these registers only when the MSI\_CMD.STARTCMD bit and the MSI\_CMD.UCLKREGS bit in the CMD register are set. When a command is successfully loaded, the MSI clears the MSI\_CMD.STARTCMD bit. This operation happens unless the MSI already has another command in the queue, at which point it generates a Hardware Locked Error (HLE).

Software must look for the MSI\_CMD.STARTCMD and the MSI\_CMD.UCLKREGS bits, and set the MSI\_CMD.WTPRIVDATA bit to ensure that clock parameters do not change during data transfer.

NOTE: Even though MSI\_CMD.STARTCMD is set for updating clock registers, the MSI does not raise the MSI\_DONE signal upon command completion.

Use the following procedure to program the clock-related registers.

1. Confirm that the card is not engaged in any transaction; if there is a transaction, wait until it finishes.
2. Stop all clocks by writing 0 to the MSI\_CLKEN register.
3. Set the MSI\_CMD.STARTCMD , MSI\_CMD.UCLKREGS , and the MSI\_CMD.WTPRIVDATA bits.
4. Wait until the MSI\_CMD.STARTCMD bit is cleared or an HLE is set; in case of an HLE, repeat the command.
5. Program the MSI\_CLKDIV register.

6. Set the MSI\_CMD.STARTCMD , MSI\_CMD.UCLKREGS , and MSI\_CMD.WTPRIVDATA bits.
7. Wait until the MSI\_CMD.STARTCMD is cleared or an HLE is set; in case of an HLE, repeat the command.
8. Re enable all clocks by programming the MSI\_CLKEN register.
9. Set the MSI\_CMD.STARTCMD , MSI\_CMD.UCLKREGS , and the MSI\_CMD.WTPRIVDATA bits.
10. Wait until the MSI\_CMD.STARTCMD bit is cleared or a HLE is set; in case of a HLE, repeat the command.

## Sending Non-Data Commands With or Without a Response Sequence

To send any non-data command, the software must program the MSI\_CMD register and the MSI\_CMDARG register with appropriate parameters. Using these two registers, the MSI forms the command and sends it to the command bus. The MSI reflects the errors in the command response through the error bits of the MSI\_ISTAT register. The Command Register Settings for No-Data Command table shows the MSI\_CMD bit settings.

When the MSI receives a response, either erroneous or valid, it sets the MSI\_ISTAT.CMDDONE bit. A short response is copied in the MSI\_RESP0 register, while a long response is copied to all four response registers. The MSI\_RESP3 register bit 31 represents the MSB, and the MSI\_RESP0 register bit 0 represents the LSB of a long response.

Table 26-11: Command Register Settings for No-Data Command

| Parameter             | Value           | Comment                                                                                                                                                                                                                                                                      |
|-----------------------|-----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Default               | Default         | Default                                                                                                                                                                                                                                                                      |
| start_cmd             | 1               |                                                                                                                                                                                                                                                                              |
| use_hold_reg          | 1/0             | Choose value based on speed mode in use; refer to 'MSI Register Descriptions'                                                                                                                                                                                                |
| update_clk_regs_only  | 0               | No clock parameters update command                                                                                                                                                                                                                                           |
| data_expected         | 0               | No data command                                                                                                                                                                                                                                                              |
| card_number           | n CardNo        | Actual card number                                                                                                                                                                                                                                                           |
| cmd_index             | command index   |                                                                                                                                                                                                                                                                              |
| send_initialization   | 0               | Can be 1, but only for card reset commands, such as CMD0                                                                                                                                                                                                                     |
| stop_abort_cmd        | 0               | Can be 1 for commands to stop data transfer, such as CMD12                                                                                                                                                                                                                   |
| response_length       | 0               | Can be 1 for R2 (long) response                                                                                                                                                                                                                                              |
| response_expect       | 1               | Can be 0 for commands with no response; for example, CMD0, CMD4, CMD15, and so on                                                                                                                                                                                            |
| user-selectable       | user-selectable | user-selectable                                                                                                                                                                                                                                                              |
| wait_prvdata_complete | 0               | Before sending a command on a command line, the host must wait for the comple- tion of any data command in process, if any. (It is recommended to set this bit al- ways, unless the current command is to query status or stop data transfer when trans- fer is in progress) |
| check_response_crc    | 1               | If host crosschecks, CRC of response received                                                                                                                                                                                                                                |

Use the following procedure to issue basic commands or non-data commands.

1. Program the MSI\_CMD register with the appropriate command argument parameter.
2. Program the MSI\_CMD register with the settings in the Command Register Settings for No-Data Command table.
3. Wait for command acceptance by host. When software loads the command into the MSI:
- The MSI accepts the command for execution and clears the MSI\_CMD.STARTCMD bit. This process occurs unless one command is in process. Then, the MSI can load and keep the second command in the buffer.
- If the MSI is unable to load the command, then it generates an HLE (hardware locked error). (For example, a command is already in progress, a second command is in the buffer, and a third command is attempted).
4. Check if there is an HLE.
5. Wait for command execution to complete. After receiving either a response from a card or response timeout, the MSI sets the MSI\_ISTAT.CMDDONE bit. Software can either poll for this bit or respond to a generated interrupt.
6. Check if the response timeout error, response CRC error, or response error is set. This confirmation can be done either by responding to an interrupt raised by these errors or by polling bits RE, RCRC, and RTO from the MSI\_ISTAT register. If no response error is received, then the response is valid. If necessary, the software can copy the response from the response registers. Software must not modify clock parameters while a command is executing.

## Single-Block or Multiple-Block Read

Use the following procedure to perform a single-block or multiple-block read.

1. Write the data size in bytes in the MSI\_BYTCNT register.
2. Write the block size in bytes in the MSI\_BLKSIZ register.

ADDITIONAL INFORMATION: The MSI expects data from the card in blocks of size BLKSIZ each.

3. If card has a round-trip delay of more than 0.5 card clock period, program the MSI\_CDTHRCTL.RDTHR bit field. The programming ensures that the card clock does not stop in the middle of a block of data transferring from the card to the host.

ADDITIONAL INFORMATION: For programming guidelines, refer to the Card Read Threshold section. If the card read threshold feature is not enabled for such cards, then the program must ensure that the Rx FIFO does not become full during a read data transfer. The Rx FIFO must drain out at a rate faster than the rate that data is pushed into the FIFO.

4. Program the MSI\_CMDARG register with the data address of the beginning of a data read. Program the MSI\_CMD register with the parameters listed in the Command Register Settings for Single-Block or MultipleBlock Read table.

ADDITIONAL INFORMATION: For SD and MMC cards, use CMD17 for a single-block read and CMD18 for a multiple-block read. For SDIO cards, use CMD53 for both single-block and multiple-block transfers.

After writing to the MSI\_CMD register, the MSI starts executing the command. When the command is sent to the bus, the CMDONE interrupt is generated.

5. Software looks for data error interrupts on bits 7, 9, 13, and 15 of the MSI\_ISTAT register. If required, software can terminate the data transfer by sending a STOP command.
6. Software or the DMA looks for a receive FIFO data request or data starvation by host timeout conditions. In both cases, the DMA reads data from the FIFO and makes space in the FIFO for receiving more data.

When all the data is received, a DTO (Data Transfer Over) interrupt is generated.

Table 26-12: Command Register Settings for Single-Block or Multiple-Block Read

| Parameter             | Value           | Comment                                                                            |
|-----------------------|-----------------|------------------------------------------------------------------------------------|
| Default               | Default         | Default                                                                            |
| start_cmd             | 1               |                                                                                    |
| use_hold_reg          | 1/0             | Choose value based on speed mode in use; refer to 'MSI Register Descriptions'      |
| update_clk_regs_only  | 0               | No clock parameters update command                                                 |
| card_number           | n CardNo        | Actual card number                                                                 |
| send_initialization   | 0               | Can be 1, but only for card reset commands, such as CMD0                           |
| stop_abort_cmd        | 0               | Can be 1 for commands to stop data transfer, such as CMD12                         |
| send_auto_stop        | 0 or 1          | See Auto-Stop                                                                      |
| transfer_mode         | 0               | Block transfer                                                                     |
| read_write            | 0               | Read from card                                                                     |
| data_expected         | 1               | Data command                                                                       |
| response_length       | 0               | Can be 1 for R2 (long) response                                                    |
| response_expect       | 1               | Can be 0 for commands with no response; for example, CMD0, CMD4, CMD15, and so on  |
| user-selectable       | user-selectable | user-selectable                                                                    |
| cmd_index             | command index   | Can be 1 for commands to stop data transfer, such as CMD12                         |
| wait_prvdata_complete | 1               | 0 = sends command immediately. 1 = sends command after previous data transfer ends |

Table 26-12: Command Register Settings for Single-Block or Multiple-Block Read (Continued)

| Parameter          |   Value | Comment                                                         |
|--------------------|---------|-----------------------------------------------------------------|
| check_response_crc |       1 | 0 = MSI does not check response CRC 1 = MSI checks response CRC |

## Single-Block or Multiple-Block Write

Use the following procedure to perform a single-block or multiple-block write.

1. Write the data size in bytes in the MSI\_BYTCNT register.
2. Write the block size in bytes in the MSI\_BLKSIZ register.

ADDITIONAL INFORMATION: The MSI sends data in blocks of size BLKSIZ each.

3. Program MSI\_CMDARG register with the data address to which data is written.
4. Write data in the FIFO; it is best to start filling data the full depth of the FIFO.
5. Program the MSI\_CMD register with the parameters listed the Command Register Settings for Single-Block or Multiple-Block Read table.

ADDITIONAL INFORMATION: For SD and MMC cards, use CMD24 for a single-block write and CMD25 for a multiple-block write. For SDIO cards, use CMD53 for both single-block and multiple-block transfers. After writing to the MSI\_CMD register, the MSI starts executing a command; when the command is sent to the bus, a CMDONE interrupt is generated.

6. Software looks for data error interrupts in the MSI\_ISTAT.DCRC , MSI\_ISTAT.DRTO , and MSI\_ISTAT.EBE bits. If necessary, software can terminate the data transfer by sending the STOP command.
7. Software looks for a transmit FIFO data request or timeout conditions from data starvation by the host. In both cases, the software or the DMA writes data into the FIFO.
8. When a DTO interrupt is received, the data command is over. For an open-ended block transfer, if the byte count is 0, the software must send the STOP command. If the byte count is not 0, then on completion of a transfer of a given number of bytes, the MSI sends the STOP command, if necessary. The MSI\_ISTAT.ACD bit reflects the completion of the AUTO-STOP command. A response to AUTO\_STOP is stored in the MSI\_RESP1 register.
9. Wait for the busy clear interrupt.

The card can drive the busy clear interrupt on the DAT line; the host controller generates the interrupt after the busy is completed.

Table 26-13: Command Register Settings for Single-Block or Multiple-Block Write

| Parameter             | Value           | Comment                                                                           |
|-----------------------|-----------------|-----------------------------------------------------------------------------------|
| Default               | Default         | Default                                                                           |
| start_cmd             | 1               |                                                                                   |
| use_hold_reg          | 1 or 0          | Choose value based on speed mode in use; refer to 'MSI Register Descriptions'     |
| update_clk_regs_only  | 0               | No clock parameters update command                                                |
| card_number           | n CardNo        | Actual card number                                                                |
| send_initialization   | 0               | Can be 1, but only for card reset commands, such as CMD0                          |
| stop_abort_cmd        | 0               | Can be 1 for commands to stop data transfer, such as CMD12                        |
| send_auto_stop        | 0 or 1          | See Auto-Stop                                                                     |
| transfer_mode         | 0               | Block transfer                                                                    |
| read_write            | 1               | Write to card                                                                     |
| data_expected         | 1               | Data command                                                                      |
| response_length       | 0               | Can be 1 for R2 (long) response                                                   |
| response_expect       | 1               | Can be 0 for commands with no response; for example, CMD0, CMD4, CMD15, and so on |
| User-selectable       | User-selectable | User-selectable                                                                   |
| cmd_index             | command index   | Can be 1 for commands to stop data transfer, such as CMD12                        |
| wait_prvdata_complete | 1               | 0 - sends command immediately 1 - sends command after previous data transfer ends |
| check_response_crc    | 1               | 0 - MSI does not check response CRC 1 - MSI checks response CRC                   |

## Stream Reads and Writes

Stream reads and writes are like the block reads and writes described in the previous sections except for the following bits in the MSI\_CMD.XFRMODE register:

- For reads, the transfer mode ( MSI\_CMD.XFRMODE bit) =1 and the command index =CMD20.
- For writes, the transfer mode ( MSI\_CMD.XFRMODE bit) =1 and the command index =CMD11.

In a stream transfer, if the byte count is 0, then the software must send the STOP command. If the byte count is not 0, when a given number of bytes completes a transfer, the MSI sends the STOP command

## Packed Commands

In order to reduce overhead, read and write commands can be packed in groups of commands. The groups are either all read or all write. The software transfers the data for all commands in the group in one transfer on the bus.

Packed commands can be of two types:

- Packed write - CMD23 &gt; CMD25
- Packed read - CMD23 &gt; CMD25 &gt;CMD23 &gt; CMD18

The application software puts packed commands in packets. The packets are transparent to the core. For more information on packed commands, refer to the eMMC specification.

## Sending Stop or Abort in Middle of Transfer

The STOP command can terminate a data transfer between a memory card and the MSI. The ABORT command can terminate an I/O data transfer for only the SDIO\_IOONLY and SDIO\_COMBO cards.

- Send STOP command. Can be sent on the command line while a data transfer is in-progress. This command can be sent at any time during a data transfer. For information on sending this command, refer to Sending Non-Data Commands With or Without a Response Sequence. Programs can use an extra setting for this command to set the MSI\_CMD register bits (5-0) to CMD12 and set the MSI\_CMD.STPABORTCMD bit to 1. If the MSI\_CMD.STPABORTCMD bit is not set to 1, the MSI does not know that the program stopped a data transfer. Reset the MSI\_CMD.WTPRIVDATA bit to 0 in order to make the MSI send the command at once, even though there is a data transfer in progress.
- Send ABORT command. Can be used with only an SDIO\_IOONLY or SDIO\_COMBO card. To abort the function that is transferring data, program the function number in ASx bits (CCCR register of card, address 0x06, bits (0-2)) using CMD52. This command is a non-data command. For information on sending this command, refer to Sending Non-Data Commands With or Without a Response Sequence.
1. Program the MSI\_CMDARG register with the appropriate command argument parameters listed in the Parameters for CMDARG Register table.
2. Program the MSI\_CMD register using the command index as CMD52. Similar to the STOP command, set the MSI\_CMD.STPABORTCMD bit =1, to inform the MSI that the program aborted the data transfer. Reset the

Figure 26-11: Command Format for CMD52

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000009_c3e535f303bc83e1d1e3613e317fcbda65a659fd1f880f8cbd2f37f19720bddc.png)

MSI\_CMD.WTPRIVDATA bit =0 in order to make the MSI send the command at once, even though a data transfer is in progress.

3. Wait for command transfer over.
4. Check response (R5) for errors.

Table 26-14: Parameters for CMDARG Register

| MSI_CMDARG Bits   | Contents         | Value                             |
|-------------------|------------------|-----------------------------------|
| 31                | R/W flag         | 1                                 |
| 30-28             | Function number  | 0 = for CCCR access               |
| 27                | RAW flag         | 1 = if needed to read after write |
| 26                | Do-not-care      | N/A                               |
| 25-9              | Register address | 0x06                              |
| 8                 | Do-not-care      | N/A                               |
| 7-0               | Write data       | Function number to be aborted     |

## Suspend or Resume Sequence

In an SDIO card, the data transfer between an I/O function and the MSI can be temporarily halted using the SUSPEND command. This command can be necessary to perform a high-priority data transfer with another function. When desired, the data transfer can be resume using the RESUME command.

The following functions can be implemented by programming the appropriate bits in the CCCR register of the SDIO card. To read from or write to the CCCR register, use the CMD52 command.

1. SUSPEND data transfer - Non-data command.
- a. Check if the SDIO card supports the SUSPEND or RESUME protocol; this verification can be done through the SBS bit in the CCCR @0x08 register of the card.
- b. Check if the data transfer for the required function number is in process; the function number that is currently active is reflected in bits 0-3 of the CCCR register @0x0D. If the BS (bus status) bit is 1, then only the function number given by the FSx bits is valid.
- c. To suspend the transfer, set BR (bit 2) of the CCCR register @0x0C.
- d. Poll for clear status of bits BR (bit 1) and BS (bit 0) of the CCCR @0x0C. The BS bit is 1 when the currently selected function uses the data bus; the BR (bus release) bit remains 1 until the bus release is complete. When the BR and BS bits =0, the data transfer from the selected function has been suspended.
- e. During a read-data transfer, the MSI can be waiting for the data from the card. If the data transfer is a read from a card, then the MSI must be informed after the successful completion of the SUSPEND command. The MSI then resets the data state machine and comes out of the wait state. To achieve this state, set the MSI\_CTL.RDABORT bit.

- f. Wait for data completion. Get pending bytes to transfer by reading the MSI\_TCBCNT register.
- g. For a write data transfer, wait for the busy clear interrupt after the Data T ransfer Over (DTO) interrupt.
2. RESUME data transfer - This command is a data command.
- a. Check that the card is not in a transfer state, which confirms that the bus is free for data transfer.
- b. If the card is in a disconnect state, select it using CMD7. The card status can be retrieved in response to CMD52/CMD53 commands.
- c. Check that a function to be resumed is ready for data transfer. Confirm the readiness by reading the RFx flag in CCCR @0x0F. If RF =1, then the function is ready for data transfer.
- d. To resume the transfer, use CMD52 to write the function number at the FSx bits (0-3) in the CCCR register @0x0D. Form the command argument for CMD52 and write it in the MSI\_CMDARG register. The Parameters for CMDARG Register table lists the bit values.
- e. Write the block size in the MSI\_BLKSIZ register. Data transfers in units of this block size.
- f. Write the byte count in the MSI\_BYTCNT register. This amount is the total size of the data; that is, the remaining bytes for transfer. It is the responsibility of the software to handle the data.
- g. Program the MSI\_CMD register similarly to a block transfer. For details, refer to the block read and write sections.
- h. When the MSI\_CMD register is programmed, the command is sent and the function resumes data transfer. Read the DF flag (Resume Data Flag). If it is 1, then the function has data for the transfer and begins a data transfer as soon as the function or memory is resumed. If it is 0, then the function has no data for the transfer.
- i. If the DF flag is 0, the MSI waits for data for a read. After the data timeout period, it gives a data timeout error.

Table 26-15: Parameters for CMDARG Register

| CMDARG Bits   | Contents         | Value                                 |
|---------------|------------------|---------------------------------------|
| 31            | R/W flag         | 1                                     |
| 30-28         | Function number  | 0 = for CCCR access                   |
| 27            | RAW flag         | 1 = read after write                  |
| 26            | Do-not-care      | N/A                                   |
| 25-9          | Register address | 0x0D                                  |
| 8             | Do-not-care      | N/A                                   |
| 7-0           | Write data       | Function number that is to be resumed |

## Read Wait Sequence

Read\_wait is used only with SDIO cards. It can temporarily stall the data transfer either from function or memory and allow the host to send commands to any function within the SDIO device. The host can stall this transfer for as long as required. The MSI provides the facility to signal this stall transfer to the card.

1. Check if the card supports the read\_wait facility; read SRW (bit 2) of the CCCR register in SDIO card. If this bit is 1, then all functions in the card support the read\_wait facility. Use CMD52 to read this bit.
2. If the card supports the read\_wait signal, then assert it by setting the MSI\_CTL.RDWAIT bit.
3. Clear the MSI\_CTL.RDWAIT bit.

## Card Read Threshold

When an application must perform a single or multiple block read command, the application must program the MSI\_CDTHRCTL register with the appropriate card read threshold size. It also must set the card MSI\_CDTHRCTL.RDTHREN (read threshold enable) bit. This additional programming ensures that the controller sends a read command only if there is space equal to the MSI\_CDTHRCTL.RDTHR (card read threshold) available in the Rx FIFO. This programming in turn ensures that the card clock is not stopped in the middle a block of data being transmitted from the card. The card read threshold can be set to the block size of the transfer. This size guarantees that there is a minimum of one block size of space in the Rx FIFO before the controller enables the card clock.

## Recommended Usage Guidelines for Card Read Threshold

1. Program the MSI\_CDTHRCTL register before the programming the CMD register for a data read command.
2. Do not program the MSI\_CDTHRCTL register when a data transfer command is in progress.
3. Program the MSI\_CDTHRCTL.RDTHR value greater than or equal to the block size of the read transfer. This programming ensures that the card clock does not stop in between a block of data.
4. If round-trip delay &gt; 0.5 card clock, then enable the card read threshold and program the card threshold as per guideline #3. This programming guarantees that the card clock does not stop in between a block of data. The controller samples data incorrectly if the card clock stops in between a block of data if round-trip delay &gt; 0.5 card clock.
5. MSI\_CDTHRCTL.RDTHR is greater than or equal to MSI\_BLKSIZ (recommended). Program the card read threshold size ( MSI\_CDTHRCTL.RDTHR ) to at least 1 × BlockSize of the multi-block transfer. This programming guarantees that the card clock does not stop in between a block of data due to the Rx FIFO becoming full during the read transfer.
6. MSI\_CDTHRCTL.RDTHR is less than MSI\_BLKSIZ . If the MSI\_CDTHRCTL.RDTHR bit is programmed to less than the MSI\_BLKSIZ of the transfer, then the system must ensure that the receive FIFO never becomes full and overflows during the read transfer. This programming can cause the card clock from the MSI to stop. The MSI is not able to guarantee that the card clock does not stop during a read transfer.

NOTE: If the MSI\_CDTHRCTL.RDTHR , MSI\_FIFOTH.RXWM , and MSI\_FIFOTH.DMAMSZ values are programmed incorrectly, then the card clock can stop indefinitely and no interrupts are generated from the controller.

## Card Read Threshold Programming Sequence

Most cards, such as SDHC or SDXC, typically support block sizes that are specified in the card or are fixed to 512 bytes. For SDIO cards, standard capacity SD cards that support READ\_BL\_PARTIAL =1 and MMC cards, the block size is variable. The application can choose block size.

Use the following steps for the card read threshold feature. The steps guarantee that the card clock does not stop because of a FIFO full condition in the middle of a block of data being read from the card.

1. Choose the block size (configured using the MSI\_BLKSIZ register). The block size requested by the application from the card for the read transfer card must be 32-bit aligned.
2. Enable the card read threshold feature. The card read threshold can be enabled only if the block size for the given transfer is less than the total depth of the FIFO ( BlkSiz ≤ FifoDepth ) Where: BlkSiz = (block size in bytes) × 8 ÷ 32; that is, the number of the block size in terms of FIFO locations FifoDepth = total number of FIFO locations.
3. Choose the card read threshold.
- If BlkSiz ≥ ½ FifoDepth , configure MSI\_CDTHRCTL.RDTHR such that the value ≤ BlkSize in bytes
- If BlkSiz &lt; ½ FifoDepth , configure MSI\_CDTHRCTL.RDTHR such that the value = BlkSize in bytes
4. Choose the DMA multiple transaction size using the MSI\_FIFOTH.DMAMSZ bit field. The possible values for the bit fields are 1, 4, 8, 16, 32, 64, 128, and 256 transfers. Choose the value of MSI\_FIFOTH.DMAMSZ from the transfer values so that BlkSiz is a multiple of DMAMSZ. BlkSize% (DMAMSZ) = 0. Note the following special cases:
- When an MSIZE transfer =1 (configured using the DMA\_CFG.MSIZE bit field). The MSI\_FIFOTH.DMAMSZ is equal to 1 when the block size chosen in Step 1 is not a multiple of the FIFO width (in bytes). If DMAMSZ =1 is not acceptable and a higher burst size is desired-that is, a higher DMAMSZ. Return to Step 1 and recalculate the block size.
- Internal DMA (IDMAC). The size of the data buffer (in bytes) for each descriptor must be a multiple of DMAMSZ × 4.
5. Choose the RX watermark using the MSI\_FIFOTH.RXWM bit field.
10. If DMAMSZ = 1, then the RXWM = 1 or RXWM = BlkSize - 1 Additionally, for all DMA modes the RXWM must be a multiple of the chosen DMAMSZ.

## MSI Programming Model, Boot Operation

This section discusses details on programming the MSI for boot operation.

## Normal Boot Operation

Normal boot operation is applicable to MMC4.3, MMC4.4, and MMC4.41 cards. It is performed in push-pull mode. The Normal Boot Timing figure shows the timing for the transmit state machine in a normal boot mode.

Figure 26-12: Normal Boot Timing

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000010_185223f17b3ff005abe4cbc4d3d91cab696d23df2756c8b135e97ee643e3f648.png)

## Normal Boot Operation; eMMC

Normal boot operation is applicable to MMC4.3, MMC4.4, and MMC4.41 cards. It must be performed in pushpull mode. See the Normal Boot Timing figure.

Software must adhere to the following steps when working with eMMC or removable MMC 4.3 cards for boot operation. Note that the software knows that the card supports boot operation (the BOOT\_PARTITION\_ENABLE bit set) in the card.

The software also knows the BOOT\_SIZE\_MULT value in the card and the data bus width to use during boot operation: cExtend CSD register byte [177] bit[0:1].

1. Set the masks for interrupts by clearing appropriate bits in the MSI\_IMSK register.
2. Set the MSI\_CTL.INTEN bit. Before this bit is set, programs should write 0xFFFF\_FFFF to the MSI\_ISTAT and MSI\_IDSTS registers to clear any pending interrupts. For internal DMAC, unmask all the relevant fields in the MSI\_IDINTEN register.
3. Set the and MSI\_CTL.INTEN bits =1. In order to use the internal DMAC to transfer the boot data received, it must also set up the descriptors and program the MSI\_CTL.INTDMAC bit =1.
4. Set the card frequency to 400 kHz using the clock-divider register.
5. Set Data Time Out = (10 × ((TAAC × f OP ) + (100 × NSAC)); this is NAC.
6. Program the MSI\_BLKSIZ register with 0x200 (512 bytes).
7. Program the MSI\_BYTCNT register with multiples of 128K bytes, as indicated by the BOOT\_SIZE\_MULT value in the card.

8. Program the Rx FIFO threshold value in bytes in the MSI\_FIFOTH register. Typically, the threshold value can be set to half the FIFO depth.
9. Program the following fields: MSI\_CMD.STARTCMD =1'b1, MSI\_CMD.BOOTEN =1'b1, MSI\_CMD.XPECTBOOTACK (for start-acknowledge pattern ACK from the card) and MSI\_CMD.DXPECT =1'b1.
10. If MSI\_CMD.XPECTBOOTACK =1'b1, the software driver must start a timer after step 9; the terminal value is 50 ms.
- Before this timer elapses, the BAR interrupt should be received from the MSI. If this action does not occur, program the CMD register as follows:
- The MSI generates a Command Done (CD) interrupt after de-asserting the CMD line of the card. In IDMAC:

```
MSI_CMD.STARTCMD =1'b1 MSI_CMD.BOOTDIS =1'b1 All other fields = 0
```

Descriptor is closed

```
MSI_IDSTS.CES =1, indicating BAR timeout MSI_IDSTS.RI =0
```

- If the BAR interrupt is received, the software should clear this interrupt by writing a 1 to it. The software should then start another timer with a terminal value of 1 - 0.05 = 0.95 seconds. Before this timer elapses, the BDS interrupt should be received from the MSI. If this action does not occur, the software driver must program the CMD register as follows:
- The MSI generates a CD interrupt after de-asserting the CMD line of the card. In IDMAC:

```
MSI_CMD.STARTCMD =1'b1 MSI_CMD.BOOTDIS =1'b1 All other fields =0
```

Descriptor is closed

```
MSI_IDSTS.CES =1, indicating BDS timeout
```

```
MSI_IDSTS.RI =0
```

- If the BDS interrupt is received, it indicates that the boot data is being received from the card. The IDMAC engine starts transferring the data from the FIFO to the system memory as soon as the programmed RX\_WMark level is attained. At the end of a successful boot data transfer from the card, the following interrupts are generated:

Command Done (CD) with the MSI\_ISTAT.CMDDONE bit

Data Transfer Over (DTO) with the MSI\_ISTAT.DTO bit

Receive Interrupt (RI) with the MSI\_IDSTS.RI bit

- If an error occurs in Boot Ack pattern (010) or an end bit error occurs

Controller automatically aborts boot by pulling CMD line high

Controller generates CD interrupt

Controller does not generate BAR interrupt

Application aborts boot transfer

- In IDMAC:

If the software creates more descriptors than the boot data received, the MSI does not close the extra descriptors.

If the software creates less descriptors than the boot data received, the MSI generates a Descriptor Unavailable (DU) interrupt and does not transfer any further data to system memory.

- If between data block transfers NAC is violated, DRTO (Data Read Timeout) is asserted. If there are errors associated with start or end bits, SBE/EBE interrupts are also generated.
11. If MSI\_CMD.XPECTBOOTACK =1'b0, the software should start a timer after the step 9 where the terminal value is 1 second.
- Before this timer elapses, a BDS interrupt should be received from the MSI. If the interrupt is not received, the software must program their CMD register with the following fields:

```
MSI_CMD.STARTCMD =1'b1
```

```
MSI_CMD.BOOTDIS =1'b1 All other fields =0
```

ADDITIONAL INFORMATION: MSI generates a CD interrupt after de-asserting the CMD line of the card. In IDMAC mode, the descriptor is closed and the MSI\_IDSTS.CES bit =1, indicating a BDS timeout.

- If a BDS interrupt is received, it indicates that the boot data is being received from the card. At the end of a successful boot data transfer from card, the following interrupts are generated.

Command Done (CD) with the MSI\_ISTAT.CMDDONE bit

```
Data Transfer Over (DTO) with the MSI_ISTAT.DTO bit Receive Interrupt (RI) with the MSI_IDSTS.RI bit
```

## Normal Boot Operation; Removable MMC4.3, MMC4.4, and MMC4.41 Cards

Removable MMC4.3, MMC4.4, and MMC4.41 cards are different than eMMC in that the software is not aware whether these cards support the boot mode of operation when plugged in. Thus, the software must:

- Enumerate these cards as it would enumerate MMC4.0/4.1/4.2 cards for the first time
- Know the card characteristics
- Decide whether to perform a boot operation or not

Use the following procedure when booting with these card types,

1. Enumerate the card.
2. Read the EXT\_CSD register of the card and examine the following fields:
- BOOT\_PARTITION\_ENABLE
- BOOT\_SIZE\_MULT
- BOOT\_INFO
3. If necessary, the controller can manipulate the boot information in the card.
4. If the controller must perform a boot operation at the next power-up cycle, it can manipulate the EXT\_CSD register contents by using a SWITCH (CMD6) command.
5. From this point, use the same steps as in Normal Boot Operation; eMMC.

## Alternate Boot Operation; eMMC

The alternate boot operation differs from the normal boot operation in that CMD0 is used to boot the card rather than holding down the CMD-line of the card. The alternate boot operation can occur only if bit 0 in the extended CSD byte[228] (BOOT\_INFO) is set to 1.

Figure 26-13: Alternate Boot Timing

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000011_979169656a0248d1120ffe15805d2697ad284f49eef21056168aa11e446ffcb6.png)

Use the following procedure when working with eMMC or removable MMC 4.3 cards for the alternative boot operation. The software is aware that the card supports the Alternate Boot operation-the BOOT\_INFO bit is set in the card. Also, the software is aware of the BOOT\_SIZE\_MULT value in the card and the data bus width to use during the boot operation-extend CSD register byte[177] bit[0:1].

1. Set the masks for interrupts by clearing appropriate bits in the Interrupt Mask register.
2. Set the MSI\_CTL.INTEN bit =1.

ADDITIONAL INFORMATION: Set the MSI\_CTL.INTEN bit. Before this bit is set, programs should write 0xFFFF\_FFFF to the MSI\_ISTAT and MSI\_IDSTS registers to clear any pending interrupts. Software also must unmask all the relevant fields in MSI\_IDINTEN register.

3. Configure the following bits in the control register.
4. Set the card frequency to 400 kHz using the clock-divider register

```
a. MSI_CTL.INTEN = 1'b1 b. Other fields are 1'b0 c. For IDMAC, set up the descriptors. d. Program the MSI_CTL.INTDMAC bit to 1
```

ADDITIONAL INFORMATION: Wait for a time that ensures that at least 74 card clock cycles have occurred on the card interface.

5. Set Data Time Out = (10 × ((TAAC × f OP ) + (100 × NSAC)); this is NAC
6. Program the MSI\_BLKSIZ register with 0x200 (512 bytes).
7. Program the MSI\_BYTCNT register with multiples of 128K bytes, as indicated by the BOOT\_SIZE\_MULT value in the card
8. Program the Rx FIFO threshold value in bytes in the MSI\_FIFOTH register. Typically, the threshold value is set to half the FIFO depth; that is, MSI\_FIFOTH.RXWM = (FIFO\_DEPTH/2) - 1.
9. Program CMDARG = 0xFFFFFFFA.
10. Program the following fields.
- e. MSI\_CMD.INDX = 0 f. Remainder of MSI\_CMD register fields = 1'b0

```
a. MSI_CMD.STARTCMD = 1'b1 b. MSI_CMD.BOOTMODE = 1'b1 c. MSI_CMD.XPECTBOOTACK depending on whether a start-acknowledge pattern is expected from the card. d. MSI_CMD.DXPECT = 1'b1
```

ADDITIONAL INFORMATION: Wait for the Command Done (CD) interrupt.

11. If MSI\_CMD.XPECTBOOTACK = 1'b1 in step 10, the software driver must start a timer with a terminal value of 50 ms.
- a. Before this timer elapses, the BAR interrupt should be received from the MSI. If this action does not occur, the software must infer that the start-pattern has not been received and must discontinue the boot process and start with normal enumeration. In IDMAC:

- Descriptor is closed
- MSI\_IDSTS.CES =1, indicating BAR timeout
- MSI\_IDSTS.RI =0
- b. If the BAR interrupt is received, the software driver should clear this interrupt by writing a 1 to it. The software driver then starts another timer with a terminal value of 1 - 0.05 = 0.95 seconds. Before this timer elapses, the BDS interrupt should be received from the MSI. If this action does not occur, the software driver discontinues the boot process and start with normal enumeration.
- Descriptor is closed
- MSI\_IDSTS.CES =1, indicating BDS timeout
- MSI\_IDSTS.RI =0
- c. If the BDS interrupt is received, it indicates that the boot data is being received from the card. The IDMAC engine starts transferring the data from the FIFO to the system memory as soon as the programmed MSI\_FIFOTH.RXWM level is hit.
- d. It is the responsibility of the software driver to terminate the boot operation by programming the MSI to send a CMD0 by programming the registers MSI\_CMDARG =0 and the command register bits MSI\_CMD.STARTCMD =1, MSI\_CMD.INDX =0, all\_other\_fields = 0.
- e. At the end of a successful boot data transfer from the card, the following status bits are set:
- Command Done (CD) with the MSI\_ISTAT.CMDDONE bit
- Data Transfer Over (DTO) with the MSI\_ISTAT.DTO bit
- Receive Interrupt (RI) with the MSI\_IDSTS.RI bit
- f. If an error occurs in Boot Ack pattern (010) or an end bit error occurs:
- Controller does not generate BAR interrupt
- Controller detects boot data start and generates BDS interrupt
- Controller continues to receive boot data
- Application must abort boot after receiving BDS interrupt

## g. In IDMAC:

- If the software driver creates more descriptors than the boot data received, the extra descriptors are not closed by the MSI.
- If the software driver creates less descriptors than the boot data received, the MSI generates a Descriptor Unavailable (DU) interrupt and does not transfer any further data to system memory.
- h. If between data block transfers NAC is violated, DRTO (Data Read Timeout) is asserted. Apart from this, if there are errors associated with Start/End bits, SBE/EBE interrupts are also generated.

12. If MSI\_CMD.XPECTBOOTACK =1'b0 in Step 10, the software should start a timer after Step 10with a terminal value of 1 second.
- a. Before this timer elapses, the BDS interrupt should be received from the MSI. If this does not occur, the software driver should discontinue the boot process and start with normal enumeration. In IDMAC:
- Descriptor is closed
- MSI\_IDSTS.CES =1, indicating BDS timeout
- MSI\_IDSTS.RI =0
- b. If the BDS interrupt is received, it indicates that the boot data is being received from the card. The IDMAC engine starts transferring the data from the FIFO to the system memory as soon as the programmed MSI\_FIFOTH.RXWM level is hit.
- c. It is the responsibility of the software to terminate the boot operation. Software programs the MSI to send a CMD0 through the MSI\_CMDARG register =0 and command register bits MSI\_CMD.STARTCMD =1, MSI\_CMD.INDX =0, and all\_other\_fields =0.
- d. At the end of a successful boot data transfer from card, the following interrupts are generated.
- Command Done (CD) with the MSI\_ISTAT.CMDDONE bit
- Data Transfer Over (DTO) with the MSI\_ISTAT.DTO bit
- Receive Interrupt (RI) with the MSI\_IDSTS.RI bit
- e. In IDMAC
- If the software driver creates more descriptors than the boot data received, the MSI does not close the extra descriptors.
- If the software driver creates less descriptors than the boot data received, the MSI generates a Descriptor Unavailable (DU) interrupt and does not transfer any further data to system memory.

## Alternate Boot Operation; Removable MMC4.3 Card

Removable MMC4.3 cards are different than eMMC in that software is not aware whether these cards support the boot mode of operation. The software must:

- Enumerate these cards as it would enumerate MMC4.0/4.1/4.2 cards for the first time
- Know the card characteristics
- Decide whether to perform a boot operation

Use the following procedure when working with removable MMC 4.3 cards for the Alternative Boot operation.

1. Enumerate the card.
2. Read the EXT\_CSD register of the card and examine the following fields:

- a. BOOT\_PARTITION\_ENABLE
- b. BOOT\_SIZE\_MULT
- c. BOOT\_INFO
3. If necessary, the controller can also manipulate the boot information in the card.
4. If the host controller must perform a boot operation at the next power-up cycle, it can manipulate the EXT\_CSD register contents by using a SWITCH (CMD6) command.
5. From this point, use the same steps as in Alternate Boot Operation; eMMC.

## ADSP-SC58x Product Specific Information

The ADSP-SC589 MSI pins support programmable Pull-up control to prevent the pins from floating. The on-chip pulls to Data and CMD lines are programmable.

- The pullup for MSI\_DATA[3:0] and MSI\_CMD is controlled by the bit PUMSIDLC in PADS\_PCFG0
- The pullup for MSI\_DATA[7:4] is controlled by PUMSIDH bit in PADS\_PCFG0

The PADS\_PCFG0 register in the GPIO chapter has more details on bit assignments.

## ADSP-SC58x MSI Register Descriptions

Mobile Storage Interface (MSI) contains the following registers.

Table 26-16: ADSP-SC58x MSI Register List

| Name         | Description                                |
|--------------|--------------------------------------------|
| MSI_BLKSIZ   | Block Size Register                        |
| MSI_BUFADDR  | Current Buffer Descriptor Address Register |
| MSI_BUSMODE  | Bus Mode Register                          |
| MSI_BYTCNT   | Byte Count Register                        |
| MSI_CDETECT  | Card Detect Register                       |
| MSI_CDTHRCTL | Card Threshold Control Register            |
| MSI_CLKDIV   | Clock Divider Register                     |
| MSI_CLKEN    | Clock Enable Register                      |
| MSI_CMD      | Command Register                           |
| MSI_CMDARG   | Command Argument Register                  |
| MSI_CTL      | Control Register                           |
| MSI_CTYPE    | Card Type Register                         |
| MSI_DBADDR   | Descriptor List Base Address Register      |

Table 26-16: ADSP-SC58x MSI Register List (Continued)

| Name         | Description                                      |
|--------------|--------------------------------------------------|
| MSI_DEBNCE   | Debounce Count Register                          |
| MSI_DSCADDR  | Current Host Descriptor Address Register         |
| MSI_ENSHIFT  | Enable Phase Shift Register                      |
| MSI_FIFOTH   | FIFO Threshold Watermark Register                |
| MSI_IDINTEN  | Internal DMAInterrupt Enable Register            |
| MSI_IDSTS    | Internal DMAStatus Register                      |
| MSI_IMSK     | Interrupt Mask Register                          |
| MSI_ISTAT    | Raw Interrupt Status Register                    |
| MSI_MSKISTAT | Masked Interrupt Status Register                 |
| MSI_PLDMND   | Poll Demand Register                             |
| MSI_RESP0    | Response Register 0                              |
| MSI_RESP1    | Response Register 1                              |
| MSI_RESP2    | Response Register 2                              |
| MSI_RESP3    | Response Register 3                              |
| MSI_STAT     | Status Register                                  |
| MSI_TBBCNT   | Transferred Host to BIU-FIFO Byte Count Register |
| MSI_TCBCNT   | Transferred CIU Card Byte Count Register         |
| MSI_TMOUT    | Timeout Register                                 |

## Block Size Register

The MSI\_BLKSIZ register provides bits that configure data block sizes. Sizes supported are 1 to 65,535 bytes.

Figure 26-14: MSI\_BLKSIZ Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000012_bf8fe65f6dd8ceb9888767fa0fe1bac93dfbbab8a197316cba9b4c6841f95569.png)

Table 26-17: MSI\_BLKSIZ Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Block Size. The MSI_BLKSIZ.VALUE bit field configures data block size in bytes. Sizes sup- ported are 1 to 65,535 bytes. |

## Current Buffer Descriptor Address Register

The MSI\_BUFADDR register points to the data buffer address of the current descriptor read by the IDMAC.

Figure 26-15: MSI\_BUFADDR Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000013_2889ea5e77c866fd98cd59721d6c4792a4b43d6eedb4d7d7074ef85bd2c6ebd9.png)

Table 26-18: MSI\_BUFADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Host Buffer Address Pointer. The MSI_BUFADDR.VALUE bit field points to the data buffer address of the current descriptor read by the IDMAC. |
| (R/NW)             |            |                                                                                                                                             |

## Bus Mode Register

The MSI\_BUSMODE register provides bits that control the burst mode, descriptor skip length, and IDMAC. This register also provides a software reset bit.

Figure 26-16: MSI\_BUSMODE Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000014_eac69dbc832a5327ea719401ad07d75a6539b85d27e697769f390d0727588205.png)

Table 26-19: MSI\_BUSMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:8 (R/NW)        | PBL        | Programmable Burst Length. The MSI_BUSMODE.PBL bit field indicate the maximum number of beats to be per- formed in one IDMAC transaction. The IDMAC always attempts to burst as specified in PBL each time it starts a burst transfer on the host bus. This value is the mirror of the MSI_FIFOTH.DMAMSZ bits. In order to change this value, write the required value to MSI_FIFOTH.DMAMSZ bit field. The units for transfer are 32-bit. | Programmable Burst Length. The MSI_BUSMODE.PBL bit field indicate the maximum number of beats to be per- formed in one IDMAC transaction. The IDMAC always attempts to burst as specified in PBL each time it starts a burst transfer on the host bus. This value is the mirror of the MSI_FIFOTH.DMAMSZ bits. In order to change this value, write the required value to MSI_FIFOTH.DMAMSZ bit field. The units for transfer are 32-bit. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                         | 1 transfer                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                         | 4 transfers                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                         | 8 transfers                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                         | 16 transfers                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                         | 32 transfers                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                         | 64 transfers                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                         | 128 transfers                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                         | 256 transfers                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 7                  | DE         | IDMAC Enable. Setting the MSI_BUSMODE.DE bit enables the internal DMAinterface.                                                                                                                                                                                                                                                                                                                                                           | IDMAC Enable. Setting the MSI_BUSMODE.DE bit enables the internal DMAinterface.                                                                                                                                                                                                                                                                                                                                                           |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                         | Disable internalDMA                                                                                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                         | Enable internalDMA                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 26-19: MSI\_BUSMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:2 (R/W)          | DSL        | Descriptor Skip Length. The MSI_BUSMODE.DSL bit field specifies the number of words to skip between two unchained descriptors. This is applicable only for dual buffer structures.                                                                                                                                               |
| 1 (R/W)            | FB         | Fixed Burst. The MSI_BUSMODE.FB bit controls whether the peripheral bus master interface performs fixed burst transfers or not. When set, the peripheral bus uses only SINGLE, INCR4, INCR8 or INCR16 during the start of normal burst transfers. When reset, the peripheral bus uses SINGLE and INCR burst transfer operations. |
| 1 (R/W)            | FB         | 0 Use SINGLE and INCR                                                                                                                                                                                                                                                                                                            |
| 1 (R/W)            | FB         | 1 Use SINGLE, INCR4, INCR8 or INCR16                                                                                                                                                                                                                                                                                             |
| 0 (R/W)            | SWR        | Software Reset. When the MSI_BUSMODE.SWR bit is set, the DMAcontroller resets all its internal registers. The MSI_BUSMODE.SWR bit is automatically cleared after 1 clock cycle.                                                                                                                                                  |
| 0 (R/W)            | SWR        | 0 No reset                                                                                                                                                                                                                                                                                                                       |
| 0 (R/W)            | SWR        | 1 Reset internal registers                                                                                                                                                                                                                                                                                                       |

## Byte Count Register

The MSI\_BYTCNT register provides bits that configure the byte count to be transferred.

In SDIO mode, if a single transfer is greater than 4 bytes and non-DWORD-aligned, the transfer should be broken where only the last transfer is non-DWORD-aligned and less than 4 bytes. For example, if a transfer of 129 bytes must occur, then the driver should start at least two transfers; one with 128 bytes and the other with 1 byte.

Figure 26-17: MSI\_BYTCNT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000015_210dd8aa6d36528f7e717187ce736e6be416fe3405a16a11e213b69dadf32a77.png)

Table 26-20: MSI\_BYTCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Byte Count. The MSI_BYTCNT.VALUE bit field provides bits that configure the byte count to be transferred. |
| (R/W)              |            |                                                                                                           |

## Card Detect Register

The MSI\_CDETECT register configures the value on card\_detect\_n input ports (1 bit per card).

Figure 26-18: MSI\_CDETECT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000016_f6bfecd919b94b5e8e8bb4b4dacdd24794813d1740c2276cddbd7d94e5840d73.png)

Table 26-21: MSI\_CDETECT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                              |
|--------------------|------------|----------------------------------------------------------------------|
| 0                  | CD0        | Card Detect for Card 0.                                              |
| (R/NW)             |            | The MSI_CDETECT.CD0 bit sets the value on card_detect_n input ports. |

## Card Threshold Control Register

The MSI\_CDTHRCTL register sets the card read threshold size and enables card read threshold. This register also has a Busy Clear interrupt generation bit.

Figure 26-19: MSI\_CDTHRCTL Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000017_4e6c476ea9b16299398c1246232ef0238c520e809995579d5b0329c139a6aae9.png)

Table 26-22: MSI\_CDTHRCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26:16 (R/W)        | RDTHR      | Card Read Threshold Size. The MSI_CDTHRCTL.RDTHR bit configures the card read threshold size. For infor- mation on using this feature, see the "Card Read Threshold" section in the MSI chap- ter. | Card Read Threshold Size. The MSI_CDTHRCTL.RDTHR bit configures the card read threshold size. For infor- mation on using this feature, see the "Card Read Threshold" section in the MSI chap- ter. |
| 1 (R/W)            | BSYCLRIEN  | Busy Clear Interrupt Enable. The MSI_CDTHRCTL.BSYCLRIEN bit indicates the completion of a busy driven by the card after a write data transfer.                                                     | Busy Clear Interrupt Enable. The MSI_CDTHRCTL.BSYCLRIEN bit indicates the completion of a busy driven by the card after a write data transfer.                                                     |
|                    |            | 0                                                                                                                                                                                                  | Busy clear interrupt disabled                                                                                                                                                                      |
|                    |            | 1                                                                                                                                                                                                  | Busy clear interrupt enabled                                                                                                                                                                       |
| 0                  | RDTHREN    | Card Read Threshold Enable.                                                                                                                                                                        | Card Read Threshold Enable.                                                                                                                                                                        |
| (R/W)              |            | MSI_CDTHRCTL.RDTHREN 0                                                                                                                                                                             | bit enables the card read Card read threshold disabled                                                                                                                                             |
|                    |            | 1                                                                                                                                                                                                  | Card read threshold enabled                                                                                                                                                                        |

## Clock Divider Register

The MSI\_CLKDIV register provides clock divider bit fields. The bit field value is: clock division = 2 x n.

Figure 26-20: MSI\_CLKDIV Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000018_423434b40e4183708077a97b738b983c94e7d5efbc37463f74c1717839503558.png)

Table 26-23: MSI\_CLKDIV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | DIV0       | Divider 0. The MSI_CLKDIV.DIV0 bit field provides clock division. The value is 2 x n. For example, a value of 0 means divide by 2 x 0 = 0 (no division, bypass), a value of 1 means divide by 2 x 1 = 2, and a value of 0xFF means divide by 2 x 255 = 510, and so on. |

## Clock Enable Register

The MSI\_CLKEN register enables clock control. This register also provides low-power control for the card clock.

Figure 26-21: MSI\_CLKEN Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000019_f17f80eb0e3a60bccd288c562ebb6e302549cef1b19cd86da1d5534574ef0042.png)

Table 26-24: MSI\_CLKEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | LP0        | Clock Low Power Mode for Card 0. Setting the MSI_CLKEN.LP0 bit puts the card clock into low-power mode; it stops the clock when the card is in IDLE (it should be normally set to only MMCand SD memory cards. For SDIO cards, if interrupts must be detected, the clock should not be stopped). Clearing the MSI_CLKEN.LP0 bit puts the cards into non-low power mode. |
| 0 (R/W)            | EN0        | MSI Clock Enable for Card 0. The MSI_CLKEN.EN0 bit is the clock-enable control for the card clock.                                                                                                                                                                                                                                                                      |

## Command Register

The MSI\_CMD register provides bits that configure various command parameters such as boot modes, data transfer modes, and command response settings.

Figure 26-22: MSI\_CMD Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000020_d175ecb0ec3a37f6e3ba503ac38b6dc18462c66c42b9953430063575c5688467.png)

Table 26-25: MSI\_CMD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | STARTCMD   | Start Command. Once the command is taken by CIU, the MSI_CMD.STARTCMD bit is cleared. When this bit is set, the host should not attempt to write to any command registers. If a write is attempted, a hardware lock error is set in the raw interrupt register. Once the command is sent and a response is received from the SD_MMC cards, the Command Done bit is set in the raw interrupt register. |

Table 26-25: MSI\_CMD Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                        |
|--------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | USEHOLDREG   | Use Hold Reg. When the MSI_CMD.USEHOLDREG bit is set, CMDand DATA are sent to the card through the HOLD register. When the MSI_CMD.USEHOLDREG bit is cleared, CMDand DATA are sent to the card bypassing the HOLD register.                                                                                    |
| 27 (R/W)           | BOOTMODE     | Boot Mode. When the MSI_CMD.BOOTMODE bit is set, the MSI has an alternate boot operation. When the MSI_CMD.BOOTMODE bit is cleared, the MSI has a mandatory boot oper- ation.                                                                                                                                  |
| 26 (R/W)           | BOOTDIS      | Disable Boot. When the MSI_CMD.BOOTDIS bit is set with the MSI_CMD.STARTCMD bit, the CIU terminates the boot operation. Do NOT set MSI_CMD.BOOTDIS and MSI_CMD.BOOTEN together.                                                                                                                                |
| 25 (R/W)           | XPECTBOOTACK | 0 No action 1 Terminate boot                                                                                                                                                                                                                                                                                   |
|                    |              | Expect Boot Ack. When the MSI_CMD.XPECTBOOTACK bit is set with with the MSI_CMD.BOOTEN bit, the CIU expects a boot acknowledge start pattern of 0-1-0 from the selected card. 0 No ACK expected 1 ACK expected                                                                                                 |
| 24 (R/W)           | BOOTEN       | Enable Boot. The MSI_CMD.BOOTEN bit should be set only for mandatory boot mode. When software sets this bit along with the MSI_CMD.STARTCMD bit, CIU starts the boot sequence for the corresponding card by asserting the CMDline low. Do NOT set the MSI_CMD.BOOTEN bit and the MSI_CMD.BOOTDIS bit together. |

Table 26-25: MSI\_CMD Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | UCLKREGS   | Update Clock Registers Only. When the MSI_CMD.UCLKREGS bit is set, do not send commands. Update the clock register value into card clock domain. When cleared, the normal command se- quence is used. The following register values are transferred into card clock domain: CLKDIV, and CLKENA. Changes card clocks (change frequency, truncate off or on, and set low-frequency mode); provided in order to change clock frequency or stop clock without having to send command to cards. During the normal command sequence, when update_clock_registers_only = 0, the following control registers are transferred from BIU to CIU: CMD, CMDARG, TMOUT, CTYPE, BLKSIZ, BYTCNT. CIU uses new register values for new com- mand sequence to card(s). When the MSI_CMD.UCLKREGS bit is set, there are no Command Done interrupts because no command is sent to SD_MMC cards. 0 Normal command sequence | Update Clock Registers Only. When the MSI_CMD.UCLKREGS bit is set, do not send commands. Update the clock register value into card clock domain. When cleared, the normal command se- quence is used. The following register values are transferred into card clock domain: CLKDIV, and CLKENA. Changes card clocks (change frequency, truncate off or on, and set low-frequency mode); provided in order to change clock frequency or stop clock without having to send command to cards. During the normal command sequence, when update_clock_registers_only = 0, the following control registers are transferred from BIU to CIU: CMD, CMDARG, TMOUT, CTYPE, BLKSIZ, BYTCNT. CIU uses new register values for new com- mand sequence to card(s). When the MSI_CMD.UCLKREGS bit is set, there are no Command Done interrupts because no command is sent to SD_MMC cards. 0 Normal command sequence |
| 20:16 (R/W)        | CARDNUM    | Card Number. The MSI_CMD.CARDNUM bit represents the physical slot number of the card being                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Card Number. The MSI_CMD.CARDNUM bit represents the physical slot number of the card being                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 15 (R/W)           | SENDINIT   | Send Initialization. When the MSI_CMD.SENDINIT bit is set, send an initialization sequence before sending this command. When the MSI_CMD.SENDINIT bit is cleared, do not send an initialization sequence (80 clocks of 1) before sending this command. After power-on, 80 clocks must be sent to the card for initialization before sending any commands to the card. The bit should be set while sending the first command to the card, so that the controller will initialize clocks before sending a command to the card. This bit should not be set for either of the boot modes (alternate or mandatory).                                                                                                                                                                                                                                                                                        | Send Initialization. When the MSI_CMD.SENDINIT bit is set, send an initialization sequence before sending this command. When the MSI_CMD.SENDINIT bit is cleared, do not send an initialization sequence (80 clocks of 1) before sending this command. After power-on, 80 clocks must be sent to the card for initialization before sending any commands to the card. The bit should be set while sending the first command to the card, so that the controller will initialize clocks before sending a command to the card. This bit should not be set for either of the boot modes (alternate or mandatory).                                                                                                                                                                                                                                                                                        |
| 15 (R/W)           | SENDINIT   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Do not send initialization sequence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 15 (R/W)           | SENDINIT   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Send initialization sequence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |

Table 26-25: MSI\_CMD Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14                 | STPABORTCMD | Stop Abort Cmd. When the MSI_CMD.STPABORTCMD bit is set, the stop or abort command is in- tended to stop the current data transfer in progress. When an open-ended or prede-                                                                                                                                                                                                                                                                                                         | Stop Abort Cmd. When the MSI_CMD.STPABORTCMD bit is set, the stop or abort command is in- tended to stop the current data transfer in progress. When an open-ended or prede-                                                                                                                                                                                                                                                                                                         |
| 13 (R/W)           | WTPRIVDATA  | Wait Prvdata Complete. When the MSI_CMD.WTPRIVDATA bit is set, wait for the previous data transfer to complete before sending a command. When the MSI_CMD.WTPRIVDATA bit is cleared, send the command at once, even if the previous data transfer has not complet- ed. The MSI_CMD.WTPRIVDATA = 0 option is typically used to query the status of the card during data transfer or to stop the current data transfer; the card number should be the same as in the previous command. | Wait Prvdata Complete. When the MSI_CMD.WTPRIVDATA bit is set, wait for the previous data transfer to complete before sending a command. When the MSI_CMD.WTPRIVDATA bit is cleared, send the command at once, even if the previous data transfer has not complet- ed. The MSI_CMD.WTPRIVDATA = 0 option is typically used to query the status of the card during data transfer or to stop the current data transfer; the card number should be the same as in the previous command. |
| 12                 | SENDASTOP   | Send Auto Stop. When the MSI_CMD.SENDASTOP bit is set, MSI sends the stop command to SD_MMC cards at the end of the data transfer. Refer to the Auto Stop section in the chapter to determine:                                                                                                                                                                                                                                                                                       | Send Auto Stop. When the MSI_CMD.SENDASTOP bit is set, MSI sends the stop command to SD_MMC cards at the end of the data transfer. Refer to the Auto Stop section in the chapter to determine:                                                                                                                                                                                                                                                                                       |
|                    |             | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Send command at once                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|                    |             | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Wait for previous data transfer completion                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| (R/W)              |             | • When the MSI_CMD.SENDASTOP bit should be set, since some data transfers do not need explicit stop commands • When software should explicitly send a stop command during open-ended trans- fers Additionally, when resuming suspended memory access of SD-Combo card, the MSI_CMD.SENDASTOP bit should be set correctly to send a stop command. The bit does not need to be set if no data is expected from the card.                                                               | • When the MSI_CMD.SENDASTOP bit should be set, since some data transfers do not need explicit stop commands • When software should explicitly send a stop command during open-ended trans- fers Additionally, when resuming suspended memory access of SD-Combo card, the MSI_CMD.SENDASTOP bit should be set correctly to send a stop command. The bit does not need to be set if no data is expected from the card.                                                               |
|                    |             | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | No stop command sent at end of data transfer                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|                    |             | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Send stop command at end of data transfer                                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 26-25: MSI\_CMD Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | XFRMODE    | Transfer Mode. When the MSI_CMD.XFRMODE bit is set, stream data transfer command. If no data expected, do-not-care.                                                                                                                                                                              |
| 11 (R/W)           | XFRMODE    | 0 Block data transfer command                                                                                                                                                                                                                                                                    |
| 11 (R/W)           | XFRMODE    | 1 Stream data transfer command                                                                                                                                                                                                                                                                   |
| 10 (R/W)           | RDWR       | Read from or Write to Card. When the MSI_CMD.RDWR bit is set, write to the card. When the MSI_CMD.RDWR bit is cleared, read from the card.                                                                                                                                                       |
| 10 (R/W)           | RDWR       | 0 Read from card                                                                                                                                                                                                                                                                                 |
| 10 (R/W)           | RDWR       | 1 Write to card                                                                                                                                                                                                                                                                                  |
| 9 (R/W)            | DXPECT     | Data Expected. When the MSI_CMD.DXPECT bit is set, data transfer is expected. When the MSI_CMD.DXPECT bit is cleared, no data transfer is expected.                                                                                                                                              |
| 9 (R/W)            | DXPECT     | 0 No data transfer expected                                                                                                                                                                                                                                                                      |
| 9 (R/W)            | DXPECT     | 1 Data transfer expected                                                                                                                                                                                                                                                                         |
| 8 (R/W)            | CHKRESPCRC | Check Response CRC. When the MSI_CMD.CHKRESPCRC bit is set, check the response CRC. When cleared, do not check the response. Some of the command responses do not return valid CRC bits. Software should disa- ble CRC checks for those commands in order to disable CRC checking by controller. |
| 8 (R/W)            | CHKRESPCRC | 0 Do not check response                                                                                                                                                                                                                                                                          |
| 8 (R/W)            | CHKRESPCRC | 1 Check response                                                                                                                                                                                                                                                                                 |
| 7 (R/W)            | RLEN       | Response Length. When the MSI_CMD.RLEN bit is set, a long response is expected from the card. When cleared, a short response is expected. expected                                                                                                                                               |
| 7 (R/W)            | RLEN       | 0 Short response                                                                                                                                                                                                                                                                                 |
| 7 (R/W)            | RLEN       | 1 Long response expected                                                                                                                                                                                                                                                                         |
| 6 (R/W)            | RXPECT     | Response Expect. When the MSI_CMD.RXPECT bit is set, a response is expected from the card. When cleared, no response is expected.                                                                                                                                                                |
| 6 (R/W)            | RXPECT     | 0 No response expected                                                                                                                                                                                                                                                                           |
| 6 (R/W)            | RXPECT     | 1 Response expected                                                                                                                                                                                                                                                                              |

Table 26-25: MSI\_CMD Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 5:0                | INDX       | Command Index.            |
| (R/W)              |            |                           |

## Command Argument Register

The MSI\_CMDARG register provides bits that specify the command argument to be passed to the card.

Figure 26-23: MSI\_CMDARG Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000021_9177b00b00e866c367eae7d913d24941e9afdfdee8c914ef0d5a9a05af96e0f9.png)

Table 26-26: MSI\_CMDARG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Command Argument. The MSI_CMDARG.VALUE bit field specifies the command argument to be passed to the card. |

## Control Register

The MSI\_CTL register controls the various settings used by the MSI module.

Figure 26-24: MSI\_CTL Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000022_4fb448e0251e32591d4a056d41d3ee5219bcdab344f1b8e3ade5df001cc2c132.png)

Table 26-27: MSI\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | INTDMAC    | Use Internal DMAController. The MSI_CTL.INTDMAC bit is present only for the internal DMAC configuration and determines whether the host or DMAis used for data transfers. 0 Host performs transfers                                                                                                                                                                                                                                                                        |
| 8 (R/W)            | RDABORT    | Abort Read Data. Setting the MSI_CTL.RDABORT bit after the suspend command is issued during a read-transfer operation, software polls the card to find out when the suspend hap- pened. Once the suspend occurs, software sets the bit to reset data state-machine, which is waiting for next block of data. The bit automatically clears once the data state machine resets to idle. It is used in the SDIO card suspend sequence. 0 No change 1 Reset data state machine |

Table 26-27: MSI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | IRQRESP    | Send IRQ Response. Setting the MSI_CTL.IRQRESP bit sends an auto IRQ response. It is cleared once the response is sent. To wait for MMCcard interrupts, the host issues CMD40, and the MSI waits for an interrupt response from the MMCcard(s). In the meantime, if the host wants the MSI to exit waiting for interrupt state, it can set the MSI_CTL.IRQRESP bit, at which time the MSI command state-machine sends the CMD40 response on the bus and re- turns to the idle state. |
| 6 (R/W)            | RDWAIT     | Read Wait. Setting the MSI_CTL.RDWAIT bit allows sending read-wait to SDIO cards.                                                                                                                                                                                                                                                                                                                                                                                                    |
| 5 (R/W)            | DMAEN      | DMAEnable. Setting the MSI_CTL.DMAEN bit enables DMAtransfer mode. 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                         |
| 4 (R/W)            | INTEN      | Global Interrupt Enable. Setting the MSI_CTL.INTEN bit enables global interrupts. The int port is 1 only when this bit is 1 and one or more unmasked interrupts are set. 0 Disable 1 Enable                                                                                                                                                                                                                                                                                          |
| 2 (R/W)            | DMARST     | DMAReset. Setting the MSI_CTL.DMARST bit resets internal DMAinterface control logic. To reset the DMAinterface, firmware should set this bit to 1. This bit is auto-cleared after two peripheral clocks.                                                                                                                                                                                                                                                                             |

Table 26-27: MSI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | FIFORST    | FIFO Reset. Setting the MSI_CTL.FIFORST bit provides a reset to data FIFO to reset FIFO pointers. To reset FIFO, firmware should set bit to 1. This bit is auto-cleared after completion of reset operation.                                                                                                                                                                                                                                 | FIFO Reset. Setting the MSI_CTL.FIFORST bit provides a reset to data FIFO to reset FIFO pointers. To reset FIFO, firmware should set bit to 1. This bit is auto-cleared after completion of reset operation.                                                                                                                                                                                                                                 |
| 1 (R/W)            | FIFORST    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                            | No change                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 1 (R/W)            | FIFORST    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                            | Reset                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 0 (R/W)            | CTLRST     | Controller Reset. Setting the MSI_CTL.CTLRST bit resets the MSI. To reset the controller, firmware should set the bit to 1. This bit is auto-cleared after two peripheral bus and two cclk_in clock cycles. This resets the: • BIU/CIU interface • CIU and state machines • MSI_CTL.RDABORT , MSI_CTL.IRQRESP , and MSI_CTL.RDWAIT bits • MSI_CMD.STARTCMD bit It does not affect any registers or DMAinterface, or FIFO or host interrupts. | Controller Reset. Setting the MSI_CTL.CTLRST bit resets the MSI. To reset the controller, firmware should set the bit to 1. This bit is auto-cleared after two peripheral bus and two cclk_in clock cycles. This resets the: • BIU/CIU interface • CIU and state machines • MSI_CTL.RDABORT , MSI_CTL.IRQRESP , and MSI_CTL.RDWAIT bits • MSI_CMD.STARTCMD bit It does not affect any registers or DMAinterface, or FIFO or host interrupts. |
| 0 (R/W)            | CTLRST     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                            | No change                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 0 (R/W)            | CTLRST     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                            | Reset                                                                                                                                                                                                                                                                                                                                                                                                                                        |

## Card Type Register

The MSI\_CTYPE register provides bits that configure card widths.

Figure 26-25: MSI\_CTYPE Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000023_0bd9263f558d806f797e0d88b10f5039c69603bbfa15df16f029f4a85e53f060.png)

Table 26-28: MSI\_CTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                   |
|--------------------|------------|-----------------------------------------------------------|
| 16                 | WIDBYTE0   | Width Byte 0.                                             |
| (R/W)              |            | The MSI_CTYPE.WIDBYTE0 bit enables 8-bit mode for card 0. |
| 0                  | WIDNIB0    | Width Nibble 0.                                           |
| (R/W)              |            | The MSI_CTYPE.WIDNIB0 bit enables 4-bit mode for card 0.  |

## Descriptor List Base Address Register

The MSI\_DBADDR register contains the base address of the first descriptor. The LSB bits [1:0] bits are ignored and taken as all-zero by the IDMAC internally and these LSB bits are read-only.

Figure 26-26: MSI\_DBADDR Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000024_6dc41f1038a114602e7ba12e5589a37dfa1d4e924f22f5460f2b2722d7b852bd.png)

Table 26-29: MSI\_DBADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------|
| 31:0               | VALUE      | Descriptor Base Address.                                                          |
| (R/W)              |            | The MSI_DBADDR.VALUE bit field contains the base address of the first descriptor. |

## Debounce Count Register

The MSI\_DEBNCE register provides the number of host clocks (clk) used by debounce filter logic; typical debounce time is 5-25 ms.

Figure 26-27: MSI\_DEBNCE Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000025_c7e81bc11c8faa5de59fef394c76fedc6e0126ebda5f5067e8c00f7d36109c55.png)

Table 26-30: MSI\_DEBNCE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------|
| 23:0 (R/W)         | VALUE      | Debounce Count. The MSI_DEBNCE.VALUE bit field provides the number of host clocks (clk) used by debounce filter logic. |

## Current Host Descriptor Address Register

The MSI\_DSCADDR register points to the start address of the current descriptor read by the IDMAC.

Figure 26-28: MSI\_DSCADDR Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000026_31acfeeda36405bda57997b4c8371194c18e830c2b7d38acad74e36e9201fb64.png)

Table 26-31: MSI\_DSCADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Host Descriptor Address Pointer. The MSI_DSCADDR.VALUE bit field points to the start address of the current de- scriptor read by the IDMAC. |
| (R/NW)             |            |                                                                                                                                             |

## Enable Phase Shift Register

The MSI\_ENSHIFT register provides control for the amount of phase shift provided on the default enables in the design. Two bits are assigned for each card/slot.

Figure 26-29: MSI\_ENSHIFT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000027_8fec32f3ff9522ad70c840f809c39a6094c04f09e27c247d2850c723f23fe7f8.png)

Table 26-32: MSI\_ENSHIFT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1:0                | CD0        | Enable Shift for Card 0. The MSI_ENSHIFT.CD0 bit field provides control for the amount of phase shift provided on the default enables in the design. |
| (R/W)              |            |                                                                                                                                                      |

## FIFO Threshold Watermark Register

The MSI\_FIFOTH register provides bits that manage the FIFO transactions.

Figure 26-30: MSI\_FIFOTH Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000028_036dfcea61115c7b6339186fe8fc10a65ddad593058ddd86307c222022f55880.png)

Table 26-33: MSI\_FIFOTH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:28 (R/W)        | DMAMSZ     | DMAMultiple Transaction Size. The MSI_FIFOTH.DMAMSZ bit field configures the burst size of a multiple transac- tion. The units for transfer are 32-bit.                                                            |
| 27:16 (R/W)        | RXWM       | RX Watermark. The MSI_FIFOTH.RXWM bit field sets the FIFO threshold watermark level when re- ceiving data to card. When the FIFO data count is greater than this number, a DMA/ FIFO request is raised.            |
| 11:0 (R/W)         | TXWM       | TX Watermark. The MSI_FIFOTH.TXWM bit field sets the FIFO threshold watermark level when transmitting data to card. When the FIFO data count is less than or equal to this num- ber, a DMA/FIFO request is raised. |

## Internal DMA Interrupt Enable Register

The MSI\_IDINTEN register provides bits for setting various interrupts in DMA mode.

Figure 26-31: MSI\_IDINTEN Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000029_730e156d5daa1ebf886399ae7e2e758f1eb9662df9e2ec7945b6ccf8553184ec.png)

Table 26-34: MSI\_IDINTEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | AI         | Abnormal Interrupt Summary. Setting the MSI_IDINTEN.AI bit enables the abnormal interrupt summary. See the bit descriptions in the MSI_IDSTS register for interrupt descriptions.                 |
| 8 (R/W)            | NI         | Normal Interrupt Summary. Setting the MSI_IDINTEN.NI bit enables the normal interrupt summary. See the bit descriptions in the MSI_IDSTS register for interrupt descriptions.                     |
| 5 (R/W)            | CES        | Card Error Summary. Setting the MSI_IDINTEN.CES bit enables the card error summary interrupt. See the bit descriptions in the MSI_IDSTS register for interrupt descriptions. 0 Interrupt disabled |
| 5 (R/W)            |            |                                                                                                                                                                                                   |

Table 26-34: MSI\_IDINTEN Register Fields (Continued)

| Bit No. (Access)   | Description/Enumeration                                                                                                                                                                  |
|--------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | Descriptor Unavailable. Setting the MSI_IDINTEN.DU bit enables the descriptor unavailable interrupt. See the bit descriptions in the MSI_IDSTS register for interrupt descriptions.      |
| 2 (R/W)            | Fatal Bus Error. Setting the MSI_IDINTEN.FBE bit enables the FBE interrupt. See the bit descrip- tions in the MSI_IDSTS register for interrupt descriptions.                             |
| 1 (R/W)            | Receive Interrupt. Setting the MSI_IDINTEN.RI bit enables the receive interrupt. See the bit descrip- tions in the MSI_IDSTS register for interrupt descriptions. 0 Interrupt disabled   |
| 0 (R/W)            | Transmit Interrupt. Setting the MSI_IDINTEN.TI bit enables the transmit interrupt. See the bit de- scriptions in the MSI_IDSTS register for interrupt descriptions. 0 Interrupt disabled |

## Internal DMA Status Register

The MSI\_IDSTS register provides DMA status information. This register is updated only when the DMA is active.

Figure 26-32: MSI\_IDSTS Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000030_6b1ac5e355d431e1b15ce9fa2dce33b586b9d73e724ff41b0e42b7bc379c8171.png)

Table 26-35: MSI\_IDSTS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:13 (R/NW)       | FSM        | DMAC Finite State Machine Present State. The MSI_IDSTS.FSM bit field indicates the present state of the finite state machine IDMAC (DMA controller). | DMAC Finite State Machine Present State. The MSI_IDSTS.FSM bit field indicates the present state of the finite state machine IDMAC (DMA controller). |
|                    |            | 0                                                                                                                                                    | DMAIdle                                                                                                                                              |
|                    |            | 1                                                                                                                                                    | DMAsuspend                                                                                                                                           |
|                    |            | 2                                                                                                                                                    | DESC_RD                                                                                                                                              |
|                    |            | 3                                                                                                                                                    | DESC_CHK                                                                                                                                             |
|                    |            | 4                                                                                                                                                    | DMA_RD_REQ_WAIT                                                                                                                                      |
|                    |            | 5                                                                                                                                                    | DMA_WR_REQ_WAIT                                                                                                                                      |
|                    |            | 6                                                                                                                                                    | DMA_RD                                                                                                                                               |
|                    |            | 7                                                                                                                                                    | DMA_WR                                                                                                                                               |
|                    |            | 8                                                                                                                                                    | DESC_CLOSE                                                                                                                                           |

Table 26-35: MSI\_IDSTS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:10 (R/NW)       | EB         | Error Bits. The MSI_IDSTS.EB bit field indicates the type of error that caused a bus error. It is valid only with MSI_IDSTS.FBE set. This field does not generate an interrupt. The MSI_IDSTS.EB bit field is read-only.                                                                                                                   | Error Bits. The MSI_IDSTS.EB bit field indicates the type of error that caused a bus error. It is valid only with MSI_IDSTS.FBE set. This field does not generate an interrupt. The MSI_IDSTS.EB bit field is read-only.                                                                                                                   |
| 12:10 (R/NW)       | EB         | 0                                                                                                                                                                                                                                                                                                                                          | 3'b001 - Host abort received during transmission                                                                                                                                                                                                                                                                                           |
| 12:10 (R/NW)       | EB         | 1                                                                                                                                                                                                                                                                                                                                          | 3'b010 - Host abort received during reception                                                                                                                                                                                                                                                                                              |
| 9 (R/W)            | AIS        | Abnormal Interrupt Summary. The MSI_IDSTS.AIS bit field is a logical OR of the following: IDSTS[2] - Fatal Bus Interrupt IDSTS[4] - DUbit Interrupt Only unmasked bits affect this bit. This is a sticky bit and must be cleared each time a corresponding bit that causes MSI_IDSTS.AIS to be set is cleared. Writing a 1                 | Abnormal Interrupt Summary. The MSI_IDSTS.AIS bit field is a logical OR of the following: IDSTS[2] - Fatal Bus Interrupt IDSTS[4] - DUbit Interrupt Only unmasked bits affect this bit. This is a sticky bit and must be cleared each time a corresponding bit that causes MSI_IDSTS.AIS to be set is cleared. Writing a 1                 |
| 8 (R/W)            | NIS        | Normal Interrupt Summary. The MSI_IDSTS.NIS bit field is a logical OR of the following: IDSTS[0] - Transmit Interrupt IDSTS[1] - Receive Interrupt Only unmasked bits affect this bit. This is a sticky bit and must be cleared each time a corresponding bit that causes MSI_IDSTS.NIS to be set is cleared. Writing a 1 clears this bit. | Normal Interrupt Summary. The MSI_IDSTS.NIS bit field is a logical OR of the following: IDSTS[0] - Transmit Interrupt IDSTS[1] - Receive Interrupt Only unmasked bits affect this bit. This is a sticky bit and must be cleared each time a corresponding bit that causes MSI_IDSTS.NIS to be set is cleared. Writing a 1 clears this bit. |
| 5 (R/W)            | CES        | Card Error Summary. The MSI_IDSTS.CES bit indicates the logical OR of the following bits: • EBE - End Bit Error • RTO - Response Timeout/Boot Ack Timeout • RCRC - Response CRC • SBE - Start Bit Error • DRTO - Data Read Timeout/BDS timeout • DCRC - Data CRC for Receive • RE - Response Error Writing a 1 clears this bit.            | Card Error Summary. The MSI_IDSTS.CES bit indicates the logical OR of the following bits: • EBE - End Bit Error • RTO - Response Timeout/Boot Ack Timeout • RCRC - Response CRC • SBE - Start Bit Error • DRTO - Data Read Timeout/BDS timeout • DCRC - Data CRC for Receive • RE - Response Error Writing a 1 clears this bit.            |
| 5 (R/W)            | CES        | 0                                                                                                                                                                                                                                                                                                                                          | No error occurred                                                                                                                                                                                                                                                                                                                          |
| 5 (R/W)            | CES        | 1                                                                                                                                                                                                                                                                                                                                          | Error occurred                                                                                                                                                                                                                                                                                                                             |

Table 26-35: MSI\_IDSTS Register Fields (Continued)

| Bit No. (Access)   | Bit Name                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DU                                                                                                                                              | Descriptor Unavailable Interrupt. The MSI_IDSTS.DU bit indicates a descriptor unavailable error. If software creates less descriptors than the boot data received, the MSI generates this interrupt and does not transfer any further data to system memory.                                                                                                                                                           |
| 2 (R/W)            | FBE                                                                                                                                             | Fatal Bus Error Interrupt. The MSI_IDSTS.FBE bit indicates a fatal bus error error. An FBE occurs due to an error response from the SCB. Because this is a system error, the software driver should not perform any further programming of the MSI. The only recovery mechanism from such an error is to issue a hard reset or to perform a controller reset by writing to the MSI_CTL.CTLRST bit. 0 No error occurred |
| 1 (R/W)            | RI                                                                                                                                              | 1 Error occurred Receive Interrupt. The MSI_IDSTS.RI bit indicates that data reception is finished for a descriptor.                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | 1 Event occurred TI Transmit Interrupt. The MSI_IDSTS.TI bit indicates that data transmission is finished for a descriptor. 0 No event occurred | 0 No event occurred                                                                                                                                                                                                                                                                                                                                                                                                    |

## Interrupt Mask Register

The MSI\_IMSK register provides bits that allow the masking of unwanted interrupts.

Figure 26-33: MSI\_IMSK Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000031_be634368c75cda5759b63a106fb9af10f1b771c005e22cdf177c6c67619bcb89.png)

Table 26-36: MSI\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | SDIOMSK0   | SDIO Interrupt Mask for SDIO Device 0. The MSI_IMSK.SDIOMSK0 bits mask SDIO interrupts, one bit for one card. When masked, SDIO interrupt detection for that card is disabled. A 0 masks an inter- rupt, and 1 enables an interrupt. |
| 15 (R/W)           | EBE        | End-bit Error (Read)/Write no CRC. The MSI_IMSK.EBE bit masks the end bit read/write no CRC error.                                                                                                                                   |
| 15 (R/W)           | EBE        | 0 Masked                                                                                                                                                                                                                             |
| 15 (R/W)           | EBE        | 1 Enabled                                                                                                                                                                                                                            |

Table 26-36: MSI\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | ACD        | Auto Command Done. The MSI_IMSK.ACD bit masks the auto command done error.                                                    |
| 13 (R/W)           | SBEBCI     | Start Bit Error(SBE)/Busy Complete Interrupt (BCI). The MSI_IMSK.SBEBCI bit masks the start bit/busy complete error. 0 Masked |
| 12 (R/W)           | HLE        | Hardware Locked Write Error. The MSI_IMSK.HLE bit masks the hardware locked write error. 0 Masked                             |
| 11 (R/W)           | FRUN       | FIFO Underrun/Overrun Error. The MSI_IMSK.FRUN bit masks the FIFO overrun or underrun                                         |
| 10 (R/W)           |            | error. 0 Masked 1 Enabled                                                                                                     |
|                    | HTO        | Data Starvation by Host Timeout. The MSI_IMSK.HTO bit masks the host timeout error. 0 Masked 1 Enabled                        |
| 9 (R/W)            | DRTO       | Data Read Timeout. The MSI_IMSK.DRTO bit masks the data read timeout error. 0 Masked 1 Enabled                                |
| 8 (R/W)            | RTO        | Response Timeout. The MSI_IMSK.RTO bit masks the response timeout error. 0 Masked                                             |

Table 26-36: MSI\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | DCRC       | Data CRC Error. The MSI_IMSK.DCRC bit masks the data CRC error where the received data CRC does not match with locally-generated CRC in CIU. The error can also occur if the write CRC status is incorrectly sampled by the host.                                                                                         |
| 7 (R/W)            | DCRC       | 0 Masked                                                                                                                                                                                                                                                                                                                  |
| 7 (R/W)            | DCRC       | 1 Enabled                                                                                                                                                                                                                                                                                                                 |
| 6 (R/W)            | RCRC       | Response CRC Error. The MSI_IMSK.RCRC bit masks the response CRC error which is set when the re- sponse CRC does not match with the locally-generated CRC in the CIU.                                                                                                                                                     |
| 6 (R/W)            | RCRC       | 0 Masked                                                                                                                                                                                                                                                                                                                  |
| 6 (R/W)            | RCRC       | 1 Enabled                                                                                                                                                                                                                                                                                                                 |
| 5 (R/W)            | RXDR       | Receive FIFO Data Request. The MSI_IMSK.RXDR bit masks the receive FIFO data request interrupt which is set during write operation to card when FIFO level reaches less than or equal to transmit- threshold level.                                                                                                       |
| 5 (R/W)            | RXDR       | 0 Masked                                                                                                                                                                                                                                                                                                                  |
| 5 (R/W)            | RXDR       | 1 Enabled                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | TXDR       | Transmit FIFO Data Request. The MSI_IMSK.TXDR bit masks the transmit FIFO data request interrupt which is set during write operation to card when FIFO level reaches less than or equal to trans- mit-threshold level.                                                                                                    |
| 4 (R/W)            | TXDR       | 0 Masked                                                                                                                                                                                                                                                                                                                  |
| 4 (R/W)            | TXDR       | 1 Enabled                                                                                                                                                                                                                                                                                                                 |
| 3 (R/W)            | DTO        | Data Transfer Over. The MSI_IMSK.DTO bit masks the data transfer over interrupt which indicates the data transfer completed. Though on detection of errors such as the start bit error, the data CRC error, and so on, DTO may or may not be set; the application must issue CMD12, which ensures that DTO is set. Masked |
| 3 (R/W)            | DTO        | 0                                                                                                                                                                                                                                                                                                                         |
| 3 (R/W)            | DTO        | 1 Enabled                                                                                                                                                                                                                                                                                                                 |

Table 26-36: MSI\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | CMDDONE    | Command Done. The MSI_IMSK.CMDDONE bit masks the command done interrupt where a com- mand is sent to a card and received a response from the card, even if a response error or a CRC error occurs.                                                                      | Command Done. The MSI_IMSK.CMDDONE bit masks the command done interrupt where a com- mand is sent to a card and received a response from the card, even if a response error or a CRC error occurs.                                                                      |
| 2 (R/W)            | CMDDONE    | 0                                                                                                                                                                                                                                                                       | Masked                                                                                                                                                                                                                                                                  |
| 2 (R/W)            | CMDDONE    | 1                                                                                                                                                                                                                                                                       | Enabled                                                                                                                                                                                                                                                                 |
| 1 (R/W)            | RE         | Response Error. The MSI_IMSK.RE masks a response error. This is an error in received response set if one of following occurs: transmission bit != 0, command index mismatch, End-bit != 1.                                                                              | Response Error. The MSI_IMSK.RE masks a response error. This is an error in received response set if one of following occurs: transmission bit != 0, command index mismatch, End-bit != 1.                                                                              |
| 1 (R/W)            | RE         | 0                                                                                                                                                                                                                                                                       | Masked                                                                                                                                                                                                                                                                  |
| 1 (R/W)            | RE         | 1                                                                                                                                                                                                                                                                       | Enabled                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | CD         | Card Detect. The MSI_IMSK.CD bit masks the card detect interrupt. On power-on, the controller should read in the card_detect port and store the value in the memory. Upon receiving a card-detect interrupt, it should again read the card_detect port and XOR with the | Card Detect. The MSI_IMSK.CD bit masks the card detect interrupt. On power-on, the controller should read in the card_detect port and store the value in the memory. Upon receiving a card-detect interrupt, it should again read the card_detect port and XOR with the |
| 0 (R/W)            | CD         | 0                                                                                                                                                                                                                                                                       | Masked                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | CD         | 1                                                                                                                                                                                                                                                                       | Enabled                                                                                                                                                                                                                                                                 |

## Raw Interrupt Status Register

The MSI\_ISTAT register provides bits that clear interrupts. Conditions 6 through 9 indicate that the received data may have errors. If there was a response timeout, then no data transfer occurred.

Figure 26-34: MSI\_ISTAT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000032_3516d4a26083cc1467b03b201b120e276811813d2c45167a72e38214599ad4a4.png)

Table 26-37: MSI\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W1C)         | SDIOINT0   | SDIO Interrupt for Device 0. The MSI_ISTAT.SDIOINT0 bit indicates that an interrupt occurred on an SDIO card.                                                                               |
| 15 (R/W1C)         | EBE        | End-bit Error. The MSI_ISTAT.EBE bit indicates that the start bit of the CRC status was not re- ceived by two clocks after the end of the data block. A CRC error is indicated by the card. |
| 15 (R/W1C)         | EBE        | 0 No error                                                                                                                                                                                  |
| 15 (R/W1C)         | EBE        | 1 Error occurred                                                                                                                                                                            |

Table 26-37: MSI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                |                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14                 | ACD        | Auto Command Done.                                                                                                                                                                                                                                                     | Auto Command Done.                                                                                                                                                                                                                                                     |
| (R/W1C)            |            | 0                                                                                                                                                                                                                                                                      | No error                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                      | Error occurred                                                                                                                                                                                                                                                         |
| 13 (R/W1C)         | SBEBCI     | Start Bit Error Busy Complete Interrupt. The MSI_ISTAT.SBEBCI bit indicates the completion of a busy signal driven by the card after a write data transfer.                                                                                                            | Start Bit Error Busy Complete Interrupt. The MSI_ISTAT.SBEBCI bit indicates the completion of a busy signal driven by the card after a write data transfer.                                                                                                            |
|                    |            | 0                                                                                                                                                                                                                                                                      | No error                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                      | Error occurred                                                                                                                                                                                                                                                         |
| 12                 | HLE        | Hardware Locked Write Error.                                                                                                                                                                                                                                           | Hardware Locked Write Error.                                                                                                                                                                                                                                           |
| (R/W1C)            |            | 0                                                                                                                                                                                                                                                                      | No error                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                      | Error occurred                                                                                                                                                                                                                                                         |
| 11                 | FRUN       | FIFO Underrun/Overrun Error.                                                                                                                                                                                                                                           | FIFO Underrun/Overrun Error.                                                                                                                                                                                                                                           |
| (R/W1C)            |            | 0                                                                                                                                                                                                                                                                      | No error                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                      | Error occurred                                                                                                                                                                                                                                                         |
| 10 (R/W1C)         | HTO        | Host Timeout. The MSI_ISTAT.HTO bit indicates that the FIFO is empty during transmission or is full during reception. Unless software/DMA writes data for an empty condition or reads data for a full condition, the MSI cannot continue with data transfer. The clock | Host Timeout. The MSI_ISTAT.HTO bit indicates that the FIFO is empty during transmission or is full during reception. Unless software/DMA writes data for an empty condition or reads data for a full condition, the MSI cannot continue with data transfer. The clock |
|                    |            | 0                                                                                                                                                                                                                                                                      | No error                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                      | Error occurred                                                                                                                                                                                                                                                         |
| 9 (R/W1C)          | DRTO       | Data Read Timeout. The MSI_ISTAT.DRTO bit indicates that the card has not sent data within the time- out period.                                                                                                                                                       | Data Read Timeout. The MSI_ISTAT.DRTO bit indicates that the card has not sent data within the time- out period.                                                                                                                                                       |
|                    |            | 0                                                                                                                                                                                                                                                                      | No error                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                      | Error occurred                                                                                                                                                                                                                                                         |
| 8                  | RTO        | Response Timeout.                                                                                                                                                                                                                                                      | Response Timeout.                                                                                                                                                                                                                                                      |
| (R/W1C)            |            | 0                                                                                                                                                                                                                                                                      | No error                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                      | Error occurred                                                                                                                                                                                                                                                         |

Table 26-37: MSI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | DCRC       | Data CRC Error. The MSI_ISTAT.DCRC bit indicates that a data CRC error where the received data CRC does not match with the locally-generated CRC in the CIU. This error can also occur if the write CRC status is incorrectly sampled by the host.                                                               |
| 7 (R/W1C)          | DCRC       | 0 No error                                                                                                                                                                                                                                                                                                       |
| 7 (R/W1C)          | DCRC       | 1 Error occurred                                                                                                                                                                                                                                                                                                 |
| 6 (R/W1C)          | RCRC       | Response CRC Error. The MSI_ISTAT.RCRC bit indicates that the response CRC error which is caused when the response CRC does not match with the locally-generated CRC in the CIU.                                                                                                                                 |
| 6 (R/W1C)          | RCRC       | 0 No error                                                                                                                                                                                                                                                                                                       |
| 6 (R/W1C)          | RCRC       | 1 Error occurred                                                                                                                                                                                                                                                                                                 |
| 5 (R/W1C)          | RXDR       | Receive FIFO Data Request. The MSI_ISTAT.RXDR bit indicates that the FIFO threshold for receiving data was reached and software/DMA is expected to read data from the FIFO.                                                                                                                                      |
| 5 (R/W1C)          | RXDR       | 0 No error                                                                                                                                                                                                                                                                                                       |
| 5 (R/W1C)          | RXDR       | 1 Error occurred                                                                                                                                                                                                                                                                                                 |
| 4 (R/W1C)          | TXDR       | Transmit FIFO Data Request. The MSI_ISTAT.TXDR bit indicates that the FIFO threshold for transmitting data was reached and is less than or equal to transmit-threshold level. Software/DMA is ex- pected to write data, if available, in the FIFO. This interrupt is set during a write oper- ation to the card. |
| 4 (R/W1C)          | TXDR       | 0 No error                                                                                                                                                                                                                                                                                                       |
| 4 (R/W1C)          | TXDR       | 1 Error occurred                                                                                                                                                                                                                                                                                                 |
| 3 (R/W1C)          | DTO        | Data Transfer Over. The MSI_ISTAT.DTO bit indicates the data transfer completed. If there is a re- sponse timeout error, then the MSI does not attempt any data transfer and this bit is never set.                                                                                                              |
| 3 (R/W1C)          | DTO        | 0 No error                                                                                                                                                                                                                                                                                                       |
| 3 (R/W1C)          | DTO        | 1 Error occurred                                                                                                                                                                                                                                                                                                 |
| 2 (R/W1C)          | CMDDONE    | Command Done. The MSI_ISTAT.CMDDONE bit indicates that a command that was sent to a card received a response from the card, even a response error or CRC error occurred.                                                                                                                                         |
| 2 (R/W1C)          | CMDDONE    | 0 No error                                                                                                                                                                                                                                                                                                       |
| 2 (R/W1C)          | CMDDONE    | 1 Error occurred                                                                                                                                                                                                                                                                                                 |

Table 26-37: MSI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | RE         | Response Error. The MSI_ISTAT.RE bit indicates that an error was received during response recep- tion. In this case, the response that copied in the response registers is invalid. Software can retry the command. 0 No error 1 Error occurred |
| 0 (R/W1C)          | CD         | Card Detect. The MSI_ISTAT.CD bit indicates a card detect interrupt. 0 No error 1 Error occurred                                                                                                                                                |

## Masked Interrupt Status Register

The MSI\_MSKISTAT register indicates the status for the bits which are unmasked in the MSI\_IMSK register. In other words, MSI\_MSKISTAT = MSI\_ISTAT and MSI\_IMSK .

Figure 26-35: MSI\_MSKISTAT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000033_4159bd0a478768dd427b9436230c86a372ad4c4c4d9efbc14ad5586ba35e3d96.png)

Table 26-38: MSI\_MSKISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/NW)          | SDIOINT0   | SDIO Interrupt for Device 0. The MSI_MSKISTAT.SDIOINT0 bit indicates an interrupt occurred on the SDIO interrupt for device 0 masked interrupt. interrupt |
| 16 (R/NW)          | SDIOINT0   | 0 No                                                                                                                                                      |
| 16 (R/NW)          | SDIOINT0   | 1 Interrupt occurred                                                                                                                                      |

Table 26-38: MSI\_MSKISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | EBE        | End-bit Error. The MSI_MSKISTAT.EBE bit indicates an interrupt occurred on the end-bit error (read)/write no CRC masked interrupt.                                      |
| 15 (R/NW)          | EBE        | 0 No interrupt                                                                                                                                                          |
| 15 (R/NW)          | EBE        | 1 Interrupt occurred                                                                                                                                                    |
| 14 (R/NW)          | ACD        | Auto Command Done. The MSI_MSKISTAT.ACD bit indicates an interrupt occurred on the auto com- mand done masked interrupt.                                                |
| 14 (R/NW)          | ACD        | 0 No interrupt                                                                                                                                                          |
| 14 (R/NW)          | ACD        | 1 Interrupt occurred                                                                                                                                                    |
| 13 (R/NW)          | SBEBCI     | Start Bit Error (SBE)/Busy Complete Interrupt (BCI). The MSI_MSKISTAT.SBEBCI bit indicates an interrupt occurred on the start bit error/busy complete masked interrupt. |
| 13 (R/NW)          | SBEBCI     | 0 No interrupt                                                                                                                                                          |
| 12 (R/NW)          | HLE        | Hardware Locked Write Error. The MSI_MSKISTAT.HLE bit indicates an interrupt occurred on the hardware locked write error masked interrupt.                              |
| 12 (R/NW)          | HLE        | 0 No interrupt                                                                                                                                                          |
| 12 (R/NW)          | HLE        | 1 Interrupt occurred                                                                                                                                                    |
| 11 (R/NW)          | FRUN       | FIFO Underrun/Overrun Error. The MSI_MSKISTAT.FRUN bit indicates an interrupt occurred on the FIFO under- run/overrun error masked interrupt.                           |
| 11 (R/NW)          | FRUN       | 0 No interrupt                                                                                                                                                          |
| 11 (R/NW)          | FRUN       | 1 Interrupt occurred                                                                                                                                                    |
| 10 (R/NW)          | HTO        | Data Starvation by Host Timeout. The MSI_MSKISTAT.HTO bit indicates an interrupt occurred on the data starvation by host timeout masked interrupt.                      |
| 10 (R/NW)          | HTO        | 0 No interrupt                                                                                                                                                          |
| 10 (R/NW)          | HTO        | 1 Interrupt occurred                                                                                                                                                    |

Table 26-38: MSI\_MSKISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/NW)           | DRTO       | Data Read Timeout. The MSI_MSKISTAT.DRTO bit indicates an interrupt occurred on the data read timeout masked interrupt.                   |
| 9 (R/NW)           | DRTO       | 0 No interrupt                                                                                                                            |
| 9 (R/NW)           | DRTO       | 1 Interrupt occurred                                                                                                                      |
| 8 (R/NW)           | RTO        | Response Timeout. The MSI_MSKISTAT.RTO bit indicates an interrupt occurred on the response time- out masked interrupt.                    |
| 8 (R/NW)           | RTO        | 0 No interrupt                                                                                                                            |
| 8 (R/NW)           | RTO        | 1 Interrupt occurred                                                                                                                      |
| 7 (R/NW)           | DCRC       | Data CRC Error. The MSI_MSKISTAT.DCRC bit indicates an interrupt occurred on the data CRC error request masked interrupt.                 |
| 7 (R/NW)           | DCRC       | 0 No interrupt                                                                                                                            |
| 7 (R/NW)           | DCRC       | 1 Interrupt occurred                                                                                                                      |
| 6 (R/NW)           | RCRC       | Response CRC Error. The MSI_MSKISTAT.RCRC bit indicates an interrupt occurred on the response CRC error data request masked interrupt.    |
| 6 (R/NW)           | RCRC       | 0 No interrupt                                                                                                                            |
| 6 (R/NW)           | RCRC       | 1 Interrupt occurred                                                                                                                      |
| 5 (R/NW)           | RXDR       | Receive FIFO Data Request. The MSI_MSKISTAT.RXDR bit indicates an interrupt occurred on the receive FIFO data request masked interrupt.   |
| 5 (R/NW)           | RXDR       | 0 No interrupt                                                                                                                            |
| 5 (R/NW)           | RXDR       | 1 Interrupt occurred                                                                                                                      |
| 4 (R/NW)           | TXDR       | Transmit FIFO Data Request. The MSI_MSKISTAT.TXDR bit indicates an interrupt occurred on the transmit FIFO data request masked interrupt. |
| 4 (R/NW)           | TXDR       | 0 No interrupt                                                                                                                            |
| 4 (R/NW)           | TXDR       | 1 Interrupt occurred                                                                                                                      |

Table 26-38: MSI\_MSKISTAT Register Fields (Continued)

| Bit No. (Access)   | Description/Enumeration                                                                                                        |
|--------------------|--------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/NW)           | Data Transfer Over. The MSI_MSKISTAT.DTO bit indicates an interrupt occurred on the data transfer over masked interrupt.       |
| 2 (R/NW)           | Command Done. The MSI_MSKISTAT.CMDDONE bit indicates an interrupt occurred on the com- mand done masked interrupt.             |
| 1 (R/NW)           | Response Error. The MSI_MSKISTAT.RE bit indicates an interrupt occurred on the response error masked interrupt. 0 No interrupt |
| 0 (R/NW)           | Card Detect. The MSI_MSKISTAT.CD bit indicates an interrupt occurred on the card detect masked interrupt. 0 No interrupt       |

## Poll Demand Register

If the OWN bit of a descriptor is not set, the FSM goes into the suspend state. The host needs to write any value into the MSI\_PLDMND register for the IDMAC FSM to resume normal descriptor fetch operation. This is a writeonly register.

Figure 26-36: MSI\_PLDMND Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000034_a0c4bd4d7df4a692b33ae87622f0fb2a99654fd099e27b9512637754f0a1d16a.png)

Table 26-39: MSI\_PLDMND Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (RX/W)        | PD         | Poll Demand. The MSI_PLDMND.PD bit field needs to be written so that the IDMAC FSM can resume normal descriptor fetch operation. |

## Response Register 0

The MSI\_RESP0 register represents bits[31:0] of a long response.

Figure 26-37: MSI\_RESP0 Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000035_7c4f0b19a57e22204542219f1f591f3fe4291aea45a835e3840761724531d225.png)

Table 26-40: MSI\_RESP0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 31:0               | VALUE      | Response Register 0 Value.                                              |
| (R/NW)             |            | The MSI_RESP0.VALUE bit field represents bits[31:0] of a long response. |

## Response Register 1

The MSI\_RESP1 register represents bits[63:32] of a long response.

Figure 26-38: MSI\_RESP1 Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000036_8810fb7e9e14bd608642b676a56d5a0bb8127fe0f26e7e072e49b0af869514bb.png)

Table 26-41: MSI\_RESP1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 31:0               | VALUE      | Response Register 1 Value.                                               |
| (R/NW)             |            | The MSI_RESP1.VALUE bit field represents bits[63:32] of a long response. |

## Response Register 2

The MSI\_RESP2 register represents bits[95:64] of a long response.

Figure 26-39: MSI\_RESP2 Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000037_99c03339b3c1fa238c7ec026c2b5245b94bb823540d6f50297a40338cd94fa5f.png)

Table 26-42: MSI\_RESP2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 31:0               | VALUE      | Response Register 2 Value.                                               |
| (R/NW)             |            | The MSI_RESP2.VALUE bit field represents bits[95:64] of a long response. |

## Response Register 3

The MSI\_RESP3 register represents bits[127:96] of a long response.

Figure 26-40: MSI\_RESP3 Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000038_e08558cb15d28be18477b3c90226e52953fdd64ba687436805020390b7ef06ac.png)

Table 26-43: MSI\_RESP3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 31:0               | VALUE      | Response Register 3 Value.                                                |
| (R/NW)             |            | The MSI_RESP3.VALUE bit field represents bits[127:96] of a long response. |

## Status Register

The MSI\_STAT register provides MSI DMA and data transfer status and information.

Figure 26-41: MSI\_STAT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000039_4318d513ae994bf1cc645f524cc7aa5fa37f431550b2224316f3d328ee8469d5.png)

Table 26-44: MSI\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | DMAREQ     | DMARequest. The MSI_STAT.DMAREQ bit indicates the DMArequest signal state.                                                           |
| 30 (R/NW)          | DMAACK     | DMAAcknowledge. The MSI_STAT.DMAACK bit indicates the DMAacknowledge signal state.                                                   |
| 29:17 (R/NW)       | FIFOCNT    | FIFO Count. The MSI_STAT.FIFOCNT bit field indicates the number of filled locations in the FIFO.                                     |
| 16:11 (R/NW)       | RSPINDX    | Response Index. The MSI_STAT.RSPINDX bit field indicates the index of the previous response, in- cluding any auto-stop sent by core. |

Table 26-44: MSI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/NW)          | DMCBUSY    | Data State Machine Busy. The MSI_STAT.DMCBUSY bit indicates that the data transmit or receive state-ma- chine is busy.                                  |
| 10 (R/NW)          | DMCBUSY    | 0 DMCidle                                                                                                                                               |
| 10 (R/NW)          | DMCBUSY    | 1 DMCbusy                                                                                                                                               |
| 9 (R/NW)           | DBUSY      | Data Busy. The MSI_STAT.DBUSY bit indicates the inverted version of the raw selected card_data[0].                                                      |
| 9 (R/NW)           | DBUSY      | 0 Card data not busy                                                                                                                                    |
| 9 (R/NW)           | DBUSY      | 1 Card data busy                                                                                                                                        |
| 8 (R/NW)           | D33STAT    | Data 3 Status.                                                                                                                                          |
| 8 (R/NW)           | D33STAT    | 0 Card not present                                                                                                                                      |
| 7:4 (R/NW)         | CMDFSM     | Command FSM States. The MSI_STAT.CMDFSM bit field indicates the state machine status of CIU (card interface unit) or in general command FSM (not IDMA). |
| 7:4 (R/NW)         | CMDFSM     | 0 Idle                                                                                                                                                  |
| 7:4 (R/NW)         | CMDFSM     | 1 Send init sequence                                                                                                                                    |
| 7:4 (R/NW)         | CMDFSM     | 2 Tx cmd start bit                                                                                                                                      |
| 7:4 (R/NW)         | CMDFSM     | 3 Tx cmd tx bit                                                                                                                                         |
| 7:4 (R/NW)         | CMDFSM     | 4 Tx cmd index + arg                                                                                                                                    |
| 7:4 (R/NW)         | CMDFSM     | 5 Tx cmd CRC7                                                                                                                                           |
| 7:4 (R/NW)         | CMDFSM     | 6 Tx cmd end bit                                                                                                                                        |
| 7:4 (R/NW)         | CMDFSM     | 7 Rx resp start bit                                                                                                                                     |
| 7:4 (R/NW)         | CMDFSM     | 8 Rx resp IRQ response                                                                                                                                  |
| 7:4 (R/NW)         | CMDFSM     | 9 Rx resp tx bit                                                                                                                                        |
| 7:4 (R/NW)         | CMDFSM     | 10 Rx resp cmd idx                                                                                                                                      |
| 7:4 (R/NW)         | CMDFSM     | 11 Rx resp data                                                                                                                                         |
| 7:4 (R/NW)         | CMDFSM     | 12 Rx resp CRC7                                                                                                                                         |
| 7:4 (R/NW)         | CMDFSM     | 13 Rx resp end bit                                                                                                                                      |
| 7:4 (R/NW)         | CMDFSM     | 14 Cmd path wait NCC                                                                                                                                    |

Table 26-44: MSI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/NW)           | FIFOFULL   | FIFO Full. The MSI_STAT.FIFOFULL bit indicates that the FIFO is full.                                                                                | FIFO Full. The MSI_STAT.FIFOFULL bit indicates that the FIFO is full.                                                                                |
| 2 (R/NW)           | FIFOEMPTY  | FIFO Empty. The MSI_STAT.FIFOEMPTY bit indicates that the FIFO is empty.                                                                             | FIFO Empty. The MSI_STAT.FIFOEMPTY bit indicates that the FIFO is empty.                                                                             |
| 1 (R/NW)           | FIFOTXWM   | FIFO Tx Watermark. The MSI_STAT.FIFOTXWM bit indicates that the FIFO reached the transmit water- mark level and is not qualified with data transfer. | FIFO Tx Watermark. The MSI_STAT.FIFOTXWM bit indicates that the FIFO reached the transmit water- mark level and is not qualified with data transfer. |
|                    |            | 0                                                                                                                                                    | Did not reach watermark level                                                                                                                        |
|                    |            | 1                                                                                                                                                    | Reached watermark level                                                                                                                              |
| 0 (R/NW)           | FIFORXWM   | FIFO Rx Watermark. The MSI_STAT.FIFORXWM bit indicates that the FIFO reached the receive water- mark level and is not qualified with data transfer.  | FIFO Rx Watermark. The MSI_STAT.FIFORXWM bit indicates that the FIFO reached the receive water- mark level and is not qualified with data transfer.  |
|                    |            | 0                                                                                                                                                    | Did not reach watermark level                                                                                                                        |
|                    |            | 1                                                                                                                                                    | Reached watermark level                                                                                                                              |

## Transferred Host to BIU-FIFO Byte Count Register

The MSI\_TBBCNT register provides the number of bytes transferred between host/DMA memory and BIU FIFO. The register should be accessed in full to avoid read-coherency problems.

Figure 26-42: MSI\_TBBCNT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000040_4488f0e76717d057506bb07b2368ddcb5111929c33266c213f2a357ca9b1f598.png)

Table 26-45: MSI\_TBBCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Transferred FIFO Byte Count. The MSI_TBBCNT.VALUE bit field provides the number of bytes transferred be- tween the host/DMA memory and the BIU FIFO. |

## Transferred CIU Card Byte Count Register

The MSI\_TCBCNT register provides the number of bytes transferred by the CIU unit to a card. This register should be accessed in full to avoid read-coherency problems. Both the MSI\_TCBCNT and MSI\_TBBCNT registers share the same coherency register.

Figure 26-43: MSI\_TCBCNT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000041_c0cfb9bc122afebc84c77ac88e7222b238b13804273b4a737c57994c67518f43.png)

Table 26-46: MSI\_TCBCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Transferred Card Byte Count. The MSI_TCBCNT.VALUE bit field provides the number of bytes transferred by the CIU unit to a card. |

## Timeout Register

The MSI\_TMOUT register provides bits that configure card data read and response timeout values.

Figure 26-44: MSI\_TMOUT Register Diagram

![Image](29_Mobile_Storage_Interface_(MSI)_artifacts/image_000042_013cebaf46e201d07beb326c226947a30c0f4631644260e0c2cdf992da10d596.png)

Table 26-47: MSI\_TMOUT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:8 (R/W)         | DATA       | Data Timeout. The MSI_TMOUT.DATA bit field configures the value for the card data read timeout; the same value is also used for the data starvation by host timeout. The timeout coun- ter is started only after the card clock is stopped. The value is in number of card out- put clocks cclk_out of a selected card. |
| 7:0 (R/W)          | RESPONSE   | Response Timeout. The MSI_TMOUT.RESPONSE bit field configures the response timeout in number of card output clocks.                                                                                                                                                                                                     |