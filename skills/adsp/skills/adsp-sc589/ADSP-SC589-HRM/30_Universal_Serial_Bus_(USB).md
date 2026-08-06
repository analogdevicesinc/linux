## 27   Universal Serial Bus (USB)

The USB OTG controller provides a low-cost connectivity solution for consumer mobile devices such as cell phones, digital still cameras, and MP3 players. It allows these devices to transfer data using a point-to-point USB connection without the need for a personal computer host.

The USB controller can operate in a traditional USB peripheral-only mode as well as the host mode presented in the On-The-Go (OTG) supplement to the USB 2.0 Specification.

NOTE: See the On-The-Go Supplement to the USB 2.0 Specification, Rev 1.0a; June 24, 2003; USB-IF and the Universal Serial Bus Specification 2.0.

The USB module supports:

- Host mode transfers at high-speed (480 Mbp/sec) rate
- Host mode transfers at full-speed (12 Mbp/sec) rate
- Host mode transfers at low-speed (1.5 Mbp/sec) rates. The connection to low-speed devices is only possible through a full-speed hub.
- Peripheral mode transfers at high-speed (480 Mbp/sec) rate
- Peripheral mode transfers at full-speed (12 Mbp/sec) rate

The USB controller uses a peripheral bus slave interface to access its control and status registers as well as read and write to the endpoint packet buffers. Data transfers to and from the USB controller through the 11 transmit and 11 receive endpoint FIFOs (EP1 - EP11), providing a total of 22 data endpoints.

## USB Features

The USB controller provides the following features:

- Low-speed, full-speed, and high-speed rates supported
- One bidirectional control endpoint
- 11 transmit and 11 receive unidirectional endpoints
- 16 KB dynamically configured FIFO RAM

- Eight DMA master channels
- Two top-level maskable general-purpose interrupts
- Low-power wake-up on activity
- VBUS control interrupts for external analog VBUS control
- Session request protocol (SRP) and host negotiation protocol (HNP) capability
- Host transaction scheduling in hardware
- Soft connect or disconnect feature
- Full- and high-speed physical layer UTMI+ level 3 interface for on-chip PHY
- Backwards compatible with existing USB 1.1 hosts
- Support for Battery Charging Specification Revision 1.1

Only device requirements or system bandwidth limit the number of active endpoints at one time because each endpoint operates independently from the next. Software determines the type of transfer for each endpoint individually and also the manner in which it is transferred between the USB controller and memory (DMA or interrupt-based). The USB uses endpoint zero solely for receive and transmit control transfer. These transfers are used for device configuration and information gathering.

## USB Functional Description

The following sections describe the function of the USB OTG interface.

## USB Architectural Concepts

The USB controller operates in either of two USB operation modes (peripheral or host mode) at a given time.

In peripheral mode, the USB controller encodes, decodes, checks, and directs all USB packets sent and received, responding appropriately to host requests. Data is transferred from the processor core memory into the Tx FIFOs of the device onto USB as IN packets. In the other direction, USB OUT packets are received into the Rx FIFOs (having been sent from the host) and transferred to system memory for processing or storage. In peripheral mode, the USB controller acts as a slave device to another USB host; either a personal computer or another OTG host controller.

When operating in host mode, the USB controller uses simple hosting capabilities to master point-to-point connections with another USB peripheral, initiating transfers on the bus for the peripheral to respond. USB IN packets are received into the Rx FIFOs for transfer into the processor core memory. Data written into Tx FIFOs is transmitted onto the bus as USB OUT packets. In this mode, the USB controller encodes, decodes, and checks USB packets sent and received. The controller automatically schedules isochronous and interrupt transfers from the endpoint buffers. It performs one transaction every n frames, where n represents the polling interval programmed for the endpoint.

Any of the endpoints can be programmed to be written to or read from using the DMA master channels. This configuration provides the most efficient means of transferring data between the controller and on-chip memory.

USB endpoints 0 through 11 have DMA interrupt lines ( USB\_DMA\_IRQ ) providing a total of eight DMA request lines.

The USB provides two top-level maskable interrupts, each of which can be sourced from any or all of transmit endpoint status, receive endpoint status or global USB status. See Interrupt Signals for details.

The RAM interface of the USB controller supports a single block of synchronous single-port RAM used to buffer the USB packets.

16K bytes of SRAM are available.

The UTMI+ level 3 PHY interface provides a means of connecting a selection of high- or full-speed PHYs to the controller, from device-only PHYs through full OTG-compliant PHYs.

For details of the PHY interface, See UTMI Interface.

ATTENTION: Check the processor data sheet for requirements regarding minimum system clock frequency needed for proper USB operation.

The USB controller is configured as either a USB OTG A device or B device depending on the type of plug inserted into its USB receptacle. The state of the USB\_ID (connector ID) pin determines this configuration.

The USB controller uses an asynchronous wake-up circuit to detect when another B device is asserting its D+ pullup. This activity initiates the SRP (session request protocol) when all other clocks are off.

This slow clock is derived from SCLK0\_0 and enabled using the USB\_PHY\_CTL.EN bit.

Use of the controller for OTG functionality requires the capability to:

- Drive VBUS (as a default A device powering the bus)
- Discharge VBUS (speeding up the time for VBUS to fall below the SessionEnd threshold as a B device checking initial conditions)
- Charge VBUS to 2.1 V (when initiating SRP as a B device).

The UTMI interface drives these controls, but the controller also provides a separate interrupt register, USB\_VBUS\_CTL , which represents the drive VBUS, discharge VBUS, and charge VBUS signaling. See the register section for more information on these controls.

## Multi-Point Support

The USB controller has the facility, when operating in host mode, to act as the host to a range of USB peripheral devices.

High-speed, full-speed, or low-speed devices connect to the USB controller through a USB hub.

The USB controller, as part of its support for multiple devices, permits individual allocation of the functions of the target to the different Rx and Tx endpoints implemented. Furthermore, the USB controller can make this allocation

dynamically, allowing the devices from the targeted peripheral list to be used in different combinations. The numbers of Tx and Rx endpoints implemented in the controller limit the combinations of peripheral devices that can be used together. Devices can only be added where the required endpoints remain available.

## On-Chip Bus Interfaces

The USB controller uses two 32-bit wide independent bus interfaces, a master and a slave, to communicate with a processor-based subsystem. The slave interface allows the processor core to access the control and status registers (including DMA master registers) and the endpoint FIFOs. The integrated DMA uses the master interface to drive data into or out of the endpoint FIFOs with minimal processor core interaction. For more information, see USB Block Diagram.

## FIFO Configuration

Each bidirectional endpoint (provided as two unidirectional endpoints) has its own endpoint number (0 for control, 1 on up for data transfer). Although two endpoints could use the same number, the endpoints can support different transfer types. Each of these bidirectional endpoints has a fixed region of the SRAM in the USB controller to which it has access. This feature dictates to some extent the types of transfers that can be used for that particular endpoint. This restriction follows from the maximum size of USB packets, which varies with each transfer type. The FIFO Sizes and Transfer Types table lists the endpoint FIFO configuration, with an indication of the transfer types possible for that particular buffer size.

Table 27-1: FIFO Sizes and Transfer Types

| Bidirectional Endpoint (Rx and Tx)   | FIFO Size (each direction)                                 | USB Transfer Types               |
|--------------------------------------|------------------------------------------------------------|----------------------------------|
| 0                                    | 64 bytes                                                   | Size fixed for control transfers |
| 1-4                                  | Dynamically configured in powers of 2 from 8 to 8192 bytes | Bulk, Interrupt, Isochronous     |
| 1-11                                 | Dynamically configured in powers of 2 from 8 to 8192 bytes | Bulk, Interrupt, Isochronous     |

Each endpoint FIFO can buffer one or two packets (in double-buffered mode). Double-buffering is recommended for most applications to improve efficiency by reducing the frequency of servicing for each endpoint.

Double-buffering bulk transactions means that data transfers over the USB are not slowed when packets are loaded or unloaded from the FIFO in the time it takes to transfer a packet. Double-buffering isochronous transactions allows more time to load or unload the FIFO. It also allows the usage of the SOF interrupt to service the endpoint rather than the endpoint interrupt. This functionality has the following advantages:

- Easy detection of lost packets
- Regular interrupt timing (making it easier to source or sink the data)
- If the USB controller uses more than one isochronous endpoint, one interrupt can service all the endpoints.

The USB controller uses the transmit or receive FIFO address registers to specify the address of each endpoint FIFO.

## Clocking

The USB controller uses the system clock SCLK0\_0 to generate an internal clock (CLK) used to clock the USB registers.

For proper operation, refer to device datasheet for minimum system clock SCLK0\_0 value.

NOTE: For best performance (best signal integrity), follow the guidelines in the data sheet for selecting an input clock frequency.

When the controller is in the SUSPEND state and when no session is active, the clock and much of the USB controller is stopped to reduce power consumption. The clock becomes operational again when RESUME signaling is detected on the USB lines.

## UTMI Interface

The interface to the on-chip PHY uses the industry-standard UTMI+ (universal transceiver macro interface) level 3.

This interface provides full- and high-speed device and OTG functionality and supports communication to a hub.

The PHY is a mixed-signal block and includes the following:

- Full-speed and high-speed drivers and receivers (single-ended and differential)
- Full-speed and high-speed CDR
- Full-speed or high-speed shift registers, NRZI encode or decode and bit-stuff encode or decode
- Data line pull-up and pull-down resistors
- VBUS and USB\_ID level detection
- Host disconnect detection

Although the UTMI specification indicates that VBUS charging, driving and discharging happen inside the PHY, for process-restricting and power reasons, implement these functions off-chip in a separate USB charge-pump chip.

## ADSP-SC58x USB Register List

The Universal Serial Bus controller (USB) is a multi-point high-speed dual-role USB 2.0-compliant controller. The USB controller can operate in a traditional USB peripheral-only mode as well as the host mode presented in the OnThe-Go (OTG) supplement to the USB 2.0 Specification, Rev 1.0a; June 24, 2003; USB-IF . A set of registers governs USB controller operations. For more information on USB controller functionality, see the USB controller register descriptions.

Table 27-2: ADSP-SC58x USB Register List

| Name          | Description                               |
|---------------|-------------------------------------------|
| USB_BAT_CHG   | Battery Charging Control Register         |
| USB_CT_HHSRTN | Host High-Speed Return to Normal Register |

Table 27-2: ADSP-SC58x USB Register List (Continued)

| Name                  | Description                                                 |
|-----------------------|-------------------------------------------------------------|
| USB_CT_HSBT           | High-Speed Timeout Register                                 |
| USB_CT_UCH            | Chirp Timeout Register                                      |
| USB_DEV_CTL           | Device Control Register                                     |
| USB_DMA[n]_ADDR       | DMAChannel n Address Register                               |
| USB_DMA[n]_CNT        | DMAChannel n Count Register                                 |
| USB_DMA[n]_CTL        | DMAChannel n Control Register                               |
| USB_DMA_IRQ           | DMAInterrupt Register                                       |
| USB_EP0I_CFGDATA[N]   | EP0 Configuration Information Register                      |
| USB_EP0I_CNT[N]       | EP0 Number of Received Bytes Register                       |
| USB_EP0I_CSR[N]_H     | EP0 Configuration and Status (Host) Register                |
| USB_EP0I_CSR[N]_P     | EP0 Configuration and Status (Peripheral) Register          |
| USB_EP0I_NAKLIMIT[N]  | EP0 NAK Limit Register                                      |
| USB_EP0I_TYPE[N]      | EP0 Connection Type Register                                |
| USB_EP0_CFGDATA[n]    | EP0 Configuration Information Register                      |
| USB_EP0_CNT[n]        | EP0 Number of Received Bytes Register                       |
| USB_EP0_CSR[n]_H      | EP0 Configuration and Status (Host) Register                |
| USB_EP0_CSR[n]_P      | EP0 Configuration and Status (Peripheral) Register          |
| USB_EP0_NAKLIMIT[n]   | EP0 NAK Limit Register                                      |
| USB_EP0_TYPE[n]       | EP0 Connection Type Register                                |
| USB_EPINFO            | Endpoint Information Register                               |
| USB_EPI[N]_RXCNT      | EPn Number of Bytes Received Register                       |
| USB_EPI[N]_RXCSR_H    | EPn Receive Configuration and Status (Host) Register        |
| USB_EPI[N]_RXCSR_P    | EPn Receive Configuration and Status (Peripheral) Register  |
| USB_EPI[N]_RXINTERVAL | EPn Receive Polling Interval Register                       |
| USB_EPI[N]_RXMAXP     | EPn Receive Maximum Packet Length Register                  |
| USB_EPI[N]_RXTYPE     | EPn Receive Type Register                                   |
| USB_EPI[N]_TXCSR_H    | EPn Transmit Configuration and Status (Host) Register       |
| USB_EPI[N]_TXCSR_P    | EPn Transmit Configuration and Status (Peripheral) Register |
| USB_EPI[N]_TXINTERVAL | EPn Transmit Polling Interval Register                      |
| USB_EPI[N]_TXMAXP     | EPn Transmit Maximum Packet Length Register                 |
| USB_EPI[N]_TXTYPE     | EPn Transmit Type Register                                  |

Table 27-2: ADSP-SC58x USB Register List (Continued)

| Name                 | Description                                                 |
|----------------------|-------------------------------------------------------------|
| USB_EP[n]_RXCNT      | EPn Number of Bytes Received Register                       |
| USB_EP[n]_RXCSR_H    | EPn Receive Configuration and Status (Host) Register        |
| USB_EP[n]_RXCSR_P    | EPn Receive Configuration and Status (Peripheral) Register  |
| USB_EP[n]_RXINTERVAL | EPn Receive Polling Interval Register                       |
| USB_EP[n]_RXMAXP     | EPn Receive Maximum Packet Length Register                  |
| USB_EP[n]_RXTYPE     | EPn Receive Type Register                                   |
| USB_EP[n]_TXCSR_H    | EPn Transmit Configuration and Status (Host) Register       |
| USB_EP[n]_TXCSR_P    | EPn Transmit Configuration and Status (Peripheral) Register |
| USB_EP[n]_TXINTERVAL | EPn Transmit Polling Interval Register                      |
| USB_EP[n]_TXMAXP     | EPn Transmit Maximum Packet Length Register                 |
| USB_EP[n]_TXTYPE     | EPn Transmit Type Register                                  |
| USB_FADDR            | Function Address Register                                   |
| USB_FIFOB[n]         | FIFO Byte (8-Bit) Register                                  |
| USB_FIFOH[n]         | FIFO Half-Word (16-Bit) Register                            |
| USB_FIFO[n]          | FIFO Word (32-Bit) Register                                 |
| USB_FRAME            | Frame Number Register                                       |
| USB_FS_EOF1          | Full-Speed EOF 1 Register                                   |
| USB_HS_EOF1          | High-Speed EOF 1 Register                                   |
| USB_IDCTL            | ID Control                                                  |
| USB_IEN              | Common Interrupts Enable Register                           |
| USB_INDEX            | Index Register                                              |
| USB_INTRRX           | Receive Interrupt Register                                  |
| USB_INTRRXE          | Receive Interrupt Enable Register                           |
| USB_INTRTX           | Transmit Interrupt Register                                 |
| USB_INTRTXE          | Transmit Interrupt Enable Register                          |
| USB_IRQ              | Common Interrupts Register                                  |
| USB_LINKINFO         | Link Information Register                                   |
| USB_LPM_ATTR         | LPM Attribute Register                                      |
| USB_LPM_CTL          | LPM Control Register                                        |
| USB_LPM_FADDR        | LPM Function Address Register                               |
| USB_LPM_IEN          | LPM Interrupt Enable Register                               |

Table 27-2: ADSP-SC58x USB Register List (Continued)

| Name                 | Description                            |
|----------------------|----------------------------------------|
| USB_LPM_IRQ          | LPM Interrupt Status Register          |
| USB_LS_EOF1          | Low-Speed EOF 1 Register               |
| USB_MP[n]_RXFUNCADDR | MPn Receive Function Address Register  |
| USB_MP[n]_RXHUBADDR  | MPn Receive Hub Address Register       |
| USB_MP[n]_RXHUBPORT  | MPn Receive Hub Port Register          |
| USB_MP[n]_TXFUNCADDR | MPn Transmit Function Address Register |
| USB_MP[n]_TXHUBADDR  | MPn Transmit Hub Address Register      |
| USB_MP[n]_TXHUBPORT  | MPn Transmit Hub Port Register         |
| USB_PHY_CTL          | PHY Control Register                   |
| USB_PLL_OSC          | PLL and Oscillator Control Register    |
| USB_POWER            | Power and Device Control Register      |
| USB_RAMINFO          | RAM Information Register               |
| USB_RQPKTCNT[n]      | EPn Request Packet Count Register      |
| USB_RXFIFOADDR       | Receive FIFO Address Register          |
| USB_RXFIFOSZ         | Receive FIFO Size Register             |
| USB_SOFT_RST         | Software Reset Register                |
| USB_TESTMODE         | Testmode Register                      |
| USB_TXFIFOADDR       | Transmit FIFO Address Register         |
| USB_TXFIFOSZ         | Transmit FIFO Size Register            |
| USB_VBUS_CTL         | VBUS Control Register                  |
| USB_VPLEN            | VBUS Pulse Length Register             |

## ADSP-SC58x USB Interrupt List

Table 27-3: ADSP-SC58x USB Interrupt List

|   Interrupt ID | Name      | Description                      | Sensitivity   | DMA Channel   |
|----------------|-----------|----------------------------------|---------------|---------------|
|            132 | USB0_STAT | USB0 Status/FIFO Data Ready      | Level         |               |
|            133 | USB0_DATA | USB0 DMAStatus/Transfer Complete | Level         |               |
|            134 | USB1_STAT | USB1 Status/FIFO Data Ready      | Level         |               |
|            135 | USB1_DATA | USB1 DMAStatus/Transfer Complete | Level         |               |

## ADSP-SC58x USB Trigger List

Table 27-4: ADSP-SC58x USB Trigger List Masters

|   Trigger ID | Name      | Description                      | Sensitivity   |
|--------------|-----------|----------------------------------|---------------|
|           72 | USB0_DATA | USB0 DMAStatus/Transfer Complete | Level         |
|           73 | USB1_DATA | USB1 DMAStatus/Transfer Complete | Level         |

Table 27-5: ADSP-SC58x USB Trigger List Slaves

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## USB Block Diagram

The USB OTG Controller Block Diagram shows the functional blocks within the USB. For more information about the blocks, see the USB Functional Description.

Figure 27-1: USB OTG Controller Block Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000000_5c4db505908acb28cbea8e41abb5efa29e9291c0e2e56901541151ae9b3caf9c.png)

## USB Definitions

A list of common USB terms and their definitions as used in this specification and based on the USB controller follows:

## 'A' Device

The USB device with a mini-A plug inserted into its receptacle. The A device always supplies power to VBUS.

## 'B' Device

The USB device with a standard-B or mini-B plug inserted into its receptacle. The B device starts a session as the peripheral.

## Bidirectional endpoint

An endpoint that can concurrently support both receive and transfer packets.

## Control endpoint

An endpoint used only for transfer of USB control packets for setup and configuration. In all USB devices, the control endpoint refers to the bidirectional endpoint 0.

## Dual role device

A USB device that can operate either as the USB host in an OTG session or as a traditional USB peripheral.

## Endpoint

A single physical communication channel for USB, implemented as a FIFO and control logic for that endpoint. Each endpoint has an associated USB transfer type, maximum packet size, bandwidth requirement, endpoint number, and (often) a fixed transfer direction.

## Frame

A regular, fixed 1 ms timeslot that can contain several transactions. The transfer type determines the permissible transactions for a given endpoint.

## HNP

Host negotiation protocol. Part of the USB OTG supplement that allows the host function to be transferred between two connected dual role devices.

## Packet

The lowest level of data exchange on USB. The transfer type and buffer size of the USB peripheral determine the size.

## PHY

The PHY is a transceiver circuit that implements the physical layer of USB. For full speed USB OTG, this circuit includes line drivers and receivers, pull-up, or pull-down resistors as well as device ID and VBUS level detection.

## Session

A period during which USB transfers take place within an OTG connection. The A device (drives VBUS) or B device (initiates SRP) can initiate this period. VBUS is powered during a session.

## SRP

Session request protocol. Part of the USB OTG supplement that allows a B device to turn on VBUS and initiate a USB session.

## Transaction

Collection of one or more packets in sequence

## Transfer

Collection of one or more transfers in sequence

## Unidirectional endpoint

Endpoint with its direction fixed in a single direction (for example, it can only receive packets from the USB) in both host and peripheral modes.

## USB References

The following references provide further information regarding the USB.

- On-The-Go Supplement to the USB 2.0 Specification , Rev 1.0a, June 24, 2003, USB-IF
- Universal Serial Bus Specification 2.0

## USB Operating Modes

The USB OTG interface can operate in peripheral mode or host mode.

When the USB controller operates in peripheral mode, the controller can be attached to a conventional host (such as a personal computer) or another OTG device operating in host mode. The second device can be high speed or full speed. When linked to another peripheral device, the USB controller can also act as the host. If the other device is also a dual role controller, the two devices can switch roles, as needed.

The role the USB controller takes depends on the way the devices are cabled together. Each USB cable has an A and a B device end. If the A end of the cable is plugged into the device containing the USB controller, the USB controller takes the role of the host device. It goes into host mode (in this case, the USB\_DEV\_CTL.HOSTMODE bit is set to 1). If the B of the cable is plugged in, the USB controller goes instead into peripheral mode (and the USB\_DEV\_CTL.HOSTMODE bit remains at 0).

When both devices contain dual role controllers, signaling can be used to switch the roles of the two devices, without switching the cable connecting the two devices. See Host Negotiation Protocol for details on the conditions under which the USB controller can switch between peripheral and host mode.

NOTE: The multi-point capability of the USB controller is associated with a range of registers recording the allocation of device functions to individual endpoints and device function characteristics. These characteristics include endpoint number, operating speed, and transaction type on an endpoint-by-endpoint basis. These registers are principally associated with the use of the USB controller as the host to a number of devices. However, set the registers when the core is used as the host for a single target device.

To enable the USB:

1. Configure the USB PLL multiplier settings in the USB PLL control register. Check the processor data sheet for the requirements for input clock frequency.
2. Enable the USB PHY by setting the USB\_PHY\_CTL.EN bit.
3. Poll the bit in the USB PLL control register to ensure that the USB PLL has locked to the new frequency.

## Peripheral Mode

USB OTG interface operations for the peripheral mode differ from host mode in a number of ways. The following sections describe peripheral mode operations.

## Endpoint Setup

In peripheral mode, the USB uses a few endpoint-specific configuration bits when setting up an endpoint for transfer for all types of peripheral transfer. The configuration determines how the processor core interacts with the endpoint FIFO.

One key parameter required before a transfer can occur through an endpoint is the maximum USB packet size that the endpoint can support. The software sets this value. It depends on various system constraints. These constraints include the size of hardware FIFO available and system latencies as well as the USB transfer type and class used. The USB\_EP[n]\_TXMAXP or USB\_EP[n]\_RXMAXP registers define the maximum amount of data that can be transferred to the selected endpoint in a single frame. The value must match the programmed maximum individual packet size ( MaxPktSize ) of the standard endpoint descriptor for the endpoint.

For transmit endpoints, program the maximum packet size using the USB\_EP[n]\_TXMAXP . For receive endpoints, the USB uses the USB\_EP[n]\_RXMAXP register. The maximum packet size must not exceed the actual hardware endpoint FIFO size.

The settings in the USB\_RXFIFOSZ or USB\_TXFIFOSZ register determine the corresponding sizes of the transmit or receive FIFOs, as well as, single or double buffered mode for endpoints 1 to 11.

Because the USB controller uses a 32-bit interface, choose an even number for the value of MaxPktSize . This selection simplifies transferring data between FIFOs and the processor core.

Configure more setup parameters using the USB\_EP[n]\_TXCSR\_H or USB\_EP[n]\_RXCSR\_H register (depending on whether the endpoint in question is for receive or transmit). The USB uses the USB\_EP[n]\_RXCSR\_H.DMAREQEN bit in this register to enable the assertion of the appropriate DMA request whenever the endpoint is able to receive or transmit another packet. The USB uses the USB\_EP[n]\_RXCSR\_H.AUTOCLR and USB\_EP[n]\_RXCSR\_H.AUTOREQ bits to set the FIFO ready triggers ( USB\_EP[n]\_RXCSR\_H.RXPKTRDY and USB\_EP[n]\_TXCSR\_H.TXPKTRDY ) automatically whenever a packet is transferred to streamline DMA operation for transfers that span multiple packets. Note, however, that the USB cannot use USB\_EP[n]\_RXCSR\_H.AUTOCLR and USB\_EP[n]\_RXCSR\_H.AUTOREQ bits with high-bandwidth endpoints. Refer to the following register sections for more information: USB\_EP[n]\_TXCSR\_H , USB\_EPI[N]\_RXCSR\_P , USB\_EPI[N]\_RXCSR\_H , USB\_EP[n]\_TXCSR\_P .

## IN Transactions as a Peripheral

When the USB controller operates in peripheral mode, the transmit FIFOs handle data for IN transactions. The maximum size of data packet that can be placed in a FIFO for a transmit endpoint is programmable. When applicable, the value written to the USB\_EP[n]\_TXMAXP register for that endpoint determines the size (maximum payload multiplied by the number of transactions per micro-frame).

The maximum packet size set for any endpoint must not exceed the FIFO size. (See FIFO Configuration.)

ATTENTION: Do not write to the USB\_EP[n]\_TXMAXP register while there is data in the FIFO, as unexpected results can occur.

The following sections describe the two types of packet buffering used for IN transactions.

Single packet buffering . Set the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit as each packet for transmission is loaded into the transmit FIFO. If the USB\_EP[n]\_TXCSR\_P.AUTOSET bit is set, the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is automatically set when a maximum-sized packet is loaded into the FIFO. For packet sizes less than the maximum, and where auto-set cannot be used (high-bandwidth isochronous or interrupt transactions), always set the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit manually (for example by the pro-

cessor core).

When the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is set, either manually or automatically, the USB\_EP[n]\_TXCSR\_P.NEFIFO bit is also set and the packet is ready to be sent. When the packet is successfully sent, both the USB\_EP[n]\_TXCSR\_P.TXPKTRDY and USB\_EP[n]\_TXCSR\_P.NEFIFO bits are cleared. The USB controller generates the appropriate transmit endpoint interrupt (if enabled). The next packet can then be loaded into the FIFO.

Double packet buffering . Set the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit as each packet for transmission is loaded into the transmit FIFO. If the USB\_EP[n]\_TXCSR\_P.AUTOSET bit is set, the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is automatically set when a maximum-sized packet is loaded into the

FIFO. For packet sizes less than the maximum, and where auto-set cannot be used (high-bandwidth isochronous or interrupt transactions), always set the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit manually (for example by the processor core).

When the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is set, either manually or automatically, the USB\_EP[n]\_TXCSR\_P.NEFIFO bit also is set. The USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is then immediately cleared (and an interrupt generated, if enabled). A second packet can now be loaded into the transmit FIFO and the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is set again (either manually or automatically if the packet is the maximum size). Both packets are now ready for transmission.

When the first packet is successfully sent, the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is cleared. The USB controller generates the appropriate transmit endpoint interrupt (if enabled) to signal that another packet can now be loaded into the transmit FIFO. The state of the USB\_EP[n]\_TXCSR\_P.NEFIFO bit indicates how many packets can be loaded. If the USB\_EP[n]\_TXCSR\_P.NEFIFO bit is set, then there is another packet in the FIFO and only one more packet can be loaded. If the USB\_EP[n]\_TXCSR\_P.NEFIFO bit is cleared, then there are no packets in the FIFO and two more packets can be loaded.

## OUT Transactions as a Peripheral

When the USB controller operates in peripheral mode, the receive FIFOs handle data for OUT transactions.

The value written to the USB\_EP[n]\_RXMAXP register for an endpoint determines the maximum amount of data received by a receive endpoint in any frame. The value is programmable. The maximum packet size must not exceed the FIFO size.

The value written to the USB\_EP[n]\_RXMAXP register for an endpoint determines maximum amount of data received by a receive endpoint in any micro-frame (in high-speed mode). The value is programmable. It is the maximum payload multiplied by the number of transactions per micro-frame. The maximum packet size must not exceed the FIFO size.

If the size of the receive endpoint FIFO is less than twice the maximum packet size for this endpoint, only one data packet can be buffered in the FIFO. Single buffering is selected. (The size is set in the USB\_EP[n]\_RXMAXP register.) When a packet is received and placed in the receive FIFO, the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit and the USB\_EP[n]\_RXCSR\_P.FIFOFULL bit are set. The USB controller generates the appropriate receive endpoint interrupt (if enabled) to signal that a packet can now be unloaded from the FIFO. After the packet is unloaded, clear the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit to allow reception of more packets. If the USB\_EP[n]\_RXCSR\_P.AUTOCLR bit is set and a maximum-sized packet is unloaded from the FIFO, the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit is cleared automatically. The USB\_EP[n]\_RXCSR\_P.FIFOFULL bit is also cleared. For packet sizes less than the maximum, clear the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit manually (for example by the processor core).

If double packet buffering is enabled, then two data packets can be buffered. When the first packet for reception is loaded into the receive FIFO, the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit is set. The USB controller generates the appropriate receive endpoint interrupt (if enabled) to signal that a packet can now be unloaded from the FIFO. The USB\_EP[n]\_RXCSR\_P.FIFOFULL bit is not set. This bit is only set if a second packet is received and loaded into the receive FIFO.

After the first packet is unloaded, clear the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit to allow reception of further packets. If the USB\_EP[n]\_RXCSR\_P.AUTOCLR bit is set and a maximum-sized packet is unloaded from the FIFO, the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit is cleared automatically. For packet sizes less than the maximum, clear the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit manually (for example by the processor core).

If the USB\_EP[n]\_RXCSR\_P.FIFOFULL bit is set to 1 when USB\_EP[n]\_RXCSR\_P.RXPKTRDY is cleared, the USB controller first clears the USB\_EP[n]\_RXCSR\_P.FIFOFULL bit. The controller then sets the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit again, indicating that there is another packet waiting in the FIFO for unloading.

## High-Bandwidth Isochronous or Interrupt Transactions

High-bandwidth isochronous or interrupt transactions use much the same protocol as other isochronous or interrupt transactions. There are, however, some special features to conducting high-bandwidth transactions.

- When setting the maximum packet size handled by the endpoint in the USB\_EP[n]\_TXMAXP / USB\_EP[n]\_RXMAXP registers, set the maximum number of transactions per micro-frame using the USB\_EP[n]\_TXMAXP.MULTM1 and USB\_EP[n]\_RXMAXP.MULTM1 bits.

The maximum number of transactions (2 or 3) also represents the maximum number of sections in which any single high-bandwidth packet can be transferred. The configuration sets the maximum size of the packet to 2 or 3 times the maximum payload specified for the endpoint in the same register.

NOTE: The maximum payload that can be sent in any transaction is 1K byte.

- When sending packets, set the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit using the application software. Similarly, when unloading packets from the receive endpoint FIFO, clear the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit using the application software.

CAUTION: The AutoSet and AutoClear functions cannot be used to set and clear these bits in high-bandwidth transactions.

- The transmission of packets as a number of sections introduces a further type of error - the transmission of incomplete packets.

For transmit endpoints, transmitting incomplete packets principally applies when the interface is in peripheral mode. It occurs when the transmission fails to receive enough IN tokens from the host to send all the parts of the data packet. It can also apply to high-bandwidth interrupt transactions in host mode where the core does not receive any response from the device to which the packet is transmitted. In both cases, the USB\_EP[n]\_TXCSR\_P.INCOMPTX bit is set.

For receive endpoints, an incomplete packet issue can occur. The PIDs of the received parts of the data packet show that one or more parts of the data packet have not been received. When this event happens, the USB\_EP[n]\_RXCSR\_P.INCOMPRX bit is set. Usually this bit is set in peripheral mode. However, it can also be set in host mode (using the USB\_EP[n]\_RXCSR\_H.INCOMPRX bit). This event occurs when the USB communicates with a device that fails to respond in accordance with the USB protocol.

## High Bandwidth Isochronous or Interrupt IN Endpoints

In high-speed mode, transmit endpoints configured for high-bandwidth isochronous or interrupt transactions can transmit up to three USB packets in any micro-frame. The transmission occurs with a payload of up to 1024 bytes in each packet, corresponding to a data transfer rate of up to 3072 bytes per micro-frame.

The High Bandwidth IN Endpoints figure provides an overview of high-bandwidth IN endpoints in USB.

Figure 27-2: High Bandwidth IN Endpoints

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000001_79c81949837bd4b901bb073bf5c306486c3cf7d2b99a9b37531d95a803831329.png)

The USB controller supports these transfers by permitting the loading of data packets with up to three times the normal packet size into the associated FIFO in a single transaction. From the software viewpoint in the processor core, the High Bandwidth IN Endpoints figure describes the operation for single or double packet buffering (as appropriate). One exception is that the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit must always be set manually (for example by the processor core) as the auto set feature does not operate with high-bandwidth isochronous transfers.

The USB controller automatically splits any data packet loaded into the FIFO that is larger than the maximum into USB packets of the maximum payload, or smaller, for transmission. The following settings define the number of USB packets transmitted per micro-frame and the maximum payload in each packet:

- Use the USB\_EP[n]\_TXMAXP.MAXPAY bits to set the maximum payload in any USB packet
- Use the USB\_EP[n]\_TXMAXP.MULTM1 bits to set the maximum number of such packets for transmission in one micro-frame (2 or 3)

Together, these settings define the maximum size of packet that can be loaded into the FIFO.

At least one USB packet always is sent. The number of further USB packets sent in the same micro-frame depends on the amount of data loaded into the FIFO. The USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is cleared and an interrupt is generated only when all the packets have been sent. Each USB packet is sent in response to an IN token. If, at the end of a micro-frame, the USB controller has not received enough IN tokens to send all the USB packets, the remaining data is flushed from the FIFO. (For example, one of the IN tokens received is corrupt). The USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is cleared and the USB\_EP[n]\_TXCSR\_P.INCOMPTX bit is set to indicate that not all of the data loaded into the FIFO transmitted.

## High-Bandwidth Isochronous or Interrupt OUT Endpoints

In high-speed mode, isochronous receive endpoints can receive up to three USB packets in any micro-frame. The reception occurs with a payload of up to 1024 bytes in each packet, corresponding to a data transfer rate of up to 3072 bytes per micro-frame. Similarly, the USB controller can receive high-bandwidth interrupt transactions in host mode, but there is no support for high-bandwidth interrupt transactions in peripheral mode.

The High-Bandwidth OUT Endpoints figure shows an overview of high-bandwidth OUT endpoints.

The USB controller supports this rate by automatically combining all the USB packets received during a microframe into a single packet of up to 3 normal packets within the receive FIFO. From the software viewpoint in the processor core, the High Bandwidth IN Endpoints figure describes the operation for single or double packet buffering (as appropriate). One exception is that the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit always must be cleared manually (for example by the processor core) because the auto-clear function does not operate with high-bandwidth isochronous transfers.

The maximum number of USB packets that can be received in any micro-frame and the maximum payload of these packets are configured as follows:

- Use the USB\_EP[n]\_RXMAXP.MAXPAY bits to set the maximum payload in any USB packet
- Use the USB\_EP[n]\_RXMAXP.MULTM1 bits to set the maximum number of these packets that can be received in a micro-frame (2 or 3)

Figure 27-3: High-Bandwidth OUT Endpoints

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000002_31cb747615d6acaff67d37d2f1d66a4f716431b8f5ec7421a2eb3943f5438b50.png)

The number of USB packets sent in any micro-frame depends on the amount of data for transfer, and is indicated through the PIDs used for the individual packets. If the indicated number of packets have not been received by the end of a micro-frame, the USB\_EP[n]\_RXCSR\_P.INCOMPRX bit is set to indicate that the data in the FIFO is incomplete. An interrupt is still generated to allow the data that has been received to be read from the FIFO.

## Peripheral Transfer Work Flows

The USB transfer types (control, bulk, isochronous, and interrupt transfers) have different system requirements as well as individual USB transfer-specific features. Software handles each type differently. There is no uniform way of doing transfers across all transfer types using the USB controller.

The following sections provide some guidelines for peripheral mode transfer flows for each of the transfer types, in both IN (transmit) and OUT (receive) directions. For bulk endpoints, the optimal transfer flow depends on whether the final size of the transfer is known or unknown. The USB driver class in use determines whether the transfer size is known or not. Some drivers define the complete transfer size, and others operate on a packet-by-packet basis using a short packet to denote the end of a transfer. (A short packet is less than the value configured in the

USB\_EP[n]\_TXMAXP register or less than the value configured in the USB\_EP[n]\_RXMAXP register.)

Each of the work flows uses the following common steps.

1. Configure the endpoint control and status registers and the USB\_EP[n]\_TXMAXP or USB\_EP[n]\_RXMAXP value.
2. Configure the appropriate data transfer mechanism (DMA or interrupt setup).
3. Data transfer occurs.

The work flows do not describe the actions of the USB controller immediately preceding the endpoint setup. (For example, the reception of an IN/OUT token from the host, token validity checking, or NAK generation, among others.) Note also that there is no error-handling contained in the work flows (for example, checking the USB\_EP[n]\_RXCSR\_P.FIFOFULL bit before writing data).

The proceeding sections use terms packets, frames, and transfers with their strict USB definitions (see USB Definitions).

## Control Transactions as a Peripheral

Endpoint 0 is the main control endpoint of the USB controller. As such, the routines required to service endpoint 0 are more complicated than the routines required to service other endpoints.

The software is required to handle all the standard device requests that the USB controller sends or receives through endpoint 0. The Universal Serial Bus Specification , Revision 2.0, Chapter 9 describes the requirements. The protocol for these device requests involves different numbers and types of transactions per transfer. To accommodate this functionality, the processor must take a state machine approach to command decoding and handling.

The standard device requests a USB peripheral receives fits into three categories:

- Zero data requests (in which the command includes all the information)
- Write requests (in which extra data follows the command)
- Read requests (in which the device sends data back to the host)

The following sections describe the sequence of actions that the software must perform to process these different types of device request.

## Write Requests

The host sends an 8-byte command followed by a write request that contains an extra packet (or packets) of data. An example of a write standard device request is SET\_DESCRIPTOR .

As with all requests, the sequence of events begins when the software receives an endpoint 0 interrupt. The USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit is also set. The host then reads and decodes the 8-byte command from the endpoint 0 FIFO.

As with a zero data request, write to the USB\_EP0\_CSR[n]\_P register to set the USB\_EP0\_CSR[n]\_P.SPKTRDY bit. (The event indicates that the host read the command from the FIFO.) But, in this case, do not set the USB\_EP0\_CSR[n]\_P.DATAEND bit (indicating that more data is expected).

When a second endpoint 0 interrupt is received, the USB\_EP0\_CSR[n]\_P register is read to check the endpoint status. The USB\_EP0\_CSR[n]\_P.RXPKTRDY bit is set to indicate that a data packet is received. Read the USB\_EP0\_CNT[n] register to determine the size of this data packet. The data packet can then be read from the endpoint 0 FIFO.

If the length of the data associated with the request is greater than the maximum packet size for endpoint 0, the host sends more data packets. (The WLENGTH field in the command indicates the length of the data.) In this case, the USB\_EP0\_CSR[n]\_P.SPKTRDY bit is set, but do not set the USB\_EP0\_CSR[n]\_P.DATAEND bit.

When all the expected data packets have been received, software writes to the USB\_EP0\_CSR[n]\_P register to set the USB\_EP0\_CSR[n]\_P.SPKTRDY bit and to set the USB\_EP0\_CSR[n]\_P.DATAEND bit (indicating that no more data is expected).

When the host moves to the status stage of the request, software generates another endpoint 0 interrupt to indicate that the request has completed. No further action is required from the software; the interrupt is just a confirmation that the request completed successfully.

If the command is an unrecognized command, or cannot be executed, then when the host decodes it, software must write to the USB\_EP0\_CSR[n]\_P register. This operation sets the USB\_EP0\_CSR[n]\_P.SPKTRDY bit and the USB\_EP0\_CSR[n]\_P.SENDSTALL bit. When the host sends more data, the USB controller sends a stall to tell the host that the request was not executed. Software generates an endpoint 0 interrupt and the USB\_EP0\_CSR[n]\_P.SENTSTALL bit is set.

If the host sends more data after the USB\_EP0\_CSR[n]\_P.DATAEND has been set, then the USB controller sends a stall. Software generates an endpoint 0 interrupt and the USB\_EP0\_CSR[n]\_P.SENTSTALL bit is set.

## Read Requests

The function sends the 8-byte command followed by read requests containing a packet (or packets) of data to the host. Examples of standard device requests for read are:

- GET\_CONFIGURATION
- GET\_INTERFACE
- GET\_DESCRIPTOR
- GET\_STATUS
- SYNCH\_FRAME

As with all requests, the sequence of events begins when the software receives an endpoint 0 interrupt. The USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit is also set. The host then reads and decodes the 8-byte command from the endpoint 0 FIFO. Write the USB\_EP0\_CSR[n]\_P.SPKTRDY bit (indicating that the command has been read from the FIFO).

The data to transmit to the host is written to the endpoint 0 FIFO. If the size of the transmit data is greater than the maximum packet size for endpoint 0, only the maximum packet size is written to the FIFO. The USB\_EP0\_CSR[n]\_P.TXPKTRDY bit is then set (indicating that there is a packet in the FIFO to be sent). When the packet has been sent to the host, software generates another endpoint 0 interrupt. The next data packet can be written to the FIFO.

When the last data packet has been written to the FIFO, software writes to the USB\_EP0\_CSR[n]\_P register to set the USB\_EP0\_CSR[n]\_P.TXPKTRDY bit and to set the USB\_EP0\_CSR[n]\_P.DATAEND bit. (This activity indicates that there is no more data after this packet.)

When the host moves to the status stage of the request, software generates another endpoint 0 interrupt to indicate that the request has completed. No further action is required from the software; the interrupt is just a confirmation that the request completed successfully.

If the command is an unrecognized command, or for some other reason cannot be executed, then when the host decodes it, software must write to the USB\_EP0\_CSR[n]\_P register. This operation sets the USB\_EP0\_CSR[n]\_P.SPKTRDY bit and the USB\_EP0\_CSR[n]\_P.SENDSTALL bit. When the host requests data, the USB controller sends a stall to tell the host that the request was not executed. Software generates an endpoint 0 interrupt and the USB\_EP0\_CSR[n]\_P.SENTSTALL bit is set.

If the host requests more data after USB\_EP0\_CSR[n]\_P.DATAEND is set, then the USB controller sends a stall. Software generates an endpoint 0 interrupt and the USB\_EP0\_CSR[n]\_P.SENTSTALL bit is set.

## Zero Data Requests

Zero data requests have all their information included in the 8-byte command and do not require transfer of extra data.

Examples of standard device requests for zero data are:

- SET\_FEATURE
- CLEAR\_FEATURE
- SET\_ADDRESS
- SET\_CONFIGURATION
- SET\_INTERFACE

As with all requests, the sequence of events begins when the software receives an endpoint 0 interrupt. The USB\_EP0\_CSR[n]\_P.RXPKTRDY bit is also set. The host must then read and decode the 8-byte command from the endpoint 0 FIFO, and take appropriate action. For example, if the command is SET\_FEATURE , the host writes 7-bit address value contained in the command to the USB\_FADDR register.

Software must set the USB\_EP0\_CSR[n]\_P.SPKTRDY bit (indicating that the command has been read from the FIFO) and the USB\_EP0\_CSR[n]\_P.DATAEND bit (indicating that no further data is expected for this request).

When the host moves to the status stage of the request, the USB controller generates a second endpoint 0 interrupt, indicating that the request has completed. No further action is required from the software; the second interrupt is just a confirmation that the request completed successfully.

If the command is an unrecognized command, or for some other reason cannot be executed, then when the host decodes it, the USB\_EP0\_CSR[n]\_P.SPKTRDY bit is set which sets the USB\_EP0\_CSR[n]\_P.SENDSTALL bit. When the host moves to the status stage of the request, the USB controller sends a stall to tell the host that the request was not executed. The USB controller generates a second endpoint 0 interrupt and sets the USB\_EP0\_CSR[n]\_P.SENTSTALL bit.

If the host sends more data after the USB\_EP0\_CSR[n]\_P.DATAEND bit is set, then the USB controller sends a stall. It generates an endpoint 0 interrupt and sets the USB\_EP0\_CSR[n]\_P.SENTSTALL bit.

## Endpoint 0 States

When the USB operates as a peripheral, the endpoint 0 control has three modes (IDLE, Tx, and Rx). The modes correspond to the phases of the control transfer and the state that endpoint 0 enters during phases of the transfer. (See Endpoint 0 Service Routine as Peripheral.)

IDLE is the default mode on power-up or reset. The processor sets RxPktRdy bit when endpoint 0 is in IDLE state, indicating a new device request. Once the processor unloads the device request from the FIFO, the USB decodes the descriptor. It determines whether there is a data phase and, if so, the direction of the data phase of the control transfer (to set the FIFO direction).

Depending on the direction of the data phase, endpoint 0 goes into either Tx state or Rx state. If there is no data phase, endpoint 0 remains in IDLE state to accept the next device request.

The processor must take different actions at the different phases of the possible transfers (for example, loading the FIFO, setting TxPktRdy ). The Endpoint 0 Control States figure shows the actions for the phase. The USB changes the FIFO direction depending on the direction of the data phase, independently of the processor.

Figure 27-4: Endpoint 0 Control States

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000003_03212608b901f78b252b3095f4dae89bdb1ef43f918bf0c6d9698d39fcde5f7a.png)

## Endpoint 0 Service Routine as Peripheral

The USB controller generates an endpoint 0 interrupt when:

- The USB controller sets the USB\_EP0\_CSR[n]\_P.RXPKTRDY bit after a valid token has been received and data has been written to the FIFO.

- The USB controller clears the USB\_EP0\_CSR[n]\_P.TXPKTRDY bit after the data packet in the FIFO has been successfully transmitted to the host.
- The USB controller sets the USB\_EP0\_CSR[n]\_P.SENTSTALL bit after a control transaction is ended due to a protocol violation.
- The USB controller sets the USB\_EP0\_CSR[n]\_P.SETUPEND bit because a control transfer has ended before USB\_EP0\_CSR[n]\_P.DATAEND is set.

Whenever the endpoint 0 service routine is entered, the firmware must first check whether the current control transfer has been ended. The transfer can end due to either a stall condition or a premature end-of-control transfer. If the control transfer ends due to a stall condition, the USB controller sets the USB\_EP0\_CSR[n]\_P.SENTSTALL bit. If the control transfer ends due to a premature end-of-control transfer, the USB controller sets USB\_EP0\_CSR[n]\_P.SETUPEND . In either case, the firmware must abort processing the current control transfer and set the state to IDLE.

Once the firmware has determined that an illegal bus state did not generate the interrupt, the next action depends on the endpoint state.

If endpoint 0 is in IDLE state, the only valid reason the USB controller can generate an interrupt is due to the core receiving data from the USB bus. The service routine must check for this state by testing the USB\_EP0\_CSR[n]\_P.RXPKTRDY bit. If the USB controller sets the bit, then the core has received a setup packet. The processor unloads this packet from the FIFO and decodes it to determine the next action. Depending on the command contained within the setup packet, endpoint 0 enters one of the following three states.

- If the command is a single packet transaction ( SET\_ADDRESS , SET\_INTERFACE and the others) without a data phase, the endpoint remains in the IDLE state.
- If the command has an OUT data phase ( SET\_DESCRIPTOR and others), the endpoint enters the Rx state.
- If the command has an IN data phase ( GET\_DESCRIPTOR and others), the endpoint enters the Tx state.

NOTE: Command transactions all include a field that indicates the amount of data the host expects to receive or send.

If the endpoint is in Tx state, the interrupt indicates that the core has received an IN token and data from the FIFO has been sent. The firmware must respond to this event by:

- Placing more data in the FIFO when the host still expects more data
- Setting the USB\_EP0\_CSR[n]\_P.DATAEND bit to indicate that the data phase is complete

Once the data phase of the transaction completes, endpoint 0 returns to the IDLE state to await the next control transaction.

If the endpoint is in the Rx state, the interrupt indicates that a data packet has been received. The firmware must respond by unloading the received data from the FIFO. The firmware must then determine whether it has received

all of the expected data. If it has, the firmware must set the USB\_EP0\_CSR[n]\_P.DATAEND bit and return endpoint 0 to IDLE state. If more data is expected, the firmware must set the USB\_EP0\_CSR[n]\_P.SPKTRDY bit to indicate that it has read the data in the FIFO and leave the endpoint in the Rx state.

Figure 27-5: Endpoint 0 Service Routine

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000004_c932e07fffbe9dce9601c09bb2439f654e307a6e3076e083ea2a0c7d7f867694.png)

## IDLE Mode

The endpoint 0 control must select IDLE mode at power-on or reset. The endpoint 0 control returns to this mode when the Rx and Tx modes terminate.

As shown in the Endpoint 0 Idle Mode (SETUP Phase) figure, the SETUP phase of control transfer is handled in IDLE mode.

Figure 27-6: Endpoint 0 Idle Mode (SETUP Phase)

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000005_6c0e074e66d65c0408cf647c83934817d5ed196e58ae121b13eb01667ed0a165.png)

## Tx Mode

Refer to the Endpoint 0 Tx Mode figure. When the endpoint is in Tx state, all arriving IN tokens must be treated as part of a data phase until the required amount of data has been sent to the host. If either a SETUP or an OUT token is received while the endpoint is in the Tx state, a USB\_EP0\_CSR[n]\_P.SETUPEND condition occurs. The core expects only IN tokens.

Three events can cause the Tx mode to terminate before the expected amount of data has been sent:

- The host sends an invalid token which sets the USB\_EP0\_CSR[n]\_P.SETUPEND bit.
- The firmware sends a packet containing less than the maximum packet size for endpoint 0.
- The firmware sends an empty data packet.

Until the transaction is terminated, when the firmware receives an interrupt which indicates that a packet has been sent from the FIFO, it simply loads the FIFO. An interrupt is generated when the USB controller clears USB\_EP0\_CSR[n]\_P.TXPKTRDY .

When the firmware forces the termination of a transfer (by sending a short or empty data packet), it must set the USB\_EP0\_CSR[n]\_P.DATAEND bit. This event indicates to the core that the data phase is complete and that the core will receive an acknowledge packet next.

Figure 27-7: Endpoint 0 Tx Mode

## Rx Mode

Refer to the Endpoint 0 Rx Mode figure. In Rx mode, all arriving data must be treated as part of a data phase until the expected amount of data is received. If either a SETUP or an IN token is received while the endpoint is in Rx state, a USB\_EP0\_CSR[n]\_P.SETUPEND condition occurs since the core expects only OUT tokens.

Three events can cause the Rx mode to terminate before the expected amount of data is received:

- The host sends an invalid token which sets the USB\_EP0\_CSR[n]\_P.SETUPEND bit.
- The host sends a packet which contains less than the maximum packet size for endpoint 0.
- The host sends an empty data packet.

The transaction terminates when the firmware receives an interrupt which indicates that new data has arrived ( USB\_EP0\_CSR[n]\_P.RXPKTRDY bit is set). Until the transaction terminates, firmware must unload the FIFO and clear USB\_EP0\_CSR[n]\_P.RXPKTRDY by setting the USB\_EP0\_CSR[n]\_P.SPKTRDY bit.

When the firmware detects the termination of a transfer (by receiving either the expected amount of data or an empty data packet), it must set the USB\_EP0\_CSR[n]\_P.DATAEND bit. This event indicates to the core that the data phase is complete and that the core will receive an acknowledge packet next.

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000006_d3dbcbf6f38ccab611fbd43caf9b5916c60dd2996c4892d6f6d8f0310e1248d0.png)

Figure 27-8: Endpoint 0 Rx Mode

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000007_1975e87b02185d98fc6744ba98601890bf50166462c0a8c2b3df5aa6e6f1a29f.png)

## Peripheral Mode, Bulk IN, Transfer Size Known

For this process, the maximum size of an individual packet ( MaxPktSize ) in bytes and the complete transfer size ( TxferSize ) in bytes, must be known.

1. Load MaxPktSize into the USB\_EP[n]\_TXMAXP register.
2. Set the following bits: USB\_EP[n]\_TXCSR\_P.DMAREQEN = 1, USB\_EP[n]\_TXCSR\_P.AUTOSET = 1, USB\_EP[n]\_TXCSR\_P.ISO = 0, USB\_EP[n]\_TXCSR\_P.FRCDATATGL = 0.
3. Load the TxferSize value into the USB\_DMA[n]\_CNT register.
4. Configure the DMA controller to write the data into the corresponding Tx FIFO address.
5. On each USB\_DMA[n]\_CNT transition, the DMA controller writes a new packet into the FIFO. The USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is automatically set when each new packet is written.
6. ADDITIONAL INFORMATION: Repeat Step 5 for each full packet of the transfer. Even if the final packet is a short packet, the USB controller automatically detects the packet because the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is set.

## Peripheral Mode, Bulk IN, Transfer Size Unknown

For this process, assume the maximum individual packet size ( MaxPktSize ) in bytes is an even number of bytes.

1. Load MaxPktSize into the USB\_EP[n]\_TXMAXP register.
2. Set the following bits: USB\_EP[n]\_TXCSR\_P.DMAREQEN = 1, USB\_EP[n]\_TXCSR\_P.AUTOSET = 1, USB\_EP[n]\_TXCSR\_P.ISO = 0, USB\_EP[n]\_TXCSR\_P.FRCDATATGL = 0.

3. Configure the DMA controller to write MaxPktSize /2 half words into the corresponding Tx FIFO address on each USB\_DMA[n]\_CNT .
4. Set up an ISR, sensitive to the DMA work-block-complete interrupt, that writes a remaining short packet into the Tx FIFO using processor core DMA. Then, set the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit or toggle the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit to send a zero-length packet.
5. On each USB\_DMA[n]\_CNT transition, the DMA controller writes a new packet into the FIFO. The USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit is set automatically when each new packet is written.
4. ADDITIONAL INFORMATION: Repeat step 5 for each full packet of the transfer. The ISR from step 4 manages the final short or zero-length packet.

## Peripheral Mode, ISO IN, Small MaxPktSize

For this process, the maximum individual packet size ( MaxPktSize ) in bytes is less than 128 bytes and is an even number of bytes. Assume that double buffering is enabled, and the auto-set feature is not used (because packets are often less than MaxPktSize ).

1. Load MaxPktSize into the USB\_EP[n]\_TXMAXP register.
2. Set the following bits: USB\_EP[n]\_TXCSR\_P.ISO = 1.
3. Preload the first two packets into the endpoint Tx FIFO and set the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit.
4. Set up an ISR, sensitive to the USB\_IRQ.SOF interrupt, which writes a new packet into the Tx FIFO and sets the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit.
5. Set the USB\_IEN.SOF bit = 1 to generate an interrupt on each start of frame.

ADDITIONAL INFORMATION: Repeat step 5 for each ISO packet.

## Peripheral Mode, ISO IN, Large MaxPktSize

For this process, the maximum individual packet size ( MaxPktSize ) in bytes is greater than 128 bytes and is an even number of bytes. Assume that double buffering is enabled, and the auto-set feature is not used (because packets are often less than MaxPktSize ).

1. Load MaxPktSize into the USB\_EP[n]\_TXMAXP register.
2. Set the USB\_EP[n]\_TXCSR\_P.ISO bit = 1.
3. Set the USB\_POWER.ISOUPDT bit = 1 to prevent the initial packet loaded into the FIFO from transmitting on the USB until the next 1ms frame.
4. Load the total number of bytes for the first two packets into the USB\_DMA[n]\_CNT register.
5. Configure the DMA controller to pre-load the two packets into the corresponding Tx FIFO address and set the USB\_EP[n]\_TXCSR\_P.TXPKTRDY bit.

6. Set up an ISR, sensitive to the USB\_IRQ.SOF interrupt, which writes a new packet into the Tx FIFO by configuring the DMA controller to load the packet.
7. Set the USB\_IEN.SOF bit = 1 to generate an interrupt on each start of frame.

ADDITIONAL INFORMATION: Repeat step 7 for each ISO packet.

## Peripheral Mode, Bulk OUT, Transfer Size Known

For this process, the maximum individual packet size ( MaxPktSize ) in bytes and the complete transfer size ( TxferSize ) in bytes must be known.

1. Load MaxPktSize into USB\_EP[n]\_RXMAXP .
2. Set the following bits: USB\_EP[n]\_RXCSR\_P.DMAREQEN = 1, USB\_EP[n]\_RXCSR\_P.AUTOCLR = 1, USB\_EP[n]\_RXCSR\_P.ISO = 0, USB\_EP[n]\_RXCSR\_P.CLRDATATGL = 0, USB\_EP[n]\_RXCSR\_P.DMAREQMODE = 0.
3. Configure the DMA controller to read the full TxferSize /2 half words from the corresponding Rx FIFO address.
4. On each USB\_DMA[n]\_CNT transition, the DMA controller reads another packet from the FIFO. The USB controller automatically clears the bit when each new packet is read.
5. USB\_EP[n]\_RXCSR\_P.RXPKTRDY ADDITIONAL INFORMATION: Repeat step 5 for each full packet of the transfer. If TxferSize is not an exact multiple of MaxPktSize , the final USB\_DMA[n]\_CNT transition causes the DMA controller to read out only the short packet that remains.

## Peripheral Mode, Bulk OUT, Transfer Size Unknown

For this process, the maximum individual packet size ( MaxPktSize ) in bytes must be known.

1. Load MaxPktSize into USB\_EP[n]\_RXMAXP .
2. Set the following bits: USB\_EP[n]\_RXCSR\_P.DMAREQEN = 1, USB\_EP[n]\_RXCSR\_P.AUTOCLR = 1, USB\_EP[n]\_RXCSR\_P.ISO = 0, USB\_EP[n]\_RXCSR\_P.CLRDATATGL = 0, USB\_EP[n]\_RXCSR\_P.DMAREQMODE = 1.
3. Set the appropriate bit in the USB\_INTRRXE register.
4. Configure the DMA controller to read MaxPktSize /2 half words from the corresponding Rx FIFO address on each USB\_DMA[n]\_CNT transition.
5. Set up an ISR, sensitive to the Rx interrupt, which reads the USB\_EP[n]\_RXCNT register and then transfers USB\_EP[n]\_RXCNT bytes (in half words) from the Rx FIFO to the processor core.
6. ADDITIONAL INFORMATION: Depending on the number of bytes in the FIFO, configure the DMA to read the data, or read it with the processor core.

ADDITIONAL INFORMATION: On each USB\_DMA[n]\_CNT transition, the DMA controller reads a packet from the FIFO. The USB controller automatically clears the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit when each new packet is read.

ADDITIONAL INFORMATION: Repeat step 5 for each full packet of the transfer.

6. If a packet is received that is less than MaxPktSize , the Rx interrupt goes high, and the ISR from step 5 reads out the remaining short packet.

## Peripheral Mode, ISO OUT, Small MaxPktSize

For this process, the maximum individual packet size ( MaxPktSize ) in bytes is less than 128 bytes. Assume that double buffering is enabled.

1. Load the MaxPktSize value into the USB\_EP[n]\_RXMAXP register.
2. Set the USB\_EP[n]\_RXCSR\_P.ISO bit = 1.
3. Set up an ISR, sensitive to the USB\_IRQ.SOF interrupt, that reads the USB\_EP[n]\_RXCSR\_P.FIFOFULL bit and then reads the USB\_EP0\_CNT[n].RXCNT status register. The ISR removes one or two packets (equal to the USB\_EP0\_CNT[n].RXCNT number of bytes) from the FIFO. It then clears the USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit.
4. Set the USB\_IEN.SOF bit = 1 to generate an interrupt on each start of frame.

ADDITIONAL INFORMATION: Repeat step 4 for each ISO packet.

## Peripheral Mode, ISO OUT, Large MaxPktSize

For this process, the maximum individual packet size ( MaxPktSize ) in bytes is greater than 128 bytes. Assume that double buffering is enabled.

1. Load MaxPktSize into the USB\_EP[n]\_RXMAXP register.
2. Set the USB\_EP[n]\_RXCSR\_P.ISO bit = 1.
3. Set up an ISR (sensitive to the USB\_IRQ.SOF interrupt), that reads the USB\_EP[n]\_RXCSR\_P.FIFOFULL bit, and then reads the USB\_EP[n]\_RXCNT status register. The ISR configures the DMA controller to remove one or two packets (equal to the USB\_EP[n]\_RXCNT number of bytes) from the FIFO.
4. Set up an ISR, sensitive to the DMA work-block-complete interrupt to clear the USB\_EP[n]\_RXCSR\_P.RXPKTRDY .
5. Set the USB\_IEN.SOF bit = 1 to generate an interrupt on each start of frame.

ADDITIONAL INFORMATION: Repeat step 5 for each ISO packet.

## Peripheral Mode Suspend

When no activity has occurred on the USB for 3 ms, the USB controller enters suspend mode. If the suspend interrupt ( USB\_IRQ.SUSPEND ) is enabled, an interrupt is generated.

When resume signaling is detected, the USB controller exits suspend mode. If the USB\_IRQ.RESUME interrupt is enabled, an interrupt is generated. The processor core can also force the USB controller to exit suspend mode by setting the USB\_POWER.RESUME bit. This event initiates a remote wake-up. When this bit is set, the USB controller exits suspend mode and drives resume signaling onto the bus. The processor core must clear this bit after 10 ms (a maximum of 15 ms) to end resume-signaling.

NOTE: The USB\_IRQ.RESUME interrupt is not generated when the processor core exits suspend mode. This interrupt is not generated when the software initiates remote wake-up.

## Start of Frame (SOF) Packets

When the USB controller operates in peripheral mode, it receives a start of frame packet from the host every millisecond when in full-speed mode, or every 125 microseconds when in high-speed mode.

When the USB controller receives a SOF packet, it writes the 11-bit frame number contained in the packet into the USB\_FRAME register. An output pulse, lasting one USB clock bit period, is generated. A start of frame interrupt is also generated (if enabled by the USB\_IRQ.SOF bit).

After the USB controller has started to receive SOF packets, the controller expects one every millisecond (faster in high-speed mode). If the USB controller does not receive an SOF packet after 1.00358 ms (faster in high-speed mode), it is assumed that the packet is lost. A start of frame pulse (together with a USB\_IRQ.SOF interrupt) is still generated even though the USB\_FRAME register is not updated. The USB controller continues to generate an SOF pulse every millisecond (faster in high-speed mode). It resynchronizes these pulses to the received SOF packets when it successfully receives these packets again.

## Soft Connect/Soft Disconnect

In peripheral mode, the USB controller sets or clears the USB\_POWER.SOFTCONN bit to switch between normal mode and non-driving mode. When USB\_POWER.SOFTCONN =1, the USB controller is in normal mode and the D+/D- lines of the USB bus are enabled. When the USB\_POWER.SOFTCONN =0, the PHY is put into non-driving mode and D+ and D- are three-stated. The USB controller appears to have been disconnected from the USB bus.

After system reset, USB\_POWER.SOFTCONN =0. From that point, the USB controller appears disconnected until the software has set USB\_POWER.SOFTCONN =1. The application software can then choose when to set the PHY to its normal mode. Systems with a lengthy initialization procedure can use this functionality to ensure that initialization is complete and the system is ready to perform enumeration before connecting to the USB. Once the USB\_POWER.SOFTCONN bit is set to 1, the software can also clear this bit to 0 to simulate a disconnect.

## Error Handling As a Peripheral

The host can abort a control transfer due to a protocol error on the USB. The function controller software can also abort the transfer (for example, because it cannot process the command).

The USB controller automatically detects protocol errors and sends a stall packet to the host under the following conditions.

1. The host sends more data during the OUT data phase of a write request than specified in the command. This condition is detected when the host sends an OUT token after the USB\_EP0\_CSR[n]\_P.DATAEND bit is set.
2. The host requests more data during the IN data phase of a read request than specified in the command. This condition is detected when the host sends an IN token after the USB\_EP0\_CSR[n]\_P.DATAEND bit is set.
3. The host sends more than MaxPktSize data bytes in an OUT data packet.
4. The host sends a non-zero length DATA1 packet during the status phase of a read request.

When the USB controller has sent the stall packet, it sets the USB\_EP0\_CSR[n]\_P.SENTSTALL bit and generates an interrupt. When the software receives an endpoint 0 interrupt with the USB\_EP0\_CSR[n]\_P.SENTSTALL bit set, it aborts the current transfer, clears the USB\_EP0\_CSR[n]\_P.SENTSTALL bit, and returns to the IDLE state.

If the host enters the status phase before all the data for the request transfers, or sends a new SETUP packet before completing the current transfer, then it prematurely ends the transfer. The USB\_EP0\_CSR[n]\_P.SETUPEND bit is set and an endpoint 0 interrupt is generated. When the software receives an endpoint 0 interrupt with the USB\_EP0\_CSR[n]\_P.SETUPEND bit set, it aborts the current transfer, sets the USB\_EP0\_CSR[n]\_P.SSETUPEND bit, and returns to the IDLE state. If the USB\_EP0\_CSR[n]\_P.RXPKTRDY bit is set, it indicates that the host has sent another SETUP packet and the software must then process this command.

If the software wants to abort the current transfer, because it cannot process the command or has some other internal error, then it must set the USB\_EP0\_CSR[n]\_P.SENTSTALL bit. The USB controller then sends a stall packet to the host, sets the USB\_EP0\_CSR[n]\_P.SENTSTALL bit, and generates an endpoint 0 interrupt.

## Stalls Issued to Control Transfers

In peripheral mode, the USB controller automatically issues a stall handshake to a control transfer under the following conditions:

1. The host sends more data during an OUT data phase of a control transfer than specified in the device request during the SETUP phase. The USB controller detects this condition when the host sends an OUT token (instead of an IN token) after the processor core unloads the last OUT packet and sets the USB\_EP0\_CSR[n]\_P.DATAEND bit.
2. The host requests more data during an IN data phase of a control transfer than specified in the device request during the SETUP phase. The USB controller detects this condition when the host sends an IN token (instead of an OUT token) after the processor core clears USB\_EP[n]\_TXCSR\_P.TXPKTRDY and sets USB\_EP0\_CSR[n]\_P.DATAEND . The processor sets USB\_EP0\_CSR[n]\_P.DATAEND in response to the host-issued ACK for the last packet.
3. The host sends more than MaxPktSize data with an OUT data token.

4. The host sends the wrong PID (packet identifier) for the OUT status phase of a control transfer.
5. The host sends more than a zero length data packet for the OUT status phase.

## Zero Length OUT Data Packets in Control Transfers

The USB controller uses a zero-length OUT data packet to indicate the end of a control transfer. In normal operation, such packets must only be received after the entire length of the device request transfers (for example, after the processor core has set the USB\_EP0\_CSR[n]\_P.DATAEND bit). If the host sends a zero-length OUT data packet before the entire length of device request transfers, this packet signals the premature end of the transfer. In this case, the USB controller automatically flushes any IN token the processor core has loaded for the data phase from the FIFO and sets the USB\_EP0\_CSR[n]\_P.SETUPEND bit.

## Host Mode

USB OTG interface operations in host mode differ from peripheral mode in a number of ways. The following sections describe host mode operations.

## Transaction Scheduling

When operating as a host, the USB controller maintains a frame counter.

If the target function is a full-speed device, the USB controller automatically sends an SOF packet at the start of each frame or micro-frame.

If the target function is a low-speed device, a K state is transmitted on the bus to act as a keep-alive . It stops the lowspeed device from going into suspend mode.

After the SOF packet is transmitted, the USB controller cycles through all the endpoints looking for active transactions. An active transaction is defined as an Rx endpoint for which the USB\_EP[n]\_RXCSR\_H.REQPKT bit is set or a Tx endpoint for which the USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit is set.

An active isochronous or interrupt transaction only starts if:

- It is found on the first transaction scheduler cycle of a frame.
- The interval counter for that endpoint has counted down to zero.

This functionality ensures that only one interrupt or isochronous transaction occurs per endpoint per n frames or micro frames (or, up to three, if high-bandwidth support is selected). n is the interval set in the USB\_EP[n]\_TXINTERVAL or USB\_EP[n]\_RXINTERVAL register for that endpoint.

An active bulk transaction starts immediately, provided there is sufficient time left in the frame to complete the transaction before the next SOF packet is due. If the transaction must be retried, then it is not retried until the transaction scheduler has checked all the other endpoints for active transactions first. (For example, the transaction is retried because a NAK was received or the target function did not respond.) This check ensures that an endpoint that is sending many NAKs does not block other transactions on the bus. The USB controller permits specifying a limit ( USB\_EP[n]\_TXINTERVAL or USB\_EP[n]\_RXINTERVAL registers) to the length of time in which NAKs can be received from a particular target before the endpoint is timed out.

## Endpoint Setup and Data Transfer

When the HOST\_MODE bit is set to 1, the USB controller operates as a host for point-to-point communications with another USB device. Or, when attached to a hub, the USB controller operates as a host for communication with a range of devices in a multi-point set-up.

The USB controller supports high-speed, full-speed, and low-speed USB functions, both for point-to-point communication and for operation through a hub.

Where necessary, the core automatically carries out the necessary transaction translation to allow usage of a lowspeed or full-speed device with a USB 2.0 hub.

The USB controller supports control, bulk, isochronous, or interrupt transactions.

Transfers between the subsystem and endpoint FIFOs in host mode are similar to peripheral mode. See the descriptions of processor core-to-FIFO data transfer in Peripheral Mode.

## Control Transaction as a Host

Host control transactions are conducted through endpoint 0. The software handles all the standard device requests that are sent or received through endpoint 0 (as described in Universal Serial Bus Specification , Revision 2.0, Chapter 9).

For a USB peripheral, there are three categories of standard device requests:

- Zero data requests. Comprised of a SETUP command followed by an IN status phase. The command includes all the information.
- Write requests. Comprised of a SETUP command, followed by an OUT data phase followed by an IN status phase. Extra data follows the command.
- Read requests Comprised of a SETUP command, followed by an IN data phase followed by an OUT status phase. The device is required to send data back to the host.

A timeout can be set to limit the length of time during which the USB controller retries a transaction that the target continually NAKs. This limit can be between 2 and 2 15  frames or micro frames and is set through the USB\_EP0\_NAKLIMIT[n] register.

The following sections describe the steps taken in different phases of a control transaction and the actions of the core when issuing standard device requests.

## Set up Phase as a Host

The processor core driving the host device performs the following actions for the SETUP phase of a control transaction.

1. Load the 8 bytes of the required device request command into the endpoint 0 FIFO.
2. Set the USB\_EP0\_CSR[n]\_H.SETUPPKT bit and USB\_EP0\_CSR[n]\_H.TXPKTRDY bit. These bits must be set together.

- The USB controller then sends a SETUP token followed by the 8-byte command to endpoint 0 of the addressed device, retrying as necessary.
3. At the end of the attempt to send the data, the USB controller generates an endpoint 0 interrupt (for example, set USB\_INTRTXE.EP0 ). The processor core then reads the USB\_EP0\_CSR[n]\_H register to establish whether the USB\_EP0\_CSR[n]\_H.RXSTALL , USB\_EP0\_CSR[n]\_H.TOERR , or the USB\_EP0\_CSR[n]\_H.NAKTO bits are set.

If USB\_EP0\_CSR[n]\_H.RXSTALL =1, the target did not accept the command (for example, because the target device does not support it) and issues a stall response.

If USB\_EP0\_CSR[n]\_H.TOERR =1, the USB controller tried to send the SETUP packet and the following data packet three times without getting a response.

- If USB\_EP0\_CSR[n]\_H.NAKTO =1, the USB controller received a NAK response to each attempt to send the SETUP packet, for longer than the time set in the USB\_EP0\_NAKLIMIT[n] register. Direct the USB controller to either clear the USB\_EP0\_CSR[n]\_H.NAKTO bit to continue trying this transaction (until it times out again) or to flush the FIFO to abort the transaction before clearing the USB\_EP0\_CSR[n]\_H.NAKTO bit.
4. If none of USB\_EP0\_CSR[n]\_H.RXSTALL , USB\_EP0\_CSR[n]\_H.TOERR or USB\_EP0\_CSR[n]\_H.NAKTO bits are set, the SETUP phase is correctly acknowledged. The processor core can proceed to the following IN data phase, OUT data phase or IN status phase specified for the particular standard device request.

## IN Data Phase as a Host

The processor core driving the host device performs the following actions for the IN data phase of a control transaction.

1. Set the USB\_EP0\_CSR[n]\_H.REQPKT bit.
2. Wait while the USB controller sends the IN token and then receives the required data back.
3. When the USB controller generates the endpoint 0 interrupt (for example, by setting the USB\_INTRTXE.EP0 bit), read the USB\_EP0\_CSR[n]\_H register. Determine whether the USB\_EP0\_CSR[n]\_H.RXSTALL bit, the USB\_EP0\_CSR[n]\_H.TOERR bit, the USB\_EP0\_CSR[n]\_H.NAKTO bit, or the USB\_EP0\_CSR[n]\_H.RXPKTRDY bit is set.
4. If USB\_EP0\_CSR[n]\_H.RXSTALL =1, the target has issued a stall response.

If USB\_EP0\_CSR[n]\_H.TOERR =1, the USB controller has tried to send the required IN token three times without getting a response.

If USB\_EP0\_CSR[n]\_H.NAKTO =1, the USB controller has received a NAK response to each attempt to send the IN token, for longer than the time set in the USB\_EP0\_NAKLIMIT[n] register. Direct the USB controller to either clear the USB\_EP0\_CSR[n]\_H.NAKTO bit to continue trying this transaction (until it times out again) or clear USB\_EP0\_CSR[n]\_H.REQPKT before clearing the USB\_EP0\_CSR[n]\_H.NAKTO bit to abort the transaction.

4. If the USB\_EP0\_CSR[n]\_H.RXPKTRDY bit is set, the processor core reads the data from the endpoint 0 FIFO, then clears USB\_EP0\_CSR[n]\_H.RXPKTRDY .
5. If further data is expected, the processor core must repeat the previous steps.

When all the data is successfully received, the processor core can proceed to the OUT status phase of the control transaction.

## OUT Data as a Host (Control)

The processor core driving the host device performs the following actions for the OUT data phase of a control transaction.

1. Load the data to be sent into the endpoint 0 FIFO.
2. Set the USB\_EP0\_CSR[n]\_H.TXPKTRDY bit.

The USB controller sends an OUT token followed by the data from the FIFO to endpoint 0 of the addressed device, retrying as necessary.

3. At the end of the attempt to send the data, the USB controller generates an endpoint 0 interrupt (for example by setting the USB\_INTRTX.EP0 bit). The processor core can then read the USB\_EP0\_CSR[n]\_H to establish whether the USB\_EP0\_CSR[n]\_H.RXSTALL bit, the USB\_EP0\_CSR[n]\_H.TOERR bit, or the USB\_EP0\_CSR[n]\_H.NAKTO bit is set.

If USB\_EP0\_CSR[n]\_H.RXSTALL =1, the target has issued a stall response.

If USB\_EP0\_CSR[n]\_H.TOERR =1 the USB controller has tried to send the OUT token and the following data packet three times without getting a response.

If USB\_EP0\_CSR[n]\_H.NAKTO =1, the USB controller has received a NAK response to each attempt to send the OUT token, for longer than the time set in the USB\_EP0\_NAKLIMIT[n] register. Direct the USB controller to either clear the USB\_EP0\_CSR[n]\_H.NAKTO bit to continue trying this transaction (until it times out again) or to flush the FIFO to abort the transaction before clearing the USB\_EP0\_CSR[n]\_H.NAKTO bit.

If none of the USB\_EP0\_CSR[n]\_H.RXSTALL , USB\_EP0\_CSR[n]\_H.TOERR , or USB\_EP0\_CSR[n]\_H.NAKTO bits are set, the OUT data is correctly acknowledged.

4. If further data must be sent, the processor core must repeat the previous steps.

When all the data is successfully sent, the processor core proceeds to the IN status phase of the control transaction.

## IN Status Phase as a Host (Following SETUP Phase or OUT Data Phase)

The processor core driving the host device performs the following actions for the IN status phase of a control transaction.

1. Set the USB\_EP0\_CSR[n]\_H.STATUSPKT and USB\_EP0\_CSR[n]\_H.REQPKT bits. These bits must be set together.

2. Wait while the USB controller both sends an IN token and receives a response from the USB peripheral.
3. When the USB controller generates the endpoint 0 interrupt (for example, it sets the USB\_INTRTX.EP0 bit), read the USB\_EP0\_CSR[n]\_H register to establish whether the USB\_EP0\_CSR[n]\_H.RXSTALL , USB\_EP0\_CSR[n]\_H.TOERR , USB\_EP0\_CSR[n]\_H.NAKTO , or the USB\_EP0\_CSR[n]\_H.RXPKTRDY bits are set.

If USB\_EP0\_CSR[n]\_H.RXSTALL =1, the target could not complete the command and so has issued a stall response.

If USB\_EP0\_CSR[n]\_H.TOERR =1, the USB controller has tried to send the required IN token three times without getting a response.

If USB\_EP0\_CSR[n]\_H.NAKTO =1, the USB controller has received a NAK response to each attempt to send the IN token, for longer than the time set in the USB\_EP0\_NAKLIMIT[n] register. Direct the USB controller to either clear the USB\_EP0\_CSR[n]\_H.NAKTO bit to continue trying this transaction (until it times out again) or clear USB\_EP0\_CSR[n]\_H.REQPKT before clearing the USB\_EP0\_CSR[n]\_H.NAKTO bit to abort the transaction.

4. If the USB\_EP0\_CSR[n]\_H.RXPKTRDY bit is set, the processor core typically clears it.

## OUT Status Phase as a Host (Following IN Data Phase)

The processor core driving the host device performs the following actions for the OUT status phase of a control transaction.

1. Set USB\_EP0\_CSR[n]\_H.STATUSPKT and USB\_EP0\_CSR[n]\_H.TXPKTRDY bits. These bits must be set together.
2. Wait while the USB controller both sends the OUT token and a zero-length DATA1 packet.
3. At the end of the attempt to send the data, the USB controller generates an endpoint 0 interrupt. The processor core then reads the USB\_EP0\_CSR[n]\_H register to discover when the USB\_EP0\_CSR[n]\_H.RXSTALL , USB\_EP0\_CSR[n]\_H.TOERR , or USB\_EP0\_CSR[n]\_H.NAKTO bits are set.

If USB\_EP0\_CSR[n]\_H.RXSTALL =1, the target could not complete the command and so has issued a stall response.

If USB\_EP0\_CSR[n]\_H.TOERR =1, the USB controller has tried to send the STATUS packet and the following data packet three times without getting a response.

If USB\_EP0\_CSR[n]\_H.NAKTO =1, the USB controller has received a NAK response to each attempt to send the IN token, for longer than the time set in the USB\_EP0\_NAKLIMIT[n] register. Direct the USB controller to either clear the USB\_EP0\_CSR[n]\_H.NAKTO bit to continue trying this transaction (until it times out again) or to flush the FIFO to abort the transaction before clearing the USB\_EP0\_CSR[n]\_H.NAKTO bit.

4. If none of the USB\_EP0\_CSR[n]\_H.RXSTALL , USB\_EP0\_CSR[n]\_H.TOERR , or USB\_EP0\_CSR[n]\_H.NAKTO bits are set, the status phase is correctly acknowledged.

## Host IN Transactions

When the USB controller operates as a host, IN transactions are handled in the same way as OUT transactions are handled when the USB controller is operating as a peripheral. First, the USB controller sets USB\_EP[n]\_RXCSR\_H.REQPKT bit to initiate the transaction. This bit indicates to the transaction scheduler that there is an active transaction on this endpoint. The transaction scheduler then sends an IN token to the target function.

When the packet is received and placed in the Rx FIFO, the USB\_EP[n]\_RXCSR\_H.RXPKTRDY bit is set. The appropriate Rx endpoint interrupt is generated (if enabled) to signal that the processor can now unload a packet can now from the FIFO. When the processor unloads the packet, USB\_EP[n]\_RXCSR\_H.RXPKTRDY is cleared. The USB controller uses the USB\_EP[n]\_RXCSR\_H.AUTOCLR bit to clear the USB\_EP[n]\_RXCSR\_H.RXPKTRDY bit automatically when the processor unloads a maximum sized packet from the FIFO. There is also an USB\_EP[n]\_RXCSR\_H.AUTOREQ bit that automatically sets the USB\_EP[n]\_RXCSR\_H.REQPKT bit when the USB\_EP[n]\_RXCSR\_H.RXPKTRDY bit is cleared. The USB controller can use the USB\_EP[n]\_RXCSR\_H.AUTOCLR and USB\_EP[n]\_RXCSR\_H.AUTOREQ bits with an external DMA controller to perform complete bulk transfers without processor core intervention.

If the target function responds to a bulk or interrupt IN token with a NAK, the USB controller retries the transaction until the NAK limit set in the USB\_EP0\_NAKLIMIT[n] register is reached. If the target function responds with a stall, the USB controller does not retry the transaction, but sets the USB\_EP[n]\_RXCSR\_H.RXSTALL bit to interrupt the processor core. If the target function does not respond to the IN token within the required time, the USB controller retries the transaction.(USB controller also retries the transaction if there was a CRC or bit-stuff error in the packet). If, after three attempts, the target function still has not responded, the USB controller clears the USB\_EP[n]\_RXCSR\_H.REQPKT bit and interrupts the processor core with the DATAERROR\_R bit in USB\_RXCSR set.

## Host OUT Transactions

When the USB controller operates as a host, OUT transactions are handled like IN transactions are handled when the USB controller operates as a peripheral.

The USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit must be set as the processor loads each packet into the Tx FIFO. The USB controller uses the USB\_EP[n]\_TXCSR\_H.AUTOSET bit to cause the USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit to be automatically set when the processor loads a maximum sized packet into the FIFO. The USB controller can use the USB\_EP[n]\_TXCSR\_H.AUTOSET bit with an external DMA controller to perform complete bulk transfers without processor core intervention.

If the target function responds to the OUT token with a NAK, the USB controller retries the transaction until the NAK limit set in the USB\_EP0\_NAKLIMIT[n] register is reached. If the target function responds with a stall, the USB controller does not retry the transaction, but sets the USB\_EP[n]\_TXCSR\_H.RXSTALL bit to interrupt the processor core. If the target function does not respond to the OUT token within the required time, the USB controller retries the transaction.(USB controller also retries the transaction if there was a CRC or bit-stuff

error in the packet). If, after three attempts, the target function still has not responded, the USB controller flushes the FIFO and sets the USB\_EP[n]\_TXCSR\_H.TXTOERR bit to interrupt the processor core.

## Multi-Point Support

The following sections describe the multi-point support of the USB controller.

- Allocating Devices to Endpoints
- Multi-Point Operation
- Multi-Point Bandwidth Considerations

## Allocating Devices to Endpoints

The separate functions of the connected devices are allocated to the endpoints within the USB controller through a group of three registers. The registers are associated with each implemented Rx or Tx endpoint (including endpoint 0).

The registers are:

- USB\_MP[n]\_TXFUNCADDR / USB\_MP[n]\_RXFUNCADDR
- USB\_MP[n]\_TXHUBADDR / USB\_MP[n]\_RXHUBADDR
- USB\_MP[n]\_TXHUBPORT / USB\_MP[n]\_RXHUBPORT

The location of these registers depends on which of the endpoints is being addressed.

Record the address of the target function that is accessed through the selected endpoint in the transmit and receive function address registers. Record this information separately for each Tx and Rx endpoint used. In particular, set both USB\_MP[n]\_TXFUNCADDR and USB\_MP[n]\_RXFUNCADDR for endpoint 0.

The USB controller uses the transmit and receive hub address and hub port registers when a full-speed or low-speed device is connected to it through a high-speed USB 2.0 hub. The hub carries out the required transaction translation between high-speed transmission and low-speed or full-speed transmission. In this situation, the USB\_MP[n]\_TXHUBADDR / USB\_MP[n]\_RXHUBADDR and USB\_MP[n]\_TXHUBPORT / USB\_MP[n]\_RXHUBPORT registers must record the address of the hub that carries out the transaction translation. It must also record the address of the port of that hub through which the associated Tx or Rx endpoint must access the device.

If endpoint 0 is connected to a hub, then set both the Tx and the Rx versions of these registers for this endpoint. The USB controller also uses hub address registers to record whether the hub offers multiple transaction translators or just a single transaction translator. This configuration has a significant effect on the overall bandwidth that can be achieved.

In addition to recording the address of the target function, record the endpoint number and operating speed of the target device and the type of transaction that is executed. For a Tx endpoint, set this information in the USB\_EP[n]\_TXTYPE register when the index register is set to select the required endpoint. For an Rx endpoint,

set this information in the USB\_EP[n]\_RXTYPE register when the index register is set to select the required endpoint. In both cases, record the endpoint number in bits [3:0], select the transaction type through bits [5:4], and select the operating speed through bits [7:6].

Set only the speed for endpoint 0 because endpoint 0 only has the facilities to handle control transactions and therefore is always associated with a device endpoint 0. Use bits [7:6] of the Type 0 register to set the speed. The register is located at address 0x1A when the index register is set to 0.

## Multi-Point Operation

After allocating functions to endpoints and recording the operating speed of the target device, multi-point operations can be configured. Most operations in a multi-point set-up are the same as for the equivalent actions where the core is attached to a single other device.

However, more steps are required when:

- The option of dynamically switching the allocation of functions to endpoints is taken (for example, to allow the support of a wider range of devices).
- The control packets normally associated with endpoint 0 are handled through a different endpoint.

If dynamic allocation is used, the program must monitor the current data toggle state associated with the endpoint and with each of the devices that are allocated to that endpoint. This knowledge allows the program to select the correct data toggle state when switching occurs between one device and the other. (This action is the programs responsibility. The core cannot determine what data toggle state is expected when a function switches in and out of use.)

The data toggle state can be switched from its current state by writing to the appropriate USB\_EP[n]\_TXCSR\_H or USB\_EP[n]\_RXCSR\_H register. This activity sets the data toggle write enable and data toggle bits that are included in the registers when the core is in host mode.

Data toggle write enable and data toggle bits are also included in the USB\_EP0\_CSR[n]\_H register. However, control operations carried out through endpoint 0 of the core normally leave the data toggle in the expected state.

Where control packets are handled through an endpoint other than endpoint 0, programs must prompt for each setup token to be sent. Programs must set the USB\_EP[n]\_TXCSR\_H.SETUPPKT bit when the core operates in host mode, along with the USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit. If the USB\_EP[n]\_TXCSR\_H.SETUPPKT bit is not set, an OUT token is sent.

Use endpoint 0 of the USB controller to handle control packets for all of the devices attached to the controller, and to switch the allocation of this endpoint, as appropriate. Sending the correct token is ensured, as is ensuring that the data toggle is correctly set for this endpoint.

Using a different endpoint for this function is possible, as described, but note the following:

- The control function must be allocated to an Rx/Tx endpoint pair (with the same endpoint number).
- The chosen endpoints must each be associated with FIFOs that can accommodate the packet size associated with EP0 transactions at the chosen operating speed. The size is a minimum of 8 bytes for low-speed or fullspeed transactions but 64 bytes for high-speed transactions.

## Multi-Point Bandwidth Considerations

The available bandwidth determines the ability of a multi-point system to cope with isochronous transactions.

Once an endpoint is set up, hardware handles all scheduling. However, as with PC-based EHCI/OHCI/UHCI hosts, before opening a periodic pipe (for use by isochronous or interrupt traffic), software must determine that there is sufficient bandwidth available.

The bandwidth required for different transactions can be determined using algorithms similar to the ones used with PC-based hosts (detailed in Section 5.11.3 of the USB 2.0 Specification).

The available bandwidth is greater where the hub used supports multiple transaction translators.

## Babble Interrupt

If the bus is still active at the end of a frame, the USB controller assumes that the function it is connected to has malfunctioned. It suspends all transactions, and generates a babble interrupt ( USB\_IRQ.RSTBABBLE ). The USB controller does not start a transaction until the bus is inactive for at least the minimum inter-packet delay. The controller also does not start a transaction unless it can be finished before the end of the frame.

To recover from a babble error condition, the processor must take the following actions inside the interrupt service routine.

1. Turn off VBUS. Wait until the VBUS level indicator reads b#01.
2. Turn on VBUS. Wait until the VBUS level indicator reads b#11.
3. Set the USB\_IRQ.SESSREQ bit.

The VBUS level indicator is the USB\_DEV\_CTL.VBUS bit field.

- NOTE: Because VBUS is sourced external to the processor, make sure that the hardware design connects a GPIO or the dedicated USB\_VBUS signal to the external source. This connection enables software to turn VBUS on and off.

## VBUS Events

The USB On-The-Go specification defines a series of thresholds to which the devices involved in point-to-point communications must respond.

- VBUS valid (between 4.4 V and 4.75 V)
- Session valid for A device (between 0.8 V and 2.1 V)
- Session end (between 0.2 V and 0.8 V)

The critical thresholds and the processor response depend on whether the device is an A device or a B device and the circumstances of the event. The following sections describe these actions.

## Actions as an A Device

VBUS &gt;VBUS Valid with session initiated by USB controller. VBUS level indicator = b#11 and the session bit is set. When VBUS is greater than VBUS valid, the USB controller selects host mode and waits for a device to connect. It then generates a connect interrupt. The processor resets and enumerates the connected B device.

VBUS &gt; Session valid with session initiated by B device. VBUS level indicator = b#10 and the session bit is clear. When VBUS is greater than session valid, the USB controller generates a session request interrupt. The processor sets the session bit. The USB controller either stays in host mode or changes to peripheral mode, depending upon the state of the pull-up resistor on the B device. For more information, refer to the host negotiation protocol of the OTG specification. The state of the host mode bit indicates the selected mode.

VBUS below VBUS Valid while the Session bit remains set. VBUS level indicator b#11 and the session bit is set. This event indicates a problem with the VBUS power level. For example, the battery power could have dropped too low to sustain VBUS valid. Or, the B device could be drawing more current than the A device can provide. In either case, the USB controller automatically terminates the session and generates a VBUS error interrupt.

To recover from this VBUS error condition, the processor must take the following actions inside the VBUS error interrupt handler.

- Turn off VBUS and wait until the USB\_DEV\_CTL.VBUS reads b#01.
- Turn on VBUS and wait until the USB\_DEV\_CTL.VBUS reads b#11.
- Set the USB\_DEV\_CTL.SESSION bit

The USB\_DEV\_CTL.VBUS bit field indicates the VBUS level.

NOTE: Because VBUS is sourced external to the processor, make sure that the hardware design connects a GPIO or the dedicated DrvVBUS signal to the external source. Then, the software can be used to turn VBUS on and off.

## Actions as a B Device

VBUS &gt; Session Valid. VBUS level indicator = b#10 and session bit is clear. This event indicates activity from the A device. The USB controller sets the session bit and disconnects the pull down resistor on the D+ line.

VBUS &lt; Session Valid. While the session bit remains set, VBUS level indicator = b#01 and session bit is set. This event indicates that the A device has lost power (or become disconnected). The USB controller clears the session bit and generates a disconnect interrupt. The processor ends the session.

VBUS &lt; Session End. VBUS level indicator = b#00. This event is the condition under which a B device can initiate a session request. If the session bit is set, then after 2 ms of SE0 on the bus, the USB controller starts SRP by first pulsing the data line, then pulsing the USB\_VBUS signal.

## Host Mode Reset

If the USB\_POWER.RESET is set while the USB controller is in host mode, the USB controller generates reset signaling on the bus. The processor core must keep this bit set for 20 ms to ensure correct resetting of the target device. After the processor core clears the bit, the USB controller starts its frame counter and transaction scheduler.

## Host Mode Suspend

The controller has a suspend mode that allows power savings for the processor. The mode operates as follows.

Entry into Suspend mode. When operating as a host, the USB controller can be prompted to go into suspend mode by setting the USB\_POWER.SUSPEND bit. When this bit is set, the USB controller completes the current transaction then stops the transaction scheduler and frame counter. No further transactions start. No SOF packets are generated. If the USB\_POWER.SUSPEND bit is set, the UTMI+ PHY goes into low-power mode when the USB controller goes into suspend mode and stops the clock.

Sending Resume Signaling. When the application requires the USB controller to leave suspend mode, it must clear and then set the USB\_POWER.RESUME bit, and leave it set for 20 ms. While the USB\_POWER.RESUME bit is high, the USB controller generates resume signaling on the bus. After 20 ms, the processor core must clear the USB\_POWER.RESUME bit, at which point the frame counter and transaction scheduler start.

Responding to Remote Wake-up. If resume signaling is detected from the target while the USB controller is in suspend mode, the UTMI+ PHY is brought out of low-power mode and the clock restarts. The USB controller then exits suspend mode and automatically sets the USB\_POWER.RESUME bit to take over generating the resume signaling from the target. If the USB\_IRQ.RESUME bit=1, software generates an interrupt.

## Suspending and Resuming the Controller

With the introduction of link power management, there are two basic methods to suspend and resume the USB controller. The Basic LPM transaction diagram demonstrates these two methods.

Figure 27-9: Basic LPM Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000008_5f2bd9eb02db304549185724d0ab4f144ce5638e4ef3cc4c727c7083c721bf10.png)

The procedure that suspends and resumes the USB controller depends on whether the core operates as a device or a host, and the method of suspend desired. The following sections describe these options.

## Suspend or Resume by Inactivity on the USB Bus (L0 to L2 State) in Peripheral Mode

The following steps occur in this mode.

1. Entry into suspend mode. When operating as a peripheral, the USB controller monitors activity on the USB
2. and when no activity has occurred for 3 ms, the controller goes into suspend mode. If the USB\_IRQ.SUSPEND interrupt has been enabled, the USB controller now generates an interrupt. The USB\_IRQ.SUSPEND output also goes low (if enabled).

The POWERDWN signal is also asserted to indicate that the application can stop USB\_CLKIN to save power. POWERDWN then remains asserted until either power is removed from the bus (indicating that the device has been disconnected) or resume signaling or reset signaling is detected on the bus.

2. When resume signaling occurs on the bus, the USB\_CLKIN must be restarted, if necessary. The USB controller then automatically exits suspend mode. If the USB\_IRQ.RESUME interrupt is enabled, the USB controller generates an interrupt.
3. Initiating a remote wake-up. To initiate a remote wake-up while the controller is in suspend mode, set the USB\_POWER.RESUME bit=1. ( If USB\_CLKIN has been stopped, it must be restarted before this write can occur.) The software must leave then this bit set for approximately 10 ms (minimum of 2 ms, a maximum of 15 ms) before resetting it to 0. By this time the hub is driving resume signaling on the USB.

NOTE: The USB\_IRQ.RESUME interrupt is not generated when the software initiates a remote wake-up.

## Suspend or Resume by Inactivity on the USB Bus (L0 To L2 State) in Host Mode

The following steps occur in this mode.

1. Entry into suspend mode. When operating as a host, the USB controller can be prompted to go into suspend mode by setting the USB\_POWER.SUSPEND bit. When this bit is set, the USB controller completes the current transaction then stops the transaction scheduler and frame counter. No further transactions start and no SOF packets are generated. If the USB\_POWER.SUSEN bit is set, the UTMI+ PHY goes into low-power mode when the controller goes into suspend mode and stops USB\_CLKIN .
2. Sending resume signaling. When the application requires the controller to leave suspend mode, it clears the USB\_POWER.SUSPEND bit, sets the USB\_POWER.RESUME bit, and leaves it set for 20 ms. While the USB\_POWER.RESUME bit is high, the controller generates resume signaling on the bus. After 20 ms, the processor core must clear the USB\_POWER.RESUME bit, at which point the frame counter and transaction scheduler start.
3. Responding to remote wake-up. If resume signaling is detected from the target while the USB controller is in suspend mode, the UTMI+ PHY is brought out of low-power mode and restarts USB\_CLKIN . The controller then exits suspend mode and automatically sets the USB\_POWER.RESUME bit to 1 to take over generating the resume signaling from the target. If the USB\_IRQ.RESUME interrupt is enabled, the USB controller generates an interrupt.

## Suspend or Resume by an LPM Transaction (L0 To L1 State) in Peripheral Mode

The following steps occur in this mode.

1. Enter into suspend mode. When operating as a peripheral, the controller never initiates an LPM suspend (transition from the L0 state to the L1 state). Rather, the controller only suspends at the request of the host. Configure the USB\_LPM\_CTL register appropriately to enable the LPM feature. The USB controller uses the register field USB\_LPM\_CTL.EN bit to enable and support extended and LPM transactions. The USB controller uses the USB\_LPM\_CTL.TX field to instruct the hardware that it is ready to suspend and to respond to the next LPM transaction with an ACK. In this case, the controller responds to the next LPM transaction with an ACK

if all other conditions are met. The Response to LPM Transaction table summarizes the response of the USB controller to an LPM transaction.

Table 27-6: Response to LPM Transaction

| LPMXMT               | LPMCNTRL                 | Data Pending (Resides in Tx FIFOs)   | Response to Next LPM Transac- tion   |
|----------------------|--------------------------|--------------------------------------|--------------------------------------|
| 1'b0, 1'b0 1'b1 1'b1 | 2'b00, 2'b10 2'b00 2'b10 | Do-not-care                          | Timeout                              |
| 1'b0, 1'b1           | 2'b01                    | Do-not-care                          | STALL                                |
| 1'b0                 | 2'b11                    | Do-not-care                          | NYET                                 |
| 1'b1                 | 2'b11                    | Yes                                  | NYET                                 |
| 1'b1                 | 2'b11                    | No                                   | ACK                                  |

For all cases in the table in which the controller responds (no timeout occurs), an LPM interrupt is generated in the USB\_LPM\_IRQ register. The controller responds with an ACK only if there is no data pending in any of the Tx endpoint FIFOs. If there is data pending, the USB controller responds with a NYET.

Once an LPM transaction is successfully received, three events occur:

- a. The USB\_LPM\_ATTR register is updated with values received in the LPM transaction. See the 'Register Descriptions' section of this chapter for complete information on this register.
- b. The controller suspends 9 µs after transmitting the ACK. The host or the controller can drive resume signaling 50 µs after this event. During the 9 µs interval, the host can continue to transmit the LPM transaction. The controller responds with an ACK in this case regardless of the USB\_LPM\_CTL.TX bit value.
- c. An interrupt is generated informing software of the response (an ACK in this case). An ACK response is the indication to software that the controller has suspended.

Since the primary purpose of LPM is to save power, software reads the USB\_LPM\_ATTR register to determine the attributes of the suspend. Software must make a determination based on these attributes whether there are more potential power savings in the system. In making this determination, note that if the host initiates the resume signaling, the controller must respond to packet transmissions within the time specified by USB\_LPM\_ATTR.HIRD + 10 µs.

2. When resume signaling occurs on the bus. When the host resumes the bus, it drives resume signaling for a minimum time specified by the host initiated resume duration bit field ( USB\_LPM\_ATTR.HIRD ). The controller must be able to respond to traffic within the time HIRD + 10 µs. The controller transitions to a normal operating state automatically and a resume interrupt is generated in the USB\_LPM\_IRQ register.

However for this event to occur, the inputs CLK and XCLK must be available. To facilitate the resume timing requirement, a negative ACK (NAK) is provided using the USB\_LPM\_CTL.NAK bit. If this bit is set to 1'b1,

all endpoints respond to any transaction (other then an LPM) with a NAK. This bit only takes effect after the controller has suspended LPM. Typically, this bit is asserted when the USB\_LPM\_CTL.TX field is also asserted. Using this feature can simplify the resume timing requirement because the controller only needs XCLK to respond (with a NAK) to traffic. Software can continue to restore the system to normal operation while the controller responds to all transactions with a NAK. After software completely restores the system, it can then clear the USB\_LPM\_CTL.NAK bit.

3. Initiating remote wake-up. To initiate a remote wake-up while in suspend mode, the controller writes a 1'b1 to the USB\_LPM\_CTL.RESUME bit. This bit is self clearing. Writing a 1'b1 drives resume signaling on the bus for 50 µs. The host responds by driving resume for 60 µs to 990 µs. 10 µs after the host stops driving resume, the controller transitions to its normal operational state and is ready for packet transmission. A resume interrupt is generated in the USB\_LPM\_IRQ register.

## Suspend or Resume by an LPM Transaction (L0 to L1 State) in Host Mode

The following steps occur in this mode.

1. Enter into suspend mode. When operating as a host, the controller initiates an LPM suspend (transition from the L0 state to the L1 state) by initiating an LPM transaction as follows.
- a. Software sets up the desired attributes of the suspend in the USB\_LPM\_ATTR register. Enabling remote wake-up and a large HIRD gives the peripheral more opportunity to conserve power.
- b. Enable all LPM interrupts in the USB\_LPM\_IEN register.
- c. Software writes 0x01 to the USB\_LPM\_CTL register to initiate the transaction.
- d. An interrupt is generated to inform software of the response to the LPM transaction. If an ACK was received, then the controller suspends automatically within 8 µs. This event indicates that the controller has suspended.

If the response from the device has a bit stuff error or a PID error, then an USB\_LPM\_IRQ.LPMERR interrupt is generated. The hardware immediately attempts the LPM transaction two more times. The device does not suspend for 8 µs after the initial LPM so it can respond to either of these subsequent LPM transactions. If an LPM timeout has occurred three times, the USB\_LPM\_IRQ.LPMNC and the USB\_LPM\_IRQ.LPMERR interrupts are set. Now, software is unaware of the device state and must deduce it by other means.

2. Send resume signaling. Software generates resume signaling as follows.
- a. Enable all LPM interrupts in the USB\_LPM\_IEN register.
- b. Software writes to the USB\_LPM\_CTL.RESUME bit which is self-clearing. This operation causes resume signaling on the bus for the time specified in the USB\_LPM\_ATTR.HIRD bit field. Hardware assumes that the last LPM transaction that caused the suspend used this value.
- c. After HIRD + 10 µs, the controller transitions to its normal operational state and is ready for packet transmission and a USB\_LPM\_IRQ.LPMRES interrupt is generated.

NOTE: Prior to resuming, software must ensure that the system is restored from a low-power state and that the inputs CLK and XCLK are available.

3. Responding to remote wake-up. If the remote wake-up feature is enabled in the LPM transaction that caused the suspend, then the device can drive resume signaling on the bus. When this event occurs, the device drives resume signaling BUS for 50 µs. The controller immediately begins driving resume signaling on the BUS and continues for 60 µs. 10 µs after completion of the resume signaling, the controller transitions to its normal operating state and is ready for packet transmission. Then, the USB\_LPM\_IRQ.LPMRES interrupt is generated.

## USB Event Control

The following sections provide information on the use of interrupts, reset, and the reporting of errors and interface status.

## Interrupt Signals

The "Interrupt Table" section at the beginning of this chapter shows the two interrupts generated from the USB controller.

The software generates interrupts from control endpoint zero under the following conditions :

- When a control transaction ends before the end of the data is transferred
- When a data packet is sent or received from the endpoint 0 FIFOs

The USB controller generates interrupts from transmit endpoints ( USB\_INTRTX ) under the following conditions:

- A packet is sent from the TX FIFO (host and peripheral mode)
- After three attempts at transmitting a packet, no valid handshake packet is received (host mode)

The software generates interrupts from receive endpoints ( USB\_INTRRX ) under the following conditions:

- A packet is received into the RX FIFO (host and peripheral mode)
- A stall handshake is received (host mode)
- After three attempts at receiving a packet, no data packet is received (host mode)

The software generates interrupts from the USB status ( USB\_IRQ ) under the following conditions:

- When VBUS drops below the VBUS valid threshold during a session (A device only)
- When SRP signaling is detected (A device only)
- When device disconnect is detected (host mode)
- When a session ends (peripheral mode)
- When a device connection is detected (host mode)
- At start of frame (SOF)
- When reset signaling is detected on USB (peripheral mode)

- When babble is detected (host mode)
- In suspend mode, when resume signaling is detected on USB
- When suspend signaling is detected (peripheral mode)

The software generates interrupts for the following VBUS control requests:

- Drive VBUS greater than 4.4 V (default A device)
- Stop driving VBUS
- Start charging VBUS (peripheral mode)
- Stop charging VBUS
- Start discharging VBUS (peripheral mode)
- Stop discharging VBUS

## Interrupt Handling

When interrupted with a USB interrupt, the processor core must read the interrupt status register to determine which endpoints have caused the interrupt and jump to the appropriate routine. If multiple endpoints have caused the interrupt, endpoint 0 must be serviced first, followed by the other endpoints. The USB Interrupt Service Routine figure shows a flowchart for the USB interrupt service routine.

Figure 27-10: USB Interrupt Service Routine

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000009_2a6415b9405abf9fa43d0e0dafcc92e5aa423e34fe78eeda814078c0b94b1c97.png)

## Reset Signals

The USB controller includes an active-high synchronous hardware reset sourced from the processor core. Another source of peripheral reset is through the USB, when USB reset signaling is detected on the I/O lines. Per the USB 2.0 Specification, this state is entered when both the D+ and D- inputs are driven low for 2.5 ms or more. (The USB host typically holds the reset for greater than 10 ms.)

## Reset in Peripheral Mode

When the USB controller detects a reset, it performs the following actions:

- Sets the USB\_FADDR register to zero
- Sets the USB\_INDEX register to zero

- Flushes all endpoint FIFOs
- Clears all control and status registers
- Enables all interrupts
- Generates a reset interrupt

The USB controller reset does not affect the USB\_IRQ and USB\_VBUS\_CTL registers. These registers are only reset (along with the ones listed) during a system reset.

If the USB\_POWER.HSEN bit was set, the USB controller also tries to negotiate for high-speed operation. The USB\_POWER.HSMODE bit indicates whether high-speed operation is selected.

When the application software receives a reset interrupt, it closes any open pipes and waits for bus enumeration to begin.

## USB Reset in Host Mode

If the USB\_POWER.RESET bit =1 while the USB controller is in host mode, the controller generates reset signaling on the bus.

If the USB\_POWER.HSEN bit =1, the controller also tries to negotiate for high-speed operation.

The processor core must keep the USB\_POWER.RESET bit set for at least 20 ms to ensure correct resetting of the target device. After the processor core clears the bit, the USB controller starts its frame counter and transaction scheduler.

The USB controller uses the USB\_POWER.HSMODE bit to select high-speed operation.

## USB Programming Model

The following sections describe the USB OTG programming model.

## Peripheral Mode Flow Charts

Figure 27-11: USB Control SETUP Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000010_614dd85ca2f6350e6a6ac2c205481b7e2b0c4a668fdbe8c75673f09652b23145.png)

Figure 27-12: Control In Data Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000011_f2cf7f360348658c9fd88ca1f42fb37ec3f835e68a433b85671763409dc5bd73.png)

Figure 27-13: Control In Status Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000012_2aa120e3cf9b1341640c38de3fd959e361898037633e7240c0c626c048006b88.png)

Figure 27-14: Control Out Data Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000013_7f563cb109b29a1c8a21d6bea7f5999b96962b6862c6c8ba68e82a39ffe1b524.png)

Figure 27-15: Control Out Status Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000014_7524d57a9eb9a2804577cc81353fb73e209217bbc13de07fb91ebc3202bc3c31.png)

Figure 27-16: Bulk/Low Bandwidth Interrupt In Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000015_01b5144984425e61101660d873b58bb82a59e2f892cfb5e573ce932ecd88a2e1.png)

Figure 27-17: Bulk/Low Bandwidth Interrupt Out Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000016_7bc4e9825222711be9aaa026b8284de74f47176535355a21283cc6960217be32.png)

Figure 27-18: Full-speed/Low Bandwidth Isochronous In Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000017_7ea34d6193d3caf45e56ed4bf860febfe875522f91aa306d57d358d803588232.png)

Figure 27-19: Full-speed/Low Bandwidth Isochronous Out Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000018_0660670c90cd92e2f3bc39ccd8d144d1c0b9bd6f569f97aee83bb16cf06b7c75.png)

## Host Mode Flow Charts

Figure 27-20: USB Control SETUP Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000019_afc8fc94104703de8e5009a759f59e2deb0a2db7f01679b41f1ada9f2b1573cb.png)

Figure 27-21: Control In Data Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000020_e5a2e9ea01099810da5649e6bb266032310700409fbdf5e6fd92ac527b45bbfd.png)

Figure 27-22: Control In Data Status Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000021_d7e31ca9c34545f4a2f06c13d527856dff385e1d41ba92b88f025d3065a07606.png)

Figure 27-23: Control Out Data Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000022_de2da3c4b2ea94bc7d18ba7bb991d83a08ccf23ccad7087ed74f10c0d51cc160.png)

Figure 27-24: Control Out Data Status Phase

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000023_a6c46dea970253730cf2a6a49c0d58a843988aba59592548da2cf200913a2303.png)

Figure 27-25: Bulk/Low Bandwidth Interrupt In Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000024_f8be0f385a0d2724bdc575b0d757d29a0c80a6ca20623a3e0950f4ced6e5e742.png)

Figure 27-26: Bulk/Low Bandwidth Interrupt Out Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000025_a8c1e18c473b9add21e46fb6f5bf5138fc16587721a5722f717a2c54ff04c310.png)

Figure 27-27: Full-Speed/Low Bandwidth Isochronous In Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000026_2ec87e2bd0c1bb3705542bf6b757549117f50f460fd223a063f7ed3872cf3b6c.png)

Figure 27-28: Full-Speed/Low Bandwidth Isochronous Out Transaction

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000027_ccc5299f5374ac927b1b3844ec2d2c5f0098e3eb9b659a39f62cbb1151795add.png)

## DMA Mode Flow Charts

Figure 27-29: Single Packet Transmit During DMA Operation

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000028_657cfdce2bb143cb57151771b5a0f8d664e951ef2a6f0952642276946c5993f8.png)

Figure 27-30: Single Packet Receive During DMA Operation

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000029_6352a214da65d610193b54c3e532aabbf3d9379dc49e3bcef99766f3d51427f8.png)

Figure 27-31: Multiple Packet Transmit During DMA Operation

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000030_40238b183a2f2bf262e09681c07b630c565a7dd25cb9bb174b434eeb6c190498.png)

Figure 27-32: Multiple Packet Receive During DMA Operation (Data Size Known)

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000031_015269de8c35ad9be107053e6b87c5b67d4da087f4e0e395ba5abfb811993c6e.png)

Figure 27-33: Multiple Packet Receive During DMA Operation (Data Size Not-known)

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000032_83de4cea76ce657295495497e8f2302b013754446bccc1242b7089e9b21b16a3.png)

## OTG Session Request

To conserve power, the USB on-the-go supplement allows VBUS to only power-up when required and to turn off when the bus is not in use.

The A device on the bus always supplies VBUS. The USB controller samples the USB\_ID input from the PHY to determine whether it is the A device or the B device. The signal is pulled low when an A-type plug is sensed (signifying that the USB controller is the A device). The input is taken high when a B-type plug is sensed (signifying that the USB controller is the B device).

## Starting a Session

When the device containing the USB controller wants to start a session, the processor core must set the USB\_DEV\_CTL.SESSION bit. The USB controller then enables ID pin sensing. This activity results in the USB\_ID input either being taken low if an A-type connection is detected or high if a B-type connection is detected. The USB\_DEV\_CTL.BDEVICE bit is also set to indicate whether the USB controller has adopted the role of the A device or the B device.

The USB controller is the A device. The USB controller then enters host mode (the A device is always the default host). It waits for VBUS to go above the VBUS valid threshold, as indicated when the USB\_DEV\_CTL.VBUS bits go to 11.

The USB controller then waits for a peripheral to be connected. When the USB controller detects a peripheral, it generates a connect interrupt ( USB\_IRQ.CON bit) (if enabled). It sets either the USB\_DEV\_CTL.FSDEV or USB\_DEV\_CTL.LSDEV bits, depending on whether a full-speed peripheral or a low-speed peripheral was detected. The processor core then resets this peripheral. To end the session, the processor core must clear the USB\_DEV\_CTL.SESSION bit.

The USB controller is the B device. The USB controller requests a session using the session request protocol defined in the USB on-the-go supplement. This functionality is accomplished by setting the USB\_DEV\_CTL.SESSION bit.

At the end of the session, typically the USB controller clears the USB\_DEV\_CTL.SESSION bit. But, the processor core can also clear it when the application software must perform a software disconnect. For more information, see the description of the USB\_DEV\_CTL register. The USB controller switches on the pull-up resistor on D+. This activity signals to the A device to end the session.

## Detecting Activity

When the other device of the OTG set-up wants to start a session, USB controller either:

- Raises VBUS above the session valid threshold (if A device) or
- First pulses the data line, then pulses VBUS (if B device)

Depending on which of these actions happens, the USB controller can determine whether it is the A device or the B device in the current set-up and act accordingly. (The USB\_DEV\_CTL.VBUS bits=10 indicates if it is the A device.)

If VBUS is raised above the session valid threshold, the USB controller is the B device. The USB controller sets the USB\_DEV\_CTL.SESSION bit. When reset signaling is detected on the bus, a reset interrupt ( USB\_IRQ.RSTBABBLE =1) is generated (if enabled) that the processor core interprets as the start of a session. The USB controller is in peripheral mode as the B device is the default peripheral.

At the end of the session, the A device turns off the power to VBUS. When VBUS drops below the session valid threshold, the USB controller detects this state and clears the USB\_DEV\_CTL.SESSION bit to indicate that the session has ended. (The USB\_DEV\_CTL.VBUS bits=01 indicates that VBUS has dropped below the session valid threshold). A disconnect interrupt ( USB\_IRQ.DISCON bit) is also generated (if enabled).

If data line or VBUS pulsing is detected, the USB controller is the A device. The controller generates a USB\_IRQ.SESSREQ interrupt to indicate that the B device is requesting a session. The processor core must then start a session by setting the USB\_DEV\_CTL.SESSION bit.

## Host Negotiation Protocol

When the USB controller is the A device ( USB\_ID low, USB\_DEV\_CTL.BDEVICE =0), the controller automatically enters host mode when a session starts.

When the USB controller is the B device ( USB\_ID high, USB\_DEV\_CTL.BDEVICE =1), the controller automatically enters peripheral mode when a session starts. The processor core can request that the USB controller become

the host by setting the USB\_DEV\_CTL.HOSTREQ bit. This bit can be set either when requesting a session start by setting the USB\_DEV\_CTL.SESSION bit or at any time after a session has started.

When the USB controller enters suspend mode (no activity on the bus for 3 ms), and assuming the USB\_DEV\_CTL.HOSTREQ bit remains set, it then enters host mode. It begins host negotiation (as specified in the USB OTG supplement), causing the PHY to disconnect the pull-up resistor on the D+ line. This event causes the A device to switch to peripheral mode and to connect its own pull-up resistor. When the USB controller detects this activity, it generates a connect interrupt ( USB\_IRQ.CON bit). The controller also sets the USB\_POWER.RESET bit to begin resetting the A device. (The USB controller begins this reset sequence automatically to ensure that reset is started as required within 1 ms of the A device connecting its pull-up resistor.) The processor core must wait at least 20 ms, then clear the USB\_POWER.RESET bit and enumerate the A device.

When the USB controller-based B device has finished using the bus, the processor core must put it into suspend mode by setting the USB\_POWER.SUSPEND . The A device detects this state and either terminates the session or reverts to host mode. If the A device is USB controller-based, it generates a disconnect interrupt ( USB\_IRQ.DISCON bit) if enabled.

## Data Transfer

Whether the USB controller is operating in host or peripheral mode, data channels through the endpoint FIFOs to construct packets that are sent or received over the USB. The USB controller uses the Rx FIFOs to receive OUT packets when in peripheral mode and IN packets when operating in host mode. Similarly, The USB controller uses the Tx FIFOs to transmit IN packets when in peripheral mode and OUT packets as a host.

Data can be moved between the FIFOs and memory using either DMA or core accesses. Each endpoint FIFO has its own individually programmable options so that each can be set up separately. The system must treat each transfer type differently. Data transfers of significant size almost certainly require DMA to move the data around; but the processor can handle smaller packet sizes completely.

Each data endpoint supports both double and single-buffering modes. In single-buffered operation, the processor loads and unloads FIFOs on a packet-by-packet basis. Double-buffering imposes less burden on the system by allowing two packets to be buffered in a FIFO before it is necessary to use DMA or interrupts to service the FIFO. Double-buffering mode is automatically enabled when a MaxPktSize is set for an endpoint that is equal to or less than half the size in bytes of that FIFO.

## Loading or Unloading Packets from Endpoints

Transfers to and from the FIFOs can be 32-bit, 16-bit, or 8-bit. When using core accesses, use the same width for transfers associated with one data packet, so that data is consistently byte, half-word, or word aligned. The last transfer can, however, contain fewer bytes than the previous transfers in order to complete an 8-bit or 16-bit transfer.

When using the DMA to access the FIFOs, the starting DMA address must be word aligned, or aligned on a 32-bit boundary. The packet transfer starts with a word transfer, but half-word or byte transfers can be added at the end to handle any leftovers.

## DMA Master Channels

The USB controller provides eight DMA master channels.

These channels provide a more efficient transfer of larger amounts of data between the FIFOs and the processor core, and the channels free up the processor core for other tasks. The processor uses the DMA control registers to configure and control each of these channels.

Each DMA controller can operate in one of two DMA modes: 0 or 1. When operating in mode 0, the DMA controller can only be programmed to load or unload one packet, so processor intervention is required for each packet transferred over the USB. The DMA controller can use this mode with any endpoint, whether it uses control, bulk, isochronous, or interrupt transactions.

When operating in DMA mode 1, the DMA controller can only be programmed to load or unload a complete bulk transfer, which can be many packets. After set up, the DMA controller loads or unloads the packets, interrupting the processor only when the transfer has completed. DMA mode 1 can only be used with endpoints that use bulk transactions. It is most valuable where large blocks of data are transferred to a bulk endpoint. The USB protocol requires splitting such packets into a series of packets of MaxPktSize for the endpoint.

The DMA controller can use mode 1 to avoid the overhead of having to interrupt the processor after each individual packet. It interrupts the processor only after the transfer completes. In some cases, the block of data transferred consists of a predefined number of these packets that the controlling software counts through the transfer process. In other cases, the last packet in the series can be less than the maximum packet size. The receiver can use this short packet to signal the end of the transfer. If the total size of the transfer is an exact multiple of the maximum packet size, the transmitting software must send a null packet for the receiver to detect.

NOTE: Each channel can be independently programmed for the selected operating mode.

For bulk OUT transfers using DMA mode 1, the DMA request line is asserted only when:

- There is an edge transition of the state of the USB\_EP[n]\_RXCSR\_H.RXPKTRDY , and
- A payload of MaxPktSize has been received

If a data packet is in the FIFO prior to setting the DMA request mode bits, the DMA request line is not asserted when the DMA is enabled. DMA is enabled using the USB\_DMA[n]\_CTL.EN bit. (DMA request mode bits are USB\_EP[n]\_RXCSR\_H.DMAREQMODE or USB\_EP[n]\_RXCSR\_P.DMAREQMODE ). The data is not read from the Rx FIFO in this situation, resulting in a DMA hang. However, since the packet arrived before DMA request mode and DMA request enable bits ( USB\_EP[n]\_RXCSR\_H.DMAREQEN or USB\_EP[n]\_RXCSR\_P.DMAREQEN ) were enabled, an Rx interrupt is generated for the corresponding endpoint. Therefore, the software must set the DMA request mode to request mode 0 to unload the pre-received packet. The Rx interrupt service routine can be similar to the following:

If USB\_EP[n]\_RXCNT = MaxPktSize

Switch to DMA mode 0 and unload the packet (in mode 0, the DMA request enable is always asserted, whenever there is data in the FIFO)

Set the USB\_EP[n]\_RXCNT to MaxPktSize so as to unload only one packet

If USB\_EP[n]\_RXCSR\_H.AUTOCLR is set, it is not necessary to clear USB\_EP[n]\_RXCSR\_H.RXPKTRDY manually

Switch back to DMA Mode 1 and set the count to

(Total\_Count - MaxPktSize)

Else

Handle as normal for case of short packet

DMA transfers can be 8-bit, 16-bit, or 32-bit. All transfers associated with one packet (except for the last) must be of the same width, so that the data is consistently byte-aligned or word-aligned. The last transfer can contain fewer bytes than the previous transfers to complete an odd-byte or odd-word transfer.

## DMA Bus Cycles

The DMA controller uses incrementing bursts of an unspecified length on the peripheral DMA bus. The controller starts a new burst when it is first granted bus mastership and when the peripheral address starts a new 1 KB block. (The controller is granted bus mastership at the start of a USB packet or when regaining the bus after losing it following a partial packet).

When unloading packets from the FIFOs, the DMA controller requests ahead to the USB controller. The DMA controller starts the transfer with two BUSY cycles while it is gets the first word from the FIFO. All subsequent words of the packet are immediately available and no further BUSY cycles are required. The DMA controller is associated with a two-word buffer, so it does not lose data when it loses bus mastership in the middle of unloading a packet. When the controller regains bus mastership, it can continue unloading the packet without adding any BUSY cycles.

The DMA start address (written to the USB\_DMA[n]\_ADDR register) must be word aligned. The DMA controller supports split transactions and retries.

The DMA request lines are individually enabled using the appropriate DMA request enable bit (there are four options: Tx peripheral and host and Rx peripheral and host) and operate in two modes, referred to as DMA request mode 0 and DMA request mode 1. The operating mode is configured using the appropriate DMA request mode bit (there are four options: Tx peripheral and host and Rx peripheral and host).

- NOTE: When operating in host mode, if the USB\_EP[n]\_TXCSR\_H.RXSTALL bit or the USB\_EP[n]\_TXCSR\_H.TXTOERR is set following three failed transmit attempts, the DMA request line is disabled until the bits are cleared.

The mode selected also affects the generation of endpoint interrupts (if enabled). In DMA request mode 0, no interrupt is generated when packets are received but the appropriate endpoint interrupt is generated to prompt the loading of all packets. In DMA request mode 1, the endpoint interrupt is suppressed except following the receipt of a short packet (one less than USB\_EP[n]\_RXMAXP bytes).

Table 27-7: Endpoint Interrupt Associated with the Receive Packet Ready Bit=1

|   DMAReqEnab | DMAReqMode   | EP Interrupt Generated?   |
|--------------|--------------|---------------------------|
|            0 | X            | YES                       |
|            1 | 0            | NO                        |
|            1 | 1            | Only is short packet      |

Table 27-8: Endpoint Interrupt Associated with the Receive Packet Ready Bit=0

|   DMAReqEnab | DMAReqMode   | EP Interrupt Generated?   |
|--------------|--------------|---------------------------|
|            0 | X            | YES                       |
|            1 | 0            | YES                       |
|            1 | 1            | NO                        |

NOTE: Set the USB\_EP[n]\_TXMAXP / USB\_EP[n]\_RXMAXP registers to an even number of bytes for proper interrupt generation in DMA mode 1.

DMA transfers can be 8-bit, 16-bit, or 32-bit as required. However, all transfers associated with one packet (except for the last) must be of the same width so that the data is consistently byte-aligned, word-aligned, or double-wordaligned. The last transfer can contain fewer bytes than the previous transfers to complete an odd-byte or odd-word transfer.

NOTE: Disable DMA requests before changing the DMA request mode bit. In particular, do not set the USB\_EP[n]\_TXCSR\_H.DMAREQMODE bit to zero either before or in the same cycle as the corresponding USB\_EP[n]\_TXCSR\_H.DMAREQEN bit is cleared to zero.

## Transferring Packets Using DMA

Both the channel and the endpoint must be programmed appropriately to use the the DMA master channels to access the USB controller FIFOs. Many variations are possible. The following sections detail the standard set ups used for the basic actions of transferring individual and multiple packets.

## Individual Rx Endpoint Packet

The transfer of individual packets normally uses DMA mode 0. Program the USB controller Rx endpoint as follows.

1. Set to 1 the relevant bit in the USB\_INTRRXE register.
2. Set to 0 the DMA enable bit of the appropriate USB\_EP[n]\_RXCSR\_H.DMAREQEN / USB\_EP[n]\_RXCSR\_P.DMAREQEN register. (There is no need to set the USB controller to support DMA for this operation.)
3. When the USB controller receives a packet, it generates the appropriate endpoint interrupt (using the USB\_INTRRXE register). The processor must then program the appropriate DMA master channel as follows.
- Configure the USB\_DMA[n]\_ADDR register with the memory address to store the packet

- Configure the USB\_DMA[n]\_CNT register with the size of packet (determined by reading the USB controller USB\_RQPKTCNT[n] register)
- Configure the USB\_DMA[n]\_CTL register using the following bit settings: USB\_DMA[n]\_CTL.IE =1 USB\_DMA[n]\_CTL.EN =1, USB\_DMA[n]\_CTL.DIR =0, USB\_DMA[n]\_CTL.MODE =0

The DMA controller then requests bus mastership and transfers the packet to memory. It interrupts the processor when it has completed the transfer. The processor then clears the USB\_EP[n]\_RXCSR\_H.RXPKTRDY bit.

## Individual Tx Endpoint Packet

Using DMA mode 0, program a USB controller Tx endpoint as follows.

1. Set to 1 the relevant bit in the USB\_INTRTXE register.
2. Set to 0 the DMA enable bit of the appropriate USB\_EP[n]\_TXCSR\_H.DMAREQEN / USB\_EP[n]\_TXCSR\_P.DMAREQEN register. (There is no need to set the USB controller to support DMA for this operation.)
3. When the FIFO can accommodate data, the USB controller interrupts the processor with the appropriate Tx endpoint interrupt. The processor must then program the DMA channel as follows:
- Configure the USB\_DMA[n]\_ADDR register with the memory address to store the packet.
- Configure the USB\_DMA[n]\_CNT register with the size of packet.
- Configure the USB\_DMA[n]\_CTL register using the following bit settings: USB\_DMA[n]\_CTL.IE =1 USB\_DMA[n]\_CTL.EN =1, USB\_DMA[n]\_CTL.DIR =1, USB\_DMA[n]\_CTL.MODE =0.

The DMA controller then requests bus mastership and transfers the packet to the USB controller FIFO. When it has completed the transfer, it generates a DMA interrupt. The processor then sets the USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit.

## Multiple Rx Endpoint Packets

The transfer of multiple packets normally uses DMA mode 1. Program the DMA controller using the DMA registers:

- Configure the USB\_DMA[n]\_ADDR register with the memory address of data block to send.
- Configure the USB\_DMA[n]\_CNT register with the maximum size of data buffer.
- Configure the USB\_DMA[n]\_CTL register using the following bit settings: USB\_DMA[n]\_CTL.EN =1, USB\_DMA[n]\_CTL.IE =1, USB\_DMA[n]\_CTL.DIR =0, USB\_DMA[n]\_CTL.MODE =1.

## Program the USB controller Rx endpoint as follows:

1. Set to 1 the relevant bit in the USB\_INTRRX register.
2. Set to 1 the USB\_EP[n]\_RXCSR\_H.AUTOCLR , USB\_EP[n]\_RXCSR\_H.DMAREQEN , and USB\_EP[n]\_RXCSR\_H.DMAREQMODE bits of the appropriate receive control and status register (host or

```
peripheral). In host mode, set to 1 the USB_EP[n]_RXCSR_H.AUTOREQ and USB_EP[n]_RXCSR_H.DMAREQMODE bits.
```

As the USB controller receives each packet, the DMA master channel requests bus mastership and transfers the packet to memory. With USB\_EP[n]\_RXCSR\_H.AUTOCLR set, the USB controller automatically clears its USB\_EP[n]\_RXCSR\_H.RXPKTRDY bit. This process continues automatically until the USB controller receives a short packet (one of less than the maximum packet size for the endpoint) signifying the end of the transfer. The DMA controller does not transfer this short packet: instead the USB controller interrupts the processor by generating the appropriate endpoint interrupt. The processor can then read the USB\_EP[n]\_RXCNT register to see the size of the short packet. It either unloads the packet manually or reprograms the DMA controller in mode 0 to unload the packet.

The USB\_DMA[n]\_ADDR register is incremented as the DMA controller unloads the packets. The processor determines the size of the transfer by comparing the current value of USB\_DMA[n]\_ADDR with the start address of the memory buffer.

If the size of the transfer exceeds the data buffer size, the DMA controller stops unloading the FIFO and interrupts the processor.

## Multiple Tx Endpoint Packets

Using DMA mode 1 for a Tx endpoint, program the DMA controller as follows:

- Configure the USB\_DMA[n]\_ADDR register with the memory address of data block to send.
- Configure the USB\_DMA[n]\_CNT register with the size of the data block.
- Configure the USB\_DMA[n]\_CTL register using the following bit settings: USB\_DMA[n]\_CTL.EN =1, USB\_DMA[n]\_CTL.IE =1 USB\_DMA[n]\_CTL.DIR =1, USB\_DMA[n]\_CTL.MODE =1.

Program the USB controller Tx endpoint as follows:

1. Set to 1 the relevant bit in the USB\_INTRTXE register.
2. Set to 1 the USB\_EP[n]\_TXCSR\_H.AUTOSET and USB\_EP[n]\_TXCSR\_H.DMAREQEN bits of the appropriate transmit control and status register (host or peripheral).

When the FIFO in the USB controller becomes available, the DMA controller requests bus mastership and transfers a packet to the FIFO. With USB\_EP[n]\_TXCSR\_H.AUTOSET set, the USB controller automatically sets the USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit. This process continues until the entire data block is transferred to the USB controller.

The DMA controller then interrupts the processor by taking the appropriate USB\_DMA\_IRQ register bit low.

- If the last packet loaded was less than the maximum packet size for the endpoint, the USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit is not set for this packet. The processor must respond to the DMA interrupt by setting the USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit to allow the last short packet to be sent.

- If the last packet loaded is the maximum packet size, then the appropriate action depends on whether the transfer is under the control of an application. One example is the mass storage software on Windows system that keeps count of the individual packets sent.
- If the transfer is not under such control, the processor must respond to the DMA interrupt by setting the USB\_EP[n]\_TXCSR\_H.TXPKTRDY bit. This operation sends a null packet for the receiving software to interpret as indicating the end of the transfer.

## ADSP-SC58x USB Register Descriptions

Universal Serial Bus Controller (USB) contains the following registers.

Table 27-9: ADSP-SC58x USB Register List

| Name                 | Description                                        |
|----------------------|----------------------------------------------------|
| USB_BAT_CHG          | Battery Charging Control Register                  |
| USB_CT_HHSRTN        | Host High-Speed Return to Normal Register          |
| USB_CT_HSBT          | High-Speed Timeout Register                        |
| USB_CT_UCH           | Chirp Timeout Register                             |
| USB_DEV_CTL          | Device Control Register                            |
| USB_DMA[n]_ADDR      | DMAChannel n Address Register                      |
| USB_DMA[n]_CNT       | DMAChannel n Count Register                        |
| USB_DMA[n]_CTL       | DMAChannel n Control Register                      |
| USB_DMA_IRQ          | DMAInterrupt Register                              |
| USB_EP0I_CFGDATA[N]  | EP0 Configuration Information Register             |
| USB_EP0I_CNT[N]      | EP0 Number of Received Bytes Register              |
| USB_EP0I_CSR[N]_H    | EP0 Configuration and Status (Host) Register       |
| USB_EP0I_CSR[N]_P    | EP0 Configuration and Status (Peripheral) Register |
| USB_EP0I_NAKLIMIT[N] | EP0 NAK Limit Register                             |
| USB_EP0I_TYPE[N]     | EP0 Connection Type Register                       |
| USB_EP0_CFGDATA[n]   | EP0 Configuration Information Register             |
| USB_EP0_CNT[n]       | EP0 Number of Received Bytes Register              |
| USB_EP0_CSR[n]_H     | EP0 Configuration and Status (Host) Register       |
| USB_EP0_CSR[n]_P     | EP0 Configuration and Status (Peripheral) Register |
| USB_EP0_NAKLIMIT[n]  | EP0 NAK Limit Register                             |
| USB_EP0_TYPE[n]      | EP0 Connection Type Register                       |
| USB_EPINFO           | Endpoint Information Register                      |

Table 27-9: ADSP-SC58x USB Register List (Continued)

| Name                  | Description                                                 |
|-----------------------|-------------------------------------------------------------|
| USB_EPI[N]_RXCNT      | EPn Number of Bytes Received Register                       |
| USB_EPI[N]_RXCSR_H    | EPn Receive Configuration and Status (Host) Register        |
| USB_EPI[N]_RXCSR_P    | EPn Receive Configuration and Status (Peripheral) Register  |
| USB_EPI[N]_RXINTERVAL | EPn Receive Polling Interval Register                       |
| USB_EPI[N]_RXMAXP     | EPn Receive Maximum Packet Length Register                  |
| USB_EPI[N]_RXTYPE     | EPn Receive Type Register                                   |
| USB_EPI[N]_TXCSR_H    | EPn Transmit Configuration and Status (Host) Register       |
| USB_EPI[N]_TXCSR_P    | EPn Transmit Configuration and Status (Peripheral) Register |
| USB_EPI[N]_TXINTERVAL | EPn Transmit Polling Interval Register                      |
| USB_EPI[N]_TXMAXP     | EPn Transmit Maximum Packet Length Register                 |
| USB_EPI[N]_TXTYPE     | EPn Transmit Type Register                                  |
| USB_EP[n]_RXCNT       | EPn Number of Bytes Received Register                       |
| USB_EP[n]_RXCSR_H     | EPn Receive Configuration and Status (Host) Register        |
| USB_EP[n]_RXCSR_P     | EPn Receive Configuration and Status (Peripheral) Register  |
| USB_EP[n]_RXINTERVAL  | EPn Receive Polling Interval Register                       |
| USB_EP[n]_RXMAXP      | EPn Receive Maximum Packet Length Register                  |
| USB_EP[n]_RXTYPE      | EPn Receive Type Register                                   |
| USB_EP[n]_TXCSR_H     | EPn Transmit Configuration and Status (Host) Register       |
| USB_EP[n]_TXCSR_P     | EPn Transmit Configuration and Status (Peripheral) Register |
| USB_EP[n]_TXINTERVAL  | EPn Transmit Polling Interval Register                      |
| USB_EP[n]_TXMAXP      | EPn Transmit Maximum Packet Length Register                 |
| USB_EP[n]_TXTYPE      | EPn Transmit Type Register                                  |
| USB_FADDR             | Function Address Register                                   |
| USB_FIFOB[n]          | FIFO Byte (8-Bit) Register                                  |
| USB_FIFOH[n]          | FIFO Half-Word (16-Bit) Register                            |
| USB_FIFO[n]           | FIFO Word (32-Bit) Register                                 |
| USB_FRAME             | Frame Number Register                                       |
| USB_FS_EOF1           | Full-Speed EOF 1 Register                                   |
| USB_HS_EOF1           | High-Speed EOF 1 Register                                   |
| USB_IDCTL             | ID Control                                                  |
| USB_IEN               | Common Interrupts Enable Register                           |

Table 27-9: ADSP-SC58x USB Register List (Continued)

| Name                 | Description                            |
|----------------------|----------------------------------------|
| USB_INDEX            | Index Register                         |
| USB_INTRRX           | Receive Interrupt Register             |
| USB_INTRRXE          | Receive Interrupt Enable Register      |
| USB_INTRTX           | Transmit Interrupt Register            |
| USB_INTRTXE          | Transmit Interrupt Enable Register     |
| USB_IRQ              | Common Interrupts Register             |
| USB_LINKINFO         | Link Information Register              |
| USB_LPM_ATTR         | LPM Attribute Register                 |
| USB_LPM_CTL          | LPM Control Register                   |
| USB_LPM_FADDR        | LPM Function Address Register          |
| USB_LPM_IEN          | LPM Interrupt Enable Register          |
| USB_LPM_IRQ          | LPM Interrupt Status Register          |
| USB_LS_EOF1          | Low-Speed EOF 1 Register               |
| USB_MP[n]_RXFUNCADDR | MPn Receive Function Address Register  |
| USB_MP[n]_RXHUBADDR  | MPn Receive Hub Address Register       |
| USB_MP[n]_RXHUBPORT  | MPn Receive Hub Port Register          |
| USB_MP[n]_TXFUNCADDR | MPn Transmit Function Address Register |
| USB_MP[n]_TXHUBADDR  | MPn Transmit Hub Address Register      |
| USB_MP[n]_TXHUBPORT  | MPn Transmit Hub Port Register         |
| USB_PHY_CTL          | PHY Control Register                   |
| USB_PLL_OSC          | PLL and Oscillator Control Register    |
| USB_POWER            | Power and Device Control Register      |
| USB_RAMINFO          | RAM Information Register               |
| USB_RQPKTCNT[n]      | EPn Request Packet Count Register      |
| USB_RXFIFOADDR       | Receive FIFO Address Register          |
| USB_RXFIFOSZ         | Receive FIFO Size Register             |
| USB_SOFT_RST         | Software Reset Register                |
| USB_TESTMODE         | Testmode Register                      |
| USB_TXFIFOADDR       | Transmit FIFO Address Register         |
| USB_TXFIFOSZ         | Transmit FIFO Size Register            |
| USB_VBUS_CTL         | VBUS Control Register                  |

Table 27-9: ADSP-SC58x USB Register List (Continued)

| Name      | Description                |
|-----------|----------------------------|
| USB_VPLEN | VBUS Pulse Length Register |

## Battery Charging Control Register

The USB\_BAT\_CHG register controls USB controller battery change-related features.

Figure 27-34: USB\_BAT\_CHG Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000033_e33a02bfde14d85dc24d00842461636dd3854bf5693b7aa1f2ca4246cf409cd9.png)

Table 27-10: USB\_BAT\_CHG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DEDCHG     | Dedicated Charging Port. The USB_BAT_CHG.DEDCHG bit is asserted if both D+ and D- are high. This can be used to determine if the attached device is a dedicated charging port. This bit is the decode of LineState[1] and LineState[0]. This bit is only valid when a session is initiat- ed, which enables a pullup on D+ when acting as a B-device. |
| 3 (R/NW)           | CHGDET     | Charging Port Detected. The USB_BAT_CHG.CHGDET bit indicates when a charging port is detected. This bit indicates that D+/- is above V DAT_REF and below V LGC .                                                                                                                                                                                      |
| 2 (R/W)            | SNSCHGDET  | Sense Charger Detection. The USB_BAT_CHG.SNSCHGDET bit enables charger detection. Setting this bit en- ables VD_SRC and ID_SINK.                                                                                                                                                                                                                      |
| 1 (R/NW)           | CONDET     | Connected Detected. The USB_BAT_CHG.CONDET bit is valid when USB_BAT_CHG.SNSCONDET is enabled. This bit reflects the inverse of D+ (!LineState[0]). If nothing is connected, D+ is pulled high. If a charger or USB port is connected, D+ is pulled low.                                                                                              |
| 0 (R/W)            | SNSCONDET  | Sense Connection Detection. The USB_BAT_CHG.SNSCONDET bit enables connection detection. Enabling this bit enables I DP_SRC and R DM_DWN .                                                                                                                                                                                                             |

## Host High-Speed Return to Normal Register

The USB\_CT\_HHSRTN register selects the delay from the end of the high-speed resume signaling (acting as a host) to the return to normal mode operation. This value is multiplied by 4 times the XCLK period (or 16.7 ns). The default setting corresponds to a delay of 100us.

Figure 27-35: USB\_CT\_HHSRTN Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000034_ae6e4664695640a922fbcb4f8dbd7d679d6e82f104a6185c102b42c24411d3c6.png)

Table 27-11: USB\_CT\_HHSRTN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                 |
|--------------------|------------|-----------------------------------------|
| 14:0               | VALUE      | Host High-Speed Return to Normal Value. |

## High-Speed Timeout Register

The USB\_CT\_HSBT register selects an amount of time to add to the minimum high-speed timeout in units of 64 bit times. The USB 2.0 specification section 7.1.19.2 states that the controller must not timeout less than 736 bit times and must timeout after 816 bit times. The value in USB\_CT\_HSBT is multiplied by 64-bit times and added to the minimum 736 bit times. Settings less than 1 violate the USB 2.0 specification, making the controller noncompliant.

Figure 27-36: USB\_CT\_HSBT Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000035_1c4852af419da485d022a8daab80aade6c119ce6e63a109131be0609bfb4aa0d.png)

Table 27-12: USB\_CT\_HSBT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | VALUE      | HS Timeout Adder. The USB_CT_HSBT.VALUE bits selects an amount of time to add to the minimum high-speed timeout in units of 64 bit times. |
| 3:0 (R/W)          | VALUE      | 0 HS Timeout = 736 (bit time)                                                                                                             |
| 3:0 (R/W)          | VALUE      | 1 HS Timeout = 800 (bit time)                                                                                                             |
| 3:0 (R/W)          | VALUE      | 2 HS Timeout = 864 (bit time)                                                                                                             |

## Chirp Timeout Register

The USB\_CT\_UCH register selects chirp timeout value. The value is multiplied by 4 times the XCLK period (or 67ns). The default setting is 1.1ms.

Figure 27-37: USB\_CT\_UCH Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000036_da896cefd66a61932e2aaf4645e4adf7145101229ad070bf91909b1ff4018aea.png)

Table 27-13: USB\_CT\_UCH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                   |
|--------------------|------------|-----------------------------------------------------------|
| 14:0               | VALUE      | Chirp Timeout Value.                                      |
| (R/W)              |            | The USB_CT_UCH.VALUE bits select the chirp timeout value. |

## Device Control Register

The USB\_DEV\_CTL register selects whether the USB controller is operating in peripheral mode or in host mode. It is used for controlling and monitoring the VBUS line.

Figure 27-38: USB\_DEV\_CTL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000037_0536d5ca0bf383aac7b512bca51c97c8e6b09c2607cde73fc8f82881c66c6f0f.png)

Table 27-14: USB\_DEV\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | BDEVICE    | A or B Devices Indicator. The USB_DEV_CTL.BDEVICE bit indicates whether the USB controller is operat- ing as the A device or the B device. This bit is only valid while a session is in progress.                                                                                                                               | A or B Devices Indicator. The USB_DEV_CTL.BDEVICE bit indicates whether the USB controller is operat- ing as the A device or the B device. This bit is only valid while a session is in progress.                                                                                                                               |
| 7 (R/NW)           | BDEVICE    | 0                                                                                                                                                                                                                                                                                                                               | A Device Detected                                                                                                                                                                                                                                                                                                               |
| 7 (R/NW)           | BDEVICE    | 1                                                                                                                                                                                                                                                                                                                               | B Device Detected                                                                                                                                                                                                                                                                                                               |
| 6 (R/NW)           | FSDEV      | Full or High-Speed Indicator. The USB_DEV_CTL.FSDEV bit is set when a full-speed or high-speed device is de- tected being connected to the port. High-speed devices are distinguished from full- speed by checking for high-speed chirps when the device detects a USB controller re- set. This bit is only valid in host mode. | Full or High-Speed Indicator. The USB_DEV_CTL.FSDEV bit is set when a full-speed or high-speed device is de- tected being connected to the port. High-speed devices are distinguished from full- speed by checking for high-speed chirps when the device detects a USB controller re- set. This bit is only valid in host mode. |
| 6 (R/NW)           | FSDEV      | 0                                                                                                                                                                                                                                                                                                                               | Not Detected                                                                                                                                                                                                                                                                                                                    |
| 6 (R/NW)           | FSDEV      | 1                                                                                                                                                                                                                                                                                                                               | Full or High-Speed Detected                                                                                                                                                                                                                                                                                                     |
| 5 (R/NW)           | LSDEV      | Low-Speed Indicator. The USB_DEV_CTL.LSDEV bit is set when a low-speed device is detected being connected to the port. This bit is only valid in host mode.                                                                                                                                                                     | Low-Speed Indicator. The USB_DEV_CTL.LSDEV bit is set when a low-speed device is detected being connected to the port. This bit is only valid in host mode.                                                                                                                                                                     |
| 5 (R/NW)           | LSDEV      | 0                                                                                                                                                                                                                                                                                                                               | Not Detected                                                                                                                                                                                                                                                                                                                    |
| 5 (R/NW)           | LSDEV      | 1                                                                                                                                                                                                                                                                                                                               | Low-Speed Detected                                                                                                                                                                                                                                                                                                              |

Table 27-14: USB\_DEV\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:3 (R/NW)         | VBUS       | VBUS Level Indicator. The USB_DEV_CTL.VBUS bits indicated the current VBUS level.                                                                                                                                                                                                                                                                                                                                                                                                                 | VBUS Level Indicator. The USB_DEV_CTL.VBUS bits indicated the current VBUS level.                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4:3 (R/NW)         | VBUS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Below SessionEnd                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 4:3 (R/NW)         | VBUS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Above SessionEnd, below AValid                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 4:3 (R/NW)         | VBUS       | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Above AValid, below VBUSValid                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 4:3 (R/NW)         | VBUS       | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Above VBUSValid                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 2 (R/NW)           | HOSTMODE   | Host Mode Indicator. The USB_DEV_CTL.HOSTMODE bit is set when the USB controller is acting as a host.                                                                                                                                                                                                                                                                                                                                                                                             | Host Mode Indicator. The USB_DEV_CTL.HOSTMODE bit is set when the USB controller is acting as a host.                                                                                                                                                                                                                                                                                                                                                                                             |
| 2 (R/NW)           | HOSTMODE   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Peripheral Mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 2 (R/NW)           | HOSTMODE   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Host Mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 1 (R/W)            | HOSTREQ    | Host Negotiation Request. When the USB_DEV_CTL.HOSTREQ bit is set, the USB controller initiates the host negotiation when suspend mode is entered. This bit is cleared when host negotiation is completed. The USB_DEV_CTL.HOSTREQ bit applies when the USB controller is operating as a B device only.                                                                                                                                                                                           | Host Negotiation Request. When the USB_DEV_CTL.HOSTREQ bit is set, the USB controller initiates the host negotiation when suspend mode is entered. This bit is cleared when host negotiation is completed. The USB_DEV_CTL.HOSTREQ bit applies when the USB controller is operating as a B device only.                                                                                                                                                                                           |
| 1 (R/W)            | HOSTREQ    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | No Request                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 1 (R/W)            | HOSTREQ    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Place Request                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 0 (R/W)            | SESSION    | Session Indicator. When operating as an A device, the USB_DEV_CTL.SESSION bit is set or cleared by the processor core to start or end a session. When operating as a B device, the USB_DEV_CTL.SESSION bit is set or cleared by the USB controller when a session starts or ends. This bit is also set by the processor core to initiate the session request protocol. When the USB controller is in suspend mode, the bit may be cleared by the processor core to perform a software disconnect. | Session Indicator. When operating as an A device, the USB_DEV_CTL.SESSION bit is set or cleared by the processor core to start or end a session. When operating as a B device, the USB_DEV_CTL.SESSION bit is set or cleared by the USB controller when a session starts or ends. This bit is also set by the processor core to initiate the session request protocol. When the USB controller is in suspend mode, the bit may be cleared by the processor core to perform a software disconnect. |
| 0 (R/W)            | SESSION    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Not Detected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 0 (R/W)            | SESSION    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Detected Session                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

## DMA Channel n Address Register

The USB\_DMA[n]\_ADDR register indicates the location in on-chip memory where DMA data is written or read. The address must be aligned to 32-bit words (The lower two address bits are always zero.) This register increments as the DMA transfer progresses.

Figure 27-39: USB\_DMA[n]\_ADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000038_a886041ebf1aeaa42fbaba2ee6d158281c518b6350a6bd81ad1bc8b8d3dea5a7.png)

Table 27-15: USB\_DMA[n]\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | DMAAddress Value. The USB_DMA[n]_ADDR.VALUE bits hold the address value for the location in on-chip memory where DMAdata is written or read. |

## DMA Channel n Count Register

The USB\_DMA[n]\_CNT register holds the DMA count, indicating the number of bytes to be transferred for a given DMA work block. If this field is set to zero, no data is transferred, and an interrupt is generated.

Figure 27-40: USB\_DMA[n]\_CNT Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000039_4905267f791d838d9541bfdd5229aba78ef775e480eb9b05a5312e10ec8539bf.png)

Table 27-16: USB\_DMA[n]\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | DMACount Value. The USB_DMA[n]_CNT.VALUE bits indicate the number of bytes to be transferred for a given DMAwork block. |

## DMA Channel n Control Register

There is a USB\_DMA[n]\_CTL register for each DMA master channel. This register assigns, configures, and controls each endpoint with a corresponding DMA master channel.

Figure 27-41: USB\_DMA[n]\_CTL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000040_97ab9f4ba4e5da1b37bf1011691ac0f89f735507ead5d164318ecad73d72a05f.png)

Table 27-17: USB\_DMA[n]\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:9 (R/W)         | BRSTM      | Burst Mode. The USB_DMA[n]_CTL.BRSTM bits select the type or length of burst transfer used by the corresponding DMAchannel to transfer data.                |
| 10:9 (R/W)         | BRSTM      | 0 Unspecified Length                                                                                                                                        |
| 10:9 (R/W)         | BRSTM      | 1 INCR4 or Unspecified Length                                                                                                                               |
| 10:9 (R/W)         | BRSTM      | 2 INCR8, INCR4, or Unspecified Length                                                                                                                       |
| 10:9 (R/W)         | BRSTM      | 3 INCR16, INCR8, INCR4, or Unspecified Length                                                                                                               |
| 8 (R/W)            | ERR        | Bus Error. The USB_DMA[n]_CTL.ERR bit indicates when a peripheral bus error has been en- countered by the master channel. This bit is cleared by software.  |
| 8 (R/W)            | ERR        | 0 No Status                                                                                                                                                 |
| 8 (R/W)            | ERR        | 1 Bus Error                                                                                                                                                 |
| 7:4 (R/W)          | EP         | DMAChannel Endpoint Assignment. The USB_DMA[n]_CTL.EP bits select the endpoint assignments for theDMA channel. (Enumeration values not shown are reserved.) |
| 7:4 (R/W)          | EP         | 0 Endpoint 0                                                                                                                                                |
| 7:4 (R/W)          | EP         | 1 Endpoint 1                                                                                                                                                |
| 7:4 (R/W)          | EP         | 2 Endpoint 2                                                                                                                                                |
| 7:4 (R/W)          | EP         | 3 Endpoint 3                                                                                                                                                |

Table 27-17: USB\_DMA[n]\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    | 10         | 4 Endpoint 4                                                                                                                                                                                         |
|                    | 5          | Endpoint 5                                                                                                                                                                                           |
|                    | 6          | Endpoint 6                                                                                                                                                                                           |
|                    | 7          | Endpoint 7                                                                                                                                                                                           |
|                    | 8          | Endpoint 8                                                                                                                                                                                           |
|                    | 9          | Endpoint 9                                                                                                                                                                                           |
|                    |            | Endpoint 10                                                                                                                                                                                          |
|                    | 11         | Endpoint 11                                                                                                                                                                                          |
|                    | 12         | Endpoint 12                                                                                                                                                                                          |
|                    | 13         | Endpoint 13                                                                                                                                                                                          |
|                    | 14         | Endpoint 14                                                                                                                                                                                          |
|                    | 15         | Endpoint 15                                                                                                                                                                                          |
| 3 (R/W)            | IE         | DMAInterrupt Enable. The USB_DMA[n]_CTL.IE bit enables DMAinterrupts for the DMAchannel, en- abling operation of the channels corresponding bit in the USB_DMA_IRQ register.                         |
| 3 (R/W)            | 0          | Disable Interrupt                                                                                                                                                                                    |
| 3 (R/W)            | 1          | Enable Interrupt                                                                                                                                                                                     |
| 2 (R/W)            | MODE       | DMAMode. The USB_DMA[n]_CTL.MODE bit selects whether the DMAchannel operates in DMAmode 0 or operates in DMAmode 1. Note that DMAmode 1 may only be used with bulk endpoints.                        |
| 2 (R/W)            | 0          | DMAMode 0                                                                                                                                                                                            |
| 2 (R/W)            | 1          | DMAMode 1                                                                                                                                                                                            |
| 1 (R/W)            | DIR        | DMATransfer Direction. The USB_DMA[n]_CTL.DIR bit selects the DMAchannel transfer direction, which must be selected for use with receive endpoints (DMA write=0) or transmit endpoints (DMA read=1). |
| 1 (R/W)            | 0          | DMAWrite (for Rx Endpoint)                                                                                                                                                                           |
| 1 (R/W)            | 1          | DMARead (for Tx Endpoint)                                                                                                                                                                            |

Table 27-17: USB\_DMA[n]\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 0                  | EN         | DMAEnable. The USB_DMA[n]_CTL.EN bit enables the DMAchannel to start the |
| (R/W)              |            | DMAtrans- fer.                                                           |
|                    |            | 0 DisableDMA                                                             |
|                    |            | 1 Enable DMA(Start Transfer)                                             |

## DMA Interrupt Register

The USB\_DMA\_IRQ register indicates which of the DMA master channels have a pending interrupt. The USB controller generates the interrupt when the corresponding DMA count register ( USB\_DMA[n]\_CNT ) reaches zero. The USB controller clears this register when it is read.

Figure 27-42: USB\_DMA\_IRQ Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000041_a91d191bfe5591eca84be4c856e44ff36fdc71e4dc8e3d39d541451372f5222a.png)

Table 27-18: USB\_DMA\_IRQ Register Fields

| Bit No. (Access)   | Bit Name                                                                                 | Description/Enumeration                                                                                                                            |
|--------------------|------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (RC/NW)          | D7                                                                                       | DMA7Interrupt Pending Status. The USB_DMA_IRQ.D7 indicates whether there is a DMA7interrupt pending. 0 No Pending Interrupt                        |
| 6 (RC/NW)          | D6                                                                                       | DMA6Interrupt Pending Status. The USB_DMA_IRQ.D6 indicates whether there is a DMA6interrupt pending. 0 No Pending Interrupt                        |
| 5 (RC/NW)          | D5                                                                                       | DMA5Interrupt Pending Status. The USB_DMA_IRQ.D5 indicates whether there is a DMA5interrupt pending. 0 No Pending Interrupt 1 Pending DMAInterrupt |
| 4 (RC/NW)          | D4 DMA4Interrupt Pending Status. The USB_DMA_IRQ.D4 indicates whether there 0 No Pending | is a DMA4interrupt pending. Interrupt 1 Pending DMAInterrupt                                                                                       |

Table 27-18: USB\_DMA\_IRQ Register Fields (Continued)

| Bit No. (Access)   | Description/Enumeration                                                                                                     |
|--------------------|-----------------------------------------------------------------------------------------------------------------------------|
| 3 (RC/NW)          | DMA3Interrupt Pending Status. The USB_DMA_IRQ.D3 indicates whether there is a DMA3interrupt pending. 0 No Pending Interrupt |
| 2 (RC/NW)          | DMA2Interrupt Pending Status. The USB_DMA_IRQ.D2 indicates whether there is a DMA2interrupt pending.                        |
| 1 (RC/NW)          | DMA1Interrupt Pending Status. The USB_DMA_IRQ.D1 indicates whether there is a DMA1interrupt pending. 0 No Pending Interrupt |
| 0 (RC/NW)          | DMA0Interrupt Pending Status. The USB_DMA_IRQ.D0 indicates whether there is a DMA0interrupt pending. 0 No Pending Interrupt |

## EP0 Configuration Information Register

The USB\_EP0I\_CFGDATA[N] register describes the USB controller hardware configuration. This register only exists for endpoint 0.

Figure 27-43: USB\_EP0I\_CFGDATA[N] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000042_1ceb6a5de8468790f8f813cc38a020cf1c52b4f539193f76afa64e937ca43a73.png)

Table 27-19: USB\_EP0I\_CFGDATA[N] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R0/NW)          | MPRX       | Multi-Packet Aggregate for Rx Enable. The USB_EP0I_CFGDATA[N].MPRX bit indicates whether the USB controller ag- gregates receive packets into bulk packets before the processor core reads the data.                                                                                                                                    |
| 6 (R0/NW)          | MPTX       | Multi-Packet Split for Tx Enable. The USB_EP0I_CFGDATA[N].MPTX bit indicates whether the USB controller permits transmit of large packets through writing to bulk endpoints. The USB control- ler splits the transmit data into packets, which are appropriately sized for transmit. 0 No Split Tx Bulk Packets 1 Split Tx Bulk Packets |
| 5 (R0/NW)          | BIGEND     | Big Endian Data. The USB_EP0I_CFGDATA[N].BIGEND bit indicates whether the USB controller uses big endian configuration or little endian configuration. 0 Little Endian Configuration                                                                                                                                                    |

Table 27-19: USB\_EP0I\_CFGDATA[N] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R1/NW)          | HBRX       | High-Bandwidth Rx Enable. The USB_EP0I_CFGDATA[N].HBRX bit indicates whether the USB controller supports high-bandwidth receive ISO endpoint.                                                                                                                                                                                                                  |
| 3 (R1/NW)          | HBTX       | High-Bandwidth Tx Enable. The USB_EP0I_CFGDATA[N].HBTX bit indicates whether the USB controller supports high-bandwidth transmit ISO endpoint. 0 No High-Bandwidth Tx                                                                                                                                                                                          |
| 2 (R1/NW)          | DYNFIFO    | Dynamic FIFO Size Enable. The USB_EP0I_CFGDATA[N].DYNFIFO bit indicates whether the USB control- ler uses dynamic FIFO size support (on products supporting this feature), enabling the dynamic FIFO registers. These registers are accessed using the configuration set in the endpoints indexed FIFO size and FIFO address registers, except for endpoint 0. |
| 1 (R1/NW)          | SOFTCON    | 1 Dynamic FIFO Size Soft Connect Enable. The USB_EP0I_CFGDATA[N].SOFTCON bit indicates whether the USB control- ler uses soft connect. 0 No Soft Connect                                                                                                                                                                                                       |
| 0 (R0/W)           | UTMIWID    | 1 Soft Connect UTMI Data Width. The USB_EP0I_CFGDATA[N].UTMIWID bit indicates whether the USB control- ler uses an 8-bit or 16-bit UTMI data width.                                                                                                                                                                                                            |
|                    |            | 0 8-bit UTMI Data Width                                                                                                                                                                                                                                                                                                                                        |

## EP0 Number of Received Bytes Register

The USB\_EP0I\_CNT[N] register indicates the number of received data bytes in the endpoint 0 FIFO. The value returned changes as the contents of the FIFO change. It is only valid while the

USB\_EP0\_CSR[n]\_H.RXPKTRDY bit or USB\_EP0\_CSR[n]\_P.RXPKTRDY bit is set.

Figure 27-44: USB\_EP0I\_CNT[N] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000043_7e34f7b19895bfc566fb4c24474ad420218ea4ed1948f2b8191ae2f2b9a91b14.png)

Table 27-20: USB\_EP0I\_CNT[N] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/NW)         | RXCNT      | Rx Byte Count Value. The USB_EP0I_CNT[N].RXCNT bits holds the number of data bytes currently in line ready to be read from the Rx FIFO. The value returned changes as the FIFO is unloaded. It is only valid while the USB_EP0_CSR[n]_H.RXPKTRDY bit or USB_EP0_CSR[n]_P.RXPKTRDY bit is set. |

## EP0 Configuration and Status (Host) Register

The USB\_EP0I\_CSR[N]\_H register provides control and status bits for endpoint 0 in host mode. Note that some bits may be set to clear automatically.

Figure 27-45: USB\_EP0I\_CSR[N]\_H Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000044_6682325e1c4bfc081541fcfa9d171efa7f24f23c6b95bd81cc68be1080f21c39.png)

Table 27-21: USB\_EP0I\_CSR[N]\_H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | DISPING    | Disable Ping. The USB_EP0I_CSR[N]_H.DISPING bit disables (in host mode) high-speed PING tokens for the data and status phases of a control transfer. 0 Issue PING tokens                                                                                                                            |
| 10 (R/W)           | DATGLEN    | Data Toggle Write Enable. The USB_EP0I_CSR[N]_H.DATGLEN bit enables (in host mode) the USB con- troller to write the current state of the endpoint 0 USB_EP0I_CSR[N]_H.DATGL bit. This bit is automatically cleared once the new value is written. 0 Disable Write to DATGL 1 Enable Write to DATGL |

Table 27-21: USB\_EP0I\_CSR[N]\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | DATGL      | Data Toggle. The USB_EP0I_CSR[N]_H.DATGL bit indicates (in host mode) the current state of the endpoint 0 data toggle. If D10 is high, this bit may be written with the required setting of the data toggle. If D10 is low, any value written to this bit is ignored. This bit is only used in host mode.                                                              |
| 9 (R/W)            | DATGL      | 0 DATA0 is Set                                                                                                                                                                                                                                                                                                                                                         |
| 9 (R/W)            | DATGL      | 1 DATA1 is Set                                                                                                                                                                                                                                                                                                                                                         |
| 8 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EP0I_CSR[N]_H.FLUSHFIFO bit directs (in host mode) the USB controller to flush data from the endpoint 0 FIFO and clear the USB_EP0I_CSR[N]_H.TXPKTRDY and USB_EP0I_CSR[N]_H.RXPKTRDY bits. The USB_EP0I_CSR[N]_H.FLUSHFIFO bit should only be set if the USB_EP0I_CSR[N]_H.TXPKTRDY and USB_EP0I_CSR[N]_H.RXPKTRDY bits are set.          |
| 8 (R/W)            | FLUSHFIFO  | 0 No Flush                                                                                                                                                                                                                                                                                                                                                             |
| 8 (R/W)            | FLUSHFIFO  | 1 Flush Endpoint FIFO                                                                                                                                                                                                                                                                                                                                                  |
| 7 (R/W0C)          | NAKTO      | NAK Timeout. The USB_EP0I_CSR[N]_H.NAKTO bit indicates (in host mode) when endpoint 0 is halted following the receipt of NAK responses for longer than the time set by the USB_EP0_NAKLIMIT[n] register. The processor core should clear this bit to allow the endpoint to continue.                                                                                   |
| 7 (R/W0C)          | NAKTO      | 0 No Status                                                                                                                                                                                                                                                                                                                                                            |
| 7 (R/W0C)          | NAKTO      | 1 Endpoint Halted (NAK Timeout)                                                                                                                                                                                                                                                                                                                                        |
| 6 (R/W)            | STATUSPKT  | Status Packet. The USB_EP0I_CSR[N]_H.STATUSPKT bit directs (in host mode) the USB controller to perform a status stage transaction. This bit is set at the same time as the USB_EP0I_CSR[N]_H.TXPKTRDY and USB_EP0I_CSR[N]_H.RXPKTRDY bits. Setting this bit ensures that the data toggle is set to 1 so that a DATA1 packet is used for the status stage transaction. |
| 6 (R/W)            | STATUSPKT  | 0 No Request                                                                                                                                                                                                                                                                                                                                                           |
| 6 (R/W)            | STATUSPKT  | 1 Request Status Transaction                                                                                                                                                                                                                                                                                                                                           |

Table 27-21: USB\_EP0I\_CSR[N]\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | REQPKT     | Request Packet. The USB_EP0I_CSR[N]_H.REQPKT bit directs (in host mode) the USB control- ler to request an IN transaction. This bit is cleared when the USB_EP0I_CSR[N]_H.RXPKTRDY bit is set.                                                                                             |
| 5 (R/W)            | REQPKT     | 0 No Request                                                                                                                                                                                                                                                                               |
| 4 (R/W0C)          | TOERR      | Timeout Error. The USB_EP0I_CSR[N]_H.TOERR bit indicates (in host mode) when three at- tempts have been made to perform a transaction with no response from the peripheral. The processor core should clear this bit. An interrupt is generated when this bit is set.                      |
| 4 (R/W0C)          | TOERR      | 0 No Status                                                                                                                                                                                                                                                                                |
| 4 (R/W0C)          | TOERR      | 1 Timeout Error                                                                                                                                                                                                                                                                            |
| 3                  | SETUPPKT   | Setup Packet.                                                                                                                                                                                                                                                                              |
| 3                  | SETUPPKT   | 0 No Request                                                                                                                                                                                                                                                                               |
| 3                  | SETUPPKT   | 1 Send SETUP token                                                                                                                                                                                                                                                                         |
| 2 (R/W0C)          | RXSTALL    | Rx Stall. The USB_EP0I_CSR[N]_H.RXSTALL bit indicates (in host mode) when a STALL handshake is received. The processor core should clear this bit.                                                                                                                                         |
| 2 (R/W0C)          | RXSTALL    | 0 No Status                                                                                                                                                                                                                                                                                |
| 2 (R/W0C)          | RXSTALL    | 1 Stall Received from Device                                                                                                                                                                                                                                                               |
| 1 (R/W1S)          | TXPKTRDY   | Tx Packet Ready. The USB_EP0I_CSR[N]_H.TXPKTRDY bit should be set (in host mode) by the processor core after loading a data packet into the FIFO. This bit is cleared automati- cally when the data packet is transmitted. An interrupt is generated (if enabled) when the bit is cleared. |
| 1 (R/W1S)          | TXPKTRDY   | 0 No Tx Packet                                                                                                                                                                                                                                                                             |
| 1 (R/W1S)          | TXPKTRDY   | 1 Tx Packet in Endpoint FIFO                                                                                                                                                                                                                                                               |

Table 27-21: USB\_EP0I\_CSR[N]\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W0C)          | RXPKTRDY   | Rx Packet Ready. The USB_EP0I_CSR[N]_H.RXPKTRDY is set (in host mode) when a data packet is received. An interrupt is generated (if enabled) when this bit is set. The processor core should clear this bit when the packet is read from the FIFO. |
| 0 (R/W0C)          | RXPKTRDY   | 0 No Rx Packet                                                                                                                                                                                                                                     |
| 0 (R/W0C)          | RXPKTRDY   | 1 Rx Packet in Endpoint FIFO                                                                                                                                                                                                                       |

## EP0 Configuration and Status (Peripheral) Register

The USB\_EP0I\_CSR[N]\_P register provides control and status bits for endpoint 0 in peripheral mode. Note that some bits may be set to clear automatically.

Figure 27-46: USB\_EP0I\_CSR[N]\_P Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000045_21767f4db6c7dbee4f06570e72811cee596a7c42f67ca593bf8fe41f4162a504.png)

Table 27-22: USB\_EP0I\_CSR[N]\_P Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EP0I_CSR[N]_P.FLUSHFIFO bit directs (in peripheral mode) the USB controller to flush data from the endpoint 0 FIFO and clear the USB_EP0I_CSR[N]_P.TXPKTRDY and USB_EP0I_CSR[N]_P.RXPKTRDY bits. The USB_EP0I_CSR[N]_P.FLUSHFIFO bit should only be set if the USB_EP0I_CSR[N]_P.TXPKTRDY and USB_EP0I_CSR[N]_P.RXPKTRDY bits are set. Note that setting this bit at other times may cause data corruption. 0 No Flush |
| 7 (R/W)            | SSETUPEND  | Service Setup End. The USB_EP0I_CSR[N]_P.SSETUPEND bit is set (in peripheral mode) by the processor core to clear the USB_EP0I_CSR[N]_P.SETUPEND . This bit is cleared automatically. Action                                                                                                                                                                                                                                                        |
|                    |            | 0 No                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |            | 1 Clear SETUPEND Bit                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 27-22: USB\_EP0I\_CSR[N]\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | SPKTRDY    | Service Rx Packet Ready. The USB_EP0I_CSR[N]_P.SPKTRDY bit is set (in peripheral mode) by the pro- cessor core to clear the USB_EP0I_CSR[N]_P.RXPKTRDY bit. This bit is cleared automatically.                                                                                                                                                                                                                             |
| 6 (R/W)            | SPKTRDY    | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                                |
| 6 (R/W)            | SPKTRDY    | 1 Clear RXPKTRDY Bit                                                                                                                                                                                                                                                                                                                                                                                                       |
| 5 (R/W)            | SENDSTALL  | Send Stall. The USB_EP0I_CSR[N]_P.SENDSTALL bit is set (in peripheral mode) by the processor core to terminate the current transaction. The STALL handshake is transmit- ted, then this bit automatically is cleared.                                                                                                                                                                                                      |
| 5 (R/W)            | SENDSTALL  | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                                |
| 5 (R/W)            | SENDSTALL  | 1 Terminate Current Transaction                                                                                                                                                                                                                                                                                                                                                                                            |
| 4 (R/NW)           | SETUPEND   | Setup End. The USB_EP0I_CSR[N]_P.SETUPEND bit indicates (in peripheral mode) when a control transaction ends before the USB_EP0I_CSR[N]_P.DATAEND bit is set. An interrupt is generated and the FIFO is flushed at this time. This bit is cleared when the processor core sets the USB_EP0I_CSR[N]_P.SSETUPEND bit.                                                                                                        |
| 4 (R/NW)           | SETUPEND   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                |
| 4 (R/NW)           | SETUPEND   | 1 Setup Ended before DATAEND                                                                                                                                                                                                                                                                                                                                                                                               |
| 3 (R/W1S)          | DATAEND    | Data End. The USB_EP0I_CSR[N]_P.DATAEND bit is set (in peripheral mode) by the pro- cessor core sets when the core: • Sets the USB_EP0I_CSR[N]_P.TXPKTRDY bit for the last data packet. • Clears the USB_EP0I_CSR[N]_P.RXPKTRDY bit after unloading the last da- ta packet. • Sets the USB_EP0I_CSR[N]_P.TXPKTRDY bit for a zero-length data packet. The USB_EP0I_CSR[N]_P.DATAEND bit is cleared automatically. No Status |
| 3 (R/W1S)          | DATAEND    | 0                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 3 (R/W1S)          | DATAEND    | 1 Data End Condition                                                                                                                                                                                                                                                                                                                                                                                                       |
| 2 (R/W0C)          | SENTSTALL  | Sent Stall. The USB_EP0I_CSR[N]_P.SENTSTALL bit is set (in peripheral mode) when a STALL handshake is transmitted. The processor core should clear this bit.                                                                                                                                                                                                                                                               |
| 2 (R/W0C)          | SENTSTALL  | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                |
| 2 (R/W0C)          | SENTSTALL  | 1 Transmitted STALL Handshake                                                                                                                                                                                                                                                                                                                                                                                              |

Table 27-22: USB\_EP0I\_CSR[N]\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1S)          | TXPKTRDY   | Tx Packet Ready. The USB_EP0I_CSR[N]_P.TXPKTRDY bit should be set (in peripheral mode) by the processor core after loading a data packet into the FIFO. This bit is cleared auto- matically when the data packet is transmitted. An interrupt is generated (if enabled) when the bit is cleared. 0 |
| 0 (R/W0C)          | RXPKTRDY   | Rx Packet Ready. The USB_EP0I_CSR[N]_P.RXPKTRDY is set (in peripheral mode) when a data packet is received. An interrupt is generated (if enabled) when this bit is set. The pro- cessor core clears this bit by setting the USB_EP0I_CSR[N]_P.SPKTRDY bit. 0 No Rx Packet                         |
| 0 (R/W0C)          | RXPKTRDY   | 1 Rx Packet in Endpoint FIFO                                                                                                                                                                                                                                                                       |
| 0 (R/W0C)          | RXPKTRDY   |                                                                                                                                                                                                                                                                                                    |

## EP0 NAK Limit Register

The USB\_EP0I\_NAKLIMIT[N] register determines the number of frames/micro-frames after which endpoint 0 should timeout on receiving a stream of NAK responses for bulk endpoints.

Figure 27-47: USB\_EP0I\_NAKLIMIT[N] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000046_ed649f70a8c8aea564d69804a2d861f02adcb6bb72de588fb224f19e8cbdaeb3.png)

Table 27-23: USB\_EP0I\_NAKLIMIT[N] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | VALUE      | Endpoint 0 Timeout Value (in Frames). The USB_EP0I_NAKLIMIT[N].VALUE bits hold the endpoint 0 timeout value (number of frames). |

## EP0 Connection Type Register

The USB\_EP0I\_TYPE[N] register selects the USB controller operating speed for endpoint 0 when acting as a host connected to devices through a hub.

Figure 27-48: USB\_EP0I\_TYPE[N] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000047_8d45f9a78aa55cc4745861c2af80f04fe8db89186226a3c8d60d8d9e9736c0a3.png)

Table 27-24: USB\_EP0I\_TYPE[N] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1:0 (R/W)          | SPEED      | Speed of Operation Value. The USB_EP0I_TYPE[N].SPEED bits select the USB controller operating speed for endpoint 0 when acting as a host connected to devices through a hub. In these in- stances. the USB controller must issue split transactions under certain conditions. If a device is directly connected (not through a hub), all endpoints use the same speed as which the controller is connected. When not connected to devices through a hub, pro- gram this field with 00. | Speed of Operation Value. The USB_EP0I_TYPE[N].SPEED bits select the USB controller operating speed for endpoint 0 when acting as a host connected to devices through a hub. In these in- stances. the USB controller must issue split transactions under certain conditions. If a device is directly connected (not through a hub), all endpoints use the same speed as which the controller is connected. When not connected to devices through a hub, pro- gram this field with 00. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Same Speed as Processor Core                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | High-Speed                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Full-Speed                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Low-Speed                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

## EP0 Configuration Information Register

The USB\_EP0\_CFGDATA[n] register describes the USB controller hardware configuration. This register only exists for endpoint 0.

Figure 27-49: USB\_EP0\_CFGDATA[n] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000048_1ceb6a5de8468790f8f813cc38a020cf1c52b4f539193f76afa64e937ca43a73.png)

Table 27-25: USB\_EP0\_CFGDATA[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R0/NW)          | MPRX       | Multi-Packet Aggregate for Rx Enable. The USB_EP0_CFGDATA[n].MPRX bit indicates whether the USB controller ag- gregates receive packets into bulk packets before the processor core reads the data.                                                                                                                                    |
| 6 (R0/NW)          | MPTX       | Multi-Packet Split for Tx Enable. The USB_EP0_CFGDATA[n].MPTX bit indicates whether the USB controller per- mits transmit of large packets through writing to bulk endpoints. The USB controller splits the transmit data into packets, which are appropriately sized for transmit. 0 No Split Tx Bulk Packets 1 Split Tx Bulk Packets |
| 5 (R0/NW)          | BIGEND     | Big Endian Data. The USB_EP0_CFGDATA[n].BIGEND bit indicates whether the USB controller uses big endian configuration or little endian configuration. 0 Little Endian Configuration                                                                                                                                                    |
| 5 (R0/NW)          | BIGEND     | 1 Big Endian Configuration                                                                                                                                                                                                                                                                                                             |
| 5 (R0/NW)          | BIGEND     |                                                                                                                                                                                                                                                                                                                                        |

Table 27-25: USB\_EP0\_CFGDATA[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R1/NW)          | HBRX       | High-Bandwidth Rx Enable. The USB_EP0_CFGDATA[n].HBRX bit indicates whether the USB controller sup- ports high-bandwidth receive ISO endpoint.                                                                                                                                                                                                                                                         |
| 3 (R1/NW)          | HBTX       | High-Bandwidth Tx Enable. The USB_EP0_CFGDATA[n].HBTX bit indicates whether the USB controller sup- ports high-bandwidth transmit ISO endpoint. 0 No High-Bandwidth Tx                                                                                                                                                                                                                                 |
| 2 (R1/NW)          | DYNFIFO    | 1 High-Bandwidth Tx Dynamic FIFO Size Enable. The USB_EP0_CFGDATA[n].DYNFIFO bit indicates whether the USB controller uses dynamic FIFO size support (on products supporting this feature), enabling the dynamic FIFO registers. These registers are accessed using the configuration set in the endpoints indexed FIFO size and FIFO address registers, except for endpoint 0. 0 No Dynamic FIFO Size |
| 1 (R1/NW)          | SOFTCON    | 1 Dynamic FIFO Size Soft Connect Enable. The USB_EP0_CFGDATA[n].SOFTCON bit indicates whether the USB controller uses soft connect. 0 No Soft Connect                                                                                                                                                                                                                                                  |
| 0 (R0/W)           | UTMIWID    | 1 Soft Connect UTMI Data Width. The USB_EP0_CFGDATA[n].UTMIWID bit indicates whether the USB controller                                                                                                                                                                                                                                                                                                |
|                    | uses       | an 8-bit or 16-bit UTMI data width. 0 8-bit UTMI Data Width                                                                                                                                                                                                                                                                                                                                            |

## EP0 Number of Received Bytes Register

The USB\_EP0\_CNT[n] register indicates the number of received data bytes in the endpoint 0 FIFO. The value returned changes as the contents of the FIFO change. It is only valid while the

USB\_EP0\_CSR[n]\_H.RXPKTRDY bit or USB\_EP0\_CSR[n]\_P.RXPKTRDY bit is set.

Figure 27-50: USB\_EP0\_CNT[n] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000049_266ea89aa9458eff50d4f791a7e5ac1dbab663b166b7a36f2c7406215a13ff6d.png)

Table 27-26: USB\_EP0\_CNT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/NW)         | RXCNT      | Rx Byte Count Value. The USB_EP0_CNT[n].RXCNT bits holds the number of data bytes currently in line ready to be read from the Rx FIFO. The value returned changes as the FIFO is unloaded. It is only valid while the USB_EP0_CSR[n]_H.RXPKTRDY bit or USB_EP0_CSR[n]_P.RXPKTRDY bit is set. |

## EP0 Configuration and Status (Host) Register

The USB\_EP0\_CSR[n]\_H register provides control and status bits for endpoint 0 in host mode. Note that some bits may be set to clear automatically.

Figure 27-51: USB\_EP0\_CSR[n]\_H Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000050_6682325e1c4bfc081541fcfa9d171efa7f24f23c6b95bd81cc68be1080f21c39.png)

Table 27-27: USB\_EP0\_CSR[n]\_H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | DISPING    | Disable Ping. The USB_EP0_CSR[n]_H.DISPING bit disables (in host mode) high-speed PING tokens for the data and status phases of a control transfer. 0 Issue PING tokens                                                                                                                           |
| 10 (R/W)           | DATGLEN    | Data Toggle Write Enable. The USB_EP0_CSR[n]_H.DATGLEN bit enables (in host mode) the USB control- ler to write the current state of the endpoint 0 USB_EP0_CSR[n]_H.DATGL bit. This bit is automatically cleared once the new value is written. 0 Disable Write to DATGL 1 Enable Write to DATGL |

Table 27-27: USB\_EP0\_CSR[n]\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | DATGL      | Data Toggle. The USB_EP0_CSR[n]_H.DATGL bit indicates (in host mode) the current state of the endpoint 0 data toggle. If D10 is high, this bit may be written with the required setting of the data toggle. If D10 is low, any value written to this bit is ignored. This bit is only used in host mode.                                                              |
| 9 (R/W)            | DATGL      | 0 DATA0 is Set                                                                                                                                                                                                                                                                                                                                                        |
| 9 (R/W)            | DATGL      | 1 DATA1 is Set                                                                                                                                                                                                                                                                                                                                                        |
| 8 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EP0_CSR[n]_H.FLUSHFIFO bit directs (in host mode) the USB con- troller to flush data from the endpoint 0 FIFO and clear the USB_EP0_CSR[n]_H.TXPKTRDY and USB_EP0_CSR[n]_H.RXPKTRDY bits. The USB_EP0_CSR[n]_H.FLUSHFIFO bit should only be set if the USB_EP0_CSR[n]_H.TXPKTRDY and USB_EP0_CSR[n]_H.RXPKTRDY bits                      |
| 8 (R/W)            | FLUSHFIFO  | 0 No Flush                                                                                                                                                                                                                                                                                                                                                            |
| 8 (R/W)            | FLUSHFIFO  | 1 Flush Endpoint FIFO                                                                                                                                                                                                                                                                                                                                                 |
| 7 (R/W0C)          | NAKTO      | NAK Timeout. The USB_EP0_CSR[n]_H.NAKTO bit indicates (in host mode) when endpoint 0 is halted following the receipt of NAK responses for longer than the time set by the USB_EP0_NAKLIMIT[n] register. The processor core should clear this bit to allow the endpoint to continue.                                                                                   |
| 7 (R/W0C)          | NAKTO      | 0 No Status                                                                                                                                                                                                                                                                                                                                                           |
| 7 (R/W0C)          | NAKTO      | 1 Endpoint Halted (NAK Timeout)                                                                                                                                                                                                                                                                                                                                       |
| 6 (R/W)            | STATUSPKT  | Status Packet. The USB_EP0_CSR[n]_H.STATUSPKT bit directs (in host mode) the USB con- troller to perform a status stage transaction. This bit is set at the same time as the USB_EP0_CSR[n]_H.TXPKTRDY and USB_EP0_CSR[n]_H.RXPKTRDY bits. Setting this bit ensures that the data toggle is set to 1 so that a DATA1 packet is used for the status stage transaction. |
| 6 (R/W)            | STATUSPKT  | 0 No Request                                                                                                                                                                                                                                                                                                                                                          |
| 6 (R/W)            | STATUSPKT  | 1 Request Status Transaction                                                                                                                                                                                                                                                                                                                                          |

Table 27-27: USB\_EP0\_CSR[n]\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | REQPKT     | Request Packet. The USB_EP0_CSR[n]_H.REQPKT bit directs (in host mode) the USB controller to request an IN transaction. This bit is cleared when the USB_EP0_CSR[n]_H.RXPKTRDY bit is set.                                                                                                |
| 5 (R/W)            | REQPKT     | 0 No Request                                                                                                                                                                                                                                                                              |
| 5 (R/W)            | REQPKT     | 1 Send IN Tokens to Device                                                                                                                                                                                                                                                                |
| 4 (R/W0C)          | TOERR      | Timeout Error. The USB_EP0_CSR[n]_H.TOERR bit indicates (in host mode) when three at- tempts have been made to perform a transaction with no response from the peripheral. The processor core should clear this bit. An interrupt is generated when this bit is set.                      |
| 4 (R/W0C)          | TOERR      | 0 No Status                                                                                                                                                                                                                                                                               |
| 4 (R/W0C)          | TOERR      | 1 Timeout Error                                                                                                                                                                                                                                                                           |
| 3                  | SETUPPKT   | Setup Packet.                                                                                                                                                                                                                                                                             |
| 3                  | SETUPPKT   | troller to send a SETUP token instead of an OUT token for the transaction. This bit is set at the same time as the USB_EP0_CSR[n]_H.TXPKTRDY bit is set. 0 No Request                                                                                                                     |
| 3                  | SETUPPKT   | 1 Send SETUP token                                                                                                                                                                                                                                                                        |
| 2 (R/W0C)          | RXSTALL    | Rx Stall. The USB_EP0_CSR[n]_H.RXSTALL bit indicates (in host mode) when a STALL handshake is received. The processor core should clear this bit.                                                                                                                                         |
| 2 (R/W0C)          | RXSTALL    | 0 No Status                                                                                                                                                                                                                                                                               |
| 2 (R/W0C)          | RXSTALL    | 1 Stall Received from Device                                                                                                                                                                                                                                                              |
| 1 (R/W1S)          | TXPKTRDY   | Tx Packet Ready. The USB_EP0_CSR[n]_H.TXPKTRDY bit should be set (in host mode) by the processor core after loading a data packet into the FIFO. This bit is cleared automati- cally when the data packet is transmitted. An interrupt is generated (if enabled) when the bit is cleared. |
| 1 (R/W1S)          | TXPKTRDY   | 0 No Tx Packet                                                                                                                                                                                                                                                                            |
| 1 (R/W1S)          | TXPKTRDY   | 1 Tx Packet in Endpoint FIFO                                                                                                                                                                                                                                                              |

Table 27-27: USB\_EP0\_CSR[n]\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W0C)          | RXPKTRDY   | Rx Packet Ready. The USB_EP0_CSR[n]_H.RXPKTRDY is set (in host mode) when a data packet is received. An interrupt is generated (if enabled) when this bit is set. The processor core should clear this bit when the packet is read from the FIFO. |
| 0 (R/W0C)          | RXPKTRDY   | 0 No Rx Packet                                                                                                                                                                                                                                    |
| 0 (R/W0C)          | RXPKTRDY   | 1 Rx Packet in Endpoint FIFO                                                                                                                                                                                                                      |

## EP0 Configuration and Status (Peripheral) Register

The USB\_EP0\_CSR[n]\_P register provides control and status bits for endpoint 0 in peripheral mode. Note that some bits may be set to clear automatically.

Figure 27-52: USB\_EP0\_CSR[n]\_P Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000051_21767f4db6c7dbee4f06570e72811cee596a7c42f67ca593bf8fe41f4162a504.png)

Table 27-28: USB\_EP0\_CSR[n]\_P Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EP0_CSR[n]_P.FLUSHFIFO bit directs (in peripheral mode) the USB controller to flush data from the endpoint 0 FIFO and clear the USB_EP0_CSR[n]_P.TXPKTRDY and USB_EP0_CSR[n]_P.RXPKTRDY bits. The USB_EP0_CSR[n]_P.FLUSHFIFO bit should only be set if the USB_EP0_CSR[n]_P.TXPKTRDY and USB_EP0_CSR[n]_P.RXPKTRDY bits are set. Note that setting this bit at other times may cause data corruption. 0 No Flush |
| 7 (R/W)            | SSETUPEND  | Service Setup End. The USB_EP0_CSR[n]_P.SSETUPEND bit is set (in peripheral mode) by the processor core to clear the USB_EP0_CSR[n]_P.SETUPEND . This bit is cleared automatically.                                                                                                                                                                                                                                                           |
|                    |            | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |            | 1 Clear SETUPEND Bit                                                                                                                                                                                                                                                                                                                                                                                                                          |

Table 27-28: USB\_EP0\_CSR[n]\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | SPKTRDY    | Service Rx Packet Ready. The USB_EP0_CSR[n]_P.SPKTRDY bit is set (in peripheral mode) by the pro- cessor core to clear the USB_EP0_CSR[n]_P.RXPKTRDY bit. This bit is cleared automatically.                                                                                                                                                                                                                        |
| 6 (R/W)            | SPKTRDY    | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                         |
| 6 (R/W)            | SPKTRDY    | 1 Clear RXPKTRDY Bit                                                                                                                                                                                                                                                                                                                                                                                                |
| 5 (R/W)            | SENDSTALL  | Send Stall. The USB_EP0_CSR[n]_P.SENDSTALL bit is set (in peripheral mode) by the processor core to terminate the current transaction. The STALL handshake is transmit- ted, then this bit automatically is cleared.                                                                                                                                                                                                |
| 5 (R/W)            | SENDSTALL  | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                         |
| 5 (R/W)            | SENDSTALL  | 1 Terminate Current Transaction                                                                                                                                                                                                                                                                                                                                                                                     |
| 4 (R/NW)           | SETUPEND   | Setup End. The USB_EP0_CSR[n]_P.SETUPEND bit indicates (in peripheral mode) when a control transaction ends before the USB_EP0_CSR[n]_P.DATAEND bit is set. An interrupt is generated and the FIFO is flushed at this time. This bit is cleared when the processor core sets the USB_EP0_CSR[n]_P.SSETUPEND bit.                                                                                                    |
| 4 (R/NW)           | SETUPEND   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                         |
| 4 (R/NW)           | SETUPEND   | 1 Setup Ended before DATAEND                                                                                                                                                                                                                                                                                                                                                                                        |
| 3 (R/W1S)          | DATAEND    | Data End. The USB_EP0_CSR[n]_P.DATAEND bit is set (in peripheral mode) by the pro- cessor core sets when the core: • Sets the USB_EP0_CSR[n]_P.TXPKTRDY bit for the last data packet. • Clears the USB_EP0_CSR[n]_P.RXPKTRDY bit after unloading the last data packet. • Sets the USB_EP0_CSR[n]_P.TXPKTRDY bit for a zero-length data packet. The USB_EP0_CSR[n]_P.DATAEND bit is cleared automatically. No Status |
| 3 (R/W1S)          | DATAEND    | 0                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 3 (R/W1S)          | DATAEND    | 1 Data End Condition                                                                                                                                                                                                                                                                                                                                                                                                |
| 2 (R/W0C)          | SENTSTALL  | Sent Stall. The USB_EP0_CSR[n]_P.SENTSTALL bit is set (in peripheral mode) when a STALL handshake is transmitted. The processor core should clear this bit.                                                                                                                                                                                                                                                         |
| 2 (R/W0C)          | SENTSTALL  | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2 (R/W0C)          | SENTSTALL  | 1 Transmitted STALL Handshake                                                                                                                                                                                                                                                                                                                                                                                       |

Table 27-28: USB\_EP0\_CSR[n]\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1S)          | TXPKTRDY   | Tx Packet Ready. The USB_EP0_CSR[n]_P.TXPKTRDY bit should be set (in peripheral mode) by the processor core after loading a data packet into the FIFO. This bit is cleared auto- matically when the data packet is transmitted. An interrupt is generated (if enabled) when the bit is cleared. 0 |
| 0 (R/W0C)          | RXPKTRDY   | Rx Packet Ready. The USB_EP0_CSR[n]_P.RXPKTRDY is set (in peripheral mode) when a data packet is received. An interrupt is generated (if enabled) when this bit is set. The pro- cessor core clears this bit by setting the USB_EP0_CSR[n]_P.SPKTRDY bit. 0 No Rx Packet                          |
| 0 (R/W0C)          | RXPKTRDY   | 1 Rx Packet in Endpoint FIFO                                                                                                                                                                                                                                                                      |
| 0 (R/W0C)          | RXPKTRDY   |                                                                                                                                                                                                                                                                                                   |

## EP0 NAK Limit Register

The USB\_EP0\_NAKLIMIT[n] register determines the number of frames/micro-frames after which endpoint 0 should timeout on receiving a stream of NAK responses for bulk endpoints.

Figure 27-53: USB\_EP0\_NAKLIMIT[n] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000052_8b28097cc282dae8075e1c3d91b0335131ea42c43caac6189c5b82cb4e820fc7.png)

Table 27-29: USB\_EP0\_NAKLIMIT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | VALUE      | Endpoint 0 Timeout Value (in Frames). The USB_EP0_NAKLIMIT[n].VALUE bits hold the endpoint 0 timeout value (number of frames). |

## EP0 Connection Type Register

The USB\_EP0\_TYPE[n] register selects the USB controller operating speed for endpoint 0 when acting as a host connected to devices through a hub.

Figure 27-54: USB\_EP0\_TYPE[n] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000053_daaf2468d944c7a4bb0a319fc0078e8a71011da1282fdab7c9f602e40d9a05b5.png)

Table 27-30: USB\_EP0\_TYPE[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1:0 (R/W)          | SPEED      | Speed of Operation Value. The USB_EP0_TYPE[n].SPEED bits select the USB controller operating speed for endpoint 0 when acting as a host connected to devices through a hub. In these instan- ces. the USB controller must issue split transactions under certain conditions. If a de- vice is directly connected (not through a hub), all endpoints use the same speed as which the controller is connected. When not connected to devices through a hub, pro- gram this field with 00. | Speed of Operation Value. The USB_EP0_TYPE[n].SPEED bits select the USB controller operating speed for endpoint 0 when acting as a host connected to devices through a hub. In these instan- ces. the USB controller must issue split transactions under certain conditions. If a de- vice is directly connected (not through a hub), all endpoints use the same speed as which the controller is connected. When not connected to devices through a hub, pro- gram this field with 00. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Same Speed as Processor Core                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | High-Speed                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Full-Speed                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Low-Speed                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

## Endpoint Information Register

The USB\_EPINFO register allows read-back of the number of Tx and Rx endpoints available

Figure 27-55: USB\_EPINFO Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000054_a2481ae1ca150166b088b14ed1f59dbdc52476b28c03c5c2526a8a55b3f95cca.png)

Table 27-31: USB\_EPINFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | RXEP       | Rx Endpoints. The USB_EPINFO.RXEP bits indicate the number of receive endpoints. excluding EP0.  |
| 3:0 (R/NW)         | TXEP       | Tx Endpoints. The USB_EPINFO.TXEP bits indicate the number of transmit endpoints, excluding EP0. |

## EPn Number of Bytes Received Register

The USB\_EPI[N]\_RXCNT register indicates the number of received data bytes in the endpoint receive FIFO. The value returned changes as the contents of the FIFO change and is only valid while the USB\_EP[n]\_RXCSR\_H.RXPKTRDY bit or USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit is set.

Figure 27-56: USB\_EPI[N]\_RXCNT Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000055_fef4b3121966c67f3686c71608045dca02599315f1d4636c752cb0870d3cbd59.png)

Table 27-32: USB\_EPI[N]\_RXCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 13:0 (R/NW)        | EPRXCNT    | EP Rx Count. The USB_EPI[N]_RXCNT.EPRXCNT bits hold the number of data bytes ready to be read from the receive FIFO. |

## EPn Receive Configuration and Status (Host) Register

The USB\_EPI[N]\_RXCSR\_H register provides (in host mode) control and status bits for transfers through the currently selected receive endpoint.

Figure 27-57: USB\_EPI[N]\_RXCSR\_H Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000056_634459defcb836051c396482836fea562ca0086320d2eed63bce0db2d7d0715b.png)

Table 27-33: USB\_EPI[N]\_RXCSR\_H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | AUTOCLR    | Auto Clear Enable. The USB_EPI[N]_RXCSR_H.AUTOCLR bit directs (in host mode) the USB con- troller to automatically clear the USB_EPI[N]_RXCSR_H.RXPKTRDY bit when a packet of size USB_EP[n]_RXMAXP bytes has been unloaded from the receive FIFO. When packets of less than the maximum packet size are unloaded, the processor must clear USB_EPI[N]_RXCSR_H.RXPKTRDY manually. When using theDMA to unload the receive FIFO, data is read from the receive FIFO in four byte chunks, regardless of the USB_EP[n]_RXMAXP value. The USB controller auto clears the USB_EPI[N]_RXCSR_H.RXPKTRDY bit as follows. (In the following: Remain- der=(RxMaxP/4), and PktSz-Clearing-RxPktRdy=Actual-Bytes-Read-Packet-Sizes- That-Clear-RxPktRdy.) • Remainder=0, Bytes-Read=RxMaxP , PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1, RxMaxP-2, RxMaxP-3 • Remainder=3, Bytes Read=RxMaxP+1, PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1, RxMaxP-2 • Remainder=2, Bytes Read=RxMaxP+2, PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1 • Remainder=1, Bytes Read=RxMaxP+3, PktSz-Clearing-RxPktRdy=RxMaxP For products supporting high-speed operation, the USB_EPI[N]_RXCSR_H.AUTOCLR bit should not be set for high-bandwidth iso- | Auto Clear Enable. The USB_EPI[N]_RXCSR_H.AUTOCLR bit directs (in host mode) the USB con- troller to automatically clear the USB_EPI[N]_RXCSR_H.RXPKTRDY bit when a packet of size USB_EP[n]_RXMAXP bytes has been unloaded from the receive FIFO. When packets of less than the maximum packet size are unloaded, the processor must clear USB_EPI[N]_RXCSR_H.RXPKTRDY manually. When using theDMA to unload the receive FIFO, data is read from the receive FIFO in four byte chunks, regardless of the USB_EP[n]_RXMAXP value. The USB controller auto clears the USB_EPI[N]_RXCSR_H.RXPKTRDY bit as follows. (In the following: Remain- der=(RxMaxP/4), and PktSz-Clearing-RxPktRdy=Actual-Bytes-Read-Packet-Sizes- That-Clear-RxPktRdy.) • Remainder=0, Bytes-Read=RxMaxP , PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1, RxMaxP-2, RxMaxP-3 • Remainder=3, Bytes Read=RxMaxP+1, PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1, RxMaxP-2 • Remainder=2, Bytes Read=RxMaxP+2, PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1 • Remainder=1, Bytes Read=RxMaxP+3, PktSz-Clearing-RxPktRdy=RxMaxP For products supporting high-speed operation, the USB_EPI[N]_RXCSR_H.AUTOCLR bit should not be set for high-bandwidth iso- |
| 15 (R/W)           | AUTOCLR    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Disable Auto Clear                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 14 (R/W)           | AUTOREQ    | Auto Request Clear Enable. The USB_EPI[N]_RXCSR_H.AUTOREQ bit directs (in host mode) the USB con- troller to automatically clear the USB_EPI[N]_RXCSR_H.REQPKT bit when USB_EPI[N]_RXCSR_H.RXPKTRDY bit is cleared. This bit is automatically cleared when a short packet is received.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Auto Request Clear Enable. The USB_EPI[N]_RXCSR_H.AUTOREQ bit directs (in host mode) the USB con- troller to automatically clear the USB_EPI[N]_RXCSR_H.REQPKT bit when USB_EPI[N]_RXCSR_H.RXPKTRDY bit is cleared. This bit is automatically cleared when a short packet is received.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 14 (R/W)           | AUTOREQ    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Disable Auto Request Clear                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14 (R/W)           | AUTOREQ    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Enable Auto Request Clear                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 13 (R/W)           | DMAREQEN   | DMARequest Enable Rx EP. The USB_EPI[N]_RXCSR_H.DMAREQEN bit enables (in host mode) DMAre- quests for this receive endpoint.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | DMARequest Enable Rx EP. The USB_EPI[N]_RXCSR_H.DMAREQEN bit enables (in host mode) DMAre- quests for this receive endpoint.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 13 (R/W)           | DMAREQEN   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Disable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 13 (R/W)           | DMAREQEN   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Enable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 27-33: USB\_EPI[N]\_RXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W0C)         | PIDERR     | Packet ID Error. The USB_EPI[N]_RXCSR_H.PIDERR bit indicates (in host mode) when a PID error occurs for isochronous transactions. This bit is ignored in host mode for bulk or interrupt transactions.                                                                                                                                                                                                                                                                      |
| 12 (R/W0C)         | PIDERR     | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 12 (R/W0C)         | PIDERR     | 1 PID                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 11 (R/W)           | DMAREQMODE | Error DMAMode Select. The USB_EPI[N]_RXCSR_H.DMAREQMODE bit selects (in host mode) between DMArequest mode 1 or 0. This bit must not be cleared the cycle before or the same cycle that the USB_EPI[N]_RXCSR_H.DMAREQEN bit is cleared. In DMArequest mode 0, the DMAis programmed to load one packet at a time. Processor intervention is required for each packet. DMAmode 1 can be used with bulk endpoints to transmit multiple packets without processor intervention. |
| 11 (R/W)           | DMAREQMODE | 0 DMARequest Mode 0                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 11 (R/W)           | DMAREQMODE | 1 DMARequest Mode 1                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 10 (R/W)           | DATGLEN    | Data Toggle Write Enable. The USB_EPI[N]_RXCSR_H.DATGLEN bit enables (in host mode) the USB con- troller to write the current state of the endpoint USB_EPI[N]_RXCSR_H.DATGL bit. This bit is automatically cleared once the new value is written.                                                                                                                                                                                                                          |
| 10 (R/W)           | DATGLEN    | 0 Disable Write to DATGL                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 10 (R/W)           | DATGLEN    | 1 Enable Write to DATGL                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 9 (R/W)            | DATGL      | Data Toggle. The USB_EPI[N]_RXCSR_H.DATGL bit indicates (in host mode) the current state of the endpoint data toggle. If D10 is high, this bit may be written with the re- quired setting of the data toggle. If D10 is low, any value written to this bit is ignored. This bit is only used in host mode.                                                                                                                                                                  |
| 9 (R/W)            | DATGL      | 0 DATA0 is Set                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 9 (R/W)            | DATGL      | 1 DATA1 is Set                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 8 (R/W)            | INCOMPRX   | Incomplete Rx. The USB_EPI[N]_RXCSR_H.INCOMPRX bit indicates (in host mode for high- bandwidth isochronous or interrupt transfers) when the received packet is incomplete because parts of the packet were not received. This bit is cleared when USB_EPI[N]_RXCSR_H.RXPKTRDY is cleared. For all other modes, this bit is zero.                                                                                                                                            |
| 8 (R/W)            | INCOMPRX   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 8 (R/W)            | INCOMPRX   | 1 Incomplete Rx                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 27-33: USB\_EPI[N]\_RXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | CLRDATATGL | Clear Endpoint Data Toggle. The USB_EPI[N]_RXCSR_H.CLRDATATGL bit is set (in host mode) by the pro- cessor to reset the endpoint data toggle to 0.                                                                                                                                                                                                                                                                              |
| 6 (R/W0C)          | RXSTALL    | Rx STALL. The USB_EPI[N]_RXCSR_H.RXSTALL bit indicates (in host mode) when a STALL handshake is received. The processor core should clear this bit. 0 No Status                                                                                                                                                                                                                                                                 |
| 5 (R/W)            | REQPKT     | Request Packet. The USB_EPI[N]_RXCSR_H.REQPKT bit directs (in host mode) the USB con- troller to request an IN transaction. This bit is cleared when USB_EPI[N]_RXCSR_H.RXPKTRDY is set. 0 No Request                                                                                                                                                                                                                           |
| 4 (R/W)            | FLUSHFIFO  | 1 Send IN Tokens to Device Flush Endpoint FIFO. The USB_EPI[N]_RXCSR_H.FLUSHFIFO bit directs (in host mode) the USB controller to flush data from the endpoint FIFO and clear the USB_EPI[N]_RXCSR_H.RXPKTRDY bit. The USB_EPI[N]_RXCSR_H.FLUSHFIFO bit should only be set if the USB_EPI[N]_RXCSR_H.RXPKTRDY bit is set. Note that setting this bit at other times may cause data corruption. 0 No Flush 1 Flush Endpoint FIFO |

Table 27-33: USB\_EPI[N]\_RXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W0C)          | NAKTODERR  | NAK Timeout Data Error. The USB_EPI[N]_RXCSR_H.NAKTODERR bit indicates (in host mode for iso- chronous transfers) a NAK timeout data error when the USB_EPI[N]_RXCSR_H.RXPKTRDY bit is set and the data packet has a CRC or bit-stuff error. This bit is cleared when the USB_EPI[N]_RXCSR_H.RXPKTRDY bit is cleared. The USB_EPI[N]_RXCSR_H.NAKTODERR bit indicates (in host mod for bulk transfers) when a receive endpoint is halted following the receipt of NAK responses greater than the limit set in the USB_EP[n]_RXINTERVAL register. The processor should clear this bit to allow the endpoint to continue. If double packet buffering is enabled, the USB_EPI[N]_RXCSR_H.REQPKT bit should also be set in the same cycle as this bit is cleared. |
| 2 (R/W0C)          | RXTOERR    | Rx Timeout Error. The USB_EPI[N]_RXCSR_H.RXTOERR bit indicates (in host mode) when three attempts have been made to receive a packet and no data packet has been received. The USB controller generates an interrupt for this condition. The processor should clear this bit. Note that USB_EPI[N]_RXCSR_H.RXTOERR is valid only when the end- point is operating in bulk or interrupt mode.                                                                                                                                                                                                                                                                                                                                                                 |
| 1 (R/NW)           | FIFOFULL   | 1 Rx Timeout Error FIFO Full. The USB_EPI[N]_RXCSR_H.FIFOFULL bit indicates (in host mode) when no more packets can be loaded into the receive FIFO. 0 No Status 1 FIFO Full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W0C)          | RXPKTRDY   | Rx Packet Ready. The USB_EPI[N]_RXCSR_H.RXPKTRDY is set (in host mode) when a data pack- et is received. An interrupt is generated (if enabled) when this bit is set. The processor core should clear this bit when the packet is read from the FIFO. 0 No Rx Packet                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

## EPn Receive Configuration and Status (Peripheral) Register

The USB\_EPI[N]\_RXCSR\_P register provides (in peripheral mode) control and status bits for transfers through the currently selected receive endpoint.

Figure 27-58: USB\_EPI[N]\_RXCSR\_P Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000057_047aef20a26013aa7edbd4ee03dd7f328378022cf36f286bc78f7201ad8f00ef.png)

Table 27-34: USB\_EPI[N]\_RXCSR\_P Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15                 | AUTOCLR    | Auto Clear Enable. The USB_EPI[N]_RXCSR_P.AUTOCLR bit directs (in peripheral mode) the USB controller to automatically clear the USB_EPI[N]_RXCSR_P.RXPKTRDY bit when a packet of size USB_EP[n]_RXMAXP bytes has been unloaded from the re- ceive FIFO. When packets of less than the maximum packet size are unloaded, the | Auto Clear Enable. The USB_EPI[N]_RXCSR_P.AUTOCLR bit directs (in peripheral mode) the USB controller to automatically clear the USB_EPI[N]_RXCSR_P.RXPKTRDY bit when a packet of size USB_EP[n]_RXMAXP bytes has been unloaded from the re- ceive FIFO. When packets of less than the maximum packet size are unloaded, the |
| 15                 | AUTOCLR    | 0                                                                                                                                                                                                                                                                                                                            | Disable Auto Clear                                                                                                                                                                                                                                                                                                           |
| 14 (R/W)           | ISO        | Isochronous Transfers. The USB_EPI[N]_RXCSR_P.ISO bit selects (in peripheral mode) between iso- chronous transfers and bulk/interrupt transfers.                                                                                                                                                                             | Isochronous Transfers. The USB_EPI[N]_RXCSR_P.ISO bit selects (in peripheral mode) between iso- chronous transfers and bulk/interrupt transfers.                                                                                                                                                                             |
| 14 (R/W)           | ISO        | 0                                                                                                                                                                                                                                                                                                                            | This bit should be cleared for bulk or interrupt trans- fers.                                                                                                                                                                                                                                                                |
| 14 (R/W)           | ISO        | 1                                                                                                                                                                                                                                                                                                                            | This bit should be set for isochronous transfers.                                                                                                                                                                                                                                                                            |
| 13 (R/W)           | DMAREQEN   | DMARequest Enable Rx EP. The USB_EPI[N]_RXCSR_P.DMAREQEN bit enables (in peripheral mode)DMA requests for this receive endpoint.                                                                                                                                                                                             | DMARequest Enable Rx EP. The USB_EPI[N]_RXCSR_P.DMAREQEN bit enables (in peripheral mode)DMA requests for this receive endpoint.                                                                                                                                                                                             |
| 13 (R/W)           | DMAREQEN   | 0                                                                                                                                                                                                                                                                                                                            | Disable DMARequest                                                                                                                                                                                                                                                                                                           |
| 13 (R/W)           | DMAREQEN   | 1                                                                                                                                                                                                                                                                                                                            | Enable DMARequest                                                                                                                                                                                                                                                                                                            |

Table 27-34: USB\_EPI[N]\_RXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | DNYETPERR  | Disable NYET Handshake. The USB_EPI[N]_RXCSR_P.DNYETPERR bit disables (in peripheral mode for high speed bulk/interrupt transactions) NYET handshakes. When this bit is set, all successful receive packets are ACK'd, including the point at which the FIFO becomes full. The USB_EPI[N]_RXCSR_P.DNYETPERR bit must be set for all interrupt endpoints in high speed mode.                                                                                                     |
|                    |            | 0 Enable NYET Handshake                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|                    |            | 1 Disable NYET Handshake                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 11 (R/W)           | DMAREQMODE | DMAMode Select. The USB_EPI[N]_RXCSR_P.DMAREQMODE bit selects (in peripheral mode) be- tween DMArequest mode 1 or 0. This bit must not be cleared the cycle before or the same cycle that the USB_EPI[N]_RXCSR_P.DMAREQEN bit is cleared. InDMA request mode 0, the DMAis programmed to load one packet at a time. Processor in- tervention is required for each packet. DMAmode 1 can be used with bulk endpoints to transmit multiple packets without processor intervention. |
|                    |            | 0 DMARequest Mode 0                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 1 DMARequest Mode 1                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 8 (R/W)            | INCOMPRX   | Incomplete Rx. The USB_EPI[N]_RXCSR_P.INCOMPRX bit indicates (in peripheral mode for high-bandwidth isochronous or interrupt transfers) when the received packet is incom- plete because parts of the packet were not received. This bit is cleared when USB_EPI[N]_RXCSR_P.RXPKTRDY is cleared. For all other modes, this bit is zero.                                                                                                                                         |
|                    |            | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |            | 1 Incomplete Rx                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 7 (R/W)            | CLRDATATGL | Clear Endpoint Data Toggle. The USB_EPI[N]_RXCSR_P.CLRDATATGL bit is set (in peripheral mode) by the processor to reset the endpoint data toggle to 0.                                                                                                                                                                                                                                                                                                                          |
|                    |            | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |            | 1 Reset EP Data Toggle to 0                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 6 (R/W0C)          | SENTSTALL  | Sent STALL. The USB_EPI[N]_RXCSR_P.SENTSTALL bit indicates (in peripheral mode) when a STALL handshake is transmitted. The processor should clear this bit.                                                                                                                                                                                                                                                                                                                     |
|                    |            | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |            | 1 STALL Handshake Transmitted                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 27-34: USB\_EPI[N]\_RXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | SENDSTALL  | Send STALL. The USB_EPI[N]_RXCSR_P.SENDSTALL bit is set (in peripheral mode) by the processor to send a STALL handshake. The processor clears this bit to terminate the stall condition. This bit has no effect for isochronous transfers.                                                                                                                                             |
| 5 (R/W)            | SENDSTALL  | 0 No Action                                                                                                                                                                                                                                                                                                                                                                            |
| 5 (R/W)            | SENDSTALL  | 1 Request STALL Handshake                                                                                                                                                                                                                                                                                                                                                              |
| 4 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EPI[N]_RXCSR_P.FLUSHFIFO bit directs (in peripheral mode) the USB controller to flush data from the endpoint FIFO and clear the USB_EPI[N]_RXCSR_P.RXPKTRDY bit. The USB_EPI[N]_RXCSR_P.FLUSHFIFO bit should only be set if the USB_EPI[N]_RXCSR_P.RXPKTRDY bit is set. Note that setting this bit at other times may cause data corruption.              |
| 4 (R/W)            | FLUSHFIFO  | 0 No Flush                                                                                                                                                                                                                                                                                                                                                                             |
| 4 (R/W)            | FLUSHFIFO  | 1 Flush Endpoint                                                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/NW)           | DATAERR    | FIFO Data Error. The USB_EPI[N]_RXCSR_P.DATAERR bit indicates (in peripheral mode for iso- chronous transfers) when the USB_EPI[N]_RXCSR_P.RXPKTRDY bit is set and the data packet has a CRC or bit-stuff error. This bit is cleared when USB_EPI[N]_RXCSR_P.RXPKTRDY is cleared. The USB_EPI[N]_RXCSR_P.DATAERR bit is always zero for bulk endpoints in pe- ripheral mode. No Status |
| 3 (R/NW)           | DATAERR    | 0                                                                                                                                                                                                                                                                                                                                                                                      |
| 3 (R/NW)           | DATAERR    | 1 Data Error                                                                                                                                                                                                                                                                                                                                                                           |
| 2 (R/W0C)          | ORUNERR    | OUT Run Error. The USB_EPI[N]_RXCSR_P.ORUNERR bit indicates (in peripheral mode for iso- chronous transfers) when an OUT packet cannot be loaded into the receive FIFO. The processor should clear this bit. The USB_EPI[N]_RXCSR_P.ORUNERR bit always returns zero in bulk mode.                                                                                                      |
| 2 (R/W0C)          | ORUNERR    | 0 No Status                                                                                                                                                                                                                                                                                                                                                                            |
| 2 (R/W0C)          | ORUNERR    | 1 OUT Run Error                                                                                                                                                                                                                                                                                                                                                                        |
| 1 (R/NW)           | FIFOFULL   | FIFO Full. The USB_EPI[N]_RXCSR_P.FIFOFULL bit indicates (in peripheral mode) when no more packets can be loaded into the receive FIFO.                                                                                                                                                                                                                                                |
| 1 (R/NW)           | FIFOFULL   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                            |
| 1 (R/NW)           | FIFOFULL   | 1 FIFO Full                                                                                                                                                                                                                                                                                                                                                                            |

Table 27-34: USB\_EPI[N]\_RXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W0C)          | RXPKTRDY   | Rx Packet Ready. The USB_EPI[N]_RXCSR_P.RXPKTRDY is set (in peripheral mode) when a data packet is received. An interrupt is generated (if enabled) when this bit is set. The pro- cessor core should clear this bit when the packet is read from the FIFO. |
| 0 (R/W0C)          | RXPKTRDY   | 0 No Rx Packet                                                                                                                                                                                                                                              |
| 0 (R/W0C)          | RXPKTRDY   | 1 Rx Packet in Endpoint FIFO                                                                                                                                                                                                                                |

## EPn Receive Polling Interval Register

The USB\_EPI[N]\_RXINTERVAL register defines the polling interval for the currently-selected receive endpoint for interrupt, isochronous, and bulk transfers. There is a USB\_EPI[N]\_RXINTERVAL register for each configured receive endpoint, except endpoint 0. The transfer types related to speed, interval value, and interval operation are as follows:

- Interrupt: Speed = low-speed or full -speed, USB\_EPI[N]\_RXINTERVAL = 1-255, and Operation = polling interval is m frames.
- Interrupt: Speed = high-speed, USB\_EPI[N]\_RXINTERVAL = 1-16, and Operation = polling interval is 2 (m-1)  micro-frames.
- Isochronous: Speed = full-speed or high-speed, USB\_EPI[N]\_RXINTERVAL = 1-16, and Operation = polling interval is 2 (m-1)  frames or micro-frames.
- Bulk: Speed = full-speed or high-speed, USB\_EPI[N]\_RXINTERVAL = 2-16, and Operation = NAK limit is 2 (m-1)  frames or micro-frames.

Note that a USB\_EPI[N]\_RXINTERVAL value of 0 or 1 disables the NAK timeout function.

Not all products support high-speed operation or micro-frames. These features do not apply for products that only support low/full-speed operation.

Figure 27-59: USB\_EPI[N]\_RXINTERVAL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000058_b270a75ac7f92afe6cfd5feeb634a6ef649d952aeca27a69240bb4f9f84886da.png)

Table 27-35: USB\_EPI[N]\_RXINTERVAL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Rx Polling Interval. The USB_EPI[N]_RXINTERVAL.VALUE bits define the polling interval value for interrupt and isochronous transfers and select the number of frames (or microframes, if the processor supports high-speed operation) after which the endpoint should timeout on receiving a stream of NAK responses for bulk and control endpoints. Note that the USB controller halts transfers to control endpoints if the host receives NAK responses for more frames than the limit set by this register. |

## EPn Receive Maximum Packet Length Register

The USB\_EPI[N]\_RXMAXP register defines the maximum amount of data that can be transferred through the selected receive endpoint in a single frame.

Note that a value greater than the maximum allowed of 1023 for full-speed USB operation produces unpredictable results. Also, note that the total amount of data represented by the value written to this register must not exceed the receive FIFO size, and should not exceed half the FIFO size if double-buffering is required.

Figure 27-60: USB\_EPI[N]\_RXMAXP Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000059_22496781726d46e045121ec0628a9f75b47057ee8c67f8012266962c98897188.png)

Table 27-36: USB\_EPI[N]\_RXMAXP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:11 (R/W)        | MULTM1     | Multi-Packets per Micro-frame. The USB_EPI[N]_RXMAXP.MULTM1 bits select the number of high-speed, high- bandwidth isochronous or interrupt packets that may be transferred in a micro-frame. The valid number of packets per micro-frame is 1-3 which corresponds to settings 0-2. If this field is not zero, the USB controller combines multiple packets received within a micro-frame into a single packet in the FIFO.                                                                                                                                                                                       |
| 10:0 (R/W)         | MAXPAY     | Maximum Payload. The USB_EPI[N]_RXMAXP.MAXPAY bits select the maximum number of bytes that may be transferred per transaction. This field can be up to 1024, but is subject to constraints by the USB specification based on endpoint mode and speed. This field should not exceed the FIFO size for the endpoint, or half the FIFO size if double buf- fering is used. This value should match the wMaxPacketSize field of the standard end- point descriptor (USB 2.0 spec, section 9). The USB_EPI[N]_RXMAXP.MAXPAY bits must be set to an even number of bytes for proper interrupt generation inDMA mode 1. |

## EPn Receive Type Register

The USB\_EPI[N]\_RXTYPE register selects the endpoint number and transaction protocol to use for the currently selected receive endpoint. There is a USB\_EPI[N]\_RXTYPE register for each receive endpoint. Note that this register is only used in host mode.

Figure 27-61: USB\_EPI[N]\_RXTYPE Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000060_267695302ce1fad6268dfd4a7270e14db169d362c175ae68d003d4b0c151f3d7.png)

Table 27-37: USB\_EPI[N]\_RXTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | SPEED      | Speed of Operation Value. The USB_EPI[N]_RXTYPE.SPEED bits select the USB controller operating speed for the endpoint when acting as a host connected to devices through a hub. In these instances, the USB controller must issue split transactions under certain conditions. If a device is directly connected (not through a hub), all endpoints use the same speed as which the controller is connected. When it is not connected to devices through a hub, program this field with 00. |
| 5:4 (R/W)          | PROTOCOL   | Protocol for Transfer. The USB_EPI[N]_RXTYPE.PROTOCOL bits select the transfer protocol for the endpoint. 0 Control                                                                                                                                                                                                                                                                                                                                                                         |
| 3:0 (R/W)          | TGTEP      | Target Endpoint Number. The USB_EPI[N]_RXTYPE.TGTEP bits select (for endpoints 1-11) the target endpoint. This value should be set to the endpoint number contained in the receive endpoint descriptor returned during device enumeration. Endpoint 0 always uses tar- get endpoint number 0. (Enumeration values not shown are reserved.)                                                                                                                                                  |

Table 27-37: USB\_EPI[N]\_RXTYPE Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration   |
|-----------|------------|---------------------------|
| (Access)  |            | 0 Endpoint 0              |
|           |            | 1 Endpoint 1              |
|           |            | 2 Endpoint 2              |
|           |            | 3 Endpoint 3              |
|           |            | 4 Endpoint 4              |
|           |            | 5 Endpoint 5              |
|           |            | 6 Endpoint 6              |
|           |            | 7 Endpoint 7              |
|           |            | 8 Endpoint 8              |
|           |            | 9 Endpoint 9              |
|           |            | 10 Endpoint 10            |
|           |            | 11 Endpoint 11            |
|           |            | 12 Endpoint 12            |
|           |            | 13 Endpoint 13            |
|           |            | 14 Endpoint 14            |
|           |            | 15 Endpoint 15            |

## EPn Transmit Configuration and Status (Host) Register

The USB\_EPI[N]\_TXCSR\_H register provides (in host mode) control and status bits for transfers through the currently-selected transmit endpoint.

Figure 27-62: USB\_EPI[N]\_TXCSR\_H Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000061_73eb24a4642ebe5376e7611956bc1125be4bb3c5d90993b8941bad6516def12d.png)

Table 27-38: USB\_EPI[N]\_TXCSR\_H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | AUTOSET    | TxPkRdy Autoset Enable. The USB_EPI[N]_TXCSR_H.AUTOSET bit enables (in host mode) the automatic setting of the USB_EPI[N]_TXCSR_H.TXPKTRDY bit when the maximum data packet size ( USB_EP[n]_TXMAXP ) is loaded into the transmit FIFO. The USB_EP[n]_TXMAXP value must be a word (4-byte) multiple. If a packet less than the maximum packet size is loaded, the USB_EPI[N]_TXCSR_H.TXPKTRDY bit needs to be set manually. For products supporting high-speed operation, this USB_EPI[N]_TXCSR_H.AUTOSET bit should not be set for high-bandwidth end- points (endpoints with USB_EP[n]_TXMAXP value greater than 1). 0 Disable Autoset |
| 12 (R/W)           | DMAREQEN   | DMARequest Enable Tx EP. The USB_EPI[N]_TXCSR_H.DMAREQEN bit enables (in host mode) DMAre- quests for this transmit endpoint.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 12 (R/W)           | DMAREQEN   | 0 Disable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 12 (R/W)           | DMAREQEN   | 1 Enable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

Table 27-38: USB\_EPI[N]\_TXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | FRCDATATGL | Force Data Toggle. The USB_EPI[N]_TXCSR_H.FRCDATATGL bit forces (in host mode) the end- point data toggle to switch and clears the data packet from the FIFO, regardless of whether an ACK was received. This feature can be used by interrupt transmit end- points to communicate rate feedback for isochronous endpoints.                                                                                                                                                                        |
| 10 (R/W)           | DMAREQMODE | DMAMode Select. The USB_EPI[N]_TXCSR_H.DMAREQMODE bit selects (in host mode) between DMArequest mode 1 or 0. This bit must not be cleared during the cycle before or the same cycle that the USB_EPI[N]_TXCSR_H.DMAREQEN bit is cleared. InDMA request mode 0, the DMAis programmed to load one packet at a time. Processor in- tervention is required for each packet. DMAmode 1 can be used with bulk endpoints to transmit multiple packets without processor intervention. 0 DMARequest Mode 0 |
| 9 (R/W)            | DATGLEN    | 1 DMARequest Mode 1 Data Toggle Write Enable. The USB_EPI[N]_TXCSR_H.DATGLEN bit enables (in host mode) the USB con- troller to write the current state of the endpoint USB_EPI[N]_TXCSR_H.DATGL bit. This bit is automatically cleared once the new value is written.                                                                                                                                                                                                                             |
|                    |            | 0 Disable Write to DATGL                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 8 (R/W)            | DATGL      | Data Toggle. The USB_EPI[N]_TXCSR_H.DATGL bit indicates (in host mode) the current state of the endpoint data toggle. If D10 is high, this bit may be written with the re- quired setting of the data toggle. If D10 is low, any value written to this bit is ignored. This bit is only used in host mode. 0 DATA0 is set                                                                                                                                                                          |
| 8 (R/W)            |            | 1 DATA1 is set                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 8 (R/W)            |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

Table 27-38: USB\_EPI[N]\_TXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W0C)          | NAKTOINCMP | NAK Timeout Incomplete. The USB_EPI[N]_TXCSR_H.NAKTOINCMP bit indicates (for bulk endpoints in host mode) when the transmit endpoint is halted following the receipt of NAK re- sponses for longer than the time set in the USB_EP[n]_TXINTERVAL register. The processor should clear this bit, allowing the endpoint to continue. For products sup- porting high-speed operation, for high-bandwidth isochronous endpoints in host mode, this bit indicates when no response is received from the device to which the packet is being sent. |
| 7 (R/W0C)          | NAKTOINCMP | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 6 (R/W)            | CLRDATATGL | Clear Endpoint Data Toggle. The USB_EPI[N]_TXCSR_H.CLRDATATGL bit is set (in host mode) by the pro- cessor to reset the endpoint data toggle to 0.                                                                                                                                                                                                                                                                                                                                                                                           |
| 6 (R/W)            | CLRDATATGL | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 6 (R/W)            | CLRDATATGL | 1 Reset EP Data Toggle to 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 5 (R/W0C)          | RXSTALL    | Rx STALL. The USB_EPI[N]_TXCSR_H.RXSTALL bit indicates (in host mode) when a STALL handshake is received. The processor core should clear this bit.                                                                                                                                                                                                                                                                                                                                                                                          |
| 5 (R/W0C)          | RXSTALL    | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 5 (R/W0C)          | RXSTALL    | 1 Stall Received from Device                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | SETUPPKT   | Setup Packet. The USB_EPI[N]_TXCSR_H.SETUPPKT bit directs (in host mode) the USB controller to send a SETUP token instead of an OUT token for the transaction. This bit is set at the same time as the USB_EPI[N]_TXCSR_H.TXPKTRDY bit is set.                                                                                                                                                                                                                                                                                               |
| 4 (R/W)            | SETUPPKT   | 0 No Request                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | SETUPPKT   | 1 Send SETUP Token                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 3 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EPI[N]_TXCSR_H.FLUSHFIFO bit directs (in host mode) the USB controller to flush data from the endpoint FIFO and clear the USB_EPI[N]_TXCSR_H.TXPKTRDY bit. The USB_EPI[N]_TXCSR_H.FLUSHFIFO bit should only be set if the USB_EPI[N]_TXCSR_H.TXPKTRDY bit is set. Note that setting this bit at other times may cause data corruption.                                                                                                                                                                          |
| 3 (R/W)            | FLUSHFIFO  | 1 Flush Endpoint                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 3 (R/W)            | FLUSHFIFO  | FIFO                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 3 (R/W)            | FLUSHFIFO  |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 27-38: USB\_EPI[N]\_TXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W0C)          | TXTOERR    | Tx Timeout Error. The USB_EPI[N]_TXCSR_H.TXTOERR bit indicates (in host mode) when three attempts have been made to send a packet and no handshake packet has been received. The USB controller generates an interrupt for this condition, clears the USB_EPI[N]_TXCSR_H.TXPKTRDY bit, and flushes the FIFO. The processor should clear this bit. Note that USB_EPI[N]_TXCSR_H.TXTOERR is valid only when the endpoint is operating in bulk or interrupt mode. 0 No Status                                                                             |
| 1 (R/NW)           | NEFIFO     | 1 Tx Timeout Error Not Empty FIFO. The USB_EPI[N]_TXCSR_H.NEFIFO bit indicates (in host mode) when there is at least one packet in the transmit FIFO. This bit is cleared automatically when a data packet has been transmitted. If the endpoints transmit interrupt is enabled (in the USB_INTRTXE register), the USB controller generates an interrupt for this condi- tion. Note that the USB_EPI[N]_TXCSR_H.TXPKTRDY bit is also automatically cleared prior to loading a second packet into a double-buffered FIFO. 0 FIFO Empty 1 FIFO Not Empty |
| 0 (R/W1S)          | TXPKTRDY   | Tx Packet Ready. The USB_EPI[N]_TXCSR_H.TXPKTRDY bit should be set (in host mode) by the processor core after loading a data packet into the FIFO. This bit is cleared automati- cally when the data packet is transmitted. An interrupt is generated (if enabled) when the bit is cleared.                                                                                                                                                                                                                                                            |
| 0 (R/W1S)          | TXPKTRDY   | 0 No Tx Packet                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 0 (R/W1S)          | TXPKTRDY   | 1 Tx Packet in Endpoint FIFO                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

## EPn Transmit Configuration and Status (Peripheral) Register

The USB\_EPI[N]\_TXCSR\_P register provides (in peripheral mode) control and status bits for transfers through the currently selected transmit endpoint.

Figure 27-63: USB\_EPI[N]\_TXCSR\_P Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000062_b3c7616dc563955d7942ed74c7bef466740ed26eed2e5ae418095497512edbab.png)

Table 27-39: USB\_EPI[N]\_TXCSR\_P Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | AUTOSET    | TxPkRdy Autoset Enable. The USB_EPI[N]_TXCSR_P.AUTOSET bit enables (in peripheral mode) auto- matic setting of the USB_EPI[N]_TXCSR_P.TXPKTRDY bit when the maximum data packet size ( USB_EP[n]_TXMAXP ) is loaded into the transmit FIFO. The USB_EP[n]_TXMAXP value must be a word (4-byte) multiple. If a packet less than the maximum packet size is loaded, the USB_EPI[N]_TXCSR_P.TXPKTRDY bit needs to be set manually. For products supporting high-speed operation, this USB_EPI[N]_TXCSR_P.AUTOSET bit should not be set for high-bandwidth end- points (endpoints with USB_EP[n]_TXMAXP value greater than 1). Autoset |
| 15 (R/W)           | AUTOSET    | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 15 (R/W)           | AUTOSET    | 1 Enable Autoset                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 27-39: USB\_EPI[N]\_TXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | ISO        | Isochronous Transfers Enable. The USB_EPI[N]_TXCSR_P.ISO bit enables (in peripheral mode) the transmit endpoint for isochronous transfers. This bit should be disabled for bulk or interrupt endpoints.                                                                                                                                                                                                            |
| 14 (R/W)           | ISO        | 0 Disable Tx EP Isochronous Transfers                                                                                                                                                                                                                                                                                                                                                                              |
| 14 (R/W)           | ISO        | 1 Enable Tx EP Isochronous Transfers                                                                                                                                                                                                                                                                                                                                                                               |
| 12 (R/W)           | DMAREQEN   | DMARequest Enable Tx EP. The USB_EPI[N]_TXCSR_P.DMAREQEN bit enables (in peripheral mode)DMA requests for this transmit endpoint.                                                                                                                                                                                                                                                                                  |
| 12 (R/W)           | DMAREQEN   | 0 Disable DMARequest                                                                                                                                                                                                                                                                                                                                                                                               |
| 12 (R/W)           | DMAREQEN   | 1 Enable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                |
| 11 (R/W)           | FRCDATATGL | Force Data Toggle. The USB_EPI[N]_TXCSR_P.FRCDATATGL bit forces (in peripheral mode) the endpoint data toggle to switch and clears the data packet from the FIFO, regardless of whether an ACK was received. This feature can be used by interrupt transmit end- points that are used to communicate rate feedback for isochronous endpoints.                                                                      |
| 11 (R/W)           | FRCDATATGL | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                        |
| 11 (R/W)           | FRCDATATGL | 1 Toggle Endpoint Data                                                                                                                                                                                                                                                                                                                                                                                             |
| 10 (R/W)           | DMAREQMODE | DMAMode Select. The USB_EPI[N]_TXCSR_P.DMAREQMODE bit selects (in peripheral mode) be- tween DMArequest mode 1 or 0. This bit must not be cleared during the cycle before or in the same cycle that the USB_EPI[N]_TXCSR_P.DMAREQEN bit is cleared. In DMArequest mode 0, the DMAis programmed to load one packet at a time. Pro- cessor intervention is required for each packet. DMAmode 1 can be used with bulk |
| 10 (R/W)           | DMAREQMODE | 0 DMARequest Mode 0                                                                                                                                                                                                                                                                                                                                                                                                |
| 10 (R/W)           | DMAREQMODE | 1 DMARequest                                                                                                                                                                                                                                                                                                                                                                                                       |
| 7 (R/W0C)          | INCOMPTX   | Mode 1 Incomplete Tx. The USB_EPI[N]_TXCSR_P.INCOMPTX bit indicates (for high-bandwidth iso- chronous endpoints in peripheral mode) when a large packet has been split into two or three packets for transmission, but insufficient IN tokens have been received to send all parts.                                                                                                                                |
| 7 (R/W0C)          | INCOMPTX   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                        |
| 7 (R/W0C)          | INCOMPTX   | 1 Incomplete Tx (Insufficient IN Tokens)                                                                                                                                                                                                                                                                                                                                                                           |

Table 27-39: USB\_EPI[N]\_TXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | CLRDATATGL | Clear Endpoint Data Toggle. The USB_EPI[N]_TXCSR_P.CLRDATATGL bit is set (in peripheral mode) by the processor to reset the endpoint data toggle to 0.                                                                                                                                                                                                                    |
| 6 (R/W)            | CLRDATATGL | 0 No Action                                                                                                                                                                                                                                                                                                                                                               |
| 6 (R/W)            | CLRDATATGL | 1 Reset EP Data Toggle to 0                                                                                                                                                                                                                                                                                                                                               |
| 5 (R/W0C)          | SENTSTALL  | Sent STALL. The USB_EPI[N]_TXCSR_P.SENTSTALL bit indicates (in peripheral mode) when the USB controller transmits a STALL handshake. When this condition occurs, the USB controller flushes the FIFO and clears the USB_EPI[N]_TXCSR_P.TXPKTRDY bit. The processor should clear this bit.                                                                                 |
| 5 (R/W0C)          | SENTSTALL  | 0 No Status                                                                                                                                                                                                                                                                                                                                                               |
| 5 (R/W0C)          | SENTSTALL  | 1 STALL Handshake Transmitted                                                                                                                                                                                                                                                                                                                                             |
| 4 (R/W)            | SENDSTALL  | Send STALL.                                                                                                                                                                                                                                                                                                                                                               |
| 4 (R/W)            | SENDSTALL  | 0 No Request                                                                                                                                                                                                                                                                                                                                                              |
| 4 (R/W)            | SENDSTALL  | 1 Request STALL Handshake Transmission                                                                                                                                                                                                                                                                                                                                    |
| 3 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EPI[N]_TXCSR_P.FLUSHFIFO bit directs (in peripheral mode) the USB controller to flush data from the endpoint FIFO and clear the USB_EPI[N]_TXCSR_P.TXPKTRDY bit. The USB_EPI[N]_TXCSR_P.FLUSHFIFO bit should only be set if the USB_EPI[N]_TXCSR_P.TXPKTRDY bit is set. Note that setting this bit at other times may cause data corruption. |
| 3 (R/W)            | FLUSHFIFO  | 0 No Flush                                                                                                                                                                                                                                                                                                                                                                |
| 3 (R/W)            | FLUSHFIFO  | 1 Flush endpoint FIFO                                                                                                                                                                                                                                                                                                                                                     |
| 2 (R/W0C)          | URUNERR    | Underrun Error. The USB_EPI[N]_TXCSR_P.URUNERR bit indicates (in peripheral mode) when an IN token is received while the USB_EPI[N]_TXCSR_P.TXPKTRDY bit is not set. The processor should clear this bit.                                                                                                                                                                 |
| 2 (R/W0C)          | URUNERR    | 0 No Status                                                                                                                                                                                                                                                                                                                                                               |

Table 27-39: USB\_EPI[N]\_TXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | NEFIFO     | Not Empty FIFO. The USB_EPI[N]_TXCSR_P.NEFIFO bit indicates (in peripheral mode) when there is at least one packet in the transmit FIFO. This bit is cleared automatically when a data packet has been transmitted. If the endpoints transmit interrupt is enabled (in the USB_INTRTXE register), the USB controller generates an interrupt for this condition. Note that the USB_EPI[N]_TXCSR_P.TXPKTRDY bit is also automatically cleared prior to loading a second packet into a double-buffered FIFO. 0 FIFO Empty |
| 0 (R/W1S)          | TXPKTRDY   | Tx Packet Ready. The USB_EPI[N]_TXCSR_P.TXPKTRDY bit should be set (in peripheral mode) by the processor core after loading a data packet into the FIFO. This bit is cleared au- tomatically when the data packet is transmitted. An interrupt is generated (if enabled) when the bit is cleared. 0 No Tx Packet                                                                                                                                                                                                       |
| 0 (R/W1S)          | TXPKTRDY   | 1 Tx Packet in Endpoint FIFO                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 0 (R/W1S)          | TXPKTRDY   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

## EPn Transmit Polling Interval Register

The USB\_EPI[N]\_TXINTERVAL register defines the polling interval for the currently-selected transmit endpoint for interrupt, isochronous, and bulk transfers. There is a USB\_EPI[N]\_TXINTERVAL register for each configured transmit endpoint, except endpoint 0. The transfer types related to the speed, interval value, and interval operation are as follows:

- Interrupt: Speed = low-speed or full-speed, USB\_EPI[N]\_TXINTERVAL = 1-255, and Operation = polling interval is m frames.
- Interrupt: Speed = high-speed, USB\_EPI[N]\_TXINTERVAL = 1-16, and Operation = polling interval is 2 (m-1)  micro-frames.
- Isochronous: Speed = full-speed or high-speed, USB\_EPI[N]\_TXINTERVAL = 1-16, and Operation = polling interval is 2 (m-1)  frames or micro-frames.
- Bulk: Speed = full-speed or high-speed, USB\_EPI[N]\_TXINTERVAL = 2-16, and Operation = NAK limit is 2 (m-1)  frames or micro-frames.

Note that a USB\_EPI[N]\_TXINTERVAL value of 0 or 1 disables the NAK timeout function.

Not all products support high-speed operation or micro-frames. These features do not apply for products that only support low/full-speed operation.

Figure 27-64: USB\_EPI[N]\_TXINTERVAL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000063_398008e6bdfd0dd06deae0a15bec393afbd0abf712db44767088748a4ef205f1.png)

Table 27-40: USB\_EPI[N]\_TXINTERVAL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Tx Polling Interval. The USB_EPI[N]_TXINTERVAL.VALUE bits define the polling interval value for interrupt and isochronous transfers. The USB_EPI[N]_TXINTERVAL.VALUE bits select the number of frames (or micro-frames, if the processor supports high-speed operation) after which the endpoint should timeout on receiving a stream of NAK re- sponses for bulk and control endpoints. Note that the USB controller halts transfers to control endpoints if the host receives NAK responses for more frames than the limit set by this register. |

## EPn Transmit Maximum Packet Length Register

The USB\_EPI[N]\_TXMAXP register defines the maximum amount of data that can be transferred through the selected transmit endpoint in a single frame. When setting this value, consider the constraints placed by the USB specification on packet sizes for bulk, interrupt and isochronous transactions in full-speed operations. The USB\_EPI[N]\_TXMAXP register provides indexed access to the maximum packet length register for each Tx endpoint, except endpoint 0.

Figure 27-65: USB\_EPI[N]\_TXMAXP Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000064_b5557a03b8cae217981592ab824be023a79974aad148a1267defc75e326a8bc5.png)

Table 27-41: USB\_EPI[N]\_TXMAXP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:11 (R/W)        | MULTM1     | Multi-Packets per Micro-Frame. The USB_EPI[N]_TXMAXP.MULTM1 bits select the number of high-speed, high- bandwidth isochronous or interrupt packets that may be transferred in a micro-frame. The valid number of packets per micro-frame is 1-3 which corresponds to settings 0-2. If this field is not zero, the USB controller splits the FIFO data into multiple packets less than or equal to the maximum payload size.                                                                                                                                                                                     |
| 10:0 (R/W)         | MAXPAY     | Maximum Payload. The USB_EPI[N]_TXMAXP.MAXPAY bits select the maximum number of bytes that may be transferred per transaction. This field can be up to 1024 but is subject to constraints by the USB specification based on endpoint mode and speed. This field should not exceed the FIFO size for the endpoint, or half the FIFO size if double buf- fering is used. This value should match the wMaxPacketSize field of the standard end- point descriptor (USB 2.0 spec, section 9). The USB_EPI[N]_TXMAXP.MAXPAY bits must be set to an even number of bytes for proper interrupt generation inDMA mode 1. |

## EPn Transmit Type Register

The USB\_EPI[N]\_TXTYPE register selects the endpoint number and transaction protocol to use for the currently selected transmit endpoint. There is a USB\_EPI[N]\_TXTYPE register for each transmit endpoint. Note that this register is only used in host mode.

Figure 27-66: USB\_EPI[N]\_TXTYPE Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000065_2e9a88168f5fa24e5a1ee8b93a5164b70482f32b786a8fa6bf8c81fd8c8eead0.png)

Table 27-42: USB\_EPI[N]\_TXTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | SPEED      | Speed of Operation Value. The USB_EPI[N]_TXTYPE.SPEED bits select the USB controller operating speed for the endpoint when acting as a host connected to devices through a hub. In these instances, the USB controller must issue split transactions under certain conditions. If a device is directly connected (not through a hub), all endpoints use the same speed as which the controller is connected. When not connected to devices through a hub, pro- gram this field with 00. |
| 5:4 (R/W)          | PROTOCOL   | 3 Low-Speed Protocol for Transfer. The USB_EPI[N]_TXTYPE.PROTOCOL bits select the transfer protocol for the endpoint. 0 Control                                                                                                                                                                                                                                                                                                                                                         |
| 3:0 (R/W)          | TGTEP      | Target Endpoint Number. The USB_EPI[N]_TXTYPE.TGTEP bits select (for endpoints 1-11) the target endpoint. This value should be set to the endpoint number contained in the transmit endpoint descriptor returned during device enumeration. Endpoint 0 always uses tar- get endpoint number 0. (Enumeration values not shown are reserved.)                                                                                                                                             |

Table 27-42: USB\_EPI[N]\_TXTYPE Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration   |
|-----------|------------|---------------------------|
| (Access)  |            | 0 Endpoint 0              |
|           |            | 1 Endpoint 1              |
|           |            | 2 Endpoint 2              |
|           |            | 3 Endpoint 3              |
|           |            | 4 Endpoint 4              |
|           |            | 5 Endpoint 5              |
|           |            | 6 Endpoint 6              |
|           |            | 7 Endpoint 7              |
|           |            | 8 Endpoint 8              |
|           |            | 9 Endpoint 9              |
|           |            | 10 Endpoint 10            |
|           |            | 11 Endpoint 11            |
|           |            | 12 Endpoint 12            |
|           |            | 13 Endpoint 13            |
|           |            | 14 Endpoint 14            |
|           |            | 15 Endpoint 15            |

## EPn Number of Bytes Received Register

The USB\_EP[n]\_RXCNT register indicates the number of received data bytes in the endpoint receive FIFO. The value returned changes as the contents of the FIFO change and is only valid while the USB\_EP[n]\_RXCSR\_H.RXPKTRDY bit or USB\_EP[n]\_RXCSR\_P.RXPKTRDY bit is set.

Figure 27-67: USB\_EP[n]\_RXCNT Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000066_fef4b3121966c67f3686c71608045dca02599315f1d4636c752cb0870d3cbd59.png)

Table 27-43: USB\_EP[n]\_RXCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------|
| 13:0 (R/NW)        | EPRXCNT    | EP Rx Count. The USB_EP[n]_RXCNT.EPRXCNT bits hold the number of data bytes ready to be read from the receive FIFO. |

## EPn Receive Configuration and Status (Host) Register

The USB\_EP[n]\_RXCSR\_H register provides (in host mode) control and status bits for transfers through the currently selected receive endpoint.

Figure 27-68: USB\_EP[n]\_RXCSR\_H Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000067_5e3248268f8bf83e261fc952743f3eda6a726030f617fd48b7b0be622a019793.png)

Table 27-44: USB\_EP[n]\_RXCSR\_H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | AUTOCLR    | Auto Clear Enable. The USB_EP[n]_RXCSR_H.AUTOCLR bit directs (in host mode) the USB con- troller to automatically clear the USB_EP[n]_RXCSR_H.RXPKTRDY bit when a packet of size USB_EP[n]_RXMAXP bytes has been unloaded from the receive FIFO. When packets of less than the maximum packet size are unloaded, the processor must clear USB_EP[n]_RXCSR_H.RXPKTRDY manually. When using theDMA to unload the receive FIFO, data is read from the receive FIFO in four byte chunks, regardless of the USB_EP[n]_RXMAXP value. The USB controller auto clears the USB_EP[n]_RXCSR_H.RXPKTRDY bit as follows. (In the following: Remain- der=(RxMaxP/4), and PktSz-Clearing-RxPktRdy=Actual-Bytes-Read-Packet-Sizes- That-Clear-RxPktRdy.) • Remainder=0, Bytes-Read=RxMaxP , PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1, RxMaxP-2, RxMaxP-3 • Remainder=3, Bytes Read=RxMaxP+1, PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1, RxMaxP-2 • Remainder=2, Bytes Read=RxMaxP+2, PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1 • Remainder=1, Bytes Read=RxMaxP+3, PktSz-Clearing-RxPktRdy=RxMaxP For products supporting high-speed operation, the USB_EP[n]_RXCSR_H.AUTOCLR bit should not be set for high-bandwidth iso- | Auto Clear Enable. The USB_EP[n]_RXCSR_H.AUTOCLR bit directs (in host mode) the USB con- troller to automatically clear the USB_EP[n]_RXCSR_H.RXPKTRDY bit when a packet of size USB_EP[n]_RXMAXP bytes has been unloaded from the receive FIFO. When packets of less than the maximum packet size are unloaded, the processor must clear USB_EP[n]_RXCSR_H.RXPKTRDY manually. When using theDMA to unload the receive FIFO, data is read from the receive FIFO in four byte chunks, regardless of the USB_EP[n]_RXMAXP value. The USB controller auto clears the USB_EP[n]_RXCSR_H.RXPKTRDY bit as follows. (In the following: Remain- der=(RxMaxP/4), and PktSz-Clearing-RxPktRdy=Actual-Bytes-Read-Packet-Sizes- That-Clear-RxPktRdy.) • Remainder=0, Bytes-Read=RxMaxP , PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1, RxMaxP-2, RxMaxP-3 • Remainder=3, Bytes Read=RxMaxP+1, PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1, RxMaxP-2 • Remainder=2, Bytes Read=RxMaxP+2, PktSz-Clearing-RxPktRdy=RxMaxP , RxMaxP-1 • Remainder=1, Bytes Read=RxMaxP+3, PktSz-Clearing-RxPktRdy=RxMaxP For products supporting high-speed operation, the USB_EP[n]_RXCSR_H.AUTOCLR bit should not be set for high-bandwidth iso- |
| 15 (R/W)           | AUTOCLR    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Disable Auto Clear                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14 (R/W)           | AUTOREQ    | Auto Request Clear Enable. The USB_EP[n]_RXCSR_H.AUTOREQ bit directs (in host mode) the USB con- troller to automatically clear the USB_EP[n]_RXCSR_H.REQPKT bit when USB_EP[n]_RXCSR_H.RXPKTRDY bit is cleared. This bit is automatically cleared when a short packet is received.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Auto Request Clear Enable. The USB_EP[n]_RXCSR_H.AUTOREQ bit directs (in host mode) the USB con- troller to automatically clear the USB_EP[n]_RXCSR_H.REQPKT bit when USB_EP[n]_RXCSR_H.RXPKTRDY bit is cleared. This bit is automatically cleared when a short packet is received.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 14 (R/W)           | AUTOREQ    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Disable Auto Request Clear                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 14 (R/W)           | AUTOREQ    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Enable Auto Request Clear                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 13 (R/W)           | DMAREQEN   | DMARequest Enable Rx EP. The USB_EP[n]_RXCSR_H.DMAREQEN bit enables (in host mode) DMAre- quests for this receive endpoint.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | DMARequest Enable Rx EP. The USB_EP[n]_RXCSR_H.DMAREQEN bit enables (in host mode) DMAre- quests for this receive endpoint.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 13 (R/W)           | DMAREQEN   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Disable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 13 (R/W)           | DMAREQEN   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Enable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 27-44: USB\_EP[n]\_RXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W0C)         | PIDERR     | Packet ID Error. The USB_EP[n]_RXCSR_H.PIDERR bit indicates (in host mode) when a PID er- ror occurs for isochronous transactions. This bit is ignored in host mode for bulk or interrupt transactions.                                                                                                                                                                                                                                                             |
| 12 (R/W0C)         | PIDERR     | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 12 (R/W0C)         | PIDERR     | 1 PID Error                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 11 (R/W)           | DMAREQMODE | DMAMode Select. The USB_EP[n]_RXCSR_H.DMAREQMODE bit selects (in host mode) between DMArequest mode 1 or 0. This bit must not be cleared the cycle before or the same cycle that the USB_EP[n]_RXCSR_H.DMAREQEN bit is cleared. In DMArequest mode 0, the DMAis programmed to load one packet at a time. Processor intervention is required for each packet. DMAmode 1 can be used with bulk endpoints to transmit multiple packets without processor intervention. |
| 11 (R/W)           | DMAREQMODE | 0 DMARequest Mode 0                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 11 (R/W)           | DMAREQMODE | 1 DMARequest Mode 1                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 10 (R/W)           | DATGLEN    | Data Toggle Write Enable. The USB_EP[n]_RXCSR_H.DATGLEN bit enables (in host mode) the USB con- troller to write the current state of the endpoint USB_EP[n]_RXCSR_H.DATGL bit. This bit is automatically cleared once the new value is written.                                                                                                                                                                                                                    |
| 10 (R/W)           | DATGLEN    | 0 Disable Write to DATGL                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 10 (R/W)           | DATGLEN    | 1 Enable Write to DATGL                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 9 (R/W)            | DATGL      | Data Toggle. The USB_EP[n]_RXCSR_H.DATGL bit indicates (in host mode) the current state of the endpoint data toggle. If D10 is high, this bit may be written with the required setting of the data toggle. If D10 is low, any value written to this bit is ignored. This bit is only used in host mode.                                                                                                                                                             |
| 9 (R/W)            | DATGL      | 0 DATA0 is Set                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 9 (R/W)            | DATGL      | 1 DATA1 is Set                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 8 (R/W)            | INCOMPRX   | Incomplete Rx. The USB_EP[n]_RXCSR_H.INCOMPRX bit indicates (in host mode for high- bandwidth isochronous or interrupt transfers) when the received packet is incomplete because parts of the packet were not received. This bit is cleared when USB_EP[n]_RXCSR_H.RXPKTRDY is cleared. For all other modes, this bit is zero.                                                                                                                                      |
| 8 (R/W)            | INCOMPRX   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 8 (R/W)            | INCOMPRX   | 1 Incomplete Rx                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 27-44: USB\_EP[n]\_RXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | CLRDATATGL | Clear Endpoint Data Toggle. The USB_EP[n]_RXCSR_H.CLRDATATGL bit is set (in host mode) by the pro- cessor to reset the endpoint data toggle to 0.                                                                                                                                                                                                                                                                           |
| 6 (R/W0C)          | RXSTALL    | Rx STALL. The USB_EP[n]_RXCSR_H.RXSTALL bit indicates (in host mode) when a STALL handshake is received. The processor core should clear this bit. 0 No Status                                                                                                                                                                                                                                                              |
| 5 (R/W)            | REQPKT     | Request Packet. The USB_EP[n]_RXCSR_H.REQPKT bit directs (in host mode) the USB control- ler to request an IN transaction. This bit is cleared when USB_EP[n]_RXCSR_H.RXPKTRDY is set. 0 No Request                                                                                                                                                                                                                         |
| 4 (R/W)            | FLUSHFIFO  | 1 Send IN Tokens to Device Flush Endpoint FIFO. The USB_EP[n]_RXCSR_H.FLUSHFIFO bit directs (in host mode) the USB controller to flush data from the endpoint FIFO and clear the USB_EP[n]_RXCSR_H.RXPKTRDY bit. The USB_EP[n]_RXCSR_H.FLUSHFIFO bit should only be set if the USB_EP[n]_RXCSR_H.RXPKTRDY bit is set. Note that setting this bit at other times may cause data corruption. 0 No Flush 1 Flush Endpoint FIFO |

Table 27-44: USB\_EP[n]\_RXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W0C)          | NAKTODERR  | NAK Timeout Data Error. The USB_EP[n]_RXCSR_H.NAKTODERR bit indicates (in host mode for isochro- nous transfers) a NAK timeout data error when the USB_EP[n]_RXCSR_H.RXPKTRDY bit is set and the data packet has a CRC or bit-stuff error. This bit is cleared when the USB_EP[n]_RXCSR_H.RXPKTRDY bit is cleared. The USB_EP[n]_RXCSR_H.NAKTODERR bit indicates (in host mod for bulk transfers) when a receive endpoint is halted following the receipt of NAK responses greater than the limit set in the USB_EP[n]_RXINTERVAL register. The processor should clear this bit to allow the endpoint to continue. If double packet buffering is enabled, the USB_EP[n]_RXCSR_H.REQPKT bit should also be set in the same cycle as this bit is cleared. |
| 2 (R/W0C)          | RXTOERR    | Rx Timeout Error. The USB_EP[n]_RXCSR_H.RXTOERR bit indicates (in host mode) when three at- tempts have been made to receive a packet and no data packet has been received. The USB controller generates an interrupt for this condition. The processor should clear this bit. Note that USB_EP[n]_RXCSR_H.RXTOERR is valid only when the end- point is operating in bulk or interrupt mode.                                                                                                                                                                                                                                                                                                                                                            |
| 1 (R/NW)           | FIFOFULL   | 1 Rx Timeout Error FIFO Full. The USB_EP[n]_RXCSR_H.FIFOFULL bit indicates (in host mode) when no more packets can be loaded into the receive FIFO. 0 No Status 1 FIFO Full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 0 (R/W0C)          | RXPKTRDY   | Rx Packet Ready. The USB_EP[n]_RXCSR_H.RXPKTRDY is set (in host mode) when a data packet is received. An interrupt is generated (if enabled) when this bit is set. The processor core should clear this bit when the packet is read from the FIFO. 0 No Rx Packet                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

## EPn Receive Configuration and Status (Peripheral) Register

The USB\_EP[n]\_RXCSR\_P register provides (in peripheral mode) control and status bits for transfers through the currently selected receive endpoint.

Figure 27-69: USB\_EP[n]\_RXCSR\_P Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000068_047aef20a26013aa7edbd4ee03dd7f328378022cf36f286bc78f7201ad8f00ef.png)

Table 27-45: USB\_EP[n]\_RXCSR\_P Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15                 | AUTOCLR    | Auto Clear Enable. The USB_EP[n]_RXCSR_P.AUTOCLR bit directs (in peripheral mode) the USB controller to automatically clear the USB_EP[n]_RXCSR_P.RXPKTRDY bit when a packet of size USB_EP[n]_RXMAXP bytes has been unloaded from the receive FIFO. When packets of less than the maximum packet size are unloaded, the processor must clear USB_EP[n]_RXCSR_P.RXPKTRDY manually. When using theDMA to unload the receive FIFO, data is read from the receive FIFO in four byte chunks, regardless of the USB_EP[n]_RXMAXP value. The USB controller auto clears the USB_EP[n]_RXCSR_P.RXPKTRDY bit as follows. (In the following: Remain- | Auto Clear Enable. The USB_EP[n]_RXCSR_P.AUTOCLR bit directs (in peripheral mode) the USB controller to automatically clear the USB_EP[n]_RXCSR_P.RXPKTRDY bit when a packet of size USB_EP[n]_RXMAXP bytes has been unloaded from the receive FIFO. When packets of less than the maximum packet size are unloaded, the processor must clear USB_EP[n]_RXCSR_P.RXPKTRDY manually. When using theDMA to unload the receive FIFO, data is read from the receive FIFO in four byte chunks, regardless of the USB_EP[n]_RXMAXP value. The USB controller auto clears the USB_EP[n]_RXCSR_P.RXPKTRDY bit as follows. (In the following: Remain- |
| 15                 | AUTOCLR    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Disable Auto Clear                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 14 (R/W)           | ISO        | Isochronous Transfers. The USB_EP[n]_RXCSR_P.ISO bit selects (in peripheral mode) between isochro- nous transfers and bulk/interrupt transfers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Isochronous Transfers. The USB_EP[n]_RXCSR_P.ISO bit selects (in peripheral mode) between isochro- nous transfers and bulk/interrupt transfers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 14 (R/W)           | ISO        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | This bit should be cleared for bulk or interrupt trans- fers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14 (R/W)           | ISO        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | This bit should be set for isochronous transfers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 13 (R/W)           | DMAREQEN   | DMARequest Enable Rx EP. The USB_EP[n]_RXCSR_P.DMAREQEN bit enables (in peripheral mode)DMA requests for this receive endpoint.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | DMARequest Enable Rx EP. The USB_EP[n]_RXCSR_P.DMAREQEN bit enables (in peripheral mode)DMA requests for this receive endpoint.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 13 (R/W)           | DMAREQEN   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Disable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 13 (R/W)           | DMAREQEN   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Enable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 27-45: USB\_EP[n]\_RXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | DNYETPERR  | Disable NYET Handshake. The USB_EP[n]_RXCSR_P.DNYETPERR bit disables (in peripheral mode for high speed bulk/interrupt transactions) NYET handshakes. When this bit is set, all successful receive packets are ACK'd, including the point at which the FIFO becomes full. The USB_EP[n]_RXCSR_P.DNYETPERR bit must be set for all interrupt endpoints in high speed mode.                                                                                                       |
| 12 (R/W)           | DNYETPERR  | 0 Enable NYET Handshake                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 11 (R/W)           | DMAREQMODE | DMAMode Select. The USB_EP[n]_RXCSR_P.DMAREQMODE bit selects (in peripheral mode) be- tween DMArequest mode 1 or 0. This bit must not be cleared the cycle before or the same cycle that the USB_EP[n]_RXCSR_P.DMAREQEN bit is cleared. In DMAre- quest mode 0, the DMAis programmed to load one packet at a time. Processor inter- vention is required for each packet. DMAmode 1 can be used with bulk endpoints to transmit multiple packets without processor intervention. |
| 11 (R/W)           | DMAREQMODE | 0 DMARequest Mode 0                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 8 (R/W)            | INCOMPRX   | 1 DMARequest Mode 1 Incomplete Rx. The USB_EP[n]_RXCSR_P.INCOMPRX bit indicates (in peripheral mode for high-bandwidth isochronous or interrupt transfers) when the received packet is incom- plete because parts of the packet were not received. This bit is cleared when USB_EP[n]_RXCSR_P.RXPKTRDY is cleared. For all other modes, this bit is zero.                                                                                                                       |
| 8 (R/W)            | INCOMPRX   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 8 (R/W)            | INCOMPRX   | 1 Incomplete Rx                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 7 (R/W)            | CLRDATATGL | Clear Endpoint Data Toggle. The USB_EP[n]_RXCSR_P.CLRDATATGL bit is set (in peripheral mode) by the processor to reset the endpoint data toggle to 0. Action                                                                                                                                                                                                                                                                                                                    |
| 7 (R/W)            | CLRDATATGL | 0 No                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 7 (R/W)            | CLRDATATGL | 1 Reset EP Data Toggle to 0                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 6 (R/W0C)          | SENTSTALL  | Sent STALL. The USB_EP[n]_RXCSR_P.SENTSTALL bit indicates (in peripheral mode) when a STALL handshake is transmitted. The processor should clear this bit.                                                                                                                                                                                                                                                                                                                      |
| 6 (R/W0C)          | SENTSTALL  | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 6 (R/W0C)          | SENTSTALL  | 1 STALL Handshake Transmitted                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 27-45: USB\_EP[n]\_RXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | SENDSTALL  | Send STALL. The USB_EP[n]_RXCSR_P.SENDSTALL bit is set (in peripheral mode) by the processor to send a STALL handshake. The processor clears this bit to terminate the stall condition. This bit has no effect for isochronous transfers.                                                                                                                                     |
| 5 (R/W)            | SENDSTALL  | 0 No Action                                                                                                                                                                                                                                                                                                                                                                   |
| 5 (R/W)            | SENDSTALL  | 1 Request STALL Handshake                                                                                                                                                                                                                                                                                                                                                     |
| 4 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EP[n]_RXCSR_P.FLUSHFIFO bit directs (in peripheral mode) the USB controller to flush data from the endpoint FIFO and clear the USB_EP[n]_RXCSR_P.RXPKTRDY bit. The USB_EP[n]_RXCSR_P.FLUSHFIFO bit should only be set if the USB_EP[n]_RXCSR_P.RXPKTRDY bit is set. Note that setting this bit at other times may cause data corruption.         |
| 4 (R/W)            | FLUSHFIFO  | 0 No Flush                                                                                                                                                                                                                                                                                                                                                                    |
| 4 (R/W)            | FLUSHFIFO  | 1 Flush Endpoint FIFO                                                                                                                                                                                                                                                                                                                                                         |
| 3 (R/NW)           | DATAERR    | Data Error. The USB_EP[n]_RXCSR_P.DATAERR bit indicates (in peripheral mode for iso- chronous transfers) when the USB_EP[n]_RXCSR_P.RXPKTRDY bit is set and the data packet has a CRC or bit-stuff error. This bit is cleared when USB_EP[n]_RXCSR_P.RXPKTRDY is cleared. The USB_EP[n]_RXCSR_P.DATAERR bit is always zero for bulk endpoints in periph- eral mode. No Status |
| 3 (R/NW)           | DATAERR    | 0                                                                                                                                                                                                                                                                                                                                                                             |
| 3 (R/NW)           | DATAERR    | 1 Data Error                                                                                                                                                                                                                                                                                                                                                                  |
| 2 (R/W0C)          | ORUNERR    | OUT Run Error. The USB_EP[n]_RXCSR_P.ORUNERR bit indicates (in peripheral mode for iso- chronous transfers) when an OUT packet cannot be loaded into the receive FIFO. The processor should clear this bit. The USB_EP[n]_RXCSR_P.ORUNERR bit al- ways returns zero in bulk mode.                                                                                             |
| 2 (R/W0C)          | ORUNERR    | 0 No Status                                                                                                                                                                                                                                                                                                                                                                   |
| 2 (R/W0C)          | ORUNERR    | 1 OUT Run Error                                                                                                                                                                                                                                                                                                                                                               |
| 1 (R/NW)           | FIFOFULL   | FIFO Full. The USB_EP[n]_RXCSR_P.FIFOFULL bit indicates (in peripheral mode) when no more packets can be loaded into the receive FIFO.                                                                                                                                                                                                                                        |
| 1 (R/NW)           | FIFOFULL   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                   |
| 1 (R/NW)           | FIFOFULL   | 1 FIFO Full                                                                                                                                                                                                                                                                                                                                                                   |

Table 27-45: USB\_EP[n]\_RXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W0C)          | RXPKTRDY   | Rx Packet Ready. The USB_EP[n]_RXCSR_P.RXPKTRDY is set (in peripheral mode) when a data packet is received. An interrupt is generated (if enabled) when this bit is set. The pro- cessor core should clear this bit when the packet is read from the FIFO. |
| 0 (R/W0C)          | RXPKTRDY   | 0 No Rx Packet                                                                                                                                                                                                                                             |
| 0 (R/W0C)          | RXPKTRDY   | 1 Rx Packet in Endpoint FIFO                                                                                                                                                                                                                               |

## EPn Receive Polling Interval Register

The USB\_EP[n]\_RXINTERVAL register defines the polling interval for the currently-selected receive endpoint for interrupt, isochronous, and bulk transfers. There is a USB\_EP[n]\_RXINTERVAL register for each configured receive endpoint, except endpoint 0. The transfer types related to speed, interval value, and interval operation are as follows:

- Interrupt: Speed = low-speed or full -speed, USB\_EP[n]\_RXINTERVAL = 1-255, and Operation = polling interval is m frames.
- Interrupt: Speed = high-speed, USB\_EP[n]\_RXINTERVAL = 1-16, and Operation = polling interval is 2 (m-1)  micro-frames.
- Isochronous: Speed = full-speed or high-speed, USB\_EP[n]\_RXINTERVAL = 1-16, and Operation = polling interval is 2 (m-1)  frames or micro-frames.
- Bulk: Speed = full-speed or high-speed, USB\_EP[n]\_RXINTERVAL = 2-16, and Operation = NAK limit is 2 (m-1)  frames or micro-frames.

Note that a USB\_EP[n]\_RXINTERVAL value of 0 or 1 disables the NAK timeout function.

Not all products support high-speed operation or micro-frames. These features do not apply for products that only support low/full-speed operation.

Figure 27-70: USB\_EP[n]\_RXINTERVAL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000069_b270a75ac7f92afe6cfd5feeb634a6ef649d952aeca27a69240bb4f9f84886da.png)

Table 27-46: USB\_EP[n]\_RXINTERVAL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Rx Polling Interval. The USB_EP[n]_RXINTERVAL.VALUE bits define the polling interval value for interrupt and isochronous transfers and select the number of frames (or microframes, if the processor supports high-speed operation) after which the endpoint should timeout on receiving a stream of NAK responses for bulk and control endpoints. Note that the USB controller halts transfers to control endpoints if the host receives NAK responses for more frames than the limit set by this register. |

## EPn Receive Maximum Packet Length Register

The USB\_EP[n]\_RXMAXP register defines the maximum amount of data that can be transferred through the selected receive endpoint in a single frame.

Note that a value greater than the maximum allowed of 1023 for full-speed USB operation produces unpredictable results. Also, note that the total amount of data represented by the value written to this register must not exceed the receive FIFO size, and should not exceed half the FIFO size if double-buffering is required.

Figure 27-71: USB\_EP[n]\_RXMAXP Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000070_22496781726d46e045121ec0628a9f75b47057ee8c67f8012266962c98897188.png)

Table 27-47: USB\_EP[n]\_RXMAXP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:11 (R/W)        | MULTM1     | Multi-Packets per Micro-frame. The USB_EP[n]_RXMAXP.MULTM1 bits select the number of high-speed, high- bandwidth isochronous or interrupt packets that may be transferred in a micro-frame. The valid number of packets per micro-frame is 1-3 which corresponds to settings 0-2. If this field is not zero, the USB controller combines multiple packets received within a micro-frame into a single packet in the FIFO.                                                                                                                                                                                      |
| 10:0 (R/W)         | MAXPAY     | Maximum Payload. The USB_EP[n]_RXMAXP.MAXPAY bits select the maximum number of bytes that may be transferred per transaction. This field can be up to 1024, but is subject to con- straints by the USB specification based on endpoint mode and speed. This field should not exceed the FIFO size for the endpoint, or half the FIFO size if double buffering is used. This value should match the wMaxPacketSize field of the standard endpoint de- scriptor (USB 2.0 spec, section 9). The USB_EP[n]_RXMAXP.MAXPAY bits must be set to an even number of bytes for proper interrupt generation in DMAmode 1. |

## EPn Receive Type Register

The USB\_EP[n]\_RXTYPE register selects the endpoint number and transaction protocol to use for the currently selected receive endpoint. There is a USB\_EP[n]\_RXTYPE register for each receive endpoint. Note that this register is only used in host mode.

Figure 27-72: USB\_EP[n]\_RXTYPE Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000071_267695302ce1fad6268dfd4a7270e14db169d362c175ae68d003d4b0c151f3d7.png)

Table 27-48: USB\_EP[n]\_RXTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | SPEED      | Speed of Operation Value. The USB_EP[n]_RXTYPE.SPEED bits select the USB controller operating speed for the endpoint when acting as a host connected to devices through a hub. In these instances, the USB controller must issue split transactions under certain conditions. If a device is directly connected (not through a hub), all endpoints use the same speed as which the controller is connected. When it is not connected to devices through a hub, program this field with 00. |
| 5:4 (R/W)          | PROTOCOL   | Protocol for Transfer. The USB_EP[n]_RXTYPE.PROTOCOL bits select the transfer protocol for the endpoint. 0 Control 1 Isochronous                                                                                                                                                                                                                                                                                                                                                           |
| 3:0 (R/W)          | TGTEP      | Target Endpoint Number. The USB_EP[n]_RXTYPE.TGTEP bits select (for endpoints 1-11) the target end- point. This value should be set to the endpoint number contained in the receive end- point descriptor returned during device enumeration. Endpoint 0 always uses target endpoint number 0. (Enumeration values not shown are reserved.)                                                                                                                                                |

Table 27-48: USB\_EP[n]\_RXTYPE Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration   |
|-----------|------------|---------------------------|
| (Access)  |            | 0 Endpoint 0              |
|           |            | 1 Endpoint 1              |
|           |            | 2 Endpoint 2              |
|           |            | 3 Endpoint 3              |
|           |            | 4 Endpoint 4              |
|           |            | 5 Endpoint 5              |
|           |            | 6 Endpoint 6              |
|           |            | 7 Endpoint 7              |
|           |            | 8 Endpoint 8              |
|           |            | 9 Endpoint 9              |
|           |            | 10 Endpoint 10            |
|           |            | 11 Endpoint 11            |
|           |            | 12 Endpoint 12            |
|           |            | 13 Endpoint 13            |
|           |            | 14 Endpoint 14            |
|           |            | 15 Endpoint 15            |

## EPn Transmit Configuration and Status (Host) Register

The USB\_EP[n]\_TXCSR\_H register provides (in host mode) control and status bits for transfers through the currently-selected transmit endpoint.

Figure 27-73: USB\_EP[n]\_TXCSR\_H Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000072_73eb24a4642ebe5376e7611956bc1125be4bb3c5d90993b8941bad6516def12d.png)

Table 27-49: USB\_EP[n]\_TXCSR\_H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | AUTOSET    | TxPkRdy Autoset Enable. The USB_EP[n]_TXCSR_H.AUTOSET bit enables (in host mode) the automatic setting of the USB_EP[n]_TXCSR_H.TXPKTRDY bit when the maximum data packet size ( USB_EP[n]_TXMAXP ) is loaded into the transmit FIFO. The USB_EP[n]_TXMAXP value must be a word (4-byte) multiple. If a packet less than the maximum packet size is loaded, the USB_EP[n]_TXCSR_H.TXPKTRDY bit needs to be set manually. For products supporting high-speed operation, this USB_EP[n]_TXCSR_H.AUTOSET bit should not be set for high-bandwidth end- points (endpoints with USB_EP[n]_TXMAXP value greater than 1). 0 Disable Autoset |
| 12 (R/W)           | DMAREQEN   | DMARequest Enable Tx EP. The USB_EP[n]_TXCSR_H.DMAREQEN bit enables (in host mode) DMAre- quests for this transmit endpoint.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 12 (R/W)           | DMAREQEN   | 0 Disable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 12 (R/W)           | DMAREQEN   | 1 Enable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 27-49: USB\_EP[n]\_TXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | FRCDATATGL | Force Data Toggle. The USB_EP[n]_TXCSR_H.FRCDATATGL bit forces (in host mode) the end- point data toggle to switch and clears the data packet from the FIFO, regardless of whether an ACK was received. This feature can be used by interrupt transmit end- points to communicate rate feedback for isochronous endpoints.                                                                                                                                                                         |
| 10 (R/W)           | DMAREQMODE | DMAMode Select. The USB_EP[n]_TXCSR_H.DMAREQMODE bit selects (in host mode) between DMArequest mode 1 or 0. This bit must not be cleared during the cycle before or the same cycle that the USB_EP[n]_TXCSR_H.DMAREQEN bit is cleared. In DMAre- quest mode 0, the DMAis programmed to load one packet at a time. Processor inter- vention is required for each packet. DMAmode 1 can be used with bulk endpoints to transmit multiple packets without processor intervention. 0 DMARequest Mode 0 |
| 9 (R/W)            | DATGLEN    | 1 DMARequest Mode 1 Data Toggle Write Enable. The USB_EP[n]_TXCSR_H.DATGLEN bit enables (in host mode) the USB con- troller to write the current state of the endpoint USB_EP[n]_TXCSR_H.DATGL bit. This bit is automatically cleared once the new value is written.                                                                                                                                                                                                                               |
|                    |            | 0 Disable Write to DATGL                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 8 (R/W)            | DATGL      | Data Toggle. The USB_EP[n]_TXCSR_H.DATGL bit indicates (in host mode) the current state of the endpoint data toggle. If D10 is high, this bit may be written with the required setting of the data toggle. If D10 is low, any value written to this bit is ignored. This bit is only used in host mode. 0 DATA0 is set                                                                                                                                                                             |
| 8 (R/W)            |            | 1 DATA1 is set                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 8 (R/W)            |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

Table 27-49: USB\_EP[n]\_TXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W0C)          | NAKTOINCMP | NAK Timeout Incomplete. The USB_EP[n]_TXCSR_H.NAKTOINCMP bit indicates (for bulk endpoints in host mode) when the transmit endpoint is halted following the receipt of NAK re- sponses for longer than the time set in the USB_EP[n]_TXINTERVAL register. The processor should clear this bit, allowing the endpoint to continue. For products sup- porting high-speed operation, for high-bandwidth isochronous endpoints in host mode, this bit indicates when no response is received from the device to which the packet is being sent. |
| 7 (R/W0C)          | NAKTOINCMP | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 6 (R/W)            | CLRDATATGL | Clear Endpoint Data Toggle. The USB_EP[n]_TXCSR_H.CLRDATATGL bit is set (in host mode) by the pro- cessor to reset the endpoint data toggle to 0.                                                                                                                                                                                                                                                                                                                                                                                           |
| 6 (R/W)            | CLRDATATGL | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 6 (R/W)            | CLRDATATGL | 1 Reset EP Data Toggle to 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 5 (R/W0C)          | RXSTALL    | Rx STALL. The USB_EP[n]_TXCSR_H.RXSTALL bit indicates (in host mode) when a STALL handshake is received. The processor core should clear this bit.                                                                                                                                                                                                                                                                                                                                                                                          |
| 5 (R/W0C)          | RXSTALL    | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 5 (R/W0C)          | RXSTALL    | 1 Stall Received from Device                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 4 (R/W)            | SETUPPKT   | Setup Packet. The USB_EP[n]_TXCSR_H.SETUPPKT bit directs (in host mode) the USB con- troller to send a SETUP token instead of an OUT token for the transaction. This bit is set at the same time as the USB_EP[n]_TXCSR_H.TXPKTRDY bit is set.                                                                                                                                                                                                                                                                                              |
| 4 (R/W)            | SETUPPKT   | 0 No Request                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 4 (R/W)            | SETUPPKT   | 1 Send SETUP Token                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 3 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EP[n]_TXCSR_H.FLUSHFIFO bit directs (in host mode) the USB controller to flush data from the endpoint FIFO and clear the USB_EP[n]_TXCSR_H.TXPKTRDY bit. The USB_EP[n]_TXCSR_H.FLUSHFIFO bit should only be set if the USB_EP[n]_TXCSR_H.TXPKTRDY bit is set. Note that setting this bit at other times may cause data corruption.                                                                                                                                                                             |
| 3 (R/W)            | FLUSHFIFO  | 0 No Flush                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 3 (R/W)            | FLUSHFIFO  | 1 Flush Endpoint FIFO                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 27-49: USB\_EP[n]\_TXCSR\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W0C)          | TXTOERR    | Tx Timeout Error. The USB_EP[n]_TXCSR_H.TXTOERR bit indicates (in host mode) when three at- tempts have been made to send a packet and no handshake packet has been received. The USB controller generates an interrupt for this condition, clears the USB_EP[n]_TXCSR_H.TXPKTRDY bit, and flushes the FIFO. The processor should clear this bit. Note that USB_EP[n]_TXCSR_H.TXTOERR is valid only when the endpoint is operating in bulk or interrupt mode. 0 No Status                                                         |
| 1 (R/NW)           | NEFIFO     | Not Empty FIFO. The USB_EP[n]_TXCSR_H.NEFIFO bit indicates (in host mode) when there is at least one packet in the transmit FIFO. This bit is cleared automatically when a data packet has been transmitted. If the endpoints transmit interrupt is enabled (in the USB_INTRTXE register), the USB controller generates an interrupt for this condi- tion. Note that the USB_EP[n]_TXCSR_H.TXPKTRDY bit is also automatically cleared prior to loading a second packet into a double-buffered FIFO. 0 FIFO Empty 1 FIFO Not Empty |
| 0 (R/W1S)          | TXPKTRDY   | Tx Packet Ready. The USB_EP[n]_TXCSR_H.TXPKTRDY bit should be set (in host mode) by the processor core after loading a data packet into the FIFO. This bit is cleared automati- cally when the data packet is transmitted. An interrupt is generated (if enabled) when the bit is cleared.                                                                                                                                                                                                                                        |
| 0 (R/W1S)          | TXPKTRDY   | 0 No Tx Packet                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 0 (R/W1S)          | TXPKTRDY   | 1 Tx Packet in Endpoint FIFO                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

## EPn Transmit Configuration and Status (Peripheral) Register

The USB\_EP[n]\_TXCSR\_P register provides (in peripheral mode) control and status bits for transfers through the currently selected transmit endpoint.

Figure 27-74: USB\_EP[n]\_TXCSR\_P Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000073_b3c7616dc563955d7942ed74c7bef466740ed26eed2e5ae418095497512edbab.png)

Table 27-50: USB\_EP[n]\_TXCSR\_P Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | AUTOSET    | TxPkRdy Autoset Enable. The USB_EP[n]_TXCSR_P.AUTOSET bit enables (in peripheral mode) automatic setting of the USB_EP[n]_TXCSR_P.TXPKTRDY bit when the maximum data packet size ( USB_EP[n]_TXMAXP ) is loaded into the transmit FIFO. The USB_EP[n]_TXMAXP value must be a word (4-byte) multiple. If a packet less than the maximum packet size is loaded, the USB_EP[n]_TXCSR_P.TXPKTRDY bit needs to be set manually. For products supporting high-speed operation, this USB_EP[n]_TXCSR_P.AUTOSET bit should not be set for high-bandwidth end- points (endpoints with USB_EP[n]_TXMAXP value greater than 1). Autoset |
| 15 (R/W)           | AUTOSET    | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 15 (R/W)           | AUTOSET    | 1 Enable Autoset                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 27-50: USB\_EP[n]\_TXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | ISO        | Isochronous Transfers Enable. The USB_EP[n]_TXCSR_P.ISO bit enables (in peripheral mode) the transmit endpoint for isochronous transfers. This bit should be disabled for bulk or interrupt endpoints.                                                                                                                                                                                                                |
| 14 (R/W)           | ISO        | 0 Disable Tx EP Isochronous Transfers                                                                                                                                                                                                                                                                                                                                                                                 |
| 14 (R/W)           | ISO        | 1 Enable Tx EP Isochronous Transfers                                                                                                                                                                                                                                                                                                                                                                                  |
| 12 (R/W)           | DMAREQEN   | DMARequest Enable Tx EP. The USB_EP[n]_TXCSR_P.DMAREQEN bit enables (in peripheral mode)DMA requests for this transmit endpoint.                                                                                                                                                                                                                                                                                      |
| 12 (R/W)           | DMAREQEN   | 0 Disable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                  |
| 12 (R/W)           | DMAREQEN   | 1 Enable DMARequest                                                                                                                                                                                                                                                                                                                                                                                                   |
| 11 (R/W)           | FRCDATATGL | Force Data Toggle. The USB_EP[n]_TXCSR_P.FRCDATATGL bit forces (in peripheral mode) the endpoint data toggle to switch and clears the data packet from the FIFO, regardless of whether an ACK was received. This feature can be used by interrupt transmit end- points that are used to communicate rate feedback for isochronous endpoints.                                                                          |
| 11 (R/W)           | FRCDATATGL | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                           |
| 11 (R/W)           | FRCDATATGL | 1 Toggle Endpoint Data                                                                                                                                                                                                                                                                                                                                                                                                |
| 10 (R/W)           | DMAREQMODE | DMAMode Select. The USB_EP[n]_TXCSR_P.DMAREQMODE bit selects (in peripheral mode) be- tween DMArequest mode 1 or 0. This bit must not be cleared during the cycle before or in the same cycle that the USB_EP[n]_TXCSR_P.DMAREQEN bit is cleared. In DMArequest mode 0, the DMAis programmed to load one packet at a time. Process- or intervention is required for each packet. DMAmode 1 can be used with bulk end- |
| 10 (R/W)           | DMAREQMODE | 0 DMARequest Mode 0                                                                                                                                                                                                                                                                                                                                                                                                   |
| 10 (R/W)           | DMAREQMODE | 1 DMARequest                                                                                                                                                                                                                                                                                                                                                                                                          |
| 7 (R/W0C)          | INCOMPTX   | Mode 1 Incomplete Tx. The USB_EP[n]_TXCSR_P.INCOMPTX bit indicates (for high-bandwidth iso- chronous endpoints in peripheral mode) when a large packet has been split into two or three packets for transmission, but insufficient IN tokens have been received to send all parts.                                                                                                                                    |
| 7 (R/W0C)          | INCOMPTX   | 0 No Status                                                                                                                                                                                                                                                                                                                                                                                                           |
| 7 (R/W0C)          | INCOMPTX   | 1 Incomplete Tx (Insufficient IN Tokens)                                                                                                                                                                                                                                                                                                                                                                              |

Table 27-50: USB\_EP[n]\_TXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | CLRDATATGL | Clear Endpoint Data Toggle. The USB_EP[n]_TXCSR_P.CLRDATATGL bit is set (in peripheral mode) by the processor to reset the endpoint data toggle to 0.                                                                                                                                                                                                                 |
| 6 (R/W)            | CLRDATATGL | 0 No Action                                                                                                                                                                                                                                                                                                                                                           |
| 6 (R/W)            | CLRDATATGL | 1 Reset EP Data Toggle to 0                                                                                                                                                                                                                                                                                                                                           |
| 5 (R/W0C)          | SENTSTALL  | Sent STALL. The USB_EP[n]_TXCSR_P.SENTSTALL bit indicates (in peripheral mode) when the USB controller transmits a STALL handshake. When this condition occurs, the USB controller flushes the FIFO and clears the USB_EP[n]_TXCSR_P.TXPKTRDY bit. The processor should clear this bit.                                                                               |
| 5 (R/W0C)          | SENTSTALL  | 0 No Status                                                                                                                                                                                                                                                                                                                                                           |
| 5 (R/W0C)          | SENTSTALL  | 1 STALL Handshake Transmitted                                                                                                                                                                                                                                                                                                                                         |
| 4                  | SENDSTALL  | Send STALL.                                                                                                                                                                                                                                                                                                                                                           |
| 4                  | SENDSTALL  | 0 No Request                                                                                                                                                                                                                                                                                                                                                          |
| 4                  | SENDSTALL  | 1 Request STALL Handshake Transmission                                                                                                                                                                                                                                                                                                                                |
| 3 (R/W)            | FLUSHFIFO  | Flush Endpoint FIFO. The USB_EP[n]_TXCSR_P.FLUSHFIFO bit directs (in peripheral mode) the USB controller to flush data from the endpoint FIFO and clear the USB_EP[n]_TXCSR_P.TXPKTRDY bit. The USB_EP[n]_TXCSR_P.FLUSHFIFO bit should only be set if the USB_EP[n]_TXCSR_P.TXPKTRDY bit is set. Note that setting this bit at other times may cause data corruption. |
| 3 (R/W)            | FLUSHFIFO  | 0 No Flush                                                                                                                                                                                                                                                                                                                                                            |
| 3 (R/W)            | FLUSHFIFO  | 1 Flush endpoint FIFO                                                                                                                                                                                                                                                                                                                                                 |
| 2 (R/W0C)          | URUNERR    | Underrun Error. The USB_EP[n]_TXCSR_P.URUNERR bit indicates (in peripheral mode) when an IN token is received while the USB_EP[n]_TXCSR_P.TXPKTRDY bit is not set. The processor should clear this bit.                                                                                                                                                               |
| 2 (R/W0C)          | URUNERR    | 0 No Status                                                                                                                                                                                                                                                                                                                                                           |
| 2 (R/W0C)          | URUNERR    | 1 Underrun Error                                                                                                                                                                                                                                                                                                                                                      |

Table 27-50: USB\_EP[n]\_TXCSR\_P Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | NEFIFO     | Not Empty FIFO. The USB_EP[n]_TXCSR_P.NEFIFO bit indicates (in peripheral mode) when there is at least one packet in the transmit FIFO. This bit is cleared automatically when a data packet has been transmitted. If the endpoints transmit interrupt is enabled (in the USB_INTRTXE register), the USB controller generates an interrupt for this condition. Note that the USB_EP[n]_TXCSR_P.TXPKTRDY bit is also automatically cleared prior to loading a second packet into a double-buffered FIFO. 0 FIFO Empty |
| 0 (R/W1S)          | TXPKTRDY   | Tx Packet Ready. The USB_EP[n]_TXCSR_P.TXPKTRDY bit should be set (in peripheral mode) by the processor core after loading a data packet into the FIFO. This bit is cleared auto- matically when the data packet is transmitted. An interrupt is generated (if enabled) when the bit is cleared. 0 No Tx Packet                                                                                                                                                                                                      |
| 0 (R/W1S)          | TXPKTRDY   | 1 Tx Packet in Endpoint FIFO                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 0 (R/W1S)          | TXPKTRDY   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

## EPn Transmit Polling Interval Register

The USB\_EP[n]\_TXINTERVAL register defines the polling interval for the currently-selected transmit endpoint for interrupt, isochronous, and bulk transfers. There is a USB\_EP[n]\_TXINTERVAL register for each configured transmit endpoint, except endpoint 0. The transfer types related to the speed, interval value, and interval operation are as follows:

- Interrupt: Speed = low-speed or full-speed, USB\_EP[n]\_TXINTERVAL = 1-255, and Operation = polling interval is m frames.
- Interrupt: Speed = high-speed, USB\_EP[n]\_TXINTERVAL = 1-16, and Operation = polling interval is 2 (m-1)  micro-frames.
- Isochronous: Speed = full-speed or high-speed, USB\_EP[n]\_TXINTERVAL = 1-16, and Operation = polling interval is 2 (m-1)  frames or micro-frames.
- Bulk: Speed = full-speed or high-speed, USB\_EP[n]\_TXINTERVAL = 2-16, and Operation = NAK limit is 2 (m-1)  frames or micro-frames.

Note that a USB\_EP[n]\_TXINTERVAL value of 0 or 1 disables the NAK timeout function.

Not all products support high-speed operation or micro-frames. These features do not apply for products that only support low/full-speed operation.

Figure 27-75: USB\_EP[n]\_TXINTERVAL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000074_398008e6bdfd0dd06deae0a15bec393afbd0abf712db44767088748a4ef205f1.png)

Table 27-51: USB\_EP[n]\_TXINTERVAL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Tx Polling Interval. The USB_EP[n]_TXINTERVAL.VALUE bits define the polling interval value for interrupt and isochronous transfers. The USB_EP[n]_TXINTERVAL.VALUE bits select the number of frames (or micro-frames, if the processor supports high-speed op- eration) after which the endpoint should timeout on receiving a stream of NAK re- sponses for bulk and control endpoints. Note that the USB controller halts transfers to control endpoints if the host receives NAK responses for more frames than the limit set by this register. |

## EPn Transmit Maximum Packet Length Register

The USB\_EP[n]\_TXMAXP register defines the maximum amount of data that can be transferred through the selected transmit endpoint in a single frame. When setting this value, consider the constraints placed by the USB specification on packet sizes for bulk, interrupt and isochronous transactions in full-speed operations. The USB\_EP[n]\_TXMAXP register provides indexed access to the maximum packet length register for each Tx endpoint, except endpoint 0.

Figure 27-76: USB\_EP[n]\_TXMAXP Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000075_abafe9dd86ec2c7674ac5d0028167b2db2e61b1aec3103d9fdfb22012bb95827.png)

Table 27-52: USB\_EP[n]\_TXMAXP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:11 (R/W)        | MULTM1     | Multi-Packets per Micro-Frame. The USB_EP[n]_TXMAXP.MULTM1 bits select the number of high-speed, high- bandwidth isochronous or interrupt packets that may be transferred in a micro-frame. The valid number of packets per micro-frame is 1-3 which corresponds to settings 0-2. If this field is not zero, the USB controller splits the FIFO data into multiple packets less than or equal to the maximum payload size.                                                                                                                                                                                    |
| 10:0 (R/W)         | MAXPAY     | Maximum Payload. The USB_EP[n]_TXMAXP.MAXPAY bits select the maximum number of bytes that may be transferred per transaction. This field can be up to 1024 but is subject to con- straints by the USB specification based on endpoint mode and speed. This field should not exceed the FIFO size for the endpoint, or half the FIFO size if double buffering is used. This value should match the wMaxPacketSize field of the standard endpoint de- scriptor (USB 2.0 spec, section 9). The USB_EP[n]_TXMAXP.MAXPAY bits must be set to an even number of bytes for proper interrupt generation in DMAmode 1. |

## EPn Transmit Type Register

The USB\_EP[n]\_TXTYPE register selects the endpoint number and transaction protocol to use for the currently selected transmit endpoint. There is a USB\_EP[n]\_TXTYPE register for each transmit endpoint. Note that this register is only used in host mode.

Figure 27-77: USB\_EP[n]\_TXTYPE Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000076_267695302ce1fad6268dfd4a7270e14db169d362c175ae68d003d4b0c151f3d7.png)

Table 27-53: USB\_EP[n]\_TXTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | SPEED      | Speed of Operation Value. The USB_EP[n]_TXTYPE.SPEED bits select the USB controller operating speed for the endpoint when acting as a host connected to devices through a hub. In these instances, the USB controller must issue split transactions under certain conditions. If a device is directly connected (not through a hub), all endpoints use the same speed as which the controller is connected. When not connected to devices through a hub, pro- gram this field with 00. |
| 5:4 (R/W)          | PROTOCOL   | Protocol for Transfer. The USB_EP[n]_TXTYPE.PROTOCOL bits select the transfer protocol for the endpoint. 0 Control                                                                                                                                                                                                                                                                                                                                                                     |
| 3:0 (R/W)          | TGTEP      | Target Endpoint Number. The USB_EP[n]_TXTYPE.TGTEP bits select (for endpoints 1-11) the target end- point. This value should be set to the endpoint number contained in the transmit end- point descriptor returned during device enumeration. Endpoint 0 always uses target endpoint number 0. (Enumeration values not shown are reserved.)                                                                                                                                           |

Table 27-53: USB\_EP[n]\_TXTYPE Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration   |
|-----------|------------|---------------------------|
| (Access)  |            | 0 Endpoint 0              |
|           |            | 1 Endpoint 1              |
|           |            | 2 Endpoint 2              |
|           |            | 3 Endpoint 3              |
|           |            | 4 Endpoint 4              |
|           |            | 5 Endpoint 5              |
|           |            | 6 Endpoint 6              |
|           |            | 7 Endpoint 7              |
|           |            | 8 Endpoint 8              |
|           |            | 9 Endpoint 9              |
|           |            | 10 Endpoint 10            |
|           |            | 11 Endpoint 11            |
|           |            | 12 Endpoint 12            |
|           |            | 13 Endpoint 13            |
|           |            | 14 Endpoint 14            |
|           |            | 15 Endpoint 15            |

## Function Address Register

The USB\_FADDR register contains the device address used in peripheral mode. The processor writes this register with the address received through a SET\_ADDRESS command from the host.

Figure 27-78: USB\_FADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000077_6a100ec084122da07ebdd9a568c76d38777a3272093841ef9e10c90593a858e7.png)

Table 27-54: USB\_FADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | VALUE      | Function Address Value. The USB_FADDR.VALUE bits contain the address of the peripheral part of the trans- action. |

## FIFO Byte (8-Bit) Register

Writes to the USB\_FIFOB[n] register go to the endpoint Tx FIFO and reads from the USB\_FIFOB[n] register come from the endpoint Rx FIFO. The USB\_FIFOB[n] , USB\_FIFOH[n] , and USB\_FIFO[n] registers are the same register. These registers exist at the same address. Typically, programs should load and unload the FIFO using word ( USB\_FIFO[n] register) writes and reads, which are more efficient. If the USB packet is a non-word (4-byte) size, the program should use a half-word ( USB\_FIFOH[n] register) or byte ( USB\_FIFOB[n] register) read or write at the end when loading or unloading the FIFO.

Note that (for correct USB controller operation) programs should not mix byte, half-word, or word accesses, except for the last few bytes if the size of the packet is odd (not a multiple of the size they were using).

Figure 27-79: USB\_FIFOB[n] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000078_6d849e5295b6244ab769dab695b49fbf988ad1e0b1f14b1fdb2fb0753182d2ef.png)

Table 27-55: USB\_FIFOB[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | FIFO       | FIFO Byte Data. The USB_FIFOB[n].FIFO bits provide byte access to the USB Tx and Rx end- point FIFOs. |

## FIFO Half-Word (16-Bit) Register

Writes to the USB\_FIFOH[n] register go to the endpoint Tx FIFO and reads from the USB\_FIFOH[n] register come from the endpoint Rx FIFO. The USB\_FIFOB[n] , USB\_FIFOH[n] , and USB\_FIFO[n] registers are the same register. These registers exist at the same address. Typically, programs should load and unload the FIFO using word ( USB\_FIFO[n] register) writes and reads, which are more efficient. If the USB packet is a non-word (4-byte) size, programs should use a half-word ( USB\_FIFOH[n] register) or byte ( USB\_FIFOB[n] register) read or write at the end when loading or unloading the FIFO.

Note that (for correct USB controller operation) programs should not mix byte, half-word, or word accesses, except for the last few bytes if the size of the packet is odd (not a multiple of the size they were using).

Figure 27-80: USB\_FIFOH[n] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000079_2e111dd791323d30bc6a0bbee10154b58733f90d07cd75344e3808742401f5fe.png)

Table 27-56: USB\_FIFOH[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | FIFO       | FIFO Half-Word Data. The USB_FIFOH[n].FIFO bits provide half-word access to the USB Tx and Rx endpoint FIFOs. |

## FIFO Word (32-Bit) Register

Writes to the USB\_FIFO[n] register go to the endpoint Tx FIFO and reads from the USB\_FIFO[n] register come from the endpoint Rx FIFO. The USB\_FIFOB[n] , USB\_FIFOH[n] , and USB\_FIFO[n] registers are the same registers. These registers exist at the same address. Typically, programs should load and unload the FIFO using word ( USB\_FIFO[n] register) writes and reads, which are more efficient. If the USB packet is a non-word (4-byte) size, programs should use a half-word ( USB\_FIFOH[n] register) or byte ( USB\_FIFOB[n] register) read or write at the end when loading or unloading the FIFO.

Note that (for correct USB controller operation) programs should not mix byte, half-word, or word accesses, except for the last few bytes if the size of the packet is odd (not a multiple of the size they were using).

Figure 27-81: USB\_FIFO[n] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000080_a3a53b772c82af00004bf17c7ef5c0f4899641cd4c9230f24fbf619612b2ae62.png)

Table 27-57: USB\_FIFO[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | FIFO Word Data. The USB_FIFO[n].VALUE bits provide word access to the USB Tx and Rx end- point FIFOs. |

## Frame Number Register

The USB\_FRAME register contains the frame number of the last received frame. The data in this register has bit 10 as the MSB and bit 0 as the LSB.

Figure 27-82: USB\_FRAME Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000081_04fcbc2fd4ea50162015edb6a27efa66c79f89125f46f8cf77d7c067f8bfea0c.png)

Table 27-58: USB\_FRAME Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/NW)        | VALUE      | Frame Number Value. The USB_FRAME.VALUE bits contains the frame number of the last received frame. The data in this field has bit 10 as the MSB and bit 0 as the LSB. |

## Full-Speed EOF 1 Register

The USB\_FS\_EOF1 register defines the minimum time gap allowed between the start of the last transaction and the end of frame for full-speed transactions.

Figure 27-83: USB\_FS\_EOF1 Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000082_c3eaf28fd1eab556382bdb50d0c6b582e957f2cdad4cf54a3e1dfe37cdd80102.png)

Table 27-59: USB\_FS\_EOF1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Full-Speed EOF 1 Value. The USB_FS_EOF1.VALUE bits set the time before the end of the frame to stop beginning new transactions (in units of 533.3ns) for full-speed transactions. The de- fault setting corresponds to 63.46us. |

## High-Speed EOF 1 Register

The USB\_HS\_EOF1 register defines the minimum time gap allowed between the start of the last transaction and the end of frame for high-speed transactions.

Figure 27-84: USB\_HS\_EOF1 Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000083_2225275852c5666a52d99de303fb6014f5fd0b645c18058b6c5e226a4846b6ab.png)

Table 27-60: USB\_HS\_EOF1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | High-Speed EOF 1 Value. The USB_HS_EOF1.VALUE sets the time before the end of the frame to stop begin- ning new transactions (in units of 133.3ns) for high-speed transactions. The default setting corresponds to 17.07us. |

## ID Control

The USB\_IDCTL register can be used to override the ID pin and force the controller to act as an A-device or Bdevice.

Figure 27-85: USB\_IDCTL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000084_087ea5fb2a34936dd8125c6f18c15aecaedb29229fefeb7006a9075f3724afb8.png)

Table 27-61: USB\_IDCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | IDVAL      | ID Value. When the USB_IDCTL.IDSEL bit =1, the USB_IDCTL.IDVAL bit sets the value of the ID input to the controller. This bit has no effect if USB_IDCTL.IDSEL =0. 0 A-Device                                                                                                     |
| 0 (R/W)            | IDSEL      | ID Select. The USB_IDCTL.IDSEL bit selects the source of the ID input to the controller. This can be used to bypass the ID input pin and force the controller to act as an A- device or B-device. 0 ID pin selected for controller input 1 IDCTL[1] selected for controller input |

## Common Interrupts Enable Register

The USB\_IEN register enables interrupts for USB controller system sources. Enabling an interrupt in this register directs the USB controller to generate an interrupt if the corresponding interrupt pending bit in the USB\_IRQ register is set.

Figure 27-86: USB\_IEN Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000085_ccf1050b5cffa34e7128c0561ea1f5d00fc44484da8153a9ed8576dafe0d8421.png)

Table 27-62: USB\_IEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | VBUSERR    | VBUS Threshold Indicator Interrupt Enable. The USB_IEN.VBUSERR bit enables the USB_IRQ.VBUSERR interrupt.                            |
| 6 (R/W)            | SESSREQ    | Session Request Indicator Interrupt Enable. The USB_IEN.SESSREQ bit enables the USB_IRQ.SESSREQ interrupt.                           |
| 5 (R/W)            | DISCON     | Disconnect Indicator Interrupt Enable. The USB_IEN.DISCON bit enables the USB_IRQ.DISCON interrupt. 0 Disable Interrupt              |
| 4 (R/W)            | CON        | Connection Indicator Interrupt Enable. The USB_IEN.CON bit enables the USB_IRQ.CON interrupt. 0 Disable Interrupt 1 Enable Interrupt |

Table 27-62: USB\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | SOF        | Start of Frame Indicator Interrupt Enable. The USB_IEN.SOF bit enables the USB_IRQ.SOF interrupt.                                         |
| 2 (R/W)            | RSTBABBLE  | Reset/Babble Indicator Interrupt Enable. The USB_IEN.RSTBABBLE bit enables the USB_IRQ.RSTBABBLE interrupt.                               |
| 1 (R/W)            | RESUME     | Resume Indicator Interrupt Enable. The USB_IEN.RESUME bit enables the USB_IRQ.RESUME interrupt. 0 Disable Interrupt                       |
| 0 (R/W)            | SUSPEND    | Suspend Indicator Interrupt Enable. The USB_IEN.SUSPEND bit enables the USB_IRQ.SUSPEND interrupt. 0 Disable Interrupt 1 Enable Interrupt |

## Index Register

The USB\_INDEX register contains an index value for mirrored addressing of USB controller endpoint control and status registers.

There is one set of registers, but they are mirrored at two address locations if the endpoint is selected by the USB\_INDEX register. An endpoint's register set only appears in the indexed location if the USB\_INDEX register is written with that endpoint number. You can read/write an endpoint's register in either the directly mapped location which is always visible, or in the indexed location which is only visible if the USB\_INDEX register is written with the endpoint number. The USB\_INDEX register and indexed address locations only affect address decoding. For example, loading a 0 into the USB\_INDEX register selects endpoint 0 access.

The USB\_INDEX register can be used for indexed access of the directly mapped control/status registers from USB controller address offset 0x100-0x1FF . For products supporting the dynamic FIFO size feature, the endpoint Tx/Rx size and address registers always use the USB\_INDEX register, there is no direct mapping for these endpoint-specific registers. The multi-point USB\_MP[n]\_TXFUNCADDR , USB\_MP[n]\_TXHUBADDR , USB\_MP[n]\_TXHUBPORT , USB\_MP[n]\_RXFUNCADDR , USB\_MP[n]\_RXHUBADDR , and USB\_MP[n]\_RXHUBPORT registers only have direct mapping, no indexed mapping.

Before accessing an endpoint's control/status registers using the indexed range, write the endpoint number to the USB\_INDEX register to ensure that the correct control/status registers appear in the indexed range of the memory map.

Figure 27-87: USB\_INDEX Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000086_ce6fc81e9c4fb01de42c4707050070aaada60ee96887e4774d5b11f46a9876b8.png)

Table 27-63: USB\_INDEX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------|
| 3:0                | EP         | Endpoint Index. The USB_INDEX.EP bits select mirrored access for an endpoints indexed control |
| (R/W)              |            | and status registers. Valid values for this bit field are 0-11.                               |

## Receive Interrupt Register

The USB\_INTRRX register indicates which interrupts are currently active for the receive (Rx) endpoints. Note that the USB controller automatically clears this register when it is read.

Figure 27-88: USB\_INTRRX Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000087_7d120578dced2fcc1a3652641e6d99d1c695de2d269731ef4cf4761db5c3d384.png)

Table 27-64: USB\_INTRRX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (RC/NW)         | EP11       | Endpoint 11 Rx Interrupt. The USB_INTRRX.EP11 bit indicates whether or not a receive interrupt is pending for this endpoint.                                  |
| 10 (RC/NW)         | EP10       | Endpoint 10 Rx Interrupt. The USB_INTRRX.EP10 bit indicates whether or not a receive interrupt is pending for this endpoint.                                  |
| 9 (RC/NW)          | EP9        | Endpoint 9 Rx Interrupt. The USB_INTRRX.EP9 bit indicates whether or not a receive interrupt is pending for this endpoint. 0 No Interrupt 1 Interrupt Pending |

Table 27-64: USB\_INTRRX Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 8 (RC/NW)          | EP8        | Endpoint 8 Rx Interrupt. The USB_INTRRX.EP8 bit indicates whether or not a receive interrupt is pending for this endpoint. |
| 8 (RC/NW)          | EP8        | 0 No Interrupt                                                                                                             |
| 8 (RC/NW)          | EP8        | 1 Interrupt Pending                                                                                                        |
| 7 (RC/NW)          | EP7        | Endpoint 7 Rx Interrupt. The USB_INTRRX.EP7 bit indicates whether or not a receive interrupt is pending for this endpoint. |
| 7 (RC/NW)          | EP7        | 0 No Interrupt                                                                                                             |
| 7 (RC/NW)          | EP7        | 1 Interrupt Pending                                                                                                        |
| 6 (RC/NW)          | EP6        | Endpoint 6 Rx Interrupt. The USB_INTRRX.EP6 bit indicates whether or not a receive interrupt is pending for this endpoint. |
| 6 (RC/NW)          | EP6        | 0 No Interrupt                                                                                                             |
| 6 (RC/NW)          | EP6        | 1 Interrupt Pending                                                                                                        |
| 5 (RC/NW)          | EP5        | Endpoint 5 Rx Interrupt. The USB_INTRRX.EP5 bit indicates whether or not a receive interrupt is pending for this endpoint. |
| 5 (RC/NW)          | EP5        | 0 No Interrupt                                                                                                             |
| 5 (RC/NW)          | EP5        | 1 Interrupt Pending                                                                                                        |
| 4 (RC/NW)          | EP4        | Endpoint 4 Rx Interrupt. The USB_INTRRX.EP4 bit indicates whether or not a receive interrupt is pending for this endpoint. |
| 4 (RC/NW)          | EP4        | 0 No Interrupt                                                                                                             |
| 4 (RC/NW)          | EP4        | 1 Interrupt Pending                                                                                                        |
| 3 (RC/NW)          | EP3        | Endpoint 3 Rx Interrupt. The USB_INTRRX.EP3 bit indicates whether or not a receive interrupt is pending for this endpoint. |
| 3 (RC/NW)          | EP3        | 0 No Interrupt                                                                                                             |
| 3 (RC/NW)          | EP3        | 1 Interrupt Pending                                                                                                        |

Table 27-64: USB\_INTRRX Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| 2 (RC/NW)          | EP2        | Endpoint 2 Rx Interrupt. The USB_INTRRX.EP2 bit indicates whether or not a receive interrupt is pending for this endpoint. | Endpoint 2 Rx Interrupt. The USB_INTRRX.EP2 bit indicates whether or not a receive interrupt is pending for this endpoint. |
| 2 (RC/NW)          | EP2        | 0                                                                                                                          | No Interrupt                                                                                                               |
| 2 (RC/NW)          | EP2        | 1                                                                                                                          | Interrupt Pending                                                                                                          |
| 1 (RC/NW)          | EP1        | Endpoint 1 Rx Interrupt. The USB_INTRRX.EP1 bit indicates whether or not a receive interrupt is pending                    | Endpoint 1 Rx Interrupt. The USB_INTRRX.EP1 bit indicates whether or not a receive interrupt is pending                    |
| 1 (RC/NW)          | EP1        | 0                                                                                                                          | No Interrupt                                                                                                               |
| 1 (RC/NW)          | EP1        | 1                                                                                                                          | Interrupt Pending                                                                                                          |

## Receive Interrupt Enable Register

The USB\_INTRRXE register enables interrupts for the receive (Rx) endpoints. Enabling an interrupt in this register directs the USB controller to generate an interrupt if the corresponding interrupt pending bit in the USB\_INTRRX register is set.

Figure 27-89: USB\_INTRRXE Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000088_6ad1922aa94c57d0a46976f007a07a6de9a200c413cc10654e071ed9735f876d.png)

Table 27-65: USB\_INTRRXE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | EP11       | Endpoint 11 Rx Interrupt Enable. The USB_INTRRXE.EP11 bit enables the receive interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt |
| 10 (R/W)           | EP10       | Endpoint 10 Rx Interrupt Enable. The USB_INTRRXE.EP10 bit enables the receive interrupt for this endpoint. 0 Disable Interrupt                    |
| 9 (R/W)            | EP9        | Endpoint 9 Rx Interrupt Enable. The USB_INTRRXE.EP9 bit enables the receive interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt   |

Table 27-65: USB\_INTRRXE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | EP8        | Endpoint 8 Rx Interrupt Enable. The USB_INTRRXE.EP8 bit enables the receive interrupt for this endpoint. 0 Disable Interrupt                    |
| 7 (R/W)            | EP7        | Endpoint 7 Rx Interrupt Enable. The USB_INTRRXE.EP7 bit enables the receive interrupt for this endpoint. 0 Disable Interrupt                    |
| 6 (R/W)            | EP6        | Endpoint 6 Rx Interrupt Enable. The USB_INTRRXE.EP6 bit enables the receive interrupt for this endpoint. 0 Disable Interrupt                    |
| 5 (R/W)            | EP5        | Endpoint 5 Rx Interrupt Enable. The USB_INTRRXE.EP5 bit enables the receive interrupt for this 0 Disable Interrupt                              |
| 4                  |            | endpoint. 1 Enable Interrupt                                                                                                                    |
| (R/W)              | EP4        | Endpoint 4 Rx Interrupt Enable. The USB_INTRRXE.EP4 bit enables the receive interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt |
| 3 (R/W)            | EP3        | Endpoint 3 Rx Interrupt Enable. The USB_INTRRXE.EP3 bit enables the receive interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt |
| 2 (R/W)            | EP2        | Endpoint 2 Rx Interrupt Enable. The USB_INTRRXE.EP2 bit enables the receive interrupt for this endpoint.                                        |
|                    |            | 0 Disable Interrupt                                                                                                                             |

Table 27-65: USB\_INTRRXE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | EP1        | Endpoint 1 Rx Interrupt Enable. The USB_INTRRXE.EP1 bit enables the receive interrupt for this endpoint. |

## Transmit Interrupt Register

The USB\_INTRTX register indicates which interrupts are currently active for endpoint 0 and the transmit (Tx) endpoints. Note that the USB controller automatically clears this register when it is read.

Figure 27-90: USB\_INTRTX Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000089_aecc21f31b25c11bcd148cebaf8502ad4de3a0f1140f4708e60597242f27a35c.png)

Table 27-66: USB\_INTRTX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (RC/NW)         | EP11       | Endpoint 11 Tx Interrupt. The USB_INTRTX.EP11 bit indicates whether or not a transmit interrupt is pend- ing for this endpoint.                                |
| 10 (RC/NW)         | EP10       | Endpoint 10 Tx Interrupt. The USB_INTRTX.EP10 bit indicates whether or not a transmit interrupt is pend- ing for this endpoint.                                |
| 9 (RC/NW)          | EP9        | Endpoint 9 Tx Interrupt. The USB_INTRTX.EP9 bit indicates whether or not a transmit interrupt is pending for this endpoint. 0 No Interrupt 1 Interrupt Pending |

Table 27-66: USB\_INTRTX Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|
| 8 (RC/NW)          | EP8        | Endpoint 8 Tx Interrupt. The USB_INTRTX.EP8 bit indicates whether or not a transmit interrupt is pending for this endpoint. |
| 8 (RC/NW)          | EP8        | 0 No Interrupt                                                                                                              |
| 8 (RC/NW)          | EP8        | 1 Interrupt Pending                                                                                                         |
| 7 (RC/NW)          | EP7        | Endpoint 7 Tx Interrupt. The USB_INTRTX.EP7 bit indicates whether or not a transmit interrupt is pending for this endpoint. |
| 7 (RC/NW)          | EP7        | 0 No Interrupt                                                                                                              |
| 7 (RC/NW)          | EP7        | 1 Interrupt Pending                                                                                                         |
| 6 (RC/NW)          | EP6        | Endpoint 6 Tx Interrupt. The USB_INTRTX.EP6 bit indicates whether or not a transmit interrupt is pending for this endpoint. |
| 6 (RC/NW)          | EP6        | 0 No Interrupt                                                                                                              |
| 6 (RC/NW)          | EP6        | 1 Interrupt Pending                                                                                                         |
| 5 (RC/NW)          | EP5        | Endpoint 5 Tx Interrupt. The USB_INTRTX.EP5 bit indicates whether or not a transmit interrupt is pending for this endpoint. |
| 5 (RC/NW)          | EP5        | 0 No Interrupt                                                                                                              |
| 5 (RC/NW)          | EP5        | 1 Interrupt Pending                                                                                                         |
| 4 (RC/NW)          | EP4        | Endpoint 4 Tx Interrupt. The USB_INTRTX.EP4 bit indicates whether or not a transmit interrupt is pending for this endpoint. |
| 4 (RC/NW)          | EP4        | 0 No Interrupt                                                                                                              |
| 4 (RC/NW)          | EP4        | 1 Interrupt Pending                                                                                                         |
| 3 (RC/NW)          | EP3        | Endpoint 3 Tx Interrupt. The USB_INTRTX.EP3 bit indicates whether or not a transmit interrupt is pending for this endpoint. |
| 3 (RC/NW)          | EP3        | 0 No Interrupt                                                                                                              |
| 3 (RC/NW)          | EP3        | 1 Interrupt Pending                                                                                                         |

Table 27-66: USB\_INTRTX Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (RC/NW)          | EP2        | Endpoint 2 Tx Interrupt. The USB_INTRTX.EP2 bit indicates whether or not a transmit interrupt is pending for this endpoint.                                    |
| 1 (RC/NW)          | EP1        | Endpoint 1 Tx Interrupt. The USB_INTRTX.EP1 bit indicates whether or not a transmit interrupt is pending for this endpoint.                                    |
| 0 (RC/NW)          | EP0        | Endpoint 0 Tx Interrupt. The USB_INTRTX.EP0 bit indicates whether or not a transmit interrupt is pending for this endpoint. 0 No Interrupt 1 Interrupt Pending |

## Transmit Interrupt Enable Register

The USB\_INTRTXE register enables interrupts for endpoint 0 and the transmit (Tx) endpoints. Enabling an interrupt in this register directs the USB controller to generate an interrupt if the corresponding interrupt pending bit in the USB\_INTRTX register is set.

Figure 27-91: USB\_INTRTXE Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000090_5b47a2297cc383593f1b6af9165a5f4e3b6b715eae0c4c263bed2f8e45a2bd72.png)

Table 27-67: USB\_INTRTXE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | EP11       | Endpoint 11 Tx Interrupt Enable. The USB_INTRTXE.EP11 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt |
| 10 (R/W)           | EP10       | Endpoint 10 Tx Interrupt Enable. The USB_INTRTXE.EP10 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt                    |
| 9 (R/W)            | EP9        | Endpoint 9 Tx Interrupt Enable. The USB_INTRTXE.EP9 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt   |

Table 27-67: USB\_INTRTXE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | EP8        | Endpoint 8 Tx Interrupt Enable. The USB_INTRTXE.EP8 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt                    |
| 7 (R/W)            | EP7        | Endpoint 7 Tx Interrupt Enable. The USB_INTRTXE.EP7 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt                    |
| 6 (R/W)            | EP6        | Endpoint 6 Tx Interrupt Enable. The USB_INTRTXE.EP6 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt                    |
| 5 (R/W)            | EP5        | Endpoint 5 Tx Interrupt Enable. The USB_INTRTXE.EP5 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt                    |
| 4                  |            | 1 Enable Interrupt                                                                                                                               |
| (R/W)              | EP4        | Endpoint 4 Tx Interrupt Enable. The USB_INTRTXE.EP4 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt |
| 3 (R/W)            | EP3        | Endpoint 3 Tx Interrupt Enable. The USB_INTRTXE.EP3 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt |
| 2 (R/W)            | EP2        | Endpoint 2 Tx Interrupt Enable. The USB_INTRTXE.EP2 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt                    |

Table 27-67: USB\_INTRTXE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | EP1        | Endpoint 1 Tx Interrupt Enable. The USB_INTRTXE.EP1 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt |
| 0 (R/W)            | EP0        | Endpoint 0 Tx Interrupt Enable. The USB_INTRTXE.EP0 bit enables the transmit interrupt for this endpoint. 0 Disable Interrupt 1 Enable Interrupt |

## Common Interrupts Register

The USB\_IRQ register indicates which interrupts are currently active for USB controller system sources. Note that the USB controller automatically clears this register when it is read.

Figure 27-92: USB\_IRQ Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000091_670d1513126527ffab56c1a11478fca79261ffdc87d45f25b0214efdafcebcec.png)

Table 27-68: USB\_IRQ Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (RC/NW)          | VBUSERR    | VBUS Threshold Indicator. The USB_IRQ.VBUSERR bit indicates whether the USB controller has detected that the VBUS is below the VBUS valid threshold. This bit is valid only when the USB controller is an A device. Note that the USB_IRQ.VBUSERR bit and the USB_VBUS_CTL.DRVINT bit share an interrupt source line. 0 No Interrupt |
| 6 (RC/NW)          | SESSREQ    | Session Request Indicator. The USB_IRQ.SESSREQ bit indicates whether the USB controller has detected a session request signal. This bit is valid only when the USB controller is an A device.                                                                                                                                        |
| 5 (RC/NW)          | DISCON     | Disconnect Indicator. The USB_IRQ.DISCON bit indicates whether the USB controller has detected a de- vice disconnect (host mode) or has detected a session end (peripheral mode). 0 No Interrupt                                                                                                                                     |
| 5 (RC/NW)          | DISCON     |                                                                                                                                                                                                                                                                                                                                      |

Table 27-68: USB\_IRQ Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (RC/NW)          | CON        | Connection Indicator. The USB_IRQ.CON bit indicates whether the USB controller has detected a device connection. This bit is valid only in host mode.                                                                                                                                                            |
| 4 (RC/NW)          | CON        | 0 No Interrupt                                                                                                                                                                                                                                                                                                   |
| 3 (RC/NW)          | SOF        | Start of Frame Indicator. The USB_IRQ.SOF bit indicates whether the USB controller has detected a start of a frame.                                                                                                                                                                                              |
| 3 (RC/NW)          | SOF        | 0 No Interrupt                                                                                                                                                                                                                                                                                                   |
| 3 (RC/NW)          | SOF        | 1 Interrupt Pending                                                                                                                                                                                                                                                                                              |
| 2 (RC/NW)          | RSTBABBLE  | Reset/Babble Indicator. The USB_IRQ.RSTBABBLE bit indicates whether the USB controller has detected reset signalling on the bus. In host mode, the USB controller also indicates when the USB controller detects babble. Note that the USB_IRQ.RSTBABBLE bit is only ac- tive after the first SOF has been sent. |
| 2 (RC/NW)          | RSTBABBLE  | 0 No Interrupt                                                                                                                                                                                                                                                                                                   |
| 2 (RC/NW)          | RSTBABBLE  | 1 Interrupt Pending                                                                                                                                                                                                                                                                                              |
| 1 (RC/NW)          | RESUME     | Resume Indicator. The USB_IRQ.RESUME bit indicates whether the USB controller has detected re- sume signaling on the bus while the USB controller is in suspend mode.                                                                                                                                            |
| 1 (RC/NW)          | RESUME     | 0 No Interrupt                                                                                                                                                                                                                                                                                                   |
| 1 (RC/NW)          | RESUME     | 1 Interrupt Pending                                                                                                                                                                                                                                                                                              |
| 0 (RC/NW)          | SUSPEND    | Suspend Indicator. The USB_IRQ.SUSPEND bit indicates whether the USB controller has detected sus- pend signalling on the bus. This bit is valid only in peripheral mode.                                                                                                                                         |
| 0 (RC/NW)          | SUSPEND    | 0 No Interrupt                                                                                                                                                                                                                                                                                                   |
| 0 (RC/NW)          | SUSPEND    | 1 Interrupt Pending                                                                                                                                                                                                                                                                                              |

## Link Information Register

The USB\_LINKINFO register specifies the PHY-related delays.

Figure 27-93: USB\_LINKINFO Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000092_4ec1579f3ffce0f1d0cbacaf585c46d4a0f191fede126925664b23521ca79fb0.png)

Table 27-69: USB\_LINKINFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/W)          | WTCON      | Wait for Connect/Disconnect. The USB_LINKINFO.WTCON bits set the wait time to be applied to allow the users to connect or disconnect filter. |
| 3:0 (R/W)          | WTID       | Wait from ID Pull-up. The USB_LINKINFO.WTID bits set the delay to be applied from IDPULLUP being asserted to IDDIG being considered valid.   |

## LPM Attribute Register

The USB\_LPM\_ATTR register defines the link power management (LPM) attributes for LPM transactions and sleep/wake operation. In peripheral mode, the USB\_LPM\_ATTR register contains values received in the most recent, accepted (ACK'd) LPM transaction. In host mode, the USB\_LPM\_ATTR register contains values (loaded by software) that set up the next LPM transaction. The USB controller inserts the LPM values within the next LPM transaction.

Figure 27-94: USB\_LPM\_ATTR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000093_9d1452c53dfb1309ebd7b8ef549d096a915c6e675e13782e39035cd47f26e0a3.png)

Table 27-70: USB\_LPM\_ATTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:12 (R/W)        | EP         | Endpoint. The USB_LPM_ATTR.EP bits select the endpoint in the token packet of the LPM transaction.                                                                                                                                                                                                                                                |
| 8 (R/W)            | RMTWAK     | Remote Wakeup Enable. The USB_LPM_ATTR.RMTWAK bit enables remote wakeup. This bit is applied on a temporary basis only and is only applied to the current suspend state. After the current suspend cycle, the remote wakeup capability that was negotiated during enumeration applies.                                                            |
| 7:4 (R/W)          | HIRD       | Host Initiated Resume Duration. The USB_LPM_ATTR.HIRD bits select the host-initiated resume duration. This val- ue is the minimum time that the host drives resume on the bus. The value in this regis- ter corresponds to an actual resume time of: Resume Time = 50us + HIRD*75us. This equation produces results in a range of 50us to 1200us. |
| 3:0 (R/W)          | LINKSTATE  | Link State. The USB_LPM_ATTR.LINKSTATE bits is value is provided by the host to the pe- ripheral to indicate what state the peripheral must transition to after the receipt and acceptance of a LPM transaction. (Enumerations not shown are reserved.) (L1)                                                                                      |
| 3:0 (R/W)          | LINKSTATE  | 1 Sleep State                                                                                                                                                                                                                                                                                                                                     |

## LPM Control Register

The USB\_LPM\_CTL register controls link power management (LPM) operations, including LPM enable, NAK, resume, and mode transition.

Figure 27-95: USB\_LPM\_CTL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000094_3334a752ca2c76ec39252394e4aa4a36758b63a2672ef426ad9eecf47d17ce58.png)

Table 27-71: USB\_LPM\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | NAK        | LPM NAK Enable. The USB_LPM_CTL.NAK bit enables (in peripheral mode) a NAK-all-non-LPM transactions mode for all end points, forcing a NAK response to all transactions other than an LPM transaction. This bit only takes effect after the controller has been LPM suspended. In this case, the USB controller continues to NAK, until this bit has been cleared by software. | LPM NAK Enable. The USB_LPM_CTL.NAK bit enables (in peripheral mode) a NAK-all-non-LPM transactions mode for all end points, forcing a NAK response to all transactions other than an LPM transaction. This bit only takes effect after the controller has been LPM suspended. In this case, the USB controller continues to NAK, until this bit has been cleared by software. |
| 4 (R/W)            | NAK        | 0                                                                                                                                                                                                                                                                                                                                                                              | Disable LPM NAK                                                                                                                                                                                                                                                                                                                                                                |
| 4 (R/W)            | NAK        | 1                                                                                                                                                                                                                                                                                                                                                                              | Enable LPM NAK                                                                                                                                                                                                                                                                                                                                                                 |
| 3:2 (R/W)          | EN         | LPM Enable. The USB_LPM_CTL.EN bits enable (In peripheral mode) LPM operations. The LPM operation may be enabled at different levels, which determine the response of the USB controller to LPM transactions.                                                                                                                                                                  | LPM Enable. The USB_LPM_CTL.EN bits enable (In peripheral mode) LPM operations. The LPM operation may be enabled at different levels, which determine the response of the USB controller to LPM transactions.                                                                                                                                                                  |
| 3:2 (R/W)          | EN         | 0                                                                                                                                                                                                                                                                                                                                                                              | Disable LPM. LPM and extended transactions are not supported. The USB controller does not respond to LPM transactions, and these transaction timeout.                                                                                                                                                                                                                          |
| 3:2 (R/W)          | EN         | 1                                                                                                                                                                                                                                                                                                                                                                              | Disable LPM. LPM and extended transactions are not supported. The USB controller does not respond to LPM transactions, and these transaction timeout.                                                                                                                                                                                                                          |
| 3:2 (R/W)          | EN         | 2                                                                                                                                                                                                                                                                                                                                                                              | Enable Extended Transactions. LPM is not supported, but extended transactions are supported. The USB con- troller responds to an LPM transaction with a STALL.                                                                                                                                                                                                                 |
| 3:2 (R/W)          | EN         | 3                                                                                                                                                                                                                                                                                                                                                                              | Enable LPM and Extended Transactions. Both LPM and extended transactions are supported. The USB con- troller responds with a NYET or an ACK as determined by the value of LPMXMT and other conditions.                                                                                                                                                                         |

Table 27-71: USB\_LPM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | RESUME     | LPM Resume (Remote Wakeup). The USB_LPM_CTL.RESUME bit initiates resume (remote wakeup). The operation of this bit differs from the USB_POWER.RESUME bit in that the LPM resume signal timing is controlled by hardware. When set, the USB controller asserts resume signal- ing for 50us in host mode or asserts resume signaling for the time specified by the USB_LPM_ATTR.HIRD field in device mode. The USB_LPM_CTL.RESUME bit is self-clearing. 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 0 (R/W)            | TX         | 1 LPM Resume LPM Transmit. The USB_LPM_CTL.TX bit puts the USB controller in LPM transmit mode. But, this mode operates differently in host mode versus peripheral mode. In peripheral mode, this bit is set by software to instruct the controller to transition to the L1 state upon receipt of the next LPM transaction. This bit is only effective if LPM enable ( USB_LPM_CTL.EN ) is set to 0x3. The LPM transmit enable bit can be set in the same cycle as LPM enable. If the USB_LPM_CTL.TX and USB_LPM_CTL.EN bits are enabled, the USB controller can respond in the follow- ing ways: • If no data is pending (all transmit FIFOs are empty), the USB controller responds with an ACK, clears the USB_LPM_CTL.TX bit, and generates a software inter- rupt. • If data is pending (data resides in at least one transmit FIFO), the USB controller responds with a NYET, does not clear the USB_LPM_CTL.TX bit, and generates a software interrupt. In host mode, this bit is set by software to transmit an LPM transaction. This bit is self-clearing. The USB controller clears this bit immediately on receipt of any token or after three timeouts have occurred. 0 Disable LPM Tx Tx |
| 0 (R/W)            |            | 1 Enable LPM                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 0 (R/W)            |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

## LPM Function Address Register

The USB\_LPM\_FADDR register selects the link power management (LPM) function address.

Figure 27-96: USB\_LPM\_FADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000095_966c82b5263d64752565cdd2e33214104ffe1027c73879eecb02083ce828650f.png)

Table 27-72: USB\_LPM\_FADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------|
| 6:0                | VALUE      | Function Address Value.                                                                                             |
| (R/W)              |            | The USB_LPM_FADDR.VALUE bits hold the LPM function address value that the USB controller places in the LPM payload. |

## LPM Interrupt Enable Register

The USB\_LPM\_IEN register enables the link power management (LPM) related interrupts. When an interrupt is enabled in this register and the corresponding interrupt is pending in USB\_LPM\_IRQ , the USB controller generates the interrupt. When an interrupt is disabled in this register, the corresponding interrupt may be pending in USB\_LPM\_IRQ , but the USB controller does not generate an interrupt.

Figure 27-97: USB\_LPM\_IEN Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000096_1539beecdce8a1876b32004ba897714b11a3bea5c3099eacc601b715dfada627.png)

Table 27-73: USB\_LPM\_IEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 5                  | LPMERR     | LPM Error Interrupt Enable.                                      |
| 4 (R/W)            | LPMRES     | LPM Resume Interrupt Enable. 0 Disable Interrupt                 |
| 3 (R/W)            | LPMNC      | LPM NYET Control Interrupt Enable. 0 Disable Interrupt           |
| 2                  | LPMACK     | 1 Enable Interrupt                                               |
| (R/W)              |            | LPM ACK Interrupt Enable. 0 Disable Interrupt 1 Enable Interrupt |
| 1 (R/W)            | LPMNY      | LPM NYET Interrupt Enable. 0 Disable Interrupt                   |
|                    |            | 1                                                                |
|                    |            | Enable Interrupt                                                 |

Table 27-73: USB\_LPM\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 0                  | LPMST      | LPM STALL Interrupt Enable. |
| (R/W)              |            | 0 Disable Interrupt         |
|                    |            | 1 Enable Interrupt          |

## LPM Interrupt Status Register

The USB\_LPM\_IRQ register indicates link power management (LPM) related interrupt status. The USB controller clears this register when it is read.

Figure 27-98: USB\_LPM\_IRQ Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000097_000fc083cd4b280ccdd556cfc2f6786dc617bffbff3e1a5dca6571483422ea58.png)

Table 27-74: USB\_LPM\_IRQ Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (RC/NW)          | LPMERR     | LPM Error Interrupt. The USB_LPM_IRQ.LPMERR bit indicates an LPM error interrupt condition. This interrupt has differing conditions for host mode versus peripheral mode. In peripheral mode, this bit is set if an LPM transaction is received that has a USB_LPM_ATTR.LINKSTATE field that is not supported. The USB controller re- sponds to the transaction with a STALL. Note that the USB controller updates the USB_LPM_ATTR register, so software can observe the non-compliant LPM packet payload. In host mode, this bit is set if the response to a LPM transaction is received with a bit stuff or PID error. No suspend occurs and the state of the device is now unknown. 0 No Interrupt Pending |
| 4 (RC/NW)          | LPMRES     | LPM Resume Interrupt. The USB_LPM_IRQ.LPMRES bit indicates that the USB controller has been re- sumed for any reason. This bit is mutually exclusive from the USB_POWER.RESUME bit. 0 No Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 27-74: USB\_LPM\_IRQ Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (RC/NW)          | LPMNC      | LPM NYET Control Interrupt. The USB_LPM_IRQ.LPMNC bit indicates an LPM NYET control interrupt condi- tion. This interrupt has differing conditions for host mode versus peripheral mode. In peripheral mode, this bit is set when an LPM transaction is received, and the USB controller responds with a NYET due to data pending in the transmit FIFOs. This in- terrupt may only occur when the USB_LPM_CTL.EN field is set to 11, the USB_LPM_CTL.TX field is set to 1, and there is data pending in the transmit FIFOs. In host mode, this bit is set when an LPM transaction has been transmitted, but has failed to complete. The transaction failure is due to a timeout or bit errors in the re- | LPM NYET Control Interrupt. The USB_LPM_IRQ.LPMNC bit indicates an LPM NYET control interrupt condi- tion. This interrupt has differing conditions for host mode versus peripheral mode. In peripheral mode, this bit is set when an LPM transaction is received, and the USB controller responds with a NYET due to data pending in the transmit FIFOs. This in- terrupt may only occur when the USB_LPM_CTL.EN field is set to 11, the USB_LPM_CTL.TX field is set to 1, and there is data pending in the transmit FIFOs. In host mode, this bit is set when an LPM transaction has been transmitted, but has failed to complete. The transaction failure is due to a timeout or bit errors in the re- |
| 3 (RC/NW)          | LPMNC      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | No Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 3 (RC/NW)          | LPMNC      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 2 (RC/NW)          | LPMACK     | LPM ACK Interrupt. The USB_LPM_IRQ.LPMACK bit indicates an LPM ACK interrupt condition. This interrupt has differing conditions for host mode versus peripheral mode. In peripheral mode, this bit is set when an LPM transaction is received, and the USB controller responds with an ACK. This interrupt may only occur when the USB_LPM_CTL.EN field is set to 11, the USB_LPM_CTL.TX field is set to 1, and there is no data pending in the controller transmit FIFOs. In host mode, this bit is set when an LPM transaction is transmitted, and the device responds with an ACK.                                                                                                                    | LPM ACK Interrupt. The USB_LPM_IRQ.LPMACK bit indicates an LPM ACK interrupt condition. This interrupt has differing conditions for host mode versus peripheral mode. In peripheral mode, this bit is set when an LPM transaction is received, and the USB controller responds with an ACK. This interrupt may only occur when the USB_LPM_CTL.EN field is set to 11, the USB_LPM_CTL.TX field is set to 1, and there is no data pending in the controller transmit FIFOs. In host mode, this bit is set when an LPM transaction is transmitted, and the device responds with an ACK.                                                                                                                    |
| 2 (RC/NW)          | LPMACK     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | No Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 2 (RC/NW)          | LPMACK     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 1 (RC/NW)          | LPMNY      | LPM NYET Interrupt. The USB_LPM_IRQ.LPMNY bit indicates an LPM NYET interrupt condition, but this interrupt has differing conditions for host mode versus peripheral mode. In peripheral mode, this bit is set when an LPM transaction is received, and the USB controller responds with a NYET. This interrupt may only occur when the USB_LPM_CTL.EN field is set to 11, and the USB_LPM_CTL.TX field is set to 0.                                                                                                                                                                                                                                                                                     | LPM NYET Interrupt. The USB_LPM_IRQ.LPMNY bit indicates an LPM NYET interrupt condition, but this interrupt has differing conditions for host mode versus peripheral mode. In peripheral mode, this bit is set when an LPM transaction is received, and the USB controller responds with a NYET. This interrupt may only occur when the USB_LPM_CTL.EN field is set to 11, and the USB_LPM_CTL.TX field is set to 0.                                                                                                                                                                                                                                                                                     |
| 1 (RC/NW)          | LPMNY      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | No Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 1 (RC/NW)          | LPMNY      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 27-74: USB\_LPM\_IRQ Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (RC/NW)          | LPMST      | LPM STALL Interrupt. The USB_LPM_IRQ.LPMST bit indicates an LPM STALL interrupt condition, but this interrupt has differing conditions for host mode versus peripheral mode. This bit is set when an LPM transaction is received, and the USB controller responds with a STALL. This interrupt may only occur when the USB_LPM_CTL.EN field is set to 01. In host mode, this bit is set when an LPM transaction is transmitted, and the device responds with a STALL. 0 No Interrupt Pending |
| 0 (RC/NW)          | LPMST      | 1 Interrupt Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 0 (RC/NW)          | LPMST      |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

## Low-Speed EOF 1 Register

The USB\_LS\_EOF1 register defines the minimum time gap allowed between the start of the last transaction and the end of frame for low-speed transactions.

Figure 27-99: USB\_LS\_EOF1 Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000098_d3037b0d5ec5b2c81f48c6d94fe9685eca05d0eb23c2d9a70917038754dea346.png)

Table 27-75: USB\_LS\_EOF1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Low-Speed EOF 1 Value. The USB_LS_EOF1.VALUE bits set the time before end of frame to stop beginning new transactions (in units of 1.067us) for low-speed transactions. The default setting corresponds to 121.6us. |

## MPn Receive Function Address Register

The USB\_MP[n]\_RXFUNCADDR register specifies the receive endpoint's target address in host mode. This register is not used in device mode. Note that the USB\_MP[n]\_RXFUNCADDR register does not exist for EP0.

Figure 27-100: USB\_MP[n]\_RXFUNCADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000099_1e038f6961345b8c5ebde54290b398e29dc57d0085a1e6575a30fd4c5c11e941.png)

Table 27-76: USB\_MP[n]\_RXFUNCADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------|
| 6:0                | VALUE      | Rx Function Address Value.                                                                   |
| (R/W)              |            | The USB_MP[n]_RXFUNCADDR.VALUE bits hold the address of the target device for this endpoint. |

## MPn Receive Hub Address Register

The USB\_MP[n]\_RXHUBADDR register specifies the hub address of the endpoint in host mode. This register is not used in device mode. Note that this register only needs to be programmed when a full-speed or low-speed device is connected to a high-speed hub, which carries out the necessary transaction translation. Note that the USB\_MP[n]\_RXHUBADDR register does not exist for EP0.

Figure 27-101: USB\_MP[n]\_RXHUBADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000100_5bd3e7404d1364cfc9f2e790ae8559c62a7c14d2adf9a9f5daa2cb0d162866a2.png)

Table 27-77: USB\_MP[n]\_RXHUBADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | MULTTRANS  | Multiple Transaction Translators. The USB_MP[n]_RXHUBADDR.MULTTRANS bit should be set if the hub has mul- tiple transaction translators. 0 Single Transaction Translator |
| 6:0 (R/W)          | ADDR       | Hub Address Value. The USB_MP[n]_RXHUBADDR.ADDR bits hold the address of the hub to which this device is connected.                                                      |

## MPn Receive Hub Port Register

The USB\_MP[n]\_RXHUBPORT register specifies the hub port for full-speed and low-speed endpoints in host mode. This register is not used in device mode. The USB\_MP[n]\_RXHUBPORT register lets the USB controller support SPLIT transactions. Note that the USB\_MP[n]\_RXHUBPORT register does not exist for EP0.

Figure 27-102: USB\_MP[n]\_RXHUBPORT Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000101_1dc133258426b1af2f99c2abc8d04a8c29dc2882fc0ee62540df2cea5dbbe74a.png)

Table 27-78: USB\_MP[n]\_RXHUBPORT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | VALUE      | Hub Port Value. The USB_MP[n]_RXHUBPORT.VALUE bits hold the hub port value of the target device for this endpoint. |

## MPn Transmit Function Address Register

The USB\_MP[n]\_TXFUNCADDR register specifies the transmit endpoint's target address in host mode. This register is not used in device mode. Note that the USB\_MP[n]\_TXFUNCADDR register must be setup for EP0. (The USB\_MP[n]\_RXFUNCADDR register does not exist for EP0.)

Figure 27-103: USB\_MP[n]\_TXFUNCADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000102_f3a58987813a23e58dd7dbfe40c909a983e3e48fc827ceadbdb0df4275585fa5.png)

Table 27-79: USB\_MP[n]\_TXFUNCADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | VALUE      | Tx Function Address Value. The USB_MP[n]_TXFUNCADDR.VALUE bits hold the address of the target device for this endpoint. |

## MPn Transmit Hub Address Register

The USB\_MP[n]\_TXHUBADDR register specifies the hub address of the endpoint in host mode. This register is not used in device mode. Note that this register only needs to be programmed when a full-speed or low-speed device is connected to a high-speed hub, which carries out the necessary transaction translation. Also, note that EP0 only uses the USB\_MP[n]\_TXHUBADDR register. (The USB\_MP[n]\_RXHUBADDR register does not exist for EP0.)

Figure 27-104: USB\_MP[n]\_TXHUBADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000103_5bd3e7404d1364cfc9f2e790ae8559c62a7c14d2adf9a9f5daa2cb0d162866a2.png)

Table 27-80: USB\_MP[n]\_TXHUBADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | MULTTRANS  | Multiple Transaction Translators. The USB_MP[n]_TXHUBADDR.MULTTRANS bit should be set if the hub has mul- tiple transaction translators. 0 Single Transaction Translator |
| 6:0 (R/W)          | ADDR       | Hub Address Value. The USB_MP[n]_TXHUBADDR.ADDR bits hold the address of the hub to which this device is connected.                                                      |

## MPn Transmit Hub Port Register

The USB\_MP[n]\_TXHUBPORT register specifies the hub port for full-speed and low-speed endpoints in host mode. This register is not used in device mode. The USB\_MP[n]\_TXHUBPORT register lets the USB controller support SPLIT transactions. EP0 only uses the USB\_MP[n]\_TXHUBPORT register. (The USB\_MP[n]\_RXHUBPORT register does not exist for EP0.)

Figure 27-105: USB\_MP[n]\_TXHUBPORT Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000104_7304ef44c37242195a3a37127eca30333134111aba3a2a8d4eab160602e3516a.png)

Table 27-81: USB\_MP[n]\_TXHUBPORT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | VALUE      | Hub Port Value. The USB_MP[n]_TXHUBPORT.VALUE bits hold the hub port value of the target device for this endpoint. |

## PHY Control Register

The USB\_PHY\_CTL register provides access to PHY control features.

Figure 27-106: USB\_PHY\_CTL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000105_0a1c6808867fa71cafad6c7a0a07c65562d4769ded3fcb25869f913570b277ab.png)

Table 27-82: USB\_PHY\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | EN         | PHY Enable. The USB_PHY_CTL.EN bit enables the USB controller PHY. This bit enables the schmitt-trigger inputs on D+ and D- to detect session request protocol. The bit also enables the bias circuits and VBUS comparators to detect when a host is connected. This bit should be set for all USB controller operations. | PHY Enable. The USB_PHY_CTL.EN bit enables the USB controller PHY. This bit enables the schmitt-trigger inputs on D+ and D- to detect session request protocol. The bit also enables the bias circuits and VBUS comparators to detect when a host is connected. This bit should be set for all USB controller operations. |
| 4 (R/W)            | DIS        | Disable PHY. The USB_PHY_CTL.DIS bit disables the PHY, so it draws minimal power.                                                                                                                                                                                                                                         | Disable PHY. The USB_PHY_CTL.DIS bit disables the PHY, so it draws minimal power.                                                                                                                                                                                                                                         |
|                    |            | 0                                                                                                                                                                                                                                                                                                                         | Enable USB PHY and 5V protection on USB signals.                                                                                                                                                                                                                                                                          |
|                    |            | 1                                                                                                                                                                                                                                                                                                                         | Disable USB PHY and 5V protection on USB signals. Caution: When 5V protection is disabled, the absolute max voltage on USB signals is reduced. See the data sheet for details.                                                                                                                                            |
| 1 (R/W)            | RESTORE    | Restore from Hibernate. The USB_PHY_CTL.RESTORE bit causes the PHY to come out of hibernate and release its latches.                                                                                                                                                                                                      | Restore from Hibernate. The USB_PHY_CTL.RESTORE bit causes the PHY to come out of hibernate and release its latches.                                                                                                                                                                                                      |
| 0 (R/W)            | HIBER      | Hibernate. The USB_PHY_CTL.HIBER bit causes the PHY to prepare for hibernate. Latches hold the pullup/pulldown state when the core power is removed.                                                                                                                                                                      | Hibernate. The USB_PHY_CTL.HIBER bit causes the PHY to prepare for hibernate. Latches hold the pullup/pulldown state when the core power is removed.                                                                                                                                                                      |

## PLL and Oscillator Control Register

The USB\_PLL\_OSC register provides access to PLL and oscillator-related control features.

Figure 27-107: USB\_PLL\_OSC Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000106_2ae7500cf40da8f2e65007d8604be4f440e3f3b43b6518fd85df2c12871fe90a.png)

Table 27-83: USB\_PLL\_OSC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/NW)          | PLLSTABLE  | PLL Stable. The USB_PLL_OSC.PLLSTABLE status bit indicates that the oscillator and PLL clock are stable.                                                      |
| 7 (R/W)            | PLLMSEL    | PLL Multiplier Select. The USB_PLL_OSC.PLLMSEL bit directs the PLL to use the PLL multiplier value stored in the USB_PLL_OSC.PLLM bits.                       |
| 6:1 (R/W)          | PLLM       | PLL Multiplier Value. The USB_PLL_OSC.PLLM bit field contains the PLL multiplier. This field should be set such that CLKIN * USB_PLL_OSC.PLLM value = 480MHz. |
| 0 (R/W)            | DIVCLKIN   | Divide CLKIN. The USB_PLL_OSC.DIVCLKIN bit enables a divide CLKIN by 2 function for the PLL.                                                                  |

## Power and Device Control Register

The USB\_POWER register controls suspend and resume signaling and some operational aspects of the USB controller.

Figure 27-108: USB\_POWER Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000107_0825afe57d8b07b91885ee13d5f4313b2bcfaaf541482d033c11681b11e67ecd.png)

Table 27-84: USB\_POWER Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | ISOUPDT    | ISO Update Enable. The USB_POWER.ISOUPDT bit directs the USB controller to waits for an SOF to- ken from the time the USB_EP[n]_TXCSR_P.TXPKTRDY bit is set before send- ing the packet. If an IN token is received before an SOF token, the USB controller send a zero length data packet. This USB_POWER.ISOUPDT bit only affects end- points performing isochronous transfers. This bit only is valid in peripheral mode ( USB_DEV_CTL.HOSTMODE = 0). | ISO Update Enable. The USB_POWER.ISOUPDT bit directs the USB controller to waits for an SOF to- ken from the time the USB_EP[n]_TXCSR_P.TXPKTRDY bit is set before send- ing the packet. If an IN token is received before an SOF token, the USB controller send a zero length data packet. This USB_POWER.ISOUPDT bit only affects end- points performing isochronous transfers. This bit only is valid in peripheral mode ( USB_DEV_CTL.HOSTMODE = 0). |
| 6 (R/W)            | SOFTCONN   | Soft Connect/Disconnect Enable. In peripheral mode, the D+/- lines default to disconnected. Setting this bit enables the D+/- termination resistors. This bit is automatically set when the USB_DEV_CTL.SESSION bit is written with 1. The USB_POWER.SOFTCONN bit enables USB controller soft connect/disconnect, enabling the termination resistors                                                                                                     | Soft Connect/Disconnect Enable. In peripheral mode, the D+/- lines default to disconnected. Setting this bit enables the D+/- termination resistors. This bit is automatically set when the USB_DEV_CTL.SESSION bit is written with 1. The USB_POWER.SOFTCONN bit enables USB controller soft connect/disconnect, enabling the termination resistors                                                                                                     |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Disable Soft Connect/Disconnect                                                                                                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Enable Soft Connect/Disconnect                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 27-84: USB\_POWER Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | HSEN       | High-Speed Mode Enable. The USB_POWER.HSEN bit enables USB controller negotiation for high speed (on devices supporting high-speed mode) when the device is reset by the hub/host. If disa- bled, the USB controller only operates in full-speed mode. When operating in full- speed mode, this bit should be cleared.                                                                                                                                                                                                                                   |
| 4 (R/NW)           | HSMODE     | 1 Enable Negotiation for HS Mode High-Speed Mode. The USB_POWER.HSMODE bit indicates whether or not the USB controller success- fully negotiated high-speed mode during a USB controller reset. In peripheral mode ( USB_DEV_CTL.HOSTMODE = 0), this bit has valid data when the USB controller completes reset. In host mode ( USB_DEV_CTL.HOSTMODE = 1), this bit has valid data when the USB_IRQ.RSTBABBLE bit is cleared, remaining valid for the dura-                                                                                              |
| 3 (R/W)            | RESET      | 1 High-Speed Mode (HS success during reset) Reset USB. The USB_POWER.RESET bit indicates (in both host and peripheral modes) that the USB controller has detected that reset signaling is present on the bus. In peripheral                                                                                                                                                                                                                                                                                                                              |
|                    |            | mode ( USB_DEV_CTL.HOSTMODE = 0), this bit is read only, but in host mode ( USB_DEV_CTL.HOSTMODE = 1), this bit is read/write, permitting the processor core to set the bit and initiate a USB controller reset. 0 No Reset 1 Reset USB                                                                                                                                                                                                                                                                                                                  |
| 2 (R/W)            | RESUME     | Resume Mode. The USB_POWER.RESUME bit directs the USB controller to generate resume signal- ing when the function is in suspend mode ( USB_POWER.SUSPEND =1). The pro- cessor core should clear this bit after 10 ms (a maximum of 15 ms) to end resume sig- naling. When the USB controller is in host mode ( USB_DEV_CTL.HOSTMODE = 1), the USB controller automatically sets the USB_POWER.RESUME bit when re- sume signaling from the target is detected while the USB controller is suspended. 0 Disable Resume Signaling 1 Enable Resume Signaling |

Table 27-84: USB\_POWER Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1S)          | SUSPEND    | Suspend Mode. When the USB controller is in host mode ( USB_DEV_CTL.HOSTMODE = 1), the USB_POWER.SUSPEND bit enables suspend mode. When the USB controller is in peripheral mode ( USB_DEV_CTL.HOSTMODE = 0), the USB controller sets the USB_POWER.SUSPEND bit on entry to suspend mode and clears the bit when the processor reads the USB_IRQ register. Note that the USB controller automatically clears this bit if the USB_POWER.RESUME bit is set. | Suspend Mode. When the USB controller is in host mode ( USB_DEV_CTL.HOSTMODE = 1), the USB_POWER.SUSPEND bit enables suspend mode. When the USB controller is in peripheral mode ( USB_DEV_CTL.HOSTMODE = 0), the USB controller sets the USB_POWER.SUSPEND bit on entry to suspend mode and clears the bit when the processor reads the USB_IRQ register. Note that the USB controller automatically clears this bit if the USB_POWER.RESUME bit is set. |
| 0 (R/W)            | SUSEN      | SUSPENDM Output Enable. The USB_POWER.SUSEN bit enables the SUSPENDM output (internal USB con- troller signal). When enabled, the SUSPENDM output signal is used by the USB con- troller PHY to power-down its drivers when the USB controller is not active.                                                                                                                                                                                             | SUSPENDM Output Enable. The USB_POWER.SUSEN bit enables the SUSPENDM output (internal USB con- troller signal). When enabled, the SUSPENDM output signal is used by the USB con- troller PHY to power-down its drivers when the USB controller is not active.                                                                                                                                                                                             |
| 0 (R/W)            | SUSEN      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Disable SUSPENDM Output                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | SUSEN      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Enable SUSPENDM Output                                                                                                                                                                                                                                                                                                                                                                                                                                    |

## RAM Information Register

The USB\_RAMINFO register provides information about the width of the USB controller RAM.

Figure 27-109: USB\_RAMINFO Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000108_0ac9a5cd5c331170c7f351febd38dd7be1083575c10addcb1166c7516433118e.png)

Table 27-85: USB\_RAMINFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | DMACHANS   | DMAChannels. The USB_RAMINFO.DMACHANS bits indicate the number of DMAchannels.                                                                                                                                                          |
| 3:0 (R/NW)         | RAMBITS    | RAM Address Bits. The USB_RAMINFO.RAMBITS bits indicate the number of RAM address bits. The USB controller FIFO RAM is 32-bits wide. The number of bytes in the FIFO RAM may be calculated from the formula: RAM_bytes = 2 (RAM_Bits+2) |

## EPn Request Packet Count Register

The USB\_RQPKTCNT[n] register specifies (in host mode) the number of packets to request in a block transfer of one or more bulk packets of size USB\_EP[n]\_RXMAXP from a receive endpoint. This register only applies for receive endpoints 1 through 11 in host mode.

Figure 27-110: USB\_RQPKTCNT[n] Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000109_5b003484579eb9330607887850ed37618f2132841f72055bda904df66ddac8b2.png)

Table 27-86: USB\_RQPKTCNT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Request Packet Count Value. The USB_RQPKTCNT[n].VALUE bits specify the number of bulk packets to re- quest in a block transfer from a receive endpoint. This field is used with the auto re- quest feature ( USB_EP[n]_RXCSR_H.AUTOREQ ). |

## Receive FIFO Address Register

The USB\_RXFIFOADDR sets the start address for the selected Rx FIFO for endpoints 1-11. There is one receive FIFO address register for each endpoint, except endpoint 0. The USB\_RXFIFOADDR register is indexed and selected by the USB\_INDEX register. Note that the endpoint 0 FIFO has a fixed 64-byte size and is always located at address 0.

Figure 27-111: USB\_RXFIFOADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000110_af179f8b755388f493ad1915ab13e8aa515f66339d9f2be92c604ec61ff35354.png)

Table 27-87: USB\_RXFIFOADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VALUE      | Rx FIFO Start Address. The USB_RXFIFOADDR.VALUE bits hold the start address of the selected endpoint FIFO (selected with the USB_INDEX register) in units of 8 bytes, according to the formula: FIFO address = USB_RXFIFOADDR.VALUE * 8 |

## Receive FIFO Size Register

The USB\_RXFIFOSZ register defines the maximum amount of data that can be transferred through the selected receive endpoint in a single frame. When setting this value, consider the constraints placed by the USB specification on packet sizes for bulk, interrupt and isochronous transactions in full-speed operations. This register provides indexed-access to the FIFO (packet) size selection for each Rx endpoint (except endpoint 0).

Note that a value greater than the maximum allowed of 1023 for full-speed USB controller operation produces unpredictable results.

Also, note that the value written to this register should match the programmed maximum individual packet size (MaxPktSize) of the standard endpoint descriptor for the associated endpoint. (See the Universal Serial Bus Specification Revision 2.0, Chapter 9). A mismatch could cause unexpected results. The total amount of data represented by the value written to this register must not exceed the Rx FIFO size, and should not exceed half the FIFO size if double-buffering is required.

Figure 27-112: USB\_RXFIFOSZ Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000111_7df307c67ebf557c42709c6948e8c457f7f740902d41a20587caddea33205ca5.png)

Table 27-88: USB\_RXFIFOSZ Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DPB        | Double Packet Buffering Enable. The USB_RXFIFOSZ.DPB bit enables double packet buffering, doubling the FIFO (packet) size selected with the USB_RXFIFOSZ.SZ field.                                                                                                                                                                                                                                                                                                 | Double Packet Buffering Enable. The USB_RXFIFOSZ.DPB bit enables double packet buffering, doubling the FIFO (packet) size selected with the USB_RXFIFOSZ.SZ field.                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | DPB        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Single Packet Buffering                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 4 (R/W)            | DPB        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Double Packet Buffering                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 3:0 (R/W)          | SZ         | Maximum Packet Size. The USB_RXFIFOSZ.SZ bits select the maximum FIFO (packet) size according to the formula: FIFOSZ= 2 (SZ+3) If the USB_RXFIFOSZ.DPB is cleared, the FIFO size is FIFOSZ from this formula. If the USB_RXFIFOSZ.DPB is set, the FIFO is twice this size. For each enumeration value, the enumerations descriptions show the packet size (PktSz=), the FIFO size if DPB=0 (DPB0=), and the FIFO size if DPB=1 (DPB1=); these values are in bytes. | Maximum Packet Size. The USB_RXFIFOSZ.SZ bits select the maximum FIFO (packet) size according to the formula: FIFOSZ= 2 (SZ+3) If the USB_RXFIFOSZ.DPB is cleared, the FIFO size is FIFOSZ from this formula. If the USB_RXFIFOSZ.DPB is set, the FIFO is twice this size. For each enumeration value, the enumerations descriptions show the packet size (PktSz=), the FIFO size if DPB=0 (DPB0=), and the FIFO size if DPB=1 (DPB1=); these values are in bytes. |
| 3:0 (R/W)          | SZ         | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | PktSz=8, DPB0=8, DPB1=16                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 3:0 (R/W)          | SZ         | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | PktSz=16, DPB0=16, DPB1=32                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 27-88: USB\_RXFIFOSZ Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration | Description/Enumeration          |
|-----------|------------|---------------------------|----------------------------------|
| (Access)  |            |                           |                                  |
|           |            |                         2 | PktSz=32, DPB0=32, DPB1=64       |
|           |            |                         3 | PktSz=64, DPB0=64, DPB1=128      |
|           |            |                         4 | PktSz=128, DPB0=128, DPB1=256    |
|           |            |                         5 | PktSz=256, DPB0=256, DPB1=512    |
|           |            |                         6 | PktSz=512, DPB0=512, DPB1=1024   |
|           |            |                         7 | PktSz=1024, DPB0=1024, DPB1=2048 |
|           |            |                         8 | PktSz=2048, DPB0=2048, DPB1=4096 |
|           |            |                         9 | PktSz=4096, DPB0=4096, DPB1=8192 |

## Software Reset Register

The USB\_SOFT\_RST register provides reset controls for the USB controller CLK domain and XCLK domain. The USB controller PHY operates in the controller's XCLK domain, and the USB controller interface to the processor core operates in the controller's CLK domain. Note that for correct operation, both of the reset control bits ( USB\_SOFT\_RST.RST and USB\_SOFT\_RST.RSTX ) should always be asserted simultaneously.

Figure 27-113: USB\_SOFT\_RST Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000112_ccee6ca9f1f18ff42b4c12303dc65217405daa9b5cece3e4c2ec90453ae36b43.png)

Table 27-89: USB\_SOFT\_RST Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | RSTX       | Reset USB XCLK Domain. The USB_SOFT_RST.RSTX bit resets logic in the USB XCLK domain. This bit is self-clearing. Note that this bit should always be asserted simultaneously with the USB_SOFT_RST.RST bit. | Reset USB XCLK Domain. The USB_SOFT_RST.RSTX bit resets logic in the USB XCLK domain. This bit is self-clearing. Note that this bit should always be asserted simultaneously with the USB_SOFT_RST.RST bit. |
| 0 (R/W)            | RST        | Reset USB CLK Domain. The USB_SOFT_RST.RST bit resets logic in the USB CLK domain. This bit is self- clearing. Note that this bit should always be asserted simultaneously with the USB_SOFT_RST.RSTX bit.  | Reset USB CLK Domain. The USB_SOFT_RST.RST bit resets logic in the USB CLK domain. This bit is self- clearing. Note that this bit should always be asserted simultaneously with the USB_SOFT_RST.RSTX bit.  |
| 0 (R/W)            | RST        | 0                                                                                                                                                                                                           | No Reset                                                                                                                                                                                                    |
| 0 (R/W)            | RST        | 1                                                                                                                                                                                                           | Reset USB CLK Domain                                                                                                                                                                                        |

## Testmode Register

The USB\_TESTMODE register places the USB controller into the test mode state and can also put the USB controller into one of the test modes for high-speed operation. For more information about these modes, see the USB 2.0 specification.

Note that the USB\_TESTMODE register is not used in normal operation. Only one of the test mode (bits 0-6) selection bits may be set at a time.

Figure 27-114: USB\_TESTMODE Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000113_82b187aa2161585e0bb22d01053d116fe03c7ce581ee9d76c1dc9b3c9b0a740b.png)

Table 27-90: USB\_TESTMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | FIFOACCESS | FIFO Access. The USB_TESTMODE bit directs the USB controller to transfer the packet in the end- point 0 Tx FIFO to the endpoint 0 Rx FIFO. The bit is cleared automatically.                                                                                                                                                                                                                                                                    |
| 3 (R/W)            | TESTPACKET | Test_Packet Mode. The USB_TESTMODE.TESTPACKET bit selects Test_Packet test mode, which ap- plies only when the USB controller is in high-speed mode. In this mode, the USB controller repetitively transmits on the bus a 53-byte test packet, whose form is defined in the USB 2.0 Specification, Section 7.1.20. Note that the test packet has a fixed for- mat and must be loaded into the endpoint 0 FIFO before this test mode is entered. |
| 2 (R/W)            | TESTK      | Test_K Mode. The USB_TESTMODE.TESTK bit selects Test_K test mode. In this mode, the USB controller transmits a continuous K on the bus.                                                                                                                                                                                                                                                                                                         |
| 1 (R/W)            | TESTJ      | Test_J Mode. The USB_TESTMODE.TESTJ bit selects Test_J test mode. In this mode, the USB controller transmits a continuous J on the bus.                                                                                                                                                                                                                                                                                                         |
| 0 (R/W)            | TESTSE0NAK | Test SE0 NAK. The USB_TESTMODE.TESTSE0NAK bit selects Test_SE0_NAK test mode, which applies only when the USB controller is in high-speed mode. In this mode, the USB controller remains in high-speed mode, but responds to any valid IN token with a NAK.                                                                                                                                                                                     |

## Transmit FIFO Address Register

The USB\_TXFIFOADDR register sets the start address for the selected Tx FIFO for endpoints 1-11. There is one transmit FIFO address register for each endpoint, except endpoint 0. The USB\_TXFIFOADDR register is indexed and selected by the USB\_INDEX register. Note that the endpoint 0 FIFO has a fixed 64-byte size and is always located at address 0.

Figure 27-115: USB\_TXFIFOADDR Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000114_eea7a39195c28479110078687f29879e11c6142d90edbb503f691b059d010543.png)

Table 27-91: USB\_TXFIFOADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VALUE      | Tx FIFO Start Address. The USB_TXFIFOADDR.VALUE bits hold the start address of the selected endpoint FIFO (selected with the USB_INDEX register) in units of 8 bytes, according to the formula: FIFO address = USB_TXFIFOADDR.VALUE * 8 |

## Transmit FIFO Size Register

The USB\_TXFIFOSZ register defines the maximum amount of data that can be transferred through the selected transmit endpoint in a single frame. When setting this value, consider the constraints placed by the USB specification on packet sizes for bulk, interrupt and isochronous transactions in full-speed operations. This register provides indexed access to the FIFO (packet) size selection for each Tx endpoint (except endpoint 0).

Figure 27-116: USB\_TXFIFOSZ Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000115_47c261774fe4f3acbf0828283fb776a957ebaba7ac4a64bcc068ee15ca86dfa3.png)

Table 27-92: USB\_TXFIFOSZ Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DPB        | Double Packet Buffering Enable. The USB_TXFIFOSZ.DPB bit enables double packet buffering, doubling the FIFO (packet) size selected with the USB_TXFIFOSZ.SZ field.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 3:0 (R/W)          | SZ         | 1 Double Packet Buffering Maximum Packet Size. The USB_TXFIFOSZ.SZ bits select the maximum FIFO (packet) size according to the formula: FIFOSZ= 2 (SZ+3) If the USB_TXFIFOSZ.DPB is cleared, the FIFO size is FIFOSZ from this formula. If the USB_TXFIFOSZ.DPB is set, the FIFO is twice this size. For each enumeration value, the enumerations descriptions show the packet size (PktSz=), the FIFO size if DPB=0 (DPB0=), and the FIFO size if DPB=1 (DPB1=); these values are in bytes. 0 PktSz=8, DPB0=8, DPB1=16 1 PktSz=16, DPB0=16, DPB1=32 2 PktSz=32, DPB0=32, DPB1=64 3 PktSz=64, DPB0=64, DPB1=128 4 PktSz=128, DPB0=128, DPB1=256 5 PktSz=256, DPB0=256, DPB1=512 |

## Table 27-92: USB\_TXFIFOSZ Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration            |
|-----------|------------|------------------------------------|
| (Access)  |            |                                    |
|           |            | 8 PktSz=2048, DPB0=2048, DPB1=4096 |
|           |            | 9 PktSz=4096, DPB0=4096, DPB1=8192 |

## VBUS Control Register

The USB\_VBUS\_CTL controls USB controller VBUS-related features.

Figure 27-117: USB\_VBUS\_CTL Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000116_d9c00ac8aaf202f38b506c08e104d5b012c703f852c33069a08382c3b75fc70d.png)

Table 27-93: USB\_VBUS\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------|
| 4 (R/NW)           | DRV        | VBUS Drive. The USB_VBUS_CTL.DRV bit indicates the state of the UTMI+ DrvVBUS signal from the USB controller. |
| 3 (R/W1C)          | DRVINT     | VBUS Drive Interrupt. The USB_VBUS_CTL.DRVINT bit indicates the state of the DrvVBUSInt interrupt.            |
| 2 (R/W)            | DRVIEN     | VBUS Drive Interrupt Enable. The USB_VBUS_CTL.DRVIEN bit enables the DrvVBUS interrupt.                       |
| 1 (R/W)            | DRVOD      | VBUS Drive Open Drain. The USB_VBUS_CTL.DRVOD selects whether the DrvVBUS output is open drain.               |
| 0 (R/W)            | INVDRV     | VBUS Invert Drive. The USB_VBUS_CTL.INVDRV bit selects whether the DrvVBUS output is invert- ed.              |

## VBUS Pulse Length Register

The USB\_VPLEN register defines the duration of the VBUS pulsing charge for SRP initiation.

Figure 27-118: USB\_VPLEN Register Diagram

![Image](30_Universal_Serial_Bus_(USB)_artifacts/image_000117_9463e9bad89a9000d6748d2a4c83b6244dd428c6653135c018a68062d5ee42e0.png)

Table 27-94: USB\_VPLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | VBUS Pulse Length Value. The USB_VPLEN.VALUE bits sets the duration of the VBUS pulsing charge in units of 546.1us. The default setting corresponds to 32.77ms. Note that VBUS pulsing was removed in the OTG specification v2.0, section 5.1.4. |