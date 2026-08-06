# Ethernet Media Access Controller (EMAC) — EMAC Functional Description

<!-- source: 032_Ethernet_Media_Access_Controller_EMAC_EMAC_Functional_Descri.pdf | original pages 1647–1743 -->

- Separate DMA, Tx FIFO, and Rx FIFO (MTL) for each additional channel
- Programmable control to route received VLAN tagged non-AV packets to channels or queues
- Standard IEEE 802.3az-2010 for Energy Efficient Ethernet
- EMAC0 supports the following FIFO sizes: 2048 bytes for transmit FIFO and also for receive FIFO
- Supports MII/RMII/RGMII interfaces for external PHY interface

## EMAC Functional Description

This section provides information on the function of Ethernet MAC peripheral.

## Hardware for the Media Access Control protocol

This function allows applications to support TCP/IP based network communication. At the system end, the module supports direct connection with the system crossbar bus for memory or MMR transactions. It supports RMII (Reduced Media Independent Interface), RGMII (Reduced Gigabit Media Independent Interface), MII (Media Independent Interface), and SMI (Station Management Interface) for interfacing with the external PHY chip.

## Dedicated DMA Controller with independent read/write channels

Performs both data and status transfers between the application and the media independent interfaces. Internal transmit and receive FIFOs are used to buffer and regulate the frames. Dedicated interrupt lines connect the EMAC interrupt sources to the System Event Controller (SEC).

## MAC Management Counters (MMC) block

An extended set of registers that collect various statistics compliant with IEEE 802.3 definitions regarding the operation of the interface. The registers are updated for each new transmitted or received frame when the condition to update the counter is met. The EMAC provides a set of such counters, along with extended usage control.

## PTP (Precision Time Protocol) engine

Provides hardware assistance for the implementation of the IEEE 1588 version 1 and version 2 standards, which allows time synchronization between systems.

## Audio Video (AV) functionality

Enables transmission of time-sensitive traffic over bridged local area networks (LANs). The EMAC provides hardware support for IEEE 802.1-Qav specified credit-based shaper (CBS) algorithm. In addition, slot number function allows scheduling of fetching of data by DMA from system memory.

## ADSP-2159x\_SC592\_SC594 EMAC Register List

The Ethernet MAC (EMAC) module provides an Ethernet interface to the processor. The interface is compliant to IEEE Standard 802.3-2005. A set of registers govern EMAC operations. For more information on EMAC functionality, see the EMAC register descriptions.

Table 29-1: ADSP-2159x\_SC592\_SC594 EMAC Register List

| Name                 | Description                                    |
|----------------------|------------------------------------------------|
| EMAC_ADDR0_HI        | MAC Address 0 High Register                    |
| EMAC_ADDR0_LO        | MAC Address 0 Low Register                     |
| EMAC_ADDR1_HI        | MAC Address 1 High Register                    |
| EMAC_ADDR1_LO        | MAC Address 1 Low Register                     |
| EMAC_DBG             | Debug Register                                 |
| EMAC_DMA0_BMMODE     | DMASCB Bus Mode Register                       |
| EMAC_DMA0_BMSTAT     | DMASCB Status Register                         |
| EMAC_DMA0_BUSMODE    | DMABus Mode Register                           |
| EMAC_DMA0_IEN        | DMAInterrupt Enable Register                   |
| EMAC_DMA0_MISS_FRM   | DMAMissed Frame Register                       |
| EMAC_DMA0_OPMODE     | DMAOperation Mode Register                     |
| EMAC_DMA0_RXBUF_CUR  | DMARx Buffer Current Register                  |
| EMAC_DMA0_RXDSC_ADDR | DMARx Descriptor List Address Register         |
| EMAC_DMA0_RXDSC_CUR  | DMARx Descriptor Current Register              |
| EMAC_DMA0_RXIWDOG    | DMARx Interrupt Watch Dog Register             |
| EMAC_DMA0_RXPOLL     | DMARx Poll Demand register                     |
| EMAC_DMA0_STAT       | DMAStatus Register                             |
| EMAC_DMA0_TXBUF_CUR  | DMATx Buffer Current Register                  |
| EMAC_DMA0_TXDSC_ADDR | DMATx Descriptor List Address Register         |
| EMAC_DMA0_TXDSC_CUR  | DMATx Descriptor Current Register              |
| EMAC_DMA0_TXPOLL     | DMATx Poll Demand Register                     |
| EMAC_DMA1_BUSMODE    | DMABus Mode Register                           |
| EMAC_DMA1_CHCBSCTL   | Channel 1 Credit Shaping Control Register      |
| EMAC_DMA1_CHCBSSTAT  | Channel 1 Average Traffic Transmitted Register |
| EMAC_DMA1_CHHIC      | Channel 1 High Credit Value Register           |
| EMAC_DMA1_CHISC      | Channel 1 Idle Slope Credit Value Register     |
| EMAC_DMA1_CHLOC      | Channel 1 Low Credit Value Register            |

Table 29-1: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                 | Description                                       |
|----------------------|---------------------------------------------------|
| EMAC_DMA1_CHSFCS     | Channel 1 Control Bits for Slot Function Register |
| EMAC_DMA1_CHSSC      | Channel 1 Send Slope Credit Value Register        |
| EMAC_DMA1_IEN        | DMAInterrupt Enable Register                      |
| EMAC_DMA1_MISS_FRM   | DMAMissed Frame Register                          |
| EMAC_DMA1_OPMODE     | DMAOperation Mode Register                        |
| EMAC_DMA1_RXBUF_CUR  | DMARx Buffer Current Register                     |
| EMAC_DMA1_RXDSC_ADDR | DMARx Descriptor List Address Register            |
| EMAC_DMA1_RXDSC_CUR  | DMARx Descriptor Current Register                 |
| EMAC_DMA1_RXIWDOG    | DMARx Interrupt Watch Dog Register                |
| EMAC_DMA1_RXPOLL     | DMARx Poll Demand Register                        |
| EMAC_DMA1_STAT       | DMAStatus Register                                |
| EMAC_DMA1_TXBUF_CUR  | DMATx Buffer Current Register                     |
| EMAC_DMA1_TXDSC_ADDR | DMATx Descriptor List Address Register            |
| EMAC_DMA1_TXDSC_CUR  | DMATx Descriptor Current Register                 |
| EMAC_DMA1_TXPOLL     | DMATx Poll Demand Register                        |
| EMAC_DMA2_BUSMODE    | DMABus Mode Register                              |
| EMAC_DMA2_CHCBSCTL   | Channel 2 Credit Shaping Control Register         |
| EMAC_DMA2_CHCBSSTAT  | Channel 2 Avg Traffic Transmitted Status Register |
| EMAC_DMA2_CHHIC      | Channel 2 High Credit Value Register              |
| EMAC_DMA2_CHISC      | Channel 2 Idle Slope Credit Value Register        |
| EMAC_DMA2_CHLOC      | Channel 2 Low Credit Value Register               |
| EMAC_DMA2_CHSFCS     | Channel 2 Control Bits for Slot Function Register |
| EMAC_DMA2_CHSSC      | Channel 2 Send Slope Credit Value Register        |
| EMAC_DMA2_IEN        | DMAInterrupt Enable Register                      |
| EMAC_DMA2_MISS_FRM   | DMAMissed Frame Register                          |
| EMAC_DMA2_OPMODE     | DMAOperation Mode Register                        |
| EMAC_DMA2_RXBUF_CUR  | DMARx Buffer Current Register                     |
| EMAC_DMA2_RXDSC_ADDR | DMARx Descriptor List Address Register            |
| EMAC_DMA2_RXDSC_CUR  | DMARx Descriptor Current Register                 |
| EMAC_DMA2_RXIWDOG    | DMARx Interrupt Watch Dog Register                |
| EMAC_DMA2_RXPOLL     | DMARx Poll Demand register                        |

Table 29-1: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                 | Description                                     |
|----------------------|-------------------------------------------------|
| EMAC_DMA2_STAT       | DMAStatus Register                              |
| EMAC_DMA2_TXBUF_CUR  | DMATx Buffer Current Register                   |
| EMAC_DMA2_TXDSC_ADDR | DMATx Descriptor List Address Register          |
| EMAC_DMA2_TXDSC_CUR  | DMATx Descriptor Current Register               |
| EMAC_DMA2_TXPOLL     | DMATx Poll Demand Register                      |
| EMAC_FLOWCTL         | FLow Control Register                           |
| EMAC_GIGE_CTLSTAT    | RGMII Control and Status Register               |
| EMAC_HASHTBL_HI      | Hash Table High Register                        |
| EMAC_HASHTBL_LO      | Hash Table Low Register                         |
| EMAC_IMSK            | Interrupt Mask Register                         |
| EMAC_IPC_RXIMSK      | MMCIPC Rx Interrupt Mask Register               |
| EMAC_IPC_RXINT       | MMCIPC Rx Interrupt Register                    |
| EMAC_ISTAT           | Interrupt Status Register                       |
| EMAC_L3L4_CTL        | Layer3 and Layer4 Control Register              |
| EMAC_L3_ADDR0        | Layer 3 Address0 Register                       |
| EMAC_L3_ADDR1        | Layer 3 Address1 Register                       |
| EMAC_L3_ADDR2        | Layer 3 Address2 Register                       |
| EMAC_L3_ADDR3        | Layer 3 Address3 Register                       |
| EMAC_L4_ADDR         | Layer 4 Address Register                        |
| EMAC_LPI_CTLSTAT     | Low Power Idle Control and Status Register      |
| EMAC_LPI_TMRSCTL     | Low Power Idle Timeout Register                 |
| EMAC_MACCFG          | MAC Configuration Register                      |
| EMAC_MACFRMFILT      | MAC Rx Frame Filter Register                    |
| EMAC_MAC_AVCTL       | AV MAC Control Register                         |
| EMAC_MMC_CTL         | MMCControl Register                             |
| EMAC_MMC_RXIMSK      | MMCRxInterrupt Mask Register                    |
| EMAC_MMC_RXINT       | MMCRxInterrupt Register                         |
| EMAC_MMC_TXIMSK      | MMCTXInterrupt Mask Register                    |
| EMAC_MMC_TXINT       | MMCTxInterrupt Register                         |
| EMAC_RX1024TOMAX_GB  | Rx 1024- to Max-Byte Frames (Good/Bad) Register |
| EMAC_RX128TO255_GB   | Rx 128- to 255-Byte Frames (Good/Bad) Register  |

Table 29-1: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                    | Description                                     |
|-------------------------|-------------------------------------------------|
| EMAC_RX256TO511_GB      | Rx 256- to 511-Byte Frames (Good/Bad) Register  |
| EMAC_RX512TO1023_GB     | Rx 512- to 1023-Byte Frames (Good/Bad) Register |
| EMAC_RX64_GB            | Rx 64-Byte Frames (Good/Bad) Register           |
| EMAC_RX65TO127_GB       | Rx 65- to 127-Byte Frames (Good/Bad) Register   |
| EMAC_RXALIGN_ERR        | Rx alignment Error Register                     |
| EMAC_RXBCASTFRM_G       | Rx Broadcast Frames (Good) Register             |
| EMAC_RXCRC_ERR          | Rx CRC Error Register                           |
| EMAC_RXCTLFRM_G         | Rx Good Control Frames Register                 |
| EMAC_RXFIFO_OVF         | Rx FIFO Overflow Register                       |
| EMAC_RXFRMCNT_GB        | Rx Frame Count (Good/Bad) Register              |
| EMAC_RXICMP_ERR_FRM     | Rx ICMP Error Frames Register                   |
| EMAC_RXICMP_ERR_OCT     | Rx ICMP Error Octets Register                   |
| EMAC_RXICMP_GD_FRM      | Rx ICMP Good Frames Register                    |
| EMAC_RXICMP_GD_OCT      | Rx ICMP Good Octets Register                    |
| EMAC_RXIPV4_FRAG_FRM    | Rx IPv4 Datagrams Fragmented Frames Register    |
| EMAC_RXIPV4_FRAG_OCT    | Rx IPv4 Datagrams Fragmented Octets Register    |
| EMAC_RXIPV4_GD_FRM      | Rx IPv4 Datagrams (Good) Register               |
| EMAC_RXIPV4_GD_OCT      | Rx IPv4 Datagrams Good Octets Register          |
| EMAC_RXIPV4_HDR_ERR_FRM | Rx IPv4 Datagrams Header Errors Register        |
| EMAC_RXIPV4_HDR_ERR_OCT | Rx IPv4 Datagrams Header Errors Register        |
| EMAC_RXIPV4_NOPAY_FRM   | Rx IPv4 Datagrams No Payload Frame Register     |
| EMAC_RXIPV4_NOPAY_OCT   | Rx IPv4 Datagrams No Payload Octets Register    |
| EMAC_RXIPV4_UDSBL_FRM   | Rx IPv4 UDP Disabled Frames Register            |
| EMAC_RXIPV4_UDSBL_OCT   | Rx IPv4 UDP Disabled Octets Register            |
| EMAC_RXIPV6_GD_FRM      | Rx IPv6 Datagrams Good Frames Register          |
| EMAC_RXIPV6_GD_OCT      | Rx IPv6 Good Octets Register                    |
| EMAC_RXIPV6_HDR_ERR_FRM | Rx IPv6 Datagrams Header Error Frames Register  |
| EMAC_RXIPV6_HDR_ERR_OCT | Rx IPv6 Header Errors Register                  |
| EMAC_RXIPV6_NOPAY_FRM   | Rx IPv6 Datagrams No Payload Frames Register    |
| EMAC_RXIPV6_NOPAY_OCT   | Rx IPv6 No Payload Octets Register              |
| EMAC_RXJAB_ERR          | Rx Jab Error Register                           |

Table 29-1: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                 | Description                                   |
|----------------------|-----------------------------------------------|
| EMAC_RXLEN_ERR       | Rx Length Error Register                      |
| EMAC_RXMCASTFRM_G    | Rx Multicast Frames (Good) Register           |
| EMAC_RXOCTCNT_G      | Rx Octet Count (Good) Register                |
| EMAC_RXOCTCNT_GB     | Rx Octet Count (Good/Bad) Register            |
| EMAC_RXOORTYPE       | Rx Out Of Range Type Register                 |
| EMAC_RXOSIZE_G       | Rx Oversize (Good) Register                   |
| EMAC_RXPAUSEFRM      | Rx Pause Frames Register                      |
| EMAC_RXRCV_ERR       | Rx Error Frames Received Register             |
| EMAC_RXRUNT_ERR      | Rx Runt Error Register                        |
| EMAC_RXTCP_ERR_FRM   | Rx TCP Error Frames Register                  |
| EMAC_RXTCP_ERR_OCT   | Rx TCP Error Octets Register                  |
| EMAC_RXTCP_GD_FRM    | Rx TCP Good Frames Register                   |
| EMAC_RXTCP_GD_OCT    | Rx TCP Good Octets Register                   |
| EMAC_RXUCASTFRM_G    | Rx Unicast Frames (Good) Register             |
| EMAC_RXUDP_ERR_FRM   | Rx UDP Error Frames Register                  |
| EMAC_RXUDP_ERR_OCT   | Rx UDP Error Octets Register                  |
| EMAC_RXUDP_GD_FRM    | Rx UDP Good Frames Register                   |
| EMAC_RXUDP_GD_OCT    | Rx UDP Good Octets Register                   |
| EMAC_RXUSIZE_G       | Rx Undersize (Good) Register                  |
| EMAC_RXVLANFRM_GB    | Rx VLAN Frames (Good/Bad) Register            |
| EMAC_RXWDOG_ERR      | Rx Watch Dog Error Register                   |
| EMAC_SMI_ADDR        | SMI Address Register                          |
| EMAC_SMI_DATA        | SMI Data Register                             |
| EMAC_TM_ADDEND       | Time Stamp Addend Register                    |
| EMAC_TM_AUXSTMP_NSEC | Time Stamp Auxiliary TS Nano Seconds Register |
| EMAC_TM_AUXSTMP_SEC  | Time Stamp Auxiliary TMSeconds Register       |
| EMAC_TM_CTL          | Time Stamp Control Register                   |
| EMAC_TM_HISEC        | Time Stamp High Second Register               |
| EMAC_TM_NSEC         | Time Stamp Nanoseconds Register               |
| EMAC_TM_NSECUPDT     | Time Stamp Nanoseconds Update Register        |
| EMAC_TM_PPS0INTVL    | Time Stamp PPS Interval Register              |

Table 29-1: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                | Description                                     |
|---------------------|-------------------------------------------------|
| EMAC_TM_PPS0NTGTM   | Time Stamp Target Time Nanoseconds Register     |
| EMAC_TM_PPS0TGTM    | Time Stamp Target Time Seconds Register         |
| EMAC_TM_PPS0WIDTH   | PPS Width Register                              |
| EMAC_TM_PPS1INTVL   | PPS 1 Interval Register                         |
| EMAC_TM_PPS1NTGTM   | PPS 1 Target Time Nanoseconds Register          |
| EMAC_TM_PPS1TGTM    | PPS 1 Target Time Seconds Register              |
| EMAC_TM_PPS1WIDTH   | PPS 1 Width Register                            |
| EMAC_TM_PPS2INTVL   | PPS 2 Interval Register                         |
| EMAC_TM_PPS2NTGTM   | PPS 2 Target Time Nanoseconds Register          |
| EMAC_TM_PPS2TGTM    | PPS 2 Target Time Seconds Register              |
| EMAC_TM_PPS2WIDTH   | PPS 2 Width Register                            |
| EMAC_TM_PPS3INTVL   | PPS 3 Interval Register                         |
| EMAC_TM_PPS3NTGTM   | PPS 3 Target Time Nanoseconds Register          |
| EMAC_TM_PPS3TGTM    | PPS 3 Target Time Seconds Register              |
| EMAC_TM_PPS3WIDTH   | PPS 3 Width Register                            |
| EMAC_TM_PPSCTL      | PPS Control Register                            |
| EMAC_TM_SEC         | Time Stamp Low Seconds Register                 |
| EMAC_TM_SECUPDT     | Time Stamp Seconds Update Register              |
| EMAC_TM_STMPSTAT    | Time Stamp Status Register                      |
| EMAC_TM_SUBSEC      | Time Stamp Sub Second Increment Register        |
| EMAC_TX1024TOMAX_GB | Tx 1024- to Max-Byte Frames (Good/Bad) Register |
| EMAC_TX128TO255_GB  | Tx 128- to 255-Byte Frames (Good/Bad) Register  |
| EMAC_TX256TO511_GB  | Tx 256- to 511-Byte Frames (Good/Bad) Register  |
| EMAC_TX512TO1023_GB | Tx 512- to 1023-Byte Frames (Good/Bad) Register |
| EMAC_TX64_GB        | Tx 64-Byte Frames (Good/Bad) Register           |
| EMAC_TX65TO127_GB   | Tx 65- to 127-Byte Frames (Good/Bad) Register   |
| EMAC_TXBCASTFRM_G   | Tx Broadcast Frames (Good) Register             |
| EMAC_TXBCASTFRM_GB  | Tx Broadcast Frames (Good/Bad) Register         |
| EMAC_TXCARR_ERR     | Tx Carrier Error Register                       |
| EMAC_TXDEFERRED     | Tx Deferred Register                            |
| EMAC_TXEXCESSCOL    | Tx Excess Collision Register                    |

Table 29-1: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name               | Description                                     |
|--------------------|-------------------------------------------------|
| EMAC_TXEXCESSDEF   | Tx Excess Deferral Register                     |
| EMAC_TXFRMCNT_G    | Tx Frame Count (Good) Register                  |
| EMAC_TXFRMCNT_GB   | Tx Frame Count (Good/Bad) Register              |
| EMAC_TXLATECOL     | Tx Late Collision Register                      |
| EMAC_TXMCASTFRM_G  | Tx Multicast Frames (Good) Register             |
| EMAC_TXMCASTFRM_GB | Tx Multicast Frames (Good/Bad) Register         |
| EMAC_TXMULTCOL_G   | Tx Multiple Collision (Good) Register           |
| EMAC_TXOCTCNT_G    | Tx Octet Count (Good) Register                  |
| EMAC_TXOCTCNT_GB   | Tx OCT Count (Good/Bad) Register                |
| EMAC_TXOVRSIZE_G   | Number of Tx Frames (Good) greater than maxsize |
| EMAC_TXPAUSEFRM    | Tx Pause Frame Register                         |
| EMAC_TXSNGCOL_G    | Tx Single Collision (Good) Register             |
| EMAC_TXUCASTFRM_GB | Tx Unicast Frames (Good/Bad) Register           |
| EMAC_TXUNDR_ERR    | Tx Underflow Error Register                     |
| EMAC_TXVLANFRM_G   | Tx VLAN Frames (Good) Register                  |
| EMAC_VLANTAG       | VLAN Tag Register                               |
| EMAC_VLAN_HSHTBL   | VLAN Hash Table Register                        |
| EMAC_VLAN_INCL     | VLAN Tag Inclusion or Replacement Register      |
| EMAC_WDOG_TIMOUT   | Watchdog Timeout Register                       |

## ADSP-2159x\_SC592\_SC594 EMAC Interrupt List

Table 29-2: ADSP-2159x\_SC592\_SC594 EMAC Interrupt List

|   Interrupt ID | Name       | Description   | Sensitivity   | DMA Channel   |
|----------------|------------|---------------|---------------|---------------|
|            221 | EMAC0_STAT | EMAC0 Status  | None          |               |
|            222 | EMAC0_PWR  | EMAC0 Power   | None          |               |
|            223 | EMAC0_DMA0 | EMAC0 DMA0    | None          |               |
|            224 | EMAC0_DMA1 | EMAC0 DMA1    | None          |               |
|            225 | EMAC0_DMA2 | EMAC0 DMA2    | None          |               |
|            226 | EMAC0_MAC  | EMAC0 MAC     | None          |               |
|            227 | EMAC1_STAT | EMAC1 Status  | None          |               |

Table 29-2: ADSP-2159x\_SC592\_SC594 EMAC Interrupt List (Continued)

|   Interrupt ID | Name       | Description   | Sensitivity   | DMA Channel   |
|----------------|------------|---------------|---------------|---------------|
|            228 | EMAC1_PWR  | EMAC1 Power   | None          |               |
|            229 | EMAC1_DMA0 | EMAC1DMA      | None          |               |
|            230 | EMAC1_MAC  | EMAC1 MAC     | None          |               |

## ADSP-2159x\_SC592\_SC594 EMAC Trigger List

Table 29-3: ADSP-2159x\_SC592\_SC594 EMAC Trigger List Generators

|   Trigger ID | Name       | Description   | Sensitivity   |
|--------------|------------|---------------|---------------|
|           30 | EMAC0_STAT | EMAC0 Status  | None          |
|           31 | EMAC1_STAT | EMAC1 Status  | None          |

Table 29-4: ADSP-2159x\_SC592\_SC594 EMAC Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
|              |        | None          |               |

## EMAC Definitions

The following definitions are helpful for using the EMAC.

## EMAC SCB

System Crossbar Interface of EMAC

## EMAC DMA

DMA Controller of EMAC

## EMAC MFL

MAC FIFO Layer inside EMAC

## EMAC CORE

CORE layer inside EMAC which performs the actual Ethernet operations, including interface with PHY through the reduced media interface(s).

## MMC

MAC Management Counter

## SMI

Station Management Interface that controls PHY through MDIO and MDC signals.

## RMII

Reduced Media Independent Interface

## MAC

Media Access Control

## PTP

Precision Time Protocol

## EMAC Block Diagram and Interfaces

The EMAC Simplified Block Diagram illustrates the overall functional architecture of the Ethernet MAC peripheral. The EMAC module is comprised of four major layers: EMAC SCB, EMAC DMA, EMAC MFL, and EMAC CORE. Each of these layers (subblocks) is explained in depth in their respective sections in this chapter.

Figure 29-1: EMAC Simplified Block Diagram

<!-- image -->

A more comprehensive block diagram is shown in the EMAC Complete Block Diagram . It includes most of the important blocks inside the EMAC. The EMAC is connected to processor memory and the system crossbar through the System Crossbar Bus Interface (SCB) and System Peripheral Bus Interface (SPB). These connections are which are part of the SCB layer. The SPB interface is connected to all modules that require MMR programming.

The DMA controller performs application data transfer frame by frame, through well-defined descriptor structures. A FIFO layer acts as a buffer between the DMA controller and EMAC CORE.

The EMAC CORE is the most important block because it contains subblocks to support IEEE802.3 based communication with external network interfaces of 10/100/1000 Mbps speeds. It includes the PTP subblock, which assists applications requiring time synchronization; the AV Feature subblock, which enables transmission of time-sensitive traffic over bridged LANs; and a MMC subblock, which generates packet transfer statistics.

Figure 29-2: EMAC Complete Block Diagram

<!-- image -->

## EMAC CORE Subblocks

The Core Transmit Engine Subblocks table summarizes the core transmit engine subblocks and their functions. Refer to the EMAC CORE section for further explanation of each of these subblocks.

Table 29-5: CORE Transmit Engine Subblocks

| CORE Transmit Engine Sub Block   | Function                                                                                                                                                                                                                                        |
|----------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Transmit Bus Interface           | Interface to the FIFO.                                                                                                                                                                                                                          |
| Transmit Frame Controller        | Appends Zero-PAD data, if required, for short frames. Appends CRC for frame checksum from the CRC generator.                                                                                                                                    |
| Transmit Protocol Controller     | Generates preamble and SFD, as per 802.3 protocol. Generates jam pattern in half-duplex mode, for collisions. Jabber timeout, for excessively large frames. Flow control for half-duplex mode (back pressure). Generates transmit frame status. |
| Transmit Scheduler               | Maintains the inter-frame gap between two transmitted frames. Follows the truncated binary exponential back-off algorithm for half-duplex mode.                                                                                                 |

Table 29-5: CORE Transmit Engine Subblocks (Continued)

| CORE Transmit Engine Sub Block   | Function                                                                                                 |
|----------------------------------|----------------------------------------------------------------------------------------------------------|
| Transmit CRC Generator           | Generate CRC for the frame checksum field of the Ethernet frame.                                         |
| Transmit Flow Control            | Receives the pause frame, appends the calculated CRC, and sends the frame to the protocol engine module. |
| Transmit Checksum Offload Engine | Supports checksum calculation and insertion in the transmit path, for IPV4/TCP/UDP/ICMP packets.         |

The Core Receive Engine Subblocks table summarizes the core receive engine subblocks and their function. Refer to the EMAC CORE section for more information on each of these subblocks.

Table 29-6: Core Receive Engine Subblocks

| CORE Receive Engine Subblock       | Functionality Overview                                                                                                                                                                                                                                      |
|------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Receive Protocol Engine            | Strips the incoming preamble and SFD. Checks for correct length or type field. Performs internal loopback, if necessary. Generates receive status. Supports watchdog of received frames. Supports jumbo frames.                                             |
| Receive CRC Module                 | Checks for CRC error, by comparing with FCS.                                                                                                                                                                                                                |
| Receive Frame Controller Module    | Packs incoming 8-bit input stream to 32-bit data internally. Performs frame filtering, for uni-cast, multi-cast, and broadcast frames. Attaches the calculated IP checksum input from checksum offload engine. Updates the receive status to bus interface. |
| Receive Flow Control Module        | Detects the receiving pause frame and pauses the frame transmission for the delay specified within the received pause frame. Works in full duplex mode.                                                                                                     |
| Receive IP Checksum Offload Engine | Calculates IPv4 header checksums and verify against the received IPv4 header checksums. Identifies a TCP, UDP, or ICMP payload in the received IP data- grams.                                                                                              |
| Receive Bus Interface Unit Module  | Interface to the FIFO.                                                                                                                                                                                                                                      |
| Address Filtering Module           | Filters destination and source address based on uni-cast, multi-cast, and broadcast frames. Provides CRC hash filtering.                                                                                                                                    |

## EMAC PHY Interface

The EMAC can interface to the PHY through the RMII interface standard. The RMII Pins table shows the RMII pins available in the EMAC, in terms of their generic names. Refer to the data sheet for exact pin names.

Table 29-7: RMII Pins

|   Signal No. | Generic Signal Name (IEEE Standards)   | RMII Pin Functionality                                         |
|--------------|----------------------------------------|----------------------------------------------------------------|
|            1 | TXD0                                   | RMII transmit data pin D0 (di-bit lower)                       |
|            2 | TXD1                                   | RMII transmit data pin D1 (di-bit higher)                      |
|            3 | RXD0                                   | RMII receive data pin D0 (di-bit lower)                        |
|            4 | RXD1                                   | RMII receive data pin D1 (di-bit higher)                       |
|            5 | RMII CLK                               | RMII common clock (for TX and RX), also called reference clock |
|            6 | TXEN                                   | RMII transmit enable pin (TX valid)                            |
|            7 | CRS                                    | RMII carrier sense / receive data valid                        |
|            8 | MDC                                    | Serial management clock driven by EMAC                         |
|            9 | MDIO                                   | Serial management bidirectional data                           |

Figure 29-3: RMII Di-bit Data Transfer

<!-- image -->

## Extended EMAC PHY Interface

The EMAC can interface to the PHY through the RGMII interface standard. The RGMII Pins table shows the RGMII pins available in the EMAC, in terms of their generic names. Refer to the data sheet for exact pin names.

Table 29-8: RGMII Pins

|   Sl. No. | Generic Signal Name (IEEE Standards)   | RGMII Pin Functionality                       |
|-----------|----------------------------------------|-----------------------------------------------|
|         1 | TXD3-0                                 | RGMII transmit data pins D3-0                 |
|         2 | RXD3-0                                 | RGMII receive data pins D3-0                  |
|         3 | TXCLK                                  | RGMII transmit reference clock driven by EMAC |
|         4 | RXCLK                                  | RGMII receive reference clock driven by PHY   |
|         5 | TXCTL                                  | RGMII transmit enable pin (TX valid)          |
|         6 | RXCTL                                  | RGMII receive data valid                      |

Table 29-8: RGMII Pins (Continued)

|   Sl. No. | Generic Signal Name (IEEE Standards)   | RGMII Pin Functionality                |
|-----------|----------------------------------------|----------------------------------------|
|         7 | MDC                                    | Serial management clock driven by EMAC |
|         8 | MDIO                                   | Serial management bidirectional data   |

Figure 29-4: RGMII Data Bit Transfer

<!-- image -->

## EMAC PHY Interface

The EMAC can interface to the PHY through the MII interface standard. The MII Pins table shows the MII pins available in the EMAC, in terms of their generic names. Refer to the data sheet for exact pin names.

Table 29-9: MII Pins

|   Sl. No. | Generic Signal Name (IEEE Standards)   | MII Pin Functionality      |
|-----------|----------------------------------------|----------------------------|
|         1 | TXCLK                                  | MII transmit clock         |
|         2 | TXD0-3                                 | MII transmit data pins 0-3 |
|         3 | TXEN                                   | MII transmit enable        |
|         4 | RXCLK                                  | MII receive clock          |
|         5 | RXD0-3                                 | MII receive data pins 0-3  |
|         6 | RXDV                                   | MII receive data valid     |
|         7 | RXER                                   | MII receive error          |
|         8 | CRS                                    | MII carrier sense          |
|         9 | COL                                    | MII collision detect       |

Figure 29-5: MII Data Bit Transfer

<!-- image -->

## PHY Interface Selection

The EMAC0 supports both 10/100 Mbps data-transfer rates with external PHY interfaced through RMII and MII and 10/100/1000 Mbps data-transfer rates with external PHY interfaced through RGMII.

Select the external PHY interface for EMAC0 using the PADS\_PCFG0 register, as shown in the following table.

| PADS_PCFG0.EMACPHYISEL   | 0 = MII Interface 1 = RGMII Interface 2 = RMII Interface                                                                                              |
|--------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| PADS_PCFG0.EMACRESET     | 0 = reset is asserted 1 = reset is deasserted To select PHY interface, set PADS_PCFG0.EMACPHYISEL bit as required and then set PADS_PCFG0.EMACRESET . |

## RGMII Board Design Recommendations

Use the following guidelines during when performing board design when using the RGMII interface.

## MAC to PHY (Transmit)

The Ethernet MAC transmits data to the Ethernet PHY. The Ethernet MAC sends data with tskewT (the timing of TXC at the MAC) that meets the RGMII specification (tskewT = -500 ps to +500 ps skew window for transmitter to drive data). The RGMII specification requires that at the PHY end, tskewR is sampled at 1.0 to 2.6 ns. According to the RGMII standard, clocks must be routed such that an additional trace delay of greater than 1.5 ns and less than 2 ns is added to the associated clock signal.

To meet this standard without adding trace delays, most of the PHYs in the industry already include delay logic that can compensate for this on-board delay. These PHY types can manage a tskewR of ±500 ps (the skew for TXC data sampling seen inside the PHY).

The PHY or the on-board delay must delay the clock signal by 1.5 to 2.0 ns so that tskewR is sampled at 1.0 to 2.6 ns. The MAC to PHY Delay Diagram (Transmitting Data) figure shows where the delays must occur.

Figure 29-6: MAC to PHY Delay Diagram (Transmitting Data)

<!-- image -->

## PHY to MAC (Receive)

The Ethernet MAC receives data from the Ethernet PHY. Just as in the transmit case, a trace delay of greater than 1.5 ns and less than 2 ns must be added to the associated clock signal, as required by the RGMII specification. Also similar to the transmit case, most of the PHYs in the industry already include delay logic that can compensate for the RXC clock as well.

As shown in the MAC to PHY Delay Diagram (Receiving Data) figure, a board trace or the PHY can be used to generate the required 1.5-2.0 ns delay to RXC and the 1.0-2.6 ns tskewR.

Figure 29-7: MAC to PHY Delay Diagram (Receiving Data)

<!-- image -->

There following are two options for board design.

- The on-board delay must delay the clock signal by 1.5 to 2.0 ns. In this case, no additional delay must be introduced by the PHY.
- Most of the PHYs in the industry already include a mode that can introduce a delay logic. When this PHY mode is used, then the trace lengths on the board must be matched exactly.

The second option is recommended since it is easier to implement the delay by using the mode in the PHY rather than during the board design. This is the approach followed in the ADI EZ-Kits.

NOTE: Refer to the product specific data sheet for exact processor timing.

For the exact requirements as recommended by the RGMII protocol, see the RGMII specification.

## Clock Sources

The Ethernet MAC is clocked internally from CLKO7. Check the processor data sheet for the valid frequency range of the appropriate CLKO7 signal for Ethernet operation.

Source a 50-MHz clock externally to operate the EMAC in RMII mode. This clock is the same for both transmit and receive. The MDC station management clock is derived from the CLKO7 and driven from the MAC to the PHY, when accessing any PHY registers. .

Figure 29-8: EMAC Clock Sources for RMII PHY interface

<!-- image -->

## EMAC0 Clock Sources

EMAC0 supports RGMII, RMII and MII interfaces. The external PHY sources a 2.5 MHz or 25 MHz clock (for 10/100 or gigabit Ethernet respectively) to operate the EMAC RXCLK in RGMII mode. The RGMII TXCLK is driven from CLK07 of the CDU (Clock Distribution Unit) and needs to be configured to 125 MHz regardless of the EMAC0 speeds (10/100/1000 Mbit/s). The EMAC\_MACCFG.PS and EMAC\_MACCFG.FES bits are used to divide the clocks down.

The following tables show the clock sources for the RGMII, MII and RMII interfaces.

Figure 29-9: EMAC Clock Sources - RGMII PHY Interface

<!-- image -->

Figure 29-10: EMAC Clock Sources - MII PHY Interface

<!-- image -->

Figure 29-11: EMAC Clock Sources - RMII PHY Interface

<!-- image -->

## EMAC Architectural Concepts

This section explains different architectural concepts relevant to EMAC peripheral, such as EMAC SCB, EMAC DMA, EMAC MFL, EMAC CORE, and others.

## EMAC Feature Summary

The EMAC Feature Summary table provides a summary of the features that are available on EMAC0 .

Table 29-10: EMAC Feature Summary

| Feature                         | Value (EMAC0)    | Value (EMAC1)   |
|---------------------------------|------------------|-----------------|
| Speed of Operation              | 10/100/1000 Mbps | 10/100 Mbps     |
| EMAC_VER.UVER hardcoded value   | 0x10             | 0x11            |
| PHY Interface                   | RGMII or RMII    | RMII            |
| Receive FIFO Size (in Bytes)    | 2048             | 512             |
| Transmit FIFO Size (in Bytes)   | 2048             | 1024            |
| Energy Efficient Ethernet (EEE) | Yes              | No              |
| PTP (IEEE 1588)                 | Yes              | No              |

Table 29-10: EMAC Feature Summary (Continued)

| Feature           | Value (EMAC0)   | Value (EMAC1)   |
|-------------------|-----------------|-----------------|
| AV Feature        | Yes             | No              |
| No of TX Channels | 3               | 1               |
| No of RX Channels | 3               | 1               |

## EMAC System Crossbar Interface (EMAC SCB)

The EMAC SCB bus interface provides the bus connectivity to support highly effective throughput of data traffic. System bus use is maximized by allowing simultaneous read and write transfers initiated from different DMA channels. The EMAC controller connects directly to the SCB0 crossbar. The following interfaces are available with the design.

- A 32-bit SCB controller interface for reading and writing to and from the application memory.
- A 32-bit SPB target interface for register programming.

Refer to the 'System Crossbars (SCB)' chapter for more information on how the crossbar operates. This chapter details only the EMAC-specific information.

Table 29-11: EMAC-SCB Interface Data Transfer Specifications with Crossbar

| Specification Term   | Comments                             |
|----------------------|--------------------------------------|
| 1 beat in SCB        | SINGLE burst                         |
| BLEN4 bursts         | 4 beats in SCB                       |
| BLEN8 bursts         | 8 beats in SCB                       |
| BLEN16 bursts        | 16 beats in SCB                      |
| Bus size             | 32-bit fixed bus size; equals 1 beat |
| INCR bursts          | Incrementing Bursts                  |
| INCR ALIGNED bursts  | Incrementing aligned bursts          |
| UNDEF bursts         | Undefined burst length               |
| PBL                  | Programmable Burst Length forDMA     |

The EMAC DMA Read/Write channels with System Crossbar figure shows DMA write channel and read channel datapaths and their connection to the system crossbar.

Figure 29-12: EMAC DMA Read/Write channels with System Crossbar

<!-- image -->

NOTE: Transmit descriptor read and receive descriptor write-back (status update) operations can occur simultaneously. However, transmit descriptor read and write-back operations cannot occur simultaneously. T ransmit DMA (or receive DMA) does not initiate the next transfer unless the previous one is complete.

## Priority of SCB Requests

The descriptor transfers have higher priority than the data transfers. For example, if there are two bus requests, such as a receive descriptor read and a transmit data read, the receive descriptor read has a higher priority. The next receive data write (subsequent to the receive descriptor read) does not depend on the completion of the transmit dataread transfer.

If there are requests for descriptor reads from both DMA channels, they are serviced based on a first-come firstserve. Receive DMA has higher priority if the descriptor-read requests are generated from both the DMA channels in the same clock cycle. Similarly, in the write channel, descriptor writes from DMA have higher priority than the data-write transfers for the receive DMA.

## SCB Interface Programming Options

The SCB bus interface supports the following programmable options for the EMAC module. These options are available using the EMAC\_DMA0\_BMMODE register with the EMAC\_DMA0\_BUSMODE register. These programming options apply to DMA1 and DMA2 as well.

- Outstanding transactions. The EMAC-SCB supports up to four outstanding read/write requests on the SCB
- bus. Software can control these requests by programming the EMAC\_DMA0\_BMMODE.WROSRLMT and EMAC\_DMA0\_BMMODE.RDOSRLMT bits. Maximum outstanding requests = EMAC\_DMA0\_BMMODE.WROSRLMT + 1 (or) EMAC\_DMA0\_BMMODE.RDOSRLMT + 1.
- Allowed burst sizes. The allowed burst sizes are 4 ( EMAC\_DMA0\_BMMODE.BLEN4 ), 8 ( EMAC\_DMA0\_BMMODE.BLEN8 ), 16 ( EMAC\_DMA0\_BMMODE.BLEN16 ) and the SINGLE burst. The EMAC-SCB uses only those burst sizes configured by the program (through the EMAC\_DMA0\_BMMODE register) for data transfer through the SCB bus. However, SINGLE burst is available by default, when the EMAC\_DMA0\_BMMODE.UNDEF bit is cleared. Data transfers are restricted to the maximum burst size from this list of programmed burst sizes.

- Burst splitting and burst selection. The EMAC-SCB splits the DMA requests into multiple bursts on the SCB system bus. Splitting is based on DMA count and software controllable burst enable bits (shown in the allowed burst sizes) as well as burst types (INCR and INCR\_ALIGNED). Burst types are also controllable through the software. SINGLE burst is enabled when the EMAC\_DMA0\_BMMODE.UNDEF bit is not set. Burst length select priority is in the sequence: UNDEF , 16, 8, and 4.

## · INCR burst type

- If the EMAC\_DMA0\_BMMODE.UNDEF bit is set, the EMAC-SCB always chooses the maximum allowed burst length based on the EMAC\_DMA0\_BMMODE.BLEN16 , EMAC\_DMA0\_BMMODE.BLEN8 , EMAC\_DMA0\_BMMODE.BLEN4 bits. When the DMA requests are not multiples of the maximum allowed burst length, the SCB can choose a burst-length of any value less than the maximum enabled. (All lesser burst-length enables are redundant). For example, when length bits are enabled and the DMA requests a burst of 42 beats, the SCB splits it into three bursts of 16, 16 and 10 beats respectively.
- If EMAC\_DMA0\_BMMODE.UNDEF is not enabled, then the burst length is based on the priority of the enabled bits in the following order EMAC\_DMA0\_BMMODE.BLEN16 , EMAC\_DMA0\_BMMODE.BLEN8 , EMAC\_DMA0\_BMMODE.BLEN4 . When the DMA requests a burst transfer, the SCB interface splits the requested bursts into multiple transfers using only the enabled burst lengths. This splitting can occur when the requested burst is not a multiple of the maximum enabled burst. If it cannot choose any of the enabled burst lengths, then it selects the burst length as 1.

For example, the DMA requests a burst transfer of 42 beats, the SCB interface splits it into multiple bursts of size 16, 16, 8, 1 and 1 beats respectively. (In this case, the allowed burst sizes are enabled and the sequence is in decreasing burst sizes).

- INCR\_ALIGNED burst type. When the address-aligned burst-type is enabled ( EMAC\_DMA0\_BMMODE.AAL ), the SCB interface splits the DMA requested bursts. The "INCR Burst Type" section explains burst splitting conditions further. Each burst-size aligns to the least significant bits of the start address. The SCB interface initially generates smaller bursts so that the remaining transfers move with the maximum (enabled) fixed burst lengths.

For example, in the same setting as explained earlier for EMAC\_DMA0\_BMMODE.UNDEF set, the DMA requests a burst size of 42 beats at the start address of 0x000003A4.( EMAC\_DMA0\_BMMODE.BLEN16 , EMAC\_DMA0\_BMMODE.BLEN8 , and EMAC\_DMA0\_BMMODE.BLEN4 are enabled). The SCB starts the first transfer with size 3 such that the address of the next burst is aligned (0x000003B0) for a burst of 16. Therefore, the sequence of bursts is 3, 16, 16, and 7, respectively.

When EMAC\_DMA0\_BMMODE.UNDEF is not set, then (having a start address of 0x000003A4 with 42 beats), the sequence of burst transfers is 1, 1, 1, 16, 16, 4, and 3 respectively. The sequence of smaller bursts at the beginning is used to align the address to the next higher enabled burst-lengths programmed in the register.

- Burst operations for DMA transactions. The EMAC\_DMA0\_BUSMODE.PBL (programmable burst length) field indicates the maximum number of beats to transfer in one DMA transaction. This value is also the maximum used in a single block read/write. It is shown in the following table.

- For example, if EMAC\_DMA0\_BUSMODE.PBL =32 and if EMAC\_DMA0\_BMMODE.BLEN16 is enabled, the DMA automatically splits 32 bursts in to 2 x 16 bursts. If EMAC\_DMA0\_BUSMODE.PBL =8, and if EMAC\_DMA0\_BMMODE.BLEN16 and EMAC\_DMA0\_BMMODE.BLEN8 are enabled, the maximum burst is limited to EMAC\_DMA0\_BMMODE.BLEN8 . If the EMAC\_DMA0\_BUSMODE.PBL8 bit is set, the programmed EMAC\_DMA0\_BUSMODE.PBL value is multiplied by 8 times internally. However, the result cannot be more than the maximum limits specified.
- Set the EMAC\_DMA0\_BUSMODE.USP bit to make the receive DMA burst length configuration independent of the transmit DMA configuration. When this bit is set, the EMAC uses the EMAC\_DMA0\_BUSMODE.RPBL bits to define the burst length of receive DMA. If the EMAC\_DMA0\_BUSMODE.USP bit is not set, the EMAC\_DMA0\_BUSMODE.RPBL bits are used for both transmit and receive. Programs must ensure that the PBL maximum limit is not violated.
- The receive and transmit descriptors are always accessed in the maximum burst-size for the 16-bytes to be read (PBL-max limit is (TX or RX FIFO size/2)/4 words. (PBL maximum for transmit and receive limits burst-size).

Table 29-12: DMA PBL Max Limits

| Burst Limit Max Term         | Definition                  |
|------------------------------|-----------------------------|
| PBL-maximum limit            | (FIFO size/2)/4 words       |
| PBL-maximum limit (transmit) | 2048 bytes/2 /4 = 256 words |
| PBL-maximum limit (receive)  | 2048 bytes/2 /4 = 256 words |

## DMA Bursts Using the SCB Interface

The transmit DMA initiates a data transfer when sufficient space to accommodate the configured burst is available in the transmit FIFO. Or, the transmit DMA initiates a data transfer when the number of bytes until the end of frame is less than the configured burst-length. The DMA indicates the start address and the number of transfers required to the SCB controller interface. When the SCB interface is configured for fixed-length burst, then it transfers data using the best combination of INCR4/8/16 and 1 beat transaction.

The receive DMA initiates a data transfer when sufficient data to accommodate the configured burst is available in the MTL receive FIFO. Or, the receive DMA initiates a data transfer when the end of frame is detected in the receive FIFO. For example, when the amount is less than the configured burst-length. The DMA indicates the start address and the number of transfers required to the SCB controller interface. When the SCB interface is configured for fixed-length burst, then it transfers data using the best combination of INCR 4, 8, 16 or 1 beat transaction. If the end-of frame is reached before the fixed-burst ends on the SCB interface, then dummy transfers are performed to complete the fixed-burst. Otherwise (if EMAC\_DMA0\_BUSMODE.FB is reset), it transfers data using INCR (undefined length) transactions.

When the SCB interface is configured for address-aligned beats, both DMA engines ensure that the first burst transfer is less than or equal to the configured PBL size. (The address-aligned beats configuration uses the EMAC\_DMA0\_BUSMODE.AAL bit). Therefore, all subsequent beats start at an address that is aligned to the configured PBL.

## SCB Bus Transaction Status

The SCB uses the EMAC\_DMA0\_BMSTAT.BUSRD and EMAC\_DMA0\_BMSTAT.BUSWR bits to indicate whether the channel is active or not.

## Fatal Bus Error

The EMAC SCB asserts the error interrupt ( EMAC\_DMA0\_STAT.FBI ) when the corresponding fatal bus error interrupt is enabled in the DMA interrupt enable register. The application must reset the core to restart the DMA.

## DMA Controller (EMAC DMA)

The EMAC has a built-in DMA controller that performs reads and writes of application data and descriptors through the SCB controller interface.

The DMA controller has independent transmit and receive engines, and a CSR (control and status register) space. The transmit engine transfers data from system memory to a FIFO, while the receive engine transfers data from the FIFO to the system memory. The controller uses a descriptor chain-based transfer mechanism to move data efficiently from source to destination with minimal processor core intervention. The DMA is specially designed for packet-oriented data transfers such as Ethernet frames. The controller can be programmed to interrupt the application for situations such as frame transmit and receive transfer completion, and other normal or abnormal conditions.

The DMA and the application device driver communicate through two internal data structures:

1. DMA control and status registers (CSR).
2. Data buffers and descriptor lists. Descriptor lists operate in ring mode and chain mode, as shown in the EMAC DMA Descriptor Models figure.

Figure 29-13: EMAC DMA Descriptor Models

<!-- image -->

Descriptors that reside in the application memory act as pointers to receive and transmit buffers. Descriptors have the following extra attributes.

- There are two descriptor lists, one for receive, and one for transmit. The base address of each list is written into the address registers of the receive and transmit descriptor lists respectively.
- A descriptor list is forward linked (either implicitly or explicitly). The last descriptor can point back to the first entry to create a ring structure.
- Explicit chaining of descriptors is accomplished by setting the second address chained in both receive and transmit descriptors.
- The descriptor lists reside in the address space of the application memory.
- Each descriptor can point to a maximum of two buffers. This attribute enables two buffers, physically addressed, rather than contiguous buffers in memory.

A data buffer resides in the physical memory space of the application. It consists of an entire frame or part of a frame, but cannot exceed a single frame. Buffers can contain only data. The descriptor maintains buffer status. Data chaining refers to frames that span multiple data buffers. However, a single descriptor cannot span multiple frames. The DMA skips to the next frame buffer when the end-of-frame is detected. Data chaining is enabled or disabled.

NOTE: It is possible to define a skip length (in terms of N × 32-bit words) between two subsequent descriptors, when using ring mode. Program the EMAC\_DMA0\_BUSMODE.DSL / EMAC\_DMA1\_BUSMODE.DSL / EMAC\_DMA2\_BUSMODE.DSL field to enable this attribute. With this option available, programs are not always restricted to a contiguous memory location in ring mode.

## DMA Related Registers

The Summary of DMA Related Registers table provides a summary of DMA registers relative to their function. Refer to the 'Register Descriptions' sections for complete bit descriptions of each of these registers.

Table 29-13: Summary of DMA Related Registers

| Register Name                   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|---------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Bus Mode *1                     | Establishes the bus operating modes for the DMAbased on the SCB controller interface.                                                                                                                                                                                                                                                                                                                                                                         |
| Transmit Poll Demand            | Enables the transmit DMAto check whether the DMAowns the current descriptor. The transmit poll demand command wakes up the TxDMA when it is in suspend mode. The TxDMA can go into suspend mode because of an underflow error in a transmitted frame or because of the unavailability of descriptors owned by transmit DMA. Issue this com- mand anytime and the TxDMA resets this command once it starts refetching the current descriptor from host memory. |
| Receive Poll Demand             | Enables the receive DMAto check for new descriptors. This command wakes up the RxDMA from the SUSPEND state. The RxDMA can go into SUSPEND state only be- cause of the unavailability of descriptors owned by it.                                                                                                                                                                                                                                             |
| Receive Descriptor List Address | Points to the start of the receive descriptor list. The descriptor lists reside in the applica- tion memory space and must be word-aligned (32- bit data bus). The DMAinternally converts the descriptor list to a bus width aligned address by making the corresponding LSBs low.                                                                                                                                                                            |

Table 29-13: Summary of DMA Related Registers (Continued)

| Register Name                            | Description                                                                                                                                                                                                                                                           |
|------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Transmit Descriptor List Address         | Points to the start of the transmit descriptor list. The descriptor lists reside in the applica- tion memory space and must be word-aligned (for 32-bit data bus). The DMAinternally converts it to bus width aligned address by making the corresponding LSB to low. |
| DMAStatus                                | Contains all the status bits that the DMAreports to the application. The software driver reads this register during an interrupt service routine or during polling. Most of the fields in this register interrupt the host.                                           |
| Operation Mode                           | Establishes the transmit and receive operating modes and commands. The operation mode register is the last control register written as part of DMAinitialization.                                                                                                     |
| Interrupt Enable                         | Enables the interrupts reported by DMAstatus register. After a hardware or software re- set, all interrupts are disabled.                                                                                                                                             |
| Missed Frame and Buffer Overflow Counter | The DMAmaintains two counters to track the number of missed frames during recep- tion. This register reports the current value of the counter, which is used for diagnostic purposes.                                                                                 |
| Receive Interrupt Watchdog Timer         | When written with non-zero value, enables the watchdog timer for receive interrupt (RI) in the DMAstatus register.                                                                                                                                                    |
| SCB Bus Mode                             | Controls the SCB interface controller behavior. It controls the burst splitting and the number of outstanding requests.                                                                                                                                               |
| SCB Status                               | Provides the active status of the SCB interface read and write channels.                                                                                                                                                                                              |
| Current Host Transmit Descriptor         | Points to the start address of the current transmit descriptor read by the DMA.                                                                                                                                                                                       |
| Current Host Receive Descriptor          | Points to the start address of the current receive descriptor read by the DMA.                                                                                                                                                                                        |
| Current Host Transmit Buffer Address     | Points to the current transmit buffer address the DMAis reading.                                                                                                                                                                                                      |
| Current Host Receive Buffer Address      | Points to the current receive buffer address the DMAis reading.                                                                                                                                                                                                       |

Table 29-14: DMA Registers with Consecutive Writes

| Registers with Implications for Consecutive Writes   |
|------------------------------------------------------|
| DMABus Mode                                          |

## DMA Descriptors

The DMA module in the Ethernet subsystem transfers data based on a linked list of descriptors. The descriptor addresses must be aligned to the 32-bit bus width. The descriptors can be either 4 x 32-bit words (16 bytes) or 8 x 32-bit words (32 bytes). Configure the controller for the appropriate word length using the DMA bus mode register. The descriptor words are numbered from 0 to 7 for both the transmit and receive engine.

Typical factors for deciding the descriptor word size are as follows:

- When the time stamping or receive checksum engines are not enabled, the extended descriptors are not required. The software can use descriptors with the default size of 16 bytes (4 words).
- When the time stamping feature is enabled, the software must allocate 32 bytes (8 words) of memory for every descriptor. (The time stamping feature is used with the IEEE 1588 PTP engine).
- When only the receive checksum offload is enabled (time stamping disabled), software must allocate 32 bytes (8 words) of memory for every descriptor. However, only word 4 of the extended words (descriptors 4-7) contains the required status information. T reat the rest of the extended words as reserved or dummy.

## Transmit Descriptor

The Transmit Descriptor Words figure shows the transmit descriptor structure in memory. The application software must program the TDES0 control bits during descriptor initialization. When the DMA updates the descriptor, it writes back all the control bits except the OWN bit (which it clears) and updates the status bits. The following tables give the contents of the transmitter descriptor word 0 (TDES0) through word 7 (TDES7).

Figure 29-14: Transmit Descriptor Words

<!-- image -->

Table 29-15: Transmit Descriptor Fields (TDES0)

|   Bit | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|-------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|    31 | OWN    | Ownership. When set, this bit indicates that the DMAowns the descriptor. When this bit is re- set, it indicates that the application owns the descriptor. The DMAclears this bit either when it completes the frame transmission or when the buffers allocated in the descriptor are read com- pletely. The ownership bit of the first descriptor of the frame must be set after all subsequent descriptors belonging to the same frame have been set. This configuration avoids a possible race condition between fetching a descriptor and the driver setting an ownership bit. |
|    30 | IC     | Interrupt on Completion. When set, this bit sets the transmit interrupt (DMA status register [0]) after the present frame is transmitted.                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 29-15: Transmit Descriptor Fields (TDES0) (Continued)

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|-------|----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29    | LS       | Last Segment. When set, this bit indicates that the buffer contains the last segment of the frame.                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 28    | FS       | First Segment. When set, this bit indicates that the buffer contains the first segment of a frame.                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 27    | DC       | Disable CRC. When this bit is set, the EMAC does not append a cyclic redundancy check (CRC) to the end of the transmitted frame. This functionality is valid only when the first segment (TDES0[28]) is set.                                                                                                                                                                                                                                                                                                             |
| 26    | DP       | Disable Pad. When set, the EMAC does not automatically add padding to a frame shorter than 64 bytes. When this bit is reset, the DMAautomatically adds padding and CRC to a frame shorter than 64 bytes. The CRC field is added despite the state of the DC (TDES0[27]) bit. This functionality is valid only when the first segment (TDES0[28]) is set.                                                                                                                                                                 |
| 25    | TTSE     | Transmit Time Stamp Enable. When set, this bit enables IEEE1588 hardware time stamping for the transmit frame referenced by the descriptor. This field is valid only when the first segment control bit (TDES0[28]) is set.                                                                                                                                                                                                                                                                                              |
| 24    | CRCR     | CRC Replacement Control. When set, the EMAC replaces the last four bytes of the transmitted packet with recalculated CRC bytes. The host should ensure that the CRC bytes are present in the frame being transferred from the transmit buffer. This bit is valid when the first segment bit (TDES0[28]) and the disable CRC bit (TDES0[27]) are set.                                                                                                                                                                     |
| 24    | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 23:22 | CIC      | Checksum Insertion Control. These bits control the checksum calculation and insertion. Bit en- codings are as follows: 00 = Checksum Insertion disabled. 01 = Only IP header checksum calculation and insertion are enabled. 10 = IP header checksum and payload checksum calculation and insertion are enabled, but pseu- do-header checksum is not calculated in hardware. 11 = IP header checksum and payload checksum calculation and insertion are enabled, and pseu- do-header checksum is calculated in hardware. |
| 21    | TER      | Transmit End of Ring. When set, this bit indicates that the descriptor list reached its final de- scriptor. The DMAreturns to the base address of the list, creating a descriptor ring.                                                                                                                                                                                                                                                                                                                                  |
| 20    | TCH      | Second Address Chained. When set, this bit indicates that the second address in the descriptor is the next descriptor address rather than the second buffer address. When TDES0[20] bit is set, TBS2 (TDES1[28:16]) is a do-not-care value. TDES0[21] takes precedence over TDES0[20].                                                                                                                                                                                                                                   |
| 19:18 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

Table 29-15: Transmit Descriptor Fields (TDES0) (Continued)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|-------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:18 | VLIC   | VLAN Insertion Control. When set, these bits request the MAC to perform VLAN tagging or untagging before transmitting the frames. If the frame is modified for VLAN tags, the MAC au- tomatically recalculates and replaces the CRC bytes. Bit encodings are as follows. 00 = Do not add a VLAN tag. 01 = Remove the VLAN tag from the frames before transmission. This option should be used only with the VLAN frames. 10 = Insert a VLAN tag with the tag value programmed in the VLAN tag inclusion or replace- ment EMAC_VLAN_INCL register. 11 = Replace the VLAN tag in frames with the tag value programmed in the VLAN tag inclusion or replacement EMAC_VLAN_INCL register. This option should be used only with the VLAN frames. These bits are valid when the first segment bit (TDES0[28]) is set. |
| 17    | TTSS   | Transmit Time Stamp Status. This bit is a status bit to indicate that a time stamp is captured for the described transmit frame. When this bit is set, TDES2 and TDES3 have a time stamp value captured for the transmit frame. This field is only valid when the last segment control bit of the descriptor (TDES0[29]) is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 16    | IHE    | IP Header Error. When set, this bit indicates that the EMAC transmitter detected an error in the IP datagram header. The transmitter checks the header length in the IPv4 packet against the number of header bytes received from the application. It indicates an error status when there is a mismatch. For IPv6 frames, a header error is reported if the main header length is not 40 bytes. Furthermore, the Ethernet length or type field value for an IPv4 or IPv6 frame must match the IP header version received with the packet. For IPv4 frames, an error status is also indicated if the header length field has a value less than 0x5.                                                                                                                                                             |
| 15    | ES     | Error Summary. Indicates the logical OR of the following bits: TDES0[14] = Jabber Timeout TDES0[13] = Frame Flush TDES0[11] = Loss of Carrier TDES0[10] = No Carrier TDES0[9] = Late Collision TDES0[8] = Excessive Collision TDES0[2] = Excessive Deferral TDES0[1] = Underflow Error TDES0[16] = IP Header Error TDES0[12] = IP Payload Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 14    | JT     | Jabber Timeout. When set, this bit indicates that the EMAC transmitter has experienced a jabber timeout. This bit is only set when the EMAC_MACCFG.JB bit is not set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 13    | FF     | Frame Flushed. When set, this bit indicates that the DMAor MFL flushed the frame due to a software flush command given by the CPU.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 29-15: Transmit Descriptor Fields (TDES0) (Continued)

| Bit   | Name    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|-------|---------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12    | IPE     | IP Payload Error. When set, this bit indicates that EMAC transmitter detected an error in the TCP, UDP, or ICMP IP datagram payload. The transmitter checks the payload length received in the IPv4 or IPv6 header against the actual number of TCP, UDP, or ICMP packet bytes received from the application. It issues an error status when there is a mismatch.                                                                                                                                                                                                                                  |
| 11    | LC      | Loss of Carrier. When set, this bit indicates a loss of carrier occurred during frame transmission. This functionality is valid only for the frames transmitted without collision when the EMAC op- erates in half-duplex mode. Loss of Carrier. When set, this bit indicates that the EMAC aborted the frame transmission be- cause of a collision occurring after the collision window (64 byte-times, including preamble, in RMII mode; 512 byte-times, including preamble and carrier extension, in RGMII mode). This bit is not valid if the underflow error bit is set.                      |
| 10    | NC      | No Carrier. When set, this bit indicates that the carrier sense signal from the PHY did not assert during transmission.                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 9     | LC      | Late Collision. When set, this bit indicates that the EMAC aborted the frame transmission be- cause of a collision occurring after the collision window (64 byte-times, including preamble). This bit is not valid if the underflow error bit is set. Late Collision. When set, this bit indicates that the EMAC aborted the frame transmission be- cause of a collision occurring after the collision window (64 byte-times, including preamble, in RMII mode; 512 byte-times, including preamble and carrier extension, in RGMII mode). This bit is not valid if the underflow error bit is set. |
| 8     | EC      | Excessive Collision. When set, this bit indicates that the EMAC aborted the transmission after 16 successive collisions, while attempting to transmit the current frame. If the EMAC_MACCFG.DR disable retry bit is set, this bit is set after the first collision, and the transmission of the frame aborts.                                                                                                                                                                                                                                                                                      |
| 7     | VF      | VLAN Frame. When set, this bit indicates that the transmitted frame was a VLAN-type frame.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 6:3   | CC      | Collision Count. When set, these bit indicate that the EMAC aborted the frame transmission due to a collision occurring after the collision window (64 byte-times, including preamble). This bit is not valid if the Underflow Error bit is set. This field is updated only in half-duplex mode.                                                                                                                                                                                                                                                                                                   |
| 6:3   | SLOTNUM | Slot Number Control Bits. In AV mode, these bits indicate the slot interval in which the data should be fetched from the corresponding buffers, addressed by TDES2 or TDES3. When the transmit descriptor is fetched, the DMAcompares the slot number value in this field with the slot function control and status register (RSN). It fetches the data from the buffers only if there is a match in values. These bits are valid only for AV channels (not channel 0).                                                                                                                            |
| 2     | ED      | Excessive Deferral. When set, this bit indicates that the transmission has ended because of exces- sive deferral when the EMAC_MACCFG.DC deferral check bit is set high. Excessive deferral is over 24,288-bit times. (155,680-bits times in 1,000-Mbps mode or when jumbo frame is ena- bled).                                                                                                                                                                                                                                                                                                    |
| 1     | UF      | Underflow Error. When set, this bit indicates that the EMAC aborted the frame because data arrived late from the application memory. Underflow error indicates that the DMAencountered                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 29-15: Transmit Descriptor Fields (TDES0) (Continued)

|   Bit | Name   | Description                                                                                                                                                                                                             |
|-------|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|       |        | an empty transmit buffer while transmitting the frame. The transmission process enters the sus- pended state and sets both transmit underflow ( EMAC_DMA0_STAT.UNF ) and transmit interrupt ( EMAC_DMA0_STAT.TI ) bits. |
|     0 | DB     | Deferred Bit. When set, this bit indicates that the EMAC defers before transmission because of the presence of carrier. This bit is valid only in half-duplex mode.                                                     |

Table 29-16: Transmit Descriptor Word 1 (TDES1)

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|-------|----------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-29 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 31-29 | SAIC     | Source Address Insertion Control. Request the MAC to add or replace the source address field in the Ethernet frame with the value given in the MAC address register 0 or MAC address register 1. If the source address field is modified in a frame, the MAC automatically recalculates and re- places the CRC bytes. SAIC[2] chooses between MAC address register 0 and MAC address regis- ter 1 for source address insertion or replacement. The following list describes SAIC[1:0]. 00 = Do not include the source address. 01 = Include or insert the source address. For reliable transmission, the application must provide frames without source addresses. 10 = Replace the source address. For reliable transmission, the application must provide frames with source addresses. 11 = Reserved. This field is valid only when the first segment bit (TDES0[28]) is set. |
| 28-16 | TBS2     | Transmit Buffer 2 Size. These bits indicate the second data buffer size in bytes. This field is not valid if TDES0[20] is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 15-13 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 12-0  | TBS1     | Transmit Buffer 1 Size. These bits indicate the first data buffer byte size, in bytes. If this field is 0, the DMAignores this buffer and uses buffer 2 or the next descriptor, depending on the value of TCH (TDES0[20]).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 29-17: Transmit Descriptor 2 (TDES2)

| Bit   | Name                     | Description                                                                                                  |
|-------|--------------------------|--------------------------------------------------------------------------------------------------------------|
| 31-0  | Buffer 1 Address Pointer | These bits indicate the physical address of buffer 1. There is no limitation on the buffer address alignment |

Table 29-18: Transmit Descriptor 3 (TDES3)

| Bit   | Name                                                 | Description                                                                                                                                                                                                                                                                                                                                                 |
|-------|------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | Buffer 2 Address Pointer (Next De- scriptor Address) | Indicates the physical address of buffer 2 when DMAuses a descriptor ring structure. If the sec- ond address chained (TDES1[24]) bit is set, this address contains the pointer to the physical memory where the next descriptor is present. The buffer address pointer must align to the bus width only when TDES1[24] is set. LSBs are ignored internally. |

Table 29-19: Transmit Descriptor 6 (TDES6)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                    |
|-------|--------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | TTSL   | Transmit Frame Time Stamp Low. The DMAupdates this field with the least significant 32 bits of the time stamp captured for the corresponding transmit frame. This field has the time stamp only if the last segment bit (LS) in the descriptor is set and time stamp status (TTSS) bit is set. |

Table 29-20: Transmit Descriptor 7 (TDES7)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                   |
|-------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | TTSH   | Transmit Frame Time Stamp High. The DMAupdates this field with the most significant 32 bits of the time stamp captured for the corresponding receive frame. This field has the time stamp only if the last segment bit (LS) in the descriptor is set and time stamp status (TTSS) bit is set. |

## DMA Transmit Process

The following sections describe how the transmission process works for direct memory access on the EMAC controller.

- Default (Non-OSF) Mode
- OSF Mode Enabled
- Transmit Frame Processing
- Transmit Polling Suspended

## Default (Non-OSF) Mode

The following sequence decribes the default process for DMA transmit works. The sequence applies DMA1 and DMA2 as well.

1. The application sets up the transmit descriptor (using TDES0- TDES3) and sets the OWN bit (TDES0) after setting up the corresponding data buffers with Ethernet frame data.
2. Once the EMAC\_DMA0\_OPMODE.ST bit is set, the DMA enters the run state.
3. While in the run state, the DMA polls the transmit descriptor list for frames requiring transmission. After polling starts, it continues in either sequential descriptor ring order or chained order. If the DMA detects a descriptor flagged as owned by the application, or if an error condition occurs, transmission suspends. Both the transmit buffer unavailable ( EMAC\_DMA0\_STAT.TU ) and normal interrupt summary ( EMAC\_DMA0\_STAT.NIS ) bits are set. The transmit engine proceeds to Step 9.
4. If the acquired descriptor is flagged as owned by DMA (TDES0 [31] = 1#b1), the DMA decodes the transmit data buffer address from the acquired descriptor.
5. The DMA fetches the transmit data from the application memory and transfers the data to the MFL for transmission.

6. If an Ethernet frame is stored over data buffers in multiple descriptors, the DMA closes the intermediate descriptor and fetches the next descriptor. Steps 3, 4, and 5 repeat until the end-of-Ethernet-frame data transfers to the MFL.
7. Frame transmission completes. If IEEE 1588 time stamping was enabled for the frame, the time stamp value obtained from MFL is written to the transmit descriptor (TDES2 and TDES3) that contains the end-of-frame buffer. (The transmit status indicates if IEEE 1588 time stamping enables). The status information is then written to this transmit descriptor (TDES0). Because the OWN bit is cleared during this step, the application now owns this descriptor. If time stamping was not enabled for this frame, the DMA does not alter the contents of TDES2 and TDES3.
8. Transmit interrupt ( EMAC\_DMA0\_STAT.TI ) is set after completing transmission of a frame. The frame has interrupt on completion (TDES1 [31]) set in its last descriptor. The DMA engine then returns to Step 3.
9. In the suspend state, the DMA tries to reacquire the descriptor (and returns to Step 3) when it receives a transmit poll demand and the EMAC\_DMA0\_STAT.UNF bit is cleared.

NOTE: If the EMAC\_DMA0\_OPMODE.OSF bit is not set, the actual inter frame gap (IFG) is more than the value programmed in the EMAC\_MACCFG register.

## OSF Mode Enabled

While in the run state, the transmit process can simultaneously acquire two frames without closing the status descriptor of the first (if the EMAC\_DMA0\_OPMODE.OSF bit is set). As the transmit process finishes transferring the first frame, it immediately polls the transmit descriptor list for the second frame. If the second frame is valid, the transmit process transfers this frame before writing the status information of the first frame.

In OSF mode, the run state transmit DMA operates in the following sequence.

1. The DMA operates as described in steps 1-6 of Default (Non-OSF) Mode.
2. Without closing the previous last descriptor of the frame, the DMA fetches the next descriptor.
3. If the DMA owns the acquired descriptor, the DMA decodes the transmit buffer address in this descriptor. If the DMA does not own the descriptor, the DMA goes into suspend mode and skips to Step 7.
4. The DMA fetches the transmit frame from the application memory and transfers the frame to the MFL until the end-of-frame data is transferred. It closes the intermediate descriptors when this frame splits across multiple descriptors.
5. The DMA waits for the frame transmission status and time stamp of the previous frame. Once the status is available, the DMA writes the time stamp to TDES2 and TDES3, if the time stamp was captured (as indicated by a status bit). The DMA then writes the status, with a cleared OWN bit, to the corresponding TDES0, thus closing the descriptor. If time stamping was not enabled for the previous frame, the DMA does not alter the contents of TDES2 and TDES3.
6. If enabled, the transmit interrupt is set; the DMA fetches the next descriptor, and then proceeds to Step 3 (when status is normal). If the previous transmission status shows an underflow error, the DMA goes into suspend mode (Step 7).

7. In suspend mode, if a pending status and time stamp are received from the MFL, the DMA writes the time stamp (if enabled for the current frame) to TDES2 and TDES3. The DMA then writes the status to the corresponding TDES0. It then sets relevant interrupts and returns to suspend mode.
8. After receiving a transmit poll demand ( EMAC\_DMA0\_TXPOLL ), the DMA can exit suspend mode and enter the run state. (Go to Step 1 or Step 2 depending on pending status)
3. NOTE: If the EMAC\_DMA0\_OPMODE.OSF bit is set, the DMA fetches the next descriptor in advance of closing the current descriptor. Therefore, the descriptor chain must have more than two different descriptors for proper operation.
4. NOTE: If the EMAC\_DMA0\_OPMODE.OSF bit is set, the DMA starts fetching the second frame immediately after completing the transfer of the first frame to the FIFO. It does not wait for the status to update. In the meantime, the MFL receives the second frame into the FIFO while transmitting the first frame. The difference in cycles is not seen for the first descriptor, because the time taken for the complete descriptor processing remains the same whether EMAC\_DMA0\_OPMODE.OSF is set or not. The difference appears only for the following descriptor because its processing began earlier.

## Transmit Frame Processing

The transmit DMA engine expects that the data buffers contain complete Ethernet frames, excluding: preamble, pad bytes, and FCS fields. The destination address, source address, and type or length fields contain valid data. If the transmit descriptor indicates that the EMAC CORE must disable CRC or PAD insertion, the buffer must have complete Ethernet frames (excluding preamble), including the CRC bytes.

Frames can be data-chained and can span several buffers. Frames must be delimited by the first descriptor (TDES0[28]) and the last descriptor (TDES0[29]), respectively.

As transmission starts, the first descriptor must have (TDES0[28]) set. Frame data transfers from the application buffer to the transmit FIFO. Concurrently, if last descriptor (TDES0[29]) of the current frame clears, the transmit process attempts to acquire the next descriptor. The transmit process expects this descriptor to have TDES0[28] clear. If TDES1[29] is clear, it indicates an intermediary buffer. If TDES0[29] is set, it indicates the last buffer of the frame.

After the last buffer of the frame has been transmitted, the DMA writes back the final status information. The DMA writes to the transmit descriptor 0 (TDES0) word of the descriptor that has the last segment set in transmit descriptor 0 (TDES0[29]). Now, if interrupt-on-completion (TDES0[30]) is set, the transmit interrupt (DMA\_STAT [0]) is set, the next descriptor is fetched, and the process repeats.

Actual frame transmission begins after the MFL transmit FIFO has reached either a programmable transmit threshold ( EMAC\_DMA0\_OPMODE.TTC ), or a full frame is contained in the FIFO. There is also an option for store-andforward mode ( EMAC\_DMA0\_OPMODE.TSF ). Descriptors are released ( OWN bit TDES0 [31] clears) when the DMA finishes transferring the frame.

## Transmit Polling Suspended

Either of the following conditions suspends transmit polling:

1. The DMA detects a descriptor owned by the application (TDES0 [31] = 0).

2. A frame transmission aborts when a transmit error due to underflow is detected. The appropriate transmit descriptor 0 (TDES0) bit is set.

If the second condition occurs, both of the abnormal interrupt summary ([15]) and transmit underflow bits ([5]) are set. The information is written to transmit descriptor 0, causing the suspension. If the DMA goes into a SUSPEND state due to the first condition, then both EMAC\_DMA0\_STAT.NIS and EMAC\_DMA0\_STAT.TU are set.

In both cases, the position in the transmit list is retained. The retained position is that of the descriptor following the last descriptor closed by the DMA.

The driver must explicitly issue a transmit poll demand command after rectifying the suspension cause. If the first condition occurs, the driver must give descriptor ownership to the DMA and then issue a poll demand command to resume the transfer.

## Receive Descriptor

The Receive Descriptor Words figure shows the structure of the receive descriptor. It can have 32 bytes of descriptor data (8 DWORDs) when advanced time stamping or checksum is enabled.

Figure 29-15: Receive Descriptor Words

<!-- image -->

Table 29-21: Receive Descriptor Fields (RDES0)

|   Bit | Name   | Description                                                                                                                                                                                                                                                                                                                    |
|-------|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|    31 | OWN    | Ownership. When set, this bit indicates that the DMAof the EMAC subsystem owns the descriptor. When this bit is reset, this bit indicates that the application owns the descriptor. The DMAclears this bit either when it completes the frame reception or when the buffers that are associated with this descriptor are full. |
|    30 | AFM    | Destination Address Filter Fail. When set, this bit indicates a frame that failed in the DA filter in the EMAC CORE.                                                                                                                                                                                                           |

Table 29-21: Receive Descriptor Fields (RDES0) (Continued)

| Bit   | Name                 | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|-------|----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29-16 | FL                   | Frame Length. These bits indicate the byte length of the received frame that transferred to applica- tion memory (including CRC). This field is valid when last descriptor (RDES0[8]) is set and either the descriptor error (RDES0[14]) or overflow error bits are reset. This field is valid when last de- scriptor (RDES0[8]) is set. When the last descriptor and error summary bits are not set, this field indicates the accumulated number of bytes that transferred for the current frame. |
| 15    | ES                   | Error Summary. Indicates the logical OR of the following bits. RDES0[1] = CRC Error RDES0[3] = RGMII Receive Error RDES0[4] = Watchdog Timeout RDES0[6] = Late Collision RDES0[7] = Time Stamp Available RDES4[4:3] = IP Header/Payload Error RDES0[11] = Overflow Error RDES0[14] = Descriptor Error. This field is valid only when the last descriptor (RDES0[8]) is set.                                                                                                                        |
| 14    | DE                   | Descriptor Error. When set, this bit indicates a frame truncation caused by a frame that does not fit within the current descriptor buffers. The DMAdoes not own the next descriptor. The frame is truncated. This field is valid only when the last descriptor (RDES0[8]) is set.                                                                                                                                                                                                                 |
| 13    | Reserved             | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 13    | SAF                  | Source Address Filter Fail. When set, this bit indicates that the SA field of frame failed the SA filter in the EMAC Core.                                                                                                                                                                                                                                                                                                                                                                         |
| 12    | LE                   | Length Error. When set, this bit indicates that the actual length of the frame received and that the length or type field does not match. This bit is valid only when the frame type (RDES0[5]) bit is reset.                                                                                                                                                                                                                                                                                      |
| 11    | OE                   | Overflow Error. When set, this bit indicates that the received frame is damaged due to buffer over- flow in MFL.                                                                                                                                                                                                                                                                                                                                                                                   |
| 10    | VLAN                 | VLAN Tag. When set, this bit indicates that the frame pointed to by this descriptor is a VLAN frame tagged by the EMAC CORE.                                                                                                                                                                                                                                                                                                                                                                       |
| 9     | FS                   | First Descriptor. When set, this bit indicates that this descriptor contains the first buffer of the frame. If the size of the first buffer is 0, the second buffer contains the beginning of the frame. If the size of the second buffer is also 0, the next descriptor contains the beginning of the frame.                                                                                                                                                                                      |
| 8     | LS                   | Last Descriptor. When set, this bit indicates that the buffers pointed to by this descriptor are the last buffers of the frame                                                                                                                                                                                                                                                                                                                                                                     |
| 7     | Time Stamp Available | When set, this bit indicates that a snapshot of the time stamp is written in descriptor words 6 (RDES6) and 7 (RDES7). This functionality is valid only when the last descriptor bit (RDES0[8]) is set                                                                                                                                                                                                                                                                                             |
| 6     | LC                   | Late Collision. When set, this bit indicates that a late collision has occurred while receiving the frame in half-duplex mode.                                                                                                                                                                                                                                                                                                                                                                     |

Table 29-21: Receive Descriptor Fields (RDES0) (Continued)

|   Bit | Name                      | Description                                                                                                                                                                                                                                                                                  |
|-------|---------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|     5 | FT                        | Frame Type. When set, this bit indicates that the receive frame is an Ethernet-type frame (the LT field is greater than or equal to 16'h0600). When this bit is reset, it indicates that the received frame is an IEEE802.3 frame. This bit is not valid for runt frames less than 14 bytes. |
|     4 | RWT                       | Receive Watchdog Timeout. When set, this bit indicates that the receive watchdog timer has expired while receiving the current frame. The current frame is truncated after the watchdog timeout.                                                                                             |
|     3 | Reserved                  | Reserved                                                                                                                                                                                                                                                                                     |
|     3 | RE                        | Receive Error. When set, this bit indicates that the RGMII PHY sent RGMII RXERR on RXCTL pin during frame reception. This error also includes the carrier extension error in RGMII and half- duplex mode.                                                                                    |
|     2 | DE                        | Dribble Bit Error. When set, this bit indicates that the received frame has a non-integer multiple of bytes (odd nibbles).                                                                                                                                                                   |
|     1 | CE                        | CRC Error. When set, this bit indicates that a Cyclic Redundancy Check (CRC) error occurred on the received frame. This field is valid only when the last descriptor (RDES0[8]) is set.                                                                                                      |
|     0 | Extended Status Available | When set, this bit indicates that the extended status is available in descriptor word 4 (RDES4). This functionality is valid only when the last descriptor bit (RDES0[8]) is set.                                                                                                            |

Table 29-22: Receive Descriptor Fields 1 (RDES1)

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                                                                                           |
|-------|----------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31    | DIC      | Disable Interrupt on Completion. When set, this bit prevents setting the EMAC_DMA0_STAT.RI bit of the status register for the received frame ending in the buffer indicated by this descriptor. This activity, in turn, disables the assertion of the interrupt to the application due to RI for that frame.                                                                                                          |
| 30-29 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                              |
| 28-16 | RBS2     | Receive Buffer 2 Size. These bits indicate the second data buffer size, in bytes. The buffer size must be a multiple of 4 (32-bit bus), even if the value of RDES3 (buffer2 address pointer) does not align to bus width. If the buffer size is not an appropriate multiple of 4, 8, or 16, the resulting behavior is undefined. This field is not valid if RDES1[14] is set.                                         |
| 15    | RER      | Receive End of Ring. When set, this bit indicates that the descriptor list reached its final descriptor. The DMAreturns to the base address of the list, creating a descriptor ring.                                                                                                                                                                                                                                  |
| 14    | RCH      | Second Address Chained. When set, this bit indicates that the second address in the descriptor is the next descriptor address rather than the second buffer address. When this bit is set, RBS2 (RDES1[28:16]) is a do-not-care value. RDES1[15] takes precedence over RDES1[14].                                                                                                                                     |
| 13    | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                              |
| 12-0  | RBS1     | Receive Buffer 1 Size. Indicates the size of the first data buffer in bytes. The buffer size must be a multiple of 4 (32-bit bus), even if the value of RDES2 (buffer1 address pointer) is not aligned. When the buffer size is not a multiple of 4, the resulting behavior is undefined. If this field is 0, the DMAignores this buffer and uses buffer 2 or next descriptor depending on the value of RCH (Bit 14). |

Table 29-23: Receive Descriptor Fields 2 (RDES2)

| Bit   | Name                     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|-------|--------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | Buffer 1 Address Pointer | These bits indicate the physical address of buffer 1. There are no limitations on the buffer address alignment except for the following condition: The DMAuses the configured value for its address generation when the RDES2 value stores the start of frame. The DMAperforms a write operation with the RDES2[1:0] bits as 0 during the transfer of the start of frame. However, the frame data shifts per the address pointer of the actual buffer. The DMAignores RDES2[1:0] when the address pointer is to a buffer where the middle or last part of the frame is stored. (RDES2[1:0] corresponds to a bus width of 32). |

Table 29-24: Receive Descriptor Fields 3 (RDES3)

| Bit   | Name                                                 | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|-------|------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | Buffer 2 Address Pointer (Next De- scriptor Address) | These bits indicate the physical address of buffer 2 when DMAuses a descriptor ring structure. If the second address chained (RDES1[24]) bit is set, this address contains the pointer to the physi- cal memory where the next descriptor is present. If RDES1[24] is set, the buffer (next descriptor) address pointer must be bus width-aligned (RDES3[1:0] = 0, corresponding to a bus width of 32. LSBs are ignored internally.) However, when RDES1[24] is reset, there are no limitations on the RDES3 value, except for the following condition: The DMAuses the configured value for its buf- fer address generation when the RDES3 value stores the start of frame. The DMAignores RDES3[1:0] when the address pointer is to a buffer where the middle or last part of the frame is stored. (RDES3[1:0] corresponds to a bus width of 32.) |

Table 29-25: Receive Descriptor Fields 4 (RDES4)

| Bit   | Name                      | Description                                                                                                                                                                                                                                                                               |
|-------|---------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-26 | Reserved                  | Reserved                                                                                                                                                                                                                                                                                  |
| 25    | Layer 4 Filter Match      | When set, this bit indicates that the received frame matches layer 4 filter fields. This status is given only when one of the following conditions is true: • Layer 3 fields are not enabled and all enabled layer 4 fields match. • All enabled layer 3 and layer 4 filter fields match. |
| 24    | Layer 3 Filter Match      | When set, this bit indicates that the received frame matches layer 3 IP address fields. This status is given only when one of the following conditions is true: • All enabled layer 3 fields match and all enabled layer 4 fields are bypassed. • All enabled filter fields match.        |
| 23-21 | Reserved                  | Reserved                                                                                                                                                                                                                                                                                  |
| 20-18 | VLAN Tag Priority         | These bits give the VLAN tag's user value in the received packet. These bits are valid only when the RDES4[16] and RDES4[17] are set.                                                                                                                                                     |
| 17    | AV Tagged Packet Received | When set, this bit indicates that an AV tagged packet is received. Otherwise, this bit indicates that an untagged AV packet is received. This bit is valid when RDES4[16] is set.                                                                                                         |
| 16    | AV Packet Re- ceived      | When set, this bit indicates that an AV packet is received.                                                                                                                                                                                                                               |
| 15    | Reserved                  | Reserved                                                                                                                                                                                                                                                                                  |

Table 29-25: Receive Descriptor Fields 4 (RDES4) (Continued)

| Bit   | Name                   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|-------|------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14    | Timestamp Drop- ped    | When set, this bit indicates that the time stamp is captured for this frame but dropped in the MFL RxFIFO because of overflow.                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 13    | PTP Version            | When set, this bit indicates that the received PTP message has the IEEE 1588 version 2 format. When reset, it has the version 1 format. This description is valid only if the message type (RDES4[11:8]) is non-zero.                                                                                                                                                                                                                                                                                                                                                         |
| 12    | PTP Frame Type         | When set, this bit indicates that the PTP message transfers directly over Ethernet. When this bit is not set and the message type is non-zero, it indicates that the PTP message transfers over UDP- IPv4 or UDP-IPv6. Bits 6 and 7 have the information on IPv4 or IPv6.                                                                                                                                                                                                                                                                                                     |
| 11-8  | Message Type           | These bits are encoded to give the type of the message received. 0000 = No PTP message received 0001 = SYNC (all clock types) 0010 = Follow_Up (all clock types) 0011 = Delay_Req (all clock types) 0100 = Delay_Resp (all clock types) 0101 = Pdelay_Req (in peer-to-peer transparent clock) or Announce (in ordinary or boundary clock) 0110 = Pdelay_Resp (in peer-to-peer transparent clock) or Management (in ordinary or boundary clock) 0111 = Pdelay_Resp_Follow_Up (in peer-to-peer transparent clock) or Signaling (for ordinary or boundary clock) 1xxx - Reserved |
| 7     | IPv6 Packet Re- ceived | When set, this bit indicates that the received packet is an IPv6 packet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 6     | IPv4 Packet Re- ceived | When set, this bit indicates that the received packet is an IPv4 packet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 5     | IP Checksum By- passed | When set, this bit indicates that the checksum offload engine is bypassed.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 4     | IP Payload Error       | When set, this bit indicates that the 16-bit IP payload checksum that the core calculated does not match the corresponding checksum field in the received segment. (For example: the TCP, UDP, or ICMP checksum). The bit is also set when the TCP, UDP, or ICMP segment length does not match the payload length value in the IP header field.                                                                                                                                                                                                                               |
| 3     | IP Header Error        | When set, this bit indicates that the 16-bit IPv4 header checksum calculated by the core does not match the received checksum bytes. Or, it indicates that the IP datagram version is not consistent with the Ethernet type value.                                                                                                                                                                                                                                                                                                                                            |

Table 29-25: Receive Descriptor Fields 4 (RDES4) (Continued)

| Bit   | Name            | Description                                                                                                                                                                                                                                                                                                                                                                                               |
|-------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2-0   | IP Payload Type | These bits indicate the type of payload encapsulated in the IP datagram processed by the receive checksum offload engine (COE). The COE also sets these bits to 2'b00 when it does not process the payload of the IP datagram. It does not process the payload because of an IP header error or fragmented IP . 000 = Unknown or did not process IP payload 001 = UDP 010 = TCP 011 = ICMP 1xx = Reserved |

Table 29-26: Extended Receive Descriptor Fields 4 (RDES4)

| Bit   | Name                      | Description                                                                                                                                                                                                                                                                               |
|-------|---------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-26 | Reserved                  | Reserved                                                                                                                                                                                                                                                                                  |
| 25    | Layer 4 Filter Match      | When set, this bit indicates that the received frame matches layer 4 filter fields. This status is given only when one of the following conditions is true: • Layer 3 fields are not enabled and all enabled layer 4 fields match. • All enabled layer 3 and layer 4 filter fields match. |
| 24    | Layer 3 Filter Match      | When set, this bit indicates that the received frame matches layer 3 IP address fields. This status is given only when one of the following conditions is true: • All enabled layer 3 fields match and all enabled layer 4 fields are bypassed. • All enabled filter fields match.        |
| 23-21 | Reserved                  | Reserved                                                                                                                                                                                                                                                                                  |
| 20-18 | VLAN Tag Priority         | These bits give the VLAN tag's user value in the received packet. These bits are valid only when the RDES4[16] and RDES4[17] are set.                                                                                                                                                     |
| 17    | AV Tagged Packet Received | When set, this bit indicates that an AV tagged packet is received. Otherwise, this bit indicates that an untagged AV packet is received. This bit is valid when RDES4[16] is set.                                                                                                         |
| 16    | AV Packet Re- ceived      | When set, this bit indicates that an AV packet is received.                                                                                                                                                                                                                               |
| 15    | Reserved                  | Reserved                                                                                                                                                                                                                                                                                  |
| 14    | Timestamp Drop- ped       | When set, this bit indicates that the timestamp was captured for this frame but got dropped in the MFL RxFIFO because of overflow.                                                                                                                                                        |

Table 29-27: Receive Descriptor Fields 6 (RDES6)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                      |
|-------|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | RTSL   | Receive Frame Time Stamp Low. The DMAupdates this field with the least significant 32 bits of the time stamp captured for the corresponding receive frame. The DMAupdates this field only for the last descriptor of the receive frame. The status bit (RDES0[8]) indicates the last descriptor. |

Table 29-28: Receive Descriptor Fields 7 (RDES7)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                      |
|-------|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | RTSH   | Receive Frame Time Stamp High. The DMAupdates this field with the most significant 32 bits of the time stamp captured for the corresponding receive frame. The DMAupdates this field only for the last descriptor of the receive frame. The status bit (RDES0[8]) indicates the last descriptor. |

## EMAC DMA Receive Process

The following sections describe how the receive process for direct memory access works on the EMAC controller.

- Receive Frame Processing
- Receive Descriptor Acquisition
- Receive Process Suspended

The reception process for DMA works as follows:

1. The application sets up receive descriptors (RDES0-RDES3) and sets the OWN bit (RDES0 [31]).
2. Once the EMAC\_DMA0\_OPMODE.SR bit is set, the DMA enters the run state. While in the run state, the DMA attempts to acquire free descriptors by polling the receive descriptor list. If the fetched descriptor is not free (the application owns the descriptor), the DMA enters the suspend state and jumps to Step 9.
3. The DMA decodes the receive data buffer address from the acquired descriptors.
4. Incoming frames are processed and placed in the data buffers of the acquired descriptor.
5. When the buffer is full or the frame transfer is complete, the receive engine fetches the next descriptor.
6. If the current frame transfer is complete, the DMA proceeds to Step 7. If IEEE 1588 time stamping is enabled, the DMA writes the time stamp (if available) to the current descriptor. If the DMA does not own the next fetched descriptor and the frame transfer is not complete, the DMA sets the descriptor error bit in the RDES0. (The bit is set unless flushing is disabled.) The DMA closes the current descriptor (clears the OWN bit). The DMA marks it as intermediate by clearing the last segment (LS) bit in the RDES0 value (marks it as last descriptor if flushing is not disabled). The DMA then proceeds to Step 8. If the DMA owns the next descriptor but the current frame transfer is not complete, the DMA closes the current descriptor as intermediate and reverts to Step 4.
7. If IEEE 1588 time stamping is enabled, the DMA writes the time stamp (if available) to RDES2 and RDES3 of the current descriptor. The DMA then takes the status of the receive frame from the MFL and writes the status word to RDES0 of the current descriptor. The OWN bit is cleared and the last segment bit is set.

8. The receive engine checks the OWN bit of the latest descriptor. If the host owns the descriptor (OWN bit is 0), the EMAC\_DMA0\_STAT.RU bit is set. The DMA receive engine enters the suspended state (Step 9). If the DMA owns the descriptor, the engine returns to Step 4 and awaits the next frame.
9. Before the receive engine enters the suspend state, partial frames are flushed from the receive FIFO (programs control flushing using the EMAC\_DMA0\_OPMODE.DFF bit).
10. The receive DMA exits the suspend state when a receive poll demand is given or the start of next frame is available from the receive FIFO of the MFL. The engine proceeds to Step 2 and refetches the next descriptor.

## Receive Frame Processing

The EMAC transfers the received frames to the application memory only when:

- the frame passes the address filter subblock, and
- the frame size is greater than or equal to configurable threshold bytes set for the receive FIFO of MFL, or
- the complete frame is written to the FIFO in store-and-forward mode.

If the frame fails the address filtering, the EMAC block drops the frame (unless the EMAC\_MACFRMFILT.RA bit is set). Frames that are shorter than 64 bytes, because of collision or premature termination, can be purged from the receive FIFO.

After receiving 64 bytes (configurable threshold), the MFL block requests that the DMA block begin transferring the frame data to the receive buffer pointed to by the current descriptor. The DMA sets first descriptor (RDES0 [9]) after the SCB becomes ready to receive the data (if DMA is not fetching transmit data from the application). The descriptors release when the OWN (RDES [31]) bit is reset to 0. The bit is reset either as the data buffer fills up or as the last segment of the frame is transferred to the receive buffer. If the frame is contained in a single descriptor, both the last descriptor (RDES [8]) and the first descriptor (RDES [9]) are set.

The DMA fetches the next descriptor, sets the last descriptor (RDES [8]) bit, and releases the RDES0 status bits in the previous frame descriptor. Then, the DMA sets the EMAC\_DMA0\_STAT.RI bit. The same process repeats unless the DMA encounters a descriptor the application owns. If this encounter occurs, the receive process sets the EMAC\_DMA0\_STAT.RU bit and then enters the suspend state. The position in the receive list is retained.

## Receive Descriptor Acquisition

The receive engine always attempts to acquire an extra descriptor in anticipation of an incoming frame. Descriptor acquisition is attempted when any of the following conditions is satisfied:

- The EMAC\_DMA0\_OPMODE.SR bit is set immediately after being placed in the run state.
- The data buffer of current descriptor is full before the frame ends for the current transfer.
- The controller completes frame reception, but the current receive descriptor is not yet closed.
- The receive process suspends because of an application-owned buffer (RDES0 [31] = 0) and a new frame is received.
- A receive poll demand issues.

## Receive Process Suspended

If a new receive frame arrives while the receive process is in the suspend state, the DMA refetches the current descriptor in the application memory. If the DMA now owns the descriptor, the receive process reenters the run state and starts frame reception. If the application still owns the descriptor, by default, the DMA discards the current frame at the top of the receive FIFO and increments the missed frame counter. If more than one frame is stored in the receive FIFO, the process repeats.

Avoid the discarding or flushing of the frame at the top of the receive FIFO by setting the EMAC\_DMA0\_OPMODE.DFF bit. In such conditions, the receive process sets the receive buffer unavailable status and returns to the suspend state.

## OWN Bit (Ownership) Semaphore

Usage or ownership of the transmit or receive descriptor between the application and EMAC is mutually exclusive. While the EMAC accesses the descriptor, the application cannot modify it. Conversely, while the host updates the descriptor, the EMAC cannot use the content of the descriptor. This functionality is implemented through the OWN bit in the transmit or receive descriptor, acting as a semaphore to prevent multiple, simultaneous access to the descriptors.

The following example is based on a usage case of 4 WORDs enabled for descriptors. A chain structure configuration is assumed. (The EMAC\_DMA0\_BUSMODE.ATDS bit is not set). However, the explanation of the OWN bit semaphore remains consistent irrespective of any particular mode of operation.

## 1. Transmit OWN Bit:

- TDES0 - TDES3 words implement the transmit descriptors. TDES0 [31] is defined as the OWN bit. When TDES0 [31] is set to 0, this bit indicates that the descriptor is available for the application to update. The application sets up the descriptors, including the buffer addresses, by updating TDES0 through TDES3.
- To release ownership of the descriptor to the EMAC, the application sets the transmit OWN bit, TDES0 [31], to 1. TDES0 [31] = 1 indicates that the descriptor is ready for the EMAC to use. DMA reads the descriptors, then fetches the data for transmission from the buffer locations pointed to by the transmit descriptors (TDES2 and TDES3). When either the last data buffer is empty or the end-of-frame is reached, DMA clears the TDES0 [31] bit to 0. Now, the transmit descriptor releases to the application for updates.

## 2. Receive OWN Bit:

- RDES0 - RDES3 words implement the receive descriptors. RDES0 [31] is defined as the OWN bit. When RDES0 [31] is set to 0, this bit indicates that the descriptor is available for the application to update. The application sets up the descriptors, including the buffer locations for writing the received data, by updating RDES0 through RDES3. To give ownership of the descriptor to the EMAC, the host sets the receive OWN bit, RDES0 [31], to 1.
- RDES0 [31] = 1 indicates that the descriptor is ready for use by the EMAC. DMA reads the descriptors, then writes the received data to the buffers with locations pointed to by the receive descriptors (RDES2

and RDES3). When either the last data buffer is full or the end-of-frame is reached, DMA clears the RDES0 [31] bit to 0. Now, the receive descriptor releases to the application for updates

## Application Data Buffer Alignment

The transmit and receive data buffers do not have any restrictions on the start address alignment. The start address for the buffers aligns to any of the 4 bytes. However, the DMA always initiates transfers with the address aligned to the bus width with dummy data for the byte lanes not required. This alignment typically happens during the transfer of the beginning or end of an Ethernet frame.

## Example for Buffer Read

If the transmit buffer address is 0x0002 and 15 bytes must transfer, the DMA reads 5 full words (5 x 32-bit data) from address 0x0000. However, when transferring data to the EMAC transmit FIFO, the extra bytes (the first 2 bytes) are dropped or ignored. Similarly, the last 3 bytes of the last transfer are also ignored. The DMA always transfers a full 32-bit data to the transmit FIFO, unless it is the end-of-frame.

## Example for Buffer Write

If the receive buffer address is 0x0002 and 15 bytes of a received frame must transfer, the DMA writes 5 full words (5 x 32-bit data) to address 0x0000. However, the first 2 bytes of first transfer and the last 3 bytes of the third transfer have dummy data.

## Buffer Size Calculations

The DMA engines do not update the size fields in the transmit and receive descriptors alone. The DMA updates only the status fields (RDES0 and TDES0) of the descriptors. The driver must perform the size calculations. The transmit DMA transfers the exact number of bytes (indicated by buffer size field of TDES1) towards the EMAC CORE. If a descriptor is marked as first (FS bit of TDES1 is set), then the DMA marks the first transfer from the buffer as the start of frame. If a descriptor is marked as last (LS bit of TDES1), the DMA marks the last transfer from that data buffer as the end-of frame to the EMAC.

The receive DMA transfers data to a buffer until the buffer is full or the end-of frame is received from the MFL. If a descriptor is not marked as last (LS bit of RDES0), then the buffers of the descriptor are full. The amount of valid data in a buffer is its buffer size field minus the data buffer pointer offset, when the FS bit of that descriptor is set. The offset is zero when the data buffer pointer aligns to the data bus width. If a descriptor is marked as last, then the buffer cannot be full (as indicated by the buffer size in RDES1). To compute the amount of valid data in this final buffer, the driver must:

- Read the frame length (FL bits of RDES0 [29:16]), and
- Subtract the sum of the buffer sizes of the preceding buffers in the frame

The receive DMA always transfers the start of next frame with a new descriptor.

## EMAC FIFO Layer (EMAC MFL)

The MAC FIFO layer provides FIFO memory to buffer and regulates the frames between the application system memory and the EMAC CORE. It also allows the transfer of data between the application clock domain and the

EMAC clock domains. The MFL layer has transfer controllers for each direction, called the transmit controller (TxFIFO) and the receive controller (RxFIFO). The datapath for both directions is 32-bit wide and each controller has a dedicated FIFO.

The EMAC0 transmit FIFO size is fixed to 2048 bytes. The EMAC0 receive FIFO size is fixed to 2048 bytes.

## FIFO Layer Transmit Path

The DMA engine controls all transactions for the transmit path with the application. Ethernet frames read from the system memory are pushed into the FIFO by the DMA. The frame is then popped out and transferred to the EMAC CORE when triggered. When the end-of-frame transfers, the status of the transmission is taken from the EMAC CORE and transferred back to the DMA. The FIFO fill level is indicated to the DMA so that it can initiate a data fetch in required bursts from the system memory through the SCB interface.

When the DMA enables the EMAC\_DMA0\_OPMODE.OSF bit, the MFL receives the second frame into the FIFO while transmitting the first frame. When the first frame has transferred, the status is sent to DMA. If the DMA has already completed sending the second packet to the MFL, it waits for the status of the first packet before proceeding to the next frame.

The following are the modes of operation for FIFO transactions.

1. Threshold mode. When the number of bytes in the FIFO crosses the configured threshold level, the data is ready to be popped out and forwarded to the EMAC CORE. The data is also ready when the end-of-frame is written before the threshold is crossed. The DMA uses the TTC bits of the DMA bus mode register to configure the threshold level.
2. Store-and-Forward mode. In this mode, the MFL pops the frame towards the EMAC CORE after a complete frame is stored in the FIFO. If the TX FIFO size is smaller than the Ethernet frame for transmission (such as a jumbo frame), then the frame forwards in two cases. The TX FIFO is almost full or the requested FIFO does not have space to accommodate the requested burst-length. Therefore, the FIFO read controller never stalls in store-and-forward mode even if the Ethernet frame length is bigger than the TX FIFO depth.

The FIFO threshold in the store-and-forward mode is given by the following formula.

DataWidth = 32 bits and PBL = Burst Length programmed through the DMA\_BUSMODE register.

- NOTE: To avoid occurrences of a TX underflow event when using the store-and-forward mode, or in other words, to ensure that the entire frame is stored in the FIFO before the MFL pops the frame towards the EMAC CORE for transmission, ensure that the FIFO threshold (calculated with the above formula) is greater than the packet size. The PBL needs to be programmed accordingly.

## Transmit FIFO and Half-Duplex Retransmissions

While a frame transfers from the FIFO, a collision event can occur on the EMAC line interface in half-duplex mode. The EMAC then indicates a retry attempt to the MFL. The EMAC gives the status before the end-of-frame transfers from MFL. Then, the MFL enables the retransmission by popping out the frame again from the FIFO.

After more than 96 bytes pop out of FIFO, the FIFO controller frees up that space. The controller makes it available to the DMA to push in more data. Retransmission is not possible after this threshold is crossed or when the EMAC CORE indicates a late-collision event.

## Transmit FIFO Flush Operation

The EMAC provides control to the software to flush the transmit FIFO in the MFL layer using the EMAC\_DMA0\_OPMODE.FTF bit. The flush operation is immediate. The MFL clears the transmit FIFO and the corresponding pointers to the initial state. It clears the FIFO and pointers even if it is in the middle of transferring a frame to the EMAC CORE. The data that the MAC transmitter has already accepted is not flushed. The data is scheduled for transmission and results in underflow. The transmit FIFO does not complete the transfer of the rest of the frame. As in all underflow conditions, a runt frame is transmitted and observed on the line. The status of the frame is marked with both underflow and frame flush events (TDES0 bits 13 and 1).

The MFL also stops accepting any data from the application (DMA) during the flush operation. The MFL generates and transfers transmit status words to the application for the number of frames flushed inside the MFL (including partial frames). Frames that completely flush in the MFL have the status bit for frame flush (TDES0 bit 13) set. The MFL completes the flush operation when the application (DMA) accepts all of the status words for the frames that flushed. The MFL then clears the transmit FIFO flush control register bit. The MFL starts accepting new frames from the application (DMA).

## FIFO Layer Receive Path

The receive controller operates in the following sequence:

1. When the EMAC CORE receives a frame, it pushes in data with the frame start and end indicators. The MFL accepts the data and pushes it into the FIFO.
2. The receive controller takes the data out of the FIFO and sends it to the DMA.
- Threshold mode (default). This mode is configured using EMAC\_DMA0\_OPMODE.RTC . When the FIFO receives 64 bytes or a full packet of data, the receive controller pops out the data and indicates its availability to the DMA. Some error frames cannot be dropped, because the error status is received at the end-of-frame. By this time, the start of that frame has already been read out of the FIFO.
- Rx FIFO Store-and-Forward mode. This mode is configured using EMAC\_DMA0\_OPMODE.RSF . A frame is read out only after being written completely into the receive FIFO. In this mode, all error frames are dropped such that only valid frames are read out and forwarded to the application. Error frames are dropped when the EMAC CORE is configured for this feature.
3. After the end-of-frame transfers, the status word from the EMAC CORE is also the pushed FIFO. When the status of a partial frame due to overflow is given out, the frame length field in the status word is not valid.

## Receive FIFO Multi-Frame Handling

Since the status is available immediately following the data, the MFL stores any number of frames into the FIFO, as long as it is not full.

## Receive FIFO Error Handling

If the MFL Rx FIFO is full before it receives the end of frame data from the EMAC, the DMA declares an overflow condition. The whole frame (including the status word) drops. The overflow counter in the DMA (overflow counter-register) increments. This activity occurs even if the EMAC\_DMA0\_OPMODE.FEF bit is set. If the start address of such a frame has already transferred, the rest of the frame drops. A dummy end of frame is written to the FIFO along with the status word. The status indicates a partial frame due to overflow. In such frames, the frame length field is invalid.

The MFL receive control logic can filter error and undersized frames using the EMAC\_DMA0\_OPMODE.FEF and EMAC\_DMA0\_OPMODE.FUF bits. If the start address of the frame has already transferred to the Rx FIFO read controller, that frame is not filtered. The start address of the frame transfers to the read controller after the frame crosses the receive threshold (set by the EMAC\_DMA0\_OPMODE.RTC bits).

If the MFL receive FIFO is configured for store-and-forward mode, it can filter and drop all error frames.

## EMAC CORE

The EMAC CORE is the lowest block in the EMAC peripheral and it performs all operations with the external world (PHY chip). It has independent transmit and receive modules. The modules interact with the EMAC FIFO layer at one end and interacts with the PHY chip through the RMII interface at the other end. Both modules have several subblocks which are discussed in subsequent sections.

Transmission is initiated when the MFL (FIFO layer) pushes in data with the start of frame. The CORE then transmits to the reduced media-independent interface. After the end of frame transfers out, the CORE gives the status of the transmission back to the MFL. The MFL forwards the transmission to the application through DMA.

A receive operation initiates when the EMAC detects an SFD on the RMII/RGMII. The CORE strips the preamble and SFD before proceeding to process the frame. The header fields are checked for the filtering and the FCS field used to verify the CRC for the frame. The frame drops in the core when it fails the address filter.

NOTE: The term CORE (written in capitals) refers to the internal block of Ethernet peripheral. Do not confuse the term with the processor core .

Table 29-29: EMAC CORE Related Registers

| Register Name        | Description                                                                                                                                                                                                                                     |
|----------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| MAC Configuration *1 | Establishes receive and transmit operating modes including: • Watchdog, Jabber, and Jumbo frame sizes • Inter Frame Gap • Speed Control - 10/100/1000 Mbps • Full or Half Duplex • Loopback Mode • Checksum Offload • Enabling Tx or Rx Engines |

Table 29-29: EMAC CORE Related Registers (Continued)

| Register Name           | Description                                                                                                                                                                                                                                                                                                                                                                                               |
|-------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| MAC Frame Filter        | Contains the filter controls for receiving frames. Some of the controls from this register go to the address check block of the MAC, which performs the first level of address filtering. The second level of filtering is performed on the incoming frame, based on other controls such as pass bad frames and pass control frames.                                                                      |
| Hash Table High/Low 1   | A 64-bit hash table is used for group address filtering. For hash filtering, the contents of the destination address in the incoming frame passes through the CRC logic. The upper 6 bits of the CRC register index the contents of the hash table.                                                                                                                                                       |
| SMI Address 1           | Controls the management cycles to the external PHY through the Station Management inter- face. The register also includes a field to program the frequency of MDC.                                                                                                                                                                                                                                        |
| SMI Data 1              | Stores write data for the PHY register at the address specified in SMI Address register. This register also stores read data from the PHY register at the address specified by SMI address reg- ister.                                                                                                                                                                                                    |
| Flow Control 1          | Controls the generation and reception of the control (pause command) frames by the flow control module of the EMAC. The fields of the control frame are selected as specified in the 802.3x specification. The EMAC uses the pause time value from this register in the pause time field of the control frame. The host must make sure that the activate bit is cleared before writ- ing to the register. |
| VLAN Tag 1              | Contains the IEEE 802.1Q VLAN tag to identify the VLAN frames. The MAC compares the 13th and 14th bytes of the receiving frame (length or type) with 16.h8100. The following 2 bytes are compared with the VLAN tag. If a match occurs, it sets the received VLAN bit in the receive frame status. The legal length of the frame increases from 1518 bytes to 1522 bytes.                                 |
| Debug                   | Provides the status of all main modules of the transmit and receive datapaths and the FIFOs. An all-zero status indicates that the MAC core is in idle state (and FIFOs are empty) and no activity exists in the datapaths.                                                                                                                                                                               |
| Interrupt Status        | The contents of this register identify the events in the EMAC-CORE that can generateMMC and PTP-related interrupts.                                                                                                                                                                                                                                                                                       |
| Interrupt Mask          | Enables the program to mask the interrupt signal because of the corresponding PTP event in the interrupt status register.                                                                                                                                                                                                                                                                                 |
| MAC Address0 High/Low 1 | Holds the upper or lower 16 bits of the MAC address of the station. The first DA byte that is received on the RMII interface corresponds to the LS byte (bits [7:0]) of the MAC address low register. For example, if 0x112233445566 is received (0x11 is the first byte) on the RMII as the destination address, then the macaddress0 register [47:0] is compared with 0x665544332211.                   |
| Operation Mode 1        |                                                                                                                                                                                                                                                                                                                                                                                                           |

- *1 There must not be any further writes to these registers until the first write updates. Otherwise, the second write operation is not updated properly. For correct operation, the delay between two writes to the same register location must be at least 8 cycles of 50MHz RMII REFCLK.

NOTE: Refer to the 'Register Description' section for the detailed bit-level explanation of the registers.

## EMAC CORE Transmission Engine

The following modules constitute the transmission function (transmission engine components) of the EMAC:

- Transmit Bus Interface Module (TBU)
- Transmit Frame Controller Module (TFC)
- Transmit Checksum Offload Engine (TCOE)
- Transmit Protocol Engine Module (TPE)
- Transmit Scheduler Module (STX)
- Transmit CRC Generator Module (CTX)
- Transmit Flow Control Module (FTX)

## Transmit Bus Interface Module (TBU)

This module interfaces the transmit path of the EMAC CORE with the MAC Layer FIFO interface. This module outputs the transmit status to the application at the end of normal transmission or collision.

## Transmit Frame Controller Module (TFC)

The transmit frame controller regulates frames as well as converts the 32-bit input data into an 8-bit stream.

When the number of bytes received from the application falls below 60 (DA+SA+LT+DATA), the state machine automatically appends zeros to the transmitting frame. The state machine makes the data length exactly 46 bytes to meet the requirement for minimum data field of IEEE 802.3. The EMAC module can also be programmed to not append any padding.

The frame controller receives the computed CRC and appends it as the FCS field to the data transmitting out. When the EMAC is programmed to not append the CRC value to the end of Ethernet frames, the TFC module ignores the computed CRC. However, when the EMAC is programmed to append pads for frames (DA+SA+LT +DATA) less than 60 bytes, then the CRC is always appended at the end of padded frame.

## Transmit Checksum Offload Engine (TCOE)

Communication protocols such as TCP and UDP implement checksum fields, which help determine the integrity of data transmitted over a network. The most widespread use of Ethernet is to encapsulate TCP and UDP over IP datagrams. Therefore, the EMAC has a checksum offload engine (COE) to support checksum calculation and insertion in the transmit path, and error detection in the receive path.

- NOTE: The checksum for TCP , UDP , or ICMP is calculated over a complete frame, and then inserted into its corresponding header field. Because of this requirement, this function is enabled only when the transmit FIFO configuration is for store-and-forward mode. (The EMAC\_DMA0\_OPMODE.TSF bit is set.) If the MAC configuration is for threshold (cut-through) mode, the transmit COE is bypassed.
- NOTE: Programs must make sure that the transmit FIFO is deep enough to store a complete frame before that frame transfers to the EMAC CORE transmitter. The program must enable the checksum insertion only in the frames that are less than the following number of bytes in size (even in the store-and-forward mode):

FIFO depth - PBL - 3 FIFO locations, where PBL is the programmed burst-length in the DMA bus mode register.

## IP Header Checksum

In IPv4 datagrams, the 16-bit header checksum field indicates the integrity of the header fields (bytes 11 and 12 of the IPv4 datagram). The COE detects an IPv4 datagram when the Ethernet type field of the frame has the value 0x0800 and the version field of the IP datagram has the value 0x4. The checksum field of the input frame is ignored during calculation and replaced with the calculated value.

The IP header error status bit in transmit descriptor word TDES0 indicates the result of this IP header checksum calculation. The status bit is set whenever the values of the Ethernet type field and the IP header version field are not consistent. Or, the status bit is set when the Ethernet frame does not have enough data, as indicated by the IP header length field. In other words, this bit is set when an IP header error is asserted under the following circumstances.

## For IPv4 datagrams:

- The received Ethernet type is 0x0800, but the version field of the IP header is not equal to 0x4.
- The IPv4 header length field indicates a value less than 0x5 (20 bytes).
- The total frame length is less than the value given in the IPv4 header length field.

## For IPv6 datagrams:

- The Ethernet type is 0x86dd but the IP header version field is not equal to 0x6.
- The frame ends before the IPv6 header (40 bytes) or extension header (as given in the corresponding header length field in an extension header) is received.

If the COE detects an IP header error, it still inserts an IPv4 header checksum if the Ethernet type field indicates an IPv4 payload.

NOTE: IPv6 headers do not have a checksum field. Therefore, the COE does not modify the IPv6 header fields. TCP/UDP/ICMP Checksum

The TCP/UDP/ICMP checksum engine processes the IPv4 or IPv6 header (including extension headers) and determines whether the encapsulated payload is TCP , UDP , or ICMP .

- NOTE: See IETF specifications RFC 791, RFC 793, RFC 768, RFC 792, RFC 2460, and RFC 4443 for IPv4, TCP, UDP , ICMP , IPv6, and ICMPv6 packet header specifications, respectively.
- NOTE: For non-TCP/UDP/ICMP/ICMPv6 payloads, this checksum engine is bypassed and nothing further is modified in the frame.
- NOTE: For ICMP-over-IPv4 packets, the checksum field in the ICMP packet must always be 0x0000 in both modes, because pseudo-headers are not defined for such packets. If it does not equal 0x0000, an incorrect checksum can be inserted into the packet.

- NOTE: This engine does not process fragmented IP frames (IPv4 or IPv6), IP frames with security features (such as an encapsulated security payload), and IPv6 frames with routing headers. The checksum engine bypasses the checksum insertion for such frames even if the checksum insertion is enabled.

The checksum is calculated for the TCP , UDP , or ICMP payload and inserted into its corresponding field in the header. This engine can work in the following two ways.

- The TCP , UDP , or ICMPv6 pseudo-header is not included in the checksum calculation and is assumed to be present in the checksum field of the input frame. This engine includes the checksum field in the checksum calculation, and then replaces the checksum field with the final calculated checksum.
- The engine ignores the checksum field, includes the TCP , UDP , or ICMPv6 pseudo-header data into the checksum calculation, and overwrites the checksum field with the final calculated value.

The status bit for the payload checksum error in the transmit descriptor word TDES0 indicates the result of this operation. The checksum engine sets the status bit for the payload checksum error when:

- The checksum engine detects that the frame has been forwarded to the MAC transmitter engine in the storeand-forward mode, and
- The end of frame (EOF) has not been written to the FIFO, or
- The packet ends before the number of bytes indicated by the payload length field in the IP header is received.

When the packet is longer than the indicated payload length, the COE ignores them as stuff bytes, and no error is reported. When the engine detects the first type of error, it does not modify the TCP , UDP , or ICMP header. For the second error type, it still inserts the calculated checksum into the corresponding header field.

Transmit checksum offloading is enabled by setting the CIC bits [23:22] of TDES0 word in the transmit descriptor.

## Transmit Protocol Engine Module (TPE)

The transmit protocol engine consists of a state-machine that controls the protocol-level operation of Ethernet frame transmission. The module performs the following functions to meet the IEEE 802.3 specifications.

- Generates preamble and SFD
- Generates carrier extension in half-duplex mode (only in RGMII mode)
- Supports frame bursting in half-duplex mode (only in RGMII mode)
- Generates jam pattern in half-duplex mode
- Contains time stamp snapshot logic for IEEE 1588 support
- Jabber timeout
- Flow control for half-duplex mode (back pressure)
- Generates transmit frame status

When a new frame transmission is requested, the protocol engine sends out the preamble and SFD, followed by the data received. The preamble is defined as 7 bytes of 10101010 pattern. The SFD is defined as 1 byte of 10101011 pattern.

The collision window is defined as 1 slot time (512-bit times for 10/100 Mbps and 4096 bit times for 1000 Mbps). The jam pattern generation is applicable only to half-duplex mode, not to full-duplex mode. A collision can occur any time from the beginning of the frame to the end of the CRC field. When a collision happens, the state machine sends a 32-bit jam pattern of 0x55555555 on the RMII/RGMII. The pattern informs all other stations that a collision has occurred. If the collision happens during the preamble transmission phase, it completes the transmission of preamble and SFD and then sends the jam pattern. If the collision occurs after the collision window and before the end of the FCS field, it sends a 32-bit jam pattern. It also sets the late collision bit in the transmit frame status.

In RGMII half-duplex mode (1,000 Mbps), the transmit state machine ensures that all valid carrier events exceed a slot time of 4,096 bit times. To accomplish this, any transmit frame shorter than 512 bytes from the TFC module is extended using a carrier extension. This is signaled to the PHY using RGMII\_TXCTL pin and sending 0x00 through RGMII\_TXD.

The module maintains a jabber timer to cut off the transmission of Ethernet frames when the TFC module transfers more than 2,048 (default) bytes. The timeout changes to 10,240 bytes when the jumbo frame is enabled.

The transmit state machine uses the deferral mechanism for the flow control (back pressure) in half-duplex mode. When the application asks to stop receiving frames, the module sends a jam pattern of 32 bytes. It sends the pattern whenever it senses a reception of a frame. T ransmit flow control must be enabled. This activity results in a collision and the remote station backs off.

The application can request a flow control signal by setting the EMAC\_FLOWCTL.FCBBPA bit. If the application requests a frame transmission, then it is scheduled and transmitted even when the back pressure is activated. If the back pressure is activated for a long time, then the remote stations abort their transmissions due to excessive collisions. (For example, a long time is when more than 16 consecutive collision events occur.)

If PTP time stamping is enabled for the transmit frame, this block takes a snapshot of the PTP system time when the SFD is put onto the transmit bus.

## Transmit Scheduler Module (STX)

The transmit scheduler is responsible for scheduling the frame transmission on the RMII RGMII/MII. The two major functions of this module are:

- Maintain the inter-frame gap between two transmitted frames.
- Follow the truncated binary exponential back-off algorithm for half-duplex mode.

The scheduler maintains an idle period of the configured inter-frame gap ( EMAC\_MACCFG.IFG bits) between any two transmitted frames. The scheduler starts its IFG counter when the carrier signal of the reduced media-independent interface goes inactive. In half-duplex mode and when IFG is configured for 96-bit times, the scheduler follows the rule of deference specified in Section 4.2.3.2.1 of the IEEE 802.3 specification. The module resets its IFG counter when a carrier is detected during the first two-thirds (64-bit times for all IFG values) of the IFG interval. If the

carrier is detected during the final one third of the IFG interval, the scheduler continues the IFG count and enables the transmitter after the IFG interval.

## Transmit CRC Generator Module (CTX)

The transmit CRC generator module generates the CRC for the FCS field of the Ethernet frame (DA + SA + LT + DATA + PAD).

This module calculates the 32-bit CRC for the FCS field of the Ethernet frame. The following polynomial defines the encoding:

<!-- formula-not-decoded -->

## Transmit Flow Control Module (FTX)

The transmit flow control module generates pause frames and transmits them to the frame controller as necessary, in full-duplex mode. The application can request the flow control module to send a pause frame by setting the EMAC\_FLOWCTL.FCBBPA bit.

If the application has requested flow control, the flow control module generates and transmits a single pause frame. The value of the pause time in the generated frame contains the programmed pause time value configured using the EMAC\_FLOWCTL.PT bit. The module can extend the pause or end the pause prior to the time specified in the previously transmitted pause frame. To change the pause, the application must request another pause frame transmission after programming the EMAC\_FLOWCTL.PT bit with an appropriate value.

If the flow control signal goes inactive prior to the sampling time, the flow control module transmits a pause frame with zero pause time. This event indicates to the remote end that the receive buffer is ready to receive new data frames.

## Source Address, VLAN, and CRC Insertion, Replacement, or Deletion

The MAC supports the following functions for transmit frames:

- Source address insertion or replacement
- VLAN insertion, replacement, or deletion
- CRC replacement

## Source Address Insertion or Replacement

The software can use the SA insertion or replacement feature to instruct the MAC to do the following for transmit frames:

- Insert the content of the MAC address registers in the SA field.
- Replace the content of the SA field with the content of the MAC address registers.

The software can enable the SA insertion or replacement feature for all transmit frames or selective frames.

- To enable SA insertion or replacement feature for all frames, program bits[30:28] of the MAC Configuration register.

- To enable the SA insertion or replacement feature for selective frames, program the SA insertion control field (TDES1 Bits [31:29]) in the first transmit descriptor of the frame. When bit 31 of TDES1 is set, the SA insertion control field indicates insertion or replacement by MAC address1 registers. When bit 31 of TDES1 is reset, it indicates insertion or replacement by MAC address 0 registers. If MAC address1 registers are not enabled, then EMAC uses MAC address0 registers for insertion or replacement. The choice is not based on the value of the most significant bit of the SA insertion control field.

When SA insertion is enabled, the application ensures that the frames sent to the MAC do not have the SA field. This functionality is because the MAC does not check the presence of SA field in the transmit frame. MAC inserts the content of the MAC address registers in the SA field. Similarly, when SA replacement is enabled, the application ensures that frames sent to the MAC have the SA field. The MAC replaces the 6 bytes, following the destination address field in the transmit frame, with the content of the MAC address registers.

## VLAN Insertion, Replacement, or Deletion

The software can use the VLAN insertion, replacement, or deletion feature to instruct the MAC to do the following for transmit frames:

- Insert or replace the VLAN type field (C-VLAN or S-VLAN indicated by bit 19 (CSVL) of VLAN tag inclusion or replacement register and VLAN tag field in the transmit frame with bit [15:0], VLT, of VLAN tag inclusion or replacement register.
- Delete the VLAN type and VLAN tag fields in transmit frame.

The software can enable the VLAN insertion, replacement, or deletion feature for all transmit frames or selective frames. To enable this function for all transmit frames, program Bits[17:16] of the VLAN Tag Inclusion or Replacement register. To enable this function for selective, program the VLAN insertion control field (TDES0 Bits [19:18]) in the first transmit descriptor of the frame. When VLAN replacement or deletion is enabled, the MAC checks the presence of the VLAN type field (0x8100 or 0x88a8), after the Destination Address (DA) and SA fields, in the transmit frame. The replace or delete operation does not occur when the VLAN type field is not detected in the 2 bytes following the DA and SA fields. However, when VLAN insertion is enabled, the MAC does not check the presence of VLAN type field in the transmit frame. MAC inserts the VLAN type and VLAN tag fields.

## CRC Replacement

The software can use the CRC replacement feature to instruct the MAC to replace the FCS field in the transmit frame with the CRC computed by the MAC. This feature works on per-frame basis. To enable the CRC replacement feature, program the CRC replacement control field (bit 24 of transmit descriptor word 0 (TDES0)) in the first transmit descriptor of the frame.

- NOTE: This feature is valid only when disable CRC control (bit 27 in TDES0) is enabled. The software provides the FCS field in the transmit frame. If SA or VLAN insertion control is enabled, the MAC appends or replaces the FCS field with the computed CRC when Disable CRC Control is enabled or disabled, respectively.

The CRC Replacement table shows how CRC replacement is performed based on the values of bit 27 (DC) and bit 24 (CRCR) of transmit descriptor word 0 (TDES0).

Table 29-30: CRC Replacement

|   DC | CRCR   | Description                                                                                   |
|------|--------|-----------------------------------------------------------------------------------------------|
|    0 | x      | Append CRC. When DC = 0, the MAC appends the computed CRC irre- spective of the CRCR setting. |
|    1 |        | Replace CRC                                                                                   |
|    1 | 0      | No operation (User has appended the CRC)                                                      |

## Source Address Filtering

The Source Address Filtering table provides filtering possibilities for the source address using the EMAC AFM module. The MAC receive frame filter register ( EMAC\_MACFRMFILT ) contains these bits.

Table 29-31: Source Address Filtering

|            | SA Filter Operation   | SA Filter Operation   | SA Filter Operation   | Result                                                               |
|------------|-----------------------|-----------------------|-----------------------|----------------------------------------------------------------------|
| Frame Type | PR                    | SAIF                  | SAF                   | Result                                                               |
| Unicast    | 1                     | X                     | X                     | Pass all frames                                                      |
|            | 0                     | 0                     | 0                     | Pass on perfect or group filter match but do not drop failing frames |
|            | 0                     | 1                     | 0                     | Fail on perfect or group filter match but do not drop frame          |
|            | 0                     | 0                     | 1                     | Pass on perfect or group filter match and drop failing frames        |
|            | 0                     | 1                     | 1                     | Fail on perfect or group filter match and drop failing frames        |

## EMAC CORE Reception Engine

The following are the functional blocks (reception engine components) in the receive path of the EMAC core.

- Receive Protocol Engine Module (RPE)
- Receive CRC Module (CRX)
- Receive Frame Controller Module (RFC)
- Receive Checksum Offload Engine (RCOE)
- Receive Bus Interface Unit Module (RBU)
- Address Filtering Module (AFM)

## Receive Protocol Engine Module (RPE)

The receive protocol engine is a state-machine that strips the preamble, SFD, and carrier extension (in 1000 Mbps half-duplex mode) of the received frame. Once the receive data valid signal ( ETH0\_CRS ) signal of RMII or RXCTL signal of RGMII becomes active, the protocol engine begins hunting for the SFD field from the receive modifier

logic. Until then, the state machine drops the receiving preambles. Once the SFD is detected, it begins sending the data of the Ethernet frame to the frame controller, beginning with the first byte following the SFD (destination address).

- NOTE: According to the IEEE 802.3 Ethernet specifications, the EMAC receiver does not need to look or check for the preamble pattern. It has to wait only for the SFD pattern to identify the start of a frame. Then the EMAC receiver accepts a frame even when no preamble is received before the SFD pattern.

If PTP time stamping is enabled, the RPE takes a snapshot of the PTP system time when detecting any SFD of the frame on the reduced media-independent interface. Unless the MAC filters out and drops the frame, this time stamp passes on to the application

The protocol engine also decodes the length or type field of the receiving Ethernet frame. The state machine sends the data of the frame up to the count specified in the length or type field if these conditions are met:

- The length or type field is less than 0x600
- The MAC is programmed for the auto CRC or PAD stripping option

It then starts dropping bytes (including the FCS field).

If the length or type field is greater than or equal to 0x600, the protocol engine sends all received Ethernet frame data to the frame controller. The transfer does not depend on the value of the programmed auto-CRC strip option.

The EMAC is programmed with the watchdog timer enabled (default setting). In this configuration, frames above 2,048 (10,240 if jumbo frame is enabled) bytes (DA + SA + LT + DATA + PAD + FCS) are cut off at the protocol engine. Set the EMAC\_MACCFG.WD bit to disable this feature. However, even when the watchdog timer is disabled, frames greater than 16 KB are cut off and a watchdog timeout status is issued.

The EMAC supports loopback of transmitted frames onto its receiver. By default, the EMAC loopback function is disabled. Set the EMAC\_MACCFG.LM bit to enable the function.

At the end of every received frame, the protocol engine generates received frame status and sends it to the frame controller. Control, missed frame, and filter fail status are added to the receive status in the frame controller.

## Receive CRC Module (CRX)

The receive CRC module checks for any CRC errors in the receiving frame.

This module calculates the 32-bit CRC for the received frame that includes the destination address field through the FCS field (DA+SA+LT+DATA+PAD+FCS). The following generating polynomial defines the encoding.

<!-- formula-not-decoded -->

Irrespective of the auto pad or CRC strip, the CRC module receives the entire frame to compute the CRC check for received frame.

## Receive Frame Controller Module (RFC)

The main functions of the frame controller are:

- Converting the 8-bit stream data to 32-bit data

- Frame filtering
- Attaching the calculated IP checksum
- Updating the receive status

If the EMAC\_MACFRMFILT.RA bit is set, the RFC module initiates the data transfer as soon as possible. At the end of the data transfer, the frame controller sends out the received frame status that includes the address filtering pass or fail status.

If the EMAC\_MACFRMFILT.RA bit is reset, the frame controller performs frame filtering based on the destination or source address. (The application still must perform another level of filtering if it decides not to receive any bad frames like runt, CRC error frames, for example.) After receiving the destination or source address bytes, the frame controller checks the filter-fail signal from the AFM module for an address match. On detecting a filter-fail from AFM, the frame is dropped and not transferred to the application.

## Receive Flow Control Module (FRX)

The receive flow controller detects the receiving pause frame and pauses the frame transmission for the delay specified within the received pause frame. The flow controller is enabled only in full-duplex mode. The EMAC uses the EMAC\_FLOWCTL.RFE bit to enable or disable the function for pause frame detection.

Once the receive flow control is enabled, the flow controller begins monitoring the received frame destination address for any match with the multicast address of the control frame (0x0180C2000001). If a match is detected, it indicates to the frame controller, that the destination address of the received frame matches the reserved control frame destination address. The RFC module then decides whether to transfer the received control frame to the application, based on the EMAC\_MACFRMFILT.PCF bit setting.

The receive flow controller also decodes the type, opcode, and pause timer field of the receiving control frame. The flow controller requests the MAC transmitter pause the transmission of any data frame.

- If the byte count of the frame status indicates 64 bytes, and
- If there is no CRC error

The transmission is paused for the decoded pause time value, multiplied by the slot time (64-byte times). Meanwhile, if another pause frame is detected with a zero pause time value, the module resets the pause time and gives another pause request to the transmitter. The module does not generate a pause request to the transmitter:

- If the received control frame does not match the type field (0x8808), opcode (0x00001), or byte length (64 bytes), or
- If there is a CRC error

For a pause frame with a multicast destination address, the frame controller filters the frame based on the address match from the flow controller. For a pause frame with a unicast destination address, the filtering in the FRX module depends on:

- If the destination address matched the contents of the MAC address register 0 ( EMAC\_ADDR0\_HI or EMAC\_ADDR0\_LO ), and

- If the EMAC\_FLOWCTL.UP bit is set

The module detects a pause frame even with a unicast destination address. The EMAC uses the EMAC\_MACFRMFILT.PCF bits to control the filtering for control frames in addition to the address filter module.

## Receive Checksum Offload Engine (RCOE)

When checksum offloading is enabled, both IPv4 and IPv6 frames in the received Ethernet frames are detected and processed for data integrity. Programs can enable this module by setting the EMAC\_MACCFG.IPC bit. The EMAC receiver identifies IPv4 or IPv6 frames by checking for value 0x0800 or 0x86DD, respectively, in the received Ethernet type field of frames. This identification applies to VLAN-tagged frames as well. Extended descriptor mode (8 x32bit words) must be enabled to get the IPC checksum engine status in RDES4. To check status, poll bit 0 of RDES0 word of receive descriptor. Then, if this bit is set, parse bits [7:0] of RDES4 word.

The receive checksum offload engine calculates IPv4 header checksums and checks if they match the received IPv4 header checksums. The IP header error bit is set for any mismatch between the indicated payload type (Ethernet type field) and the IP header version. The IP header error bit is also set when the received frame does not have enough bytes, as indicated by the length field of the IPv4 header. (The bit is set when fewer than 20 bytes are available in an IPv4 or IPv6 header).

This engine also identifies a TCP , UDP , or ICMP payload in the received IP datagrams (IPv4 or IPv6). The engine calculates the checksum of such payloads properly, as defined in the TCP , UDP , or ICMP specifications. This engine includes the TCP/UDP/ICMPv6 pseudo-header bytes for checksum calculation and checks whether the received checksum field matches the calculated value. The result of this operation appears as a payload checksum error bit in the receive status word. This status bit is also set if the length of the TCP , UDP , or ICMP payload does not tally to the expected payload length given in the IP header.

NOTE: The COE engine bypasses the payload of fragmented IP datagrams, IP datagrams with security features, IPv6 routing headers, and payloads other than TCP , UDP , or ICMP . This information is given in the receive status(whether the checksum engine is bypassed or not).

The Checksum Error Status table shows bit combination in receive descriptors (frame status with full checksum offload engine enabled and advanced timestamps not enabled).

Table 29-32: Checksum Error Status

|   IEEE802.3 Frame: bit 5 of RDES0 |   Header Checksum Error (HCE): bit 3 of RDES4 |   Payload Checksum Er- ror (PCE): bit 4 of RDES4 | Frame Status                                                                                |
|-----------------------------------|-----------------------------------------------|--------------------------------------------------|---------------------------------------------------------------------------------------------|
|                                 0 |                                             0 |                                                0 | The frame is an IEEE 802.3 frame (length field value is less than 0x0600).                  |
|                                 1 |                                             0 |                                                0 | IPv4/IPv6 type frame in which no checksum error is de- tected.                              |
|                                 1 |                                             0 |                                                1 | IPv4/IPv6 type frame in which a payload checksum er- ror (as described for PCE) is detected |

Table 29-32: Checksum Error Status (Continued)

|   IEEE802.3 Frame: bit 5 of RDES0 |   Header Checksum Error (HCE): bit 3 of RDES4 |   Payload Checksum Er- ror (PCE): bit 4 of RDES4 | Frame Status                                                                                                     |
|-----------------------------------|-----------------------------------------------|--------------------------------------------------|------------------------------------------------------------------------------------------------------------------|
|                                 1 |                                             1 |                                                0 | IPv4/IPv6 type frame in which IP header checksum er- ror (as described for IPC HCE) is detected.                 |
|                                 1 |                                             1 |                                                1 | IPv4/IPv6 type frame in which both PCE and IPC HCE is detected.                                                  |
|                                 0 |                                             0 |                                                1 | IPv4/IPv6 type frame in which there is no IP HCE and the payload check is bypassed due to unsupported pay- load. |
|                                 0 |                                             1 |                                                1 | Type frame which is neither IPv4 or IPv6 (COE bypass- es the checksum check completely)                          |
|                                 0 |                                             1 |                                                0 | Reserved                                                                                                         |

## Receive Bus Interface Unit Module (RBU)

The receive bus interface unit (RBU) constructs the 32-bit data received from the frame controller into a 32-bit FIFO-based protocol.

## Address Filtering Module (AFM)

The address filtering (AFM) module performs the destination checking function on all received frames and reports the address filtering status to the frame controller. The address checking is based on different parameters (frame filter register, EMAC\_MACFRMFILT ) chosen by the application. These parameters are inputs to the AFM module as control signals. The AFM module reports the status of the address filtering based on the combination of these inputs. The AFM module also reports whether the receiving frame is a multicast frame or a broadcast frame, as well as the address filter status. The AFM module uses the physical (MAC) address of the station and the multicast hash table for address checking.

- Hash or Perfect Address Filter. The destination address filter can be configured to pass a frame when its destination address matches either the hash filter or the perfect filter. Set the EMAC\_MACFRMFILT.HPF bit, the corresponding EMAC\_MACFRMFILT.HUC , or EMAC\_MACFRMFILT.HMC bits. This configuration applies to both unicast and multicast frames. If the EMAC\_MACFRMFILT.HPF bit is reset, only one of the filters (hash or perfect) is applied to the received frame.

NOTE: Hash filtering is not perfect filtering because a 48-bit MAC address is reduced to a 6-bit hash value. So, there can be instances where more than one address has the same hash value.

- Unicast Destination Address Filter.
- The AFM supports one MAC address two MAC addresses for unicast perfect filtering. If perfect filtering is selected, the AFM compares all 48 bits of the received unicast address with the programmed MAC address for any match. (The EMAC\_MACFRMFILT.HUC bit is reset for perfect filtering).

- In hash filtering mode, the AFM performs imperfect filtering for unicast addresses using a 64-bit hash table. (The EMAC\_MACFRMFILT.HUC bit is set in hash filtering mode.) For hash filtering, the AFM uses the upper 6-bit CRC of the received destination address to index the content of the hash table. A value of 000000 selects bit 0 of the selected register, and a value of 111111 selects bit 63 of the hash table register. If the corresponding bit (indicated by the 6-bit CRC) is set to 1, the unicast frame has passed the hash filter. Otherwise, the frame has failed the hash filter.

## · Multicast Destination Address Filter.

- Program the EMAC to pass all multicast frames by setting the EMAC\_MACFRMFILT.PM bit. If the EMAC\_MACFRMFILT.PM bit is reset, the AFM filters multicast addresses based on the EMAC\_MACFRMFILT.HMC bit. In perfect filtering mode, the multicast address is compared with the programmed MAC destination address register. The EMAC also supports group address filtering.
- In hash filtering mode, the AFM performs imperfect filtering using a 64-bit hash table. For hash filtering, the AFM uses the upper 6-bit CRC of the received multicast address to index the content of the hash table. A value of 000000 selects bit 0 of the selected register and a value of 111111 selects bit 63 of the hash table register. If the corresponding bit is set to 1, then the multicast frame has passed the hash filter. Otherwise, the frame has failed the hash filter.
- Broadcast Address Filter. The AFM does not filter any broadcast frames in the default mode. However, if the EMAC is programmed to reject all broadcast frames by setting the EMAC\_MACFRMFILT.DBF bit, the AFM asserts the filter fail signal, whenever a broadcast frame is received.
- Unicast Source Addrress Filter. EMAC can also perform a perfect filtering, based on the source address field of the received frames. By default, the AFM compares the SA field with the values programmed in the SA registers. The MAC address register 1 can be configured to contain SA instead of DA for comparison, by setting bit 30 of the corresponding register. Group filtering with SA is also supported. User can filter a group of addresses by masking one or more bytes of the address. The frames that fail the SA filter are dropped by the MAC if the EMAC\_MACFRMFILT.SAF bit is set. When the bit is set, the result of SA filter and DA filter is AND'ed to decide whether the frame needs to be forwarded. This means that either of the filter fail result drops the frame and both filters have to pass in order to forward the frame to the application.
- Inverse Filtering Operation. There is an option to invert the filter-match result at the final output. The EMAC uses the EMAC\_MACFRMFILT.DAIF bit to control this operation. The function of this bit applies to both unicast and multicast DA frames. The result of the unicast or multicast destination address filter is inverted in this mode.
- Inverse Filtering Operation. For both destination and source address filtering, there is an option to invert the filter-match result at the final output. This is controlled by the EMAC\_MACFRMFILT.DAIF or EMAC\_MACFRMFILT.SAIF bits. The EMAC\_MACFRMFILT.DAIF bit is applicable for both unicast and multicast DA frames. The result of the unicast /multicast destination address filter is inverted in this mode. Similarly, when the EMAC\_MACFRMFILT.SAIF bit is set, the result of unicast source address filter is reversed.

- Inverse Filtering Operation. There is an option to invert the filter-match result at the final output. The EMAC uses the EMAC\_MACFRMFILT.DAIF bit to control this operation. The function of this bit applies to both unicast and multicast DA frames. The result of the unicast or multicast destination address filter is inverted in this mode.

## Destination Address Filtering

The Destination Address Filtering table provides filtering possibilities for the destination address using the EMAC AFM module. The MAC receive frame filter register ( EMAC\_MACFRMFILT ) contains these bits.

Table 29-33: Destination Address Filtering

| Frame     | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | DA Filter Operation                                                                     |
|-----------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-----------------------------------------------------------------------------------------|
| Frame     | PR                                                    | HPF                                                   | HUC                                                   | HMC                                                   | DAIF                                                  | PM                                                    | DBF                                                   | DA Filter Operation                                                                     |
| Broadcast | 1                                                     | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | Pass                                                                                    |
| Broadcast | 0                                                     | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | 0                                                     | Pass                                                                                    |
| Broadcast | 0                                                     | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | 1                                                     | Fail                                                                                    |
| Unicast   | 1                                                     | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | Pass all frames                                                                         |
| Unicast   | 0                                                     | X                                                     | 0                                                     | X                                                     | 0                                                     | X                                                     | X                                                     | Pass on perfect or group filter match                                                   |
| Unicast   | 0                                                     | X                                                     | 0                                                     | X                                                     | 1                                                     | X                                                     | X                                                     | Fail on perfect or group filter match                                                   |
| Unicast   | 0                                                     | 0                                                     | 1                                                     | X                                                     | 0                                                     | X                                                     | X                                                     | Pass on hash filter match                                                               |
| Unicast   | 0                                                     | 0                                                     | 1                                                     | X                                                     | 1                                                     | X                                                     | X                                                     | Fail on hash filter match                                                               |
| Unicast   | 0                                                     | 1                                                     | 1                                                     | X                                                     | 0                                                     | X                                                     | X                                                     | Pass on hash or perfect or group filter match                                           |
| Unicast   | 0                                                     | 1                                                     | 1                                                     | X                                                     | 1                                                     | X                                                     | X                                                     | Fail on hash or perfect or group filter match                                           |
| Multicast | 1                                                     | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | Pass all frames                                                                         |
| Multicast | X                                                     | X                                                     | X                                                     | X                                                     | X                                                     | 1                                                     | X                                                     | Pass all frames                                                                         |
| Multicast | 0                                                     | X                                                     | X                                                     | 0                                                     | 0                                                     | 0                                                     | X                                                     | Pass on perfect or group filter match and drop PAUSE con- trol frames if PCF = 0x       |
| Multicast | 0                                                     | 0                                                     | X                                                     | 1                                                     | 0                                                     | 0                                                     | X                                                     | Pass on hash filter match and drop PAUSE control frames if PCF = 0x                     |
| Multicast | 0                                                     | 1                                                     | X                                                     | 1                                                     | 0                                                     | 0                                                     | X                                                     | Pass on hash or perfect or group filter match and drop PAUSE control frames if PCF = 0x |

Table 29-33: Destination Address Filtering (Continued)

|            | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   | Bit Setting (0 = Cleared, 1 = Set, X = Do-not-care)   |                                                                                         |
|------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-----------------------------------------------------------------------------------------|
| Frame Type | PR                                                    | HPF                                                   | HUC                                                   | HMC                                                   | DAIF                                                  | PM                                                    | DBF                                                   | DA Filter Operation                                                                     |
|            | 0                                                     | X                                                     | X                                                     | 0                                                     | 1                                                     | 0                                                     | X                                                     | Fail on perfect or group filter match and drop PAUSE control frames if PCF = 0x         |
|            | 0                                                     | 0                                                     | X                                                     | 1                                                     | 1                                                     | 0                                                     | X                                                     | Fail on hash filter match and drop PAUSE control frames if PCF = 0x                     |
|            | 0                                                     | 1                                                     | X                                                     | 1                                                     | 1                                                     | 0                                                     | X                                                     | Fail on hash or perfect or group filter match and drop PAUSE control frames if PCF = 0x |

## Source Address Filtering

The Source Address Filtering table provides filtering possibilities for the source address using the EMAC AFM module. The MAC receive frame filter register ( EMAC\_MACFRMFILT ) contains these bits.

Table 29-34: Source Address Filtering

|            | SA Filter Operation   | SA Filter Operation   | SA Filter Operation   | Result                                                               |
|------------|-----------------------|-----------------------|-----------------------|----------------------------------------------------------------------|
| Frame Type | PR                    | SAIF                  | SAF                   | Result                                                               |
| Unicast    | 1                     | X                     | X                     | Pass all frames                                                      |
|            | 0                     | 0                     | 0                     | Pass on perfect or group filter match but do not drop failing frames |
|            | 0                     | 1                     | 0                     | Fail on perfect or group filter match but do not drop frame          |
|            | 0                     | 0                     | 1                     | Pass on perfect or group filter match and drop failing frames        |
|            | 0                     | 1                     | 1                     | Fail on perfect or group filter match and drop failing frames        |

## VLAN Tag Based Filtering

EMAC0 supports VLAN tag perfect filtering and VLAN tag hash filtering.

## VLAN Tag Perfect Filtering

In VLAN tag perfect filtering, the MAC compares the VLAN tag of the received frame and provides the VLAN frame status to the application. Based on the programmed mode, the MAC compares the lower 12 bits or all 16 bits of the received VLAN tag to determine the perfect match. If VLAN tag perfect filtering is enabled, the MAC forwards the VLAN-tagged frames along with VLAN tag match status. It drops the VLAN frames that do not match. Inverse matching for VLAN frames can be enabled using the EMAC\_VLANTAG.VTIM bit. In addition, matching of S-VLAN tagged frames along with the default C-VLAN tagged frames can be enabled using EMAC\_VLANTAG.ESVL bit. The VLAN frame status bit (Bit 10 of RDES0) indicates the VLAN tag match status for the matched frames.

NOTE: The source or destination address (if enabled) has precedence over the VLAN tag filters. A frame which fails the source or destination address filter is dropped irrespective of the VLAN tag filter results. By default, the VLAN tag-based perfect filter is available in all configurations.

## VLAN Tag Hash Filtering

The MAC provides VLAN tag hash filtering with a 16-bit hash table. The MAC performs the VLAN hash matching based on the EMAC\_VLANTAG.VTHM bit setting. If the EMAC\_VLANTAG.VTHM bit is set, the most significant 4 bits of VLAN tag's CRC-32 are used to index the content of the VLAN hash table register ( EMAC\_VLAN\_HSHTBL ). A value of 1 in the EMAC\_VLAN\_HSHTBL register, corresponding to the index, indicates that the VLAN tag of the frame matched and the packet is forwarded. A value of 0 indicates that VLANtagged frame is dropped.

The MAC also supports the inverse matching of the VLAN frames. In the inverse matching mode, when the VLAN tag of a frame matches the perfect or hash filter, the packet is dropped. If the VLAN perfect and VLAN hash match are enabled, a frame matches if either the VLAN hash or the VLAN perfect filter matches. When inverse match is set, a packet is forwarded only when both perfect and hash filters indicate mismatch. The VLAN Matching and Final VLAN Match Status table shows the possibilities for VLAN matching and the final VLAN match status. When the EMAC\_MACFRMFILT.RA bit is set, all frames are received and the VLAN match status is indicated in bit 10 of receive descriptor word 0 (RDES0). When the EMAC\_MACFRMFILT.RA bit is not set and the EMAC\_MACFRMFILT.VTFE bit register is set, the frame is dropped when the final VLAN match status is fail.

When VLAN VID is programmed to 0 in the EMAC\_VLANTAG.VL bit field, all VLAN-tagged frames are perfect matches. But the status of the VLAN hash match depends on the VLAN hash enable bit and VLAN inverse filter bit. The VLAN Matching and Final VLAN Match Status table shows the possibilities for VLAN matching and the final VLAN match status.

Table 29-35: VLAN Matching and Final VLAN Match Status

| VID    | VLAN Perfect Filter Match Status (VPF)   | VLAN Hash Enable Bit   | VLAN Hash filter Match Status (VTHMS)   | VLAN Inverse Filter Bit (VTIM)   | Final VLAN Match Status   |
|--------|------------------------------------------|------------------------|-----------------------------------------|----------------------------------|---------------------------|
| VID=0  | Pass                                     | 0                      | X                                       | X                                | Pass                      |
| VID=0  | Pass                                     | 1                      | X                                       | 0                                | Pass                      |
| VID=0  | Pass                                     | 1                      | Fail                                    | 1                                | Pass                      |
| VID=0  | Pass                                     | 1                      | Pass                                    | 1                                | Fail                      |
| VID!=0 | Pass                                     | X                      | X                                       | 0                                | Pass                      |
| VID!=0 | Fail                                     | 0                      | X                                       | 0                                | Fail                      |
| VID!=0 | Fail                                     | 1                      | Fail                                    | 0                                | Fail                      |
| VID!=0 | Fail                                     | 1                      | Pass                                    | 0                                | Pass                      |
| VID!=0 | Fail                                     | 0                      | X                                       | 1                                | Pass                      |
| VID!=0 | Pass                                     | X                      | X                                       | 1                                | Fail                      |

Table 29-35: VLAN Matching and Final VLAN Match Status (Continued)

| VID   | VLAN Perfect Filter Match Status (VPF)   |   VLAN Hash Enable Bit | VLAN Hash filter Match Status (VTHMS)   |   VLAN Inverse Filter Bit (VTIM) | Final VLAN Match Status   |
|-------|------------------------------------------|------------------------|-----------------------------------------|----------------------------------|---------------------------|
|       | Fail                                     |                      1 | Pass                                    |                                1 | Fail                      |
|       | Fail                                     |                      1 | Fail                                    |                                1 | Pass                      |

## Layer 3 and Layer 4 Frame Filtering

The MAC supports layer 3 and layer 4 based frame filtering. The layer 3 filtering refers to the IP source or destination address filtering in the IPv4 or IPv6 frames whereas layer 4 filtering refers to the source or destination port number filtering in TCP or UDP .

When layer 3 and layer 4 filtering is enabled, the frames are filtered in the following way:

Matched Packets. The MAC forwards the packets that match all enabled fields to the application along with the status. The MAC gives the matched field status only if the EMAC\_MACCFG.IPC bit is set and one of the following conditions is true:

- All enabled layer 3 and layer 4 fields match
- At least one of the enabled field matches and other fields are bypassed or disabled

When multiple layer 3 and layer 4 filters are enabled, any filter match is considered as a match. If more than one filter matches, the MAC provides the status of the lowest filter where filter 0 is the lowest filter and filter 3 is the highest filter. For example, if filter 0 and filter 1 match, the MAC gives the status corresponding to filter 0.

NOTE: The source or destination address and VLAN tag filters (if enabled) have precedence over layer 3 and layer 4 filter. This means that a packet which fails the source or destination address, or VLAN tag filter is dropped irrespective of the layer 3 and layer 4 filter results.

Unmatched Packets. The MAC drops the packets that do not match any of the enabled fields. Programs can use the inverse match feature to block or drop a packet with specific TCP or UDP over IP fields and forward all other packets. When a packet is dropped, the aborted or partial packets can be dropped in the MTL Rx FIFO. If the Rx FIFO operates in the threshold (cut-through) mode and the threshold is programmed to a small value, such that packet transfer to application starts before the failed layer 3 and layer 4 filter results are available, the application may receive a partial packet with appropriate abort status.

Non-TCP or UDP IP Packets. By default, all non-TCP or UDP IP packets are bypassed from the layer 3 and layer 4 filters. The program can optionally program the MAC to drop all non-TCP or UDP over IP packets.

## Layer 3 Filtering

The EMAC supports perfect matching or inverse matching for IP source address and destination address. The matching compares all bits of the address except the specified lower mask bits.

For IPv6 packets filtering, the program can enable the last four data registers of a register set to contain the 128-bit IP source address or IP destination address. The IP source address or destination address should be programmed in

the order defined in the IPv6 specification, that is, the first byte of the IP source address or destination address in the received packet is in the higher byte of the register and the subsequent registers follow the same order.

For IPv4 packet filtering, the program can enable the second and third data registers of a register set to contain the 32-bit IP source address and IP destination address. The remaining two data registers are reserved. The IP source address or destination address should be programmed in the order defined in the IPv4 specification, that is, the first byte of IP source address and destination address in the received packet in the higher byte of the respective register.

## Layer 4 Filtering

The EMAC supports perfect matching or inverse matching for TCP or UDP source and destination port numbers. However, only one type (TCP or UDP) can be programmed at a time. The first data register contains the 16-bit source and destination port numbers of TCP or UDP , that is, the lower 16 bits for source port number and higher 16 bits for destination port number.

The TCP or UDP source and destination port numbers should be programmed in the order defined in the TCP or UDP specification, that is, the first byte of TCP or UDP source and destination port number in the received packet is in the higher byte of the register.

## Layer 3 and Layer 4 Filters Registers

The MAC implements a set of registers for layer 3 and layer 4 based frame filtering. In a register set, there is a control register, such as the EMAC\_L3L4\_CTL register (layer 3 and layer 4 control register), to control the frame filtering. In addition, there are five address registers to program the layer 3 and layer 4 fields to be matched, which are:

- EMAC\_L4\_ADDR (layer 4 address register)
- EMAC\_L3\_ADDR0 (layer 3 address 0 register)
- EMAC\_L3\_ADDR1 (layer 3 address 1 register)
- EMAC\_L3\_ADDR2 (layer 3 address 2 register)
- EMAC\_L3\_ADDR3 (layer 3 address 3 register)

## EMAC Station Management Interface (SMI)

The IEEE 802.3 MII station management interface, also known as the MDIO management interface, allows the processor to monitor and control one or more external Ethernet physical-layer transceivers. (Physical-layer transceivers are commonly called PHYs). The management interface physically consists of a 2-wire serial connection composed of the MDC (management data clock) output signal and the MDIO (management data input/output) bidirectional data signal. The IEEE 802.3 MII station management interface also applies to RMII.

The application can address only one register in the PHY in any given time and send control data or receive status information. All the transfers are initiated by the EMAC CORE, and the PHY chip only acts as a target device.

Standard PHY control and status registers typically provide

- Device capability status bits (for example: auto-negotiation, duplex modes, 10/100 speeds, and protocols)

- Device status bits (for example: auto-negotiation complete, link status, remote fault)
- Device control bits (for example: reset, speed selection, loop back, and auto-negotiation start)

Upon power-up, an MDIO read access (at default rates) of device capabilities in PHY status registers can determine the supported PHY features.

The MII management logical interface specifies:

- A set of 16-bit device control or status registers within the PHYs, including both required registers with standardized bit definitions as well as optional vendor-specified registers.
- A 5-bit device addressing scheme which allows the MAC to select one of up to 32 externally connected PHY devices.
- A 5-bit register addressing scheme for selecting the target register within the addressed device.
- A transfer frame protocol for 16-bit read and write accesses to PHY registers through the MDC and MDIO signals under control of the MAC.

Table 29-36: Station Management Interface pins

| Station Management Interface Pins   | Pin Description                                                                                         |
|-------------------------------------|---------------------------------------------------------------------------------------------------------|
| MDIO - Management Data I/O          | A periodic clock that runs at a maximum period of 400 ns. Always driven by the EMAC to PHY.             |
| MDC-Management Data Clock           | Data signal driven by EMAC or PHY, depending on write or read access based on EMAC; synchronous to MDC. |

## MDC Clock Frequency

The EMAC uses the EMAC\_SMI\_ADDR.CR bit field to determine the frequency of MDC as shown in the MDC Clock Frequency Selection table. The clock range selection determines the frequency of the clock relative to the CLKO7 frequency. The table shows the suggested range of CLKO7 frequency applicable for each value of the EMAC\_SMI\_ADDR.CR field. The programmability based on CLKO7 frequency range ensures that the MDC clock frequency range is within the IEEE specifications of 1.0 MHz to 2.4 MHz. However, the EMAC MDC can also support higher frequencies for PHY devices that support the frequencies.

Table 29-37: MDC Clock Frequency Selection

|   EMAC_SMI_ADDR.CR Selec- tion | Programmed CLKO7 Frequen- cy Range   | Frequency ofMDC   | Min and Max MDCFreq (Per Specifications)   |
|--------------------------------|--------------------------------------|-------------------|--------------------------------------------|
|                           0000 | 60-100 MHz                           | CLKO7/42          | MIN = 1.43 MHz and MAX = 2.39 MHz          |
|                           0010 | 20-35 MHz                            | CLKO7/16          | MIN = 1.25 MHz and MAX = 2.19 MHz          |
|                           0011 | 35-60 MHz                            | CLKO7/26          | MIN = 1.35 MHz and MAX = 2.31 MHz          |

The MDIO Frame Parameters table provides MDIO data transfer parameters. The write and read sequences provided in the tables, MDIO Write Data Sequence and MDIO Read Data Sequence , are based on these parameters.

Table 29-38: MDIO Frame Parameters

| Parameter   | Description                                                                      |
|-------------|----------------------------------------------------------------------------------|
| IDLE        | The MDIO line is three-state (noted as Z in sequence); there is no clock on MDC. |
| PREAMBLE    | 32 continuous bits, each of value 1                                              |
| START       | Start of frame is 01                                                             |
| OPCODE      | 10 for read and 01 for write                                                     |
| PHY ADDR    | 5-bit address select for one of 32 PHYs (noted as AAAAA in sequence)             |
| REG ADDR    | Register address in the selected PHY (noted as RRRRR in sequence)                |
| TA          | Turnaround is Z0 for read and 10 for write (Z = high impedance)                  |
| DATA        | Any 16-bit value. Driven by MAC or PHY based on direction (noted as DDD...DDD).  |

Table 29-39: MDIO Write Data Sequence

| IDLE   | PREAMBLE   |   START |   OPCODE | PHY ADDR   | REG ADDR   |   TA | DATA       | IDLE   |
|--------|------------|---------|----------|------------|------------|------|------------|--------|
| Z      | 111...111  |      01 |       01 | AAAAA      | RRRRR      |   10 | DDD... DDD | Z      |

Table 29-40: MDIO Read Data Sequence

| IDLE   | PREAMBLE   |   START |   OPCODE | PHY ADDR   | REG ADDR   | TA   | DATA       | IDLE   |
|--------|------------|---------|----------|------------|------------|------|------------|--------|
| Z      | 111...111  |      01 |       10 | AAAAA      | RRRRR      | Z0   | DDD... DDD | Z      |

## SMI Write Operation

When programs set the EMAC\_SMI\_ADDR.SMIW (write) and EMAC\_SMI\_ADDR.SMIB (busy) bits, the station management interface (SMI) initiates a write operation into the PHY registers. The write operation uses the management frame format (the PHY address, the register address in PHY, and the write data) specified in the IEEE specifications. (Section 22.2.4.5 of IEEE standard). The application must not change the EMAC\_SMI\_ADDR register contents or the EMAC\_SMI\_DATA register while the transaction is ongoing.

Write operations to the EMAC\_SMI\_ADDR register or the EMAC\_SMI\_DATA register during the transfer period are ignored (while the EMAC\_SMI\_ADDR.SMIB bit is high). The transaction completes without error. After the write operation has completed, the SMI indicates the same by resetting the EMAC\_SMI\_ADDR.SMIB bit. The EMAC drives the MDIO line for the complete duration of the frame. The SMI Write Operation through MDIO/MDC Pins figure shows this operation.

Figure 29-16: SMI Write Operation through MDIO/MDC Pins

<!-- image -->

## SMI Read Operation

When programs set the EMAC\_SMI\_ADDR.SMIB bit with the EMAC\_SMI\_ADDR.SMIW bit cleared (=0), the station management interface (SMI) initiates a read operation in the PHY registers. It transfers the PHY address and the register address in the PHY to the SMI. The application must not change the EMAC\_SMI\_ADDR register contents or the EMAC\_SMI\_DATA register while the transaction is ongoing.

Write operations to the EMAC\_SMI\_ADDR register or the EMAC\_SMI\_DATA register during the transfer period are ignored (while the EMAC\_SMI\_ADDR.SMIB bit is high). The transaction completes without error. After the read operation has completed, the SMI resets the EMAC\_SMI\_ADDR.SMIB bit and updates the EMAC\_SMI\_DATA register with the data read from the PHY. The EMAC drives the MDIO line for the complete duration of the frame except during the data fields when the PHY drives the MDIO line. The SMI Read Operation through MDIO/MDC Pins figure shows this operation.

Figure 29-17: SMI Read Operation through MDIO/MDC Pins

<!-- image -->

## EMAC Management Counters (MMC)

The EMAC provides a comprehensive set of 32-bit MAC management counters. It uses these counters for gathering statistics on the received and transmitted frames. The MMC subblock also includes

- A control register ( EMAC\_MMC\_CTL ) for managing the behavior of the counters
- Two 32-bit registers containing interrupts generated ( EMAC\_MMC\_RXINT and EMAC\_MMC\_TXINT )
- Two 32-bit registers containing masks for the interrupt register ( EMAC\_MMC\_RXIMSK and EMAC\_MMC\_TXIMSK )

The MMC receive counters are updated for frames passed by the address filtering subblock in the EMAC CORE. Statistics of frames dropped by the AFM module are not updated unless they are runt frames of less than 6 bytes. (Destination address bytes are not received fully.) The module is also capable of gathering statistics on encapsulated IPv4, IPv6, and TCP , UDP , or ICMP payloads in received Ethernet frames.

The MMC register naming conventions are as follows:

- Tx as a prefix or suffix indicates counters associated with transmission.
- Rx as a prefix or suffix indicates counters associated with reception.
- \_G as a suffix indicates registers that count good frames only.
- \_GB as a suffix indicates registers that count frames regardless of whether they are good or bad.

Transmitted frames are considered good when transmitted successfully. In other words, a transmitted frame is good if the frame transmission does not abort due to any of the following errors:

- Jabber timeout
- No carrier or loss of carrier
- Late collision
- Frame underflow
- Excessive deferral
- Excessive collision

Received frames are good when none of the following errors exists:

- CRC error
- Runt frame (shorter than 64 bytes)
- Alignment error
- Length error (non-type frames only)
- Out-of-range (non-type frames only, longer than maximum size)

The maximum frame size depends on the frame type, as follows:

- Untagged frame maxsize = 1518
- VLAN frame maxsize = 1522
- Jumbo frame maxsize = 9018
- Jumbo VLAN frame maxsize = 9022

The EMAC\_MMC\_CTL register also contains bits that control preset, freeze and roll-over of counters. The EMAC uses the EMAC\_MMC\_CTL.RDRST bit to enable an auto-reset feature whenever the counters are read. The EMAC uses the EMAC\_MMC\_CTL.RST bit to reset all the counters.

The MMC can trigger an interrupt when the corresponding bits are enabled in the transmit, receive, and IPC mask registers, and when the particular counter reaches half or full. The status is also updated in the corresponding interrupt register.

## MMC Receive Interrupt Register

The EMAC\_MMC\_RXINT register maintains the interrupts that are generated when the receive statistic counters reach half their maximum values (0x80000000), and when they cross their maximum values (0xFFFFFFFF). When EMAC\_MMC\_CTL.NOROLL is set, then interrupts are set, but the counter remains at all ones. The EMAC\_MMC\_RXINT register is a 32-bit wide register. An interrupt bit is cleared when the respective MMC counter that caused the interrupt is read. The least significant byte lane (bits 7:0) of the respective counter must be read to clear the interrupt bit.

## MMC Transmit Interrupt Register

The EMAC\_MMC\_TXINT register maintains the interrupts generated when the transmit statistic counters reach half their maximum values (0x80000000), and when they cross their maximum values (0xFFFFFFFF). When EMAC\_MMC\_CTL.NOROLL is set, then interrupts are set, but the counter remains at all ones. The EMAC\_MMC\_TXINT register is a 32-bit wide register. An interrupt bit is cleared when the respective MMC counter that caused the interrupt is read. The least significant byte lane (bits 7:0) of the respective counter must be read to clear the interrupt bit.

## MMC Receive Checksum Offload Interrupt Register

The EMAC\_MMC\_RXINT.CRCERR register maintains the interrupts generated when receive IPC statistic counters reach half their maximum values (0x80000000), and when they cross their maximum values (0xFFFFFFFF). When EMAC\_MMC\_CTL.NOROLL is set, then interrupts are set, but the counter remains at all ones. The EMAC\_MMC\_RXINT.CRCERR register is 32 bits wide. When the MMC IPC counter that caused the interrupt is read, its corresponding interrupt bit is cleared. The least-significant byte lane (bits 7:0) of the counter must be read to clear the interrupt bit.

## EMAC Precision Time Protocol (PTP) Engine

The following sections describe the precision time protocol (PTP) engine.

## IEEE1588 and the PTP Engine

The Ethernet MAC peripheral includes a PTP engine to assist applications requiring time synchronization. The PTP module is tightly integrated with the EMAC CORE to aid hardware time stamping defined in the IEEE1588 2002/2008 standards. Applications can use accurate hardware time stamps through TCP/IP stacks (if using network layer communication) to exchange time information across devices connected over network. Applications can also use accurate hardware time stamps through Ethernet device drivers (if using MAC layer communication) to exchange time information.

## PTP Engine

For calculation of drift in time between two Ethernet devices, the device records its system time whenever a timing message is sent or received (IEEE 1588 protocol). Due to the indeterministic delay of a software system for a node, the software is unable to capture an accurate time when the message is sent or received. However, the hardware can monitor the signal on the communication media and get an accurate message of arrival and departure time.

The PTP (precision time protocol) module is closely integrated with the EMAC module. It provides hardware assistance to implement both the IEEE 1588-2002 and IEEE 1588-2008 standards on Ethernet (IEEE 802.3). It takes

one input clock signal as its PTP clock and maintains the timing information (called system time ) at the nanosecond level.

The PTP module includes hardware for clock and system time adjustment. The pulse-per-second (PPS) signals physically represent the system time. PPS can be programmed to a fixed frequency or provide flexibility to the signal in terms of pulse width, interval, start, and stop time of the signals. The PTP module can be programmed to trigger an alarm interrupt when system time reaches specified time.

The PTP module can be programmed to detect different types of received frames, capture the system time, and time stamp those frames with the captured system time. Programs can configure any frame so that the PTP module capture the system time when it is transmitted. The PTP module can also capture the system time when an event is detected on the auxiliary snapshot trigger input pins ( EMAC\_PTPAUXIN[n] ).

## IEEE 1588 Standard

Many systems require two independent devices to operate in a time synchronized fashion. If each system relied solely on its oscillator, differences between the characteristics and operating conditions of each oscillator would limit the ability of the clocks to operate synchronously. To serve applications requiring synchronized clocks, the system uses a periodic correction mechanism.

A simple way to synchronize multiple systems is to choose one system (with the best clock) as a requester. The system requester broadcasts the clock and timing information to other systems (completers); subsequently, the completers adjust their clocks and timing according to that of requester. However, this method has limitations. The requester cannot broadcast the time at infinitesimal intervals. Path delay (propagation delay) exists between a requester and a completer and the delay varies between each completer and requester.

IEEE 1588 is also known as precision time protocol or PTP . The standard specifies a protocol used to synchronize the time and clock of multiple devices, dispersed but interconnected by any communication, (for example, Ethernet IEEE 802.3). According to the protocol, timing messages are exchanged between two devices. Then, one of the devices calculates its drift from other device and corrects its system time. (Both devices must have the same representation of their system time). The protocol resolves path delay between devices. It also helps synchronize the clocks of multiple devices and all of the limitations mentioned are resolved.

IEEE 1588 was published in 2002 where four types of timing messages were defined: Sync, Follow\_Up, Delay\_Req, and Delay\_Resp. Here the protocol synchronizes two or more devices where one is a requester and others are completers. The requester device sends Sync, Follow\_Up, and Delay\_Resp messages to the completer device in the system. The target sends the Delay\_Req messages to the requester device. A following section provides more information on IEEE 1588-2002.

In 2008, a newer version of IEEE 1588 was introduced which provides further mechanisms to measure the peer-topeer delay. Three more timing messages (PdelayReq, PdelayResp, and PdelayRespFollowup) were added to implement peer-to-peer synchronization. The following section provides more information on IEEE 1588-2008.

## IEEE 1588-2002

The IEEE 1588-2002 standard defines the precision time protocol (PTP). The protocol allows precise synchronization of clocks in measurement and control systems that use network communication, local computing, and distributed objects. The protocol applies to systems that communicate by local area networks that support multicast

messaging, including (but not limited to) Ethernet. This protocol also allows heterogeneous systems that include clocks of varying inherent precision, resolution, and stability to synchronize. The protocol supports system-wide synchronization accuracy in the sub-microsecond range using minimal network and local clock computing resources.

The PTP is transported over UDP/IP . The system or network is classified into requester and completer nodes for distributing the timing or clock information. The IEEE 1588-2002 PTP Process figure shows the PTP process used for synchronizing a target node to a controller node by exchanging PTP messages.

Figure 29-18: IEEE 1588-2002 PTP Process

<!-- image -->

As shown in the figure, the PTP uses the following process:

1. The requester broadcasts the PTP sync messages to all its nodes. The sync message contains the reference time information of the requester. The time at which this message leaves the system of the requester is t 1 . The requester must capture this time for Ethernet ports, at RMII.
2. The completer receives the Sync message and also captures the exact time, t 2 , using its timing reference.
3. The requester sends a Follow\_up message to the completer, which contains t1 information for later use.
4. The completer sends a Delay\_Req message to the requester, noting the exact time, t 3 , at which this frame leaves the RMII.
5. The requester receives the message, capturing the exact time, t 4 , at which it enters its system.
6. The requester sends the t 4  information to the completer in the Delay\_Resp message.
7. The completer uses the four values of t 1 , t 2 , t 3 , and t 4  to synchronize its local timing reference to the timing reference of the requester.

Most of the PTP implementation occurs in the software above the UDP layer. However, the hardware support must capture the exact time when specific PTP packets enter or leave the Ethernet port at the RMII/RGMII. Hardware must capture this timing information and return it to the software for the proper implementation of PTP with high accuracy.

## IEEE 1588-2008 Advanced Time Stamps

In addition to the basic time stamp features mentioned in IEEE 1588-2002 time stamps, the EMAC supports the following advanced time stamp features defined in the IEEE 1588-2008 standard.

- Support for the IEEE 1588-2008 (Version 2) time stamp format.
- Provides an option to take snapshot of all frames or only PTP type frames.
- Provides an option to take snapshot of only event messages.
- Provides an option to select the node to be a requester or completer.
- Identifies the PTP message type, version, and PTP payload in frames sent directly over Ethernet and sends the status.
- Provides an option to run nanoseconds time in digital or binary format.

## Peer-to-Peer (P2P) PTP Message Support

The IEEE 1588-2008 version supports Peer-to-Peer PTP (Pdelay) message in addition to SYNC, Delay Request, Follow-up, and Delay Response messages. Refer to the Propagation Delay Calculation between Nodes Supporting P2P Path Correction figure. The figure shows the method to calculate the propagation delay between nodes supporting peer-to-peer path correction.

Figure 29-19: Propagation Delay Calculation between Nodes Supporting P2P Path Correction

<!-- image -->

As shown in the figure, the propagation delay is calculated in the following way:

1. Port 1 issues a Pdelay\_Req message and generates a time stamp, t1, for the Pdelay\_Req message.
2. Port 2 receives the Pdelay\_Req message and generates a time stamp, t2, for this message.
3. Port 2 returns a Pdelay\_Resp message and generates a time stamp, t3, for this message. To minimize errors due to frequency offset between the two ports, port 2 returns the Pdelay\_Resp message as quickly as possible after the receipt of the Pdelay\_Req message. The port 2 returns any one of the following:
- The difference between the time stamps t2 and t3 in the Pdelay\_Resp message.
- The difference between the time stamps t2 and t3 in the Pdelay\_Resp\_Follow\_Up message.
- The time stamps t2 and t3 in the Pdelay\_Resp and Pdelay\_Resp\_Follow\_Up messages respectively.
4. Port 1 generates a time stamp, t4, on receiving the Pdelay\_Resp message.

Port 1 uses all four time stamps to compute the mean link delay.

## Block Diagram

The PTP Block Diagram figure shows the functional block diagram of PTP module.

Figure 29-20: PTP Block Diagram

<!-- image -->

TRIGGER INTERRUPT

A system time module is present which keeps the time of PTP module. It consists of hardware which can be programmed for time initialization, time correction, and clock correction.

The time stamp module captures the time (provided by the system time module) at various conditions. For example, when the EMAC sends or receives a frame or during the rising edge of the auxiliary snapshot trigger EMAC\_PTPAUXIN[n] pins. When system time is captured after detection of a frame, the time stamp module automatically includes the time information in the frame descriptor. Time stamping on the detection of a frame can be programmed on a per frame basis.

The PTP clock drives the PTP module. This clock can be selected from three different clock sources.

The Pulse per Second (PPS) module generates a pulse or train of pulse on the PPS output pins, ( EMAC\_PTPPPS[n] ). It is the physical representation of system time. PPS can be fixed (where only frequency varies) or flexible (where width, interval, start time, and stop time can be programmed).

The target time module acts as an alarm for the PTP module. Whenever system time reaches a value equal to programmed target time, the target time trigger interrupt is generated. By appropriate programming, The target time trigger can also start or stop flexible PPS output at specific time.

## PTP Module Clock

The PTP module clock features include Clock Source Selection and Clock Frequency Range.

## Clock Source Selection

The PTP module can take one of these clock sources as its input clock - CLKO7, RMII reference RGMII Rx clock, or PTP external clock.

As shown in the PTP Clock Source Selection table, the PADS\_PCFG0 register selects the PTP clock source.

Table 29-41: PTP Clock Source Selections

| PADS_PCFG0 [1:0]   | PTP Clock Source             | Clock Description                       |
|--------------------|------------------------------|-----------------------------------------|
| 00 - RMII or MII   | ETH0_RXCLK_REFCLK/ETH1REFCLK | MII/RII reference clock                 |
| 00 - RGMII         | CLK07                        | Processor Ethernet clock                |
| 10                 | PTP external clock           | Clock available on EMAC_PTPCLKIN[n] pin |
| 01 or 11           | SCLK0                        | Processor system clock                  |

## Clock Frequency Range

The resolution, or granularity, of the reference time source determines the accuracy of the synchronization. Therefore, a higher PTP clock frequency gives better system performance. The timing constraints achievable for logic operating on the selected PTP clock source limit the maximum PTP clock frequency.

The minimum PTP clock frequency depends on the time required between two consecutive frames. The IEEE specification determines the RMII clock frequency. The minimum PTP clock frequency required for proper operation depends on the operating mode and operating speed of the MAC. See the Minimum PTP Clock Frequency table.

A minimum delay required between two consecutive time stamp captures is 8 clock cycles of RMII or 4 clock cycles of RGMIIand 3 clock cycles of PTP clocks. If the delay between two time stamp captures is less than this delay, the EMAC does not take a time stamp snapshot for the second frame.

## The following table assumes:

- Minimum Ethernet packet size = 64 bytes
- Miminum IFG = 96 bit times = 12 bytes
- Preamble = 8 bytes
- 3 PTP Clock + 4GMII/MII Clock Min gap between two SFDs (start of frame delimiters)
- 3 PTP Clock + 8 RMII Clock Min gap between two SFDs
- 3 PTP Clock + 4 RGMII Clock Min gap between two SFDs

| Mode (Full Duplex in all cases)   | Minimum Gap between two SFDs                       | PTP clock                                                     | Comments                                                  |
|-----------------------------------|----------------------------------------------------|---------------------------------------------------------------|-----------------------------------------------------------|
| 10 Mbps RMII                      | 64 bytes of data + 8 bytes of pre- amble + min IFG | 3 PTP clock cycle + 8RMII clock cycle 3360 RMII clock cycles  | In RMII @10 Mbps, 1 byte trans- mitted in 40 clock cycles |
| 10 Mbps RMII                      | (2560 + 320 + 480) RMII clock cycles               | 3 PTP clock cycle 3352 RMII clock cycles                      | RMII clock = 50 MHz                                       |
| 10 Mbps RMII                      | 3360 RMII clocks                                   | PTP clock cycle 1117.33 20 ns = 22346 ns                      | 1 RMII clock cycle = (1/50 MHz) = 20 ns                   |
| 10 Mbps RMII                      |                                                    | PTP frequency min = 0.045 MHz                                 |                                                           |
| 10 Mbps RGMII                     | 64 bytes of data + 8 bytes of pre- amble + min IFG | 3 PTP clock cycle + 4RGMII clock cycle 168 RGMII clock cycles | In RGMII, 1 byte transmitted in 4 clock cycles            |
| 10 Mbps RGMII                     | (128 + 16 + 24 ) MII clock cy- cles                | 3 PTP clock cycle<= 164 RGMII clock cycles                    | RGMII clock at 100 Mbps = 2.5 MHz                         |
| 10 Mbps RGMII                     | 168 MII clock cycles                               | PTP clock cycle 54.67 400ns = 21860 ns                        | 1 RGMII clock cycle = (1/2.5 MHz) = 400 ns                |
| 10 Mbps RGMII                     |                                                    | PTP frequency min = 0.045 MHz                                 |                                                           |
| 100 Mbps RMII                     | 64 bytes of data + 8 bytes of pre- amble + min IFG | 3 PTP clock cycle + 8RMII clock cycle 336 RMII clock cycles   | In RMII, 1 byte transmitted in 4 clock cycles             |
| 100 Mbps RMII                     | (256 + 32 + 48) RMII clock cy- cles                | 3 PTP clock cycle 328 RMII clock cycles                       | RMII clock = 50 MHz                                       |
| 100 Mbps RMII                     | 336 RMII clocks                                    | PTP clock cycle 109.33 20 ns = 2186 ns                        | 1 RMII clock cycle = (1/50 MHz) = 20 ns                   |
| 100 Mbps RMII                     |                                                    | PTP frequency min = 0.45 MHz                                  |                                                           |

| Mode (Full Duplex in all cases)   | Minimum Gap between two SFDs                       | PTP clock                                                     | Comments                                                  |
|-----------------------------------|----------------------------------------------------|---------------------------------------------------------------|-----------------------------------------------------------|
| 100 Mbps RGMII                    | 64 bytes of data + 8 bytes of pre- amble + min IFG | 3 PTP clock cycle + 4RGMII clock cycle 168 RGMII clock cycles | In RGMII, 1 byte transmitted in 4 clock cycles            |
| 100 Mbps RGMII                    | (128 + 16 + 24 ) MII clock cy- cles                | 3 PTP clock cycle 164 RGMII clock cycles                      | RGMII clock at 100 Mbps = 25 MHz                          |
| 100 Mbps RGMII                    | 168 MII clock cycles                               | PTP clock cycle 54.67 40 ns = 2186 ns                         | 1 RGMII clock cycle = (1/25 MHz) = 40 ns                  |
| 100 Mbps RGMII                    |                                                    | PTP frequency min = 0.45 MHz                                  |                                                           |
| 1000 Mbps RGMII                   | 64 bytes of data + 8 bytes of pre- amble + min IFG | 3 PTP clock cycle + 4 RGMII clock cycle 84 RGMII clock cycles | In RGMII @1000 Mbps, 1 byte transmitted in 1 clock cycles |
| 1000 Mbps RGMII                   | (64 + 8 + 12) RGMII clock cy- cles                 | 3 PTP clock cycle 80 RGMII clock cycles                       | RGMII clock at 100 Mbps = 125 MHz                         |
| 1000 Mbps RGMII                   | 84 RGMII clocks                                    | PTP clock cycle 26.67 8 ns = 213 ns                           | 1 RGMII clock cycle = (1/125 MHz) = 8 ns                  |
| 1000 Mbps RGMII                   |                                                    | PTP frequency min = 4.6 MHz                                   |                                                           |

The minimum PTP clock frequency also depends on the maximum value of the EMAC\_TM\_SUBSEC register, so that even at the highest EMAC\_TM\_SUBSEC.SSINC value, the EMAC\_TM\_SEC register value can be incremented every second. Since the EMAC\_TM\_SUBSEC.SSINC is an 8-bit field, the minimum PTP clock frequency allowed is approximately 4 MHz.

## Time Stamp Module

The time stamp module captures time in seconds and nanoseconds maintained as system time. The time stamp module also captures time when specific events occur. Events include detection of a frame transmitted or received over the EMAC and a rising edge on the EMAC\_PTPAUXIN[n] pins. The time stamp module does not need to time stamp all of the transmitted or received frames over the EMAC. The PTP module can be programmed to detect specific kinds of frames for time stamping.

## Frame Detection and Time Stamping

The PTP module automatically monitors all received and transmitted IEEE 1588 event messages on the Ethernet. If the module detects an event message, it takes a snapshot of the system time. The PTP module stores the value to the 64-bit fields in transmit or receive descriptor.

The time stamping occurs at the EMAC RMII/RGMII interface when the module sees the start of frame of an event message packet.

## Transmit Path Time Stamping

The EMAC captures a time stamp when a frame transmits on the RMII/RGMII. Time stamp capture is controllable on a per-frame basis. In other words, each transmit frame can be marked to indicate whether a time stamp is captured for that frame or not.

Applications can extend the descriptor word length from 4 words to 8 words by setting the EMAC\_DMA0\_BUSMODE.ATDS bit. T o enable the time stamp function, set the TTSE (transmit time stamp enable) bit in transmit descriptor word TDES0. When the PTP module captures a time stamp of a transmitted frame, it notifies the application by setting the TTSS (transmit time stamp status) in TDES0.

The EMAC returns the time stamp to the software inside the corresponding transmit descriptor, automatically connecting the time stamp to the specific frame. The 64-bit time stamp information is written to the TDES6 and TDES7 fields. The TDES6 field holds the 32-bit LSBs of the time stamp (system time nanoseconds), except as described in transmit time stamp field, and the TDES7 field holds the 32-bit MSBs (system time seconds). After the PTP module time stamps the frame, the application can get the time stamp along with the transmit status from the EMAC.

NOTE: The PTP module time stamps all the transmitting frames having TTSE set in its TDES0. It does not distinguish according to the type of transmitting frame.

## Receive Path Time Stamping

The PTP module automatically monitors all received and transmitted IEEE 1588 event messages on the Ethernet. If an event message is detected, the module takes a snapshot of the system time. The module stores its value to the 64bit fields in transmit or receive descriptor. The time stamping is done at the EMAC RMII/RGMII interface when the module sees the start of frame of an event message packet.

PTP module captures the time stamp of received frames on the RMII/RGMII. Time stamp capture is controllable on a per-frame and per-type basis. In other words, each received frame is time stamped according to the frame type.

Applications can extend the descriptor word length from 4 words to 8 words by setting the EMAC\_DMA0\_BUSMODE.ATDS bit to store time stamp and received message status. The PTP notifies the application of receive time stamp availability when it sets bit 7 (time stamp available) in receive descriptor word RDES0.

When bit 0 (extended status available) is set in RDES0, it indicates that the extended status of the PTP frame is provided in the RDES4 word. Extended status includes PTP version, PTP frame type, and message type. The EMAC returns the time stamp to the software inside the corresponding receive descriptor. The 64-bit time stamp information is written back to the RDES6 and RDES7 fields in memory. The RDES6 holds the 32-bit LSBs of the time stamp (system time nanoseconds), except as mentioned in receive time stamp field, and the RDES7 field holds 32-bit MSBs (system time seconds).

The time stamp is written only to that receive descriptor for which the last descriptor status field has been set to 1. When the time stamp is not available (for example, because of an RxFIFO overflow), an all-ones pattern is written to the descriptors (RDES6 and RDES7). The write operation indicates that the time stamp is not correct. RDES0 [7] indicates whether the time stamp is updated in RDES6/7.

The PTP module processes received frames to identify valid PTP frames. Use the EMAC\_TM\_CTL register to control the snapshot of the time sent to the application.

The PTP module can be programmed to detect all received frames or only some types of PTP frames, according to bit settings in the EMAC\_TM\_CTL register. Refer to the PTP Frame Type Selections table.

Table 29-42: PTP Frame Type Selections

|   TSENALL (bit 8) | SNAPTYPSEL (bits [17:16])   | TSMSTRENA (bit 15)   | TSEVNTENA (bit 14)   | Frames                                                                                      |
|-------------------|-----------------------------|----------------------|----------------------|---------------------------------------------------------------------------------------------|
|                 1 | X                           | X                    | X                    | All                                                                                         |
|                 0 | 00                          | X                    | 0                    | Sync, Follow_Up, De- lay_Req, Delay_Resp                                                    |
|                 0 | 00                          | 0                    | 1                    | Sync                                                                                        |
|                 0 | 00                          | 1                    | 1                    | Delay_Req                                                                                   |
|                 0 | 01                          | X                    | 0                    | Sync, Follow_Up, De- lay_Req, Delay_Resp, Pdelay_Req, Pde- lay_Resp, Pdelay_Resp_ Follow_Up |
|                 0 | 01                          | 0                    | 1                    | Sync, Pdelay_Req, Pde- lay_Resp                                                             |
|                 0 | 01                          | 1                    | 1                    | Delay_Req, Pdelay_Req, Pdelay_Resp                                                          |
|                 0 | 10                          | X                    | X                    | Sync, Delay_Req                                                                             |
|                 0 | 11                          | X                    | X                    | Pdelay_Req, Pdelay_Resp                                                                     |

## PTP Processing and Control

When the EMAC module receives a frame, frame detection and time stamping are based on some of the PTP fields in the frame. The PTP Message Format (IEEE 1588-2008) table shows the common message header for the PTP messages. This format is derived from the IEEE standard 1588-2008. When the EMAC module sends a PTP frame, the frame follows this format.

When a frame is received, the PTP module compares these fields with standard values and finds out the type of PTP frame and other information (for example, PTP version, IP version, and others). The module then updates the related fields in RDES4. When a frame is transmitted, programs must ensure that all the fields are appropriate. The PTP module on the other end of a communication must correctly detect and decode the frame.

Table 29-43: PTP Message Format (IEEE 1588-2008)

| Bits              | Bits              | Bits              | Bits              | Bits          | Bits          | Bits          | Bits          |   Octets |   Offset |
|-------------------|-------------------|-------------------|-------------------|---------------|---------------|---------------|---------------|----------|----------|
| 7                 | 6                 | 5                 | 4                 | 3             | 2             | 1             | 0             |          |          |
| transportSpecific | transportSpecific | transportSpecific | transportSpecific | messageType   | messageType   | messageType   | messageType   |        1 |        0 |
| Reserved          | Reserved          | Reserved          | Reserved          | versionPTP    | versionPTP    | versionPTP    | versionPTP    |        1 |        1 |
| messageLength     | messageLength     | messageLength     | messageLength     | messageLength | messageLength | messageLength | messageLength |        2 |        2 |
| domainNumber      | domainNumber      | domainNumber      | domainNumber      | domainNumber  | domainNumber  | domainNumber  | domainNumber  |        1 |        4 |
| Reserved          | Reserved          | Reserved          | Reserved          | Reserved      | Reserved      | Reserved      | Reserved      |        1 |        5 |

Table 29-43: PTP Message Format (IEEE 1588-2008)  (Continued)

| Bits                                                                                                                 |   Octets |   Offset |
|----------------------------------------------------------------------------------------------------------------------|----------|----------|
| flagField                                                                                                            |        2 |        6 |
| correctionField                                                                                                      |        8 |        8 |
| Reserved                                                                                                             |        4 |       16 |
| sourcePortIdentity                                                                                                   |       10 |       20 |
| sequenceId                                                                                                           |        2 |       30 |
| controlField (used in version 1. For version 2, messageType field is used for detecting dif- ferent message types. ) |        1 |       32 |
| logMessageInterva                                                                                                    |        1 |       33 |

There are some fields in the Ethernet payload that can be used to detect the PTP packet type and control the snapshot taken. These fields are different for the following PTP frames:

- PTP Frames Over IPv4
- PTP Frames Over IPv6
- PTP Frames Over Ethernet

For these PTP frames, EMAC does not consider the PTP version 1 messages as valid when the frame consists of peer delay multicast address as destination address (DA).

## PTP Frame Over IPv4

The IPv4-UDP PTP Frame Fields Required for Control and Status table provides information about the fields that are matched to control snapshot for the PTP packets. The packets are sent over UDP over IPv4 for IEEE 1588 version 1 and 2. The octet positions for the tagged frames are offset by 4. The positions are based on IEEE 15882008 standards and the message format. The format is defined in the PTP Message Format (IEEE 1588-2008) table in the PTP Processing and Control section.

Table 29-44: IPv4-UDP PTP Frame Fields Required for Control and Status

| Field Matched                              | Octet Position   | Matched Value                                  | Description                                                                      |
|--------------------------------------------|------------------|------------------------------------------------|----------------------------------------------------------------------------------|
| MAC Frame type                             | 12, 13           | 0x0800                                         | IPv4 datagram                                                                    |
| IP Version and Header Length               | 14               | 0x45                                           | IP version is IPv4                                                               |
| Layer 4 Protocol                           | 23               | 0x11                                           | UDP                                                                              |
| IP Multicast Address (IEEE 1588 Version 1) | 30, 31, 32, 33   | 0xE0,0x00, 0x01,0x81 (or 0x82 or 0x83 or 0x84) | Multicast IPv4 addresses allowed 224.0.1.129 224.0.1.130 224.0.1.131 224.0.1.132 |

Table 29-44: IPv4-UDP PTP Frame Fields Required for Control and Status (Continued)

| Field Matched                              | Octet Position   | Matched Value                                             | Description                                                                                                                                                                |
|--------------------------------------------|------------------|-----------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| IP Multicast Address (IEEE 1588 Version 2) | 30, 31, 32, 33   | 0xE0, 0x00, 0x01, 0x81 (Hex) 0xE0, 0x00, 0x00, 0x6B (Hex) | PTP-Primary multicast address: 224.0.1.129 PTP-Peer delay multicast address: 224.0.0.107                                                                                   |
| UDP Destination Port                       | 36, 37           | 0x013F 0x0140                                             | 0x013F - PTP Event Messages. These are SYNC, Delay_Req (IEEE 1588 version 1 and 2) or Pdelay_Req, Pdelay_Resp (IEEE 1588 version 2 only). 0x0140 - PTP general messages    |
| PTP Control Field (IEEE ver- sion 1)       | 74               | 0x00/0x01/0x02/ 0x03/0x04                                 | 0x00 - SYNC 0x01 - Delay_Req 0x02 - Follow_Up 0x03 - Delay_Resp 0x04 - Management                                                                                          |
| PTP Message Type Field (IEEE version 2)    | 42 (nibble)      | 0x0/0x1/0x2/0x3/0x8/0x9/0xA/0xB/ 0xC/0xD                  | 0x0 - SYNC 0x1 - Delay_Req 0x2 - Pdelay_Req 0x3 - Pdelay_Resp 0x8 - Follow_Up 0x9 - Delay_Resp 0xA - Pdelay_Resp_Follow_Up 0xB - Announce 0xC - Signaling 0xD - Management |
| PTP Version                                | 43 (nibble)      | 0x1 or 0x2                                                | 0x1 - Supports PTP version 1 0x2 - Supports PTP version 2                                                                                                                  |

## PTP Frame Over IPv6

The IPv6-UDP PTP Frame Fields Required for Control And Status table provides information about the fields that are matched to control the snapshots for the PTP packets. The packets are sent over UDP over IPv6 for IEEE 1588 version 1 and 2. The octet positions for the tagged frames are offset by 4. The positions are based on IEEE 15882008 standards and the message format defined in PTP Message Format (IEEE 1588-2008).

Table 29-45: IPv6-UDP PTP Frame Fields Required for Control and Status

| Field Matched   | Octet Position   | Matched Value   | Description   |
|-----------------|------------------|-----------------|---------------|
| MAC Frame type  | 12, 13           | 0x86DD          | IP datagram   |

Table 29-45: IPv6-UDP PTP Frame Fields Required for Control and Status (Continued)

| Field Matched                           | Octet Position                                                    | Matched Value                                                | Description                                                                                                                                                                |
|-----------------------------------------|-------------------------------------------------------------------|--------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| IP Version                              | 14 (bits [7:4])                                                   | 0x06                                                         | IP version is IPv6                                                                                                                                                         |
| Layer 4 Protocol                        | 20 (IPv6 extension header not defined for PTP packets)            | 0x11                                                         | UDP                                                                                                                                                                        |
| PTP Multicast Address                   | 38-53                                                             | FF0x:0:0:0:0:0:0:0:0:181 (Hex) FF02:0:0:0:0:0:0:0:0:6B (Hex) | PTP - primary multicast address: FF0x: 0:0:0:0:0:0:0:0:181 (Hex) PTP - Peer delay multicast address: FF02:0:0:0:0:0:0:0:0:6B (Hex)                                         |
| UDP Destination Port                    | 56, 57 (IPv6 exten- sion header not de- fined for PTP pack- ets)  | 0x013F, 0x0140                                               | 0x013F - PTP event messages 0x0140 - PTP general messages                                                                                                                  |
| PTP Control Field (IEEE 1588 version 1) | 93 (IPv6 extension header not defined for PTP packets)            | 0x00/0x01/0x02/ 0x03/0x04                                    | 0x00 - SYNC 0x01 - Delay_Req 0x02 - Follow_Up 0x03 - Delay_Resp 0x04 - Management (version1)                                                                               |
| PTP Message Type Field (IEEE version 2) | 74 (nibble) (IPv6 ex- tension header not defined for PTP packets) | 0x0/0x1/0x2/0x3/0x8/0x9/0xA/0xB/ 0xC/0xD                     | 0x0 - SYNC 0x1 - Delay_Req 0x2 - Pdelay_Req 0x3 - Pdelay_Resp 0x8 - Follow_Up 0x9 - Delay_Resp 0xA - Pdelay_Resp_Follow_Up 0xB - Announce 0xC - Signaling 0xD - Management |
| PTP Version                             | 75 (nibble)                                                       | 0x1 or 0x2                                                   | 0x1 - Supports PTP version 1 0x2 - Supports PTP version 2                                                                                                                  |

## PTP Frame Over Ethernet

Refer to the Ethernet PTP Frame Fields Required for Control and Status table. The table provides information about the fields that are matched to control the snapshots for the PTP packets sent over Ethernet for IEEE 1588 version 1 and 2. The octet positions for the tagged frames are offset by 4. The positions are based on IEEE 15882008 standards and the message format defined in the table.

Table 29-46: Ethernet PTP Frame Fields Required for Control and Status

| Field Matched                                                                                                                                                        | Octet Position   | Matched value                             | Description                                                                                                                                                                |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|-------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| MAC Destination Multicast Address (The address match of destination address (DA) pro- grammed in MAC address 0 is used when the EMAC_TM_CTL. TSENMACADDR bit is set) | 0-5              | 01-1B-19-00-00-00 01-80-C2-00-00-0E       | All PTP messages can use any of the fol- lowing multicast addresses: 01-1B-19-00-00-00 01-80-C2-00-00-0E                                                                   |
| MAC Frame Type                                                                                                                                                       | 12, 13           | 0x88F7                                    | PTP Ethernet frame                                                                                                                                                         |
| PTP control field (IEEE Ver- sion 1)                                                                                                                                 | 45               | 0x00/0x01/0x02/ 0x03/0x04                 | 0x00 - SYNC 0x01 - Delay_Req 0x02 - Follow_Up 0x03 - Delay_Resp 0x04 - Management                                                                                          |
| PTP Message Type Field (IEEE version 2)                                                                                                                              | 14 (nibble)      | 0x0/0x1/0x2/0x3/0x8/0x9/0x A/0xB/ 0xC/0xD | 0x0 - SYNC 0x1 - Delay_Req 0x2 - Pdelay_Req 0x3 - Pdelay_Resp 0x8 - Follow_Up 0x9 - Delay_Resp 0xA - Pdelay_Resp_Follow_Up 0xB - Announce 0xC - Signaling 0xD - Management |
| PTP Version                                                                                                                                                          | 15(nibble)       | 0x1 or 0x2                                | 0x1 - Supports PTP version 1 0x2 - Supports PTP version 2                                                                                                                  |

## Auxiliary Time Stamp Snapshot

The auxiliary snapshot feature stores snapshots of the system time whenever a rising edge is detected on the EMAC\_PTPAUXIN[n] pins.

The PTP stores 64 bits of captured time stamp in a 4-deep FIFO. When a snapshot is stored, the PTP indicates this event to the EMAC with the auxiliary snapshot interrupt. The EMAC\_TM\_STMPSTAT.ATSTS bit is set. The value of the snapshot is read through the EMAC\_TM\_AUXSTMP\_SEC and EMAC\_TM\_AUXSTMP\_NSEC registers. If the FIFO becomes full and an external trigger to take the snapshot is asserted, then the snapshot trigger-missed status is set in the EMAC\_TM\_STMPSTAT.ATSSTM bit. The latest snapshot is not written to the FIFO when it is full.

When a host reads the 64-bit time stamp from the FIFO through the EMAC\_TM\_AUXSTMP\_SEC and EMAC\_TM\_AUXSTMP\_NSEC registers, the space becomes available to store the next snapshot.

NOTE: A space in the FIFO is created whenever the EMAC\_TM\_AUXSTMP\_SEC register is read. Therefore, read the EMAC\_TM\_AUXSTMP\_NSEC register before reading the EMAC\_TM\_AUXSTMP\_SEC register.

The program can clear the FIFO by setting the EMAC\_TM\_CTL.ATSFC bit. When multiple snapshots are present in the FIFO, the EMAC\_TM\_STMPSTAT.ATSNS bits indicate the count.

NOTE: The minimum gap between two events on the EMAC\_PTPAUXIN[n] pin must be 4 cycles of PTP\_CLK + 3 cycles of CLKO7 ). Otherwise, the logic misses the rising-edge of the trigger.

## System Time

To get a snapshot of the time, the EMAC requires a reference time in 64-bit format as defined in the IEEE 1588 specification. The PTP module maintains 80-bit time, known as system time. The PTP clock updates system time.

The 80-bit timing reference is split into the following three registers:

- EMAC\_TM\_NSEC - 32-bit nanoseconds register which provides time in nanoseconds
- EMAC\_TM\_SEC - 32-bit seconds register which provides time in seconds
- EMAC\_TM\_HISEC - 16-bit high seconds register which provides time beyond the seconds register. The IEEE 1588 standard does not include this register. Its use is application-specific.

The 64-bit system time (seconds and nanoseconds) is the source for taking time stamps for Ethernet frames being transmitted or received at the RMII.

Since the PTP clock frequency does not correspond to a 1ns period, the EMAC\_TM\_NSEC register is incremented with a value equal to the PTP clock period for every PTP clock cycle. The function uses the EMAC\_TM\_SUBSEC register. The EMAC\_TM\_NSEC value is incremented with the value programmed in EMAC\_TM\_SUBSEC register every PTP clock cycle.

Whenever the EMAC\_TM\_SEC register overflows from 0xFFFFFFFF to 0x00000000, the seconds overflow interrupt is triggered. The EMAC uses the EMAC\_TM\_STMPSTAT.TSSOVF bit to indicate the event. After a seconds overflow, the EMAC\_TM\_HISEC register increments by one.

The system time module supports the following two types of rollover modes for the EMAC\_TM\_NSEC register.

- Digital rollover mode. The maximum value in the nanoseconds field is 0x3B9AC9FF , that is, 10 9  nanoseconds. After it reaches this value, the EMAC\_TM\_SEC register increments and the EMAC\_TM\_NSEC register restarts counting from zero. Accuracy in digital rollover mode it is 1 ns per bit.
- Binary rollover mode. The nanoseconds field rolls over and increments the seconds field after the value reaches 0x7FFFFFFF. Accuracy in binary rollover mode is ~0.466 ns per bit.

## System Time Adjustment

The following sections describe the process for system time adjustment.

## System Time Initialization

System time can be initialized with 64-bit time when the PTP module is enabled. The initial value is written to the EMAC\_TM\_SECUPDT and EMAC\_TM\_NSECUPDT system time update registers. The system time counter is written with the value in the registers when the EMAC\_TM\_CTL.TSINIT bit is set.

## Coarse Correction Method

If the completer system time has an offset based on the system time of the requester, then the coarse correction method can correct it. The time offset value is written to the EMAC\_TM\_SECUPDT and EMAC\_TM\_NSECUPDT registers. The offset value is then added to or subtracted from the system time when the EMAC\_TM\_CTL.TSUPDT bit is set. Use the EMAC\_TM\_NSECUPDT.ADDSUB bit to choose addition or subtraction. System time correction occurs in one clock cycle using the coarse correction method.

NOTE: During subtraction, the EMAC\_TM\_SECUPDT register value must be less than the value of the EMAC\_TM\_SEC register. Check the value prior to subtracting using coarse correction.

## Fine Correction Method

If a target PTP clock frequency has a drift based on the controller PTP clock (as defined in IEEE 1588), it can be corrected using the fine correction method. Using this method, system time is corrected over a period (unlike coarse correction where it happens in one clock cycle). This correction helps maintain linear time and does not introduce drastic changes (or a large jitter) in the reference time between PTP sync message intervals.

Using this method, an accumulator sums the contents of the EMAC\_TM\_ADDEND register. The System Time Update, Fine Correction Method figure shows the method. The arithmetic-carry that the accumulator generates acts as a pulse to increment the system time counter. The accumulator and the addend are 32-bit registers. Here, the accumulator acts as a high-precision frequency divider.

Figure 29-21: System Time Update, Fine Correction Method

<!-- image -->

## Calculating Addend Value

This section describes the fine correction process for system time.

In this example, the requester clock runs at 50 MHz and the completer clock has drifted to 66 MHz. The goal is to adjust the completer system time to 50 MHz, so that the completer PTP module synchronizes with the requester. Using the figure in Fine Correction Method, the nanoseconds increment signal runs at 50 MHz. The nanoseconds increment is the carry from accumulator register. The addend value increments the carry from accumulator at the rate of the completer clock (66 MHz).

The accumulator overflows and generates a carry every N addend value, so N × Addend = 2 32 .

The accumulator increments at 66 MHz. This addition brings the carry to 50 MHz N = 66/50 = 1.32.

The addend = 2 32 /1.32 = 0xC1F07C1F.

Therefore, if addend is programmed with 0xC1F07C1F, the completer system time runs at 50 MHz which synchronizes with the requester.

In the Fine Correction Method figure, the subsecond increment is the value programmed in the EMAC\_TM\_SUBSEC register which increments the EMAC\_TM\_NSEC register according to the frequency of the nanoseconds increment signal.

In the example, the sub second increment is 20 (for digital rollover) or 43 (for binary rollover). This addition increments the EMAC\_TM\_NSEC register by 20 ns (1/50 MHz).

The software must calculate the drift in frequency and update the EMAC\_TM\_ADDEND register accordingly.

NOTE: The PTP reference clock is the clock at which the system time is updated. When the EMAC\_TM\_CTL.TSCFUPDT bit is set to 0, this clock equals the PTP clock. Using fine correction, the PTP reference clock is generated on the nanoseconds increment signal at which the system time updates.

## Target Time Trigger (Alarm)

The PTP module provides an alarm function by triggering an alarm at a preset time. It sets the EMAC\_TM\_STMPSTAT.TSTARGT0 bit when the system time matches the value of the EMAC\_TM\_PPS0TGTM and EMAC\_TM\_PPS0NTGTM registers. This trigger can generate an interrupt and command the flexible PPS module to start or stop PPS output, depending on value programmed in EMAC\_TM\_PPSCTL.TRGTMODSEL0 bits.

The trigger is enabled by setting EMAC\_TM\_CTL.TSTRIG bit. Once an alarm has occurred, if the PTP needs another alarm, the software must:

- Clear the status bit
- Reprogram the EMAC\_TM\_PPS0TGTM and EMAC\_TM\_PPS0NTGTM registers to a future value, and
- Set the EMAC\_TM\_CTL.TSTRIG bit

If the time programmed in the target time registers has elapsed, then a target time programming error is indicated by setting the EMAC\_TM\_STMPSTAT.TSTRGTERR0 bit.

The alarm time is represented in absolute units, not relative units. For example, if the software must generate an alarm after 10 seconds, it must read the current system time value. Then, the software must add the number corresponding to 10 seconds, and write the result back to the target time registers.

NOTE: The EMAC\_TM\_CTL.TSTRIG bit is common for all the four PPS outputs and the reset values of the EMAC\_TM\_PPSCTL.TRGTMODSEL0 , EMAC\_TM\_PPSCTL.TRGTMODSEL1 , EMAC\_TM\_PPSCTL.TRGTMODSEL2 , and EMAC\_TM\_PPSCTL.TRGTMODSEL3 bit fields is zero. Therefore, the target time alarm interrupt is active for all PPS outputs, by default. To avoid spurious interrupts, for unused PPS outputs configure the value of the TRGTMDOSELx bit field in the EMAC\_TM\_PPSCTL register from 0x0 to 0x3.

## Pulse-Per-Second (PPS)

Pulse-per-second (PPS) is a physical representation of system time. It consists of a single pulse or train of pulses. The PTP uses PPS for extra synchronization or to monitor the synchronization performance between clocks. With proper configuration, the PTP module can generate PPS signals that are output on the EMAC\_PTPPPS[n] pins. The PTP supports two kinds of PPS output, fixed and flexible.

## Fixed Pulse-Per-Second Output

The EMAC supports fixed pulse-per-second (PPS) output that indicates 1-second intervals (default). Change the frequency of the PPS output by configuring the EMAC\_TM\_PPSCTL.PPSCTL0 bits. The default value for these bits is 0000, which configures a 1Hz signal with a pulse width equal to the period of the PTP clock.

The PPS Output Frequencies table shows various PPS output frequencies.

Table 29-47: PPS Output Frequencies

| PPSCTL Bit Setting   | Binary Rollover   | Digital Rollover   |
|----------------------|-------------------|--------------------|
| 0001                 | 2 Hz              | 1 Hz               |
| 0010                 | 4 Hz              | 2 Hz               |
| 0011                 | 8 Hz              | 4 Hz               |
| ...                  | ...               | ...                |
| 1111                 | 32.768 kHz        | 16.384 kHz         |

In binary rollover mode, the PPS output has a duty cycle of 50% with these frequencies.

In digital rollover mode, the PPS output frequency is an average number. The actual clock is a different frequency that is synchronized every second. PPS output pulses have different periods and duty cycles and this behavior is because of the non-linear toggling of the bits in digital rollover mode. For example:

- When EMAC\_TM\_PPSCTL.PPSCTL0 = 0001, the PPS (1 Hz) has a low period of 537 ms and a high period of 463 ms.
- When EMAC\_TM\_PPSCTL.PPSCTL0 = 0010, the PPS (2 Hz) is a sequence of:
- One clock of 50-percent duty cycle and 537-ms period
- Second clock of 463-ms period (268 ms low and 195 ms high)
- When EMAC\_TM\_PPSCTL.PPSCTL0 = 0011, the PPS (4 Hz) is a sequence of:
- Three clocks of 50-percent duty cycle and 268-ms period
- Fourth clock of 195-ms period (134 ms low and 61 ms high)

## Flexible Pulse-Per-Second Output

The EMAC also provides the flexibility to program the start or stop time, width, and interval of the pulse generated on the PPS output. Enable this feature, called flexible PPS, by setting the EMAC\_TM\_PPSCTL.PPSEN bit.

The flexible PPS output options are:

- Supports programming the start point of the single pulse and start and stop points of the pulse train in terms of system time. The target time registers program the start and stop time.
- Supports programming the stop time in advance. Programs can configure the stop time before the actual start time has elapsed.

- Supports programming the width, between the rising edge and corresponding falling edge of the PPS signal output, in terms of number of units of subsecond increment. This value is configured in the EMAC\_TM\_SUBSEC register.
- Supports programming the interval, between the rising edges of PPS signal, in terms of number of units of subsecond increment. This value is configured in the EMAC\_TM\_SUBSEC register.
- Provides the option to cancel the programmed PPS start or stop request.
- Indicates error if the start or stop time programmed has already elapsed.

## PPS Start or Stop Time

Start time can initially be programmed in the target time registers. If necessary, the start or stop time can be programmed again but only after the earlier programmed value is synchronized to the PTP clock domain. The EMAC\_TM\_PPS0NTGTM.TSTRBUSY bit indicates the status of synchronization. Programs can configure the start or stop time in advance, even before the earlier stop or start time has elapsed.

Program the start or stop time with advanced system time to ensure proper PPS signal output. If the application programs a start or stop time that has already elapsed, then the EMAC sets the EMAC\_TM\_STMPSTAT.TSTRGTERR0 bit, indicating the error. If enabled, the EMAC also sets the target time trigger (alarm) interrupt event. The application can cancel the start or stop request only if the corresponding start or stop time has not elapsed. If the time has elapsed, the cancel command has no effect.

## PPS Width and Interval

The PPS width and interval are programmed in terms of granularity of system time, that is, the number of the units of subsecond increment value. For example, with the PTP reference clock of 50 MHz, a PPS pulse width of 40 ns, and an interval of 100 ns, program the width and interval to 2 and 5, respectively.

Use a faster PTP reference clock to achieve smaller granularity. Before commanding to trigger a pulse or pulse train on the PPS output, programs must configure or update the interval and width of the PPS signal output.

## PPS Command

When the PPS module has a flexible PPS output configuration, the PTP can use the EMAC\_TM\_PPSCTL.PPSCTL0 bits to command the PPS module to use any of the flexible PPS features.

Programming these bits with a non-zero value instructs the PPS module to initiate an event. Once the command transfers or synchronizes to the PTP clock domain, these bits clear automatically. Software must ensure that these bits are programmed only when they are all-zero.

The Flexible PPS Output Commands table explains the different commands and their function.

Table 29-48: Flexible PPS Output Commands

|   PPSCTL (Bits 3-0) | Command            | Description                                                                                                                            |
|---------------------|--------------------|----------------------------------------------------------------------------------------------------------------------------------------|
|                0000 | No Command         | No Command                                                                                                                             |
|                0001 | Start Single Pulse | Generates single pulse rising at start point defined in target time registers and of dura- tion defined in EMAC_TM_PPS0WIDTH register. |

Table 29-48: Flexible PPS Output Commands (Continued)

| PPSCTL (Bits 3-0)   | Command                        | Description                                                                                                                                                                                                                                                                                                                                                                            |
|---------------------|--------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0010                | Start Pulse Train              | Generates train of pulses rising at the start time configured in the target time registers, of duration configured in the EMAC_TM_PPS0WIDTH register. The train of pulses re- peats at the interval configured in the EMAC_TM_PPS0INTVL register. By default, the PPS pulse train is free-running unless stopped by stop pulse train at time or stop pulse train immediately commands. |
| 0011                | Cancel Start                   | Cancels the start single pulse and start pulse train commands when the system time has not crossed the programmed start time.                                                                                                                                                                                                                                                          |
| 0100                | Stop Pulse Train at Time       | Stops the train of pulses initiated by the command for start pulse train after the time programmed in the target time registers elapses.                                                                                                                                                                                                                                               |
| 0101                | Stop Pulse Train Im- mediately | Immediately stops the train of pulses initiated by the command for start pulse train.                                                                                                                                                                                                                                                                                                  |
| 0110                | Cancel Stop Pulse Train        | Cancels the stop pulse train at time command when the programmed stop time has not elapsed. The PPS pulse train becomes free-running on the successful execution of this command.                                                                                                                                                                                                      |
| 0111-1111           | Reserved                       | Reserved                                                                                                                                                                                                                                                                                                                                                                               |

## PTP Interrupts

Set the EMAC\_IMSK.TS bit to enable interrupts from the PTP module. The EMAC uses the EMAC\_ISTAT.TS bit to indicate the status of the interrupt. The PTP supports the following three types of interrupts.

## Auxiliary Snapshot Trigger

When an external event occurs on the EMAC\_PTPAUXIN[n] pins and a time stamp snapshot occurs, an auxiliary snapshot interrupt is triggered. The EMAC uses the EMAC\_TM\_STMPSTAT.ATSTS bit to indicate the interrupt.

## Target Time Reached

This interrupt is triggered when the system time becomes equal to the value written in the EMAC\_TM\_PPS0NTGTM and EMAC\_TM\_PPS0NTGTM registers. Enable or disable the interrupt using the EMAC\_TM\_CTL.TSTRIG and EMAC\_TM\_PPSCTL.TRGTMODSEL0 bits. This interrupt can be used as an alarm and is indicated on the EMAC\_TM\_STMPSTAT.TSTARGT0 bit.

## System Time Seconds Register Overflow

This interrupt is triggered when the EMAC\_TM\_SEC register overflows from 0xFFFF FFFF to 0x0000 0000. This interrupt is indicated on the EMAC\_TM\_STMPSTAT.TSSOVF bit. As soon as EMAC\_TM\_SEC register overflows, the EMAC\_TM\_HISEC register increments by one.

## Audio Video Data Transmission

The audio video (AV) feature enables transmission of time-sensitive traffic over bridged local area networks (LANs). The following standards define the various aspects of the AV feature implementation.

- IEEE 802.1Qav-2009: Allows the bridges to provide time-sensitive and loss-sensitive real-time audio video data transmission (AV traffic). It specifies the priority regeneration and controlled bandwidth queue draining algorithms that are used in bridges and AV traffic sources.
- IEEE 802.1Qat-2009: Allows reservation of the network resources for specific traffic streams traversing a bridged local area network.
- IEEE 802.1AS-2011: Specifies the protocol and procedures used to ensure that the synchronization requirements are met for time-sensitive applications. For example, audio and video and across bridged and virtualbridged LANs. Virtual-bridged LANs consist of LAN media where the transmission delays are fixed and symmetrical. For example, IEEE 802.3 full-duplex links include the maintenance of synchronized time during normal operation followed by addition, removal, or failure of network components and network reconfiguration.

As shown in the Transmit and Receive Path Block Diagram figure, one SCB controller interface connects to three DMA channels (channel 0, channel 1, and channel 2). The DMA arbiter helps in arbitration of all the paths (transmit and receive) in channel 0, channel 1 and channel 2. Each channel has a separate control and status register (CSR) for managing the transmit and receive functions, descriptor handling, and interrupt handling.

Figure 29-22: Transmit and Receive Path Block Diagram

<!-- image -->

## Transmit Path Functions

The transmit path of channel 0 supports strict-priority algorithm and is used for best-effort traffic. For a channel, the strict-priority algorithm determines that a frame is available for transmission if the channel contains one or more frames. When the threshold mode for EMAC MFL Tx FIFO is enabled, the strict-priority algorithm determines that a frame is available for transmission. The algorithm determines when the channel contains a partial frame of size equal to the programmed threshold limit.

The transmit paths of channel 1 and channel 2 support traffic management by using the credit-based shaper algorithm. For a channel, the credit-based shaper algorithm determines that a frame is available for transmission if the following conditions are true:

- The channel contains one or more frames.

- The credit for the channel is positive as per the algorithm.

Programs can disable the credit-based shaper algorithm for all channels or for lower-priority channels. The creditbased shaper algorithm can be disabled for channel 1 and channel 2 or for channel 1 only. When the credit-based shaper algorithm for a channel is disabled, the channel uses the default strict-priority algorithm.

Each transmit DMA has a separate descriptor chain for fetching the transmit data. The transmit channel that receives access to the system bus depends on the DMA arbiter. For information about the DMA arbiter, see DMA Arbiter.

The transmit path has separate FIFOs for each channel, as shown in the Transmit and Receive Path Block Diagram figure. The data fetched by the DMA is placed in the respective FIFO. The traffic management and scheduler unit (TMS) controls which FIFO data the MAC transmits. If the credit-based shaper algorithm is enabled for a channel (1 or 2), then the corresponding channel is selected for transmission when the following conditions are true:

- The frame is available in the channel and has a positive or zero credit.
- The higher priority channel has no frame waiting in the FIFO.

If the credit-based shaper algorithm is disabled for all channels, then the frame awaiting transmission from a channel is selected. The selection is made based on the priority scheme described in the Fixed Priority Scheme for DMA Channels table.

## Receive Path Functions

To differentiate between the AV and non-AV traffic, the MAC provides a status. The status indicates if data is an AV packet and identifies its corresponding VLAN Priority tag value. This status is updated in the extended status field of the receive descriptor as explained in "Receive Descriptor". The EMAC\_MAC\_AVCTL.AVT bit field specifies a value that is compared with the EtherType field of the incoming Ethernet frame to detect an AV packet. The AV packets can be of the following two types:

- AV data packets. The AV data packets are always tagged. The tagged AV control packets are received based on the programmed priority value. The EMAC\_MAC\_AVCTL.AVP bit field specifies the channel to which an AV packet with a given priority must be sent.
- AV control packets. The AV control packets are either tagged or untagged. The untagged AV control packets are received on channel 0 by default. To receive these packets on channel 1 or channel 2, program the EMAC\_MAC\_AVCTL.AVCH bit field. Similar to the AV data packets, the tagged AV control packets are received based on the programmed priority value.

The following describes how tagged AV control packets and AV data packets are sent to a channel.

Table 29-49: AV Packets

| Receive paths of channel 1 and channel 2 are en- abled.                             | Channel 2 The following packets, with priority value greater than or equal to the value program- med in EMAC_MAC_AVCTL.AVP bit field, are received on channel 2: AV packets Non-AV tagged packets (if EMAC_MAC_AVCTL.VQE bit is set)   |
|-------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Receive paths of channel 1 and channel 2 are en- abled.                             | Channel 1 The following packets, with priority value less than the value programmed in AV MAC control register, are received on channel 1: AV packets Non-AV tagged packets (if EMAC_MAC_AVCTL.VQE bit is set)                         |
| Receive paths of channel 1 and channel 2 are en- abled.                             | Channel 0 All other packets are received on channel 0.                                                                                                                                                                                 |
| For example, priority value of 3 is programmed in the EMAC_MAC_AVCTL.AVP bit field. | Channel 2 The following packets with priority value from 3 to 7 are received on channel 2: AV packets Non-AV tagged packets (if the EMAC_MAC_AVCTL.VQE bit is set)                                                                     |
| For example, priority value of 3 is programmed in the EMAC_MAC_AVCTL.AVP bit field. | Channel 1 The following packets with priority value 2 are received on channel 1: AV packets Non-AV tagged packets (if the EMAC_MAC_AVCTL.VQE bit is set)                                                                               |
| For example, priority value of 3 is programmed in the EMAC_MAC_AVCTL.AVP bit field. | Channel 0 All other packets are received on channel 0.                                                                                                                                                                                 |

## DMA Arbiter

The DMA arbitrates between the transmit and receive paths of DMA channel 0, channel 1, and channel 2 for accessing descriptors and data buffers.

The fixed priority scheme is the default priority scheme for the DMA channels. In fixed priority scheme, the highest priority channel (channel 2) always wins the arbitration whenever it requests the bus. The Fixed Priority Scheme for DMA Channels table provides information about the priority levels of the DMA channels.

Table 29-50: Fixed Priority Scheme for DMA Channels

| Priority Level   | Channel   |
|------------------|-----------|
| 0 (low)          | Channel 0 |
| 1                | Channel 1 |
| 2 (high)         | Channel 2 |

## Slot Number Function

The slot number function schedules the data fetching by DMA from the system memory. This feature is useful when the source AV data must transmit at specific intervals. The transmit descriptor word 0 (TDES0) [6:3] bits program the slot number at which the DMA fetches the data from system memory. This 4-bit field allows the host to schedule data up to 16 slots of 125 micro-second each. This field is applicable only for the AV channels (channel 1 and channel 2).

When DMA fetches a transmit descriptor, it compares the slot number of the transmit descriptor with the internally generated reference slot interval. The slot interval is a counter that updates every 125 usec of the IEEE 1588 system time. In addition, the slot interval counter is initialized to zero when the value in the EMAC\_TM\_SEC register increments, that is, EMAC\_TM\_NSEC rolls over. The DMA fetches the data only if it matches the current slot or the next slot. The DMA remains in the descriptor fetch state until there is a match.

Programs can also set the EMAC\_DMA1\_CHSFCS.ASC bit to enable the DMA to fetch the data only if it matches the current slot or the next two slots.

NOTE: If the slot number in the descriptor is less than the reference slot number, the DMA takes it as a future slot.

Programs can enable the check for slot number by setting the EMAC\_DMA1\_CHSFCS.ESC bit. When this check is not enabled, the packets are fetched immediately after the descriptor is read. Programs can read the EMAC\_DMA1\_CHSFCS.RSN bit field to discover the value of the reference slot number in DMA.

## Interrupts

Each DMA channel has it own dedicated interrupt channel, the software can take advantage of it for processing various DMA channels.

## Credit-Based Shaper Algorithm Functions

The Traffic Manager and Scheduler (TMS) block (shown in the Transmit and Receive Path Block Diagram figure) uses the credit-based shaper algorithm to arbitrate the AV traffic in all channels and the legacy Ethernet traffic in channel 0. Channel 1 and channel 2 can be programmed to use the credit-based shaper algorithm.The following sections provide information about implementing the Credit-Based Shaper Algorithm:

## Credit Value

The credit value is accumulated every transmit clock cycle, that is, 40 ns for 100 Mbps or 8 ns for 1000 Mbps. The credit to be added or subtracted per cycle can be fractional, based on the required idleSlope and sendSlope values as described in the following table.

Table 29-51: idleSlope and sendSlope Values

| Mode     | Values                                                                                                                                    | Description                                                                                                                                                                                                                                   |
|----------|-------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 100 Mbps | portTransmitRate = 100 Mbps idleSlope = 70 Mbps (assuming 70% bandwidth reserved for a higher priority traffic class) sendSlope = 30 Mbps | Credit = 2.8 bits accumulates per cycle (40 ns) for the higher priority traffic class when besteffort frame is being transmitted. Credit = 1.2 bits drains per cycle (40 ns) when higher priority traffic class frame is be- ing transmitted. |

The DMA stores the channel traffic in the respective Tx FIFO based on the slot number in the transmit descriptor (if enabled) or depending on the bandwidth availability on the SCB.

The credit for a channel builds up only when the frame is available but it cannot be transmitted because the MAC is sending a frame from another channel. The EMAC supports another mode in which the credit can build up in advance for a channel in which no frame is available in its FIFO. This enables sending a burst of high priority traffic in a channel as soon as data is available. This can be enabled with the EMAC\_DMA1\_CHCBSCTL.CC / EMAC\_DMA2\_CHCBSCTL.CC bit of the channel 1 and channel 2 CBS control registers.

When the EMAC\_DMA1\_CHCBSCTL.CC / EMAC\_DMA2\_CHCBSCTL.CC bit is reset, the accumulated credit parameter in the Credit-Based Shaper Algorithm is set to zero if there is positive credit and there is no frame to transmit in a channel. The credit does not accumulate when there is no frame waiting in a channel and other channels are transmitting. When the EMAC\_DMA1\_CHCBSCTL.CC / EMAC\_DMA2\_CHCBSCTL.CC bit is set, the accumulated credit parameter in the Credit-Based Shaper Algorithm is not reset to zero if there is positive credit and no frame to transmit in a channel. The credit accumulates even when there is no frame waiting in a channel and other channels are transmitting.

## idleSlopeCredit and sendSlopeCredit Values

The software must program the idleSlopeCredit and sendSlopeCredit values. The programmed values should be the credit accumulated or drained per clock cycle scaled by 1024, such as, 2.8x1024=2867 and 1.2x1024=1229 respectively. In addition, the software must program the hiCredit and loCredit values, scaled by 1024, to adjust for scaling of the idleSlopeCredit and sendSlopeCredit values.

This means that if computed hiCredit and loCredit values are 12000 bits and 3036 bits respectively, then the values to be programmed in the EMAC\_DMA1\_CHHIC / EMAC\_DMA2\_CHHIC and EMAC\_DMA1\_CHLOC / EMAC\_DMA2\_CHLOC registers are 12000x1024 bits and two's complement of 3036x1024 respectively.

## Bandwidth Status

The hardware maintains the status of the actual bandwidth consumed by each higher priority channel (channel 1 and channel 2) in the CBS status registers ( EMAC\_DMA1\_CHCBSSTAT / EMAC\_DMA2\_CHCBSSTAT ). This allows the software to estimate the average bandwidth consumed by numerically higher traffic classes as compared to the reserved bandwidth.

The CBS status register gives the average number of bits transmitted during the previous programmed slot interval (1, 2, 4, 8, or 16 slots of 125 us) in a channel. The status register is updated even if the Credit-Based Shaper Algorithm is not enabled for a channel. The number of slots over which the average bits transmitted per slot are computed is programmed in the EMAC\_DMA1\_CHCBSCTL.SLC / EMAC\_DMA2\_CHCBSCTL.SLC bits. For example, if these bit fields are programmed for two slots, then the average bits are computed over slot numbers 0-1, 2-3, 4-5, and so on.

The value programmed in the EMAC\_DMA1\_CHISC / EMAC\_DMA2\_CHISC register of a channel is proportional to the bandwidth reserved for the channel. The software can allocate any bandwidth that is not used by the higher priority channel to the reserved bandwidth of the lower priority channel.

A lower priority channel, using the Credit-Based Shaper Algorithm, cannot use the unused reserved bandwidth of any higher priority channel that is using the Credit-Based Shaper Algorithm. However, a lower priority channel that is using the strict-priority algorithm can use the unused reserved bandwidth of any higher priority channel that uses the Credit-Based Shaper Algorithm. For example, channel 1 and channel 2 use the Credit-Based Shaper Algorithm (with reserved bandwidth of 50% and 25% respectively) and channel 0 uses the strict-priority algorithm. If channel 1 uses only 40% of the reserved bandwidth, then the remaining 10% is used by channel 0. The channel 2 cannot exceed the reserved bandwidth of 25%.

## Energy-Efficient Ethernet

Energy-Efficient Ethernet (EEE) is an optional operational mode that enables the IEEE 802.3 Media Access Control (MAC) sublayer along with a family of Physical layers to operate in the Low-Power Idle (LPI) mode. The EEE operational mode supports the IEEE 802.3 MAC operation at 100 Mbps, 1000 Mbps, and 10 Gbps. The MAC supports the IEEE Standard 802.3az-2010 for EEE.

The LPI mode allows power saving by switching off parts of the communication device functionality when there is no data awaiting transmission and receipt. The systems on both sides of the link can disable some functionality and save power during the periods of low-link utilization. The MAC controls whether the system should enter or exit the LPI mode and communicates this to the PHY.

The EEE specifies the capabilities negotiation methods that the link partners can use to determine whether EEE is supported and then select the set of parameters that common to both devices.

- NOTE: The EEE feature is not supported when the MAC is configured to use the RMII interface. You should activate the EEE mode only when the MAC is operating with RGMII interface.
- NOTE: According to the Energy-Efficient Ethernet standard (IEEE 802.3az-2010), the LPI mode is supported only in the full-duplex mode. Therefore, you should not enable the LPI mode when the MAC Transmitter is configured for the half-duplex mode.

## Transmit Path Functions

In the transmit path, the software must set the EMAC\_LPI\_CTLSTAT.LPIEN bit to indicate to the MAC to stop transmission and initiate the LPI protocol. The MAC completes the transmission in progress, generates its transmission status, and then starts transmitting the LPI pattern instead of the IDLE pattern during Interframe gap (IFG).

To make the PHY enter the LPI state, the MAC performs the following tasks:

1. De-asserts the ETH0\_TXCTL\_TXEN signal.
2. Asserts the TX\_ER signal.
3. Sets EMAC\_TXD[n] [3:0] to 0x1 (for 100 Mbps) or EMAC\_TXD[n] [7:0] to 0x01 (for 1000 Mbps).

NOTE: The MAC maintains the same state of the TX\_EN, TX\_ER, and EMAC\_TXD[n] signals for the entire duration during which the PHY remains in the LPI state.

4. Updates the status using the EMAC\_LPI\_CTLSTAT.TLPIEN bit and generates an interrupt.

To bring the PHY out of the LPI state, that is, when the software resets the EMAC\_LPI\_CTLSTAT.LPIEN bit, the MAC performs the following tasks:

1. Stops transmitting the LPI pattern and starts transmitting the IDLE pattern.
2. Starts the LPI TW TIMER: The MAC cannot start the transmission until the wake-up time specified for the PHY expires. The auto-negotiated wake-up interval is programmed using the EMAC\_LPI\_TMRSCTL.TWT bit field.
3. Updates the LPI exit status using the EMAC\_LPI\_CTLSTAT.TLPIEX bit and generates an interrupt.

The LPI Transitions (Transmit) figure shows the behavior of TX\_EN, TX\_ER, and EMAC\_TXD[n] [3:0] signals during the LPI mode transitions.

Figure 29-23: LPI Transitions (Transmit)

<!-- image -->

NOTE: The MAC does not stop the TX\_CLK clock.

- NOTE: If the MAC is in the Tx LPI mode and the Tx clock is stopped, the application should not write to CSR registers that are synchronized to Tx clock domain.
- NOTE: If the MAC is in the LPI mode and the host issues a soft reset or hard reset, the MAC transmitter comes out of the LPI mode.

## LPI Timers

LPI LS TIMER