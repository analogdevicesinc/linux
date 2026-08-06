# Ethernet Media Access Controller (EMAC) — ADSP-2159x_SC592_SC594 EMAC Register Descriptions

<!-- source: 036_Ethernet_Media_Access_Controller_EMAC_ADSP-2159x_SC592_SC594.pdf | original pages 1757–2046 -->

```
{ unsigned int        Status;        //TDES0 OR RDES0 unsigned int        ControlDesc;      //TDES1 OR RDES1 unsigned int        StartAddr;     //TDES2 OR RDES2 struct EMAC_DMADESC_CHAIN     *pNextDesc;  //TDES3 OR RDES3 #ifdef CHECKSUM_OFFLOAD struct EMAC_EXT_STAT     ExtendedStat; #endif } EMAC_DMADESC_CHAIN; /* Extended Status Descriptor with PTP not enabled*/ typedef struct EMAC_EXT_STAT { #ifdef RX_DESC unsigned int        CheckSumStat;    //RDES4 #ifdef TX_DESC unsigned int        Reserved;     //TDES4 #endif unsigned int        Reserved;        //RDES5 OR TDES5 unsigned int        Reserved;     //RDES6 OR TDES6 unsigned int        Reserved;     //RDES7 OR TDES7 } EMAC_EXT_STAT;
```

## PTP Header Structure in C

The following code is an example of the PTP message format.

```
/* PTP Message Format (Refer to PTP Frame Over IPv4)*/ typedef struct EMAC_PTP_HEADER { unsigned char    messageType:4,    //PTP Version 2 message type transportSpecific:4; unsigned char    versionPTP;       //PTP Version (1 or 2) unsigned short    messageLength; unsigned char    domainNumber; unsigned char    RESERVED1; unsigned short    flagField; unsigned char    correctionField[8]; unsigned char    RESERVED2[4]; unsigned char    sourcePortIdentity[10]; unsigned short    sequenceid; unsigned char    controlField;     //PTP Version 1 message type unsigned char    logMessageInterval; }EMAC_PTP_HEADER;
```

## ADSP-2159x\_SC592\_SC594 EMAC Register Descriptions

Ethernet MAC (EMAC) contains the following registers.

Table 29-55: ADSP-2159x\_SC592\_SC594 EMAC Register List

| Name                 | Description                                       |
|----------------------|---------------------------------------------------|
| EMAC_ADDR0_HI        | MAC Address 0 High Register                       |
| EMAC_ADDR0_LO        | MAC Address 0 Low Register                        |
| EMAC_ADDR1_HI        | MAC Address 1 High Register                       |
| EMAC_ADDR1_LO        | MAC Address 1 Low Register                        |
| EMAC_DBG             | Debug Register                                    |
| EMAC_DMA0_BMMODE     | DMASCB Bus Mode Register                          |
| EMAC_DMA0_BMSTAT     | DMASCB Status Register                            |
| EMAC_DMA0_BUSMODE    | DMABus Mode Register                              |
| EMAC_DMA0_IEN        | DMAInterrupt Enable Register                      |
| EMAC_DMA0_MISS_FRM   | DMAMissed Frame Register                          |
| EMAC_DMA0_OPMODE     | DMAOperation Mode Register                        |
| EMAC_DMA0_RXBUF_CUR  | DMARx Buffer Current Register                     |
| EMAC_DMA0_RXDSC_ADDR | DMARx Descriptor List Address Register            |
| EMAC_DMA0_RXDSC_CUR  | DMARx Descriptor Current Register                 |
| EMAC_DMA0_RXIWDOG    | DMARx Interrupt Watch Dog Register                |
| EMAC_DMA0_RXPOLL     | DMARx Poll Demand register                        |
| EMAC_DMA0_STAT       | DMAStatus Register                                |
| EMAC_DMA0_TXBUF_CUR  | DMATx Buffer Current Register                     |
| EMAC_DMA0_TXDSC_ADDR | DMATx Descriptor List Address Register            |
| EMAC_DMA0_TXDSC_CUR  | DMATx Descriptor Current Register                 |
| EMAC_DMA0_TXPOLL     | DMATx Poll Demand Register                        |
| EMAC_DMA1_BUSMODE    | DMABus Mode Register                              |
| EMAC_DMA1_CHCBSCTL   | Channel 1 Credit Shaping Control Register         |
| EMAC_DMA1_CHCBSSTAT  | Channel 1 Average Traffic Transmitted Register    |
| EMAC_DMA1_CHHIC      | Channel 1 High Credit Value Register              |
| EMAC_DMA1_CHISC      | Channel 1 Idle Slope Credit Value Register        |
| EMAC_DMA1_CHLOC      | Channel 1 Low Credit Value Register               |
| EMAC_DMA1_CHSFCS     | Channel 1 Control Bits for Slot Function Register |
| EMAC_DMA1_CHSSC      | Channel 1 Send Slope Credit Value Register        |
| EMAC_DMA1_IEN        | DMAInterrupt Enable Register                      |
| EMAC_DMA1_MISS_FRM   | DMAMissed Frame Register                          |

Table 29-55: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                 | Description                                       |
|----------------------|---------------------------------------------------|
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
| EMAC_DMA2_STAT       | DMAStatus Register                                |
| EMAC_DMA2_TXBUF_CUR  | DMATx Buffer Current Register                     |
| EMAC_DMA2_TXDSC_ADDR | DMATx Descriptor List Address Register            |
| EMAC_DMA2_TXDSC_CUR  | DMATx Descriptor Current Register                 |

Table 29-55: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                | Description                                     |
|---------------------|-------------------------------------------------|
| EMAC_DMA2_TXPOLL    | DMATx Poll Demand Register                      |
| EMAC_FLOWCTL        | FLow Control Register                           |
| EMAC_GIGE_CTLSTAT   | RGMII Control and Status Register               |
| EMAC_HASHTBL_HI     | Hash Table High Register                        |
| EMAC_HASHTBL_LO     | Hash Table Low Register                         |
| EMAC_IMSK           | Interrupt Mask Register                         |
| EMAC_IPC_RXIMSK     | MMCIPC Rx Interrupt Mask Register               |
| EMAC_IPC_RXINT      | MMCIPC Rx Interrupt Register                    |
| EMAC_ISTAT          | Interrupt Status Register                       |
| EMAC_L3L4_CTL       | Layer3 and Layer4 Control Register              |
| EMAC_L3_ADDR0       | Layer 3 Address0 Register                       |
| EMAC_L3_ADDR1       | Layer 3 Address1 Register                       |
| EMAC_L3_ADDR2       | Layer 3 Address2 Register                       |
| EMAC_L3_ADDR3       | Layer 3 Address3 Register                       |
| EMAC_L4_ADDR        | Layer 4 Address Register                        |
| EMAC_LPI_CTLSTAT    | Low Power Idle Control and Status Register      |
| EMAC_LPI_TMRSCTL    | Low Power Idle Timeout Register                 |
| EMAC_MACCFG         | MAC Configuration Register                      |
| EMAC_MACFRMFILT     | MAC Rx Frame Filter Register                    |
| EMAC_MAC_AVCTL      | AV MAC Control Register                         |
| EMAC_MMC_CTL        | MMCControl Register                             |
| EMAC_MMC_RXIMSK     | MMCRxInterrupt Mask Register                    |
| EMAC_MMC_RXINT      | MMCRxInterrupt Register                         |
| EMAC_MMC_TXIMSK     | MMCTXInterrupt Mask Register                    |
| EMAC_MMC_TXINT      | MMCTxInterrupt Register                         |
| EMAC_RX1024TOMAX_GB | Rx 1024- to Max-Byte Frames (Good/Bad) Register |
| EMAC_RX128TO255_GB  | Rx 128- to 255-Byte Frames (Good/Bad) Register  |
| EMAC_RX256TO511_GB  | Rx 256- to 511-Byte Frames (Good/Bad) Register  |
| EMAC_RX512TO1023_GB | Rx 512- to 1023-Byte Frames (Good/Bad) Register |
| EMAC_RX64_GB        | Rx 64-Byte Frames (Good/Bad) Register           |
| EMAC_RX65TO127_GB   | Rx 65- to 127-Byte Frames (Good/Bad) Register   |

Table 29-55: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                    | Description                                    |
|-------------------------|------------------------------------------------|
| EMAC_RXALIGN_ERR        | Rx alignment Error Register                    |
| EMAC_RXBCASTFRM_G       | Rx Broadcast Frames (Good) Register            |
| EMAC_RXCRC_ERR          | Rx CRC Error Register                          |
| EMAC_RXCTLFRM_G         | Rx Good Control Frames Register                |
| EMAC_RXFIFO_OVF         | Rx FIFO Overflow Register                      |
| EMAC_RXFRMCNT_GB        | Rx Frame Count (Good/Bad) Register             |
| EMAC_RXICMP_ERR_FRM     | Rx ICMP Error Frames Register                  |
| EMAC_RXICMP_ERR_OCT     | Rx ICMP Error Octets Register                  |
| EMAC_RXICMP_GD_FRM      | Rx ICMP Good Frames Register                   |
| EMAC_RXICMP_GD_OCT      | Rx ICMP Good Octets Register                   |
| EMAC_RXIPV4_FRAG_FRM    | Rx IPv4 Datagrams Fragmented Frames Register   |
| EMAC_RXIPV4_FRAG_OCT    | Rx IPv4 Datagrams Fragmented Octets Register   |
| EMAC_RXIPV4_GD_FRM      | Rx IPv4 Datagrams (Good) Register              |
| EMAC_RXIPV4_GD_OCT      | Rx IPv4 Datagrams Good Octets Register         |
| EMAC_RXIPV4_HDR_ERR_FRM | Rx IPv4 Datagrams Header Errors Register       |
| EMAC_RXIPV4_HDR_ERR_OCT | Rx IPv4 Datagrams Header Errors Register       |
| EMAC_RXIPV4_NOPAY_FRM   | Rx IPv4 Datagrams No Payload Frame Register    |
| EMAC_RXIPV4_NOPAY_OCT   | Rx IPv4 Datagrams No Payload Octets Register   |
| EMAC_RXIPV4_UDSBL_FRM   | Rx IPv4 UDP Disabled Frames Register           |
| EMAC_RXIPV4_UDSBL_OCT   | Rx IPv4 UDP Disabled Octets Register           |
| EMAC_RXIPV6_GD_FRM      | Rx IPv6 Datagrams Good Frames Register         |
| EMAC_RXIPV6_GD_OCT      | Rx IPv6 Good Octets Register                   |
| EMAC_RXIPV6_HDR_ERR_FRM | Rx IPv6 Datagrams Header Error Frames Register |
| EMAC_RXIPV6_HDR_ERR_OCT | Rx IPv6 Header Errors Register                 |
| EMAC_RXIPV6_NOPAY_FRM   | Rx IPv6 Datagrams No Payload Frames Register   |
| EMAC_RXIPV6_NOPAY_OCT   | Rx IPv6 No Payload Octets Register             |
| EMAC_RXJAB_ERR          | Rx Jab Error Register                          |
| EMAC_RXLEN_ERR          | Rx Length Error Register                       |
| EMAC_RXMCASTFRM_G       | Rx Multicast Frames (Good) Register            |
| EMAC_RXOCTCNT_G         | Rx Octet Count (Good) Register                 |
| EMAC_RXOCTCNT_GB        | Rx Octet Count (Good/Bad) Register             |

Table 29-55: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                 | Description                                   |
|----------------------|-----------------------------------------------|
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
| EMAC_TM_PPS0NTGTM    | Time Stamp Target Time Nanoseconds Register   |
| EMAC_TM_PPS0TGTM     | Time Stamp Target Time Seconds Register       |
| EMAC_TM_PPS0WIDTH    | PPS Width Register                            |
| EMAC_TM_PPS1INTVL    | PPS 1 Interval Register                       |

Table 29-55: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name                | Description                                     |
|---------------------|-------------------------------------------------|
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
| EMAC_TXEXCESSDEF    | Tx Excess Deferral Register                     |
| EMAC_TXFRMCNT_G     | Tx Frame Count (Good) Register                  |
| EMAC_TXFRMCNT_GB    | Tx Frame Count (Good/Bad) Register              |
| EMAC_TXLATECOL      | Tx Late Collision Register                      |

Table 29-55: ADSP-2159x\_SC592\_SC594 EMAC Register List (Continued)

| Name               | Description                                     |
|--------------------|-------------------------------------------------|
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

## MAC Address 0 High Register

The EMAC\_ADDR0\_HI register holds the address 0 high bits.

Figure 29-24: EMAC\_ADDR0\_HI Register Diagram

<!-- image -->

Table 29-56: EMAC\_ADDR0\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | ADDR       | Address. The EMAC_ADDR0_HI.ADDR bits contain the upper 16 bits (47:32) of the 6-byte first MAC address. This address is used by the MAC for filtering for received frames and for inserting the MAC address in the Transmit Flow Control (PAUSE) Frames. |

## MAC Address 0 Low Register

The EMAC\_ADDR0\_LO register holds the address 0 low bits.

Figure 29-25: EMAC\_ADDR0\_LO Register Diagram

<!-- image -->

Table 29-57: EMAC\_ADDR0\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | ADDR       | Address. The EMAC_ADDR0_LO.ADDR bits contain the lower 32 bits of the 6-byte first MAC address. This address is used by the MAC for filtering for received frames and for in- serting the MAC address in the Transmit Flow Control (PAUSE) Frames. |

## MAC Address 1 High Register

The EMAC\_ADDR1\_HI register Contains the higher 16 bits of the second MAC address.

Figure 29-26: EMAC\_ADDR1\_HI Register Diagram

<!-- image -->

Table 29-58: EMAC\_ADDR1\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | AE         | Address Enable. The EMAC_ADDR1_HI.AE bit, When this bit is set, the address filter module uses the second MAC address for perfect filtering.        |
| 30 (R/W)           | SA         | Source Address. The EMAC_ADDR1_HI.SA bit, When this bit is set, the MAC Address1[47:0] is used to compare with the SA fields of the received frame. |
| 29:24 (R/W)        | MBC        | Mask byte control. The EMAC_ADDR1_HI.MBC bit, are mask control bits for comparison of each of the MAC Address bytes.                                |
| 15:0 (R/W)         | ADDRHI     | Mac address. The EMAC_ADDR1_HI.ADDRHI bit, contains the upper 16 bits (47:32) of the sec- ond 6-byte MAC address.                                   |

## MAC Address 1 Low Register

The EMAC\_ADDR1\_LO register Contains the lower 32 bits of the second MAC address.

Figure 29-27: EMAC\_ADDR1\_LO Register Diagram

<!-- image -->

Table 29-59: EMAC\_ADDR1\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | ADDRLO     | Mac Address. The EMAC_ADDR1_LO.ADDRLO bit, contains the lower 32 bits of the first 6-byte MAC address. This is used by the MAC for filtering the received frames and inserting the MAC address in the Transmit Flow Control (Pause) Frames. |

## Debug Register

The EMAC\_DBG register contains EMAC debug status information.

Figure 29-28: EMAC\_DBG Register Diagram

<!-- image -->

Table 29-60: EMAC\_DBG Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                         |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/NW)          | TXFIFOFULL  | Tx FIFO Full. The EMAC_DBG.TXFIFOFULL bit, when high, indicates that the MFL TxStatus FIFO is full, and the MFL cannot accept any more frames for transmission.                                                                                                                                 |
| 24 (R/NW)          | TXFIFONE    | Tx FIFO Not Empty. The EMAC_DBG.TXFIFONE bit, when high, indicates that the MFL TxFIFO is not empty and has some data left for transmission.                                                                                                                                                    |
| 22 (R/NW)          | TXFIFOACT   | Tx FIFO Active. The EMAC_DBG.TXFIFOACT bit, when high, indicates that the MFL TxFIFO write controller is active and transferring data to the TxFIFO.                                                                                                                                            |
| 21:20 (R/NW)       | TXFIFOCTLST | Tx FIFO Controller State. The EMAC_DBG.TXFIFOCTLST bits indicate the state of the TxFIFO read control- ler as: 00=IDLE state, 01=READ state (transferring data to MAC transmitter), 10=Waiting for TxStatus from MAC transmitter, and 11=Writing the received TxSta- tus or flushing the TxFIFO |

Table 29-60: EMAC\_DBG Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                      | Description/Enumeration                                                                                                                                                                     |
|--------------------|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/NW)          | TXPAUSE     | Tx Paused. The EMAC_DBG.TXPAUSE bit, when high, indicates that the MAC transmitter is in PAUSE condition (in full-duplex only) and does not schedule any frame for transmis- | Tx Paused. The EMAC_DBG.TXPAUSE bit, when high, indicates that the MAC transmitter is in PAUSE condition (in full-duplex only) and does not schedule any frame for transmis-                |
| 18:17 (R/NW)       | TXFRCTL     | Tx Frame Controller State. The EMAC_DBG.TXFRCTL bits indicate the state of the MAC transmit frame con- troller module.                                                       | Tx Frame Controller State. The EMAC_DBG.TXFRCTL bits indicate the state of the MAC transmit frame con- troller module.                                                                      |
| 18:17 (R/NW)       | TXFRCTL     | 0                                                                                                                                                                            | Idle Frame controller is in idle state.                                                                                                                                                     |
| 18:17 (R/NW)       | TXFRCTL     | 1 2                                                                                                                                                                          | Wait Frame controller is waiting for status of previous frame or IFG/backoff period end. Pause Frame controller is generating and transmitting a PAUSE control frame (in full duplex mode). |
| 18:17 (R/NW)       | TXFRCTL     | 3                                                                                                                                                                            | Transmit Frame controller is transferring input frame for transmission.                                                                                                                     |
| 16 (R/NW)          | MMTEA       | MMTxEngine Active. The EMAC_DBG.MMTEA bit, when high, indicates that the MAC core transmit pro- tocol engine is actively transmitting data and is not in IDLE state.         | MMTxEngine Active. The EMAC_DBG.MMTEA bit, when high, indicates that the MAC core transmit pro- tocol engine is actively transmitting data and is not in IDLE state.                        |
| 9:8 (R/NW)         | RXFIFOST    | Rx FIFO State. The EMAC_DBG.RXFIFOST bits give the status of the RxFIFO fill level and indi- cate the relationship to the flow-control activation threshold.                 | Rx FIFO State. The EMAC_DBG.RXFIFOST bits give the status of the RxFIFO fill level and indi- cate the relationship to the flow-control activation threshold.                                |
| 9:8 (R/NW)         | RXFIFOST    | 0                                                                                                                                                                            | Rx FIFO Empty                                                                                                                                                                               |
| 9:8 (R/NW)         | RXFIFOST    | 1                                                                                                                                                                            | Rx FIFO Below De-activate FCT                                                                                                                                                               |
| 9:8 (R/NW)         | RXFIFOST    | 2                                                                                                                                                                            | Rx FIFO Above De-activate FCT                                                                                                                                                               |
| 9:8 (R/NW)         | RXFIFOST    | 3                                                                                                                                                                            | Rx FIFO Full                                                                                                                                                                                |
| 6:5 (R/NW)         | RXFIFOCTLST | Rx FIFO Controller State. The EMAC_DBG.RXFIFOCTLST bits give the state of the RxFIFO read controller.                                                                        | Rx FIFO Controller State. The EMAC_DBG.RXFIFOCTLST bits give the state of the RxFIFO read controller.                                                                                       |
| 6:5 (R/NW)         | RXFIFOCTLST | 0                                                                                                                                                                            | Idle Read controller is in idle state.                                                                                                                                                      |
| 6:5 (R/NW)         | RXFIFOCTLST | 1 2                                                                                                                                                                          | Read Data Read controller is reading frame data. Read Status Read controller is reading frame status or                                                                                     |
| 6:5 (R/NW)         | RXFIFOCTLST |                                                                                                                                                                              | time-stamp. Flush Read controller is flushing the frame data and                                                                                                                            |
| 6:5 (R/NW)         | RXFIFOCTLST | 3                                                                                                                                                                            | sta- tus.                                                                                                                                                                                   |
| 4 (R/NW)           | RXFIFOACT   | Rx FIFO Active. The EMAC_DBG.RXFIFOACT bit, when high, indicates that the MFL RxFIFO write controller is active and is transferring a received frame to the FIFO.            | Rx FIFO Active. The EMAC_DBG.RXFIFOACT bit, when high, indicates that the MFL RxFIFO write controller is active and is transferring a received frame to the FIFO.                           |

Table 29-60: EMAC\_DBG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:1 (R/NW)         | SFIFOST    | Small FIFO State. The EMAC_DBG.SFIFOST bit, when high, indicates the active state of the small FIFO read and write controllers respectively of the MAC receive frame controller mod- ule. |
| 0 (R/NW)           | MMREA      | MMRxEngine Active. The EMAC_DBG.MMREA bit, when high, indicates that the MAC core receive proto- col engine is actively receiving data and is not in IDLE state.                          |

## DMA SCB Bus Mode Register

The EMAC\_DMA0\_BMMODE register selects EMAC DMA system cross bar bus mode features.

Figure 29-29: EMAC\_DMA0\_BMMODE Register Diagram

<!-- image -->

Table 29-61: EMAC\_DMA0\_BMMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | ENLPI      | Enable Low Power Interface. The EMAC_DMA0_BMMODE.ENLPI bit field's when set to 1, it enable LPI mode supported by the GMAC configuration and accepts the LPI request from the system Clock controller.                                                                     |
| 22:20 (R/W)        | WROSRLMT   | SCB Maximum Write Outstanding Request. The EMAC_DMA0_BMMODE.WROSRLMT bit field's value limits the maximum out- standing request on the SCB write interface. Maximum outstanding requests = WR_OSR_LMT+1. EMAC-SCB supports up to 4 outstanding write requests.             |
| 18:16 (R/W)        | RDOSRLMT   | SCB Maximum Read Outstanding Request. The EMAC_DMA0_BMMODE.RDOSRLMT bit field's value limits the maximum out- standing request on the SCB read interface. Maximum outstanding requests = RD_OSR_LMT+1. EMAC-SCB supports up to 4 outstanding read requests.                |
| 13 (R/W)           | ONEKBBE    | 1K Boundary Crossing Enable. When the EMAC_DMA0_BMMODE.ONEKBBE bit is set, the GMAC extensible bus master performs burst transfers that do not cross 1 KB boundary. When reset, the GMAC extensible bus master performs burst transfers that do not cross 4 KB boun- dary. |

Table 29-61: EMAC\_DMA0\_BMMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/NW)          | AAL        | Address Aligned Beats. The EMAC_DMA0_BMMODE.AAL bit (read-only) reflects the state of the EMAC_DMA0_BUSMODE.AAL bit. When this bit is set to 1, EMAC-SCB performs address-aligned burst transfers on both read and write channels.                                                                                                                                                                                                                   |
| 3 (R/W)            | BLEN16     | SCB Burst Length 16. The EMAC_DMA0_BMMODE.BLEN16 bit, when set (or when EMAC_DMA0_BMMODE.UNDEF is set to 1), directs the EMAC-SCB to select a burst length of 16 on the SCB master interface.                                                                                                                                                                                                                                                        |
| 2 (R/W)            | BLEN8      | SCB Burst Length 8. The EMAC_DMA0_BMMODE.BLEN8 bit, when set (or when EMAC_DMA0_BMMODE.UNDEF is set to 1), directs the EMAC-SCB to select a burst length of 8 on the SCB master interface.                                                                                                                                                                                                                                                           |
| 1 (R/W)            | BLEN4      | SCB Burst Length 4. The EMAC_DMA0_BMMODE.BLEN4 bit, when set (or when EMAC_DMA0_BMMODE.UNDEF is set to 1), directs the EMAC-SCB to select a burst length of 4 on the SCB master interface.                                                                                                                                                                                                                                                           |
| 0 (R/NW)           | UNDEF      | SCB Undefined Burst Length. The EMAC_DMA0_BMMODE.UNDEF bit (read-only) indicates the complement (in- vert) value of EMAC_DMA0_BUSMODE.FB bit . When this bit is set to 1, the EMAC-SCB is allowed to perform any burst length equal to or below the maximum allowed burst length as programmed in bits[3:1]. When this bit is set to 0, the EMAC- SCB is allowed to perform only fixed burst lengths as indicated by 16/8/4, or a burst length of 1. |

## DMA SCB Status Register

The EMAC\_DMA0\_BMSTAT register indicates EMAC DMA system cross bar status.

Figure 29-30: EMAC\_DMA0\_BMSTAT Register Diagram

<!-- image -->

Table 29-62: EMAC\_DMA0\_BMSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | BUSRD      | Bus (SCB master) Read Active. The EMAC_DMA0_BMSTAT.BUSRD bit, when high, indicates that SCB Master's read channel is active and transferring data.   |
| 0 (R/NW)           | BUSWR      | Bus (SCB master) Write Active. The EMAC_DMA0_BMSTAT.BUSWR bit, when high, indicates that SCB Master's write channel is active and transferring data. |

## DMA Bus Mode Register

The EMAC\_DMA0\_BUSMODE register selects the DMA bus operating modes for EMAC DMA.

Figure 29-31: EMAC\_DMA0\_BUSMODE Register Diagram

<!-- image -->

Table 29-63: EMAC\_DMA0\_BUSMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | AAL        | Address Aligned Bursts. The EMAC_DMA0_BUSMODE.AAL bit, when set high and the FB bit equals 1, di- rects the SCB interface to generate all bursts aligned to the start address LS bits. If the FB bit is equal to 0, the first burst (accessing the data buffers start address) is not aligned, but subsequent bursts are aligned to the address. |
| 24 (R/W)           | PBL8       | PBL * 8. The EMAC_DMA0_BUSMODE.PBL8 bit, when set high, multiplies the PBL value programmed (bits [22:17] and bits [13:8]) eight times. Therefore, the DMAtransfers the data in 8, 16, and 32 beats depending on the PBL value.                                                                                                                  |
| 23 (R/W)           | USP        | Use Separate PBL. The EMAC_DMA0_BUSMODE.USP bit, when set high, configures the Rx DMAto use the value configured in bits [22:17] as PBL while the PBL value in bits [13:8] is applicable to Tx DMAoperations only.                                                                                                                               |

Table 29-63: EMAC\_DMA0\_BUSMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22:17 (R/W)        | RPBL       | Receive Programmable Burst Length. The EMAC_DMA0_BUSMODE.RPBL bits indicate the maximum number of beats to be transferred in one Rx DMAtransaction. This is the maximum value that is used in a single block Read/Write. The Rx DMAalways attempts to burst as specified in RPBL every time it starts a Burst transfer on the host bus. RPBL can be programmed with permissible values of 1, 2, 4, 8, 16, and 32. Any other value results in undefined behavior. These bits are valid and applicable only when USP is set high.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 16 (R/W)           | FB         | Fixed Burst. The EMAC_DMA0_BUSMODE.FB bit controls whether the SCB Master interface performs fixed burst transfers or not. See the EMAC_DMA0_BMMODE.UNDEF bit de- scription for more information.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 13:8 (R/W)         | PBL        | Programmable Burst Length. The EMAC_DMA0_BUSMODE.PBL bits indicate the maximum number of beats to be transferred in one DMAtransaction. This is the maximum value that is used in a single block Read/Write. The DMAalways attempts to burst as specified in PBL each time it starts a Burst transfer on the host bus. Any other value results in undefined be- havior. When USP is set high, this PBL value is applicable for Tx DMAtransactions only. PBL-max limit = (FIFO size / 2) / 4. PBL-max limit (transmit) = 256 bytes / 2 /4 = 32. PBL-max limit (receive) = 128 bytes / 2 /4 = 16. Note that this PBL is at the DMAend. If PBL= 32 and if BLEN16 is enabled, the DMAautomatically splits 32 bursts in to 2 x 16 bursts. If EMAC_DMA0_BUSMODE.PBL =8, and if EMAC_DMA0_BMMODE.BLEN16 is ena- bled, the max burst is limited to EMAC_DMA0_BMMODE.BLEN8 . If EMAC_DMA0_BUSMODE.PBL8 bit is set, the programmed PBL value is multiplied by 8 times internally. However, the result cannot be more than the above maximum limits specified above. |
| 7 (R/W)            | ATDS       | Alternate Descriptor Size. The EMAC_DMA0_BUSMODE.ATDS bit, when set, increases the size of the alternate descriptor to 32 bytes (8 DWORDS). This is required when the Advanced Time Stamp feature or Full IPC Offload Engine is enabled in the receiver. When reset, the descriptor size reverts back to 4 DWORDs (16 bytes). The enhanced descriptor is not required if the Advanced Time Stamp and IPC Full Checksum Offload features are not enabled. In such case, you can use the 16 bytes descriptor to save 4 bytes of memory.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 29-63: EMAC\_DMA0\_BUSMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:2 (R/W)          | DSL        | Descriptor Skip Length. The EMAC_DMA0_BUSMODE.DSL bit specifies the number of 32-bit words to skip between two unchained descriptors. The address skipping starts from the end of cur- rent descriptor to the start of next descriptor. When DSL value is equal to zero, then the descriptor table is taken as contiguous by the DMA, in Ring mode.                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 0 (R/W1S)          | SWR        | Software Reset. The EMAC_DMA0_BUSMODE.SWR bit, when set, directs the MAC DMAController to reset all MAC Subsystem internal registers and logic. It is cleared automatically after the reset operation has completed in all of the core clock domains. Read a 0 value in this bit before re-programming any register of the core. Note: The reset operation is completed only when all the resets in all the active clock domains are de-asserted. Therefore, it is essential that all the PHY inputs clocks (applicable for the selected PHY interface) are present for software reset completion. This field cleared to 1b0 by the core (Self Clear). The application cannot clear this type of field, and a register write of 1b0 to this bit has no effect on this field. |

## DMA Interrupt Enable Register

The EMAC\_DMA0\_IEN register enables (unmasks) EMAC DMA interrupts.

Figure 29-32: EMAC\_DMA0\_IEN Register Diagram

<!-- image -->

Table 29-64: EMAC\_DMA0\_IEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | NIE        | Normal Interrupt Summary Enable. The EMAC_DMA0_IEN.NIE bit, when set, enables a normal interrupt. When this bit is reset, a normal interrupt is disabled. This bit enables the following bits: EMAC_DMA0_STAT.TI , EMAC_DMA0_STAT.TU , EMAC_DMA0_STAT.RI , and EMAC_DMA0_STAT.ERI .                                                                                               |
| 15 (R/W)           | AIE        | Abnormal Interrupt Summary Enable. The EMAC_DMA0_IEN.AIE bit, when set, enables an abnormal interrupt. When this bit is reset, an Abnormal Interrupt is disabled. This bit enables the following bits: EMAC_DMA0_STAT.TPS , EMAC_DMA0_STAT.TJT , EMAC_DMA0_STAT.OVF , EMAC_DMA0_STAT.RU , EMAC_DMA0_STAT.RPS , EMAC_DMA0_STAT.RWT , EMAC_DMA0_STAT.ETI , and EMAC_DMA0_STAT.FBI . |

Table 29-64: EMAC\_DMA0\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | ERE        | Early Receive Interrupt Enable. The EMAC_DMA0_IEN.ERE bit, when set (and with EMAC_DMA0_IEN.NIE =1), enables the Early Receive Interrupt. When this bit is reset, Early Receive Interrupt is disabled.                                  |
| 13 (R/W)           | FBE        | Fatal Bus Error Enable. The EMAC_DMA0_IEN.FBE bit, when set (and with EMAC_DMA0_IEN.AIE =1), enables the Fatal Bus Error Interrupt. When this bit is reset, Fatal Bus Error Enable Interrupt is disabled.                               |
| 10 (R/W)           | ETE        | Early Transmit Interrupt Enable. The EMAC_DMA0_IEN.ETE bit, when this bit is set (and with EMAC_DMA0_IEN.AIE =1), enables the Early Transmit Interrupt. When this bit is reset, Early Transmit Interrupt is disabled.                   |
| 9 (R/W)            | RWE        | Receive WatchdogTimeout Enable. The EMAC_DMA0_IEN.RWE bit, when set (and with EMAC_DMA0_IEN.AIE =1), enables the Receive Watchdog Timeout Interrupt. When this bit is reset, Receive Watchdog Timeout Interrupt is disabled.            |
| 8 (R/W)            | RSE        | Receive Stopped Enable. The EMAC_DMA0_IEN.RSE bit, when set (and with EMAC_DMA0_IEN.AIE =1), enables the Receive Stopped Interrupt is enabled. When this bit is reset, Receive Stop- ped Interrupt is disabled.                         |
| 7 (R/W)            | RUE        | Receive Buffer Unavailable Enable. The EMAC_DMA0_IEN.RUE bit, when set (and with EMAC_DMA0_IEN.AIE =1), enables the Receive Buffer Unavailable Interrupt. When this bit is reset, the Receive Buffer Unavailable Interrupt is disabled. |
| 6 (R/W)            | RIE        | Receive Interrupt Enable. The EMAC_DMA0_IEN.RIE bit, when set (and with EMAC_DMA0_IEN.NIE =1), enables the Receive Interrupt. When this bit is reset, Receive Interrupt is disabled.                                                    |
| 5 (R/W)            | UNE        | Underflow Interrupt Enable. The EMAC_DMA0_IEN.UNE bit, when set (and with EMAC_DMA0_IEN.AIE =1), enables the Transmit Underflow Interrupt. When this bit is reset, Underflow Interrupt is disabled.                                     |
| 4 (R/W)            | OVE        | Overflow Interrupt Enable. The EMAC_DMA0_IEN.OVE bit, when set (and with EMAC_DMA0_IEN.AIE =1), enables the Receive Overflow Interrupt. When this bit is reset, Overflow Interrupt is disabled.                                         |

Table 29-64: EMAC\_DMA0\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | TJE        | Transmit Jabber Timeout Enable. The EMAC_DMA0_IEN.TJE bit, when set (and with EMAC_DMA0_IEN.AIE =1), enables the Transmit Jabber Timeout Interrupt. When this bit is reset, Transmit Jabber Timeout Interrupt is disabled.             |
| 2 (R/W)            | TUE        | Transmit Buffer Unavailable Enable. The EMAC_DMA0_IEN.TUE bit, when set (and with EMAC_DMA0_IEN.NIE =1), enables the Transmit Buffer Unavailable Interrupt. When this bit is reset, Transmit Buffer Unavailable Interrupt is disabled. |
| 1 (R/W)            | TSE        | Transmit Stopped Enable. The EMAC_DMA0_IEN.TSE bit, when set (and with EMAC_DMA0_IEN.AIE =1), enables the Transmission Stopped Interrupt. When this bit is reset, Transmission Stop- ped Interrupt is disabled.                        |
| 0 (R/W)            | TIE        | Transmit Interrupt Enable. The EMAC_DMA0_IEN.TIE bit, when set (and with EMAC_DMA0_IEN.NIE =1), enables the Transmit Interrupt. When this bit is reset, Transmit Interrupt is disabled.                                                |

## DMA Missed Frame Register

The EMAC\_DMA0\_MISS\_FRM register contains counters for EMAC DMA missed frames and buffer overflows.

Figure 29-33: EMAC\_DMA0\_MISS\_FRM Register Diagram

<!-- image -->

Table 29-65: EMAC\_DMA0\_MISS\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (RC/NW)         | OVFFIFO    | Overflow bit for FIFO Overflow Counter. The EMAC_DMA0_MISS_FRM.OVFFIFO bit holds the overflow bit for FIFO Overflow Counter.                                                               |
| 27:17 (RC/NW)      | MISSFROV   | Missed Frames Buffer Overflow. The EMAC_DMA0_MISS_FRM.MISSFROV bits indicate the number of frames missed by the application due to buffer overflow.                                        |
| 16 (RC/NW)         | OVFMISS    | Overflow bit for Missed Frame Counter. The EMAC_DMA0_MISS_FRM.OVFMISS bit holds the overflow bit for the Missed Frame Counter.                                                             |
| 15:0 (RC/NW)       | MISSFRUN   | Missed Frames Unavailable Buffer. The EMAC_DMA0_MISS_FRM.MISSFRUN bits indicate the number of frames missed by the controller because of the Application Receive Buffer being unavailable. |

## DMA Operation Mode Register

The EMAC\_DMA0\_OPMODE register selects receive and transmit DMA operating modes.

Figure 29-34: EMAC\_DMA0\_OPMODE Register Diagram

<!-- image -->

Table 29-66: EMAC\_DMA0\_OPMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | DT         | Disable Dropping TCP/IP Errors. The EMAC_DMA0_OPMODE.DT bit, when set, directs the core not to drop frames that only have errors detected by the Receive Checksum Offload engine. Such frames do not have any errors (including FCS error) in the Ethernet frame received by the MAC but have errors in the encapsulated payload only. When this bit is reset, all error frames are dropped if the EMAC_DMA0_OPMODE.FEF bit is reset. |
| 25 (R/W)           | RSF        | Receive Store and Forward. The EMAC_DMA0_OPMODE.RSF bit, when set, directs the MFL only to read a frame from the Rx FIFO after the complete frame has been written to it, ignoring the EMAC_DMA0_OPMODE.RTC bits. When this bit is reset, the Rx FIFO operates in threshold mode, subject to the threshold specified by the EMAC_DMA0_OPMODE.RTC bits.                                                                                |
| 24 (R/W)           | DFF        | Disable Flushing of received Frames. The EMAC_DMA0_OPMODE.DFF bit, when set, directs the Rx DMAnot to flush any frames because of the unavailability of receive descriptors/buffers as it does nor- mally when this bit is reset.                                                                                                                                                                                                     |

Table 29-66: EMAC\_DMA0\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | TSF        | Transmit Store and Forward. The EMAC_DMA0_OPMODE.TSF bit, when set, starts transmission when a full frame resides in the MFL Transmit FIFO. When this bit is set, the TTC values speci- fied in Register 6[16:14] are ignored. This bit should be changed only when transmis- sion is stopped.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Transmit Store and Forward. The EMAC_DMA0_OPMODE.TSF bit, when set, starts transmission when a full frame resides in the MFL Transmit FIFO. When this bit is set, the TTC values speci- fied in Register 6[16:14] are ignored. This bit should be changed only when transmis- sion is stopped.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 20 (R/W1S)         | FTF        | Flush Transmit FIFO. The EMAC_DMA0_OPMODE.FTF bit, when set, directs the transmit FIFO controller logic to reset to its default values and thus all data in the Tx FIFO is lost/flushed. This bit is cleared internally when the flushing operation is completed fully. The Operation Mode register should not be written to until this bit is cleared. The data which is al- ready accepted by the MAC transmitter is not flushed. It is scheduled for transmission and results in underflow and runt frame transmission. Note: The flush operation com- pletes only after emptying the Tx FIFO of its contents and all the pending Transmit Status of the transmitted frames are accepted by the host. In order to complete this flush operation, the PHY transmit clock is required to be active. This field cleared to 1b0 by the core (Self Clear). The application cannot clear this type of field, and a reg- ister write of 1b0 to this bit has no effect on this field. | Flush Transmit FIFO. The EMAC_DMA0_OPMODE.FTF bit, when set, directs the transmit FIFO controller logic to reset to its default values and thus all data in the Tx FIFO is lost/flushed. This bit is cleared internally when the flushing operation is completed fully. The Operation Mode register should not be written to until this bit is cleared. The data which is al- ready accepted by the MAC transmitter is not flushed. It is scheduled for transmission and results in underflow and runt frame transmission. Note: The flush operation com- pletes only after emptying the Tx FIFO of its contents and all the pending Transmit Status of the transmitted frames are accepted by the host. In order to complete this flush operation, the PHY transmit clock is required to be active. This field cleared to 1b0 by the core (Self Clear). The application cannot clear this type of field, and a reg- ister write of 1b0 to this bit has no effect on this field. |
| 16:14 (R/W)        | TTC        | Transmit Threshold Control. The EMAC_DMA0_OPMODE.TTC bits control the threshold level of the MFL Trans- mit FIFO. Transmission starts when the frame size within the MFL Transmit FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are also transmitted. These bits are used only when the EMAC_DMA0_OPMODE.TSF bit is reset. The value =011 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Transmit Threshold Control. The EMAC_DMA0_OPMODE.TTC bits control the threshold level of the MFL Trans- mit FIFO. Transmission starts when the frame size within the MFL Transmit FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are also transmitted. These bits are used only when the EMAC_DMA0_OPMODE.TSF bit is reset. The value =011 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 64                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 128                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 192                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 256                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 40                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 32                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 24                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 16                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 29-66: EMAC\_DMA0\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | ST         | Start/Stop Transmission. The EMAC_DMA0_OPMODE.ST bit, when set, places transmission in the Running state, and the DMAchecks the Transmit List at the current position for a frame to be transmitted. Descriptor acquisition is attempted either from the current position in the list, which is the Transmit List Base Address set by Transmit Descriptor List Address, or from the position retained when transmission was stopped previously. If the current descriptor is not owned by the DMA, transmission enters the Suspended state, and the EMAC_DMA0_STAT.TU bit is set. The Start Transmission command is effective only when transmission is stopped. If the command is issued before setting the EMAC_DMA0_TXDSC_CUR address register, then the DMAbehavior is unpredictable. When this bit is reset, the transmission proc- ess is placed in the Stopped state after completing the transmission of the current frame. The Next Descriptor position in the Transmit List is saved, and becomes the current position when transmission is restarted. The stop transmission command is ef- fective only when the transmission of the current frame is complete or the transmission is in the Suspended state. |
| 7 (R/W)            | FEF        | Forward Error Frames. The EMAC_DMA0_OPMODE.FEF bit, when reset, directs the Rx FIFO to drop frames with error status (CRC error, collision error, giant frame, watchdog timeout, overflow). However, if the frames start byte (write) pointer is already transferred to the read controller side (in Threshold mode), then the frames are not dropped. When EMAC_DMA0_OPMODE.FEF bit is set, all frames except runt error frames are for- warded to the DMA. But when Rx FIFO overflows when a partial frame is written, then such frames are dropped even when EMAC_DMA0_OPMODE.FEF is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 6 (R/W)            | FUF        | Forward Undersized good Frames. The EMAC_DMA0_OPMODE.FUF bit, when set, directs the Rx FIFO to forward Un- dersized frames (frames with no Error and length less than 64 bytes) including pad- bytes and CRC). When reset, the Rx FIFO drops all frames of less than 64 bytes, un- less it is already transferred because of lower value of Receive Threshold (for example, EMAC_DMA0_OPMODE.RTC =01).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 5 (R/W)            | DGF        | Drop Gaint Frames. The EMAC_DMA0_OPMODE.DGF bit, when set, the MAC drops the received giant frames in the Rx FIFO, that is, frames that are larger than the computed giant frame limit. When reset, the MAC does not drop the giant frames in the Rx FIFO.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 29-66: EMAC\_DMA0\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:3 (R/W)          | RTC        | Receive Threshold Control. The EMAC_DMA0_OPMODE.RTC bits control the threshold level of the MFL Re- ceive FIFO. Transfer (request) to DMAstarts when the frame size within the MFL Re- ceive FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are transferred automatically. These bits are valid only when the EMAC_DMA0_OPMODE.RSF bit is zero, and are ignored when the EMAC_DMA0_OPMODE.RSF bit is set to 1. The value =11 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 2 (R/W)            | OSF        | Operate on Second Frame. The EMAC_DMA0_OPMODE.OSF bit, when set, instructs the DMAto process a sec- ond frame of Transmit data even before status for first frame is obtained.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 1 (R/W)            | SR         | Start/Stop Receive. The EMAC_DMA0_OPMODE.SR bit, when set, places the Receive process in the Run- ning state. The DMAattempts to acquire the descriptor from the Receive list and processes incoming frames. Descriptor acquisition is attempted from the current posi- tion in the list, which is the address set by DMAReceive Descriptor List Address or the position retained when the Receive process was previously stopped. If no descriptor is owned by the DMA, reception is suspended, and the EMAC_DMA0_STAT.RU bit is set. The Start Receive command is effective only when reception has stopped. If the com- mand was issued before setting EMAC_DMA0_RXDSC_CUR address register,DMA behavior is unpredictable. When this bit is cleared, Rx DMAoperation is stopped after the transfer of the current frame. The next descriptor position in the Receive list is saved and becomes the current position after the Receive process is restarted. The Stop Receive command is effective only when the Receive process is in either the Running (waiting for receive packet) or in the Suspended state. |

## DMA Rx Buffer Current Register

The EMAC\_DMA0\_RXBUF\_CUR register holds the pointer to the current receive DMA buffer.

Figure 29-35: EMAC\_DMA0\_RXBUF\_CUR Register Diagram

<!-- image -->

Table 29-67: EMAC\_DMA0\_RXBUF\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Receive Current Buffer Address. The EMAC_DMA0_RXBUF_CUR.ADDR bit field points to the current Receive Buffer address being read by the DMA. Pointer updated by DMAduring operation. Cleared on Reset. |

## DMA Rx Descriptor List Address Register

The EMAC\_DMA0\_RXDSC\_ADDR register holds the address for the DMA receive descriptor list. Writing to this Register is permitted only when reception is stopped. When stopped, this must be written to before the receive Start command is given. The processor can write to EMAC\_DMA0\_RXDSC\_ADDR only when Rx DMA has stopped ( EMAC\_DMA0\_OPMODE.SR bit =0). When stopped, it can be written with a new descriptor list address. When the processor sets the EMAC\_DMA0\_OPMODE.SR bit to 1, the DMA takes the newly programmed descriptor base address. If this register is not changed when the EMAC\_DMA0\_OPMODE.SR bit is cleared to 0, the DMA takes the descriptor address where it was stopped earlier.

Figure 29-36: EMAC\_DMA0\_RXDSC\_ADDR Register Diagram

<!-- image -->

Table 29-68: EMAC\_DMA0\_RXDSC\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Start of Receive List. The EMAC_DMA0_RXDSC_ADDR.VALUE bit field contains the base address of the First Descriptor in the Receive Descriptor list. The LSB bits [1:0] for the 32bit bus width are ignored and are taken as all-zero by the DMAinternally. Therefore, these LSB bits are Read-Only (RO). |

## DMA Rx Descriptor Current Register

The EMAC\_DMA0\_RXDSC\_CUR register contains the current DMA receive descriptor.

Figure 29-37: EMAC\_DMA0\_RXDSC\_CUR Register Diagram

<!-- image -->

Table 29-69: EMAC\_DMA0\_RXDSC\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Receive Current Descriptor Address. The EMAC_DMA0_RXDSC_CUR.ADDR bit field points to the start address of the current Receive Descriptor read by the DMA. Pointer updated by DMAduring opera- tion. Cleared on Reset. |

## DMA Rx Interrupt Watch Dog Register

The EMAC\_DMA0\_RXIWDOG register contains the timeout value for the EMAC DMA receive interrupt watch dog timer.

Figure 29-38: EMAC\_DMA0\_RXIWDOG Register Diagram

<!-- image -->

Table 29-70: EMAC\_DMA0\_RXIWDOG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | RIWT       | RI WatchDog Timer Count. The EMAC_DMA0_RXIWDOG.RIWT bit field indicates the number of system clock cycles multiplied by 256 for which the watchdog timer is set. The watchdog timer gets triggered with the programmed value after the Rx DMAcompletes the transfer of a frame for which the RI status bit is not set because of the setting in the corresponding descriptor RDES1[31]. When the watch-dog timer runs out, the RI bit is set and the timer is stopped. The watchdog timer is reset when EMAC_DMA0_STAT.RI bit is set high because of automatic setting of EMAC_DMA0_STAT.RI as per RDES1[31] of any received frame. |

## DMA Rx Poll Demand register

The EMAC\_DMA0\_RXPOLL register directs the EMAC to poll the receive descriptor list.

Figure 29-39: EMAC\_DMA0\_RXPOLL Register Diagram

<!-- image -->

Table 29-71: EMAC\_DMA0\_RXPOLL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | START      | Receive Poll Demand. The EMAC_DMA0_RXPOLL.START bits, when written with any value, cause the DMAto read the current descriptor pointed to by the EMAC_DMA0_RXDSC_CUR register. If that descriptor is not available (owned by application), reception returns to the Suspended state, and the EMAC_DMA0_STAT.RU bit is asserted. If the descriptor is available, the Receive DMAreturns to the active state. |

## DMA Status Register

The EMAC\_DMA0\_STAT register indicates EMAC DMA status.

Figure 29-40: EMAC\_DMA0\_STAT Register Diagram

<!-- image -->

Table 29-72: EMAC\_DMA0\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/NW)          | GLPII      | GMAC LPI Interrupt. The EMAC_DMA0_STAT.GLPII bit indicates an interrupt event in the LPI logic of the MAC. To reset this bit to 1'b0, the software must read the corresponding registers in the DWC_gmac to get the exact cause of the interrupt and clear its source. |

Table 29-72: EMAC\_DMA0\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/NW)          | TTI        | Time Stamp Trigger Interrupt. The EMAC_DMA0_STAT.TTI bit indicates an interrupt event in the MAC core's Time Stamp Generator block. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear its source to reset this bit to =0. When this bit is high, the interrupt signal from the MAC is high. | Time Stamp Trigger Interrupt. The EMAC_DMA0_STAT.TTI bit indicates an interrupt event in the MAC core's Time Stamp Generator block. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear its source to reset this bit to =0. When this bit is high, the interrupt signal from the MAC is high. |
| 27 (R/NW)          | MCI        | MAC MMCInterrupt. The EMAC_DMA0_STAT.MCI bit reflects an interrupt event in the MMCmodule of the MAC core. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear the source of interrupt to make this bit as =0. The interrupt signal from the MAC is high when this bit is high.               | MAC MMCInterrupt. The EMAC_DMA0_STAT.MCI bit reflects an interrupt event in the MMCmodule of the MAC core. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear the source of interrupt to make this bit as =0. The interrupt signal from the MAC is high when this bit is high.               |
| 26 (R/NW)          | GLI        | Line Interface Interrupt. The EMAC_DMA0_STAT.GLI bit When set, this bit reflects any of the following in- terrupt events in the DWC_gmac interfaces                                                                                                                                                                                                        | Line Interface Interrupt. The EMAC_DMA0_STAT.GLI bit When set, this bit reflects any of the following in- terrupt events in the DWC_gmac interfaces                                                                                                                                                                                                        |
| 25:23 (R/NW)       | EB         | Error Bits. The EMAC_DMA0_STAT.EB bits indicate the type of error that caused a Bus Error (for example, error response on the SCB interface). These bits are valid only when the EMAC_DMA0_STAT.FBI bit is set. This field does not generate an interrupt.                                                                                                 | Error Bits. The EMAC_DMA0_STAT.EB bits indicate the type of error that caused a Bus Error (for example, error response on the SCB interface). These bits are valid only when the EMAC_DMA0_STAT.FBI bit is set. This field does not generate an interrupt.                                                                                                 |
| 25:23 (R/NW)       | EB         | 0                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, write transfer, RxDMA                                                                                                                                                                                                                                                                                                     |
| 25:23 (R/NW)       | EB         | 1                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, write transfer, TxDMA                                                                                                                                                                                                                                                                                                     |
| 25:23 (R/NW)       | EB         | 2                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, read transfer, RxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 3                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, read transfer, TxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 4                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, write transfer, RxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 5                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, write transfer, TxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 6                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, read transfer, RxDMA                                                                                                                                                                                                                                                                                                       |
| 25:23 (R/NW)       | EB         | 7                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, read transfer, TxDMA                                                                                                                                                                                                                                                                                                       |
| 22:20 (R/NW)       | TS         | Tx Process State. The EMAC_DMA0_STAT.TS bits indicate the transmit DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                    | Tx Process State. The EMAC_DMA0_STAT.TS bits indicate the transmit DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                    |
| 22:20 (R/NW)       | TS         | 0                                                                                                                                                                                                                                                                                                                                                          | Stopped; Reset or Stop Tx Command Issued                                                                                                                                                                                                                                                                                                                   |
| 22:20 (R/NW)       | TS         | 1                                                                                                                                                                                                                                                                                                                                                          | Running; Fetching Tx Transfer Descriptor                                                                                                                                                                                                                                                                                                                   |
| 22:20 (R/NW)       | TS         | 2                                                                                                                                                                                                                                                                                                                                                          | Running; Waiting for Status                                                                                                                                                                                                                                                                                                                                |
| 22:20 (R/NW)       | TS         | 3                                                                                                                                                                                                                                                                                                                                                          | Reading Data from Host Memory Buffer and Queuing It to Tx Buffer                                                                                                                                                                                                                                                                                           |
| 22:20 (R/NW)       | TS         | 4                                                                                                                                                                                                                                                                                                                                                          | TIME_STAMP Write State                                                                                                                                                                                                                                                                                                                                     |

Table 29-72: EMAC\_DMA0\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Suspended; Tx Descriptor Unavailable or Tx Buffer Un- derflow                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Closing Tx Descriptor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 19:17 (R/NW)       | RS         | Rx Process State. The EMAC_DMA0_STAT.RS bits indicate the receive DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                                                                                                                                                                                                  | Rx Process State. The EMAC_DMA0_STAT.RS bits indicate the receive DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 19:17 (R/NW)       |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Stopped: Reset or Stop Rx Command Issued.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 19:17 (R/NW)       |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Running: Fetching Rx Transfer Descriptor.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 19:17 (R/NW)       |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 19:17 (R/NW)       |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Running: Waiting for Rx Packet                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 19:17 (R/NW)       |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Suspended: Rx Descriptor Unavailable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 19:17 (R/NW)       |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Running: Closing Rx Descriptor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 19:17 (R/NW)       |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | TIME_STAMP Write State                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 19:17 (R/NW)       |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Running: Transferring Rx Packet Data from Rx Buffer to Host Memory                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 16 (R/W1C)         | NIS        | Normal Interrupt Summary. The value of the EMAC_DMA0_STAT.NIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA0_STAT.TI , EMAC_DMA0_STAT.TU ,                                                                                                                                                                                                                                                                                               | Normal Interrupt Summary. The value of the EMAC_DMA0_STAT.NIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA0_STAT.TI , EMAC_DMA0_STAT.TU ,                                                                                                                                                                                                                                                                                               |
| 15 (R/W1C)         | AIS        | Abnormal Interrupt Summary. The value of the EMAC_DMA0_STAT.AIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA0_IEN.TSE , EMAC_DMA0_IEN.TJE , EMAC_DMA0_IEN.OVE , EMAC_DMA0_IEN.UNE , EMAC_DMA0_IEN.RUE , EMAC_DMA0_IEN.RSE , EMAC_DMA0_IEN.RWE , EMAC_DMA0_IEN.ETE , and EMAC_DMA0_IEN.FBE . Only unmasked bits affect the Abnormal Interrupt Sum- mary bit. This is a sticky bit and must be cleared each time a corresponding bit that | Abnormal Interrupt Summary. The value of the EMAC_DMA0_STAT.AIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA0_IEN.TSE , EMAC_DMA0_IEN.TJE , EMAC_DMA0_IEN.OVE , EMAC_DMA0_IEN.UNE , EMAC_DMA0_IEN.RUE , EMAC_DMA0_IEN.RSE , EMAC_DMA0_IEN.RWE , EMAC_DMA0_IEN.ETE , and EMAC_DMA0_IEN.FBE . Only unmasked bits affect the Abnormal Interrupt Sum- mary bit. This is a sticky bit and must be cleared each time a corresponding bit that |
| 14 (R/W1C)         | ERI        | Early Receive Interrupt. The EMAC_DMA0_STAT.ERI bit indicates that the DMAhad filled the first data                                                                                                                                                                                                                                                                                                                                                                                                                                     | Early Receive Interrupt. The EMAC_DMA0_STAT.ERI bit indicates that the DMAhad filled the first data                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 29-72: EMAC\_DMA0\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1C)         | FBI        | Fatal Bus Error Interrupt. The EMAC_DMA0_STAT.FBI bit indicates that a bus error occurred, as detailed in the EMAC_DMA0_STAT.EB field. When this bit is set, the corresponding DMAen- gine disables all its bus accesses.                                                                                                                                                                                                                                                                                                                                        |
| 10 (R/W1C)         | ETI        | Early Transmit Interrupt. The EMAC_DMA0_STAT.ETI bit indicates that the frame to be transmitted was fully transferred to the MFL Transmit FIFO.                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 9 (R/W1C)          | RWT        | Receive WatchDog Timeout. The EMAC_DMA0_STAT.RWT bit is asserted when a frame with a length greater than 2,048 bytes is received (10, 240 when Jumbo Frame mode is enabled).                                                                                                                                                                                                                                                                                                                                                                                     |
| 8 (R/W1C)          | RPS        | Receive Process Stopped. The EMAC_DMA0_STAT.RPS bit is asserted when the Receive Process enters the Stopped state.                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 7 (R/W1C)          | RU         | Receive Buffer Unavailable. The EMAC_DMA0_STAT.RU bit indicates that the Next Descriptor in the Receive List is owned by the application and cannot be acquired by the DMA. Receive Process is suspended. To resume processing Receive descriptors, the application should change the ownership of the descriptor and issue a Receive Poll Demand command. If no Re- ceive Poll Demand is issued, Receive Process resumes when the next recognized incom- ing frame is received. This bit is set only when the previous Receive Descriptor was owned by the DMA. |
| 6 (R/W1C)          | RI         | Receive Interrupt. The EMAC_DMA0_STAT.RI bit indicates the completion of frame reception. Specif- ic frame status information has been posted in the descriptor. Reception remains in the Running state.                                                                                                                                                                                                                                                                                                                                                         |
| 5 (R/W1C)          | UNF        | Transmit Buffer Underflow. The EMAC_DMA0_STAT.UNF bit indicates that the Transmit Buffer had an Under- flow during frame transmission. Transmission is suspended and an Underflow Error TDES0[1] is set.                                                                                                                                                                                                                                                                                                                                                         |
| 4 (R/W1C)          | OVF        | Receive Buffer Overflow. The EMAC_DMA0_STAT.OVF bit indicates that the Receive Buffer had an Overflow during frame reception. If the partial frame is transferred to application, the overflow status is set in RDES0[11].                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W1C)          | TJT        | Transmit Jabber Timeout. The EMAC_DMA0_STAT.TJT bit indicates that the Transmit Jabber Timer expired, meaning that the transmitter had been excessively active. The transmission process is aborted and placed in the Stopped state. This causes the Transmit Jabber Timeout TDES0[14] flag to assert.                                                                                                                                                                                                                                                           |

Table 29-72: EMAC\_DMA0\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | TU         | Transmit Buffer Unavailable. The EMAC_DMA0_STAT.TU bit indicates that the Next Descriptor in the Transmit List is owned by the application and cannot be acquired by the DMA. Transmission is suspended. The value in the EMAC_DMA0_STAT.TS bits explain the Transmit Proc- ess state transitions. To resume processing transmit descriptors, the application should change the ownership of the bit of the descriptor and then issue a Transmit Poll De- mand command. |
| 1 (R/W1C)          | TPS        | Transmit Process Stopped. The EMAC_DMA0_STAT.TPS bit is set when the transmission is stopped.                                                                                                                                                                                                                                                                                                                                                                           |
| 0 (R/W1C)          | TI         | Transmit Interrupt. The EMAC_DMA0_STAT.TI bit indicates that frame transmission is finished and TDES1[31] is set in the First Descriptor.                                                                                                                                                                                                                                                                                                                               |

## DMA Tx Buffer Current Register

The EMAC\_DMA0\_TXBUF\_CUR register holds the pointer to the current transmit DMA buffer.

Figure 29-41: EMAC\_DMA0\_TXBUF\_CUR Register Diagram

<!-- image -->

Table 29-73: EMAC\_DMA0\_TXBUF\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Transmit Current Buffer Address. The EMAC_DMA0_TXBUF_CUR.ADDR bit field points to the current Transmit Buf- fer Address being read by the DMA. Pointer updated by DMAduring operation. Cleared on Reset. |

## DMA Tx Descriptor List Address Register

The EMAC\_DMA0\_TXDSC\_ADDR register holds the address for the DMA transmit descriptor list. The processor can write to this Register only when Tx DMA has stopped ( EMAC\_DMA0\_OPMODE.ST bit =0). When stopped, this can be written with a new descriptor list address. When the processor sets the EMAC\_DMA0\_OPMODE.ST bit to 1, the DMA takes the newly programmed descriptor base address. If this register is not changed when the EMAC\_DMA0\_OPMODE.ST bit is cleared to 0, then the DMA takes the descriptor address where it was stopped earlier.

Figure 29-42: EMAC\_DMA0\_TXDSC\_ADDR Register Diagram

<!-- image -->

Table 29-74: EMAC\_DMA0\_TXDSC\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Start of Transmit List. The EMAC_DMA0_TXDSC_ADDR.VALUE bit field contains the base address of the First Descriptor in the Transmit Descriptor list. The LSB bits [1:0] for 32bit bus width are ignored and are taken as all-zero by the DMAinternally. Therefore, these LSB bits are Read-Only (RO). |

## DMA Tx Descriptor Current Register

The EMAC\_DMA0\_TXDSC\_CUR register contains the current DMA transmit descriptor.

Figure 29-43: EMAC\_DMA0\_TXDSC\_CUR Register Diagram

<!-- image -->

Table 29-75: EMAC\_DMA0\_TXDSC\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Transmit Descriptor Address. The EMAC_DMA0_TXDSC_CUR.ADDR bit field points to the start address of the current Transmit Descriptor read by the DMA. Pointer updated by DMAduring oper- ation. Cleared on Reset. |

## DMA Tx Poll Demand Register

The EMAC\_DMA0\_TXPOLL register directs the EMAC to poll the transmit descriptor list.

Figure 29-44: EMAC\_DMA0\_TXPOLL Register Diagram

<!-- image -->

Table 29-76: EMAC\_DMA0\_TXPOLL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | START      | Transmit Poll Demand. The EMAC_DMA0_TXPOLL.START bits, when written with any value, cause the DMAto read the current descriptor pointed to by EMAC_DMA0_TXDSC_CUR regis- ter. If that descriptor is not available (owned by application), transmission returns to the Suspend state, and the EMAC_DMA0_STAT.TU bit is asserted. If the descriptor is available, transmission resumes. |

## DMA Bus Mode Register

The EMAC\_DMA1\_BUSMODE register selects the DMA bus operating modes for EMAC DMA.

Figure 29-45: EMAC\_DMA1\_BUSMODE Register Diagram

<!-- image -->

Table 29-77: EMAC\_DMA1\_BUSMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | AAL        | Address Aligned Bursts. The EMAC_DMA1_BUSMODE.AAL bit, when set high and the FB bit equals 1, di- rects the SCB interface to generate all bursts aligned to the start address LS bits. If the FB bit is equal to 0, the first burst (accessing the data buffers start address) is not aligned, but subsequent bursts are aligned to the address. |
| 24 (R/W)           | PBL8       | PBL * 8. The EMAC_DMA1_BUSMODE.PBL8 bit, when set high, multiplies the PBL value programmed (bits [22:17] and bits [13:8]) eight times. Therefore, the DMAtransfers the data in 8, 16, and 32 beats depending on the PBL value.                                                                                                                  |
| 23 (R/W)           | USP        | Use Separate PBL. The EMAC_DMA1_BUSMODE.USP bit, when set high, configures the Rx DMAto use the value configured in bits [22:17] as PBL while the PBL value in bits [13:8] is applicable to Tx DMAoperations only.                                                                                                                               |

Table 29-77: EMAC\_DMA1\_BUSMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22:17 (R/W)        | RPBL       | Receive Programmable Burst Length. The EMAC_DMA1_BUSMODE.RPBL bits indicate the maximum number of beats to be transferred in one Rx DMAtransaction. This is the maximum value that is used in a single block Read/Write. The Rx DMAalways attempts to burst as specified in RPBL every time it starts a Burst transfer on the host bus. RPBL can be programmed with permissible values of 1, 2, 4, 8, 16, and 32. Any other value results in undefined behavior. These bits are valid and applicable only when USP is set high.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 16 (R/W)           | FB         | Fixed Burst. The EMAC_DMA1_BUSMODE.FB bit controls whether the SCB Master interface performs fixed burst transfers or not. See the EMAC_DMA0_BMMODE.UNDEF bit de- scription for more information.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 13:8 (R/W)         | PBL        | Programmable Burst Length. The EMAC_DMA1_BUSMODE.PBL bits indicate the maximum number of beats to be transferred in one DMAtransaction. This is the maximum value that is used in a single block Read/Write. The DMAalways attempts to burst as specified in PBL each time it starts a Burst transfer on the host bus. Any other value results in undefined be- havior. When USP is set high, this PBL value is applicable for Tx DMAtransactions only. PBL-max limit = (FIFO size / 2) / 4. PBL-max limit (transmit) = 256 bytes / 2 /4 = 32. PBL-max limit (receive) = 128 bytes / 2 /4 = 16. Note that this PBL is at the DMAend. If PBL= 32 and if BLEN16 is enabled, the DMAautomatically splits 32 bursts in to 2 x 16 bursts. If EMAC_DMA1_BUSMODE.PBL =8, and if EMAC_DMA0_BMMODE.BLEN16 is ena- bled, the max burst is limited to EMAC_DMA0_BMMODE.BLEN8 . If EMAC_DMA1_BUSMODE.PBL8 bit is set, the programmed PBL value is multiplied by 8 times internally. However, the result cannot be more than the above maximum limits specified above. |
| 7 (R/W)            | ATDS       | Alternate Descriptor Size. The EMAC_DMA1_BUSMODE.ATDS bit, when set, increases the size of the alternate descriptor to 32 bytes (8 DWORDS). This is required when the Advanced Time Stamp feature or Full IPC Offload Engine is enabled in the receiver. When reset, the descriptor size reverts back to 4 DWORDs (16 bytes). The enhanced descriptor is not required if the Advanced Time Stamp and IPC Full Checksum Offload features are not enabled. In such case, you can use the 16 bytes descriptor to save 4 bytes of memory.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 29-77: EMAC\_DMA1\_BUSMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:2 (R/W)          | DSL        | Descriptor Skip Length. The EMAC_DMA1_BUSMODE.DSL bit specifies the number of 32-bit words to skip between two unchained descriptors. The address skipping starts from the end of cur- rent descriptor to the start of next descriptor. When DSL value is equal to zero, then the descriptor table is taken as contiguous by the DMA, in Ring mode. |

## Channel 1 Credit Shaping Control Register

The EMAC\_DMA1\_CHCBSCTL register controls the credit-based shaper algorithm in the T raffic Manager for scheduling the frames for transmission. This register is present only when you select the T ransmit Channel 1 in the AV mode.

Figure 29-46: EMAC\_DMA1\_CHCBSCTL Register Diagram

<!-- image -->

Table 29-78: EMAC\_DMA1\_CHCBSCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | ABPSSIE    | Average Bits Per Slot Interrupt Enable. When the EMAC_DMA1_CHCBSCTL.ABPSSIE bit is set, the MAC asserts an inter- rupt (sbd_intr_o or mci_intr_o) when the average bits per slot status is updated for Channel 1. When this bit is cleared, an interrupt is not asserted for such an event.                    | Average Bits Per Slot Interrupt Enable. When the EMAC_DMA1_CHCBSCTL.ABPSSIE bit is set, the MAC asserts an inter- rupt (sbd_intr_o or mci_intr_o) when the average bits per slot status is updated for Channel 1. When this bit is cleared, an interrupt is not asserted for such an event.                    |
| 6:4 (R/W)          | SLC        | Slot Count. The EMAC_DMA1_CHCBSCTL.SLC bit field programs the number of slots (of dura- tion 125 micro-sec) over which the average transmitted bits per slot (provided in the CBS Status register) are computed for Channel 1 when the credit-based shaper algo- rithm is enabled. The encoding is as follows: | Slot Count. The EMAC_DMA1_CHCBSCTL.SLC bit field programs the number of slots (of dura- tion 125 micro-sec) over which the average transmitted bits per slot (provided in the CBS Status register) are computed for Channel 1 when the credit-based shaper algo- rithm is enabled. The encoding is as follows: |
|                    |            | 0                                                                                                                                                                                                                                                                                                              | 1 Slot                                                                                                                                                                                                                                                                                                         |
|                    |            | 1                                                                                                                                                                                                                                                                                                              | 2 Slots                                                                                                                                                                                                                                                                                                        |
|                    |            | 2                                                                                                                                                                                                                                                                                                              | 4 Slots                                                                                                                                                                                                                                                                                                        |
|                    |            | 3                                                                                                                                                                                                                                                                                                              | 8 Slots                                                                                                                                                                                                                                                                                                        |
|                    |            | 4                                                                                                                                                                                                                                                                                                              | 16 Slots                                                                                                                                                                                                                                                                                                       |
|                    |            | 5                                                                                                                                                                                                                                                                                                              | Reserved                                                                                                                                                                                                                                                                                                       |
|                    |            | 6                                                                                                                                                                                                                                                                                                              | Reserved                                                                                                                                                                                                                                                                                                       |
|                    |            | 7                                                                                                                                                                                                                                                                                                              | Reserved                                                                                                                                                                                                                                                                                                       |

Table 29-78: EMAC\_DMA1\_CHCBSCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | CC         | Credit Control. The EMAC_DMA1_CHCBSCTL.CC bit, when reset, sets the accumulated credit pa- rameter in the credit-based shaper algorithm logic to zero when there is positive credit and no frame to transmit in Channel 1.                                                                                                   |
| 0 (R/W)            | CBSD       | Credit based shaper disable. The EMAC_DMA1_CHCBSCTL.CBSD bit disables the credit-based shaper algorithm for Channel 1 traffic and makes the traffic management algorithm to strict priority for Channel 1 over Channel 0. When reset, the credit-based shaper algorithm schedules the traffic in Channel 1 for transmission. |

## Channel 1 Average Traffic Transmitted Register

The EMAC\_DMA1\_CHCBSSTAT register provides the average traffic transmitted in Channel 1. This register is present only when you select the Transmit Channel 1 in the AV mode.

Figure 29-47: EMAC\_DMA1\_CHCBSSTAT Register Diagram

<!-- image -->

Table 29-79: EMAC\_DMA1\_CHCBSSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/NW)          | ABSU       | ABS Updated. The EMAC_DMA1_CHCBSSTAT.ABSU bit indicates that the MAC has updated the ABS value. This bit is cleared when the application reads the ABS value.                                                                                                                                        |
| 16:0 (R/NW)        | ABS        | Average Bits Per Slot. The EMAC_DMA1_CHCBSSTAT.ABS bit field contains the average transmitted bits per slot. This field is computed over programmed number of slots ( EMAC_DMA1_CHCBSCTL.SLC bit field) for Channel 1 traffic. The maximum val- ue is 0x30D4 for 100 Mbps and 0x1E848 for 1000 Mbps. |

## Channel 1 High Credit Value Register

The EMAC\_DMA1\_CHHIC register provides the maximum value that can be accumulated for Channel 1 in the credit parameter of the credit-based shaper algorithm. This register is present only when you select the T ransmit Channel 1 in the AV mode.

Figure 29-48: EMAC\_DMA1\_CHHIC Register Diagram

<!-- image -->

Table 29-80: EMAC\_DMA1\_CHHIC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 28:0 (R/W)         | HC         | High Credit. The EMAC_DMA1_CHHIC.HC bit field contains the hiCredit value required for the credit-based shaper algorithm for Channel 1. |

## Channel 1 Idle Slope Credit Value Register

The EMAC\_DMA1\_CHISC register provides the bandwidth allocated for the AV traffic on Channel 1. This register is present only when you select the T ransmit Channel 1 in the AV mode.

Figure 29-49: EMAC\_DMA1\_CHISC Register Diagram

<!-- image -->

Table 29-81: EMAC\_DMA1\_CHISC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:0 (R/W)         | ISC        | Idle Slope Credit. The EMAC_DMA1_CHISC.ISC bit field contains the idleSlopeCredit value required for the credit-based shaper algorithm for Channel 1. |

## Channel 1 Low Credit Value Register

The EMAC\_DMA1\_CHLOC register provides the minimum value that can be accumulated for Channel 1 in the credit parameter of the credit-based shaper algorithm. This register is present only when you select T ransmit Channel 1 in the AV mode.

Figure 29-50: EMAC\_DMA1\_CHLOC Register Diagram

<!-- image -->

Table 29-82: EMAC\_DMA1\_CHLOC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------|
| 28:0 (R/W)         | LC         | Low Credit. The EMAC_DMA1_CHLOC.LC bit field contains the loCredit value required for the credit-based shaper algorithm for Channel 1. |

## Channel 1 Control Bits for Slot Function Register

The EMAC\_DMA1\_CHSFCS register controls the slot comparison feature that the Channel 1 transmit DMA uses to fetch the buffer data from system memory.

Figure 29-51: EMAC\_DMA1\_CHSFCS Register Diagram

<!-- image -->

Table 29-83: EMAC\_DMA1\_CHSFCS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/NW)       | RSN        | Reference Slot Number. The EMAC_DMA1_CHSFCS.RSN bits, gives the current value of the reference slot number in DMAused for comparison checking.                                                                                                                                                                                                                                                                                                                                                                                 |
| 1 (R/W)            | ASC        | Advance Slot Check. When set, this bit enables the DMAto fetch the data from the buffer when the slot number (SLOTNUM) programmed in the transmit descriptor is ---equal to the refer- ence slot number given in Bits [19:16] -or- ahead of the reference slot number by up to two slots. This bit is applicable only when Bit 0 (ESC) is set.                                                                                                                                                                                 |
| 0 (R/W)            | ESC        | Enable Slot Comparison. The EMAC_DMA1_CHSFCS.ESC bits,When set, this bit enables the checking of the slot numbers, programmed in the transmit descriptor, with the current reference given in Bits [19:16]. The DMAfetches the data from the corresponding buffer only when the slot number is equal to the reference slot number or is ahead of the reference slot number by one slot. When reset, this bit disables the checking of the slot numbers. The DMAfetches the data immediately after the descriptor is processed. |

## Channel 1 Send Slope Credit Value Register

The EMAC\_DMA1\_CHSSC register provides the bandwidth that is available for the AV traffic on other channels. This register is present only when you select the T ransmit Channel 1 in the AV mode.

Figure 29-52: EMAC\_DMA1\_CHSSC Register Diagram

<!-- image -->

Table 29-84: EMAC\_DMA1\_CHSSC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------|
| 13:0               | SSC        | Send Slope Credit. The EMAC_DMA1_CHSSC.SSC bit field contains the sendSlopeCredit value re- |
| (R/W)              |            | quired for credit-based shaper algorithm for Channel 1.                                     |

## DMA Interrupt Enable Register

The EMAC\_DMA1\_IEN register enables (unmasks) EMAC DMA interrupts.

Figure 29-53: EMAC\_DMA1\_IEN Register Diagram

<!-- image -->

Table 29-85: EMAC\_DMA1\_IEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | NIE        | Normal Interrupt Summary Enable. The EMAC_DMA1_IEN.NIE bit, when set, enables a normal interrupt. When this bit is reset, a normal interrupt is disabled. This bit enables the following bits: EMAC_DMA1_STAT.TI , EMAC_DMA1_STAT.TU , EMAC_DMA1_STAT.RI , and EMAC_DMA1_STAT.ERI .                                                                                               |
| 15 (R/W)           | AIE        | Abnormal Interrupt Summary Enable. The EMAC_DMA1_IEN.AIE bit, when set, enables an abnormal interrupt. When this bit is reset, an Abnormal Interrupt is disabled. This bit enables the following bits: EMAC_DMA1_STAT.TPS , EMAC_DMA1_STAT.TJT , EMAC_DMA1_STAT.OVF , EMAC_DMA1_STAT.RU , EMAC_DMA1_STAT.RPS , EMAC_DMA1_STAT.RWT , EMAC_DMA1_STAT.ETI , and EMAC_DMA1_STAT.FBI . |

Table 29-85: EMAC\_DMA1\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | ERE        | Early Receive Interrupt Enable. The EMAC_DMA1_IEN.ERE bit, when set (and with EMAC_DMA1_IEN.NIE =1), enables the Early Receive Interrupt. When this bit is reset, Early Receive Interrupt is disabled.                                  |
| 13 (R/W)           | FBE        | Fatal Bus Error Enable. The EMAC_DMA1_IEN.FBE bit, when set (and with EMAC_DMA1_IEN.AIE =1), enables the Fatal Bus Error Interrupt. When this bit is reset, Fatal Bus Error Enable Interrupt is disabled.                               |
| 10 (R/W)           | ETE        | Early Transmit Interrupt Enable. The EMAC_DMA1_IEN.ETE bit, when this bit is set (and with EMAC_DMA1_IEN.AIE =1), enables the Early Transmit Interrupt. When this bit is reset, Early Transmit Interrupt is disabled.                   |
| 9 (R/W)            | RWE        | Receive Watchdog Timeout Enable. The EMAC_DMA1_IEN.RWE bit, when set (and with EMAC_DMA1_IEN.AIE =1), enables the Receive Watchdog Timeout Interrupt. When this bit is reset, Receive Watchdog Timeout Interrupt is disabled.           |
| 8 (R/W)            | RSE        | Receive Stopped Enable. The EMAC_DMA1_IEN.RSE bit, when set (and with EMAC_DMA1_IEN.AIE =1), enables the Receive Stopped Interrupt is enabled. When this bit is reset, Receive Stop- ped Interrupt is disabled.                         |
| 7 (R/W)            | RUE        | Receive Buffer Unavailable Enable. The EMAC_DMA1_IEN.RUE bit, when set (and with EMAC_DMA1_IEN.AIE =1), enables the Receive Buffer Unavailable Interrupt. When this bit is reset, the Receive Buffer Unavailable Interrupt is disabled. |
| 6 (R/W)            | RIE        | Receive Interrupt Enable. The EMAC_DMA1_IEN.RIE bit, when set (and with EMAC_DMA1_IEN.NIE =1), enables the Receive Interrupt. When this bit is reset, Receive Interrupt is disabled.                                                    |
| 5 (R/W)            | UNE        | Underflow Interrupt Enable. The EMAC_DMA1_IEN.UNE bit, when set (and with EMAC_DMA1_IEN.AIE =1), enables the Transmit Underflow Interrupt. When this bit is reset, Underflow Interrupt is disabled.                                     |
| 4 (R/W)            | OVE        | Overflow Interrupt Enable. The EMAC_DMA1_IEN.OVE bit, when set (and with EMAC_DMA1_IEN.AIE =1), enables the Receive Overflow Interrupt. When this bit is reset, Overflow Interrupt is disabled.                                         |

Table 29-85: EMAC\_DMA1\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | TJE        | Transmit Jabber Timeout Enable. The EMAC_DMA1_IEN.TJE bit, when set (and with EMAC_DMA1_IEN.AIE =1), enables the Transmit Jabber Timeout Interrupt. When this bit is reset, Transmit Jabber Timeout Interrupt is disabled.             |
| 2 (R/W)            | TUE        | Transmit Buffer Unavailable Enable. The EMAC_DMA1_IEN.TUE bit, when set (and with EMAC_DMA1_IEN.NIE =1), enables the Transmit Buffer Unavailable Interrupt. When this bit is reset, Transmit Buffer Unavailable Interrupt is disabled. |
| 1 (R/W)            | TSE        | Transmit Stopped Enable. The EMAC_DMA1_IEN.TSE bit, when set (and with EMAC_DMA1_IEN.AIE =1), enables the Transmission Stopped Interrupt. When this bit is reset, Transmission Stop- ped Interrupt is disabled.                        |
| 0 (R/W)            | TIE        | Transmit Interrupt Enable. The EMAC_DMA1_IEN.TIE bit, when set (and with EMAC_DMA1_IEN.NIE =1), enables the Transmit Interrupt. When this bit is reset, Transmit Interrupt is disabled.                                                |

## DMA Missed Frame Register

The EMAC\_DMA1\_MISS\_FRM register contains counters for EMAC DMA missed frames and buffer overflows.

Figure 29-54: EMAC\_DMA1\_MISS\_FRM Register Diagram

<!-- image -->

Table 29-86: EMAC\_DMA1\_MISS\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (RC/NW)         | OVFFIFO    | Overflow bit for FIFO Overflow Counter. The EMAC_DMA1_MISS_FRM.OVFFIFO bit holds the overflow bit for FIFO Overflow Counter.                                                               |
| 27:17 (RC/NW)      | MISSFROV   | Missed Frames Buffer Overflow. The EMAC_DMA1_MISS_FRM.MISSFROV bits indicate the number of frames missed by the application due to buffer overflow.                                        |
| 16 (RC/NW)         | OVFMISS    | Overflow bit for Missed Frame Counter. The EMAC_DMA1_MISS_FRM.OVFMISS bit holds the overflow bit for the Missed Frame Counter.                                                             |
| 15:0 (RC/NW)       | MISSFRUN   | Missed Frames Unavailable Buffer. The EMAC_DMA1_MISS_FRM.MISSFRUN bits indicate the number of frames missed by the controller because of the Application Receive Buffer being unavailable. |

## DMA Operation Mode Register

The EMAC\_DMA1\_OPMODE register selects receive and transmit DMA operating modes.

Figure 29-55: EMAC\_DMA1\_OPMODE Register Diagram

<!-- image -->

Table 29-87: EMAC\_DMA1\_OPMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | DT         | Disable Dropping TCP/IP Errors. The EMAC_DMA1_OPMODE.DT bit, when set, directs the core not to drop frames that only have errors detected by the Receive Checksum Offload engine. Such frames do not have any errors (including FCS error) in the Ethernet frame received by the MAC but have errors in the encapsulated payload only. When this bit is reset, all error frames are dropped if the EMAC_DMA1_OPMODE.FEF bit is reset. |
| 25 (R/W)           | RSF        | Receive Store and Forward. The EMAC_DMA1_OPMODE.RSF bit, when set, directs the MFL only to read a frame from the Rx FIFO after the complete frame has been written to it, ignoring the EMAC_DMA1_OPMODE.RTC bits. When this bit is reset, the Rx FIFO operates in threshold mode, subject to the threshold specified by the EMAC_DMA1_OPMODE.RTC bits.                                                                                |
| 24 (R/W)           | DFF        | Disable Flushing of received Frames. The EMAC_DMA1_OPMODE.DFF bit, when set, directs the Rx DMAnot to flush any frames because of the unavailability of receive descriptors/buffers as it does nor- mally when this bit is reset.                                                                                                                                                                                                     |

Table 29-87: EMAC\_DMA1\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | TSF        | Transmit Store and Forward. The EMAC_DMA1_OPMODE.TSF bit, when set, starts transmission when a full frame resides in the MFL Transmit FIFO. When this bit is set, the TTC values speci- fied in Register 6[16:14] are ignored. This bit should be changed only when transmis- sion is stopped.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Transmit Store and Forward. The EMAC_DMA1_OPMODE.TSF bit, when set, starts transmission when a full frame resides in the MFL Transmit FIFO. When this bit is set, the TTC values speci- fied in Register 6[16:14] are ignored. This bit should be changed only when transmis- sion is stopped.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 20 (R/W1S)         | FTF        | Flush Transmit FIFO. The EMAC_DMA1_OPMODE.FTF bit, when set, directs the transmit FIFO controller logic to reset to its default values and thus all data in the Tx FIFO is lost/flushed. This bit is cleared internally when the flushing operation is completed fully. The Operation Mode register should not be written to until this bit is cleared. The data which is al- ready accepted by the MAC transmitter is not flushed. It is scheduled for transmission and results in underflow and runt frame transmission. Note: The flush operation com- pletes only after emptying the Tx FIFO of its contents and all the pending Transmit Status of the transmitted frames are accepted by the host. In order to complete this flush operation, the PHY transmit clock is required to be active. This field cleared to 1b0 by the core (Self Clear). The application cannot clear this type of field, and a reg- ister write of 1b0 to this bit has no effect on this field. | Flush Transmit FIFO. The EMAC_DMA1_OPMODE.FTF bit, when set, directs the transmit FIFO controller logic to reset to its default values and thus all data in the Tx FIFO is lost/flushed. This bit is cleared internally when the flushing operation is completed fully. The Operation Mode register should not be written to until this bit is cleared. The data which is al- ready accepted by the MAC transmitter is not flushed. It is scheduled for transmission and results in underflow and runt frame transmission. Note: The flush operation com- pletes only after emptying the Tx FIFO of its contents and all the pending Transmit Status of the transmitted frames are accepted by the host. In order to complete this flush operation, the PHY transmit clock is required to be active. This field cleared to 1b0 by the core (Self Clear). The application cannot clear this type of field, and a reg- ister write of 1b0 to this bit has no effect on this field. |
| 16:14 (R/W)        | TTC        | Transmit Threshold Control. The EMAC_DMA1_OPMODE.TTC bits control the threshold level of the MFL Trans- mit FIFO. Transmission starts when the frame size within the MFL Transmit FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are also transmitted. These bits are used only when the EMAC_DMA1_OPMODE.TSF bit is reset. The value =011 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Transmit Threshold Control. The EMAC_DMA1_OPMODE.TTC bits control the threshold level of the MFL Trans- mit FIFO. Transmission starts when the frame size within the MFL Transmit FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are also transmitted. These bits are used only when the EMAC_DMA1_OPMODE.TSF bit is reset. The value =011 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 64                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 128                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 192                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 256                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 40                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 32                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 24                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 16                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 29-87: EMAC\_DMA1\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | ST         | Start/Stop Transmission. The EMAC_DMA1_OPMODE.ST bit, when set, places transmission in the Running state, and the DMAchecks the Transmit List at the current position for a frame to be transmitted. Descriptor acquisition is attempted either from the current position in the list, which is the Transmit List Base Address set by Transmit Descriptor List Address, or from the position retained when transmission was stopped previously. If the current descriptor is not owned by the DMA, transmission enters the Suspended state, and the EMAC_DMA1_STAT.TU bit is set. The Start Transmission command is effective only when transmission is stopped. If the command is issued before setting the EMAC_DMA1_TXDSC_CUR address register, then the DMAbehavior is unpredictable. When this bit is reset, the transmission proc- ess is placed in the Stopped state after completing the transmission of the current frame. The Next Descriptor position in the Transmit List is saved, and becomes the current position when transmission is restarted. The stop transmission command is ef- fective only when the transmission of the current frame is complete or the transmission is in the Suspended state. |
| 7 (R/W)            | FEF        | Forward Error Frames. The EMAC_DMA1_OPMODE.FEF bit, when reset, directs the Rx FIFO to drop frames with error status (CRC error, collision error, giant frame, watchdog timeout, overflow). However, if the frames start byte (write) pointer is already transferred to the read controller side (in Threshold mode), then the frames are not dropped. When EMAC_DMA1_OPMODE.FEF bit is set, all frames except runt error frames are for- warded to the DMA. But when Rx FIFO overflows when a partial frame is written, then such frames are dropped even when EMAC_DMA1_OPMODE.FEF is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 6 (R/W)            | FUF        | Forward Undersized Good Frames. The EMAC_DMA1_OPMODE.FUF bit, when set, directs the Rx FIFO to forward Un- dersized frames (frames with no Error and length less than 64 bytes) including pad- bytes and CRC). When reset, the Rx FIFO drops all frames of less than 64 bytes, un- less it is already transferred because of lower value of Receive Threshold (for example, EMAC_DMA1_OPMODE.RTC =01).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 5 (R/W)            | DGF        | Drop Giant Frames. The EMAC_DMA1_OPMODE.DGF bit, when set, the MAC drops the received giant frames in the Rx FIFO, that is, frames that are larger than the computed giant frame limit. When reset, the MAC does not drop the giant frames in the Rx FIFO.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 29-87: EMAC\_DMA1\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:3 (R/W)          | RTC        | Receive Threshold Control. The EMAC_DMA1_OPMODE.RTC bits control the threshold level of the MFL Re- ceive FIFO. Transfer (request) to DMAstarts when the frame size within the MFL Re- ceive FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are transferred automatically. These bits are valid only when the EMAC_DMA1_OPMODE.RSF bit is zero, and are ignored when the EMAC_DMA1_OPMODE.RSF bit is set to 1. The value =11 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 2 (R/W)            | OSF        | Operate on Second Frame. The EMAC_DMA1_OPMODE.OSF bit, when set, instructs the DMAto process a sec- ond frame of Transmit data even before status for first frame is obtained.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 1 (R/W)            | SR         | Start/Stop Receive. The EMAC_DMA1_OPMODE.SR bit, when set, places the Receive process in the Run- ning state. The DMAattempts to acquire the descriptor from the Receive list and processes incoming frames. Descriptor acquisition is attempted from the current posi- tion in the list, which is the address set by DMAReceive Descriptor List Address or the position retained when the Receive process was previously stopped. If no descriptor is owned by the DMA, reception is suspended, and the EMAC_DMA1_STAT.RU bit is set. The Start Receive command is effective only when reception has stopped. If the com- mand was issued before setting EMAC_DMA1_RXDSC_CUR address register,DMA behavior is unpredictable. When this bit is cleared, Rx DMAoperation is stopped after the transfer of the current frame. The next descriptor position in the Receive list is saved and becomes the current position after the Receive process is restarted. The Stop Receive command is effective only when the Receive process is in either the Running (waiting for receive packet) or in the Suspended state. |

## DMA Rx Buffer Current Register

The EMAC\_DMA1\_RXBUF\_CUR register holds the pointer to the current receive DMA buffer.

Figure 29-56: EMAC\_DMA1\_RXBUF\_CUR Register Diagram

<!-- image -->

Table 29-88: EMAC\_DMA1\_RXBUF\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Receive Current Buffer Address. The EMAC_DMA1_RXBUF_CUR.ADDR bit field points to the current Receive Buffer address being read by the DMA. Pointer updated by DMAduring operation. Cleared on Reset. |

## DMA Rx Descriptor List Address Register

The EMAC\_DMA1\_RXDSC\_ADDR register holds the address for the DMA receive descriptor list. Writing to this Register is permitted only when reception is stopped. When stopped, this must be written to before the receive Start command is given. The processor can write to EMAC\_DMA1\_RXDSC\_ADDR only when Rx DMA has stopped ( EMAC\_DMA1\_OPMODE.SR bit =0). When stopped, it can be written with a new descriptor list address. When the processor sets the EMAC\_DMA1\_OPMODE.SR bit to 1, the DMA takes the newly programmed descriptor base address. If this register is not changed when the EMAC\_DMA1\_OPMODE.SR bit is cleared to 0, the DMA takes the descriptor address where it was stopped earlier.

Figure 29-57: EMAC\_DMA1\_RXDSC\_ADDR Register Diagram

<!-- image -->

Table 29-89: EMAC\_DMA1\_RXDSC\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Start of Receive List. The EMAC_DMA1_RXDSC_ADDR.VALUE bit field contains the base address of the First Descriptor in the Receive Descriptor list. The LSB bits [1:0] for the 32bit bus width are ignored and are taken as all-zero by the DMAinternally. Therefore, these LSB bits are Read-Only (RO). |

## DMA Rx Descriptor Current Register

The EMAC\_DMA1\_RXDSC\_CUR register contains the current DMA receive descriptor.

Figure 29-58: EMAC\_DMA1\_RXDSC\_CUR Register Diagram

<!-- image -->

Table 29-90: EMAC\_DMA1\_RXDSC\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Receive Current Descriptor Address. The EMAC_DMA1_RXDSC_CUR.ADDR bit field points to the start address of the current Receive Descriptor read by the DMA. Pointer updated by DMAduring opera- tion. Cleared on Reset. |

## DMA Rx Interrupt Watch Dog Register

The EMAC\_DMA1\_RXIWDOG register contains the timeout value for the EMAC DMA receive interrupt watch dog timer.

Figure 29-59: EMAC\_DMA1\_RXIWDOG Register Diagram

<!-- image -->

Table 29-91: EMAC\_DMA1\_RXIWDOG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | RIWT       | RI Watch Dog Timer Count. The EMAC_DMA1_RXIWDOG.RIWT bit field indicates the number of system clock cycles multiplied by 256 for which the watchdog timer is set. The watchdog timer gets triggered with the programmed value after the Rx DMAcompletes the transfer of a frame for which the RI status bit is not set because of the setting in the corresponding descriptor RDES1[31]. When the watchdog timer runs out, the RI bit is set and the timer is stopped. The watchdog timer is reset when EMAC_DMA1_STAT.RI bit is set high because of automatic setting of EMAC_DMA1_STAT.RI as per RDES1[31] of any received frame. |

## DMA Rx Poll Demand Register

The EMAC\_DMA1\_RXPOLL register directs the EMAC to poll the receive descriptor list.

Figure 29-60: EMAC\_DMA1\_RXPOLL Register Diagram

<!-- image -->

Table 29-92: EMAC\_DMA1\_RXPOLL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | START      | Receive Poll Demand. The EMAC_DMA1_RXPOLL.START bits, when written with any value, cause the DMAto read the current descriptor pointed to by the EMAC_DMA1_RXDSC_CUR register. If that descriptor is not available (owned by application), reception returns to the Suspended state, and the EMAC_DMA1_STAT.RU bit is asserted. If the descriptor is available, the Receive DMAreturns to the active state. |

## DMA Status Register

The EMAC\_DMA1\_STAT register indicates EMAC DMA status.

Figure 29-61: EMAC\_DMA1\_STAT Register Diagram

<!-- image -->

Table 29-93: EMAC\_DMA1\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/NW)          | GTMSI      | MAC TMS Interrupt. MAC TMS Interrupt: The EMAC_DMA1_STAT.GTMSI bit indicates an interrupt event in the traffic manager and scheduler logic. To reset this bit, the software must read the corresponding registers (Channel Status Register) to get the exact cause of the interrupt and clear its source. |

Table 29-93: EMAC\_DMA1\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/NW)          | TTI        | Time Stamp Trigger Interrupt. The EMAC_DMA1_STAT.TTI bit indicates an interrupt event in the MAC core's Time Stamp Generator block. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear its source to reset this bit to =0. When this bit is high, the interrupt signal from the MAC is high. | Time Stamp Trigger Interrupt. The EMAC_DMA1_STAT.TTI bit indicates an interrupt event in the MAC core's Time Stamp Generator block. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear its source to reset this bit to =0. When this bit is high, the interrupt signal from the MAC is high. |
| 27 (R/NW)          | MCI        | MAC MMCInterrupt. The EMAC_DMA1_STAT.MCI bit reflects an interrupt event in the MMCmodule of the MAC core. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear the source of interrupt to make this bit as =0. The interrupt signal from the MAC is high when this bit is high.               | MAC MMCInterrupt. The EMAC_DMA1_STAT.MCI bit reflects an interrupt event in the MMCmodule of the MAC core. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear the source of interrupt to make this bit as =0. The interrupt signal from the MAC is high when this bit is high.               |
| 26 (R/NW)          | GLI        | Line Interface Interrupt. The EMAC_DMA1_STAT.GLI bit When set, this bit reflects any of the following in- terrupt events in the DWC_gmac interfaces                                                                                                                                                                                                        | Line Interface Interrupt. The EMAC_DMA1_STAT.GLI bit When set, this bit reflects any of the following in- terrupt events in the DWC_gmac interfaces                                                                                                                                                                                                        |
| 25:23 (R/NW)       | EB         | Error Bits. The EMAC_DMA1_STAT.EB bits indicate the type of error that caused a Bus Error (for example, error response on the SCB interface). These bits are valid only when the EMAC_DMA1_STAT.FBI bit is set. This field does not generate an interrupt.                                                                                                 | Error Bits. The EMAC_DMA1_STAT.EB bits indicate the type of error that caused a Bus Error (for example, error response on the SCB interface). These bits are valid only when the EMAC_DMA1_STAT.FBI bit is set. This field does not generate an interrupt.                                                                                                 |
| 25:23 (R/NW)       | EB         | 0                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, write transfer, RxDMA                                                                                                                                                                                                                                                                                                     |
| 25:23 (R/NW)       | EB         | 1                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, write transfer, TxDMA                                                                                                                                                                                                                                                                                                     |
| 25:23 (R/NW)       | EB         | 2                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, read transfer, RxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 3                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, read transfer, TxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 4                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, write transfer, RxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 5                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, write transfer, TxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 6                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, read transfer, RxDMA                                                                                                                                                                                                                                                                                                       |
| 25:23 (R/NW)       | EB         | 7                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, read transfer, TxDMA                                                                                                                                                                                                                                                                                                       |
| 22:20 (R/NW)       | TS         | Tx Process State. The EMAC_DMA1_STAT.TS bits indicate the transmit DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                    | Tx Process State. The EMAC_DMA1_STAT.TS bits indicate the transmit DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                    |
| 22:20 (R/NW)       | TS         | 0                                                                                                                                                                                                                                                                                                                                                          | Stopped; Reset or Stop Tx Command Issued                                                                                                                                                                                                                                                                                                                   |
| 22:20 (R/NW)       | TS         | 1                                                                                                                                                                                                                                                                                                                                                          | Running; Fetching Tx Transfer Descriptor                                                                                                                                                                                                                                                                                                                   |
| 22:20 (R/NW)       | TS         | 2                                                                                                                                                                                                                                                                                                                                                          | Running; Waiting for Status                                                                                                                                                                                                                                                                                                                                |
| 22:20 (R/NW)       | TS         | 3                                                                                                                                                                                                                                                                                                                                                          | Reading Data from Host Memory Buffer and Queuing It to Tx Buffer                                                                                                                                                                                                                                                                                           |
| 22:20 (R/NW)       | TS         | 4                                                                                                                                                                                                                                                                                                                                                          | TIME_STAMP Write State                                                                                                                                                                                                                                                                                                                                     |

Table 29-93: EMAC\_DMA1\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Suspended; Tx Descriptor Unavailable or Tx Buffer Un- derflow                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Closing Tx Descriptor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 19:17 (R/NW)       | RS         | Rx Process State. The EMAC_DMA1_STAT.RS bits indicate the receive DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Rx Process State. The EMAC_DMA1_STAT.RS bits indicate the receive DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 19:17 (R/NW)       |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Stopped: Reset or Stop Rx Command Issued.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 19:17 (R/NW)       |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Running: Fetching Rx Transfer Descriptor.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 19:17 (R/NW)       |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 19:17 (R/NW)       |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Running: Waiting for Rx Packet                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 19:17 (R/NW)       |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Suspended: Rx Descriptor Unavailable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 19:17 (R/NW)       |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Running: Closing Rx Descriptor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 19:17 (R/NW)       |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | TIME_STAMP Write State                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 19:17 (R/NW)       |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Running: Transferring Rx Packet Data from Rx Buffer to Host Memory                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 16 (R/W1C)         | NIS        | Normal Interrupt Summary. The value of the EMAC_DMA1_STAT.NIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA1_STAT.TI , EMAC_DMA1_STAT.TU ,                                                                                                                                                                                                                                                                                                                                        | Normal Interrupt Summary. The value of the EMAC_DMA1_STAT.NIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA1_STAT.TI , EMAC_DMA1_STAT.TU ,                                                                                                                                                                                                                                                                                                                                        |
| 15 (R/W1C)         | AIS        | EMAC_DMA1_STAT.NIS to be set is cleared. Abnormal Interrupt Summary. The value of the EMAC_DMA1_STAT.AIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA1_IEN.TSE , EMAC_DMA1_IEN.TJE , EMAC_DMA1_IEN.OVE , EMAC_DMA1_IEN.UNE , EMAC_DMA1_IEN.RUE , EMAC_DMA1_IEN.RSE , EMAC_DMA1_IEN.RWE , EMAC_DMA1_IEN.ETE , and EMAC_DMA1_IEN.FBE . Only unmasked bits affect the Abnormal Interrupt Sum- mary bit. This is a sticky bit and must be cleared each time a corresponding bit that | EMAC_DMA1_STAT.NIS to be set is cleared. Abnormal Interrupt Summary. The value of the EMAC_DMA1_STAT.AIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA1_IEN.TSE , EMAC_DMA1_IEN.TJE , EMAC_DMA1_IEN.OVE , EMAC_DMA1_IEN.UNE , EMAC_DMA1_IEN.RUE , EMAC_DMA1_IEN.RSE , EMAC_DMA1_IEN.RWE , EMAC_DMA1_IEN.ETE , and EMAC_DMA1_IEN.FBE . Only unmasked bits affect the Abnormal Interrupt Sum- mary bit. This is a sticky bit and must be cleared each time a corresponding bit that |
| 14 (R/W1C)         | ERI        | Early Receive Interrupt. The EMAC_DMA1_STAT.ERI bit indicates that the DMAhad filled the first data                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Early Receive Interrupt. The EMAC_DMA1_STAT.ERI bit indicates that the DMAhad filled the first data                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 29-93: EMAC\_DMA1\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1C)         | FBI        | Fatal Bus Error Interrupt. The EMAC_DMA1_STAT.FBI bit indicates that a bus error occurred, as detailed in the EMAC_DMA1_STAT.EB field. When this bit is set, the corresponding DMAen- gine disables all its bus accesses.                                                                                                                                                                                                                                                                                                                                        |
| 10 (R/W1C)         | ETI        | Early Transmit Interrupt. The EMAC_DMA1_STAT.ETI bit indicates that the frame to be transmitted was fully transferred to the MFL Transmit FIFO.                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 9 (R/W1C)          | RWT        | Receive WatchDog Timeout. The EMAC_DMA1_STAT.RWT bit is asserted when a frame with a length greater than 2,048 bytes is received (10, 240 when Jumbo Frame mode is enabled).                                                                                                                                                                                                                                                                                                                                                                                     |
| 8 (R/W1C)          | RPS        | Receive Process Stopped. The EMAC_DMA1_STAT.RPS bit is asserted when the Receive Process enters the Stopped state.                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 7 (R/W1C)          | RU         | Receive Buffer Unavailable. The EMAC_DMA1_STAT.RU bit indicates that the Next Descriptor in the Receive List is owned by the application and cannot be acquired by the DMA. Receive Process is suspended. To resume processing Receive descriptors, the application should change the ownership of the descriptor and issue a Receive Poll Demand command. If no Re- ceive Poll Demand is issued, Receive Process resumes when the next recognized incom- ing frame is received. This bit is set only when the previous Receive Descriptor was owned by the DMA. |
| 6 (R/W1C)          | RI         | Receive Interrupt. The EMAC_DMA1_STAT.RI bit indicates the completion of frame reception. Specif- ic frame status information has been posted in the descriptor. Reception remains in the Running state.                                                                                                                                                                                                                                                                                                                                                         |
| 5 (R/W1C)          | UNF        | Transmit Buffer Underflow. The EMAC_DMA1_STAT.UNF bit indicates that the Transmit Buffer had an Under- flow during frame transmission. Transmission is suspended and an Underflow Error TDES0[1] is set.                                                                                                                                                                                                                                                                                                                                                         |
| 4 (R/W1C)          | OVF        | Receive Buffer Overflow. The EMAC_DMA1_STAT.OVF bit indicates that the Receive Buffer had an Overflow during frame reception. If the partial frame is transferred to application, the overflow status is set in RDES0[11].                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W1C)          | TJT        | Transmit Jabber Timeout. The EMAC_DMA1_STAT.TJT bit indicates that the Transmit Jabber Timer expired, meaning that the transmitter had been excessively active. The transmission process is aborted and placed in the Stopped state. This causes the Transmit Jabber Timeout TDES0[14] flag to assert.                                                                                                                                                                                                                                                           |

Table 29-93: EMAC\_DMA1\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | TU         | Transmit Buffer Unavailable. The EMAC_DMA1_STAT.TU bit indicates that the Next Descriptor in the Transmit List is owned by the application and cannot be acquired by the DMA. Transmission is suspended. The value in the EMAC_DMA1_STAT.TS bits explain the Transmit Proc- ess state transitions. To resume processing transmit descriptors, the application should change the ownership of the bit of the descriptor and then issue a Transmit Poll De- mand command. |
| 1 (R/W1C)          | TPS        | Transmit Process Stopped. The EMAC_DMA1_STAT.TPS bit is set when the transmission is stopped.                                                                                                                                                                                                                                                                                                                                                                           |
| 0 (R/W1C)          | TI         | Transmit Interrupt. The EMAC_DMA1_STAT.TI bit indicates that frame transmission is finished and TDES1[31] is set in the First Descriptor.                                                                                                                                                                                                                                                                                                                               |

## DMA Tx Buffer Current Register

The EMAC\_DMA1\_TXBUF\_CUR register holds the pointer to the current transmit DMA buffer.

Figure 29-62: EMAC\_DMA1\_TXBUF\_CUR Register Diagram

<!-- image -->

Table 29-94: EMAC\_DMA1\_TXBUF\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Transmit Current Buffer Address. The EMAC_DMA1_TXBUF_CUR.ADDR bit field points to the current Transmit Buf- fer Address being read by the DMA. Pointer updated by DMAduring operation. Cleared on Reset. |

## DMA Tx Descriptor List Address Register

The EMAC\_DMA1\_TXDSC\_ADDR register holds the address for the DMA transmit descriptor list. The processor can write to this Register only when Tx DMA has stopped ( EMAC\_DMA1\_OPMODE.ST bit =0). When stopped, this can be written with a new descriptor list address. When the processor sets the EMAC\_DMA1\_OPMODE.ST bit to 1, the DMA takes the newly programmed descriptor base address. If this register is not changed when the EMAC\_DMA1\_OPMODE.ST bit is cleared to 0, then the DMA takes the descriptor address where it was stopped earlier.

Figure 29-63: EMAC\_DMA1\_TXDSC\_ADDR Register Diagram

<!-- image -->

Table 29-95: EMAC\_DMA1\_TXDSC\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Start of Transmit List. The EMAC_DMA1_TXDSC_ADDR.VALUE bit field contains the base address of the First Descriptor in the Transmit Descriptor list. The LSB bits [1:0] for 32bit bus width are ignored and are taken as all-zero by the DMAinternally. Therefore, these LSB bits are Read-Only (RO). |

## DMA Tx Descriptor Current Register

The EMAC\_DMA1\_TXDSC\_CUR register contains the current DMA transmit descriptor.

Figure 29-64: EMAC\_DMA1\_TXDSC\_CUR Register Diagram

<!-- image -->

Table 29-96: EMAC\_DMA1\_TXDSC\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Transmit Descriptor Address. The EMAC_DMA1_TXDSC_CUR.ADDR bit field points to the start address of the current Transmit Descriptor read by the DMA. Pointer updated by DMAduring oper- ation. Cleared on Reset. |

## DMA Tx Poll Demand Register

The EMAC\_DMA1\_TXPOLL register directs the EMAC to poll the transmit descriptor list.

Figure 29-65: EMAC\_DMA1\_TXPOLL Register Diagram

<!-- image -->

Table 29-97: EMAC\_DMA1\_TXPOLL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | START      | Transmit Poll Demand. The EMAC_DMA1_TXPOLL.START bits, when written with any value, cause the DMAto read the current descriptor pointed to by EMAC_DMA1_TXDSC_CUR regis- ter. If that descriptor is not available (owned by application), transmission returns to the Suspend state, and the EMAC_DMA1_STAT.TU bit is asserted. If the descriptor is available, transmission resumes. |

## DMA Bus Mode Register

The EMAC\_DMA2\_BUSMODE register selects the DMA bus operating modes for EMAC DMA.

Figure 29-66: EMAC\_DMA2\_BUSMODE Register Diagram

<!-- image -->

Table 29-98: EMAC\_DMA2\_BUSMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | AAL        | Address Aligned Bursts. The EMAC_DMA2_BUSMODE.AAL bit, when set high and the FB bit equals 1, di- rects the SCB interface to generate all bursts aligned to the start address LS bits. If the FB bit is equal to 0, the first burst (accessing the data buffers start address) is not aligned, but subsequent bursts are aligned to the address. |
| 24 (R/W)           | PBL8       | PBL * 8. The EMAC_DMA2_BUSMODE.PBL8 bit, when set high, multiplies the PBL value programmed (bits [22:17] and bits [13:8]) eight times. Therefore, the DMAtransfers the data in 8, 16, and 32 beats depending on the PBL value.                                                                                                                  |
| 23 (R/W)           | USP        | Use Separate PBL. The EMAC_DMA2_BUSMODE.USP bit, when set high, configures the Rx DMAto use the value configured in bits [22:17] as PBL while the PBL value in bits [13:8] is applicable to Tx DMAoperations only.                                                                                                                               |

Table 29-98: EMAC\_DMA2\_BUSMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22:17 (R/W)        | RPBL       | Receive Programmable Burst Length. The EMAC_DMA2_BUSMODE.RPBL bits indicate the maximum number of beats to be transferred in one Rx DMAtransaction. This is the maximum value that is used in a single block Read/Write. The Rx DMAalways attempts to burst as specified in RPBL every time it starts a Burst transfer on the host bus. RPBL can be programmed with permissible values of 1, 2, 4, 8, 16, and 32. Any other value results in undefined behavior. These bits are valid and applicable only when USP is set high.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 16 (R/W)           | FB         | Fixed Burst. The EMAC_DMA2_BUSMODE.FB bit controls whether the SCB Master interface performs fixed burst transfers or not. See the EMAC_DMA0_BMMODE.UNDEF bit de- scription for more information.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 13:8 (R/W)         | PBL        | Programmable Burst Length. The EMAC_DMA2_BUSMODE.PBL bits indicate the maximum number of beats to be transferred in one DMAtransaction. This is the maximum value that is used in a single block Read/Write. The DMAalways attempts to burst as specified in PBL each time it starts a Burst transfer on the host bus. Any other value results in undefined be- havior. When USP is set high, this PBL value is applicable for Tx DMAtransactions only. PBL-max limit = (FIFO size / 2) / 4. PBL-max limit (transmit) = 256 bytes / 2 /4 = 32. PBL-max limit (receive) = 128 bytes / 2 /4 = 16. Note that this PBL is at the DMAend. If PBL= 32 and if BLEN16 is enabled, the DMAautomatically splits 32 bursts in to 2 x 16 bursts. If EMAC_DMA2_BUSMODE.PBL =8, and if EMAC_DMA0_BMMODE.BLEN16 is ena- bled, the max burst is limited to EMAC_DMA0_BMMODE.BLEN8 . If EMAC_DMA2_BUSMODE.PBL8 bit is set, the programmed PBL value is multiplied by 8 times internally. However, the result cannot be more than the above maximum limits specified above. |
| 7 (R/W)            | ATDS       | Alternate Descriptor Size. The EMAC_DMA2_BUSMODE.ATDS bit, when set, increases the size of the alternate descriptor to 32 bytes (8 DWORDS). This is required when the Advanced Time Stamp feature or Full IPC Offload Engine is enabled in the receiver. When reset, the descriptor size reverts back to 4 DWORDs (16 bytes). The enhanced descriptor is not required if the Advanced Time Stamp and IPC Full Checksum Offload features are not enabled. In such case, you can use the 16 bytes descriptor to save 4 bytes of memory.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 29-98: EMAC\_DMA2\_BUSMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:2 (R/W)          | DSL        | Descriptor Skip Length. The EMAC_DMA2_BUSMODE.DSL bit specifies the number of 32-bit words to skip between two unchained descriptors. The address skipping starts from the end of cur- rent descriptor to the start of next descriptor. When DSL value is equal to zero, then the descriptor table is taken as contiguous by the DMA, in Ring mode.                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 0 (R/W1S)          | SWR        | Software Reset. The EMAC_DMA2_BUSMODE.SWR bit, when set, directs the MAC DMAController to reset all MAC Subsystem internal registers and logic. It is cleared automatically after the reset operation has completed in all of the core clock domains. Read a 0 value in this bit before re-programming any register of the core. Note: The reset operation is completed only when all the resets in all the active clock domains are de-asserted. Therefore, it is essential that all the PHY inputs clocks (applicable for the selected PHY interface) are present for software reset completion. This field cleared to 1b0 by the core (Self Clear). The application cannot clear this type of field, and a register write of 1b0 to this bit has no effect on this field. |

## Channel 2 Credit Shaping Control Register

The EMAC\_DMA2\_CHCBSCTL register controls the credit-based shaper algorithm in the T raffic Manager for scheduling the frames for transmission. This register is present only when you select the T ransmit Channel 1 in the AV mode.

Figure 29-67: EMAC\_DMA2\_CHCBSCTL Register Diagram

<!-- image -->

Table 29-99: EMAC\_DMA2\_CHCBSCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | ABPSSIE    | Average bits per slot Interrupt Enable. The EMAC_DMA2_CHCBSCTL.ABPSSIE bit asserts an interrupt (sbd_intr_o or mci_intr_o) when the average bits per slot status is updated for Channel 1. When this bit is cleared, interrupt is not asserted for such an event.                                                            |
| 6:4 (R/W)          | SLC        | Slot Count. The EMAC_DMA2_CHCBSCTL.SLC bit field programs the number of slots (of dura- tion 125 micro-sec) over which the average transmitted bits per slot (provided in the CBS Status register) need to be computed for Channel 1 when the credit-based shaper algorithm is enabled. The                                  |
| 1 (R/W)            | CC         | Credit Control. The EMAC_DMA2_CHCBSCTL.CC bit, when reset, sets the accumulated credit pa- rameter in the credit-based shaper algorithm logic to zero when there is positive credit and no frame to transmit in Channel 1.                                                                                                   |
| 0 (R/W)            | CBSD       | Credit Based Shaper Disable. The EMAC_DMA2_CHCBSCTL.CBSD bit disables the credit-based shaper algorithm for Channel 1 traffic and makes the traffic management algorithm to strict priority for Channel 1 over Channel 0. When reset, the credit-based shaper algorithm schedules the traffic in Channel 1 for transmission. |

## Channel 2 Avg Traffic Transmitted Status Register

The EMAC\_DMA2\_CHCBSSTAT register provides the average traffic transmitted in Channel 1. This register is present only when you select the Transmit Channel 1 in the AV mode.

Figure 29-68: EMAC\_DMA2\_CHCBSSTAT Register Diagram

<!-- image -->

Table 29-100: EMAC\_DMA2\_CHCBSSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/NW)          | ABSU       | ABS Updated. The EMAC_DMA2_CHCBSSTAT.ABSU bits When set, this bit indicates that the MAC has updated the ABS value. This bit is cleared when the application reads the ABS value.                                                                                                                    |
| 16:0 (R/NW)        | ABS        | Average Bits per Slot. The EMAC_DMA2_CHCBSSTAT.ABS bit field contains the average transmitted bits per slot. This field is computed over programmed number of slots ( EMAC_DMA2_CHCBSCTL.SLC bit field) for Channel 1 traffic. The maximum val- ue is 0x30D4 for 100 Mbps and 0x1E848 for 1000 Mbps. |

## Channel 2 High Credit Value Register

The EMAC\_DMA2\_CHHIC register provides the maximum value that can be accumulated for Channel 1 in the credit parameter of the credit-based shaper algorithm. This register is present only when you select the T ransmit Channel 1 in the AV mode.

Figure 29-69: EMAC\_DMA2\_CHHIC Register Diagram

<!-- image -->

Table 29-101: EMAC\_DMA2\_CHHIC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 28:0 (R/W)         | HC         | High Credit. The EMAC_DMA2_CHHIC.HC bit field contains the hiCredit value required for the credit-based shaper algorithm for Channel 1. |

## Channel 2 Idle Slope Credit Value Register

The EMAC\_DMA2\_CHISC register provides the bandwidth allocated for the AV traffic on Channel 1. This register is present only when you select the T ransmit Channel 1 in the AV mode.

Figure 29-70: EMAC\_DMA2\_CHISC Register Diagram

<!-- image -->

Table 29-102: EMAC\_DMA2\_CHISC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:0 (R/W)         | ISC        | Idle Slope Credit. The EMAC_DMA2_CHISC.ISC bit field contains the idleSlopeCredit value required for the credit-based shaper algorithm for Channel 1. |

## Channel 2 Low Credit Value Register

The EMAC\_DMA2\_CHLOC register provides the minimum value that can be accumulated for Channel 1 in the credit parameter of the credit-based shaper algorithm. This register is present only when you select T ransmit Channel 1 in the AV mode.

Figure 29-71: EMAC\_DMA2\_CHLOC Register Diagram

<!-- image -->

Table 29-103: EMAC\_DMA2\_CHLOC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------|
| 28:0 (R/W)         | LC         | Low Credit. The EMAC_DMA2_CHLOC.LC bit field contains the loCredit value required for the credit-based shaper algorithm for Channel 1. |

## Channel 2 Control Bits for Slot Function Register

The EMAC\_DMA2\_CHSFCS register controls the slot comparison feature that the Channel 1 transmit DMA uses to fetch the buffer data from system memory.

Figure 29-72: EMAC\_DMA2\_CHSFCS Register Diagram

<!-- image -->

Table 29-104: EMAC\_DMA2\_CHSFCS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/NW)       | RSN        | Reference Slot number. The EMAC_DMA2_CHSFCS.RSN bits, gives the current value of the reference slot number in DMAused for comparison checking.                                                                                                                                                                                                                                                                                                                                                               |
| 1 (R/W)            | ASC        | Advance Slot Check. The EMAC_DMA2_CHSFCS.ASC bits, When set, this bit enables the DMAto fetch the data from the buffer when the slot number (SLOTNUM) programmed in the transmit descriptor                                                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | ESC        | Enable Slot Comparison. The EMAC_DMA2_CHSFCS.ESC bit enables the checking of the slot numbers, pro- grammed in the transmit descriptor, with the current reference given in Bits [19:16]. The DMAfetches the data from the corresponding buffer only when the slot number is equal to the reference slot number or is ahead of the reference slot number by one slot. When reset, this bit disables the checking of the slot numbers. The DMAfetches the data immediately after the descriptor is processed. |

## Channel 2 Send Slope Credit Value Register

The EMAC\_DMA2\_CHSSC register provides the bandwidth that is available for the AV traffic on other channels. This register is present only when you select the T ransmit Channel 1 in the AV mode.

Figure 29-73: EMAC\_DMA2\_CHSSC Register Diagram

<!-- image -->

Table 29-105: EMAC\_DMA2\_CHSSC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------|
| 13:0               | SSC        | Send Slope Credit. The EMAC_DMA2_CHSSC.SSC bit field contains the sendSlopeCredit value re- |
| (R/W)              |            | quired for credit-based shaper algorithm for Channel 1.                                     |

## DMA Interrupt Enable Register

The EMAC\_DMA2\_IEN register enables (unmasks) EMAC DMA interrupts.

Figure 29-74: EMAC\_DMA2\_IEN Register Diagram

<!-- image -->

Table 29-106: EMAC\_DMA2\_IEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | NIE        | Normal Interrupt Summary Enable. The EMAC_DMA2_IEN.NIE bit, when set, enables a normal interrupt. When this bit is reset, a normal interrupt is disabled. This bit enables the following bits: EMAC_DMA2_STAT.TI , EMAC_DMA2_STAT.TU , EMAC_DMA2_STAT.RI , and EMAC_DMA2_STAT.ERI .                                                                                               |
| 15 (R/W)           | AIE        | Abnormal Interrupt Summary Enable. The EMAC_DMA2_IEN.AIE bit, when set, enables an abnormal interrupt. When this bit is reset, an Abnormal Interrupt is disabled. This bit enables the following bits: EMAC_DMA2_STAT.TPS , EMAC_DMA2_STAT.TJT , EMAC_DMA2_STAT.OVF , EMAC_DMA2_STAT.RU , EMAC_DMA2_STAT.RPS , EMAC_DMA2_STAT.RWT , EMAC_DMA2_STAT.ETI , and EMAC_DMA2_STAT.FBI . |

Table 29-106: EMAC\_DMA2\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | ERE        | Early Receive Interrupt Enable. The EMAC_DMA2_IEN.ERE bit, when set (and with EMAC_DMA2_IEN.NIE =1), enables the Early Receive Interrupt. When this bit is reset, Early Receive Interrupt is disabled.                                  |
| 13 (R/W)           | FBE        | Fatal Bus Error Enable. The EMAC_DMA2_IEN.FBE bit, when set (and with EMAC_DMA2_IEN.AIE =1), enables the Fatal Bus Error Interrupt. When this bit is reset, Fatal Bus Error Enable Interrupt is disabled.                               |
| 10 (R/W)           | ETE        | Early Transmit Interrupt Enable. The EMAC_DMA2_IEN.ETE bit, when this bit is set (and with EMAC_DMA2_IEN.AIE =1), enables the Early Transmit Interrupt. When this bit is reset, Early Transmit Interrupt is disabled.                   |
| 9 (R/W)            | RWE        | Receive Watchdog Timeout Enable. The EMAC_DMA2_IEN.RWE bit, when set (and with EMAC_DMA2_IEN.AIE =1), enables the Receive Watchdog Timeout Interrupt. When this bit is reset, Receive Watchdog Timeout Interrupt is disabled.           |
| 8 (R/W)            | RSE        | Receive Stopped Enable. The EMAC_DMA2_IEN.RSE bit, when set (and with EMAC_DMA2_IEN.AIE =1), enables the Receive Stopped Interrupt is enabled. When this bit is reset, Receive Stop- ped Interrupt is disabled.                         |
| 7 (R/W)            | RUE        | Receive Buffer Unavailable Enable. The EMAC_DMA2_IEN.RUE bit, when set (and with EMAC_DMA2_IEN.AIE =1), enables the Receive Buffer Unavailable Interrupt. When this bit is reset, the Receive Buffer Unavailable Interrupt is disabled. |
| 6 (R/W)            | RIE        | Receive Interrupt Enable. The EMAC_DMA2_IEN.RIE bit, when set (and with EMAC_DMA2_IEN.NIE =1), enables the Receive Interrupt. When this bit is reset, Receive Interrupt is disabled.                                                    |
| 5 (R/W)            | UNE        | Underflow Interrupt Enable. The EMAC_DMA2_IEN.UNE bit, when set (and with EMAC_DMA2_IEN.AIE =1), enables the Transmit Underflow Interrupt. When this bit is reset, Underflow Interrupt is disabled.                                     |
| 4 (R/W)            | OVE        | Overflow Interrupt Enable. The EMAC_DMA2_IEN.OVE bit, when set (and with EMAC_DMA2_IEN.AIE =1), enables the Receive Overflow Interrupt. When this bit is reset, Overflow Interrupt is disabled.                                         |

Table 29-106: EMAC\_DMA2\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | TJE        | Transmit Jabber Timeout Enable. The EMAC_DMA2_IEN.TJE bit, when set (and with EMAC_DMA2_IEN.AIE =1), enables the Transmit Jabber Timeout Interrupt. When this bit is reset, Transmit Jabber Timeout Interrupt is disabled.             |
| 2 (R/W)            | TUE        | Transmit Buffer Unavailable Enable. The EMAC_DMA2_IEN.TUE bit, when set (and with EMAC_DMA2_IEN.NIE =1), enables the Transmit Buffer Unavailable Interrupt. When this bit is reset, Transmit Buffer Unavailable Interrupt is disabled. |
| 1 (R/W)            | TSE        | Transmit Stopped Enable. The EMAC_DMA2_IEN.TSE bit, when set (and with EMAC_DMA2_IEN.AIE =1), enables the Transmission Stopped Interrupt. When this bit is reset, Transmission Stop- ped Interrupt is disabled.                        |
| 0 (R/W)            | TIE        | Transmit Interrupt Enable. The EMAC_DMA2_IEN.TIE bit, when set (and with EMAC_DMA2_IEN.NIE =1), enables the Transmit Interrupt. When this bit is reset, Transmit Interrupt is disabled.                                                |

## DMA Missed Frame Register

The EMAC\_DMA2\_MISS\_FRM register contains counters for EMAC DMA missed frames and buffer overflows.

Figure 29-75: EMAC\_DMA2\_MISS\_FRM Register Diagram

<!-- image -->

Table 29-107: EMAC\_DMA2\_MISS\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (RC/NW)         | OVFFIFO    | Overflow bit for FIFO Overflow Counter. The EMAC_DMA2_MISS_FRM.OVFFIFO bit holds the overflow bit for FIFO Overflow Counter.                                                               |
| 27:17 (RC/NW)      | MISSFROV   | Missed Frames Buffer Overflow. The EMAC_DMA2_MISS_FRM.MISSFROV bits indicate the number of frames missed by the application due to buffer overflow.                                        |
| 16 (RC/NW)         | OVFMISS    | Overflow bit for Missed Frame Counter. The EMAC_DMA2_MISS_FRM.OVFMISS bit holds the overflow bit for the Missed Frame Counter.                                                             |
| 15:0 (RC/NW)       | MISSFRUN   | Missed Frames Unavailable Buffer. The EMAC_DMA2_MISS_FRM.MISSFRUN bits indicate the number of frames missed by the controller because of the Application Receive Buffer being unavailable. |

## DMA Operation Mode Register

The EMAC\_DMA2\_OPMODE register selects receive and transmit DMA operating modes.

Figure 29-76: EMAC\_DMA2\_OPMODE Register Diagram

<!-- image -->

Table 29-108: EMAC\_DMA2\_OPMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | DT         | Disable Dropping TCP/IP Errors. The EMAC_DMA2_OPMODE.DT bit, when set, directs the core not to drop frames that only have errors detected by the Receive Checksum Offload engine. Such frames do not have any errors (including FCS error) in the Ethernet frame received by the MAC but have errors in the encapsulated payload only. When this bit is reset, all error frames are dropped if the EMAC_DMA2_OPMODE.FEF bit is reset. |
| 25 (R/W)           | RSF        | Receive Store and Forward. The EMAC_DMA2_OPMODE.RSF bit, when set, directs the MFL only to read a frame from the Rx FIFO after the complete frame has been written to it, ignoring the EMAC_DMA2_OPMODE.RTC bits. When this bit is reset, the Rx FIFO operates in threshold mode, subject to the threshold specified by the EMAC_DMA2_OPMODE.RTC bits.                                                                                |
| 24 (R/W)           | DFF        | Disable Flushing of Received Frames. The EMAC_DMA2_OPMODE.DFF bit, when set, directs the Rx DMAnot to flush any frames because of the unavailability of receive descriptors/buffers as it does nor- mally when this bit is reset.                                                                                                                                                                                                     |

Table 29-108: EMAC\_DMA2\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | TSF        | Transmit Store and Forward. The EMAC_DMA2_OPMODE.TSF bit, when set, starts transmission when a full frame resides in the MFL Transmit FIFO. When this bit is set, the TTC values speci- fied in Register 6[16:14] are ignored. This bit should be changed only when transmis- sion is stopped.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Transmit Store and Forward. The EMAC_DMA2_OPMODE.TSF bit, when set, starts transmission when a full frame resides in the MFL Transmit FIFO. When this bit is set, the TTC values speci- fied in Register 6[16:14] are ignored. This bit should be changed only when transmis- sion is stopped.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 20 (R/W1S)         | FTF        | Flush Transmit FIFO. The EMAC_DMA2_OPMODE.FTF bit, when set, directs the transmit FIFO controller logic to reset to its default values and thus all data in the Tx FIFO is lost/flushed. This bit is cleared internally when the flushing operation is completed fully. The Operation Mode register should not be written to until this bit is cleared. The data which is al- ready accepted by the MAC transmitter is not flushed. It is scheduled for transmission and results in underflow and runt frame transmission. Note: The flush operation com- pletes only after emptying the Tx FIFO of its contents and all the pending Transmit Status of the transmitted frames are accepted by the host. In order to complete this flush operation, the PHY transmit clock is required to be active. This field cleared to 1b0 by the core (Self Clear). The application cannot clear this type of field, and a reg- ister write of 1b0 to this bit has no effect on this field. | Flush Transmit FIFO. The EMAC_DMA2_OPMODE.FTF bit, when set, directs the transmit FIFO controller logic to reset to its default values and thus all data in the Tx FIFO is lost/flushed. This bit is cleared internally when the flushing operation is completed fully. The Operation Mode register should not be written to until this bit is cleared. The data which is al- ready accepted by the MAC transmitter is not flushed. It is scheduled for transmission and results in underflow and runt frame transmission. Note: The flush operation com- pletes only after emptying the Tx FIFO of its contents and all the pending Transmit Status of the transmitted frames are accepted by the host. In order to complete this flush operation, the PHY transmit clock is required to be active. This field cleared to 1b0 by the core (Self Clear). The application cannot clear this type of field, and a reg- ister write of 1b0 to this bit has no effect on this field. |
| 16:14 (R/W)        | TTC        | Transmit Threshold Control. The EMAC_DMA2_OPMODE.TTC bits control the threshold level of the MFL Trans- mit FIFO. Transmission starts when the frame size within the MFL Transmit FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are also transmitted. These bits are used only when the EMAC_DMA2_OPMODE.TSF bit is reset. The value =011 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Transmit Threshold Control. The EMAC_DMA2_OPMODE.TTC bits control the threshold level of the MFL Trans- mit FIFO. Transmission starts when the frame size within the MFL Transmit FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are also transmitted. These bits are used only when the EMAC_DMA2_OPMODE.TSF bit is reset. The value =011 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 64                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 128                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 192                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 256                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 40                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 32                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 24                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | 16                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 29-108: EMAC\_DMA2\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | ST         | Start/Stop Transmission. The EMAC_DMA2_OPMODE.ST bit, when set, places transmission in the Running state, and the DMAchecks the Transmit List at the current position for a frame to be transmitted. Descriptor acquisition is attempted either from the current position in the list, which is the Transmit List Base Address set by Transmit Descriptor List Address, or from the position retained when transmission was stopped previously. If the current descriptor is not owned by the DMA, transmission enters the Suspended state, and the EMAC_DMA2_STAT.TU bit is set. The Start Transmission command is effective only when transmission is stopped. If the command is issued before setting the EMAC_DMA2_TXDSC_CUR address register, then the DMAbehavior is unpredictable. When this bit is reset, the transmission proc- ess is placed in the Stopped state after completing the transmission of the current frame. The Next Descriptor position in the Transmit List is saved, and becomes the current position when transmission is restarted. The stop transmission command is ef- fective only when the transmission of the current frame is complete or the transmission is in the Suspended state. |
| 7 (R/W)            | FEF        | Forward Error Frames. The EMAC_DMA2_OPMODE.FEF bit, when reset, directs the Rx FIFO to drop frames with error status (CRC error, collision error, giant frame, watchdog timeout, overflow). However, if the frames start byte (write) pointer is already transferred to the read controller side (in Threshold mode), then the frames are not dropped. When EMAC_DMA2_OPMODE.FEF bit is set, all frames except runt error frames are for- warded to the DMA. But when Rx FIFO overflows when a partial frame is written, then such frames are dropped even when EMAC_DMA2_OPMODE.FEF is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 6 (R/W)            | FUF        | Forward Undersized good Frames. The EMAC_DMA2_OPMODE.FUF bit, when set, directs the Rx FIFO to forward Un- dersized frames (frames with no Error and length less than 64 bytes) including pad- bytes and CRC). When reset, the Rx FIFO drops all frames of less than 64 bytes, un- less it is already transferred because of lower value of Receive Threshold (for example, EMAC_DMA2_OPMODE.RTC =01).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 5 (R/W)            | DGF        | Drop Giant Frames. The EMAC_DMA2_OPMODE.DGF bit, when set, the MAC drops the received giant frames in the Rx FIFO, that is, frames that are larger than the computed giant frame limit. When reset, the MAC does not drop the giant frames in the Rx FIFO.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 29-108: EMAC\_DMA2\_OPMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:3 (R/W)          | RTC        | Receive Threshold Control. The EMAC_DMA2_OPMODE.RTC bits control the threshold level of the MFL Re- ceive FIFO. Transfer (request) to DMAstarts when the frame size within the MFL Re- ceive FIFO is larger than the threshold. In addition, full frames with a length less than the threshold are transferred automatically. These bits are valid only when the EMAC_DMA2_OPMODE.RSF bit is zero, and are ignored when the EMAC_DMA2_OPMODE.RSF bit is set to 1. The value =11 is not used.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 2 (R/W)            | OSF        | Operate on Second Frame. The EMAC_DMA2_OPMODE.OSF bit, when set, instructs the DMAto process a sec- ond frame of Transmit data even before status for first frame is obtained.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 1 (R/W)            | SR         | Start/Stop Receive. The EMAC_DMA2_OPMODE.SR bit, when set, places the Receive process in the Run- ning state. The DMAattempts to acquire the descriptor from the Receive list and processes incoming frames. Descriptor acquisition is attempted from the current posi- tion in the list, which is the address set by DMAReceive Descriptor List Address or the position retained when the Receive process was previously stopped. If no descriptor is owned by the DMA, reception is suspended, and the EMAC_DMA2_STAT.RU bit is set. The Start Receive command is effective only when reception has stopped. If the com- mand was issued before setting EMAC_DMA2_RXDSC_CUR address register,DMA behavior is unpredictable. When this bit is cleared, Rx DMAoperation is stopped after the transfer of the current frame. The next descriptor position in the Receive list is saved and becomes the current position after the Receive process is restarted. The Stop Receive command is effective only when the Receive process is in either the Running (waiting for receive packet) or in the Suspended state. |

## DMA Rx Buffer Current Register

The EMAC\_DMA2\_RXBUF\_CUR register holds the pointer to the current receive DMA buffer.

Figure 29-77: EMAC\_DMA2\_RXBUF\_CUR Register Diagram

<!-- image -->

Table 29-109: EMAC\_DMA2\_RXBUF\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Receive Current Buffer Address. The EMAC_DMA2_RXBUF_CUR.ADDR bit field points to the current Receive Buffer address being read by the DMA. Pointer updated by DMAduring operation. Cleared on Reset. |

## DMA Rx Descriptor List Address Register

The EMAC\_DMA2\_RXDSC\_ADDR register holds the address for the DMA receive descriptor list. Writing to this Register is permitted only when reception is stopped. When stopped, this must be written to before the receive Start command is given. The processor can write to EMAC\_DMA2\_RXDSC\_ADDR only when Rx DMA has stopped ( EMAC\_DMA2\_OPMODE.SR bit =0). When stopped, it can be written with a new descriptor list address. When the processor sets the EMAC\_DMA2\_OPMODE.SR bit to 1, the DMA takes the newly programmed descriptor base address. If this register is not changed when the EMAC\_DMA2\_OPMODE.SR bit is cleared to 0, the DMA takes the descriptor address where it was stopped earlier.

Figure 29-78: EMAC\_DMA2\_RXDSC\_ADDR Register Diagram

<!-- image -->

Table 29-110: EMAC\_DMA2\_RXDSC\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Start of Receive List. The EMAC_DMA2_RXDSC_ADDR.VALUE bit field contains the base address of the First Descriptor in the Receive Descriptor list. The LSB bits [1:0] for the 32bit bus width are ignored and are taken as all-zero by the DMAinternally. Therefore, these LSB bits are Read-Only (RO). |

## DMA Rx Descriptor Current Register

The EMAC\_DMA2\_RXDSC\_CUR register contains the current DMA receive descriptor.

Figure 29-79: EMAC\_DMA2\_RXDSC\_CUR Register Diagram

<!-- image -->

Table 29-111: EMAC\_DMA2\_RXDSC\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Receive Current Descriptor Address. The EMAC_DMA2_RXDSC_CUR.ADDR bit field points to the start address of the current Receive Descriptor read by the DMA. Pointer updated by DMAduring opera- tion. Cleared on Reset. |

## DMA Rx Interrupt Watch Dog Register

The EMAC\_DMA2\_RXIWDOG register contains the timeout value for the EMAC DMA receive interrupt watch dog timer.

Figure 29-80: EMAC\_DMA2\_RXIWDOG Register Diagram

<!-- image -->

Table 29-112: EMAC\_DMA2\_RXIWDOG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | RIWT       | RI WatchDog Timer Count. The EMAC_DMA2_RXIWDOG.RIWT bit field indicates the number of system clock cycles multiplied by 256 for which the watchdog timer is set. The watchdog timer gets triggered with the programmed value after the Rx DMAcompletes the transfer of a frame for which the RI status bit is not set because of the setting in the corresponding descriptor RDES1[31]. When the watch-dog timer runs out, the RI bit is set and the timer is stopped. The watchdog timer is reset when EMAC_DMA2_STAT.RI bit is set high because of automatic setting of EMAC_DMA2_STAT.RI as per RDES1[31] of any received frame. |

## DMA Rx Poll Demand register

The EMAC\_DMA2\_RXPOLL register directs the EMAC to poll the receive descriptor list.

Figure 29-81: EMAC\_DMA2\_RXPOLL Register Diagram

<!-- image -->

Table 29-113: EMAC\_DMA2\_RXPOLL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | START      | Receive Poll Demand. The EMAC_DMA2_RXPOLL.START bits, when written with any value, cause the DMAto read the current descriptor pointed to by the EMAC_DMA2_RXDSC_CUR register. If that descriptor is not available (owned by application), reception returns to the Suspended state, and the EMAC_DMA2_STAT.RU bit is asserted. If the descriptor is available, the Receive DMAreturns to the active state. |

## DMA Status Register

The EMAC\_DMA2\_STAT register indicates EMAC DMA status.

Figure 29-82: EMAC\_DMA2\_STAT Register Diagram

<!-- image -->

Table 29-114: EMAC\_DMA2\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/NW)          | GTMSI      | MAC TMS Interrupt. The EMAC_DMA2_STAT.GTMSI bit indicates an interrupt event in the traffic man- ager and scheduler logic. To reset this bit, the software must read the corresponding registers (Channel Status Register) to get the exact cause of the interrupt and clear its source. |

Table 29-114: EMAC\_DMA2\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/NW)          | TTI        | Time Stamp Trigger Interrupt. The EMAC_DMA2_STAT.TTI bit indicates an interrupt event in the MAC core's Time Stamp Generator block. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear its source to reset this bit to =0. When this bit is high, the interrupt signal from the MAC is high. | Time Stamp Trigger Interrupt. The EMAC_DMA2_STAT.TTI bit indicates an interrupt event in the MAC core's Time Stamp Generator block. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear its source to reset this bit to =0. When this bit is high, the interrupt signal from the MAC is high. |
| 27 (R/NW)          | MCI        | MAC MMCInterrupt. The EMAC_DMA2_STAT.MCI bit reflects an interrupt event in the MMCmodule of the MAC core. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear the source of interrupt to make this bit as =0. The interrupt signal from the MAC is high when this bit is high.               | MAC MMCInterrupt. The EMAC_DMA2_STAT.MCI bit reflects an interrupt event in the MMCmodule of the MAC core. The software must read the corresponding registers in the MAC core to get the exact cause of interrupt and clear the source of interrupt to make this bit as =0. The interrupt signal from the MAC is high when this bit is high.               |
| 26 (R/NW)          | GLI        | Line Interface Interrupt. The EMAC_DMA2_STAT.GLI bit When set, this bit reflects any of the following in- terrupt events in the DWC_gmac interfaces                                                                                                                                                                                                        | Line Interface Interrupt. The EMAC_DMA2_STAT.GLI bit When set, this bit reflects any of the following in- terrupt events in the DWC_gmac interfaces                                                                                                                                                                                                        |
| 25:23 (R/NW)       | EB         | Error Bits. The EMAC_DMA2_STAT.EB bits indicate the type of error that caused a Bus Error (for example, error response on the SCB interface). These bits are valid only when the EMAC_DMA2_STAT.FBI bit is set. This field does not generate an interrupt.                                                                                                 | Error Bits. The EMAC_DMA2_STAT.EB bits indicate the type of error that caused a Bus Error (for example, error response on the SCB interface). These bits are valid only when the EMAC_DMA2_STAT.FBI bit is set. This field does not generate an interrupt.                                                                                                 |
| 25:23 (R/NW)       | EB         | 0                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, write transfer, RxDMA                                                                                                                                                                                                                                                                                                     |
| 25:23 (R/NW)       | EB         | 1                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, write transfer, TxDMA                                                                                                                                                                                                                                                                                                     |
| 25:23 (R/NW)       | EB         | 2                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, read transfer, RxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 3                                                                                                                                                                                                                                                                                                                                                          | Error during data buffer access, read transfer, TxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 4                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, write transfer, RxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 5                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, write transfer, TxDMA                                                                                                                                                                                                                                                                                                      |
| 25:23 (R/NW)       | EB         | 6                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, read transfer, RxDMA                                                                                                                                                                                                                                                                                                       |
| 25:23 (R/NW)       | EB         | 7                                                                                                                                                                                                                                                                                                                                                          | Error during descriptor access, read transfer, TxDMA                                                                                                                                                                                                                                                                                                       |
| 22:20 (R/NW)       | TS         | Tx Process State. The EMAC_DMA2_STAT.TS bits indicate the transmit DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                    | Tx Process State. The EMAC_DMA2_STAT.TS bits indicate the transmit DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                    |
| 22:20 (R/NW)       | TS         | 0                                                                                                                                                                                                                                                                                                                                                          | Stopped; Reset or Stop Tx Command Issued                                                                                                                                                                                                                                                                                                                   |
| 22:20 (R/NW)       | TS         | 1                                                                                                                                                                                                                                                                                                                                                          | Running; Fetching Tx Transfer Descriptor                                                                                                                                                                                                                                                                                                                   |
| 22:20 (R/NW)       | TS         | 2                                                                                                                                                                                                                                                                                                                                                          | Running; Waiting for Status                                                                                                                                                                                                                                                                                                                                |
| 22:20 (R/NW)       | TS         | 3                                                                                                                                                                                                                                                                                                                                                          | Reading Data from Host Memory Buffer and Queuing It to Tx Buffer                                                                                                                                                                                                                                                                                           |
| 22:20 (R/NW)       | TS         | 4                                                                                                                                                                                                                                                                                                                                                          | TIME_STAMP Write State                                                                                                                                                                                                                                                                                                                                     |

Table 29-114: EMAC\_DMA2\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Suspended; Tx Descriptor Unavailable or Tx Buffer Un- derflow                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Closing Tx Descriptor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 19:17 (R/NW)       | RS         | Rx Process State. The EMAC_DMA2_STAT.RS bits indicate the receive DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                                                                                                                                                                                                                     | Rx Process State. The EMAC_DMA2_STAT.RS bits indicate the receive DMAstate. This field does not generate an interrupt.                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 19:17 (R/NW)       |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Stopped: Reset or Stop Rx Command Issued.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 19:17 (R/NW)       |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Running: Fetching Rx Transfer Descriptor.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 19:17 (R/NW)       |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 19:17 (R/NW)       |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Running: Waiting for Rx Packet                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 19:17 (R/NW)       |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Suspended: Rx Descriptor Unavailable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 19:17 (R/NW)       |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Running: Closing Rx Descriptor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 19:17 (R/NW)       |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | TIME_STAMP Write State                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 19:17 (R/NW)       |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Running: Transferring Rx Packet Data from Rx Buffer to Host Memory                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 16 (R/W1C)         | NIS        | Normal Interrupt Summary. The value of the EMAC_DMA2_STAT.NIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA2_STAT.TI , EMAC_DMA2_STAT.TU ,                                                                                                                                                                                                                                                                                                                  | Normal Interrupt Summary. The value of the EMAC_DMA2_STAT.NIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA2_STAT.TI , EMAC_DMA2_STAT.TU ,                                                                                                                                                                                                                                                                                                                  |
| 15 (R/W1C)         | AIS        | EMAC_DMA2_STAT.NIS Abnormal Interrupt Summary. The value of the EMAC_DMA2_STAT.AIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA2_IEN.TSE , EMAC_DMA2_IEN.TJE , EMAC_DMA2_IEN.OVE , EMAC_DMA2_IEN.UNE , EMAC_DMA2_IEN.RUE , EMAC_DMA2_IEN.RSE , EMAC_DMA2_IEN.RWE , EMAC_DMA2_IEN.ETE , and EMAC_DMA2_IEN.FBE . Only unmasked bits affect the Abnormal Interrupt Sum- mary bit. This is a sticky bit and must be cleared each time a corresponding bit that | EMAC_DMA2_STAT.NIS Abnormal Interrupt Summary. The value of the EMAC_DMA2_STAT.AIS bit field is the logical OR of the follow- ing when the corresponding interrupt bits are enabled in DMAInterrupt Enable Reg- ister: EMAC_DMA2_IEN.TSE , EMAC_DMA2_IEN.TJE , EMAC_DMA2_IEN.OVE , EMAC_DMA2_IEN.UNE , EMAC_DMA2_IEN.RUE , EMAC_DMA2_IEN.RSE , EMAC_DMA2_IEN.RWE , EMAC_DMA2_IEN.ETE , and EMAC_DMA2_IEN.FBE . Only unmasked bits affect the Abnormal Interrupt Sum- mary bit. This is a sticky bit and must be cleared each time a corresponding bit that |
| 14 (R/W1C)         | ERI        | Early Receive Interrupt. The EMAC_DMA2_STAT.ERI bit indicates that the DMAhad filled the first data                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Early Receive Interrupt. The EMAC_DMA2_STAT.ERI bit indicates that the DMAhad filled the first data                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 29-114: EMAC\_DMA2\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1C)         | FBI        | Fatal Bus Error Interrupt. The EMAC_DMA2_STAT.FBI bit indicates that a bus error occurred, as detailed in the EMAC_DMA2_STAT.EB field. When this bit is set, the corresponding DMAen- gine disables all its bus accesses.                                                                                                                                                                                                                                                                                                                                        |
| 10 (R/W1C)         | ETI        | Early Transmit Interrupt. The EMAC_DMA2_STAT.ETI bit indicates that the frame to be transmitted was fully transferred to the MFL Transmit FIFO.                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 9 (R/W1C)          | RWT        | Receive WatchDog Timeout. The EMAC_DMA2_STAT.RWT bit is asserted when a frame with a length greater than 2,048 bytes is received (10, 240 when Jumbo Frame mode is enabled).                                                                                                                                                                                                                                                                                                                                                                                     |
| 8 (R/W1C)          | RPS        | Receive Process Stopped. The EMAC_DMA2_STAT.RPS bit is asserted when the Receive Process enters the Stopped state.                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 7 (R/W1C)          | RU         | Receive Buffer Unavailable. The EMAC_DMA2_STAT.RU bit indicates that the Next Descriptor in the Receive List is owned by the application and cannot be acquired by the DMA. Receive Process is suspended. To resume processing Receive descriptors, the application should change the ownership of the descriptor and issue a Receive Poll Demand command. If no Re- ceive Poll Demand is issued, Receive Process resumes when the next recognized incom- ing frame is received. This bit is set only when the previous Receive Descriptor was owned by the DMA. |
| 6 (R/W1C)          | RI         | Receive Interrupt. The EMAC_DMA2_STAT.RI bit indicates the completion of frame reception. Specif- ic frame status information has been posted in the descriptor. Reception remains in the Running state.                                                                                                                                                                                                                                                                                                                                                         |
| 5 (R/W1C)          | UNF        | Transmit Buffer Underflow. The EMAC_DMA2_STAT.UNF bit indicates that the Transmit Buffer had an Under- flow during frame transmission. Transmission is suspended and an Underflow Error TDES0[1] is set.                                                                                                                                                                                                                                                                                                                                                         |
| 4 (R/W1C)          | OVF        | Receive Buffer Overflow. The EMAC_DMA2_STAT.OVF bit indicates that the Receive Buffer had an Overflow during frame reception. If the partial frame is transferred to application, the overflow status is set in RDES0[11].                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W1C)          | TJT        | Transmit Jabber Timeout. The EMAC_DMA2_STAT.TJT bit indicates that the Transmit Jabber Timer expired, meaning that the transmitter had been excessively active. The transmission process is aborted and placed in the Stopped state. This causes the Transmit Jabber Timeout TDES0[14] flag to assert.                                                                                                                                                                                                                                                           |

Table 29-114: EMAC\_DMA2\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | TU         | Transmit Buffer Unavailable. The EMAC_DMA2_STAT.TU bit indicates that the Next Descriptor in the Transmit List is owned by the application and cannot be acquired by the DMA. Transmission is suspended. The value in the EMAC_DMA2_STAT.TS bits explain the Transmit Proc- ess state transitions. To resume processing transmit descriptors, the application should change the ownership of the bit of the descriptor and then issue a Transmit Poll De- mand command. |
| 1 (R/W1C)          | TPS        | Transmit Process Stopped. The EMAC_DMA2_STAT.TPS bit is set when the transmission is stopped.                                                                                                                                                                                                                                                                                                                                                                           |
| 0 (R/W1C)          | TI         | Transmit Interrupt. The EMAC_DMA2_STAT.TI bit indicates that frame transmission is finished and TDES1[31] is set in the First Descriptor.                                                                                                                                                                                                                                                                                                                               |

## DMA Tx Buffer Current Register

The EMAC\_DMA2\_TXBUF\_CUR register holds the pointer to the current transmit DMA buffer.

Figure 29-83: EMAC\_DMA2\_TXBUF\_CUR Register Diagram

<!-- image -->

Table 29-115: EMAC\_DMA2\_TXBUF\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Transmit Current Buffer Address. The EMAC_DMA2_TXBUF_CUR.ADDR bit field points to the current Transmit Buf- fer Address being read by the DMA. Pointer updated by DMAduring operation. Cleared on Reset. |

## DMA Tx Descriptor List Address Register

The EMAC\_DMA2\_TXDSC\_ADDR register holds the address for the DMA transmit descriptor list. The processor can write to this Register only when Tx DMA has stopped ( EMAC\_DMA2\_OPMODE.ST bit =0). When stopped, this can be written with a new descriptor list address. When the processor sets the EMAC\_DMA2\_OPMODE.ST bit to 1, the DMA takes the newly programmed descriptor base address. If this register is not changed when the EMAC\_DMA2\_OPMODE.ST bit is cleared to 0, then the DMA takes the descriptor address where it was stopped earlier.

Figure 29-84: EMAC\_DMA2\_TXDSC\_ADDR Register Diagram

<!-- image -->

Table 29-116: EMAC\_DMA2\_TXDSC\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Start of Transmit List. The EMAC_DMA2_TXDSC_ADDR.VALUE bit field contains the base address of the First Descriptor in the Transmit Descriptor list. The LSB bits [1:0] for 32bit bus width are ignored and are taken as all-zero by the DMAinternally. Therefore, these LSB bits are Read-Only (RO). |

## DMA Tx Descriptor Current Register

The EMAC\_DMA2\_TXDSC\_CUR register contains the current DMA transmit descriptor.

Figure 29-85: EMAC\_DMA2\_TXDSC\_CUR Register Diagram

<!-- image -->

Table 29-117: EMAC\_DMA2\_TXDSC\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | ADDR       | Host Transmit Descriptor Address. The EMAC_DMA2_TXDSC_CUR.ADDR bit field points to the start address of the current Transmit Descriptor read by the DMA. Pointer updated by DMAduring oper- ation. Cleared on Reset. |

## DMA Tx Poll Demand Register

The EMAC\_DMA2\_TXPOLL register directs the EMAC to poll the transmit descriptor list.

Figure 29-86: EMAC\_DMA2\_TXPOLL Register Diagram

<!-- image -->

Table 29-118: EMAC\_DMA2\_TXPOLL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | START      | Transmit Poll Demand. The EMAC_DMA2_TXPOLL.START bits, when written with any value, cause the DMAto read the current descriptor pointed to by EMAC_DMA2_TXDSC_CUR regis- ter. If that descriptor is not available (owned by application), transmission returns to the Suspend state, and the EMAC_DMA2_STAT.TU bit is asserted. If the descriptor is available, transmission resumes. |

## FLow Control Register

The EMAC\_FLOWCTL register controls EMAC flow control features.

Figure 29-87: EMAC\_FLOWCTL Register Diagram

<!-- image -->

Table 29-119: EMAC\_FLOWCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | PT         | Pause Time. The EMAC_FLOWCTL.PT bits hold the value to be used in the Pause Time field in the transmit control frame.                                                                                                                                                                                                                                                                                                                       |
| 7 (R/W)            | DZPQ       | Disable Zero-Quanta Pause. The EMAC_FLOWCTL.DZPQ bit disables the automatic generation of the Zero- Quanta Pause frames on the de-assertion of the flow-control signal from the FIFO lay- er                                                                                                                                                                                                                                                |
| 5:4 (R/W)          | PLT        | Pause Low Threshold. The EMAC_FLOWCTL.PLT bit configures the threshold of the Pause timer at which the input flow control signal mti_flowctrl_i (or sbd_flowctrl_i) is checked for automat- ic retransmission of the Pause frame.                                                                                                                                                                                                           |
| 3 (R/W)            | UP         | Unicast Pause Frame Detect. The EMAC_FLOWCTL.UP bit, when set, directs the MAC to detect the Pause frames with the station's unicast address specified in EMAC_ADDR0_HI and EMAC_ADDR0_LO address registers. This bit also directs the MAC to the detect Pause frames with the unique multicast address. When this bit is reset, the MAC will detect only a Pause frame with the unique multicast address specified in the 802.3x standard. |

Table 29-119: EMAC\_FLOWCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | RFE        | Receive Flow Control Enable. The EMAC_FLOWCTL.RFE bit, when set, directs the MAC to decode the received Pause frame and disable its transmitter for a specified (Pause Time) time. When this bit is reset, the decode function of the Pause frame is disabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 1 (R/W)            | TFE        | Transmit Flow Control Enable. In Full-Duplex mode, when the EMAC_FLOWCTL.TFE bit is set, the MAC enables the flow control operation to transmit Pause frames. When this bit is reset, the flow control operation in the MAC is disabled, and the MAC does not transmit any Pause frames. In Half-Duplex mode, when this bit is set, the MAC enables the back pressure operation. When this bit is reset, the back pressure feature is disabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 0 (R/W1S)          | FCBBPA     | Initiate Pause Control Frame. The EMAC_FLOWCTL.FCBBPA bit initiates a Pause Control frame in Full-Duplex mode and activates the back pressure function in Half-Duplex mode if TFE bit is set. In Full-Duplex mode, this bit should be read as =0 before writing to the EMAC_FLOWCTL register. To initiate a Pause control frame, the Application must set this bit to =1. During a transfer of the Control Frame, this bit continues to be set to signify that a frame transmission is in progress. After the completion of Pause control frame transmission, the MAC resets this bit to =0. The EMAC_FLOWCTL register should not be written to until this bit is cleared. In Half-Duplex mode, when this bit is set (and EMAC_FLOWCTL.TFE is set), the back pressure is asserted by the MAC Core. During back pressure, when the MAC receives a new frame, the transmitter starts sending a JAM pattern resulting in a collision. The EMAC_FLOWCTL.FCBBPA bit is logically OR'ed with the flow control input signal for the back pressure function. When the MAC is configured to Full-Duplex mode, the back pressure function is au- tomatically disabled. |

## RGMII Control and Status Register

The EMAC\_GIGE\_CTLSTAT register indicates the status signals received from the PHY through the SGMII, RGMII, or SMII interface.

Figure 29-88: EMAC\_GIGE\_CTLSTAT Register Diagram

<!-- image -->

Table 29-120: EMAC\_GIGE\_CTLSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/NW)           | LNKSTS     | Link Status. The EMAC_GIGE_CTLSTAT.LNKSTS bit indicates that the link is up between the local PHY and the remote PHY. When cleared, this bit indicates that the link is down between the local PHY and the remote PHY. |
| 2:1 (R/NW)         | LNKSPEED   | Link Speed. The EMAC_GIGE_CTLSTAT.LNKSPEED bit indicates the current speed of the link.                                                                                                                                |
| 0 (R/NW)           | LNKMOD     | Link Mode. The EMAC_GIGE_CTLSTAT.LNKMOD bit indicates whether the current mode of link operation is half duplex or full duplex.                                                                                        |

## Hash Table High Register

The EMAC\_HASHTBL\_HI register contains the upper 32 bits of the hash table.

Figure 29-89: EMAC\_HASHTBL\_HI Register Diagram

<!-- image -->

Table 29-121: EMAC\_HASHTBL\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 31:0               | VALUE      | Hash Table High.                                                        |
| (R/W)              |            | The EMAC_HASHTBL_HI.VALUE bits contain the upper 32 bits of Hash table. |

## Hash Table Low Register

The EMAC\_HASHTBL\_LO register contains the lower 32 bits of the hash table.

Figure 29-90: EMAC\_HASHTBL\_LO Register Diagram

<!-- image -->

Table 29-122: EMAC\_HASHTBL\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 31:0               | VALUE      | Hash Table Low.                                                         |
| (R/W)              |            | The EMAC_HASHTBL_LO.VALUE bits contain the lower 32 bits of Hash table. |

## Interrupt Mask Register

The EMAC\_IMSK register enables (unmasks) EMAC interrupts.

Figure 29-91: EMAC\_IMSK Register Diagram

<!-- image -->

Table 29-123: EMAC\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | LPIIM      | LPI Interrupt Mask. The EMAC_IMSK.LPIIM bit, When set, disables the assertion of the interrupt signal because of the setting of the LPI Interrupt Status bit in Interrupt Status Register.                 |
| 9 (R/W)            | TS         | Time Stamp Interrupt Mask. The EMAC_IMSK.TS bit, when set, disables the assertion of the interrupt signal, which is generated when the EMAC_ISTAT.TS bit is set.                                           |
| 0 (R/W)            | RGMIIIM    | RGMII or SMII Interrupt Mask. The EMAC_IMSK.RGMIIIM bit, When set, disables the assertion of the interrupt signal because of the setting of the RGMII Interrupt Status bit in Interrupt Status Reg- ister. |

## MMC IPC Rx Interrupt Mask Register

The EMAC\_IPC\_RXIMSK register enables (unmasks) MMC IPC receive interrupts.

Figure 29-92: EMAC\_IPC\_RXIMSK Register Diagram

<!-- image -->

Table 29-124: EMAC\_IPC\_RXIMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | ICMPERROCT | Rx ICMP Error Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.ICMPERROCT bit, when set, masks the interrupt when the EMAC_RXICMP_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value.            |
| 28 (R/W)           | ICMPGOCT   | Rx ICMP (Good) Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.ICMPGOCT bit, when set, masks the interrupt when the EMAC_RXICMP_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.              |
| 27 (R/W)           | TCPERROCT  | Rx TCP Error Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.TCPERROCT bit, when set, masks the interrupt when the EMAC_RXTCP_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value.               |
| 26 (R/W)           | TCPGOCT    | Rx TCP (Good) Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.TCPGOCT bit, when set, masks the interrupt when the EMAC_RXTCP_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.                 |
| 25 (R/W)           | UDPERROCT  | Rx UDP Error Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.UDPERROCT bit, when set, masks the interrupt when the EMAC_RXUDP_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value.               |
| 24 (R/W)           | UDPGOCT    | Rx UDP (Good) Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.UDPGOCT bit, when set, masks the interrupt when the EMAC_RXUDP_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.                 |
| 23 (R/W)           | V6NOPAYOCT | Rx IPv6 No Payload Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.V6NOPAYOCT bit, when set, masks the interrupt when the EMAC_RXIPV6_NOPAY_OCT counter reaches half the maximum value, and also when it reaches the maximum value.     |
| 22 (R/W)           | V6HDERROCT | Rx IPv6 Header Error Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.V6HDERROCT bit, when set, masks the interrupt when the EMAC_RXIPV6_HDR_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value. |
| 21 (R/W)           | V6GOCT     | Rx IPv6 (Good) Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.V6GOCT bit, when set, masks the interrupt when the EMAC_RXIPV6_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.                |

Table 29-124: EMAC\_IPC\_RXIMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | V4UDSBLOCT | Rx IPv4 UDS Disable Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4UDSBLOCT bit, when set, masks the interrupt when the EMAC_RXIPV4_UDSBL_OCT counter reaches half the maximum value, and also when it reaches the maximum value.    |
| 19 (R/W)           | V4FRAGOCT  | Rx IPv4 Fragmented Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4FRAGOCT bit, when set, masks the interrupt when the EMAC_RXIPV4_FRAG_OCT counter reaches half the maximum value, and also when it reaches the maximum value.       |
| 18 (R/W)           | V4NOPAYOCT | Rx IPv4 No Payload Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4NOPAYOCT bit, when set, masks the interrupt when the EMAC_RXIPV4_NOPAY_OCT counter reaches half the maximum value, and also when it reaches the maximum value.     |
| 17 (R/W)           | V4HDERROCT | Rx IPv4 Header Error Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4HDERROCT bit, when set, masks the interrupt when the EMAC_RXIPV4_HDR_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value. |
| 16 (R/W)           | V4GOCT     | Rx IPv4 (Good) Octets Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4GOCT bit, when set, masks the interrupt when the EMAC_RXIPV4_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.                |
| 13 (R/W)           | ICMPERRFRM | Rx ICMP Error Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.ICMPERRFRM bit, when set, masks the interrupt when the EMAC_RXICMP_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value.            |
| 12 (R/W)           | ICMPGFRM   | Rx ICMP (Good) Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.ICMPGFRM bit, when set, masks the interrupt when the EMAC_RXICMP_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.              |
| 11 (R/W)           | TCPERRFRM  | Rx TCP Error Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.TCPERRFRM bit, when set, masks the interrupt when the EMAC_RXTCP_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value.               |
| 10 (R/W)           | TCPGFRM    | Rx TCP (Good) Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.TCPGFRM bit, when set, masks the interrupt when the EMAC_RXTCP_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.                 |

Table 29-124: EMAC\_IPC\_RXIMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | UDPERRFRM  | Rx UDP Error Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.UDPERRFRM bit, when set, masks the interrupt when the EMAC_RXUDP_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value.               |
| 8 (R/W)            | UDPGFRM    | Rx UDP (Good) Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.UDPGFRM bit, when set, masks the interrupt when the EMAC_RXUDP_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.                 |
| 7 (R/W)            | V6NOPAYFRM | Rx IPv6 No Payload Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.V6NOPAYFRM bit, when set, masks the interrupt when the EMAC_RXIPV6_NOPAY_FRM counter reaches half the maximum value, and also when it reaches the maximum value.     |
| 6 (R/W)            | V6HDERRFRM | Rx IPv6 Header Error Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.V6HDERRFRM bit, when set, masks the interrupt when the EMAC_RXIPV6_HDR_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value. |
| 5 (R/W)            | V6GFRM     | Rx IPv6 (Good) Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.V6GFRM bit, when set, masks the interrupt when the EMAC_RXIPV6_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.                |
| 4 (R/W)            | V4UDSBLFRM | Rx IPv4 UDS Disable Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4UDSBLFRM bit, when set, masks the interrupt when the EMAC_RXIPV4_UDSBL_FRM counter reaches half the maximum value, and also when it reaches the maximum value.    |
| 3 (R/W)            | V4FRAGFRM  | Rx IPv4 Fragmented Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4FRAGFRM bit, when set, masks the interrupt when the EMAC_RXIPV4_FRAG_FRM counter reaches half the maximum value, and also when it reaches the maximum value.       |
| 2 (R/W)            | V4NOPAYFRM | Rx IPv4 No Payload Frame Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4NOPAYFRM bit, when set, masks the interrupt when the EMAC_RXIPV4_NOPAY_FRM counter reaches half the maximum value, and also when it reaches the maximum value.      |
| 1 (R/W)            | V4HDERRFRM | Rx IPv4 Header Error Frame Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4HDERRFRM bit, when set, masks the interrupt when the EMAC_RXIPV4_HDR_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value.  |

Table 29-124: EMAC\_IPC\_RXIMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | V4GFRM     | Rx IPv4 (Good) Frames Count Half/Full Mask. The EMAC_IPC_RXIMSK.V4GFRM bit, when set, masks the interrupt when the EMAC_RXIPV4_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value. |

## MMC IPC Rx Interrupt Register

The EMAC\_IPC\_RXINT register indicates status of MMC IPC receive interrupts.

Figure 29-93: EMAC\_IPC\_RXINT Register Diagram

<!-- image -->

Table 29-125: EMAC\_IPC\_RXINT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/NW)          | ICMPERROCT | Rx ICMP Error Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.ICMPERROCT bit is set when the EMAC_RXICMP_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value.            |
| 28 (R/NW)          | ICMPGOCT   | Rx ICMP (Good) Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.ICMPGOCT bit is set when the EMAC_RXICMP_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.              |
| 27 (R/NW)          | TCPERROCT  | Rx TCP Error Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.TCPERROCT bit is set when the EMAC_RXTCP_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value.               |
| 26 (R/NW)          | TCPGOCT    | Rx TCP (Good) Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.TCPGOCT bit is set when the EMAC_RXTCP_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.                 |
| 25 (R/NW)          | UDPERROCT  | Rx UDP Error Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.UDPERROCT bit is set when the EMAC_RXUDP_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value.               |
| 24 (R/NW)          | UDPGOCT    | Rx UDP (Good) Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.UDPGOCT bit is set when the EMAC_RXUDP_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.                 |
| 23 (R/NW)          | V6NOPAYOCT | Rx IPv6 No Payload Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.V6NOPAYOCT bit is set when the EMAC_RXIPV6_NOPAY_OCT counter reaches half the maximum value, and also when it reaches the maximum value.     |
| 22 (R/NW)          | V6HDERROCT | Rx IPv6 Header Error Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.V6HDERROCT bit is set when the EMAC_RXIPV6_HDR_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value. |
| 21 (R/NW)          | V6GOCT     | Rx IPv6 (Good) Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.V6GOCT bit is set when the EMAC_RXIPV6_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.                |
| 20 (R/NW)          | V4UDSBLOCT | Rx IPv4 UDS Disable Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4UDSBLOCT bit is set when the EMAC_RXIPV4_UDSBL_OCT counter reaches half the maximum value, and also when it reaches the maximum value.    |

Table 29-125: EMAC\_IPC\_RXINT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/NW)          | V4FRAGOCT  | Rx IPv4 Fragmented Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4FRAGOCT bit is set when the EMAC_RXIPV4_FRAG_OCT counter reaches half the maximum value, and also when it reaches the maximum value.       |
| 18 (R/NW)          | V4NOPAYOCT | Rx IPv4 No Payload Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4NOPAYOCT bit set when the EMAC_RXIPV4_NOPAY_OCT counter reaches half the maximum value, and also when it reaches the maximum value.        |
| 17 (R/NW)          | V4HDERROCT | Rx IPv4 Header Error Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4HDERROCT bit is set when the EMAC_RXIPV4_HDR_ERR_OCT counter reaches half the maximum value, and also when it reaches the maximum value. |
| 16 (R/NW)          | V4GOCT     | Rx IPv4 (Good) Octets Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4GOCT bit is set when the EMAC_RXIPV4_GD_OCT counter reaches half the maximum value, and also when it reaches the maximum value.                |
| 13 (R/NW)          | ICMPERRFRM | Rx ICMP Error Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.ICMPERRFRM bit is set when the EMAC_RXICMP_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value.            |
| 12 (R/NW)          | ICMPGFRM   | Rx ICMP (Good) Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.ICMPGFRM bit is set when the EMAC_RXICMP_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.              |
| 11 (R/NW)          | TCPERRFRM  | Rx TCP Error Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.TCPERRFRM bit is set when the EMAC_RXTCP_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value.               |
| 10 (R/NW)          | TCPGFRM    | Rx TCP (Good) Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.TCPGFRM bit is set when the EMAC_RXTCP_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.                 |
| 9 (R/NW)           | UDPERRFRM  | Rx IDP Error Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.UDPERRFRM bit is set when the EMAC_RXUDP_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value.               |
| 8 (R/NW)           | UDPGFRM    | Rx UDP (Good) Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.UDPGFRM bit is set when the EMAC_RXUDP_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.                 |

Table 29-125: EMAC\_IPC\_RXINT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | V6NOPAYFRM | Rx IPv6 No Payload Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.V6NOPAYFRM bit is set when the EMAC_RXIPV6_NOPAY_FRM counter reaches half the maximum value, and also when it reaches the maximum value.     |
| 6 (R/NW)           | V6HDERRFRM | Rx IPv6 Header Error Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.V6HDERRFRM bit is set when the EMAC_RXIPV6_HDR_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value. |
| 5 (R/NW)           | V6GFRM     | Rx IPv6 (Good) Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.V6GFRM bit is set when the EMAC_RXIPV6_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.                |
| 4 (R/NW)           | V4UDSBLFRM | Rx IPv4 UDS Disable Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4UDSBLFRM bit is set when the EMAC_RXIPV4_UDSBL_FRM counter reaches half the maximum value, and also when it reaches the maximum value.    |
| 3 (R/NW)           | V4FRAGFRM  | Rx IPv4 Fragmented Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4FRAGFRM bit is set when the EMAC_RXIPV4_FRAG_FRM counter reaches half the maximum value, and also when it reaches the maximum value.       |
| 2 (R/NW)           | V4NOPAYFRM | Rx IPv4 No Payload Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4NOPAYFRM bit is set when the EMAC_RXIPV4_NOPAY_FRM counter reaches half the maximum value, and also when it reaches the maximum value.     |
| 1 (R/NW)           | V4HDERRFRM | Rx IPv4 Header Error Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4HDERRFRM bit is set when the EMAC_RXIPV4_HDR_ERR_FRM counter reaches half the maximum value, and also when it reaches the maximum value. |
| 0 (R/NW)           | V4GFRM     | Rx IPv4 (Good) Frames Count Half/Full Interrupt. The EMAC_IPC_RXINT.V4GFRM bit is set when the EMAC_RXIPV4_GD_FRM counter reaches half the maximum value, and also when it reaches the maximum value.                |

## Interrupt Status Register

The EMAC\_ISTAT register indicates EMAC interrupt status.

Figure 29-94: EMAC\_ISTAT Register Diagram

<!-- image -->

<!-- image -->

Table 29-126: EMAC\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/NW)          | LPIIS      | LPI Interrupt Status. The EMAC_ISTAT.LPIIS bit is set for any LPI state entry or exit in the MAC Transmitter or Receiver.                                                                                                                                 |
| 9 (R/NW)           | TS         | Time Stamp Interrupt Status. The EMAC_ISTAT.TS bit is set when: There is an overflow in the EMAC_TM_SEC register, or The EMAC_TM_STMPSTAT.ATSTS bit is asserted. The EMAC_ISTAT.TS bit is cleared on reading the byte 0 of the EMAC_TM_STMPSTAT register. |
| 7 (R/NW)           | MMCRC      | MMCReceive Checksum Offload Interrupt Status. The EMAC_ISTAT.MMCRC bit is set high whenever an interrupt is generated in the EMAC_IPC_RXINT . This bit is cleared when all the bits in this interrupt register are cleared.                               |
| 6 (R/NW)           | MMCTX      | MMCTransmit Interrupt Status. The EMAC_ISTAT.MMCTX bit is set high whenever an interrupt is generated in the EMAC_MMC_TXINT register. This bit is cleared when all the bits in this interrupt reg- ister are cleared.                                     |

Table 29-126: EMAC\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/NW)           | MMCRX      | MMCReceive Interrupt Status. The EMAC_ISTAT.MMCRX bit is set high whenever an interrupt is generated in the EMAC_MMC_RXINT register. This bit is cleared when all the bits in this interrupt reg- ister are cleared. |
| 4 (R/NW)           | MMC        | MMCInterrupt Status. The EMAC_ISTAT.MMC bit is set high whenever any of EMAC_ISTAT bits [7:5] is set (=1) and is cleared only when all of these bits are cleared (=0).                                               |
| 0 (R/NW)           | RGMIIIS    | RGMII or SMII Interrupt Status. The EMAC_ISTAT.RGMIIIS bit is set because of any change in value of the Link Status of RGMII interface.                                                                              |

## Layer3 and Layer4 Control Register

The EMAC\_L3L4\_CTL register controls the operations of the filter 0 of Layer 3 and Layer 4. This register is reserved if the Layer 3 and Layer 4 Filtering feature is not selected during core configuration.

Figure 29-95: EMAC\_L3L4\_CTL Register Diagram

<!-- image -->

Table 29-127: EMAC\_L3L4\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | L4DPIM     | Layer 4 Destination Port Inverse Matching. The EMAC_L3L4_CTL.L4DPIM bit indicates that the Layer 4 Destination Port number field is enabled for inverse matching. |
| 20 (R/W)           | L4DPM      | Layer 4 Destination Port Matching. The EMAC_L3L4_CTL.L4DPM bit indicates that the Layer 4 Destination Port number field is enabled for matching.                  |
| 19 (R/W)           | L4SPIM     | Layer 4 Source Port Inverse Matching. The EMAC_L3L4_CTL.L4SPIM bit indicates that the Layer 4 Source Port number field is enabled for inverse matching.           |
| 18 (R/W)           | L4SPM      | Layer 4 Source Port Matching. The EMAC_L3L4_CTL.L4SPM bit indicates that the Layer 4 Source Port number field is enabled for matching.                            |

Table 29-127: EMAC\_L3L4\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | L4PEN      | Layer 4 Filtering Enable. The EMAC_L3L4_CTL.L4PEN bit indicates that source and destination port num- ber fields for UDP (TCP) frames are used for matching.                 |
| 15:11 (R/W)        | L3HDBM     | Layer 3 Destination Address Bits Mask. The EMAC_L3L4_CTL.L3HDBM bits are the number of lower bits of IP destination Address that are masked for matching in the IPv4 frames. |
| 10:6 (R/W)         | L3HSBM     | Layer 3 Source Address Bits Mask. The EMAC_L3L4_CTL.L3HSBM bit are the number of lower bits of IP source ad- dress that are masked for matching in the IPv4 frames.          |
| 5 (R/W)            | L3DAIM     | Layer 3 Destination Address Inverse Matching. The EMAC_L3L4_CTL.L3DAIM bit indicates that the Layer 3 IP destination ad- dress field is enabled for inverse matching.        |
| 4 (R/W)            | L3DAM      | Layer 3 Destination Address Matching. The EMAC_L3L4_CTL.L3DAM bit indicates that Layer 3 IP destination address field is enabled for matching.                               |
| 3 (R/W)            | L3SAIM     | Layer 3 Source Address Inverse Matching. The EMAC_L3L4_CTL.L3SAIM bit indicates that the Layer 3 IP Source Address field is enabled for inverse matching.                    |
| 2 (R/W)            | L3SAM      | Layer 3 Source Address Matching. The EMAC_L3L4_CTL.L3SAM bit indicates that the Layer 3 IP Source Address field is enabled for matching.                                     |
| 0 (R/W)            | L3PEN      | Layer 3 Enabled. The EMAC_L3L4_CTL.L3PEN bit indicates that layer 3 source or destination ad- dress matching is enabled for IPV6 (IPV4) frames.                              |

## Layer 3 Address0 Register

The EMAC\_L3\_ADDR0 register tells For IPv4 frames, the Layer 3 Address 0 Register 0 contains the 32-bit IP Source Address field.

Figure 29-96: EMAC\_L3\_ADDR0 Register Diagram

<!-- image -->

Table 29-128: EMAC\_L3\_ADDR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | L3A0       | Layer 3 Address 0 Field. When Bit 0 (L3PEN0) and Bit 2 (L3SAM0) are set in Register 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with Bits [31:0] of the IP Source Address field in the IPv6 frames. When Bit 0 (L3PEN0) and Bit 4 (L3DAM0) are set in Register 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with Bits [31:0] of the IP Destination Address field in the IPv6 frames. When Bit 0 (L3PEN0) is reset and Bit 2 (L3SAM0) is set in Regis- ter 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with the IP Source Address field in the IPv4 frames. |

## Layer 3 Address1 Register

The EMAC\_L3\_ADDR1 register tells For IPv4 frames, the Layer 3 Address 1 Register 0 contains the 32-bit IP Source Address field.

Figure 29-97: EMAC\_L3\_ADDR1 Register Diagram

<!-- image -->

Table 29-129: EMAC\_L3\_ADDR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | L3A1       | Layer 3 Address 1 Field. When Bit 0 (L3PEN0) and Bit 2 (L3SAM0) are set in Register 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with Bits [31:0] of the IP Source Address field in the IPv6 frames. When Bit 0 (L3PEN0) and Bit 4 (L3DAM0) are set in Register 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with Bits [31:0] of the IP Destination Address field in the IPv6 frames. When Bit 0 (L3PEN0) is reset and Bit 2 (L3SAM0) is set in Regis- ter 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with the IP Source Address field in the IPv4 frames. |

## Layer 3 Address2 Register

The EMAC\_L3\_ADDR2 register tells For IPv4 frames, the Layer 3 Address 2 Register 0 contains the 32-bit IP Source Address field.

Figure 29-98: EMAC\_L3\_ADDR2 Register Diagram

<!-- image -->

Table 29-130: EMAC\_L3\_ADDR2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | L3A2       | Layer 3 Address 2 Field. When Bit 0 (L3PEN0) and Bit 2 (L3SAM0) are set in Register 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with Bits [31:0] of the IP Source Address field in the IPv6 frames. When Bit 0 (L3PEN0) and Bit 4 (L3DAM0) are set in Register 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with Bits [31:0] of the IP Destination Address field in the IPv6 frames. When Bit 0 (L3PEN0) is reset and Bit 2 (L3SAM0) is set in Regis- ter 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with the IP Source Address field in the IPv4 frames. |

## Layer 3 Address3 Register

The EMAC\_L3\_ADDR3 register tells For IPv4 frames, the Layer 3 Address 3 Register 0 contains the 32-bit IP Source Address field.

Figure 29-99: EMAC\_L3\_ADDR3 Register Diagram

<!-- image -->

Table 29-131: EMAC\_L3\_ADDR3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | L3A3       | Layer 3 Address 3 Field. When Bit 0 (L3PEN0) and Bit 2 (L3SAM0) are set in Register 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with Bits [31:0] of the IP Source Address field in the IPv6 frames. When Bit 0 (L3PEN0) and Bit 4 (L3DAM0) are set in Register 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with Bits [31:0] of the IP Destination Address field in the IPv6 frames. When Bit 0 (L3PEN0) is reset and Bit 2 (L3SAM0) is set in Regis- ter 256 (Layer 3 and Layer 4 Control Register 0), this field contains the value to be matched with the IP Source Address field in the IPv4 frames. |

## Layer 4 Address Register

The EMAC\_L4\_ADDR register contains Layer 4 Port number field. It contains the 16-bit Source and Destination Port numbers of the TCP or UDP frame.

Figure 29-100: EMAC\_L4\_ADDR Register Diagram

<!-- image -->

Table 29-132: EMAC\_L4\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | L4DP       | Layer 4 Destination Port. When EMAC_L3L4_CTL.L4PEN is reset and EMAC_L3L4_CTL.L4DPM is set, the EMAC_L4_ADDR.L4DP bit field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 frames. When EMAC_L3L4_CTL.L4PEN and EMAC_L3L4_CTL.L4DPM are set, the EMAC_L4_ADDR.L4DP bit field contains the value to be matched with the UDP Destination Port Number field in the IPv4 or IPv6 frames. |
| 15:0 (R/W)         | L4SP       | Layer 4 Source Port. When EMAC_L3L4_CTL.L4PEN is reset and EMAC_L3L4_CTL.L4DPM is set, the EMAC_L4_ADDR.L4SP bit field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 frames. When EMAC_L3L4_CTL.L4PEN and EMAC_L3L4_CTL.L4DPM are set, the EMAC_L4_ADDR.L4SP bit field contains the value to be matched with the UDP Source Port Number field in the IPv4 or IPv6 frames.                |

## Low Power Idle Control and Status Register

The EMAC\_LPI\_CTLSTAT register controls the behavior of EMAC0 for LPI mode and reports status.

Figure 29-101: EMAC\_LPI\_CTLSTAT Register Diagram

<!-- image -->

Table 29-133: EMAC\_LPI\_CTLSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | LPITXA     | LPI Mode Transmit Behavior. The EMAC_LPI_CTLSTAT.LPITXA bit controls the behavior of the MAC when it is entering or exiting of the LPI mode on the transmit side.                               |
| 18 (R/W)           | PLSEN      | LPI Link Status Enable. The EMAC_LPI_CTLSTAT.PLSEN bit enables the link status received on the RGMII receive paths to be used for activating the LPI LS timer.                                  |
| 17 (R/W)           | PLS        | PHY Link Status. The EMAC_LPI_CTLSTAT.PLS bit indicates the link status of the PHY                                                                                                              |
| 16 (R/W0C)         | LPIEN      | LPI Enable. The EMAC_LPI_CTLSTAT.LPIEN bit instructs the MAC Transmitter to enter the LPI state. When reset, this bit instructs the MAC to exit the LPI state and resume nor- mal transmission. |
| 9 (R/NW)           | RLPIST     | Receive LPI Status. The EMAC_LPI_CTLSTAT.RLPIST bit indicates that the MAC is receiving the LPI pattern on the GMII or MII interface.                                                           |

Table 29-133: EMAC\_LPI\_CTLSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | TLPIST     | Transmit LPI Status. The EMAC_LPI_CTLSTAT.TLPIST bit indicates that the MAC is transmitting the LPI pattern on the GMII or MII interface.                                                                                                                                       |
| 3 (RC/NW)          | RLPIEX     | Receiver LPI State Exit Status. The EMAC_LPI_CTLSTAT.RLPIEX bit indicates that the MAC Receiver has stop- ped receiving the LPI pattern on the GMII or MII interface, exited the LPI state, and resumed the normal reception. This bit is cleared by a read into this register. |
| 2 (RC/NW)          | RLPIEN     | Receiver LPI State Enable Status. The EMAC_LPI_CTLSTAT.RLPIEN bit indicates that the MAC Receiver has re- ceived an LPI pattern and entered the LPI state. This bit is cleared by a read into this register.                                                                    |
| 1 (RC/NW)          | TLPIEX     | Transmitter LPI State Exit Status. The EMAC_LPI_CTLSTAT.TLPIEX bit indicates that the MAC transmitter has exited the LPI state after the program has cleared the EMAC_LPI_CTLSTAT.LPIEN bit and the LPI TWTimer has expired. This bit is cleared by a read into this register.  |
| 0 (RC/NW)          | TLPIEN     | Transmitter LPI State Enable Status. The EMAC_LPI_CTLSTAT.TLPIEN bit indicates that the MAC Transmitter has entered the LPI state due to the setting of the EMAC_LPI_CTLSTAT.LPIEN bit. This bit is cleared by a read into this register.                                       |

## Low Power Idle Timeout Register

The EMAC\_LPI\_TMRSCTL controls the timeout values in the LPI states. It specifies the time for which the MAC transmits the LPI pattern and also the time for which the MAC waits before resuming the normal transmission.

Figure 29-102: EMAC\_LPI\_TMRSCTL Register Diagram

<!-- image -->

Table 29-134: EMAC\_LPI\_TMRSCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:16 (R/W)        | LST        | Link Status Timer. The EMAC_LPI_TMRSCTL.LST bit, specifies the minimum time (in milliseconds) for which the link status from the PHY should be up (OKAY) before the LPI pattern can be transmitted to the PHY.            |
| 15:0 (R/W)         | TWT        | Timer Wait Time. The EMAC_LPI_TMRSCTL.TWT bit, specifies the minimum time (in microseconds) for which the MAC waits after it stops transmitting the LPI pattern to the PHY and before it resumes the normal transmission. |

## MAC Configuration Register

The EMAC\_MACCFG register configures MAC features.

Figure 29-103: EMAC\_MACCFG Register Diagram

<!-- image -->

Table 29-135: EMAC\_MACCFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:28 (R/W)        | SARC       | Source Address Insertion or Replacement Control. The EMAC_MACCFG.SARC bit, controls the source address insertion or replacement for all transmitted frames. |

Table 29-135: EMAC\_MACCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27 (R/W)           | TWOKPE     | Support for 2K packets. The EMAC_MACCFG.TWOKPE bit, IEEE 802.3as Support for 2K Packets When set, the MAC considers all frames, with up to 2,000 bytes length, as normal packets.                                                                                                                                              | Support for 2K packets. The EMAC_MACCFG.TWOKPE bit, IEEE 802.3as Support for 2K Packets When set, the MAC considers all frames, with up to 2,000 bytes length, as normal packets.                                                                                                                                              |
| 25 (R/W)           | CST        | CRC Stripping. The EMAC_MACCFG.CST bit, when set, directs the MAC to strip the last 4 bytes (FCS) of all frames of Ether type (Type field of frame greater than 0x0600) and drop these bytes before forwarding the frame to the application.                                                                                   | CRC Stripping. The EMAC_MACCFG.CST bit, when set, directs the MAC to strip the last 4 bytes (FCS) of all frames of Ether type (Type field of frame greater than 0x0600) and drop these bytes before forwarding the frame to the application.                                                                                   |
| 24 (R/W)           | TC         | Transmit Configuration in RGMII, SGMII or SMII. The EMAC_MACCFG.TC bit, enables the transmission of duplex mode, link speed, and link up or down information to the PHY in the RGMII port.                                                                                                                                     | Transmit Configuration in RGMII, SGMII or SMII. The EMAC_MACCFG.TC bit, enables the transmission of duplex mode, link speed, and link up or down information to the PHY in the RGMII port.                                                                                                                                     |
| 23 (R/W)           | WD         | Watch Dog Disable. The EMAC_MACCFG.WD bit, when set, disables the watchdog timer on the receiver, and can receive frames of up to 16,384 bytes. When this bit is reset, the MAC allows no more than 2,048 bytes (10,240 if EMAC_MACCFG.JE is set high) of the frame being received and cuts off any bytes received after that. | Watch Dog Disable. The EMAC_MACCFG.WD bit, when set, disables the watchdog timer on the receiver, and can receive frames of up to 16,384 bytes. When this bit is reset, the MAC allows no more than 2,048 bytes (10,240 if EMAC_MACCFG.JE is set high) of the frame being received and cuts off any bytes received after that. |
| 22 (R/W)           | JB         | Jabber Disable. The EMAC_MACCFG.JB bit, when set, disables the jabber timer on the transmitter, and can transfer frames of up to 16,384 bytes. When this bit is reset, the MAC cuts off the transmitter if the application sends out more than 2,048 bytes of data (10,240 if EMAC_MACCFG.JE is set high) during transmission. | Jabber Disable. The EMAC_MACCFG.JB bit, when set, disables the jabber timer on the transmitter, and can transfer frames of up to 16,384 bytes. When this bit is reset, the MAC cuts off the transmitter if the application sends out more than 2,048 bytes of data (10,240 if EMAC_MACCFG.JE is set high) during transmission. |
| 21 (R/W)           | BE         | Frame Burst Enable. The EMAC_MACCFG.BE bit, allows frame bursting during transmission in the GMII half-duplex mode.                                                                                                                                                                                                            | Frame Burst Enable. The EMAC_MACCFG.BE bit, allows frame bursting during transmission in the GMII half-duplex mode.                                                                                                                                                                                                            |
| 20 (R/W)           | JE         | Jumbo Frame Enable. The EMAC_MACCFG.JE bit, when set, directs the MAC to allow Jumbo frames of 9,018 bytes (9,022 bytes for VLAN tagged frames).                                                                                                                                                                               | Jumbo Frame Enable. The EMAC_MACCFG.JE bit, when set, directs the MAC to allow Jumbo frames of 9,018 bytes (9,022 bytes for VLAN tagged frames).                                                                                                                                                                               |
| 19:17 (R/W)        | IFG        | Inter-Frame Gap. The EMAC_MACCFG.IFG bits control the minimum inter-frame gap between frames during transmission. Note that in Half-Duplex mode, the minimum gap can be con- figured for 64 bit times ( EMAC_MACCFG.IFG =100) only. Lower values are not con- sidered.                                                         | Inter-Frame Gap. The EMAC_MACCFG.IFG bits control the minimum inter-frame gap between frames during transmission. Note that in Half-Duplex mode, the minimum gap can be con- figured for 64 bit times ( EMAC_MACCFG.IFG =100) only. Lower values are not con- sidered.                                                         |
| 19:17 (R/W)        | IFG        | 0                                                                                                                                                                                                                                                                                                                              | 96 bit times                                                                                                                                                                                                                                                                                                                   |
| 19:17 (R/W)        | IFG        | 1                                                                                                                                                                                                                                                                                                                              | 88 bit times                                                                                                                                                                                                                                                                                                                   |
| 19:17 (R/W)        | IFG        | 2                                                                                                                                                                                                                                                                                                                              | 80 bit times                                                                                                                                                                                                                                                                                                                   |
| 19:17 (R/W)        | IFG        | 3                                                                                                                                                                                                                                                                                                                              | 72 bit times                                                                                                                                                                                                                                                                                                                   |

Table 29-135: EMAC\_MACCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | DCRS       | 7 40 bit times Disable Carrier Sense. The EMAC_MACCFG.DCRS bit, when set, makes the MAC transmitter ignore the CRS signal during frame transmission in Half-Duplex mode. This request results in no errors generated due to Loss of Carrier or No Carrier during such transmission. When                                                                                                                                                                                                                                                                                                                                                                                                |
| 15 (R/NW)          | PS         | Port Select. The EMAC_MACCFG.PS bit selects between GMII and MII as: 0=GMII (1000 Mbps) and 1=MII (10/100 Mbps). This bit is read-only with the appropriate value in the 10/100 Mbps-only (always 1) or 1000 Mbps-only (always 0) configurations, and R_W in the default 10/100/1000 Mbps configuration.                                                                                                                                                                                                                                                                                                                                                                                |
| 14 (R/W)           | FES        | Speed of Operation. The EMAC_MACCFG.FES bit indicates the Ethernet speed as 10 Mbps (bit =0) or 100 Mbps (bit =1).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 13 (R/W)           | DO         | Disable Receive Own. The EMAC_MACCFG.DO bit, when set, disables MAC reception of frames when MAC is transmitting in Half-Duplex mode. When this bit is reset, the MAC receives all packets that are given by the PHY while transmitting. This bit is not applicable if the MAC is operating in Full-Duplex mode.                                                                                                                                                                                                                                                                                                                                                                        |
| 12 (R/W)           | LM         | Loopback Mode. The EMAC_MACCFG.LM bit, when set, directs the MAC to operate in internal loop back mode. (The media independent interface pins are not driven or sampled.)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 11 (R/W)           | DM         | Duplex Mode. The EMAC_MACCFG.DM bit, when set, directs the MAC to operate in a Full-Duplex mode where it can transmit and receive simultaneously.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 10 (R/W)           | IPC        | IP Checksum. The EMAC_MACCFG.IPC bit, when set, directs the MAC to calculate the 16-bit one's complement of the one's complement sum of all received Ethernet frame pay- loads. It also checks whether the IPv4 Header checksum (assumed to be bytes 25-26 or 29-30 (VLAN-tagged) of the received Ethernet frame) is correct for the received frame and gives the status in the receive status word. The EMAC_MACCFG.IPC bit, when set, enables IPv4 checksum checking for received frame payloads' TCP/UDP/ICMP headers. When this bit is reset, the Checksum Offload Engine function in the receiver is disabled and the corresponding PCE and IP HCE status bits are always cleared. |

Table 29-135: EMAC\_MACCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | DR         | Disable Retry. The EMAC_MACCFG.DR bit, when set, directs the MAC to attempt only 1 transmis- sion. When a collision occurs on the media independent interface, the MAC ignores the current frame transmission and reports a Frame Abort with excessive collision error in the transmit frame status. When the EMAC_MACCFG.DR bit is reset, the MAC at- tempts retries based on the settings of BL. This bit is applicable only to Half-Duplex mode.                                   | Disable Retry. The EMAC_MACCFG.DR bit, when set, directs the MAC to attempt only 1 transmis- sion. When a collision occurs on the media independent interface, the MAC ignores the current frame transmission and reports a Frame Abort with excessive collision error in the transmit frame status. When the EMAC_MACCFG.DR bit is reset, the MAC at- tempts retries based on the settings of BL. This bit is applicable only to Half-Duplex mode.                                   |
| 9 (R/W)            | DR         | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Retry enabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 9 (R/W)            | DR         | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Retry disabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 8 (R/W)            | LUD        | Link Up or Down. The EMAC_MACCFG.LUD bit, indicates whether the link is up or down during the transmission of configuration in the RGMII, SGMII, or SMII interface: 0 means Link down and 1 means Link Up                                                                                                                                                                                                                                                                             | Link Up or Down. The EMAC_MACCFG.LUD bit, indicates whether the link is up or down during the transmission of configuration in the RGMII, SGMII, or SMII interface: 0 means Link down and 1 means Link Up                                                                                                                                                                                                                                                                             |
| 7 (R/W)            | ACS        | Automatic Pad/CRC Stripping. The EMAC_MACCFG.ACS bit, when set, directs the MAC to strip the Pad/FCS field on incoming frames only if the length fields value is less than or equal to 1,500 bytes. All received frames with length field greater than or equal to 1,501 bytes are passed to the application without stripping the Pad/FCS field. When the EMAC_MACCFG.ACS bit is reset, the MAC passes all incoming frames to the Host unmodified.                                   | Automatic Pad/CRC Stripping. The EMAC_MACCFG.ACS bit, when set, directs the MAC to strip the Pad/FCS field on incoming frames only if the length fields value is less than or equal to 1,500 bytes. All received frames with length field greater than or equal to 1,501 bytes are passed to the application without stripping the Pad/FCS field. When the EMAC_MACCFG.ACS bit is reset, the MAC passes all incoming frames to the Host unmodified.                                   |
| 6:5 (R/W)          | BL         | Back Off Limit. The EMAC_MACCFG.BL bit selects the back-off limit, determining the random inte- ger number (r) of slot time delays (512 bit times for 10/100 Mbps) the MAC waits before rescheduling a transmission attempt during retries after a collision. This bit is applicable only to Half-Duplex mode.The random integer r takes the value in the range: 0 less-than-equal-to r less-than 2 k Where k is the minimum of n (number of transmission attempts) or a limit value. | Back Off Limit. The EMAC_MACCFG.BL bit selects the back-off limit, determining the random inte- ger number (r) of slot time delays (512 bit times for 10/100 Mbps) the MAC waits before rescheduling a transmission attempt during retries after a collision. This bit is applicable only to Half-Duplex mode.The random integer r takes the value in the range: 0 less-than-equal-to r less-than 2 k Where k is the minimum of n (number of transmission attempts) or a limit value. |
| 6:5 (R/W)          | BL         | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | k = min (n, 10)                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 6:5 (R/W)          | BL         | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | k = min (n, 8)                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 6:5 (R/W)          | BL         | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | k = min (n, 4)                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 6:5 (R/W)          | BL         | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | k = min (n, 1)                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 29-135: EMAC\_MACCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DC         | Deferral Check. The EMAC_MACCFG.DC bit, when set, enables the deferral check function in the MAC. The MAC issues a Frame Abort status, along with the excessive deferral error bit set in the transmit frame status when the transmit state machine is deferred for more than 24,288 bit times in 10/100-Mbps mode. If the Jumbo frame mode is ena- bled in 10/100-Mbps mode, the threshold for deferral is 155,680 bits times. Deferral begins when the transmitter is ready to transmit, but is prevented because of an active CRS (carrier sense) signal. Defer time is not cumulative. If the transmitter defers for 10,000 bit times, then transmits, collides, backs off, and then has to defer again after completion of back-off, the deferral timer resets to 0 and restarts. When the EMAC_MACCFG.DC bit is reset, the deferral check function is disabled and the MAC defers until the CRS signal goes inactive. This bit is applicable only in Half-Duplex mode. |
| 3 (R/W)            | TE         | Transmitter Enable. The EMAC_MACCFG.TE bit, when set, enables the transmit state machine of the MAC for transmission. When this bit is reset, the MAC transmit state machine is disa- bled after the completion of the transmission of the current frame, and will not trans- mit any further frames.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 2 (R/W)            | RE         | Receiver Enable. The EMAC_MACCFG.RE bit, when set, enables the receiver state machine of the MAC for receiving frames. When this bit is reset, the MAC receive state machine is disabled after the completion of the reception of the current frame, and does not re- ceive any further frames.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 1:0 (R/W)          | PRELEN     | Preamble Length for Transmit Frames. The EMAC_MACCFG.PRELEN bit, control the number of preamble bytes that are added to the beginning of every Transmit frame.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

## MAC Rx Frame Filter Register

The EMAC\_MACFRMFILT register controls receive frame filter features.

Figure 29-104: EMAC\_MACFRMFILT Register Diagram

<!-- image -->

Table 29-136: EMAC\_MACFRMFILT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | RA         | Receive All Frames. The EMAC_MACFRMFILT.RA bit, when set, directs the MAC Receiver module to pass to the Application all frames received irrespective of whether they pass the address filter. The result of the DA filtering is updated (pass or fail) in the corresponding bits in the Receive Status Word. When this bit is reset, the Receiver module passes to the Application only those frames that pass the DA address filter. |
| 21 (R/W)           | DNTU       | Drop non-TCP/UDP over IP Frames. The EMAC_MACFRMFILT.DNTU bit, enables the MAC to drop the non-TCP or UDP over IP frames. The MAC forward only those frames that are processed by the Layer 4 filter. When reset, this bit enables the MAC to forward all non-TCP or UDP over IP frames.                                                                                                                                               |
| 20 (R/W)           | IPFE       | Layer 3 and Layer 4 Filter Enable. The EMAC_MACFRMFILT.IPFE bit, enables the MAC to drop frames that do not match the enabled Layer 3 and Layer 4 filters. If Layer 3 or Layer 4 filters are not ena- bled for matching, this bit does not have any effect.                                                                                                                                                                            |

Table 29-136: EMAC\_MACFRMFILT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | VTFE       | VAN Tag Filter Enable. The EMAC_MACFRMFILT.VTFE bit, enables the MAC to drop VLAN tagged frames that do not match the VLAN Tag comparison.                                                                                                                                                                                                                                                         | VAN Tag Filter Enable. The EMAC_MACFRMFILT.VTFE bit, enables the MAC to drop VLAN tagged frames that do not match the VLAN Tag comparison.                                                                                                                                                                                                                                                         |
| 10 (R/W)           | HPF        | Hash or Perfect Filter. The EMAC_MACFRMFILT.HPF bit. when set, configures the address filter to pass a frame if it matches either the perfect filtering or the hash filtering as set by EMAC_MACFRMFILT.HMC or EMAC_MACFRMFILT.HUC bits. When EMAC_MACFRMFILT.HPF is low and either the EMAC_MACFRMFILT.HUC bit or EMAC_MACFRMFILT.HMC bit is set, the frame is passed only if it matches the Hash | Hash or Perfect Filter. The EMAC_MACFRMFILT.HPF bit. when set, configures the address filter to pass a frame if it matches either the perfect filtering or the hash filtering as set by EMAC_MACFRMFILT.HMC or EMAC_MACFRMFILT.HUC bits. When EMAC_MACFRMFILT.HPF is low and either the EMAC_MACFRMFILT.HUC bit or EMAC_MACFRMFILT.HMC bit is set, the frame is passed only if it matches the Hash |
| 9 (R/W)            | SAF        | filter. Source Address Filter Enable. When the EMAC_MACFRMFILT.SAF bit, is set, the MAC compares the SA field of the received frames with the values programmed in the enabled SA registers. If the comparison fails, the MAC drops the frame.                                                                                                                                                     | filter. Source Address Filter Enable. When the EMAC_MACFRMFILT.SAF bit, is set, the MAC compares the SA field of the received frames with the values programmed in the enabled SA registers. If the comparison fails, the MAC drops the frame.                                                                                                                                                     |
| 8 (R/W)            | SAIF       | Source Address Inverse Filtering. WHen the EMAC_MACFRMFILT.SAIF bit is set, the Address Check block operates in inverse filtering mode for the SA address comparison. The frames whose SA match- es the SA registers are marked as failing the SA Address filter.                                                                                                                                  | Source Address Inverse Filtering. WHen the EMAC_MACFRMFILT.SAIF bit is set, the Address Check block operates in inverse filtering mode for the SA address comparison. The frames whose SA match- es the SA registers are marked as failing the SA Address filter.                                                                                                                                  |
| 7:6 (R/W)          | PCF        | Pass Control Frames. The EMAC_MACFRMFILT.PCF bits control the forwarding of all control frames (in- cluding unicast and multicast PAUSE frames). Note that the processing of PAUSE control frames depends only on the value of the EMAC_FLOWCTL.RFE bit.                                                                                                                                           | Pass Control Frames. The EMAC_MACFRMFILT.PCF bits control the forwarding of all control frames (in- cluding unicast and multicast PAUSE frames). Note that the processing of PAUSE control frames depends only on the value of the EMAC_FLOWCTL.RFE bit.                                                                                                                                           |
| 7:6 (R/W)          | PCF        | 0                                                                                                                                                                                                                                                                                                                                                                                                  | Pass no control frames All control frames are filtered from reaching the application.                                                                                                                                                                                                                                                                                                              |
| 7:6 (R/W)          | PCF        | 1                                                                                                                                                                                                                                                                                                                                                                                                  | Pass no PAUSE frames All control frames are passed to the application (even if the fail the address filter), except for PAUSE frames.                                                                                                                                                                                                                                                              |
| 7:6 (R/W)          | PCF        | 2                                                                                                                                                                                                                                                                                                                                                                                                  | Pass all control frames All control frames are passed to the application (even if the fail the address filter).                                                                                                                                                                                                                                                                                    |
| 7:6 (R/W)          | PCF        | 3                                                                                                                                                                                                                                                                                                                                                                                                  | Pass address filtered control frames All control frames that pass the address filter are passed to the application.                                                                                                                                                                                                                                                                                |
| 5 (R/W)            | DBF        | Disable Broadcast Frames. The EMAC_MACFRMFILT.DBF bit, when set, directs the AFM module to filter all incoming broadcast frames. When this bit is reset, the AFM module passes all received broadcast frames.                                                                                                                                                                                      | Disable Broadcast Frames. The EMAC_MACFRMFILT.DBF bit, when set, directs the AFM module to filter all incoming broadcast frames. When this bit is reset, the AFM module passes all received broadcast frames.                                                                                                                                                                                      |
| 5 (R/W)            | DBF        | 0                                                                                                                                                                                                                                                                                                                                                                                                  | AFM module passes all received broadcast frames                                                                                                                                                                                                                                                                                                                                                    |
| 5 (R/W)            | DBF        | 1                                                                                                                                                                                                                                                                                                                                                                                                  | AFM module filters all incoming broadcast frames                                                                                                                                                                                                                                                                                                                                                   |

Table 29-136: EMAC\_MACFRMFILT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | PM         | Pass All Multicast Frames. The EMAC_MACFRMFILT.PM bit, when set, indicates that all received frames with a multicast destination address (first bit in the destination address field is =1) are passed. When this bit is reset, filtering of multicast frame depends on EMAC_MACFRMFILT.HMC bit.                                                                                                                |
| 3 (R/W)            | DAIF       | Destination Address Inverse Filtering. The EMAC_MACFRMFILT.DAIF bit, when set, directs the Address Check block to operate in inverse filtering mode for the DA address comparison for both unicast and multicast frames. When this bit is reset, normal filtering of frames is performed.                                                                                                                       |
| 2 (R/W)            | HMC        | Hash Multicast. The EMAC_MACFRMFILT.HMC bit, when set, directs the EMAC to perform desti- nation address filtering of received multicast frames according to the hash table. When this bit is reset, the MAC performs a perfect destination address filtering for multicast frames, that is, the MAC compares the DA field with the values programmed in the EMAC_ADDR0_HI and EMAC_ADDR0_LO address registers. |
| 1 (R/W)            | HUC        | Hash Unicast. The EMAC_MACFRMFILT.HUC bit, when set, directs the EMAC to perform desti- nation address filtering of unicast frames according to the hash table. When this bit is reset, the MAC performs a perfect destination address filtering for unicast frames, that is, it compares the DA field with the values programmed in the EMAC_ADDR0_HI and EMAC_ADDR0_LO address registers.                     |
| 0 (R/W)            | PR         | Promiscuous Mode. The EMAC_MACFRMFILT.PR bit, when set, directs the Address Filter module to pass all incoming frames regardless of its destination or source address. The DA Filter Fails status bits of the Receive Status Word is always cleared when EMAC_MACFRMFILT.PR is set.                                                                                                                             |

## AV MAC Control Register

The EMAC\_MAC\_AVCTL register controls the AV traffic by identifying the AV traffic and queuing it to appropriate channel. This register is present only when you select the AV feature during core configuration.

Figure 29-105: EMAC\_MAC\_AVCTL Register Diagram

<!-- image -->

Table 29-137: EMAC\_MAC\_AVCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:24 (R/W)        | PTPCH      | Channel for Queuing the PTP Packets. The EMAC_MAC_AVCTL.PTPCH bit field specifies the channel that untagged PTP packets, sent over the Ethernet payload and not over IPv4 or IPv6, are queued on. These bits are reserved if the receive paths of Channel 1 or Channel 2 are not enabled. Channel 0 |
| 25:24 (R/W)        | PTPCH      | 0                                                                                                                                                                                                                                                                                                   |
| 25:24 (R/W)        | PTPCH      | 1 Channel 1                                                                                                                                                                                                                                                                                         |
| 25:24 (R/W)        | PTPCH      | 2 Channel 2                                                                                                                                                                                                                                                                                         |
| 25:24 (R/W)        | PTPCH      | 3 Reserved                                                                                                                                                                                                                                                                                          |
| 22:21 (R/W)        | AVCH       | Channel for Queuing AV Control Packets. The EMAC_MAC_AVCTL.AVCH bit field specifies the channel the received untagged AV control packets are queued on. These bits are reserved if the receive paths of Chan- nel 1 or Channel 2 are not enabled.                                                   |
| 22:21 (R/W)        | AVCH       | 0 Channel 0                                                                                                                                                                                                                                                                                         |
| 22:21 (R/W)        | AVCH       | 1 Channel 1                                                                                                                                                                                                                                                                                         |
| 22:21 (R/W)        | AVCH       | 2 Channel 2                                                                                                                                                                                                                                                                                         |
| 22:21 (R/W)        | AVCH       | 3 Reserved                                                                                                                                                                                                                                                                                          |

Table 29-137: EMAC\_MAC\_AVCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | AVCD       | AV Channel Disable. When the EMAC_MAC_AVCTL.AVCD bit is set, the MAC forwards all packets to the default Channel 0 and the values programmed in the AVP, AVCH, and PTPCH fields are ignored. This bit is reserved and read-only if Channel 1 or Channel 2 receive paths are not selected during core configuration.                                                                                                                                                                                                                                                                                                                                                                           |
| 19 (R/W)           | VQE        | VLAN Tagged non-AV packets queuing enable. When the EMAC_MAC_AVCTL.VQE bit is set, the MAC also queues non-AV VLAN tagged packets into the available channels according to the value of the AVP bits. This bit is reserved and read-only if the Channel 1 and Channel 2 receive paths are not se- lected during core configuration.                                                                                                                                                                                                                                                                                                                                                           |
| 18:16 (R/W)        | AVP        | AV Priority for Queuing. The value programmed in the EMAC_MAC_AVCTL.AVP bits control the receive channel (0, 1, or 2) to which an AV packet with a given priority must be queued. If only Channel 2 receive path is enabled, the AV packets with priority value greater than or equal to the programmed value are queued on Channel 1 and all other packets are queued on Channel 0. If Channel 2 receive path is also enabled, the AV packets with priority value greater than or equal to the programmed value are queued on Channel 2 and all other packets are queued on Channel 0. These bits are applicable only if at least one additional receive channel is selected in the AV mode. |
| 15:0 (R/W)         | AVT        | AV Ether Type Value. The EMAC_MAC_AVCTL.AVT bit field contains the value that is compared with the EtherType field of the incoming (tagged or untagged) Ethernet frame to detect an AV packet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

## MMC Control Register

The EMAC\_MMC\_CTL register selects the MMC operating mode.

Figure 29-106: EMAC\_MMC\_CTL Register Diagram

<!-- image -->

Table 29-138: EMAC\_MMC\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | FULLPSET   | Full Preset. The EMAC_MMC_CTL.FULLPSET bit, when =0 (and EMAC_MMC_CTL.CNTRPSET =1), presets all MMCcounters to almost-half value. All octet counters get preset to 0x7FFF_F800 (half - 2KBytes) and all frame-counters gets preset to 0x7FFF_FFF0 (half - 16). When EMAC_MMC_CTL.FULLPSET =1 (and EMAC_MMC_CTL.CNTRPSET =1), all MMCcounters get preset to almost-full value. All octet counters get preset to 0xFFFF_F800 (full - 2KBytes) and all frame- counters gets preset to 0xFFFF_FFF0 (full - 16). For 16-bit counters, the almost-half preset values are 0x7800 and 0x7FF0 for the respective octet and frame counters. Simi- larly, the almost-full preset values for the 16-bit counters are 0xF800 and 0xFFF0. |
| 4 (R/W)            | CNTRPSET   | Counter Reset/Preset. The EMAC_MMC_CTL.CNTRPSET bit, when set, initializes all counters or presets counters to almost full or almost half as per EMAC_MMC_CTL.FULLPSET . The EMAC_MMC_CTL.CNTRPSET bit is cleared automatically after 1 clock cycle. This bit along with bit5 is useful for debugging and testing the assertion of interrupts be- cause of MMCcounter becoming half-full or full.                                                                                                                                                                                                                                                                                                                           |
| 3 (R/W)            | CNTRFRZ    | Counter Freeze. The EMAC_MMC_CTL.CNTRFRZ bit, when set, freezes all the MMCcounters to their current value. None of the MMCcounters are updated due to any transmitted or received frame, until this bit is reset to 0. If any MMCcounter is read with the EMAC_MMC_CTL.RDRST bit set, then that counter is also cleared in this mode.                                                                                                                                                                                                                                                                                                                                                                                      |

Table 29-138: EMAC\_MMC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | RDRST      | Read Reset. The EMAC_MMC_CTL.RDRST bit, when set, resets the MMCcounters to zero after Read (self-clearing after reset). The counters are cleared when the least significant byte lane (bits[7:0]) is read. |
| 1 (R/W)            | NOROLL     | No Rollover. The EMAC_MMC_CTL.NOROLL bit, when set, prevents counter rolls over to 0 after reaching max.                                                                                                    |
| 0 (R/W)            | RST        | Reset. The EMAC_MMC_CTL.RST bit, when set, resets all counters. This bit is cleared auto- matically after 1 clock cycle.                                                                                    |

## MMC Rx Interrupt Mask Register

The EMAC\_MMC\_RXIMSK register enables (unmasks) MMC receive interrupts.

Figure 29-107: EMAC\_MMC\_RXIMSK Register Diagram

<!-- image -->

Table 29-139: EMAC\_MMC\_RXIMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | CTLFIM     | Rx Control Frame Counter Interrupt Mask. The EMAC_MMC_RXIMSK.CTLFIM bit, masks the interrupt when the rxctrlframes_g counter reaches half of the maximum value or the maximum value. |
| 24 (R/W)           | RCVERRFIM  | Rx Error Frame Counter Interrupt Mask. The EMAC_MMC_RXIMSK.RCVERRFIM bit,masks the interrupt when the rxrcver- ror counter reaches half of the maximum value or the maximum value.   |
| 23 (R/W)           | WATCHERR   | Rx Watch Dog Error Count Half/Full Mask. The EMAC_MMC_RXIMSK.WATCHERR bit, when set, masks the interrupt when EMAC_RXWDOG_ERR counter reaches full or half.                          |
| 22 (R/W)           | VLANFRGB   | Rx VLAN Frames (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.VLANFRGB bit, when set, masks the interrupt when EMAC_RXVLANFRM_GB counter reaches full or half.                 |
| 21 (R/W)           | FIFOOV     | Rx FIFO Overflow Count Half/Full Mask. The EMAC_MMC_RXIMSK.FIFOOV bit, when set, masks the interrupt when EMAC_RXFIFO_OVF counter reaches full or half.                              |
| 20 (R/W)           | PAUSEFRM   | Rx Pause Frames Count Half/Full Mask. The EMAC_MMC_RXIMSK.PAUSEFRM bit, when set, masks the interrupt when EMAC_RXPAUSEFRM counter reaches full or half.                             |
| 19 (R/W)           | OUTRANGE   | Rx Out Of Range Type Count Half/Full Mask. The EMAC_MMC_RXIMSK.OUTRANGE bit, when set, masks the interrupt when EMAC_RXOORTYPE counter reaches full or half.                         |
| 18 (R/W)           | LENERR     | Rx Length Error Count Half/Full Mask. The EMAC_MMC_RXIMSK.LENERR bit, when set, masks the interrupt when EMAC_RXLEN_ERR counter reaches full or half.                                |
| 17 (R/W)           | UCASTG     | Rx Unicast Frames (Good) Count Half/Full Mask. The EMAC_MMC_RXIMSK.UCASTG bit, when set, masks the interrupt when EMAC_RXUCASTFRM_G counter reaches full or half.                    |
| 16 (R/W)           | R1024TOMAX | Rx 1024-to-max Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.R1024TOMAX bit, when set, masks the interrupt when EMAC_RX1024TOMAX_GB counter reaches full or half.      |
| 15 (R/W)           | R512TO1023 | Rx 512-to-1023 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.R512TO1023 bit, when set, masks the interrupt when EMAC_RX512TO1023_GB counter reaches full or half.      |
| 14 (R/W)           | R256TO511  | Rx 255-to-511 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.R256TO511 bit, when set, masks the interrupt when EMAC_RX256TO511_GB counter reaches full or half.         |

Table 29-139: EMAC\_MMC\_RXIMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | R128TO255  | Rx 128-to-255 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.R128TO255 bit, when set, masks the interrupt when EMAC_RX128TO255_GB counter reaches full or half. |
| 12 (R/W)           | R65TO127   | Rx 65-to-127 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.R65TO127 bit, when set, masks the interrupt when EMAC_RX65TO127_GB counter reaches full or half.    |
| 11 (R/W)           | R64        | Rx 64 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.R64 bit, when set, masks the interrupt when EMAC_RX64_GB counter reaches full or half.                     |
| 10 (R/W)           | OSIZEG     | Rx Oversize (Good) Count Half/Full Mask. The EMAC_MMC_RXIMSK.OSIZEG bit, when set, masks the interrupt when EMAC_RXOSIZE_G counter reaches full or half.                     |
| 9 (R/W)            | USIZEG     | Rx Undersize (Good) Count Half/Full Mask. The EMAC_MMC_RXIMSK.USIZEG bit, when set, masks the interrupt when EMAC_RXUSIZE_G counter reaches full or half.                    |
| 8 (R/W)            | JABERR     | Rx Jabber Error Count Half/Full Mask. The EMAC_MMC_RXIMSK.JABERR bit, when set, masks the interrupt when EMAC_RXJAB_ERR counter reaches full or half.                        |
| 7 (R/W)            | RUNTERR    | Rx Runt Error Count Half/Full Mask. The EMAC_MMC_RXIMSK.RUNTERR bit, when set, masks the interrupt when EMAC_RXRUNT_ERR counter reaches full or half.                        |
| 6 (R/W)            | ALIGNERR   | Rx Alignment Error Count Half/Full Mask. The EMAC_MMC_RXIMSK.ALIGNERR bit, when set, masks the interrupt when EMAC_RXALIGN_ERR counter reaches full or half.                 |
| 5 (R/W)            | CRCERR     | Rx CRC Error Count Half/Full Mask. The EMAC_MMC_RXIMSK.CRCERR bit, when set, masks the interrupt when EMAC_RXCRC_ERR counter reaches full or half.                           |
| 4 (R/W)            | MCASTG     | Rx Multicast Frames (Good) Count Half/Full Mask. The EMAC_MMC_RXIMSK.MCASTG bit, when set, masks the interrupt when EMAC_RXMCASTFRM_G counter reaches full or half.          |
| 3 (R/W)            | BCASTG     | Rx Broadcast Frames (Good) Count Half/Full Mask. The EMAC_MMC_RXIMSK.BCASTG bit, when set, masks the interrupt when EMAC_RXBCASTFRM_G counter reaches full or half.          |

Table 29-139: EMAC\_MMC\_RXIMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | OCTCNTG    | Rx Octet Count (Good) Count Half/Full Mask. The EMAC_MMC_RXIMSK.OCTCNTG bit, when set, masks the interrupt when EMAC_RXOCTCNT_G counter reaches full or half.       |
| 1 (R/W)            | OCTCNTGB   | Rx Octet Count (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.OCTCNTGB bit, when set, masks the interrupt when EMAC_RXOCTCNT_GB counter reaches half or full. |
| 0 (R/W)            | FRCNTGB    | Rx Frame Count (Good/Bad) Count Half/Full Mask. The EMAC_MMC_RXIMSK.FRCNTGB bit, when set, masks the interrupt when EMAC_RXFRMCNT_GB counter reaches half or full.  |

## MMC Rx Interrupt Register

The EMAC\_MMC\_RXINT register indicates status of MMC receive interrupts.

Figure 29-108: EMAC\_MMC\_RXINT Register Diagram

<!-- image -->

Table 29-140: EMAC\_MMC\_RXINT Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                   |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (RC/NW)         | RXCTLFIS    | Rx Control Frame Counter Interrupt Status. The EMAC_MMC_RXINT.RXCTLFIS bit is set when the rxctrlframes_g counter reaches half of the maximum value or the maximum value. |
| 24 (RC/NW)         | RXRCVERRFIS | Rx Error Frame Counter Interrupt Status. The EMAC_MMC_RXINT.RXRCVERRFIS bit is set when the rxrcverror counter reaches half of the maximum value or the maximum value.    |
| 23 (R/NW)          | WDOGERR     | Rx Watch Dog Error Count Half/Full. The EMAC_MMC_RXINT.WDOGERR bit is set when the EMAC_RXWDOG_ERR counter reaches full or half.                                          |
| 22 (R/NW)          | VLANFRGB    | Rx VLAN Frames (Good/Bad) Count Half/Full. The EMAC_MMC_RXINT.VLANFRGB bit is set when EMAC_RXVLANFRM_GB counter reaches full or half.                                    |
| 21 (R/NW)          | FIFOOVF     | Rx FIFO Overflow Count Half/Full. The EMAC_MMC_RXINT.FIFOOVF bit is set when EMAC_RXFIFO_OVF counter reaches full or half.                                                |
| 20 (R/NW)          | PAUSEFR     | Rx Pause Frames Count Half/Full. The EMAC_MMC_RXINT.PAUSEFR bit is set when EMAC_RXPAUSEFRM counter reaches full or half.                                                 |
| 19 (R/NW)          | OUTRANGE    | Rx Out Of Range Type Count Half/Full. The EMAC_MMC_RXINT.OUTRANGE bit is set when EMAC_RXOORTYPE counter reaches full or half.                                            |
| 18 (R/NW)          | LENERR      | Rx Length Error Count Half/Full. The EMAC_MMC_RXINT.LENERR bit is set when EMAC_RXLEN_ERR counter reaches full or half.                                                   |
| 17 (R/NW)          | UCASTG      | Rx Unicast Frames (Good) Count Half/Full. The EMAC_MMC_RXINT.UCASTG bit is set when EMAC_RXUCASTFRM_G coun- ter reaches full or half.                                     |
| 16 (R/NW)          | R1024TOMAX  | Rx 1024-to-max Octets (Good/Bad) Count Half/Full. The EMAC_MMC_RXINT.R1024TOMAX bit is set when EMAC_RX1024TOMAX_GB counter reaches full or half.                         |
| 15 (R/NW)          | R512TO1023  | Rx 512-to-1023 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_RXINT.R512TO1023 bit is set when EMAC_RX512TO1023_GB counter reaches full or half.                         |
| 14 (R/NW)          | R256TO511   | Rx 255-to-511 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_RXINT.R256TO511 bit is set when EMAC_RX256TO511_GB counter reaches full or half.                            |

Table 29-140: EMAC\_MMC\_RXINT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/NW)          | R128TO255  | Rx 128-to-255 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_RXINT.R128TO255 bit is set when EMAC_RX128TO255_GB counter reaches full or half. |
| 12 (R/NW)          | R65TO127   | Rx 65-to-127 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_RXINT.R65TO127 bit is set when EMAC_RX65TO127_GB counter reaches full or half.    |
| 11 (R/NW)          | R64        | Rx 64 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_RXINT.R64 bit is set when EMAC_RX64_GB counter reaches full or half.                     |
| 10 (R/NW)          | OSIZEG     | Rx Oversize (Good) Count Half/Full. The EMAC_MMC_RXINT.OSIZEG bit is set when EMAC_RXOSIZE_G counter reaches full or half.                     |
| 9 (R/NW)           | USIZEG     | Rx Undersize (Good) Count Half/Full. The EMAC_MMC_RXINT.USIZEG bit is set when EMAC_RXUSIZE_G counter reaches full or half.                    |
| 8 (R/NW)           | JABERR     | Rx Jabber Error Count Half/Full. The EMAC_MMC_RXINT.JABERR bit is set when EMAC_RXJAB_ERR counter reaches full or half.                        |
| 7 (R/NW)           | RUNTERR    | Rx Runt Error Count Half/Full. The EMAC_MMC_RXINT.RUNTERR bit is set when EMAC_RXRUNT_ERR counter reaches full or half.                        |
| 6 (R/NW)           | ALIGNERR   | Rx Alignment Error Count Half/Full. The EMAC_MMC_RXINT.ALIGNERR bit is set when EMAC_RXALIGN_ERR counter reaches full or half                  |
| 5 (R/NW)           | CRCERR     | Rx CRC Error Counter Half/Full. The EMAC_MMC_RXINT.CRCERR bit is set when EMAC_RXCRC_ERR counter reaches full or half.                         |
| 4 (R/NW)           | MCASTG     | Rx Multicast Count (Good) Half/Full. The EMAC_MMC_RXINT.MCASTG bit is set when EMAC_RXMCASTFRM_G coun- ter reaches full or half.               |
| 3 (R/NW)           | BCASTG     | Rx Broadcast Count (Good) Half/Full. The EMAC_MMC_RXINT.BCASTG bit is set when EMAC_RXBCASTFRM_G coun- ter reaches full or half.               |

Table 29-140: EMAC\_MMC\_RXINT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/NW)           | OCTCNTG    | Octet Count (Good) Half/Full. The EMAC_MMC_RXINT.OCTCNTG bit is set when EMAC_RXOCTCNT_G counter reaches full or half.        |
| 1 (R/NW)           | OCTCNTGB   | Octet Count (Good/Bad) Half/Full. The EMAC_MMC_RXINT.OCTCNTGB bit is set when EMAC_RXOCTCNT_GB counter reaches half or full.  |
| 0 (R/NW)           | FRCNTGB    | Frame Count (Good/Bad) Half/Full. The EMAC_MMC_RXINT.FRCNTGB bit is set when EMAC_RXFRMCNT_GB coun- ter reaches half or full. |

## MMC TX Interrupt Mask Register

The EMAC\_MMC\_TXIMSK register enables (unmasks) MMC transmit interrupts.

Figure 29-109: EMAC\_MMC\_TXIMSK Register Diagram

<!-- image -->

Table 29-141: EMAC\_MMC\_TXIMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | OSZGFIM    | Tx Oversize Good Frame Count Interrupt Mask. The EMAC_MMC_TXIMSK.OSZGFIM bit masks the interrupt when the txoversize_g counter reaches half of the maximum value or the maximum value. |
| 24 (R/W)           | VLANFRG    | Tx VLAN Frames (Good) Count Half/Full Mask. The EMAC_MMC_TXIMSK.VLANFRG bit, when set, masks the interrupt when EMAC_TXVLANFRM_G counter reaches full or half.                         |
| 23 (R/W)           | PAUSEFRM   | Tx Pause Frames Count Half/Full Mask. The EMAC_MMC_TXIMSK.PAUSEFRM bit, when set, masks the interrupt when EMAC_TXPAUSEFRM counter reaches full or half.                               |
| 22 (R/W)           | EXCESSDEF  | Tx Excess Deferred Count Half/Full Mask. The EMAC_MMC_TXIMSK.EXCESSDEF bit, when set, masks the interrupt when EMAC_TXEXCESSDEF counter reaches full or half.                          |
| 21 (R/W)           | FRCNTG     | Tx Frame Count (Good) Count Half/Full Mask. The EMAC_MMC_TXIMSK.FRCNTG bit, when set, masks the interrupt when EMAC_TXFRMCNT_G counter reaches full or half.                           |
| 20 (R/W)           | OCTCNTG    | Tx Octet Count (Good) Count Half/Full Mask. The EMAC_MMC_TXIMSK.OCTCNTG bit, when set, masks the interrupt when EMAC_TXOCTCNT_G counter reaches full or half.                          |
| 19 (R/W)           | CARRERR    | Tx Carrier Error Count Half/Full Mask. The EMAC_MMC_TXIMSK.CARRERR bit, when set, masks the interrupt when EMAC_TXCARR_ERR counter reaches full or half.                               |
| 18 (R/W)           | EXCESSCOL  | Tx Excess collision Count Half/Full Mask. The EMAC_MMC_TXIMSK.EXCESSCOL bit, when set, masks the interrupt when EMAC_TXEXCESSCOL counter reaches full or half.                         |
| 17 (R/W)           | LATECOL    | Tx Late Collision Count Half/Full Mask. The EMAC_MMC_TXIMSK.LATECOL bit, when set, masks the interrupt when EMAC_TXLATECOL counter reaches full or half.                               |
| 16 (R/W)           | DEFERRED   | Tx Deferred Count Half/Full Mask. The EMAC_MMC_TXIMSK.DEFERRED bit, when set, masks the interrupt when EMAC_TXDEFERRED counter reaches full or half.                                   |
| 15 (R/W)           | MULTCOLG   | Tx Multiple Collisions (Good) Count Mask. The EMAC_MMC_TXIMSK.MULTCOLG bit, when set, masks the interrupt when EMAC_TXMULTCOL_G counter reaches full or half.                          |
| 14 (R/W)           | SNGCOLG    | Tx Single Collision (Good) Count Half/Full Mask. The EMAC_MMC_TXIMSK.SNGCOLG bit, when set, masks the interrupt when EMAC_TXSNGCOL_G counter reaches full or half.                     |

Table 29-141: EMAC\_MMC\_TXIMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | UNDERR     | Tx Underflow Error Count Half/Full Mask. The EMAC_MMC_TXIMSK.UNDERR bit, when set, masks the interrupt when EMAC_TXUNDR_ERR counter reaches full or half.                       |
| 12 (R/W)           | BCASTGB    | Tx Broadcast Frames (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.BCASTGB bit, when set, masks the interrupt when EMAC_TXBCASTFRM_GB counter reaches full or half.       |
| 11 (R/W)           | MCASTGB    | Tx Multicast Frames (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.MCASTGB bit, when set, masks the interrupt when EMAC_TXMCASTFRM_GB counter reaches full or half.       |
| 10 (R/W)           | UCASTGB    | Tx Unicast Frames (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.UCASTGB bit, when set, masks the interrupt when EMAC_TXUCASTFRM_GB counter reaches full or half.         |
| 9 (R/W)            | T1024TOMAX | Tx 1024-to-max Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.T1024TOMAX bit, when set, masks the interrupt when EMAC_TX1024TOMAX_GB counter reaches full or half. |
| 8 (R/W)            | T512TO1023 | Tx 512-to-1023 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.T512TO1023 bit, when set, masks the interrupt when EMAC_TX512TO1023_GB counter reaches full or half. |
| 7 (R/W)            | T256TO511  | Tx 256-to-511 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.T256TO511 bit, when set, masks the interrupt when EMAC_TX256TO511_GB counter reaches full or half.    |
| 6 (R/W)            | T128TO255  | Tx 128-to-255 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.T128TO255 bit, when set, masks the interrupt when EMAC_TX128TO255_GB counter reaches full or half.    |
| 5 (R/W)            | T65TO127   | Tx 65-to-127 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.T65TO127 bit, when set, masks the interrupt when EMAC_TX65TO127_GB counter reaches full or half.       |
| 4 (R/W)            | T64        | Tx 64 Octets (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.T64 bit, when set, masks the interrupt when EMAC_TX64_GB counter reaches full or half.                        |
| 3 (R/W)            | MCASTG     | Tx Multicast Frames (Good) Count Half/Full Mask. The EMAC_MMC_TXIMSK.MCASTG bit, when set, masks the interrupt when EMAC_TXMCASTFRM_G counter reaches full or half.             |

Table 29-141: EMAC\_MMC\_TXIMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | BCASTG     | Tx Broadcast Frames (Good) Count Half/Full Mask. The EMAC_MMC_TXIMSK.BCASTG bit, when set, masks the interrupt when EMAC_TXBCASTFRM_G counter reaches full or half. |
| 1 (R/W)            | FRCNTGB    | Tx Frame Count (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.FRCNTGB bit, when set, masks the interrupt when EMAC_TXFRMCNT_GB counter reaches full or half.  |
| 0 (R/W)            | OCTCNTGB   | Tx Octet Count (Good/Bad) Count Half/Full Mask. The EMAC_MMC_TXIMSK.OCTCNTGB bit, when set, masks the interrupt when EMAC_TXOCTCNT_GB counter reaches full or half. |

## MMC Tx Interrupt Register

The EMAC\_MMC\_TXINT register indicates status of MMC transmit interrupts.

Figure 29-110: EMAC\_MMC\_TXINT Register Diagram

<!-- image -->

Table 29-142: EMAC\_MMC\_TXINT Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                        |
|--------------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (RC/NW)         | TXOSIZEGFIS | Tx Oversize Good Frame Count Interrupt Status. The EMAC_MMC_TXINT.TXOSIZEGFIS bit is set when the txoversize_g counter reaches half of the maximum value or the maximum value. |
| 24 (R/NW)          | VLANFRGB    | Tx VLAN Frames (Good) Count Half/Full. The EMAC_MMC_TXINT.VLANFRGB bit is set when EMAC_TXVLANFRM_G counter reaches full or half.                                              |
| 23 (R/NW)          | PAUSEFRM    | Tx Pause Frames Count Half/Full. The EMAC_MMC_TXINT.PAUSEFRM bit is set when EMAC_TXPAUSEFRM coun- ter reaches full or half.                                                   |
| 22 (R/NW)          | EXCESSDEF   | Tx Excess Deferred Count Half/Full. The EMAC_MMC_TXINT.EXCESSDEF bit is set when EMAC_TXEXCESSDEF counter reaches full or half.                                                |
| 21 (R/NW)          | FRCNTG      | Tx Frame Count (Good) Count Half/Full. The EMAC_MMC_TXINT.FRCNTG bit is set when EMAC_TXFRMCNT_G counter reaches full or half.                                                 |
| 20 (R/NW)          | OCTCNTG     | Tx Octet Count (Good) Count Half/Full. The EMAC_MMC_TXINT.OCTCNTG bit is set when EMAC_TXOCTCNT_G counter reaches full or half.                                                |
| 19 (R/NW)          | CARRERR     | Tx Carrier Error Count Half/Full. The EMAC_MMC_TXINT.CARRERR bit is set when EMAC_TXCARR_ERR counter reaches full or half.                                                     |
| 18 (R/NW)          | EXCESSCOL   | Tx Excess Collision Count Half/Full. The EMAC_MMC_TXINT.EXCESSCOL bit is set when EMAC_TXEXCESSCOL counter reaches full or half.                                               |
| 17 (R/NW)          | LATECOL     | Tx Late Collision Count Half/Full. The EMAC_MMC_TXINT.LATECOL bit is set when EMAC_TXLATECOL counter reaches full or half.                                                     |
| 16 (R/NW)          | DEFERRED    | Tx Deferred Count Half/Full. The EMAC_MMC_TXINT.DEFERRED bit is set when EMAC_TXDEFERRED coun- ter reaches full or half.                                                       |
| 15 (R/NW)          | MULTCOLG    | Tx Multiple collision (Good) Count Half/Full. The EMAC_MMC_TXINT.MULTCOLG bit is set when EMAC_TXMULTCOL_G counter reaches full or half.                                       |
| 14 (R/NW)          | SNGCOLG     | Tx Single Collision (Good) Count Half/Full. The EMAC_MMC_TXINT.SNGCOLG bit is set when EMAC_TXSNGCOL_G counter reaches full or half.                                           |

Table 29-142: EMAC\_MMC\_TXINT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/NW)          | UNDERR     | Tx Underflow Error Count Half/Full. The EMAC_MMC_TXINT.UNDERR bit is set when EMAC_TXUNDR_ERR counter reaches full or half.                       |
| 12 (R/NW)          | BCASTGB    | Tx Broadcast Frames (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.BCASTGB bit is set when EMAC_TXBCASTFRM_GB counter reaches full or half.       |
| 11 (R/NW)          | MCASTGB    | Tx Multicast Frames (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.MCASTGB bit is set when EMAC_TXMCASTFRM_GB counter reaches full or half.       |
| 10 (R/NW)          | UCASTGB    | Tx Unicast Frames (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.UCASTGB bit is set when EMAC_TXUCASTFRM_GB counter reaches full or half.         |
| 9 (R/NW)           | T1024TOMAX | Tx 1024-to-max Octets (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.T1024TOMAX bit is set when EMAC_TX1024TOMAX_GB counter reaches full or half. |
| 8 (R/NW)           | T512TO1023 | Tx 512-to-1023 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.T512TO1023 bit is set when EMAC_TX512TO1023_GB counter reaches full or half. |
| 7 (R/NW)           | T256TO511  | Tx 256-to-511 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.T256TO511 bit is set when EMAC_TX256TO511_GB counter reaches full or half.    |
| 6 (R/NW)           | T128TO255  | Tx 128-to-255 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.T128TO255 bit is set when EMAC_TX128TO255_GB counter reaches full or half.    |
| 5 (R/NW)           | T65TO127   | Tx 65-to-127 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.T65TO127 bit is set when EMAC_TX65TO127_GB counter reaches full or half.       |
| 4 (R/NW)           | T64        | Tx 64 Octets (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.T64 bit is set when EMAC_TX64_GB counter reaches full or half.                        |
| 3 (R/NW)           | MCASTG     | Tx Multicast Frames (Good) Count Half/Full. The EMAC_MMC_TXINT.MCASTG bit is set when EMAC_TXMCASTFRM_G coun- ter reaches full or half.           |

Table 29-142: EMAC\_MMC\_TXINT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/NW)           | BCASTG     | Tx Broadcast Frames (Good) Count Half/Full. The EMAC_MMC_TXINT.BCASTG bit is set when EMAC_TXBCASTFRM_G coun- ter reaches full or half. |
| 1 (R/NW)           | FRCNTGB    | Tx Frame Count (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.FRCNTGB bit is set when EMAC_TXFRMCNT_GB coun- ter reaches full or half.  |
| 0 (R/NW)           | OCTCNTGB   | Tx Octet Count (Good/Bad) Count Half/Full. The EMAC_MMC_TXINT.OCTCNTGB bit is set when EMAC_TXOCTCNT_GB counter reaches full or half.   |

## Rx 1024- to Max-Byte Frames (Good/Bad) Register

The EMAC\_RX1024TOMAX\_GB register contains a count of the number of good and bad frames received with length between 1024 and maxsize (inclusive) bytes, exclusive of preamble.

Figure 29-111: EMAC\_RX1024TOMAX\_GB Register Diagram

<!-- image -->

Table 29-143: EMAC\_RX1024TOMAX\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx 128- to 255-Byte Frames (Good/Bad) Register

The EMAC\_RX128TO255\_GB register contains a count of the number of good and bad frames received with length between 128 and 255 (inclusive) bytes, exclusive of preamble.

Figure 29-112: EMAC\_RX128TO255\_GB Register Diagram

<!-- image -->

Table 29-144: EMAC\_RX128TO255\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx 256- to 511-Byte Frames (Good/Bad) Register

The EMAC\_RX256TO511\_GB register contains a count of the number of good and bad frames received with length between 256 and 511 (inclusive) bytes, exclusive of preamble.

Figure 29-113: EMAC\_RX256TO511\_GB Register Diagram

<!-- image -->

Table 29-145: EMAC\_RX256TO511\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx 512- to 1023-Byte Frames (Good/Bad) Register

The EMAC\_RX512TO1023\_GB register contains a count of the number of good and bad frames received with length between 512 and 1023 (inclusive) bytes, exclusive of preamble.

Figure 29-114: EMAC\_RX512TO1023\_GB Register Diagram

<!-- image -->

Table 29-146: EMAC\_RX512TO1023\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx 64-Byte Frames (Good/Bad) Register

The EMAC\_RX64\_GB register contains a count of the number of good and bad frames received with length 64 bytes, exclusive of preamble.

Figure 29-115: EMAC\_RX64\_GB Register Diagram

<!-- image -->

Table 29-147: EMAC\_RX64\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx 65- to 127-Byte Frames (Good/Bad) Register

The EMAC\_RX65TO127\_GB register contains a count of the number of good and bad frames received with length between 65 and 127 (inclusive) bytes, exclusive of preamble.

Figure 29-116: EMAC\_RX65TO127\_GB Register Diagram

<!-- image -->

Table 29-148: EMAC\_RX65TO127\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx alignment Error Register

The EMAC\_RXALIGN\_ERR register contains a count of the number of frames received with alignment (dribble) error. Valid only in 10/100 mode.

Figure 29-117: EMAC\_RXALIGN\_ERR Register Diagram

<!-- image -->

Table 29-149: EMAC\_RXALIGN\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Broadcast Frames (Good) Register

The EMAC\_RXBCASTFRM\_G register contains a count of the number of good broadcast frames received.

Figure 29-118: EMAC\_RXBCASTFRM\_G Register Diagram

<!-- image -->

Table 29-150: EMAC\_RXBCASTFRM\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx CRC Error Register

The EMAC\_RXCRC\_ERR register contains a count of the number of frames received with CRC error.

Figure 29-119: EMAC\_RXCRC\_ERR Register Diagram

<!-- image -->

Table 29-151: EMAC\_RXCRC\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Good Control Frames Register

The EMAC\_RXCTLFRM\_G register contains a count of the number of good control frames received

Figure 29-120: EMAC\_RXCTLFRM\_G Register Diagram

<!-- image -->

Table 29-152: EMAC\_RXCTLFRM\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx FIFO Overflow Register

The EMAC\_RXFIFO\_OVF register contains a count of the number of missed received frames due to FIFO overflow.

Figure 29-121: EMAC\_RXFIFO\_OVF Register Diagram

<!-- image -->

Table 29-153: EMAC\_RXFIFO\_OVF Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Frame Count (Good/Bad) Register

The EMAC\_RXFRMCNT\_GB register contains a count of the number of good and bad frames received.

Figure 29-122: EMAC\_RXFRMCNT\_GB Register Diagram

<!-- image -->

Table 29-154: EMAC\_RXFRMCNT\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx ICMP Error Frames Register

The EMAC\_RXICMP\_ERR\_FRM register contains a count of the number of good IP datagrams whose ICMP payload has a checksum error.

Figure 29-123: EMAC\_RXICMP\_ERR\_FRM Register Diagram

<!-- image -->

Table 29-155: EMAC\_RXICMP\_ERR\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx ICMP Error Octets Register

The EMAC\_RXICMP\_ERR\_OCT register contains a count of the number of bytes received in an ICMP segment with checksum errors.

Figure 29-124: EMAC\_RXICMP\_ERR\_OCT Register Diagram

<!-- image -->

Table 29-156: EMAC\_RXICMP\_ERR\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx ICMP Good Frames Register

The EMAC\_RXICMP\_GD\_FRM register contains a count of the number of good IP datagrams with a good ICMP payload.

Figure 29-125: EMAC\_RXICMP\_GD\_FRM Register Diagram

<!-- image -->

Table 29-157: EMAC\_RXICMP\_GD\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx ICMP Good Octets Register

The EMAC\_RXICMP\_GD\_OCT register contains a count of the Number of bytes received in a good ICMP segment.

Figure 29-126: EMAC\_RXICMP\_GD\_OCT Register Diagram

<!-- image -->

Table 29-158: EMAC\_RXICMP\_GD\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 Datagrams Fragmented Frames Register

The EMAC\_RXIPV4\_FRAG\_FRM register contains a count of the number of good IPv4 datagrams with fragmentation.

Figure 29-127: EMAC\_RXIPV4\_FRAG\_FRM Register Diagram

<!-- image -->

Table 29-159: EMAC\_RXIPV4\_FRAG\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 Datagrams Fragmented Octets Register

The EMAC\_RXIPV4\_FRAG\_OCT register contains a count of the number of bytes received in fragmented IPv4 datagrams. The value in the IPv4 headers Length field is used to update this counter.

Figure 29-128: EMAC\_RXIPV4\_FRAG\_OCT Register Diagram

<!-- image -->

Table 29-160: EMAC\_RXIPV4\_FRAG\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 Datagrams (Good) Register

The EMAC\_RXIPV4\_GD\_FRM register contains a count of the number of good IPv4 datagrams received with the TCP, UDP , or ICMP payload.

Figure 29-129: EMAC\_RXIPV4\_GD\_FRM Register Diagram

<!-- image -->

Table 29-161: EMAC\_RXIPV4\_GD\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 Datagrams Good Octets Register

The EMAC\_RXIPV4\_GD\_OCT register contains a count of the number of bytes received in good IPv4 datagrams encapsulating TCP , UDP , or ICMP data.

Figure 29-130: EMAC\_RXIPV4\_GD\_OCT Register Diagram

<!-- image -->

Table 29-162: EMAC\_RXIPV4\_GD\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 Datagrams Header Errors Register

The EMAC\_RXIPV4\_HDR\_ERR\_FRM register contains a count of the number of IPv4 datagrams received with header (checksum, length, or version mismatch) errors.

Figure 29-131: EMAC\_RXIPV4\_HDR\_ERR\_FRM Register Diagram

<!-- image -->

Table 29-163: EMAC\_RXIPV4\_HDR\_ERR\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 Datagrams Header Errors Register

The EMAC\_RXIPV4\_HDR\_ERR\_OCT register contains a count of the number of bytes received in IPv4 datagrams with header errors (checksum, length, version mismatch). The value in the Length field of IPv4 header is used to update this counter.

Figure 29-132: EMAC\_RXIPV4\_HDR\_ERR\_OCT Register Diagram

<!-- image -->

Table 29-164: EMAC\_RXIPV4\_HDR\_ERR\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 Datagrams No Payload Frame Register

The EMAC\_RXIPV4\_NOPAY\_FRM register contains a count of the number of IPv4 datagram frames received that did not have a TCP , UDP , or ICMP payload processed by the Checksum engine.

Figure 29-133: EMAC\_RXIPV4\_NOPAY\_FRM Register Diagram

<!-- image -->

Table 29-165: EMAC\_RXIPV4\_NOPAY\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 Datagrams No Payload Octets Register

The EMAC\_RXIPV4\_NOPAY\_OCT register contains a count of the number of bytes received in IPv4 datagrams that did not have a TCP , UDP , or ICMP payload. The value in the IPv4 headers Length field is used to update this counter.

Figure 29-134: EMAC\_RXIPV4\_NOPAY\_OCT Register Diagram

<!-- image -->

Table 29-166: EMAC\_RXIPV4\_NOPAY\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 UDP Disabled Frames Register

The EMAC\_RXIPV4\_UDSBL\_FRM register contains a count of the number of good IPv4 datagrams received that had a UDP payload with checksum disabled.

Figure 29-135: EMAC\_RXIPV4\_UDSBL\_FRM Register Diagram

<!-- image -->

Table 29-167: EMAC\_RXIPV4\_UDSBL\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv4 UDP Disabled Octets Register

The EMAC\_RXIPV4\_UDSBL\_OCT register contains a count of the number of bytes received in a UDP segment that had the UDP checksum disabled. This counter does not count IP Header bytes.

Figure 29-136: EMAC\_RXIPV4\_UDSBL\_OCT Register Diagram

<!-- image -->

Table 29-168: EMAC\_RXIPV4\_UDSBL\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv6 Datagrams Good Frames Register

The EMAC\_RXIPV6\_GD\_FRM register contains a count of the number of good IPv6 datagrams received with TCP, UDP , or ICMP payloads.

Figure 29-137: EMAC\_RXIPV6\_GD\_FRM Register Diagram

<!-- image -->

Table 29-169: EMAC\_RXIPV6\_GD\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv6 Good Octets Register

The EMAC\_RXIPV6\_GD\_OCT register contains a count of the number of bytes received in good IPv6 datagrams encapsulating TCP , UDP or ICMPv6 data

Figure 29-138: EMAC\_RXIPV6\_GD\_OCT Register Diagram

<!-- image -->

Table 29-170: EMAC\_RXIPV6\_GD\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv6 Datagrams Header Error Frames Register

The EMAC\_RXIPV6\_HDR\_ERR\_FRM register contains a count of the number of IPv6 datagrams received with header errors (length or version mismatch).

Figure 29-139: EMAC\_RXIPV6\_HDR\_ERR\_FRM Register Diagram

<!-- image -->

Table 29-171: EMAC\_RXIPV6\_HDR\_ERR\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv6 Header Errors Register

The EMAC\_RXIPV6\_HDR\_ERR\_OCT register contains a count of the number of bytes received in IPv6 datagrams with header errors (length, version mismatch). The value in the IPv6 headers Length field is used to update this counter.

Figure 29-140: EMAC\_RXIPV6\_HDR\_ERR\_OCT Register Diagram

<!-- image -->

Table 29-172: EMAC\_RXIPV6\_HDR\_ERR\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv6 Datagrams No Payload Frames Register

The EMAC\_RXIPV6\_NOPAY\_FRM register contains a count of the number of IPv6 datagram frames received that did not have a TCP , UDP , or ICMP payload. This includes all IPv6 datagrams with fragmentation or security extension headers.

Figure 29-141: EMAC\_RXIPV6\_NOPAY\_FRM Register Diagram

<!-- image -->

Table 29-173: EMAC\_RXIPV6\_NOPAY\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx IPv6 No Payload Octets Register

The EMAC\_RXIPV6\_NOPAY\_OCT register contains a count of the number of bytes received in IPv6 datagrams that did not have a TCP , UDP , or ICMP payload. The value in the IPv6 headers Length field is used to update this counter.

Figure 29-142: EMAC\_RXIPV6\_NOPAY\_OCT Register Diagram

<!-- image -->

Table 29-174: EMAC\_RXIPV6\_NOPAY\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Jab Error Register

The EMAC\_RXJAB\_ERR register contains a count of the number of giant frames received with length greater than 1,518 bytes and with CRC error.

Figure 29-143: EMAC\_RXJAB\_ERR Register Diagram

<!-- image -->

Table 29-175: EMAC\_RXJAB\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Length Error Register

The EMAC\_RXLEN\_ERR register contains a count of the number of frames received with length error (Length type field frame size), for all frames with valid length field.

Figure 29-144: EMAC\_RXLEN\_ERR Register Diagram

<!-- image -->

Table 29-176: EMAC\_RXLEN\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Multicast Frames (Good) Register

The EMAC\_RXMCASTFRM\_G register contains a count of the number of good multicast frames received.

Figure 29-145: EMAC\_RXMCASTFRM\_G Register Diagram

<!-- image -->

Table 29-177: EMAC\_RXMCASTFRM\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Octet Count (Good) Register

The EMAC\_RXOCTCNT\_G register contains a count of the number of bytes received, exclusive of preamble, only in good frames.

Figure 29-146: EMAC\_RXOCTCNT\_G Register Diagram

<!-- image -->

Table 29-178: EMAC\_RXOCTCNT\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Octet Count (Good/Bad) Register

The EMAC\_RXOCTCNT\_GB register contains a count of the number of bytes received, exclusive of preamble, in good and bad frames.

Figure 29-147: EMAC\_RXOCTCNT\_GB Register Diagram

<!-- image -->

Table 29-179: EMAC\_RXOCTCNT\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Out Of Range Type Register

The EMAC\_RXOORTYPE register contains a count of the number of frames received with length field not equal to the valid frame size (greater than 1,500 but less than 1,536).

Figure 29-148: EMAC\_RXOORTYPE Register Diagram

<!-- image -->

Table 29-180: EMAC\_RXOORTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Oversize (Good) Register

The EMAC\_RXOSIZE\_G register contains a count of the number of frames received with length greater than the maxsize, without errors.

Figure 29-149: EMAC\_RXOSIZE\_G Register Diagram

<!-- image -->

Table 29-181: EMAC\_RXOSIZE\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Pause Frames Register

The EMAC\_RXPAUSEFRM register contains a count of the number of good and valid PAUSE frames received.

Figure 29-150: EMAC\_RXPAUSEFRM Register Diagram

<!-- image -->

Table 29-182: EMAC\_RXPAUSEFRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Error Frames Received Register

The EMAC\_RXRCV\_ERR register contains a count of the number of frames received with Receive error or Frame Extension error on GMII or MII interface

Figure 29-151: EMAC\_RXRCV\_ERR Register Diagram

<!-- image -->

Table 29-183: EMAC\_RXRCV\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Runt Error Register

The EMAC\_RXRUNT\_ERR register contains a count of the number of frames received with runt error.

Figure 29-152: EMAC\_RXRUNT\_ERR Register Diagram

<!-- image -->

Table 29-184: EMAC\_RXRUNT\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx TCP Error Frames Register

The EMAC\_RXTCP\_ERR\_FRM register contains a count of the number of good IP datagrams whose TCP payload has a checksum error.

Figure 29-153: EMAC\_RXTCP\_ERR\_FRM Register Diagram

<!-- image -->

Table 29-185: EMAC\_RXTCP\_ERR\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx TCP Error Octets Register

The EMAC\_RXTCP\_ERR\_OCT register contains a count of the number of bytes received in a TCP segment with checksum errors.

Figure 29-154: EMAC\_RXTCP\_ERR\_OCT Register Diagram

<!-- image -->

Table 29-186: EMAC\_RXTCP\_ERR\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx TCP Good Frames Register

The EMAC\_RXTCP\_GD\_FRM register contains a count of the number of good IP datagrams with a good TCP payload.

Figure 29-155: EMAC\_RXTCP\_GD\_FRM Register Diagram

<!-- image -->

Table 29-187: EMAC\_RXTCP\_GD\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx TCP Good Octets Register

The EMAC\_RXTCP\_GD\_OCT register contains a count of the number of bytes received in a good TCP segment.

Figure 29-156: EMAC\_RXTCP\_GD\_OCT Register Diagram

<!-- image -->

Table 29-188: EMAC\_RXTCP\_GD\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Unicast Frames (Good) Register

The EMAC\_RXUCASTFRM\_G register contains a count of the number of good unicast frames received.

Figure 29-157: EMAC\_RXUCASTFRM\_G Register Diagram

<!-- image -->

Table 29-189: EMAC\_RXUCASTFRM\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx UDP Error Frames Register

The EMAC\_RXUDP\_ERR\_FRM register contains a count of the number of good IP datagrams whose UDP payload has a checksum error.

Figure 29-158: EMAC\_RXUDP\_ERR\_FRM Register Diagram

<!-- image -->

Table 29-190: EMAC\_RXUDP\_ERR\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx UDP Error Octets Register

The EMAC\_RXUDP\_ERR\_OCT register contains a count of the number of bytes received in a UDP segment that had checksum errors.

Figure 29-159: EMAC\_RXUDP\_ERR\_OCT Register Diagram

<!-- image -->

Table 29-191: EMAC\_RXUDP\_ERR\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx UDP Good Frames Register

The EMAC\_RXUDP\_GD\_FRM register contains a count of the number of good IP datagrams with a good UDP payload. This counter is not updated when the rxipv4\_udsbl\_frms counter is incremented.

Figure 29-160: EMAC\_RXUDP\_GD\_FRM Register Diagram

<!-- image -->

Table 29-192: EMAC\_RXUDP\_GD\_FRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx UDP Good Octets Register

The EMAC\_RXUDP\_GD\_OCT register contains a count of the number of bytes received in a good UDP segment. This counter (and the counters below) does not count IP header bytes.

Figure 29-161: EMAC\_RXUDP\_GD\_OCT Register Diagram

<!-- image -->

Table 29-193: EMAC\_RXUDP\_GD\_OCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Undersize (Good) Register

The EMAC\_RXUSIZE\_G register contains a count of the number of frames received with length less than 64 bytes, without any errors.

Figure 29-162: EMAC\_RXUSIZE\_G Register Diagram

<!-- image -->

Table 29-194: EMAC\_RXUSIZE\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx VLAN Frames (Good/Bad) Register

The EMAC\_RXVLANFRM\_GB register contains a count of the number of good and bad VLAN frames received.

Figure 29-163: EMAC\_RXVLANFRM\_GB Register Diagram

<!-- image -->

Table 29-195: EMAC\_RXVLANFRM\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Rx Watch Dog Error Register

The EMAC\_RXWDOG\_ERR register contains a count of the number of frames received with error due to watchdog timeout error (frames with a data load larger than 2,048 bytes).

Figure 29-164: EMAC\_RXWDOG\_ERR Register Diagram

<!-- image -->

Table 29-196: EMAC\_RXWDOG\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## SMI Address Register

The EMAC\_SMI\_ADDR register contains the station management interface address and feature settings.

Figure 29-165: EMAC\_SMI\_ADDR Register Diagram

<!-- image -->

Table 29-197: EMAC\_SMI\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:11 (R/W)        | PA         | Physical Layer Address. The EMAC_SMI_ADDR.PA bits select the PHY. This field tells which of the 32 pos- sible PHY devices are being accessed.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Physical Layer Address. The EMAC_SMI_ADDR.PA bits select the PHY. This field tells which of the 32 pos- sible PHY devices are being accessed.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 10:6 (R/W)         | SMIR       | SMI Register Address. The EMAC_SMI_ADDR.SMIR bits select the desired Station Management Interface register in the selected PHY device.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | SMI Register Address. The EMAC_SMI_ADDR.SMIR bits select the desired Station Management Interface register in the selected PHY device.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 5:2 (R/W)          | CR         | Clock Range. The EMAC_SMI_ADDR.CR bits select the Clock Range, determining the frequency of the MDCclock as per the CLKO7 frequency. The suggested range of CLKO7 fre- quency applicable for each value below (when Bit[5] =0) ensures that the MDCclock is approximately between the frequency range 1.0 MHz - 2.5 MHz. When the MSB of this field is set, you can achieve MDCclock of frequency higher than the IEEE 802.3 specified frequency limit of 2.5 MHz and program a clock divider of lower value. For example, when CLKO7=100 MHz and you program these bits to b#1010, the result- ing MDCclock is 12.5 MHz, which is outside the limit of IEEE 802.3 specified range. Use the values shown only if the interface chips support faster MDCclocks. | Clock Range. The EMAC_SMI_ADDR.CR bits select the Clock Range, determining the frequency of the MDCclock as per the CLKO7 frequency. The suggested range of CLKO7 fre- quency applicable for each value below (when Bit[5] =0) ensures that the MDCclock is approximately between the frequency range 1.0 MHz - 2.5 MHz. When the MSB of this field is set, you can achieve MDCclock of frequency higher than the IEEE 802.3 specified frequency limit of 2.5 MHz and program a clock divider of lower value. For example, when CLKO7=100 MHz and you program these bits to b#1010, the result- ing MDCclock is 12.5 MHz, which is outside the limit of IEEE 802.3 specified range. Use the values shown only if the interface chips support faster MDCclocks. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | MDCClock= CLKO7/42 (for CLKO7=60-100MHz)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | MDCClock= CLKO7/62 (for CLKO7=100-125 MHz)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | MDCClock= CLKO7/16 (for CLKO7=20-35 MHz)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | MDCClock= CLKO7/26 (for CLKO7=35-60 MHz)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | MDCClock= CLKO7/4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 29-197: EMAC\_SMI\_ADDR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | SMIW       | 15 MDCClock= CLKO7/18 SMI Write. The EMAC_SMI_ADDR.SMIW bit, when set, tells the PHY this is a Write operation using the Station Management Interface Data register. If this bit is not set, this is a Read operation.                                                                                                                                                                                                                                                                                                                                                                               |
| 0 (R/W1S)          | SMIB       | SMI Busy. The EMAC_SMI_ADDR.SMIB bit should read low (=0) before writing to the EMAC_SMI_ADDR and EMAC_SMI_DATA registers. This bit must also =0 during a Write to EMAC_SMI_ADDR . During a PHY register access, this bit is set (=1) by the Application to indicate that a Read or Write access is in progress. The EMAC_SMI_DATA register should be kept valid until this bit is cleared by the MAC during a PHY Write operation. EMAC_SMI_DATA is invalid until this bit is cleared by the MAC during a PHY Read operation. The EMAC_SMI_ADDR should not be written to until this bit is cleared. |

## SMI Data Register

The EMAC\_SMI\_DATA register contains the station management interface data.

Figure 29-166: EMAC\_SMI\_DATA Register Diagram

<!-- image -->

Table 29-198: EMAC\_SMI\_DATA Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | SMID       | SMI Data. The EMAC_SMI_DATA.SMID bits contain the 16-bit data value read from the PHY after a Management Read operation or the 16-bit data value to be written to the PHY before a Management Write operation. |

## Time Stamp Addend Register

The EMAC\_TM\_ADDEND register lets software adjust the clock frequency linearly to match the master clock frequency.

Time Stamp Addend Register

Figure 29-167: EMAC\_TM\_ADDEND Register Diagram

<!-- image -->

Table 29-199: EMAC\_TM\_ADDEND Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | TSAR       | Time Stamp Addend Register. The EMAC_TM_ADDEND.TSAR bits indicate the 32-bit time value to be added to the Accumulator register to achieve time synchronization. |
| (R/W)              |            |                                                                                                                                                                  |

## Time Stamp Auxiliary TS Nano Seconds Register

The EMAC\_TM\_AUXSTMP\_NSEC register contains the low 32 bits (nanoseconds field) of the auxiliary time stamp.

Figure 29-168: EMAC\_TM\_AUXSTMP\_NSEC Register Diagram

<!-- image -->

Table 29-200: EMAC\_TM\_AUXSTMP\_NSEC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Time Value.               |
| (R/W)              |            |                           |

## Time Stamp Auxiliary TM Seconds Register

The EMAC\_TM\_AUXSTMP\_SEC register contains the low 32 bits of the seconds field of the auxiliary time stamp.

Figure 29-169: EMAC\_TM\_AUXSTMP\_SEC Register Diagram

<!-- image -->

Table 29-201: EMAC\_TM\_AUXSTMP\_SEC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Time Value.               |
| (R/W)              |            |                           |

## Time Stamp Control Register

The EMAC\_TM\_CTL register controls time stamp generation and update. The EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA , and EMAC\_TM\_CTL.TSEVNTENA bits work together to decide the set of PTP packet types for which snapshot needs to be taken. (Encoding shown in table.)

|   SNAPTYPSEL() | TSMSTRENA   | TSEVNTENA   | Messages for which snapshot is taken                                                   |
|----------------|-------------|-------------|----------------------------------------------------------------------------------------|
|             00 | X           | 0           | SYNC, Follow_Up, Delay_Req, Delay_Resp                                                 |
|             00 | 0           | 1           | SYNC                                                                                   |
|             00 | 1           | 1           | Delay_Req                                                                              |
|             01 | X           | 0           | SYNC, Follow_Up, Delay_Req, Delay_Resp, Pdelay_Req, Pdelay_Resp, Pdelay_Resp_Follow_Up |
|             01 | 0           | 1           | SYNC, Pdelay_Req, Pdelay_Resp                                                          |
|             01 | 1           | 1           | Delay_Req, Pdelay_Req, Pdelay_Resp                                                     |
|             10 | X           | X           | SYNC, Delay_Req                                                                        |
|             11 | X           | X           | Pdelay_Req, Pdelay_Resp                                                                |

Figure 29-170: EMAC\_TM\_CTL Register Diagram

<!-- image -->

Table 29-202: EMAC\_TM\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | ATSEN3     | Auxiliary Snapshot 3 Enable. The EMAC_TM_CTL.ATSEN3 bit, controls capturing the Auxiliary Snapshot Trigger 3 |
| 27 (R/W)           | ATSEN2     | Auxiliary Snapshot 2 Enable. The EMAC_TM_CTL.ATSEN2 bit, controls capturing the Auxiliary Snapshot Trigger 2 |
| 26 (R/W)           | ATSEN1     | Auxiliary Snapshot 1 Enable. The EMAC_TM_CTL.ATSEN1 bit, controls capturing the Auxiliary Snapshot Trigger 1 |

Table 29-202: EMAC\_TM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                      |
|--------------------|-------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | ATSEN0      | Auxiliary Snapshot 0 Enable. The EMAC_TM_CTL.ATSEN0 bit, controls capturing the Auxiliary Snapshot Trigger 0                                                                                                                                                                                 |
| 24 (R/W)           | ATSFC       | Auxiliary Time Stamp FIFO Clear. The EMAC_TM_CTL.ATSFC bit, when set, resets the pointers of the Auxiliary Snap- shot FIFO. This bit is cleared when the pointers are reset and the FIFO is empty. When this bit is cleared, auxiliary snapshots gets stored in the FIFO.                    |
| 18 (R/W)           | TSENMACADDR | Time Stamp Enable MAC Address. The EMAC_TM_CTL.TSENMACADDR bit, when set, uses the DA MAC address (that matches the EMAC_ADDR0_LO and EMAC_ADDR0_HI registers) to filter the PTP frames when PTP is sent directly over Ethernet.                                                             |
| 18 (R/W)           | TSENMACADDR | 0 Disable PTP MAC address filter                                                                                                                                                                                                                                                             |
| 17:16 (R/W)        | SNAPTYPSEL  | Snapshot Type Select. The EMAC_TM_CTL.SNAPTYPSEL bits along with bit 15 and 14 decide the set of PTP packet types for which snapshot needs to be taken. (See the table in the EMAC_TM_CTL register description.)                                                                             |
| 15 (R/W)           | TSMSTRENA   | Time Stamp Master (Frames) Enable. The EMAC_TM_CTL.TSMSTRENA bit, when set, takes the snapshot for messages relevant to master node only else snapshot is taken for PTP messages relevant to slave node.                                                                                     |
| 15 (R/W)           | TSMSTRENA   | 0 Enable Snapshot for Slave Messages                                                                                                                                                                                                                                                         |
| 15 (R/W)           | TSMSTRENA   | 1 Enable Snapshot for Master Messages                                                                                                                                                                                                                                                        |
| 14 (R/W)           | TSEVNTENA   | Time Stamp Event (PTP Frames) Enable. The EMAC_TM_CTL.TSEVNTENA bit, when set, takes the time stamp snapshot for PTP event messages only (SYNC, Delay_Req, Pdelay_Req, or Pdelay_Resp). When re- set, the snapshot is taken for all PTP messages except Announce, Management, and Signaling. |
| 14 (R/W)           | TSEVNTENA   | 0 Enable Time Stamp for All Messages                                                                                                                                                                                                                                                         |
| 14 (R/W)           | TSEVNTENA   | 1 Enable Time Stamp for Event Messages Only                                                                                                                                                                                                                                                  |

Table 29-202: EMAC\_TM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | TSIPV4ENA  | Time Stamp IPV4 (PTP Frames) Enable. The EMAC_TM_CTL.TSIPV4ENA bit, when set, directs the EMAC receiver to process the PTP packets encapsulated in UDP over IPv4 packets. When this bit is clear, the MAC ignores the PTP transported over UDP-IPv4 packets. This bit is set by default.                                                                                                                 |
| 13 (R/W)           | TSIPV4ENA  | 0 Disable Time Stamp for PTP Over IPv4 Frames                                                                                                                                                                                                                                                                                                                                                            |
| 13 (R/W)           | TSIPV4ENA  | 1 Enable Time Stamp for PTP Over IPv4 Frames                                                                                                                                                                                                                                                                                                                                                             |
| 12 (R/W)           | TSIPV6ENA  | Time Stamp IPV6 (PTP Frames) Enable. The EMAC_TM_CTL.TSIPV6ENA bit, when set, directs the EMAC receiver to process PTP packets encapsulated in UDP over IPv6 packets. When this bit is clear, the MAC ignores the PTP transported over UDP-IPv6 packets.                                                                                                                                                 |
| 12 (R/W)           | TSIPV6ENA  | 0 Disable Time Stamp for PTP Over IPv6 frames                                                                                                                                                                                                                                                                                                                                                            |
| 12 (R/W)           | TSIPV6ENA  | 1 Enable Time Stamp for PTP Over IPv6 Frames                                                                                                                                                                                                                                                                                                                                                             |
| 11 (R/W)           | TSIPENA    | Time Stamp IP Enable. The EMAC_TM_CTL.TSIPENA bit, when set, directs the EMAC receiver to process the PTP packets encapsulated directly in the Ethernet frames. When this bit is clear, the MAC ignores PTP over Ethernet packets.                                                                                                                                                                       |
| 11 (R/W)           | TSIPENA    | 0 Disable PTP Over Ethernet Frames                                                                                                                                                                                                                                                                                                                                                                       |
| 11 (R/W)           | TSIPENA    | 1 Enable PTP Over Ethernet Frames                                                                                                                                                                                                                                                                                                                                                                        |
| 10 (R/W)           | TSVER2ENA  | Time Stamp VER2 (Snooping) Enable. The EMAC_TM_CTL.TSVER2ENA bit, when set, processes the PTP packets using the 1588 version 2 format (enables PTP packet snooping for VER2) else processed us- ing the version 1 format.                                                                                                                                                                                |
| 10 (R/W)           | TSVER2ENA  | 0 Disable packet snooping for V2 frames                                                                                                                                                                                                                                                                                                                                                                  |
| 10 (R/W)           | TSVER2ENA  | 1 Enable packet snooping for V2 frames                                                                                                                                                                                                                                                                                                                                                                   |
| 9 (R/W)            | TSCTRLSSR  | Time Stamp Control Nanosecond Rollover. The EMAC_TM_CTL.TSCTRLSSR bit, when set, rolls over the EMAC_TM_NSEC register after 0x3B9A_C9FF value (10 9 -1) and increments the EMAC_TM_SEC regis- ter. When reset, the roll over value of EMAC_TM_NSEC register is 0x7FFF_FFFF. The nanosecond increment has to be programmed correctly depending on the PTP refer- ence clock frequency and this bit value. |
| 9 (R/W)            | TSCTRLSSR  | 0 Roll Over Nanosecond After 0x7FFFFFFF                                                                                                                                                                                                                                                                                                                                                                  |
| 9 (R/W)            | TSCTRLSSR  | 1 Roll Over Nanosecond After 0x3B9AC9FF                                                                                                                                                                                                                                                                                                                                                                  |

Table 29-202: EMAC\_TM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | TSENALL    | Time Stamp Enable All (Frames). The EMAC_TM_CTL.TSENALL bit, when set, enables the time stamp snapshot for all frames received by the core.                                                                                                                                                                                                                       |
| 5 (R/W1S)          | TSADDREG   | Time Stamp Addend Register Update. The EMAC_TM_CTL.TSADDREG bit, when set, updates the contents of the EMAC_TM_ADDEND register for fine correction. This bit is cleared when the update is completed. This bit should be zero before setting it.                                                                                                                  |
| 4 (R/W1S)          | TSTRIG     | Time Stamp (Target Time) Trigger Enable. The EMAC_TM_CTL.TSTRIG bit, when set, generates the time stamp interrupt when the System Time becomes greater than the value written in Time Stamp Target Time Seconds register. This bit is reset after the generation of the Time Stamp Trigger Interrupt. 1 Interrupt (TS) if system time is greater than target time |
| 3 (R/W1S)          | TSUPDT     | Time Stamp (System Time) Update. The EMAC_TM_CTL.TSUPDT bit, when set, updates (adds/subtracts) the system time with the value specified in the EMAC_TM_SECUPDT and EMAC_TM_NSECUPDT registers. This bit should read =0 before updating it. This bit is reset when the update is completed in hardware. The EMAC_TM_NSEC register is not updated.                 |
| 2 (R/W1S)          | TSINIT     | 1 System time updated with Time stamp register values Time Stamp (System Time) Initialize. The EMAC_TM_CTL.TSINIT bit, when set, initializes (over-writes) the system time                                                                                                                                                                                        |
| 1 (R/W)            | TSCFUPDT   | 1 System time initialized with Time stamp register values Time Stamp (System Time) Fine/Coarse Update. The EMAC_TM_CTL.TSCFUPDT bit, when set, indicates that the system time up- date is done using the fine correction method. When reset, it indicates the system time correction to be done using Coarse method.                                              |

Table 29-202: EMAC\_TM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | TSENA      | Time Stamp (PTP) Enable. The EMAC_TM_CTL.TSENA bit, when set, enables PTP module for time stamping transmitted and received frames. It also enables System Time which is used for time stamping the frames. Programs should initialize the System Time after setting this bit. |
| 0 (R/W)            | TSENA      | 0 Disable PTP Module                                                                                                                                                                                                                                                           |
| 0 (R/W)            | TSENA      | 1 Enable PTP Module                                                                                                                                                                                                                                                            |

## Time Stamp High Second Register

The EMAC\_TM\_HISEC register contains the upper 32 bits of the seconds field of the system time.

Figure 29-171: EMAC\_TM\_HISEC Register Diagram

<!-- image -->

Table 29-203: EMAC\_TM\_HISEC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | TSHWR      | Time Stamp Higher Word Seconds Register. The EMAC_TM_HISEC.TSHWR bit field contains the most significant 16-bits of the time stamp seconds value. The register is directly written to initialize the value. This register is incremented when there is an overflow from the 32-bits of the EMAC_TM_SEC register. |

## Time Stamp Nanoseconds Register

The EMAC\_TM\_NSEC register contains the nanoseconds field of the system time.

Time Stamp Nanoseconds

Figure 29-172: EMAC\_TM\_NSEC Register Diagram

<!-- image -->

Table 29-204: EMAC\_TM\_NSEC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:0 (R/NW)        | TSSS       | Time Stamp Nanoseconds. The value in the EMAC_TM_NSEC.TSSS bit field has the nanosecond representation of time, with an accuracy of 0.46 nanosecond. (When EMAC_TM_CTL.TSCTRLSSR is set, each bit represents 1 ns and the maximum val- ue will be 0x3B9A_C9FF, after which it rolls-over to zero). |

## Time Stamp Nanoseconds Update Register

The EMAC\_TM\_NSECUPDT register contains the low 32 bits to be added to, subtracted from, or written to the nanoseconds field of the system time.

Figure 29-173: EMAC\_TM\_NSECUPDT Register Diagram

<!-- image -->

Table 29-205: EMAC\_TM\_NSECUPDT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | ADDSUB     | Add or Subtract the Time. The EMAC_TM_NSECUPDT.ADDSUB bit, when set, subtracts the time value with the contents of the update registers. When this bit is reset, the time value is added with the contents of the update registers. |
| 30:0 (R/W)         | TSSS       | Time Stamp Sub Second Initialize/Increment. The value in the EMAC_TM_NSECUPDT.TSSS bit field indicates the time, in nano- seconds, to be initialized or added to or subtracted from the system time nanoseconds.                    |

## Time Stamp PPS Interval Register

The EMAC\_TM\_PPS0INTVL register contains the interval value for the time between rising edges (period) of PPS output.

Figure 29-174: EMAC\_TM\_PPS0INTVL Register Diagram

<!-- image -->

Table 29-206: EMAC\_TM\_PPS0INTVL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PPSINT     | PPS Output Signal Interval. The EMAC_TM_PPS0INTVL.PPSINT bits store the interval between the rising edges of PPS signal output in terms of units of sub-second increment value. You need to program one value less than the required interval. For example, if the PTP reference clock is 50 MHz (period of 20ns), and desired interval between rising edges of PPS signal output is 100ns (that is, 5 units of sub-second increment value), then you should program value 4 (5-1) in this register. |

## Time Stamp Target Time Nanoseconds Register

The EMAC\_TM\_PPS0NTGTM register contains the high 32 bits of the target nanoseconds field for comparison to the corresponding system time field.

Figure 29-175: EMAC\_TM\_PPS0NTGTM Register Diagram

<!-- image -->

Table 29-207: EMAC\_TM\_PPS0NTGTM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | TSTRBUSY   | Target Time Register Busy. The EMAC_TM_PPS0NTGTM.TSTRBUSY bit is set when Flexible PPS is enabled and the PPS Frequency Control bit field in the PPS Control register is programmed to 0001, 0010 or 0100. Programming the PPS Frequency Control bit field to 0001, 0010 or 0100, instructs the core to synchronize the EMAC_TM_PPS0TGTM and EMAC_TM_NSEC registers to the PTP clock domain. The EMAC clears this bit after synchronizing the EMAC_TM_PPS0TGTM and EMAC_TM_NSEC registers to the PTP clock domain The application must not update the EMAC_TM_PPS0TGTM and EMAC_TM_NSEC registers when this bit is read as 1. Otherwise, the synchronization of the previous programmed time gets corrupted. |
| 30:0 (R/W)         | TSTR       | Target Time Nano Seconds. The EMAC_TM_PPS0NTGTM.TSTR bit field stores the time in (signed) nanosec- onds. When the value of the time stamp matches the both EMAC_TM_PPS0TGTM and EMAC_TM_NSEC registers, based on the Target Time Register Mode bit field in the PPS control register, the MAC starts or stops the PPS signal output and generates an interrupt (if enabled). This value should not exceed 0x3B9A_C9FF when the Tar- get Time Register Mode bit field is set. The actual start or stop time of the PPS signal output may have an error margin up to one unit of sub-second increment value.                                                                                                  |

## Time Stamp Target Time Seconds Register

The EMAC\_TM\_PPS0TGTM register contains the high 32 bits of the target seconds field for comparison to the corresponding system time field.

Figure 29-176: EMAC\_TM\_PPS0TGTM Register Diagram

<!-- image -->

Table 29-208: EMAC\_TM\_PPS0TGTM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TSTR       | Target Time Seconds Register. The EMAC_TM_PPS0TGTM.TSTR bit field stores the time in seconds. When the time stamp value matches or exceeds both the value in this field and the value in the EMAC_TM_NSEC register, based on the selection in the Target Time Register Mode bit field, the MAC starts or stops the PPS signal output and generates an interrupt (if enabled). |

## PPS Width Register

The EMAC\_TM\_PPS0WIDTH register contains the interval value for the time between a rising and the next falling edge (width) of PPS output.

Figure 29-177: EMAC\_TM\_PPS0WIDTH Register Diagram

<!-- image -->

Table 29-209: EMAC\_TM\_PPS0WIDTH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PPSINT     | PPS Output Signal Interval. The EMAC_TM_PPS0WIDTH.PPSINT bits store the interval between a rising edge and the next falling edge (width) of PPS output in terms of units of sub second incre- ment value. Program one value less than the required interval. For example, if the PTP reference clock is 50 MHz (period of 20 ns) and the desired width of the PPS signal output is 60 ns (3 units of sub-second increment value), program the value 2 (3-1) in this register. |

## PPS 1 Interval Register

The EMAC\_TM\_PPS1INTVL register contains the number of units of sub-second increment value between the rising edges of PPS0 signal output.

Figure 29-178: EMAC\_TM\_PPS1INTVL Register Diagram

<!-- image -->

Table 29-210: EMAC\_TM\_PPS1INTVL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PPSINT     | PPS Output Signal Interval. The EMAC_TM_PPS1INTVL.PPSINT bit field contains the number of units of sub-second increment value between the rising edges of PPS0 signal output. |

## PPS 1 Target Time Nanoseconds Register

The EMAC\_TM\_PPS1NTGTM register is present only when the IEEE 1588 time-stamp feature is selected without external time-stamp input.

Figure 29-179: EMAC\_TM\_PPS1NTGTM Register Diagram

<!-- image -->

Table 29-211: EMAC\_TM\_PPS1NTGTM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1S)         | TSTRBUSY   | Target-Time Register. The EMAC_TM_PPS1NTGTM.TSTRBUSY bit field is cleared to 1b0 by the core (self clear). The application cannot clear this type of field, and a register write of 1b0 to this bit has no effect on this field. |
| 30:0 (R/W)         | TSTR       | Target-Time Low Register.                                                                                                                                                                                                        |

## PPS 1 Target Time Seconds Register

The EMAC\_TM\_PPS1TGTM register schedule an interrupt event when the system time exceeds the value programmed in these registers.

Figure 29-180: EMAC\_TM\_PPS1TGTM Register Diagram

<!-- image -->

Table 29-212: EMAC\_TM\_PPS1TGTM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TSTR       | Target Time Seconds Register. The EMAC_TM_PPS1TGTM.TSTR bit field stores the time in seconds. When the time-stamp value matches or exceeds both target time-stamp registers, the MAC starts or stops the PPS signal output and generates an interrupt. |

## PPS 1 Width Register

The EMAC\_TM\_PPS1WIDTH register contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS0 signal output

Figure 29-181: EMAC\_TM\_PPS1WIDTH Register Diagram

<!-- image -->

Table 29-213: EMAC\_TM\_PPS1WIDTH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PPSINT     | PPS Output Signal Width. The EMAC_TM_PPS1WIDTH.PPSINT bits store the interval between a rising edge and the next falling edge (width) of PPS output in terms of units of sub second incre- ment value. Program one value less than the required interval. For example, if the PTP reference clock is 50 MHz (period of 20 ns) and the desired width of the PPS signal output is 60 ns (3 units of sub-second increment value), program the value 2 (3-1) in this register. |

## PPS 2 Interval Register

The EMAC\_TM\_PPS2INTVL register contains the number of units of sub-second increment value between the rising edges of PPS0 signal output

Figure 29-182: EMAC\_TM\_PPS2INTVL Register Diagram

<!-- image -->

Table 29-214: EMAC\_TM\_PPS2INTVL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PPSINT     | PPS Output Signal Interval. The EMAC_TM_PPS2INTVL.PPSINT bit field contains the number of units of sub-second increment value between the rising edges of PPS2 signal output. |

## PPS 2 Target Time Nanoseconds Register

The EMAC\_TM\_PPS2NTGTM register is present only when the IEEE 1588 Time stamp feature is selected without external time stamp input.

Figure 29-183: EMAC\_TM\_PPS2NTGTM Register Diagram

<!-- image -->

Table 29-215: EMAC\_TM\_PPS2NTGTM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1S)         | TSTRBUSY   | Target time register. The EMAC_TM_PPS2NTGTM.TSTRBUSY bit field is cleared to 1b0 by the core (self clear). The application cannot clear this type of field, and a register write of 1b0 to this bit has no effect on this field. |
| 30:0 (R/W)         | TSTR       | Target Time Low Register.                                                                                                                                                                                                        |

## PPS 2 Target Time Seconds Register

The EMAC\_TM\_PPS2TGTM register schedule an interrupt event when the system time exceeds the value programmed in these registers.

Figure 29-184: EMAC\_TM\_PPS2TGTM Register Diagram

<!-- image -->

Table 29-216: EMAC\_TM\_PPS2TGTM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TSTR       | Target time seconds register. The EMAC_TM_PPS2TGTM.TSTR bit field stores the time in seconds. When the time-stamp value matches or exceeds both target time-stamp registers, the MAC starts or stops the PPS signal output and generates an interrupt. |

## PPS 2 Width Register

The EMAC\_TM\_PPS2WIDTH register contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS0 signal output

Figure 29-185: EMAC\_TM\_PPS2WIDTH Register Diagram

<!-- image -->

Table 29-217: EMAC\_TM\_PPS2WIDTH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PPSINT     | PPS Output Signal Width. The EMAC_TM_PPS2WIDTH.PPSINT bits store the interval between a rising edge and the next falling edge (width) of PPS output in terms of units of sub second incre- ment value. Program one value less than the required interval. For example, if the PTP reference clock is 50 MHz (period of 20 ns) and the desired width of the PPS signal output is 60 ns (3 units of sub-second increment value), program the value 2 (3-1) in this register. |

## PPS 3 Interval Register

The EMAC\_TM\_PPS3INTVL register contains the number of units of sub-second increment value between the rising edges of PPS0 signal output

Figure 29-186: EMAC\_TM\_PPS3INTVL Register Diagram

<!-- image -->

Table 29-218: EMAC\_TM\_PPS3INTVL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PPSINT     | PPS Output Signal Interval. The EMAC_TM_PPS3INTVL.PPSINT bit field contains the number of units of sub-second increment value between the rising edges of PPS3 signal output. |

## PPS 3 Target Time Nanoseconds Register

The EMAC\_TM\_PPS3NTGTM register is present only when the IEEE 1588 Time stamp feature is selected without external time stamp input.

Figure 29-187: EMAC\_TM\_PPS3NTGTM Register Diagram

<!-- image -->

Table 29-219: EMAC\_TM\_PPS3NTGTM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1S)         | TSTRBUSY   | Target Time Register. The EMAC_TM_PPS3NTGTM.TSTRBUSY bit field is cleared to 1b0 by the core (self clear). The application cannot clear this type of field, and a register write of 1b0 to this bit has no effect on this field. |
| 30:0 (R/W)         | TSTR       | Target Time Low Register.                                                                                                                                                                                                        |

## PPS 3 Target Time Seconds Register

The EMAC\_TM\_PPS3TGTM register schedule an interrupt event when the system time exceeds the value programmed in these registers.

Figure 29-188: EMAC\_TM\_PPS3TGTM Register Diagram

<!-- image -->

Table 29-220: EMAC\_TM\_PPS3TGTM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TSTR       | Target time seconds register. The EMAC_TM_PPS3TGTM.TSTR bit field stores the time in seconds. When the time-stamp value matches or exceeds both target time-stamp registers, the MAC starts or stops the PPS signal output and generates an interrupt. |

## PPS 3 Width Register

The EMAC\_TM\_PPS3WIDTH register contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS0 signal output

Figure 29-189: EMAC\_TM\_PPS3WIDTH Register Diagram

<!-- image -->

Table 29-221: EMAC\_TM\_PPS3WIDTH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PPSINT     | PPS Output Signal Width. The EMAC_TM_PPS3WIDTH.PPSINT bits store the interval between a rising edge and the next falling edge (width) of PPS output in terms of units of sub second incre- ment value. Program one value less than the required interval. For example, if the PTP reference clock is 50 MHz (period of 20 ns) and the desired width of the PPS signal output is 60 ns (3 units of sub-second increment value), program the value 2 (3-1) in this register. |

## PPS Control Register

The EMAC\_TM\_PPSCTL register controls the interval of PPS output.

When the EMAC\_TM\_PPSCTL.PPSEN bit is disabled (=0, fixed PPS output), the PPS Frequency Control (PPSCTL0) bits control the behavior of the PPS output signal. The default value of PPSCTRL is 0000 and the PPS output is 1 pulse every second. For other values of PPSCTRL, the PPS output becomes a generated clock. (See bit enumerations for frequencies.) In the binary rollover mode, the PPS output has a duty cycle of 50 percent with these frequencies. In the digital rollover mode, the PPS output frequency is an average number. The actual clock is of different frequency that gets synchronized every second. This behavior is because of the non-linear toggling of the bits in the digital rollover mode in System Time - Nanoseconds Register.

When the EMAC\_TM\_PPSCTL.PPSEN bit is enabled (=1, flexible PPS output), the PPS Frequency Control bits function as PPSCMD. (See bit enumerations for commands.) Programming these bits with a non-zero value instructs the core to initiate an event. After the command is transferred or synchronized to the PTP clock domain, these bits gets cleared automatically. Software should ensure that these bits are programmed only when they are allzero.

Figure 29-190: EMAC\_TM\_PPSCTL Register Diagram

<!-- image -->

Table 29-222: EMAC\_TM\_PPSCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                  |
|--------------------|-------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 30:29 (R/W)        | TRGTMODSEL3 | Target Time Register Mode for PPS3. The EMAC_TM_PPSCTL.TRGTMODSEL3 bits indicates the Target Time registers mode for PPS3 output signal. |

Table 29-222: EMAC\_TM\_PPSCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                           |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26:24 (R/W1S)      | PPSCMD3     | Flexible PPS3 Output Control. The EMAC_TM_PPSCTL.PPSCMD3 bits controls the flexible PPS3 output signal.                                                                                           | Flexible PPS3 Output Control. The EMAC_TM_PPSCTL.PPSCMD3 bits controls the flexible PPS3 output signal.                                                                                           |
| 22:21 (R/W)        | TRGTMODSEL2 | Target Time Register Mode for PPS2. The EMAC_TM_PPSCTL.TRGTMODSEL2 bits indicates the Target Time registers mode for PPS2 output signal.                                                          | Target Time Register Mode for PPS2. The EMAC_TM_PPSCTL.TRGTMODSEL2 bits indicates the Target Time registers mode for PPS2 output signal.                                                          |
| 18:16 (R/W1S)      | PPSCMD2     | Flexible PPS2 Output Control. The EMAC_TM_PPSCTL.PPSCMD2 bits controls the flexible PPS3 output signal.                                                                                           | Flexible PPS2 Output Control. The EMAC_TM_PPSCTL.PPSCMD2 bits controls the flexible PPS3 output signal.                                                                                           |
| 14:13 (R/W)        | TRGTMODSEL1 | Target Time Register Mode for PPS1. The EMAC_TM_PPSCTL.TRGTMODSEL1 bits indicates the Target Time registers mode for PPS1 output signal.                                                          | Target Time Register Mode for PPS1. The EMAC_TM_PPSCTL.TRGTMODSEL1 bits indicates the Target Time registers mode for PPS1 output signal.                                                          |
| 10:8 (R/W1S)       | PPSCMD1     | Flexible PPS1 Output Control. The EMAC_TM_PPSCTL.PPSCMD1 bits controls the flexible PPS1 output signal.                                                                                           | Flexible PPS1 Output Control. The EMAC_TM_PPSCTL.PPSCMD1 bits controls the flexible PPS1 output signal.                                                                                           |
| 6:5 (R/W)          | TRGTMODSEL0 | Target Time Register Mode. The EMAC_TM_PPSCTL.TRGTMODSEL0 bits select the target time register mode.                                                                                              | Target Time Register Mode. The EMAC_TM_PPSCTL.TRGTMODSEL0 bits select the target time register mode.                                                                                              |
| 6:5 (R/W)          | TRGTMODSEL0 | 0                                                                                                                                                                                                 | Interrupt Only The Target Time registers are program- med only for interrupt event generation.                                                                                                    |
| 6:5 (R/W)          | TRGTMODSEL0 | 1                                                                                                                                                                                                 | Reserved                                                                                                                                                                                          |
| 6:5 (R/W)          | TRGTMODSEL0 | 2                                                                                                                                                                                                 | Interrupt and PPS Start/Stop The Target Time registers are programmed for interrupt event and for starting or stopping the PPS output signal generation.                                          |
| 6:5 (R/W)          | TRGTMODSEL0 | 3                                                                                                                                                                                                 | PPS Start/Stop Only The Target Time registers are pro- grammed only for starting or stopping the PPS output signal generation. No interrupt is asserted.                                          |
| 4 (R/W)            | PPSEN       | Enable the flexible PPS output mode. The EMAC_TM_PPSCTL.PPSEN bit enables PPS operation. When set low, the PPS Frequency Control field controls frequency of Fixed PPS output. When set high, PPS | Enable the flexible PPS output mode. The EMAC_TM_PPSCTL.PPSEN bit enables PPS operation. When set low, the PPS Frequency Control field controls frequency of Fixed PPS output. When set high, PPS |

Table 29-222: EMAC\_TM\_PPSCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | PPSCTL0    | PPS Frequency Control. When the EMAC_TM_PPSCTL.PPSEN bit is disabled (=0, fixed PPS output), the EMAC_TM_PPSCTL.PPSCTL0 bits control the behavior of the PPS output signal. When the EMAC_TM_PPSCTL.PPSEN bit is enabled (=1, flexible PPS output), the EMAC_TM_PPSCTL.PPSCTL0 bits function as PPSCMD. (See bit enumerations for PPS output frequency, rollover, and PPS commands.) Programming these bits with a non-zero value instructs the core to initiate an event. After the command is transfer- red or synchronized to the PTP clock domain, these bits gets cleared automatically. Software should ensure that these bits are programmed only when they are all-zero. All values not shown in the bit enumerations are reserved. For more information about the EMAC_TM_PPSCTL.PPSCTL0 bits, see the pulse-per-second functional descrip- tion. | PPS Frequency Control. When the EMAC_TM_PPSCTL.PPSEN bit is disabled (=0, fixed PPS output), the EMAC_TM_PPSCTL.PPSCTL0 bits control the behavior of the PPS output signal. When the EMAC_TM_PPSCTL.PPSEN bit is enabled (=1, flexible PPS output), the EMAC_TM_PPSCTL.PPSCTL0 bits function as PPSCMD. (See bit enumerations for PPS output frequency, rollover, and PPS commands.) Programming these bits with a non-zero value instructs the core to initiate an event. After the command is transfer- red or synchronized to the PTP clock domain, these bits gets cleared automatically. Software should ensure that these bits are programmed only when they are all-zero. All values not shown in the bit enumerations are reserved. For more information about the EMAC_TM_PPSCTL.PPSCTL0 bits, see the pulse-per-second functional descrip- tion. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | CMD=No Command                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | CMD=START Single; BR=2kHz; DR=1kHz For more info, see register description.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | CMD=START Pulse; BR=4kHz; DR=2kHz For more info, see register description.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | CMD=Cancel START; BR=8kHz; DR=4kHz For more info, see register description.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | CMD=STOP Pulse Time; BR=16kHz; DR=8kHz For more info, see register description.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | CMD=STOP Pulse Now For more info, see register de- scription.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | CMD=Cancel STOP Pulse For more info, see register description.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

## Time Stamp Low Seconds Register

The EMAC\_TM\_SEC register contains the lower 32 bits of the seconds field of the system time.

Figure 29-191: EMAC\_TM\_SEC Register Diagram

<!-- image -->

Table 29-223: EMAC\_TM\_SEC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | TSS        | Time Stamp Second. The value in the EMAC_TM_SEC.TSS bit field indicates the current value in seconds of the System Time maintained by the core. |

## Time Stamp Seconds Update Register

The EMAC\_TM\_SECUPDT register contains the low 32 bits to be added to, subtracted from, or written to the seconds field of the system time.

Figure 29-192: EMAC\_TM\_SECUPDT Register Diagram

<!-- image -->

Table 29-224: EMAC\_TM\_SECUPDT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TSS        | Time Stamp Second Initialize/Update. The value in the EMAC_TM_SECUPDT.TSS bit field indicates the time, in seconds, to be initialized or added to or subtracted from the system time seconds. |

## Time Stamp Status Register

The EMAC\_TM\_STMPSTAT register contains the PTP status.

Figure 29-193: EMAC\_TM\_STMPSTAT Register Diagram

<!-- image -->

Table 29-225: EMAC\_TM\_STMPSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:25 (R/NW)       | ATSNS      | Auxiliary Time Stamp Number of Snapshots. The EMAC_TM_STMPSTAT.ATSNS bits indicate the number of Snapshots available in the FIFO. A value of 4 (100) indicates that the Auxiliary Snapshot FIFO is full. These bits are cleared (to 000) when the Auxiliary snapshot FIFO clear bit is set. |
| 24 (RC/NW)         | ATSSTM     | Auxiliary Time Stamp Snapshot Trigger Missed. The EMAC_TM_STMPSTAT.ATSSTM bit is set when the Auxiliary time stamp snap- shot FIFO is full and external trigger was set. This indicates that the latest snapshot was not stored in the FIFO.                                                |
| 19:16 (RC/NW)      | ATSSTN     | Auxiliary Timestamp Snapshot Trigger Identifier. The EMAC_TM_STMPSTAT.ATSSTN bit identify the Auxiliary trigger inputs for which the timestamp available in the Auxiliary Snapshot Register is applicable.                                                                                  |

Table 29-225: EMAC\_TM\_STMPSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (RC/NW)          | TSTRGTERR3 | Timestamp Target Time Error 3. The EMAC_TM_STMPSTAT.TSTRGTERR3 bit when set the target time being pro- grammed in PPS3 Target Time High Register and PPS3 Target Time Low Register, is already elapsed.                                                      |
| 8 (RC/NW)          | TSTARGT3   | Timestamp Target Time Reached for Target Time PPS3. The EMAC_TM_STMPSTAT.TSTARGT3 bit indicates that the value of system time is greater than or equal to the value specified in PPS3 Target Time High Register and PPS3 Target Time Low Register.           |
| 7 (RC/NW)          | TSTRGTERR2 | Timestamp Target Time Error 2. The EMAC_TM_STMPSTAT.TSTRGTERR2 bit when set the target time being pro- grammed in PPS2 Target Time High Register and PPS2 Target Time Low Register, is already elapsed.                                                      |
| 6 (RC/NW)          | TSTARGT2   | Timestamp Target Time Reached for Target Time PPS2. The EMAC_TM_STMPSTAT.TSTARGT2 bit indicates that the value of system time is greater than or equal to the value specified in PPS2 Target Time High Register and PPS2 Target Time Low Register.           |
| 5 (RC/NW)          | TSTRGTERR1 | Timestamp Target Time Error 1. The EMAC_TM_STMPSTAT.TSTRGTERR1 bit when set the target time being pro- grammed in PPS1 Target Time High Register and PPS1 Target Time Low Register, is already elapsed.                                                      |
| 4 (RC/NW)          | TSTARGT1   | Timestamp Target Time Reached for Target Time PPS1. The EMAC_TM_STMPSTAT.TSTARGT1 bit indicates that the value of system time is greater than or equal to the value specified in PPS1 Target Time High Register and PPS1 Target Time Low Register.           |
| 3 (R/W)            | TSTRGTERR0 | Time Stamp Target Time Programming Error. The EMAC_TM_STMPSTAT.TSTRGTERR0 bit is set when the target time, which is being programmed in the EMAC_TM_SEC and EMAC_TM_NSEC registers, has al- ready elapsed. This bit is cleared when read by the application. |
| 2 (RC/NW)          | ATSTS      | Auxiliary Time Stamp Trigger Snapshot. The EMAC_TM_STMPSTAT.ATSTS bit is set high when the auxiliary snapshot is written to the FIFO.                                                                                                                        |
| 1 (RC/NW)          | TSTARGT0   | Time Stamp Target Time Reached. The EMAC_TM_STMPSTAT.TSTARGT0 bit, when set, indicates the value of system time has reached or passed the value specified in the EMAC_TM_PPS0TGTM and EMAC_TM_NSEC registers.                                                |

Table 29-225: EMAC\_TM\_STMPSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (RC/NW)          | TSSOVF     | Time Stamp Seconds Overflow. The EMAC_TM_STMPSTAT.TSSOVF bit, when set, indicates that the seconds value of the time stamp (when supporting PTP version 2 format) has overflowed beyond 0xFFFF_FFFF. |

## Time Stamp Sub Second Increment Register

The EMAC\_TM\_SUBSEC register contains the value by which the system time nano second is incremented.

Figure 29-194: EMAC\_TM\_SUBSEC Register Diagram

<!-- image -->

Table 29-226: EMAC\_TM\_SUBSEC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | SSINC      | Sub-Second Increment Value. The value in the EMAC_TM_SUBSEC.SSINC bits is accumulated every PTP clock cycle with the contents of the nanosecond register. For example, when PTP clock is 50 MHz (period is 20 ns), the processor should program 20 (0x14) when the EMAC_TM_NSEC register has an accuracy of 1 ns ( EMAC_TM_CTL.TSCTRLSSR bit is set). When EMAC_TM_CTL.TSCTRLSSR is clear, the EMAC_TM_NSEC regis- ter has a resolution of ~0.465ns. In this case, the processor should program a value of 43 (0x2B) that is derived by 20ns/0.465. |

## Tx 1024- to Max-Byte Frames (Good/Bad) Register

The EMAC\_TX1024TOMAX\_GB register contains the count of the number of good and bad frames transmitted with length between 1024 and maxsize (inclusive) bytes, exclusive of preamble and retried frames.

Figure 29-195: EMAC\_TX1024TOMAX\_GB Register Diagram

<!-- image -->

Table 29-227: EMAC\_TX1024TOMAX\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx 128- to 255-Byte Frames (Good/Bad) Register

The EMAC\_TX128TO255\_GB register contains the count of the number of good and bad frames transmitted with length between 128 and 255 (inclusive) bytes, exclusive of preamble and retried frames.

Figure 29-196: EMAC\_TX128TO255\_GB Register Diagram

<!-- image -->

Table 29-228: EMAC\_TX128TO255\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx 256- to 511-Byte Frames (Good/Bad) Register

The EMAC\_TX256TO511\_GB register contains the count of the number of good and bad frames transmitted with length between 256 and 511 (inclusive) bytes, exclusive of preamble and retried frames.

Figure 29-197: EMAC\_TX256TO511\_GB Register Diagram

<!-- image -->

Table 29-229: EMAC\_TX256TO511\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx 512- to 1023-Byte Frames (Good/Bad) Register

The EMAC\_TX512TO1023\_GB register contains the count of the number of good and bad frames transmitted with length between 512 and 1023 (inclusive) bytes, exclusive of preamble and retried frames.

Figure 29-198: EMAC\_TX512TO1023\_GB Register Diagram

<!-- image -->

Table 29-230: EMAC\_TX512TO1023\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx 64-Byte Frames (Good/Bad) Register

The EMAC\_TX64\_GB register contains the count of the number of good and bad frames transmitted with length 64 bytes, exclusive of preamble and retried frames.

Figure 29-199: EMAC\_TX64\_GB Register Diagram

<!-- image -->

Table 29-231: EMAC\_TX64\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx 65- to 127-Byte Frames (Good/Bad) Register

The EMAC\_TX65TO127\_GB register contains the count of the number of good and bad frames transmitted with length between 65 and 127 (inclusive) bytes, exclusive of preamble and retried frames.

Figure 29-200: EMAC\_TX65TO127\_GB Register Diagram

<!-- image -->

Table 29-232: EMAC\_TX65TO127\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Broadcast Frames (Good) Register

The EMAC\_TXBCASTFRM\_G register contains the count of the number of good broadcast frames transmitted.

Figure 29-201: EMAC\_TXBCASTFRM\_G Register Diagram

<!-- image -->

Table 29-233: EMAC\_TXBCASTFRM\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Broadcast Frames (Good/Bad) Register

The EMAC\_TXBCASTFRM\_GB register contains the count of the number of good and bad broadcast frames transmitted.

Figure 29-202: EMAC\_TXBCASTFRM\_GB Register Diagram

<!-- image -->

Table 29-234: EMAC\_TXBCASTFRM\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Carrier Error Register

The EMAC\_TXCARR\_ERR register contains a count of the number of frames aborted due to carrier sense error (no carrier or loss of carrier).

Figure 29-203: EMAC\_TXCARR\_ERR Register Diagram

<!-- image -->

Table 29-235: EMAC\_TXCARR\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Deferred Register

The EMAC\_TXDEFERRED register contains a count of the number of successfully transmitted frames after a deferral in Half-duplex mode.

Figure 29-204: EMAC\_TXDEFERRED Register Diagram

<!-- image -->

Table 29-236: EMAC\_TXDEFERRED Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Excess Collision Register

The EMAC\_TXEXCESSCOL register contains a count of the number of frames aborted due to excessive (16) collision errors.

Figure 29-205: EMAC\_TXEXCESSCOL Register Diagram

<!-- image -->

Table 29-237: EMAC\_TXEXCESSCOL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Excess Deferral Register

The EMAC\_TXEXCESSDEF register contains a count of the number of frames aborted due to excessive deferral error (deferred for more than two max-sized frame times).

Figure 29-206: EMAC\_TXEXCESSDEF Register Diagram

<!-- image -->

Table 29-238: EMAC\_TXEXCESSDEF Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Frame Count (Good) Register

The EMAC\_TXFRMCNT\_G register contains a count of the number of good frames transmitted.

Figure 29-207: EMAC\_TXFRMCNT\_G Register Diagram

<!-- image -->

Table 29-239: EMAC\_TXFRMCNT\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Frame Count (Good/Bad) Register

The EMAC\_TXFRMCNT\_GB register contains the count of the number of good and bad frames transmitted, exclusive of retried frames.

Figure 29-208: EMAC\_TXFRMCNT\_GB Register Diagram

<!-- image -->

Table 29-240: EMAC\_TXFRMCNT\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Late Collision Register

The EMAC\_TXLATECOL register contains a count of the number of frames aborted due to late collision error.

Figure 29-209: EMAC\_TXLATECOL Register Diagram

<!-- image -->

Table 29-241: EMAC\_TXLATECOL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Multicast Frames (Good) Register

The EMAC\_TXMCASTFRM\_G register contains the count of the number of good multicast frames transmitted.

Figure 29-210: EMAC\_TXMCASTFRM\_G Register Diagram

<!-- image -->

Table 29-242: EMAC\_TXMCASTFRM\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Multicast Frames (Good/Bad) Register

The EMAC\_TXMCASTFRM\_GB register contains the count of the number of good and bad multicast frames transmitted.

Figure 29-211: EMAC\_TXMCASTFRM\_GB Register Diagram

<!-- image -->

Table 29-243: EMAC\_TXMCASTFRM\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Multiple Collision (Good) Register

The EMAC\_TXMULTCOL\_G register contains a count of the number of successfully transmitted frames after more than a single collision in Half-duplex mode.

Figure 29-212: EMAC\_TXMULTCOL\_G Register Diagram

<!-- image -->

Table 29-244: EMAC\_TXMULTCOL\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Octet Count (Good) Register

The EMAC\_TXOCTCNT\_G register contains a count of the number of bytes transmitted, exclusive of preamble, in good frames only.

Figure 29-213: EMAC\_TXOCTCNT\_G Register Diagram

<!-- image -->

Table 29-245: EMAC\_TXOCTCNT\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx OCT Count (Good/Bad) Register

The EMAC\_TXOCTCNT\_GB register contains the count of the number of bytes transmitted, exclusive of the preamble and retried bytes, in good and bad frames.

Figure 29-214: EMAC\_TXOCTCNT\_GB Register Diagram

<!-- image -->

Table 29-246: EMAC\_TXOCTCNT\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Number of Tx Frames (Good) greater than maxsize

The EMAC\_TXOVRSIZE\_G register contains a count of the number of good frames transmitted with length greater than the maxsize.

Figure 29-215: EMAC\_TXOVRSIZE\_G Register Diagram

<!-- image -->

Table 29-247: EMAC\_TXOVRSIZE\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count value.              |
| (R/NW)             |            |                           |

## Tx Pause Frame Register

The EMAC\_TXPAUSEFRM register contains a count of the number of good PAUSE frames transmitted.

Figure 29-216: EMAC\_TXPAUSEFRM Register Diagram

<!-- image -->

Table 29-248: EMAC\_TXPAUSEFRM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Single Collision (Good) Register

The EMAC\_TXSNGCOL\_G register contains a count of the number of successfully transmitted frames after a single collision in Half-duplex mode.

Figure 29-217: EMAC\_TXSNGCOL\_G Register Diagram

<!-- image -->

Table 29-249: EMAC\_TXSNGCOL\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Unicast Frames (Good/Bad) Register

The EMAC\_TXUCASTFRM\_GB register contains the count of the number of good and bad unicast frames transmitted.

Figure 29-218: EMAC\_TXUCASTFRM\_GB Register Diagram

<!-- image -->

Table 29-250: EMAC\_TXUCASTFRM\_GB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx Underflow Error Register

The EMAC\_TXUNDR\_ERR register contains a count of the number of frames aborted due to frame underflow error.

Figure 29-219: EMAC\_TXUNDR\_ERR Register Diagram

<!-- image -->

Table 29-251: EMAC\_TXUNDR\_ERR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## Tx VLAN Frames (Good) Register

The EMAC\_TXVLANFRM\_G register contains a count of the number of good VLAN frames transmitted, exclusive of retried frames.

Figure 29-220: EMAC\_TXVLANFRM\_G Register Diagram

<!-- image -->

Table 29-252: EMAC\_TXVLANFRM\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | CNT        | Count Value.              |
| (R/NW)             |            |                           |

## VLAN Tag Register

The EMAC\_VLANTAG register contains the VLAN tag.

Figure 29-221: EMAC\_VLANTAG Register Diagram

<!-- image -->

Table 29-253: EMAC\_VLANTAG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | VTHM       | VLAN Tag Hash Table Match Enable. The EMAC_VLANTAG.VTHM bit, When set, the most significant four bits of the VLAN tags CRC are used to index the content of VLAN Hash Table Register. A value of 1 in the VLAN Hash Table register, corresponding to the index, indicates that the frame matched the VLAN hash table.                                                                                                     |
| 18 (R/W)           | ESVL       | Enable S-VLAN. When this bit is set, the MAC transmitter and receiver also consider the S-VLAN (Type = 0x88A8) frames as valid VLAN tagged frames.                                                                                                                                                                                                                                                                        |
| 17 (R/W)           | VTIM       | VLAN Tag Inverse Match Enable. The EMAC_VLANTAG.VTIM bit, enables the VLAN Tag inverse matching. The frames that do not have matching VLAN Tag are marked as matched.                                                                                                                                                                                                                                                     |
| 16 (R/W)           | ETV        | Enable Tag VLAN Comparison. The EMAC_VLANTAG.ETV bit, when set, directs the EMAC to use a 12-bit VLAN identifier, rather than the complete 16-bit VLAN tag, for comparison and filtering. Bits[11:0] of the VLAN tag are compared with the corresponding field in the received VLAN-tagged frame. When this bit is reset, all 16 bits of the received VLAN frame's fifteenth and sixteenth bytes are used for comparison. |

Table 29-253: EMAC\_VLANTAG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VL         | VLAN Tag Id Receive Frames. The EMAC_VLANTAG.VL bits contain the 802.1Q VLAN tag to identify VLAN frames, and is compared to the fifteenth and sixteenth bytes of the frames being re- ceived for VLAN frames. Bits[15:13] are the User Priority, Bit[12] is the Canonical Format Indicator (CFI) and bits[11:0] are the VLAN tag's VLAN Identifier (VID) field. When the ETV bit is set, only the VID (Bits[11:0]) is used for comparison. If VL (VL[11:0] if ETV is set) is all zeros, the MAC does not check the fifteenth and sixteenth bytes for VLAN tag comparison, and declares all frames with a Type field val- ue of 0x8100 to be VLAN frames. |

## VLAN Hash Table Register

The EMAC\_VLAN\_HSHTBL register contains the VLAN hash table

Figure 29-222: EMAC\_VLAN\_HSHTBL Register Diagram

<!-- image -->

Table 29-254: EMAC\_VLAN\_HSHTBL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                         |
|--------------------|------------|-------------------------------------------------|
| 15:0               | VLHT       | VLAN Hash Table.                                |
| (R/W)              |            | This field contains the 16-bit VLAN Hash Table. |

## VLAN Tag Inclusion or Replacement Register

The EMAC\_VLAN\_INCL register contains the VLAN tag for insertion into or replacement in the transmit frames.

Figure 29-223: EMAC\_VLAN\_INCL Register Diagram

<!-- image -->

Table 29-255: EMAC\_VLAN\_INCL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | CSVL       | C-VLAN or S-VLAN. The EMAC_VLAN_INCL.CSVL bit, When this bit is set, S-VLAN type (0x88A8) is inserted or replaced in the 13th and 14th bytes of transmitted frames.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 18 (R/W)           | VLP        | VLAN Priority Control. The EMAC_VLAN_INCL.VLP bit, When this bit is set, the control Bits [17:16] are used for VLAN deletion, insertion, or replacement                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 17:16 (R/W)        | VLC        | VLAN Tag Control in Transmit Frames. The EMAC_VLAN_INCL.VLC bit, 2'b00:No VLAN tag deletion, insertion, or re- placement 2'b01:VLAN tag deletion. The MAC removes the VLAN type (bytes 13 and 14) and VLAN tag (bytes 15 and 16) of all transmitted frames with VLAN tags. 2'b10:VLAN tag insertion. The MAC inserts VLT in bytes 15 and 16 of the frame after inserting the Type value (0x8100/0x88a8) in bytes 13 and 14. This operation is performed on all transmitted frames, irrespective of whether they already have a VLAN tag. 2'b11:VLAN tag replacement. The MAC replaces VLT in bytes 15 and 16 of all VLAN-type transmitted frames (Bytes 13 and 14 are 0x8100/0x88a8). |
| 15:0 (R/W)         | VLT        | VLAN Tag for Transmit Frames. The EMAC_VLAN_INCL.VLT bit, contains the value of the VLAN tag to be insert- ed or replaced.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

## Watchdog Timeout Register

The EMAC\_WDOG\_TIMOUT register controls the watchdog timeout for received frames.

<!-- image -->

PWE (R/W)

Programmable Watchdog Enable

Figure 29-224: EMAC\_WDOG\_TIMOUT Register Diagram

Table 29-256: EMAC\_WDOG\_TIMOUT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | PWE        | Programmable Watchdog Enable. The EMAC_WDOG_TIMOUT.PWE bit, When this bit is set and Bit 23 (WD) of Reg- ister 0 (MAC Configuration Register) is reset, the WTOfield (Bits[13:0]) is used as watchdog timeout for a received frame.                                                                                                    |
| 13:0 (R/W)         | WTO        | Watchdog Timeout. The EMAC_WDOG_TIMOUT.WTO bit, When Bit 16 (PWE) is set and Bit 23 (WD) of Register 0 (MAC Configuration Register) is reset, this field is used as watchdog timeout for a received frame. If the length of a received frame exceeds the value of this field, such frame is terminated and declared as an error frame. |