# Ethernet Media Access Controller (EMAC) — EMAC Functional Description

<!-- source: 033_Ethernet_Media_Access_Controller_EMAC_EMAC_Functional_Descri.pdf | original pages 1864–2071 -->

Table 30-1: EMAC0/EMAC1 Feature Differences (Continued)

| Feature                                        | EMAC0   | EMAC0   | EMAC1   | EMAC1   |
|------------------------------------------------|---------|---------|---------|---------|
|                                                | SC594   | SC598   | SC594   | SC598   |
| TCP Segmentation Offloading for TCP/IP Packets | No      | Yes     | No      | No      |
| Split Header Feature                           | No      | Yes     | No      | No      |
| IPv4 ARP Offload                               | No      | Yes     | No      | No      |
| Energy Efficient Ethernet (EEE)                | Yes     | Yes     | Yes     | No      |

## EMAC Functional Description

This section provides information on the function of Ethernet MAC peripheral.

## Hardware for the Media Access Control protocol

This function allows applications to support TCP/IP based network communication. At the system end, the module supports direct connection with the system crossbar bus for memory or MMR transactions. It supports RMII (Reduced Media Independent Interface), RGMII (Reduced Gigabit Media Independent Interface), MII (Media Independent Interface), and SMI (Station Management Interface) for interfacing with the external PHY chip.

## Dedicated DMA Controller with independent read/write channels

Performs both data and status transfers between the application and the media independent interfaces. Internal transmit and receive FIFOs are used to buffer and regulate the frames. Dedicated interrupt lines connect the EMAC interrupt sources to the System Event Controller (SEC) for the SHARC core or Generic Interrupt Controller (GIC) for ARM core.

## MAC Management Counters (MMC) block

An extended set of registers that collect various statistics compliant with IEEE 802.3 definitions regarding the operation of the interface. The registers are updated for each new transmitted or received frame when the condition to update the counter is met. The EMAC provides a set of such counters, along with extended usage control.

## PTP (Precision Time Protocol) engine

Provides hardware assistance for the implementation of the IEEE 1588 version 1 and version 2 standards, which allows time synchronization between systems.

## Audio Video (AV) functionality

Enables transmission of time-sensitive traffic over bridged local area networks (LANs). The EMAC provides hardware support for IEEE 802.1-Qav specified credit-based shaper (CBS) algorithm. In addition, slot number function allows scheduling of fetching of data by DMA from system memory.

## Data Center Bridging (DCB)

Data center bridging (DCB) is a set of enhancements to the Ethernet local area network communication protocol for use in data center environments, for use with clustering and storage area networks. DCB aims, for selected traffic, to eliminate loss due to queue overflow (sometimes called lossless Ethernet) and to be able to allocate bandwidth on links. DCB enables, to some extent, the treatment of different priorities as if they were different pipes.

## Time Sensitive Networking

Time sensitive network (TSN) is a set of standards developed by the IEEE 802.1 work group. The standards define mechanisms for the time-sensitive transmission of data over Ethernet.

## ADSP-SC59x EMAC Register List

RGMII, RMII, MII Ethernet Controller (EMAC)

Table 30-2: ADSP-SC59x EMAC Register List

| Name                    | Description                                                       |
|-------------------------|-------------------------------------------------------------------|
| EMAC_DMA[n]_CTL         | DMAChannel n Control Register                                     |
| EMAC_DMA[n]_RXBUF_CUR   | DMAChannel n Current Application Receive Buffer Address Register  |
| EMAC_DMA[n]_RXDSC_CUR   | DMAChannel n Current Application Receive Descriptor Register      |
| EMAC_DMA[n]_TXBUF_CUR   | DMAChannel n Current Application Transmit Buffer Address Register |
| EMAC_DMA[n]_TXDSC_CUR   | DMAChannel n Current Application Transmit Descriptor Register     |
| EMAC_DMA[n]_IEN         | DMAChannel n Interrupt Enable Register                            |
| EMAC_DMA[n]_MISSFRM_CNT | DMAChannel n Missed Frame Counter Register                        |
| EMAC_DMA[n]_RXDSC_ADDR  | DMAChannel n Receive Descriptor List Address Register             |
| EMAC_DMA[n]_RXDSC_TLPTR | DMAChannel n Receive Descriptor Tail Pointer Register             |
| EMAC_DMA[n]_RXCTL       | DMAChannel n Receive Control Register                             |
| EMAC_DMA[n]_RXCTL2      | DMAChannel n Extended Receive Control Register                    |
| EMAC_DMA[n]_RXINTWDTMR  | DMAChannel n Receive Interrupt Watchdog Timer Register            |
| EMAC_DMA[n]_SFN_CTLSTAT | DMAChannel n Slot Function Control and Status register            |
| EMAC_DMA[n]_STAT        | DMAChannel n Status Register                                      |
| EMAC_DMA[n]_TXDSC_ADDR  | DMAChannel n Transmit Descriptor List Address Register            |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                    | Description                                            |
|-------------------------|--------------------------------------------------------|
| EMAC_DMA[n]_TXDSC_RLEN  | DMAChannel n Transmit Descriptor Ring Length Register  |
| EMAC_DMA[n]_TXDSC_TLPTR | DMAChannel n Transmit Descriptor Tail Pointer Register |
| EMAC_DMA[n]_TXCTL       | DMAChannel n Transmit Control Register                 |
| EMAC_DMA_DBG_STAT0      | DMADebug Status 0 Register                             |
| EMAC_DMA_DBG_STAT1      | DMADebug Status 1 Register                             |
| EMAC_DMA_DBG_STAT2      | DMADebug Status 2 Register                             |
| EMAC_DMA_ISTAT          | DMAInterrupt Status Register                           |
| EMAC_DMA_MODE           | DMAMode Register                                       |
| EMAC_DMA_SYSBMODE       | DMASystem Bus Mode Register                            |
| EMAC_DMA_TBS_CTL0       | DMATBS Control 0 Register                              |
| EMAC_DMA_TBS_CTL1       | DMATBS Control 1 Register                              |
| EMAC_DMA_TBS_CTL2       | DMATBS Control 2 Register                              |
| EMAC_DMA_TBS_CTL3       | DMATBS Control 3 Register                              |
| EMAC_ONEUS_TIC_CNT      | LPI One Microsecond Tic Counter Register               |
| EMAC_ARP_ADDR           | Address Resolution Protocol (ARP) Register             |
| EMAC_ADDR0_HI           | MAC Address 0 High Register                            |
| EMAC_ADDR0_LO           | MAC Address 0 Low Register                             |
| EMAC_ADDR[i]_HI         | MAC Address n High Register                            |
| EMAC_ADDR[i]_LO         | MAC Address n Low Register                             |
| EMAC_TM_AUX_CTL         | Auxiliary Time Stamp Control Register                  |
| EMAC_TM_AUXTS_NSEC      | Auxiliary Time Stamp Nanoseconds Register              |
| EMAC_TM_AUXTS_SEC       | Auxiliary Time Stamp Seconds Register                  |
| EMAC_CSR_SW_CTL         | CSR Software Control Register                          |
| EMAC_MAC_CFG            | MAC Configuration Register                             |
| EMAC_MAC_DBG            | Debug Register                                         |
| EMAC_SPLM_OFST_CFG      | MAC Split Mode and Offset Configuration Register       |
| EMAC_MAC_EXT_CFG        | MAC Extended Configuration Register                    |
| EMAC_FPE_CTLSTAT        | Frame Preemption Control and Status Register           |
| EMAC_HW_FTR_PSNT0       | Hardware Feature Presence Register 0                   |
| EMAC_HW_FTR_PSNT1       | Hardware Feature Presence Register 1                   |
| EMAC_HW_FTR_PSNT2       | Hardware Feature Presence Register 2                   |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                 | Description                                      |
|----------------------|--------------------------------------------------|
| EMAC_HW_FTR_PSNT3    | Hardware Feature Presence Register 3             |
| EMAC_HASHTBL_REG0    | Hash Table Register 0                            |
| EMAC_HASHTBL_REG1    | Hash Table Register 1                            |
| EMAC_HASHTBL_REG2    | Hash Table Register 2                            |
| EMAC_HASHTBL_REG3    | Hash Table Register 3                            |
| EMAC_HASHTBL_REG4    | Hash Table Register 4                            |
| EMAC_HASHTBL_REG5    | Hash Table Register 5                            |
| EMAC_HASHTBL_REG6    | Hash Table Register 6                            |
| EMAC_HASHTBL_REG7    | Hash Table Register 7                            |
| EMAC_INDR_ACC_CTL    | Indirect Access Control Register                 |
| EMAC_INDR_ACC_DAT    | Indirect Access Data Register                    |
| EMAC_INVLAN_INCL     | Inner VLAN Tag Inclusion or Replacement Register |
| EMAC_MAC_IEN         | MAC Interrupt Enable Register                    |
| EMAC_MAC_ISTAT       | MAC Interrupt Status Register                    |
| EMAC_L3L4_CTL7       | Layer 3 and Layer 4 Control Register             |
| EMAC_L3L4_CTL[i]     | Layer 3 and Layer 4 Control Register             |
| EMAC_LPI_CTLSTAT     | LPI Control and Status Register                  |
| EMAC_LPI_ENTR_TMR    | Transmit LPI Entry Timer Register                |
| EMAC_LPI_TMRSCTL     | LPI Timers Control Register                      |
| EMAC_L3_ADDR0_REG7   | Layer 3 Address 0 Register 7                     |
| EMAC_L3_ADDR0_REG[i] | Layer 3 Address 0 Register [i]                   |
| EMAC_L3_ADDR1_REG7   | Layer 3 Address 1 Register 7                     |
| EMAC_L3_ADDR1_REG[i] | Layer 3 Address 1 Register [i]                   |
| EMAC_L3_ADDR2_REG7   | Layer 3 Address 2 Register 7                     |
| EMAC_L3_ADDR2_REG[i] | Layer 3 Address 2 Register [i]                   |
| EMAC_L3_ADDR3_REG7   | Layer 3 Address 3 Register 7                     |
| EMAC_L3_ADDR3_REG[i] | Layer 3 Address 3 Register [i]                   |
| EMAC_L4_ADDR7        | Layer 4 Address Register [i]                     |
| EMAC_L4_ADDR[i]      | Layer 4 Address Register [i]                     |
| EMAC_LOG_MSG_INTVL   | PTP Log Message Interval Register                |
| EMAC_MDIO_ADDR       | MDIO Address Register                            |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                  | Description                               |
|-----------------------|-------------------------------------------|
| EMAC_MDIO_DATA        | MDIO Data Register                        |
| EMAC_PHYIF_CTLSTAT    | PHY Interface Control and Status Register |
| EMAC_PPS[n]_INTVL     | PPS n Interval Register                   |
| EMAC_PPS[n]_TGTM_NSEC | PPS n Target Time Nanoseconds Register    |
| EMAC_PPS[n]_TGTM_SEC  | PPS n Target Time Seconds Register        |
| EMAC_PPS[n]_WID       | PPS n Width Register                      |
| EMAC_PPS_CTL          | PPS Control Register                      |
| EMAC_PTO_CTL          | PTP Time Stamp Offload Control Register   |
| EMAC_MACPKT_FILT      | MAC Packet Filter Register                |
| EMAC_PRSNTM_UPDT      | Presentation Time Update Register         |
| EMAC_PRSNTM_NSEC      | Presentation Time Nanosecond Register     |
| EMAC_Q0_TXFLOW_CTL    | Queue 0 Transmit Flow Control Register    |
| EMAC_Q[i]_TXFLOW_CTL  | Queue [i] Transmit Flow Control Register  |
| EMAC_RXQ_CTL0         | Receive Queue Control 0 Register          |
| EMAC_RXQ_CTL1         | Receive Queue Control 1 Register          |
| EMAC_RXQ_CTL2         | Receive Queue Control 2 Register          |
| EMAC_RXQ_CTL3         | Receive Queue Control 3 Register          |
| EMAC_RXQ_CTL4         | Receive Queue Control 4 Register          |
| EMAC_RXFLOW_CTL       | Receive Flow Control Register             |
| EMAC_RXTX_STAT        | Receive Transmit Status Register          |
| EMAC_SRCPRT_IDNTY0    | Source Port Identity 0 Register           |
| EMAC_SRCPRT_IDNTY1    | Source Port Identity 1 Register           |
| EMAC_SRCPRT_IDNTY2    | Source Port Identity 2 Register           |
| EMAC_TM_SUBSEC        | Subsecond Increment Register              |
| EMAC_TM_HISEC         | System Time Higher Word Seconds Register  |
| EMAC_TM_NSEC          | System Time Nanoseconds Register          |
| EMAC_TM_NSECUPDT      | System Time Nanoseconds Update Register   |
| EMAC_TM_SEC           | System Time Seconds Register              |
| EMAC_TM_SECUPDT       | System Time Seconds Update Register       |
| EMAC_TM_ADDEND        | Time Stamp Addend Register                |
| EMAC_TM_CTL           | Time Stamp Control Register               |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                  | Description                                           |
|-----------------------|-------------------------------------------------------|
| EMAC_TM_EGASYM_CORR   | Time Stamp Egress Asymmetry Correction Register       |
| EMAC_TM_EGCORR_NSEC   | Time Stamp Egress Correction Nanosecond Register      |
| EMAC_TM_EGCORR_SNSEC  | Time Stamp Egress Correction Sub Nanosecond Register  |
| EMAC_TM_EG_LTNCY      | Time Stamp Egress Latency Register                    |
| EMAC_TM_INGASYM_CORR  | Time Stamp Ingress Asymmetry Correction Register      |
| EMAC_TM_INGCORR_NSEC  | Time Stamp Ingress Correction Nanosecond Register     |
| EMAC_TM_INGCORR_SNSEC | Time Stamp Ingress Correction Sub Nanosecond Register |
| EMAC_TM_ING_LTNCY     | Time Stamp Ingress Latency Register                   |
| EMAC_TM_STAT          | Time Stamp Status Register                            |
| EMAC_TXQ_PRTY_MAP0    | Transmit Queue Priority Mapping 0 Register            |
| EMAC_TXQ_PRTY_MAP1    | Transmit Queue Priority Mapping 1 Register            |
| EMAC_TXTMSTAT_NSEC    | Transmit Time Stamp Status Nanoseconds Register       |
| EMAC_TXTMSTAT_SEC     | Transmit Time Stamp Status Seconds Register           |
| EMAC_VLAN_HASHTBL     | VLAN Hash Table Register                              |
| EMAC_VLAN_INCL        | VLAN Tag Inclusion or Replacement Register            |
| EMAC_VLANTAG_CTL      | VLAN Tag Control Register                             |
| EMAC_VLANTAG_DAT      | VLAN Tag Indirect Access Data Register                |
| EMAC_WDT_TMOUT        | Watchdog Timeout Register                             |
| EMAC_MMC_CTRL         | MMCControl Register                                   |
| EMAC_MMC_FPE_RXINT    | MMCReceive FPE Interrupt Status Register              |
| EMAC_MMC_FPE_RXIMSK   | MMCReceive FPE Interrupt Mask Register                |
| EMAC_MMC_FPE_TXINT    | MMCTransmit FPE Interrupt Status Register             |
| EMAC_MMC_FPE_TXIMSK   | MMCTransmit FPE Interrupt Mask Register               |
| EMAC_MMC_IPC_RXINT    | MMCReceive IPC Interrupt Status Register              |
| EMAC_MMC_IPC_RXIMSK   | MMCReceive IPC Interrupt Mask Register                |
| EMAC_RXFPE_FRGMNT_CNT | MMCReceive FPE Fragment Counter Register              |
| EMAC_MMC_RXINT        | MMCReceive Interrupt Status Register                  |
| EMAC_MMC_RXIMSK       | MMCReceive Interrupt Mask Register                    |
| EMAC_RXPKT_ASERR_CNT  | MMCReceive FPE Packet Assembly Error Counter Register |
| EMAC_RXPKT_ASOK_CNT   | MMCReceive FPE Packet Assembly Okay Counter Register  |
| EMAC_RXPKT_SMDERR_CNT | MMCReceive FPE Packet SMD Error Counter Register      |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                   | Description                                                 |
|------------------------|-------------------------------------------------------------|
| EMAC_TXFPE_FRGMNT_CNT  | MMCTransmit FPE Packet Fragment Counter Register            |
| EMAC_TXHLDRQ_CNT       | MMCTransmit FPE Hold Request Counter Register               |
| EMAC_MMC_TXINT         | MMCTransmit Interrupt Status Register                       |
| EMAC_MMC_TXIMSK        | MMCTransmit Interrupt Mask Register                         |
| EMAC_EST_CTL           | EST Control Register                                        |
| EMAC_EST_EXT_CTL       | EST Extended Control Register                               |
| EMAC_EST_FRMSZ_CAP     | EST Frame Size Capture Register                             |
| EMAC_EST_FRMSZ_ERR     | EST Frame Size Error Register                               |
| EMAC_EST_GCL_CTL       | EST Gate Control List Control Register                      |
| EMAC_EST_GCL_DAT       | EST Gate Control List Data Register                         |
| EMAC_EST_IEN           | EST Interrupt Enable Register                               |
| EMAC_EST_SCH_ERR       | EST Scheduling Error Register                               |
| EMAC_EST_STAT          | EST Status Register                                         |
| EMAC_FPE_ADV           | Frame Preemption Advance Register                           |
| EMAC_FPE_CTRLSTS       | MTL Frame Preemption Control and Status Register            |
| EMAC_MTL_ISTAT         | MTL Interrupt Status Register                               |
| EMAC_MTL_OPMODE        | MTL Operation Mode Register                                 |
| EMAC_Q0_INT_CTLSTAT    | Queue 0 Interrupt Control and Status Register               |
| EMAC_Q[n]_INT_CTLSTAT  | Queue n Interrupt Control and Status Register               |
| EMAC_RQ0_CTL           | Receive Queue Control Register                              |
| EMAC_RQ0_DBG           | Receive Queue 0 Debug Register                              |
| EMAC_RQ0_MSPKTOF_CNT   | Receive Queue 0 Missed Packet and Overflow Counter Register |
| EMAC_RQ0_OPMODE        | Receive Queue 0 Operation Mode Register                     |
| EMAC_RQ[n]_CTL         | Receive Queue n Control Register                            |
| EMAC_RQ[n]_DBG         | Receive Queue n Debug Register                              |
| EMAC_RQ[n]_MSPKTOF_CNT | Receive Queue n Missed Packet and Overflow Counter Register |
| EMAC_RQ[n]_OPMODE      | Receive Queue n Operation Mode Register                     |
| EMAC_RXQ_DMA_MAP0      | Receive Queue to DMAChannel Mapping Register 0              |
| EMAC_RXQ_DMA_MAP1      | Receive Queue to DMAChannel Mapping Register 1              |
| EMAC_TBS_CTRL          | MTL Time Based Scheduling Control Register                  |
| EMAC_TQ0_DBG           | Transmit Queue 0 Debug Register                             |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                    | Description                                                   |
|-------------------------|---------------------------------------------------------------|
| EMAC_TQ0_ETS_STAT       | Transmit Queue 0 ETS Status Register                          |
| EMAC_TQ0_OPMODE         | Transmit Queue 0 Operation Mode Register                      |
| EMAC_TQ0_QTMWGT         | Transmit Queue 0 IdleSlopeCredit, Quantum or Weights Register |
| EMAC_TQ0_URFL           | Transmit Queue 0 Underflow Packet Counter Register            |
| EMAC_TQ[n]_DBG          | Transmit Queue n Debug Register                               |
| EMAC_TQ[n]_ETS_CTL      | Transmit Queue n ETS Control Register                         |
| EMAC_TQ[n]_ETS_STAT     | Transmit Queue n ETS Status Register                          |
| EMAC_TQ[n]_HICRDT       | Transmit Queue n CBS HiCredit Register                        |
| EMAC_TQ[n]_LOCRDT       | Transmit Queue n CBS LoCredit Register                        |
| EMAC_TQ[n]_OPMODE       | Transmit Queue n Operation Mode Register                      |
| EMAC_TQ[n]_QTMWGT       | Transmit Queue n IdleSlopeCredit, Quantum or Weights Register |
| EMAC_TQ[n]_SSCRDT       | Transmit Queue n SendSlopeCredit Register                     |
| EMAC_TQ[n]_URFL         | Transmit Queue n Underflow Packet Counter Register            |
| EMAC_RXICMP_ERR_OCNT    | Receive ICMP Error Octets Counter Register                    |
| EMAC_RXICMP_ERR_PCNT    | Receive ICMP Error Packets Counter Register                   |
| EMAC_RXICMP_OCNT_G      | Receive ICMP Good Octets Counter Register                     |
| EMAC_RXICMP_PCNT_G      | Receive ICMP Good Packets Counter Register                    |
| EMAC_RXIPV4_FRGMT_OCNT  | Receive IPv4 Fragmented Octets Counter Register               |
| EMAC_RXIPV4_FRGMT_PCNT  | Receive IPv4 Fragmented Packets Counter Register              |
| EMAC_RXIPV4_OCNT_G      | Receive IPv4 Good Octets Counter Register                     |
| EMAC_RXIPV4_PCNT_G      | Receive IPv4 Good Packets Counter Register                    |
| EMAC_RXIPV4_HDRERR_OCNT | Receive IPv4 Header Error Octets Counter Register             |
| EMAC_RXIPV4_HDRERR_PCNT | Receive IPv4 Header Error Packets Counter Register            |
| EMAC_RXIPV4_PYLD_OCNT   | Receive IPv4 Payload Octets Counter Register                  |
| EMAC_RXIPV4_PYLD_PCNT   | Receive IPv4 Payload Packets Counter Register                 |
| EMAC_RXIPV4_UDPCSD_OCNT | Receive IPv4 UDP Checksum Disable Octets Counter Register     |
| EMAC_RXIPV4_UDPCSD_PCNT | Receive IPv4 UDP Checksum Disable Packets Counter Register    |
| EMAC_RXIPV6_OCNT_G      | Receive IPv6 Good Octets Counter Register                     |
| EMAC_RXIPV6_PCNT_G      | Receive IPv6 Good Packets Counter Register                    |
| EMAC_RXIPV6_HDRERR_OCNT | Receive IPv6 Header Error Octets Counter Register             |
| EMAC_RXIPV6_HDRERR_PCNT | Receive IPv6 Header Error Packets Counter Register            |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                  | Description                                                     |
|-----------------------|-----------------------------------------------------------------|
| EMAC_RXIPV6_PYLD_OCNT | Receive IPv6 Payload Octets Counter Register                    |
| EMAC_RXIPV6_PYLD_PCNT | Receive IPv6 Payload Packets Counter Register                   |
| EMAC_RXTCP_ERR_OCNT   | Receive TCP Error Octets Counter Register                       |
| EMAC_RXTCP_ERR_PCNT   | Receive TCP Error Packets Counter Register                      |
| EMAC_RXTCP_OCNT_G     | Receive TCP Good Octets Counter Register                        |
| EMAC_RXTCP_PCNT_G     | Receive TCP Good Packets Counter Register                       |
| EMAC_RXUDP_ERR_OCNT   | Receive UDP Error Octets Counter Register                       |
| EMAC_RXUDP_ERR_PCNT   | Receive UDP Error Packets Counter Register                      |
| EMAC_RXUDP_OCNT_G     | Receive UDP Good Octets Counter Register                        |
| EMAC_RXUDP_PCNT_G     | Receive UDP Good Packets Counter Register                       |
| EMAC_RX1024TOMAX_GB   | Receive 1024 - Maximum Octets Packets Good Bad Counter Register |
| EMAC_RX128TO255_GB    | Receive 128 - 255 Octets Packets Good Bad Counter Register      |
| EMAC_RX256TO511_GB    | Receive 256 - 511 Octets Packets Good Bad Counter Register      |
| EMAC_RX512TO1023_GB   | Receive 512 - 1023 Octets Packets Good Bad Counter Register     |
| EMAC_RX64_GB          | Receive 64 Octets Packets Good Bad Counter Register             |
| EMAC_RX65TO127_GB     | Receive 65 - 127 Octets Packets Good Bad Counter Register       |
| EMAC_RX_ALGNERR_PCNT  | Receive Alignment Error Packets Counter Register                |
| EMAC_RX_BCERR_PCNT_G  | Receive Broadcast Packets Good Counter Register                 |
| EMAC_RX_CRCERR_PCNT   | Receive CRC Error Packets Counter Register                      |
| EMAC_RX_CTL_PCNT_G    | Receive Control Packets Good Counter Register                   |
| EMAC_RX_FIFOOVFL_PCNT | Receive FIFO Overflow Packets Counter Register                  |
| EMAC_RX_JBERR_PCNT    | Receive Jabber Error Packets Counter Register                   |
| EMAC_RX_LPITRAN_CNT   | Receive LPI Transition Counter Register                         |
| EMAC_RX_LPIUSEC_CNT   | Receive LPI Microseconds Counter Register                       |
| EMAC_RX_LENERR_PCNT   | Receive Length Error Packets Counter Register                   |
| EMAC_RX_MCST_PCNT_G   | Receive Multicast Packets Good Counter Register                 |
| EMAC_RX_OCNT_G        | Receive Octet Good Counter Register                             |
| EMAC_RX_OCNT_GB       | Receive Octet Good Bad Counter Register                         |
| EMAC_RX_ORTYPE_PCNT   | Receive Out of Range Type Packets Counter Register              |
| EMAC_RX_OVRSZ_PCNT    | Receive Oversize Packets Good Counter Register                  |
| EMAC_RX_PCNT_GB       | Receive Packets Good Bad Counter Register                       |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                  | Description                                                |
|-----------------------|------------------------------------------------------------|
| EMAC_RX_PAUSE_PCNT    | Receive Pause Packets Counter Register                     |
| EMAC_RX_ERR_PCNT      | Receive Error Packets Counter Register                     |
| EMAC_RX_RNTERR_PCNT   | Receive Runt Error Packets Counter Register                |
| EMAC_RX_UNDRSZ_PCNT_G | Receive Undersize Packets Good Counter Register            |
| EMAC_RX_UCST_PCNT_G   | Receive Unicast Packets Good Counter Register              |
| EMAC_RX_VLAN_PCNT_GB  | Receive VLAN Packets Good Bad Counter Register             |
| EMAC_RX_WDTERR_PCNT   | Receive Watchdog Error Packets Counter Register            |
| EMAC_TX1024TOMAX_GB   | Transmit 1024-Max Octets Packets Good Bad Counter Register |
| EMAC_TX128TO255_GB    | Transmit 128-255 Octets Packets Good Bad Counter Register  |
| EMAC_TX256TO511_GB    | Transmit 256-511 Octets Packets Good Bad Counter Register  |
| EMAC_TX512TO1023_GB   | Transmit 512-1023 Octets Packets Good Bad Counter Register |
| EMAC_TX64_GB          | Transmit 64 Octets Packets Good Bad Counter Register       |
| EMAC_TX65TO127_GB     | Transmit 65-127 Octets Packets Good Bad Counter Register   |
| EMAC_TX_BCST_PCNT_G   | Transmit Broadcast Packets Good Counter Register           |
| EMAC_TX_BCST_PCNT_GB  | Transmit Broadcast Packets Good Bad Counter Register       |
| EMAC_TX_CRERR_PCNT    | Transmit Carrier Error Packets Counter Register            |
| EMAC_TX_DFRD_PCNT     | Transmit Deferred Packets Counter Register                 |
| EMAC_TX_EXCOL_PCNT    | Transmit Excessive Collision Packets Counter Register      |
| EMAC_TX_EXDFRL_PCNT   | Transmit Excessive Deferral Error Counter Register         |
| EMAC_TX_LPITRAN_CNT   | Transmit LPI Transition Counter Register                   |
| EMAC_TX_LPIUSEC_CNT   | Transmit LPI Microseconds Counter Register                 |
| EMAC_TX_LTCOL_PCNT    | Transmit Late Collision Packets Counter Register           |
| EMAC_TX_MCST_PCNT_G   | Transmit Multicast Packets Good Counter Register           |
| EMAC_TX_MCST_PCNT_GB  | Transmit Multicast Packets Good Bad Counter Register       |
| EMAC_TX_MCOL_PCNT_G   | Transmit Multiple Collision Packets Good Counter Register  |
| EMAC_TX_OSIZE_PCNT_G  | Transmit Osize Packets Good Counter Register               |
| EMAC_TX_OCNT_G        | Transmit Octet Good Counter Register                       |
| EMAC_TX_OCNT_GB       | Transmit Octet Good Bad Counter Register                   |
| EMAC_TX_PCNT_G        | Transmit Packet Good Counter Register                      |
| EMAC_TX_PCNT_GB       | Transmit Packet Good Bad Counter Register                  |
| EMAC_TX_PAUSE_PCNT    | Transmit Pause Packets Counter Register                    |

Table 30-2: ADSP-SC59x EMAC Register List (Continued)

| Name                   | Description                                             |
|------------------------|---------------------------------------------------------|
| EMAC_TX_SNGLCOL_PCNT_G | Transmit Single Collision Packets Good Counter Register |
| EMAC_TX_URFL_PCNT      | Transmit Underflow Error Packets Counter Register       |
| EMAC_TX_UCST_PCNT_GB   | Transmit Unicast Packets Good Bad Counter Register      |
| EMAC_TX_VLAN_PCNT_G    | Transmit VLAN Packets Good Counter Register             |

## ADSP-SC59x EMAC Interrupt List

Table 30-3: ADSP-SC59x EMAC Interrupt List

|   Interrupt ID | Name          | Description               | Sensitivity   | DMAChannel   |
|----------------|---------------|---------------------------|---------------|--------------|
|             19 | EMAC0_DMA3_TX | DMA3_TX Channel Interrupt | None          |              |
|             20 | EMAC0_DMA4_TX | DMA4_TX Channel Interrupt | None          |              |
|             21 | EMAC0_DMA5_TX | DMA5_TX Channel Interrupt | None          |              |
|             22 | EMAC0_DMA6_TX | DMA6_TX Channel Interrupt | None          |              |
|             23 | EMAC0_DMA7_TX | DMA7_TX Channel Interrupt | None          |              |
|             24 | EMAC0_DMA0_RX | DMA0_RX Channel Interrupt | None          |              |
|             31 | EMAC0_DMA1_RX | DMA1_RX Channel Interrupt | None          |              |
|             32 | EMAC0_DMA2_RX | DMA2_RX Channel Interrupt | None          |              |
|             33 | EMAC0_DMA3_RX | DMA3_RX Channel Interrupt | None          |              |
|            162 | EMAC0_DMA4_RX | DMA4_RX Channel Interrupt | None          |              |
|            163 | EMAC0_DMA5_RX | DMA5_RX Channel Interrupt | None          |              |
|            164 | EMAC0_DMA6_RX | DMA6_RX Channel Interrupt | None          |              |
|            165 | EMAC0_DMA7_RX | DMA7_RX Channel Interrupt | None          |              |
|            221 | EMAC0_STAT    | Status                    | None          |              |
|            222 | EMAC0_PWR     | Power Interrupt           | None          |              |
|            223 | EMAC0_DMA0_TX | DMA0_TX Channel Interrupt | None          |              |
|            224 | EMAC0_DMA1_TX | DMA1_TX Channel Interrupt | None          |              |
|            225 | EMAC0_DMA2_TX | DMA2_TX Channel Interrupt | None          |              |
|            226 | EMAC1_MAC     | MAC Interrupt             | None          |              |
|            227 | EMAC0_STAT    | Status                    | None          |              |
|            228 | EMAC1_PWR     | Power Interrupt           | None          |              |
|            229 | EMAC1_DMA0_TX | DMA0_TX Channel Interrupt | None          |              |
|            230 | EMAC1_MAC     | MAC Interrupt             | None          |              |

Table 30-3: ADSP-SC59x EMAC Interrupt List (Continued)

|   Interrupt ID | Name          | Description               | Sensitivity   | DMAChannel   |
|----------------|---------------|---------------------------|---------------|--------------|
|            232 | EMAC1_DMA0_RX | DMA0_RX Channel Interrupt | None          |              |

## ADSP-SC59x EMAC Trigger List

Table 30-4: ADSP-SC59x EMAC Trigger List Generators

|   Trigger ID | Name                 | Description     | Sensitivity   |
|--------------|----------------------|-----------------|---------------|
|          180 | EMAC0_MCGR_DMA_RE Q0 | MCGR DMARequest | Edge          |
|          181 | EMAC0_MCGR_DMA_RE Q1 | MCGR DMARequest | Edge          |
|          182 | EMAC0_MCGR_DMA_RE Q2 | MCGR DMARequest | Edge          |
|          183 | EMAC0_MCGR_DMA_RE Q3 | MCGR DMARequest | Edge          |

Table 30-5: ADSP-SC59x EMAC Trigger List Receivers

|   Trigger ID | Name               | Description                | Sensitivity   |
|--------------|--------------------|----------------------------|---------------|
|          206 | EMAC0_DMA0_RX_STOP | DMARXChannel Start Trigger | Edge          |
|          207 | EMAC0_DMA1_RX_STOP | DMARXChannel Start Trigger | Edge          |
|          208 | EMAC0_DMA2_RX_STOP | DMARXChannel Start Trigger | Edge          |
|          209 | EMAC0_DMA3_RX_STOP | DMARXChannel Start Trigger | Edge          |
|          210 | EMAC0_DMA4_RX_STOP | DMARXChannel Start Trigger | Edge          |
|          211 | EMAC0_DMA5_RX_STOP | DMARXChannel Start Trigger | Edge          |
|          212 | EMAC0_DMA6_RX_STOP | DMARXChannel Start Trigger | Edge          |
|          213 | EMAC0_DMA7_RX_STOP | DMARXChannel Start Trigger | Edge          |
|          214 | EMAC0_DMA0_TX_STOP | DMATXChannel Stop Trigger  | Edge          |
|          215 | EMAC0_DMA1_TX_STOP | DMATXChannel Stop Trigger  | Edge          |
|          216 | EMAC0_DMA2_TX_STOP | DMATXChannel Stop Trigger  | Edge          |
|          217 | EMAC0_DMA3_TX_STOP | DMATXChannel Stop Trigger  | Edge          |
|          218 | EMAC0_DMA4_TX_STOP | DMATXChannel Stop Trigger  | Edge          |
|          219 | EMAC0_DMA5_TX_STOP | DMATXChannel Stop Trigger  | Edge          |
|          220 | EMAC0_DMA6_TX_STOP | DMATXChannel Stop Trigger  | Edge          |
|          221 | EMAC0_DMA7_TX_STOP | DMATXChannel Stop Trigger  | Edge          |

Table 30-5: ADSP-SC59x EMAC Trigger List Receivers (Continued)

|   Trigger ID | Name                 | Description               | Sensitivity   |
|--------------|----------------------|---------------------------|---------------|
|          222 | EMAC1_DMA0_RX_STOP   | DMARXChannel Stop Trigger | Edge          |
|          223 | EMAC1_DMA0_TX_STOP   | DMATXChannel Stop Trigger | Edge          |
|          224 | EMAC0_MCGR_DMA_AC K0 | MCGR DMAAcknowledge       | Edge          |
|          225 | EMAC0_MCGR_DMA_AC K1 | MCGR DMAAcknowledge       | Edge          |
|          226 | EMAC0_MCGR_DMA_AC K2 | MCGR DMAAcknowledge       | Edge          |
|          227 | EMAC0_MCGR_DMA_AC K3 | MCGR DMAAcknowledge       | Edge          |

## EMAC Definitions

The following definitions are helpful for using the EMAC.

## ARI

Application Receive Interface

## ATI

Application Transmit Interface

## CPT

Current Presentation Time

## DA

Destination Address

## DCB

Data Center Bridging

## EMAC CORE

CORE layer inside EMAC which performs the actual Ethernet operations, including interface with PHY through the reduced media interface(s).

## EMAC DMA

DMA Controller of EMAC

## EMAC MTL

MAC Transaction Layer inside EMAC

## EMAC SCB

System Crossbar Interface of EMAC

## EOP

End of Packet

## EST

Enhancement to Scheduled Traffic

## ETS

Enhanced Transmission Selection

## FPE

Frame Preemption

## ICMP

Internet Message Control Protocol

## MAC

Media Access Control

## MII

Media Independent Interface

## MMC

MAC Management Counter

## OST

One Step Time Stamp

## PTO

PTP Time Stamp Offload

## PTP

Precision Time Protocol

## LPI

Low Power Interface

## MDIO

Management Data Input/Output

## RGMII

Reduced Gigabit Media Independent Interface

## RMII

Reduced Media Independent Interface

## SA

Source Address

## SMI

Station Management Interface that controls PHY through MDIO and MDC signals.

## SOP

Start Of Packet

## TBS

Time Based Scheduling

## TPT

Target Presentation Time

## TSN

Time Sensitive Networking

## VLAN

Virtual LAN

## EMAC Block Diagram and Interfaces

The EMAC Simplified Block Diagram illustrates the overall functional architecture of the Ethernet MAC peripheral. The EMAC module is comprised of four major layers: EMAC SCB, EMAC DMA, EMAC MTL, and EMAC CORE. Each of these layers (sub blocks) is explained in depth in their respective sections in this chapter.

Figure 30-1: EMAC Simplified Block Diagram

<!-- image -->

A more comprehensive block diagram is shown in the EMAC Complete Block Diagram . It includes most of the important blocks inside the EMAC. The EMAC is connected to processor memory and the system crossbar through

the System Crossbar Bus Interface (SCB) and System Peripheral Bus Interface (SPB). These connections are which are part of the SCB layer. The SPB interface is connected to all modules that require MMR programming.

The DMA controller performs application data transfer frame by frame, through well-defined descriptor structures. A FIFO layer acts as a buffer between the DMA controller and EMAC CORE.

Figure 30-2: EMAC Complete Block Diagram

<!-- image -->

The EMAC System Level Block Diagram shows how the EMAC module interacts with rest of the system inside the processor.

Figure 30-3: EMAC System Level Block Diagram

<!-- image -->

The EMAC FIFO/Memory Details table shows details about various FIFO and memory instances used inside the EMAC module. Each SPRAM/DPRAM stores data along with parity bits generated by MEPU. The parity errors are handled by the Memory Error Controller (MEC) unit.

Table 30-6: EMAC FIFO/Memory Details

| EMAC In- stance   | FIFO/Memory Type       | Single (SPRAM)/ Dual Ported (DPRAM)   | Runs on   |   No. of Instan- ces | Size    |   No. of data bits |   No. of parity bits |
|-------------------|------------------------|---------------------------------------|-----------|----------------------|---------|--------------------|----------------------|
| EMAC0             | Rx/Tx FIFOs (Odd/Even) | SPRAM                                 | SCLK0     |                    4 | 2048x40 |                 35 |                    5 |
| EMAC0             | TSO Memory             | SPRAM                                 | SCLK0     |                    1 | 512x36  |                 32 |                    4 |
| EMAC0             | EST Memory             | SPRAM                                 | PTP Clock |                    1 |         |                    |                      |
| EMAC1             | Rx FIFO                | DPRAM                                 | SCLK0     |                    1 | 256x40  |                 35 |                    5 |
| EMAC1             | Tx FIFO                | DPRAM                                 | SCLK0     |                    1 | 128x40  |                 35 |                    5 |

## EMAC CORE Sub Blocks

The Core Transmit Engine Sub Blocks table summarizes the core transmit engine sub blocks and their functions. Refer to the EMAC CORE section for further explanation of each of these sub blocks.

Table 30-7: CORE Transmit Engine Sub Blocks

| CORE Transmit Engine Sub Block           | Function                                                                                                                                                                                                                                        |
|------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Transmit Bus Interface                   | Interface to the FIFO.                                                                                                                                                                                                                          |
| Transmit Packet Controller               | Appends Zero-PAD data, if required, for short frames. Appends CRC for frame checksum from the CRC generator.                                                                                                                                    |
| Transmit Protocol Controller             | Generates preamble and SFD, as per 802.3 protocol. Generates jam pattern in half-duplex mode, for collisions. Jabber timeout, for excessively large frames. Flow control for half-duplex mode (back pressure). Generates transmit frame status. |
| Transmit Scheduler                       | Maintains the inter-frame gap between two transmitted frames. Follows the truncated binary exponential back-off algorithm for half-duplex mode.                                                                                                 |
| Transmit CRC Generator                   | Generate CRC for the frame checksum field of the Ethernet frame.                                                                                                                                                                                |
| Transmit Flow Control                    | Receives the pause frame, appends the calculated CRC, and sends the frame to the protocol engine module.                                                                                                                                        |
| Transmit Checksum Offload Engine         | Supports checksum calculation and insertion in the transmit path, for IPV4/TCP/UDP/ICMP packets.                                                                                                                                                |
| TCP/IP Segmentation Offload (TSO) Engine | Useful in offloading the TCP segmentation functions to the hard- ware.                                                                                                                                                                          |
| UDP/IPv4 Fragment Offload (UFO) Engine   | Supports breaking a large UDP packet into smaller multiple packets for transmission.                                                                                                                                                            |

The Core Receive Engine Sub Blocks table summarizes the core receive engine sub blocks and their function. Refer to the EMAC CORE section for more information on each of these sub blocks.

Table 30-8: Core Receive Engine Sub Blocks

| CORE Receive Engine Sub block   | Functionality Overview                                                                                                                                                                                           |
|---------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Receive Protocol Engine         | Strips the incoming preamble and SFD. Checks for correct length or type field. Performs internal loop back, if necessary. Generates receive status. Supports watchdog of received frames. Supports jumbo frames. |
| Receive CRC Module              | Checks for CRC error, by comparing with FCS.                                                                                                                                                                     |

Table 30-8: Core Receive Engine Sub Blocks (Continued)

| CORE Receive Engine Sub block      | Functionality Overview                                                                                                                                                                                                                                      |
|------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Receive Packet Controller Module   | Packs incoming 8-bit input stream to 32-bit data internally. Performs frame filtering, for uni-cast, multi-cast, and broadcast frames. Attaches the calculated IP checksum input from checksum offload engine. Updates the receive status to bus interface. |
| Receive Flow Control Module        | Detects the receiving pause frame and pauses the frame transmission for the delay specified within the received pause frame. Works in full duplex mode.                                                                                                     |
| Receive IP Checksum Offload Engine | Calculates IPv4 header checksums and verify against the received IPv4 header checksums. Identifies a TCP, UDP, or ICMP payload in the received IP data- grams.                                                                                              |
| Receive Bus Interface Unit Module  | Interface to the FIFO.                                                                                                                                                                                                                                      |
| Address Filtering Module           | Filters destination and source address based on uni-cast, multi-cast, and broadcast frames. Provides CRC hash filtering.                                                                                                                                    |
| IPv4 ARP Offload Engine            | This feature allows the processing of the IPv4 ARP request packet in the receive path and generating corresponding ARP response packet in the transmit path                                                                                                 |

## EMAC PHY Interface

The EMAC can interface to the PHY through the RMII interface standard. The RMII Pins table shows the RMII pins available in the EMAC, in terms of their generic names. Refer to the data sheet for exact pin names.

Table 30-9: RMII Pins

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

Figure 30-4: RMII Di-bit Data Transfer

<!-- image -->

## RGMII Board Design Recommendations

Use the following guidelines during when performing board design when using the RGMII interface.

## MAC to PHY (Transmit)

The Ethernet MAC transmits data to the Ethernet PHY. The Ethernet MAC sends data with tskewT (the timing of TXC at the MAC) that meets the RGMII specification (tskewT = -500 ps to +500 ps skew window for transmitter to drive data). The RGMII specification requires that at the PHY end, tskewR is sampled at 1.0 to 2.6 ns. According to the RGMII standard, clocks must be routed such that an additional trace delay of greater than 1.5 ns and less than 2 ns is added to the associated clock signal.

To meet this standard without adding trace delays, most of the PHYs in the industry already include delay logic that can compensate for this on-board delay. These PHY types can manage a tskewR of ±500 ps (the skew for TXC data sampling seen inside the PHY).

The PHY or the on-board delay must delay the clock signal by 1.5 to 2.0 ns so that tskewR is sampled at 1.0 to 2.6 ns. The MAC to PHY Delay Diagram (Transmitting Data) figure shows where the delays must occur.

Figure 30-5: MAC to PHY Delay Diagram (Transmitting Data)

<!-- image -->

## PHY to MAC (Receive)

The Ethernet MAC receives data from the Ethernet PHY. Just as in the transmit case, a trace delay of greater than 1.5 ns and less than 2 ns must be added to the associated clock signal, as required by the RGMII specification. Also similar to the transmit case, most of the PHYs in the industry already include delay logic that can compensate for the RXC clock as well.

As shown in the MAC to PHY Delay Diagram (Receiving Data) figure, a board trace or the PHY can be used to generate the required 1.5-2.0 ns delay to RXC and the 1.0-2.6 ns tskewR.

Figure 30-6: MAC to PHY Delay Diagram (Receiving Data)

<!-- image -->

There following are two options for board design.

- The on-board delay must delay the clock signal by 1.5 to 2.0 ns. In this case, no additional delay must be introduced by the PHY.
- Most of the PHYs in the industry already include a mode that can introduce a delay logic. When this PHY mode is used, then the trace lengths on the board must be matched exactly.

The second option is recommended since it is easier to implement the delay by using the mode in the PHY rather than during the board design. This is the approach followed in the ADI EZ-Kits.

NOTE: Refer to the product specific data sheet for exact processor timing.

For the trace length recommendations for the EMAC signals, see the data sheet.

For the exact requirements as recommended by the RGMII protocol, see the RGMII specification.

## Clock Sources

The Ethernet MAC is clocked internally from CLKO7. Check the processor data sheet for the valid frequency range of the appropriate CLKO7 signal for Ethernet operation.

## EMAC0 Clock Sources

EMAC0 supports RGMII, RMII and MII interfaces. The external PHY sources a 2.5 MHz or 25 MHz clock (for 10/100 or gigabit Ethernet respectively) to operate the EMAC RXCLK in RGMII mode. The RGMII TXCLK is driven from CLK07 of the CDU (Clock Distribution Unit) and needs to be configured to 125 MHz regardless of the EMAC0 speeds (10/100/1000 Mbit/s). The EMAC\_MAC\_CFG.PS and EMAC\_MAC\_CFG.FES bits are used to divide the clocks down.

The following tables show the clock sources for the RGMII, MII and RMII interfaces.

<!-- image -->

Figure 30-7: EMAC Clock Sources - RGMII PHY Interface

Figure 30-8: EMAC Clock Sources - MII PHY Interface

<!-- image -->

Figure 30-9: EMAC Clock Sources - RMII PHY Interface

<!-- image -->

## EMAC Architectural Concepts

This section explains different architectural concepts relevant to EMAC peripheral, such as EMAC SCB, EMAC DMA, EMAC MTL, EMAC CORE, and others.

## EMAC System Crossbar Interface (EMAC SCB)

The EMAC SCB bus interface provides the bus connectivity to support highly effective throughput of data traffic. System bus use is maximized by allowing simultaneous read and write transfers initiated from different DMA channels. The EMAC controller connects directly to the SCB0 crossbar. The following interfaces are available with the design.

- A 32-bit SCB controller interface for reading and writing to and from the application memory.

- A 32-bit SPB target interface for register programming.

Refer to the 'System Crossbars (SCB)' chapter for more information on how the crossbar operates. This chapter details only the EMAC-specific information.

Table 30-10: EMAC-SCB Interface Data Transfer Specifications with Crossbar

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

The EMAC DMA Read/Write channels with System Crossbar figure shows DMA write channel and read channel data paths and their connection to the system crossbar.

Figure 30-10: EMAC DMA Read/Write channels with System Crossbar

<!-- image -->

NOTE: Transmit descriptor read and receive descriptor write-back (status update) operations can occur simultaneously. However, transmit descriptor read and write-back operations cannot occur simultaneously. T ransmit DMA (or receive DMA) does not initiate the next transfer unless the previous one is complete.

## Priority of SCB Requests

The descriptor transfers have higher priority than the data transfers. For example, if there are two bus requests, such as a receive descriptor read and a transmit data read, the receive descriptor read has a higher priority. The next receive data write (after the receive descriptor read) does not depend on the completion of the transmit data-read transfer.

If there are requests for descriptor reads from both DMA channels, they are serviced based on a first-come firstserve. Receive DMA has higher priority if the descriptor-read requests are generated from both the DMA channels in the same clock cycle. Similarly, in the write channel, descriptor writes from DMA have higher priority than the data-write transfers for the receive DMA.

When programs enable the AV or DCB feature, the DMA may contain multiple channels. If the requests are generated from different channels simultaneously in the same clock cycles, the request priority among the channels is: Channel 7 (if present), Channel 6, Channel 5, Channel 4, Channel 3, Channel 2, Channel 1, and Channel 0.

## SCB Interface Programming Options

The SCB bus interface supports the following programmable options for the EMAC module. These options are available using the EMAC\_DMA\_SYSBMODE and EMAC\_DMA[n]\_CTL registers.

- Outstanding transactions. The EMAC-SCB supports up to 16 (EMAC0) / 4 (EMAC1) outstanding read/write requests on the SCB bus. Software can control these requests by programming the EMAC\_DMA\_SYSBMODE.WR\_OSR\_LMT and EMAC\_DMA\_SYSBMODE.RD\_OSR\_LMT bits. Maximum outstanding requests = EMAC\_DMA\_SYSBMODE.WR\_OSR\_LMT + 1 or EMAC\_DMA\_SYSBMODE.RD\_OSR\_LMT + 1.
- Allowed burst sizes. The allowed burst sizes are 4 ( EMAC\_DMA\_SYSBMODE.BLEN4 ), 8 ( EMAC\_DMA\_SYSBMODE.BLEN8 ), 16 ( EMAC\_DMA\_SYSBMODE.BLEN16 ) and the SINGLE burst. The EMAC-SCB uses only those burst sizes configured by the program (through the EMAC\_DMA\_SYSBMODE register) for data transfer through the SCB bus. Data transfers are restricted to the maximum burst size from this list of programmed burst sizes.
- Burst splitting and burst selection. The EMAC-SCB splits the DMA requests into multiple bursts on the SCB system bus. Splitting is based on DMA count and software controllable burst enable bits ( EMAC\_DMA\_SYSBMODE.FB , BLEN16 , BLEN8 , BLEN4 ) as well as burst types (INCR and INCR\_ALIGNED). Burst types are also controllable through the software. Burst length select priority is in the sequence: FB, BLEN16, BLEN8, BLEN4

## · INCR burst type

- If the EMAC\_DMA\_SYSBMODE.FB bit is disabled, the EMAC-SCB always chooses the maximum allowed burst length based on the BLEN16 , BLEN8 and BLEN4 , bits. When the DMA requests are not multiples of the maximum allowed burst length, the SCB can choose a burst-length of any value less than the maximum enabled. (All lesser burst-length enables are redundant). For example, when length bits are enabled and the DMA requests a burst of 42 beats, the SCB splits it into three bursts of 16, 16 and 10 beats respectively.

- If EMAC\_DMA\_SYSBMODE.FB is enabled, then the burst length is based on the priority of the enabled bits in the following order: BLEN16 , BLEN8 and BLEN4 . When the DMA requests a burst transfer, the SCB interface splits the requested bursts into multiple transfers using only the enabled burst lengths. This splitting can occur when the requested burst is not a multiple of the maximum enabled burst. If it cannot choose any of the enabled burst lengths, then it selects the burst length as 1.

For example, the DMA requests a burst transfer of 42 beats, the SCB interface splits it into multiple bursts of size 16, 16, 8, 1 and 1 beats respectively. (In this case, the allowed burst sizes are enabled and the sequence is in decreasing burst sizes).

- INCR\_ALIGNED burst type. When the address-aligned burst-type is enabled ( EMAC\_DMA\_SYSBMODE.AAL ), the SCB interface splits the DMA requested bursts. The "INCR Burst Type" section explains burst splitting conditions further. Each burst-size aligns to the least significant bits of the start address. The SCB interface initially generates smaller bursts so that the remaining transfers move with the maximum (enabled) fixed burst lengths.

For example, in the same setting as explained earlier for the EMAC\_DMA\_SYSBMODE.FB bit disabled, the DMA requests a burst size of 42 beats at the start address of 0x000003A4 ( EMAC\_DMA\_SYSBMODE.BLEN4 , EMAC\_DMA\_SYSBMODE.BLEN8 and ( EMAC\_DMA\_SYSBMODE.BLEN16 are enabled). The SCB starts the first transfer with size 3 such that the address of the next burst is aligned (0x000003B0) for a burst of 16. Therefore, the sequence of bursts is 3, 16, 16, and 7, respectively.

When the EMAC\_DMA\_SYSBMODE.FB bit is enabled, then (having a start address of 0x000003A4 with 42 beats), the sequence of burst transfers is 1, 1, 1, 16, 16, 4, and 3 respectively. The sequence of smaller bursts at the beginning is used to align the address to the next higher enabled burst-lengths programmed in the register.

## Burst Reordering and Data Interleaving

The SCB protocol allows reordering of data transfers with different SCB IDs with respect to the sequence of requests. It also allows data interleaving between transfers of different SCB IDs for read channels. In the EMAC, the requests from each internal requester (receive DMA and transmit DMA) are generated with different SCB IDs. Therefore, reordering and data interleaving can be performed as two DMAs operate independently and are allocated different address space on the target memory.

The SCB requester supports reordering and interleaving from the SCB completer on the read channel. Each DMA internally ensures that a read request and a write request is never generated at the same time to the same address (it can occur for only descriptor accesses). The transmit DMA ensures that transmit descriptor reads, transmit data reads, and transmit descriptor write-back requests are generated only after the previous requests are complete. The receive DMA generates only descriptor reads on the read channel with a different ID. Therefore, data reordering and interleaving on the read channel does not cause any problems in the MAC.

## Posted Writes

In posted writes, the write channel of SCB requester transfers an OKAY response to the DMA as soon as the last cycle or beat of data is accepted by the SCB interconnect. The SCB requester sends this response without waiting for the response from the target SCB completer on the write response channel.

In non-posted writes, the SCB controller interface transfers the response received from the SCB target channel to the DMA. To transfer the Ethernet data to system memory, the receive DMA always issues posted write requests to the SCB controller interface. This enables pipelining of the data requests without delays.

The SCB controller does not support out-of-order write transfers. However, the receive DMA ensures that the sequence is maintained for descriptor writes, descriptor reads, and data transfers. For writing descriptors (status or time stamp), the DMA always issues the non-posted write requests. This is required because the transfer complete interrupt is generated based on the descriptor writes for which the DMA needs completion response from the memory target. This ensures no race condition between the hardware and the software because the interrupt is generated only after the data and descriptor are written to the target memory.

## SCB Bus Transaction Status

The SCB uses the EMAC\_DMA\_DBG\_STAT0.AXRHSTS and EMAC\_DMA\_DBG\_STAT0.AXWHSTS bits to indicate whether the channel is active or not.

## DMA Controller (EMAC DMA)

The DMA has independent transmit and receive engines, and a CSR (control and status register) space. The transmit engine transfers data from the system memory to the FIFO (MTL), whereas the receive engine transfers data from the FIFO (MTL) to the system memory. The DMA engine uses descriptors to efficiently move data from source to destination with minimal processor core intervention. The DMA is designed for packet-oriented data transfers such as packets in Ethernet. The DMA controller can be programmed to interrupt the application for situations such as packet transmit and receive transfer completion, and other normal or error conditions.

The DMA and the application communicate through two internal data structures:

1. Control and status registers (CSR).
2. Descriptor lists and data buffers

The EMAC supports the ring structure for DMA descriptor as shown in the following figure.

Figure 30-11: Descriptor Ring Structure

<!-- image -->

In the ring structure, descriptors are separated by the number programmed in the DSL field (32-bit unit) of the EMAC\_DMA[n]\_CTL registers. The application needs to program the total ring length, that is, the total number of descriptors in ring span in the following registers of a DMA channel:

- Transmit Descriptor Ring Length register ( EMAC\_DMA[n]\_TXDSC\_RLEN )
- Receive Descriptor Ring Length register ( EMAC\_DMA[n]\_RXCTL2.RDRL )

The EMAC\_DMA[n]\_TXDSC\_TLPTR or EMAC\_DMA[n]\_RXDSC\_TLPTR register contains the pointer to the descriptor address (N). The base address registers ( EMAC\_DMA[n]\_TXDSC\_ADDR and EMAC\_DMA[n]\_RXDSC\_ADDR ) and the current descriptor pointer registers ( EMAC\_DMA[n]\_TXDSC\_CUR or EMAC\_DMA[n]\_RXDSC\_CUR ) determine the address of the current descriptor that the DMA can process. The descriptors up to one location less than the one indicated by the descriptor tail pointer (N - 1) are owned by the DMA. The DMA continues to process the descriptors until the following condition occurs:

Current Descriptor Pointer == Descriptor Tail Pointer;

The DMA goes into the Suspend mode when this condition occurs. The application must perform a write to the Descriptor Tail pointer register and update the tail pointer so that the following condition is true:

Current Descriptor Pointer &lt; Descriptor Tail Pointer;

The DMA automatically wraps around the base address when the end of ring is reached, as shown in the DMA Descriptor Ring figure.

Figure 30-12: DMA Descriptor Ring

<!-- image -->

For descriptors owned by the application, the OWN bit of DES3 is reset to 0. For descriptors owned by the DMA, the OWN bit is set to 1. If the application has only one descriptor in the beginning, the application sets the last descriptor address (tail pointer) to Descriptor Base Address + 1. The DMA processes the first descriptor and then waits for the application to advance the tail pointer.

The DMA supports up to 8 transmit and 8 receive descriptor lists (or DMA channels). The descriptor lists reside in the physical memory address space of the application. Each descriptor can point to a maximum of two buffers in the

system memory. This enables two buffers to be used, physically addressed, rather than contiguous buffers in memory.

A data buffer resides in the application physical memory space and consists of an entire packet or part of a packet but cannot exceed a single packet. Buffers contain only data. Buffer status is maintained in the descriptor. Data chaining refers to packets that span multiple data buffers. However, a single descriptor cannot span multiple packets. The DMA skips to the data buffer of next packet when EOP (End of Packet) is detected.

For more information on the descriptor, see DMA Descriptors section that describes the descriptor structure and how the DMA accesses the descriptors.

## DMA Bursts Using the SCB Interface

The DMA engines attempt to transfer data in maximum size bursts that are programmed using the EMAC\_DMA[n]\_TXCTL and EMAC\_DMA[n]\_RXCTL bits. The receive and transmit descriptors are always accessed at the maximum possible burst length to read 16 bytes (limited by PBL or 32). The burst transfers initiated by the DMA can be split into multiple burst transfers as per the SCB protocol requirements and the settings of the EMAC\_DMA\_SYSBMODE register. The transmit DMA initiates a data transfer only when sufficient space is available in the MTL transmit queue to accommodate either of the following:

- Bytes corresponding to the configured burst (PBL x 4)
- Remaining bytes in the transmit Buffer without EOP
- Number of bytes until EOP

The receive DMA initiates a data transfer in the following conditions:

- Sufficient data is available in MTL receive queue to accommodate the configured burst
- EOP (when it is less than the configured burst length) is detected in the receive queue

The DMA indicates the start address and the number of transfers required to the SCB interface. When the SCB interface is configured for a fixed-length burst, it transfers the data by using the best combination of INCR4, INCR8, or INCR16 and SINGLE transactions. If EOP is reached before the fixed-burst ends on the SCB interface, dummy transfers are performed in-order to complete the fixed-burst. Otherwise, the DMA transfers the data using INCR (undefined length) and SINGLE transactions. When the SCB interface is configured for address-aligned beats, both DMA engines ensure that the first burst transfer initiated by the SCB is less than or equal to the size of the configured PBL. Therefore, all subsequent beats start at an address that is aligned to the configured PBL.

## Application Data Buffer Alignment

The transmit and receive data buffers do not have any restrictions on start address alignment. The start address for the buffers aligns to any of the 4 bytes. However, the DMA always initiates write transfers with address aligned to the bus width and dummy data (old data) in the invalid byte lanes. This typically happens during the transfer of the beginning or end of an Ethernet packet. The software driver should discard the dummy bytes based on the start address of the buffer and size of the packet.

## Example for Buffer Read

If the transmit buffer address is 0x0002 and 15 bytes must transfer, the DMA reads 5 full words (5 x 32-bit data) from address 0x0000. However, when transferring data to the EMAC transmit FIFO, the extra bytes (the first 2 bytes) are dropped or ignored. Similarly, the last 3 bytes of the last transfer are also ignored. The DMA always transfers a full 32-bit data to the transmit FIFO unless it is the end-of-frame.

## Example for Buffer Write

If the receive buffer address is 0x0002 and 15 bytes of a received frame must transfer, the DMA writes 5 full words (5 x 32-bit data) to address 0x0000. However, the first 2 bytes of first transfer and the last 3 bytes of the third transfer have dummy data.

## Buffer Size Calculations

The DMA engines do not update the size fields in the transmit and receive descriptors alone. The DMA updates only the status fields (RDES and TDES) of the descriptors. The driver must perform the size calculations.

The transmit DMA transfers the exact number of bytes (indicated by buffer size field of TDES2) towards the MAC. If a descriptor is marked as first (FD bit of TDES3 is set), the DMA marks the first transfer from the buffer as SOP . If a descriptor is marked as last (LD bit of TDES3), the DMA marks the last transfer from that data buffer as EOP to the MTL.

The receive DMA transfers data to a buffer until the buffer is full or the end of packet is received from the MTL. When the FD bit of a descriptor is set, the amount of valid data in a buffer is accurately indicated by the buffer size field (programmed in the EMAC\_DMA[n]\_RXCTL registers) minus the data buffer pointer offset. The offset is zero when the data buffer pointer is aligned to the data bus width (32 bits). If a descriptor is marked as last, the buffer may not be full (as indicated by the buffer size in Bits [14:1] of the EMAC\_DMA[n]\_RXCTL registers). T o compute the amount of valid data in this final buffer, the driver must:

- Read the packet length (PL bits of RDES3 [14:0]), and
- Subtract the sum of the buffer sizes of the preceding buffers in this packet

The receive DMA always transfers the start of next packet with a new descriptor.

## DMA Transmit Operation

The following sections describe how the transmission process works for direct memory access on the EMAC controller.

## Default (Non-OSF) Mode

The following sequence describes the default process for DMA transmit works.

1. The application sets up the transmit descriptor (TDES0-TDES3) and sets the Own bit (TDES3[31]) after setting up the corresponding data buffer(s) with Ethernet packet data.
2. The application advances the descriptor tail pointer offset value of the transmit channel.

3. While in the run state, the DMA runs an arbitration cycle to select the next transmit DMA channel from which the packets requiring transmission should be processed.
4. The DMA fetches the descriptor from the application memory.
5. If the DMA detects one of the following conditions, the transmission from that channel is suspended, the EMAC\_DMA[n]\_STAT.TBU and EMAC\_DMA[n]\_STAT.TEB bits are set, and the transmit engine proceeds to step 11:
- The descriptor is flagged as owned by the application (TDES3 [31] = 1'b0)
- The descriptor tail pointer is equal to the current descriptor pointer in ring descriptor list mode
- An error condition occurs
6. If the acquired descriptor is flagged as owned by DMA (TDES0 [31] = 1#b1), the DMA decodes the transmit data buffer address from the acquired descriptor.
7. The DMA fetches the transmit data from the application memory and transfers the data to the MTL for transmission.
8. If an Ethernet packet is stored over data buffers in multiple descriptors, the DMA closes the intermediate descriptor and fetches the next descriptor. Steps 3 through 7 are repeated until the end of- Ethernet-packet data is transferred to the MTL.
9. When packet transmission is complete, if the IEEE 1588 time stamp feature was enabled for the packet (as indicated in the transmit status), the time stamp value obtained from MTL is written to the transmit descriptor (TDES0 and TDES1) that contains the EOP buffer. The status information is written to this transmit descriptor (TDES3). The application now owns this descriptor because the Own bit is cleared during this step.

If time stamp feature is not enabled for this packet, the DMA does not alter the contents of TDES0 and TDES1.

- NOTE: By default, the transmit DMA places subsequent request after the current descriptor write request is completed to the descriptor memory. If posted descriptor write is enabled by setting the EMAC\_DMA\_MODE.DSPW bit, the transmit DMA places subsequent request after the current descriptor write request is placed without waiting for its completion, thereby improving the throughput.
10. The transmit interrupt ( EMAC\_DMA[n]\_STAT.TI ) bits are set after completing transmission of a packet that has interrupt on completion (TDES2[31]) set in its last descriptor. The DMA engine returns to step 3.
11. In the suspend state, the DMA tries to acquire the descriptor again (and thereby return to step 3). A poll demand command is triggered by writing any value to the EMAC\_DMA[n]\_TXDSC\_TLPTR registers when it receives a transmit poll demand and the underflow interrupt status bit is cleared. If the application stopped the DMA by clearing transmit interrupt bit, the DMA enters the stop state.
- NOTE: In non-OSF mode, it is not required for the driver/application to extract the packet status and release the descriptor for the packet, even though FD is set to 1. The driver must track the descriptor that has OWN = 0 and LD = 1, as 1; such a descriptor contains the transmission status for the packet.

## OSF Mode

While in the run state, the transmit process can simultaneously acquire two packets without closing the status descriptor of the first packet (if the EMAC\_DMA[n]\_TXCTL.OSF bit is set). As the transmit process finishes transferring the first frame, it immediately polls the transmit descriptor list for the second packet. If the second packet is valid, the transmit process transfers this frame before writing the status information of the first packet.

In OSF mode, the run state transmit DMA operates in the following sequence.

1. The DMA operates as described in steps 1-7 of default (non-OSF) mode
2. Without closing the previous last descriptor of the frame, the DMA fetches the next descriptor.
3. If the DMA owns the acquired descriptor, the DMA decodes the transmit buffer address in this descriptor. If the DMA does not own the descriptor, the DMA goes into suspend mode and skips to Step 7.
4. The DMA fetches the transmit packet from the system memory and transfers the packet to the MTL until the EOP data is transferred, closing the intermediate descriptors if this packet is split across multiple descriptors.
5. The DMA waits for the packet transmission status and time stamp of previous packet. When the status is available, the DMA writes the time stamp to TDES0 and TDES1 if such time stamp was captured (as indicated by a status bit). The DMA writes the status, with a cleared Own bit, to the corresponding TDES1, thus closing the descriptor.

If the time stamp feature is not enabled for the previous packet, the DMA does not alter the contents of TDES0 and TDES1.

6. If enabled, the transmit interrupt is set; the DMA fetches the next descriptor, and then proceeds to Step 3 (when status is normal). If the previous transmission status shows an underflow error, the DMA goes into suspend mode (Step 7).
7. In suspend mode, if a pending status and time stamp are received from the MTL, the DMA:
- Writes the time stamp (if enabled for the current packet) to TDES0 and TDES1
- Writes the status to the corresponding TDES1
- Sets relevant interrupts and returns to Suspend mode

If no status is pending and the application stopped the DMA by clearing the EMAC\_DMA[n]\_TXCTL.ST bits, the DMA enters the Stop state.

8. The DMA can exit suspend mode and enter the run state (goes to step 1 or step 2 depending on pending status) only after receiving a transmit poll demand in transmit descriptor tail pointer register of the corresponding channel.

NOTE: The DMA fetches the next descriptor before closing the current descriptor. Therefore, the descriptor ring length must be more than 2. A minimum descriptor length of 4 is recommended.

This indicates that descriptors up to this descriptor can be released for reuse by the driver/application for subsequent packets.

In OSF mode, all except the last descriptor is closed immediately. The last descriptor is closed after the packet is transmitted on the line. Therefore, to minimize the complexity, the transmission status and only the required control bits are updated in the pending last descriptor of the previous packet.

## Transmit Packet Processing

The transmit DMA engine expects that the data buffers contain complete Ethernet packets, excluding the preamble, pad bytes, and FCS fields. The DA, SA, and type/length fields contain valid data. When the transmit descriptor indicates that the MAC must disable CRC or PAD insertion, the buffer must have complete Ethernet packets (excluding preamble), including the CRC bytes.

Packets can be data-chained and can span several buffers. Packets must be delimited by the first descriptor (TDES3[29]) and the last descriptor (TDES3[28]). As transmission starts, the first descriptor must have TDES3[29] set. When this occurs, the packet data is transferred from the application buffer to the MTL transmit queue. Concurrently, iwhen the current packet has the last descriptor (TDES3[28]) clear, the transmit process attempts to acquire the next descriptor. The transmit process expects this descriptor to have TDES3[29] clear. When TDES3[28] is clear, it indicates an intermediary buffer. When TDES3[28] is set, it indicates the last buffer of the packet.

After the last buffer of the packet has been transmitted, the DMA writes back the final status information to the transmit descriptor 3 (TDES3) word of the descriptor that has the last descriptor bit set in transmit descriptor 3 (TDES3[28]). At this time, if interrupt on completion (TDES2[31]) is set, the EMAC\_DMA[n]\_STAT.TI bit is set, the next descriptor is fetched, and the process repeats. The actual packet transmission begins after either of the following:

- The MTL transmit queue has reached a programmable transmit threshold (programmed in the EMAC\_TQ0\_OPMODE.TTC bit field)
- A full packet is contained in the FIFO

Programs can also use the store-and-forward mode ( EMAC\_TQ0\_OPMODE.TSF bit is set). In this mode, descriptors are released (OWN bit in TDES0[31] clears) when the DMA finishes transferring the packet.

NOTE: To ensure proper transmission of a packet and the next packet, specify a non-zero buffer size for the transmit descriptor that has the last descriptor (TDES3[28]) set.

## Transmit Polling Suspended

Either of the following conditions suspends transmit polling:

- The DMA detects a descriptor owned by the application (TDES3[31] = 0). To resume, the driver must give descriptor ownership to the DMA and then issue a poll demand command by writing the EMAC\_DMA[n]\_TXDSC\_TLPTR registers. If the DMA goes into the SUSPEND state because of this condition, the EMAC\_DMA[n]\_STAT.NIS and EMAC\_DMA[n]\_STAT.TBU bits are set.
- A packet transmission is aborted when a transmit error is detected because of underflow.

The appropriate transmit descriptor 3 (TDES3) bit is set. When this condition occurs, the following bits are set and the information is written to transmit descriptor 0, causing the suspension:

- EMAC\_DMA[n]\_STAT.AIS
- Transmit Underflow bit of corresponding queue ( EMAC\_Q0\_INT\_CTLSTAT.TXUNFIS bits)

In all conditions, the position in the transmit list is retained. The retained position is that of the descriptor following the last descriptor closed by the DMA. The driver must explicitly issue a transmit poll demand command after rectifying the suspension cause.

## DMA Transmit Channel Arbitration

An arbiter provides access to multiple DMAs trying to access the Bus Interface Unit (BIU).

When there is a request in the transmit DMA, the DMA arbiter checks the type of the request: packet buffer fetch or descriptor fetch request. The descriptor fetch requests have higher priority than the buffer requests. Therefore, when there is a descriptor fetch request, the DMA arbiter acknowledges the DMA channel that is requesting for a descriptor fetch. If there is no descriptor fetch request, the arbiter looks for packet buffer fetch requests.

The DMA arbiter acknowledges the descriptor fetch request of one DMA channel at a time. Descriptor fetch requests are granted using a fixed priority with the higher channel having higher priority (channel 1 having priority over channel 0, channel 2 having priority over channel 1 and so on). For packet buffer fetches, the DMA arbiter uses the programmed channel weight and priority to decide which channel to acknowledge. The DMA arbiter performs a burst-by-burst arbitration based on one of the following algorithms, which can be selected by programming the EMAC\_DMA\_MODE.TAA bit field.

- Weighted Strict Priority (WSP). In WSP arbitration mode, the arbiter first processes channel 7 (or the last selected channel) followed by channel 6, channel 5, and so on. If a channel does not have a frame to transmit, the weight of the channel is reassigned to channel 7 (or the last selected channel). If channel 7 has no frames to transmit, the remaining weight is assigned to channel 6 and so on.
- Weighted Round Robin (WRR). In WRR arbitration mode, the arbiter first selects the channel with the highest weight programmed, and then the channel with next highest weight, and so on. If any channel does not have a frame to transmit, the weight of that channel is equally distributed to all channels that have frames to transmit. The weight of the channel is programmed in the EMAC\_DMA[n]\_TXCTL.TCW (transmit channel weight) bit fields.
- Fixed priority (FP). In fixed priority mode, channel 0 has the lowest priority and the last selected channel has the highest priority. The weight programmed in the TCW bits of a channel is ignored. In WSP or WRR arbitration, the channel weight corresponds to the number of DMA burst transfers for which the DMA arbiter grants the bus to a channel. When a channel completes all the DMA burst transfers, the arbiter grants the bus to the next channel.

In WSP or WRR arbitration, the channel weight corresponds to the number of DMA burst transfers for which the DMA arbiter grants the bus to a channel. When a channel completes all the DMA burst transfers, the arbiter grants the bus to the next channel.

## DMA Receive Operation

In the receive path, the DMA reads a packet from the MTL receive queue and writes it to the packet data buffers of the corresponding DMA channel. The ARI data at the start of the frame indicates the channel number to which the current frame must be written. If only one DMA channel is selected, this information is not provided.

The following list shows the reception sequence for the receive DMA engine:

1. The application sets up the receive descriptors (RDES0-RDES3) and the OWN bit (RDES3[31]). The application must set the correct value in the EMAC\_DMA[n]\_RXDSC\_TLPTR register.
2. When the EMAC\_DMA[n]\_RXCTL.SR bit is set, the DMA enters the run state. The DMA looks for free descriptors based on the EMAC\_DMA[n]\_RXDSC\_ADDR and EMAC\_DMA[n]\_RXDSC\_TLPTR register values. If there are no free descriptors, the DMA channel enters the suspend state and goes to step 11.
3. The DMA fetches the next available descriptor in the ring and decodes the receive data buffer address from the acquired descriptors.
4. If IEEE 1588 time stamping is enabled and the time stamp is available for the previous packet, the DMA writes the time stamp (if available) to the RDES0 and RDES1 registers of the current descriptor and sets the CTXT bit field (RDES3[30]).
5. The DMA processes the incoming packets and places these in the data buffers of acquired descriptor.
6. If the current packet transfer is not complete, the DMA closes the current descriptor as intermediate and goes to step 10.
7. The DMA takes the status of the receive frame from the MTL and writes the status word to current descriptor with the OWN bit cleared and the last descriptor bit set.
8. The DMA writes the frame length to RDES3 and VLAN tag to RDES0. The DMA also writes the MAC control frame opcode, OAM control frame code, and extended status information (if available) to RDES1 of the last descriptor.
9. If the IEEE 1588 time stamp feature is enabled, the DMA stores the time stamp (if available). The DMA writes the context descriptor after the last descriptor for the current packet (in the next available descriptor).
10. If more descriptors are available in the receive DMA descriptor ring, go to step 3; otherwise, go to the suspend state (step 11).
11. The receive DMA exits the suspend state when a receive poll demand is given, and the application advances the receive tail pointer register of a channel.

The engine proceeds to step 2 and refetches the next descriptor.

## Receive Descriptor Acquisition

The receive engine always attempts to acquire an extra descriptor in anticipation of an incoming packet. descriptor acquisition is attempted if any of the following conditions is satisfied:

- The EMAC\_DMA[n]\_RXCTL.SR bit is set immediately after being placed in the run state.

- The EMAC\_DMA[n]\_RXDSC\_TLPTR register value is ahead of the current descriptor acquired by the receive DMA.
- The controller has completed packet reception, but the current receive descriptor is not yet closed.
- A receive poll demand is issued (update of the tail pointer register).

## Receive Packet Processing

The sequence for processing a receive packet is as follows:

1. The MAC transfers the received packets to the MTL memory only if the packet passes the address filter. If the packet fails the address filtering, it is dropped in the MAC block (unless the EMAC\_MACPKT\_FILT.RA bit is set).
2. If the packet size is greater than or equal to the configurable threshold bytes set for the receive queue of the MTL, or when the complete packet is written to the queue in the store-and-forward mode, the MTL block requests the DMA block to begin transferring the packet data to the receive buffer pointed to by the current descriptor.

Because of collision or premature termination, packets smaller than 64 bytes are removed from the MTL receive queue.

3. When the DMA application interface (SCB or MDC) is ready, it transfers the data and sets the following:
- If the packet fits in a single descriptor, the DMA sets both last descriptor (RDES3[28]) and first descriptor (RDES3[29]).
- If the packets fits into more than one descriptor, the DMA sets the first descriptor (RDES3[29]) to delimit the packet.
4. The DMA releases the descriptors by resetting the OWN (RDES3[31]) bit to 1'b0, either because the receive is full or the last segment of the packet is transferred to the receive buffer. The received packet's status is updated in the last descriptor.
5. If the interrupt enabled on completion (RDES3[30]) bit is set in any of the descriptors between the first and last descriptor of the packet and the EMAC\_DMA[n]\_IEN.RIE bit is set, the DMA sets the EMAC\_DMA[n]\_STAT.RI bit.

The same process repeats unless the DMA encounters a descriptor flagged as being owned by the application or when there are no more descriptors in the ring. When the DMA finds a descriptor owned by the application and if the EMAC\_DMA[n]\_IEN.RBUE bits are set, the receive process sets the EMAC\_DMA[n]\_STAT.RBU bits and then enters the suspend state. The position in the receive list is retained.

## DMA Error Response

For any data transfer initiated by a DMA channel, if the completer replies with an error response, the DMA stops all operations and updates the error bits and the EMAC\_DMA[n]\_STAT.FBE bits. The application can either perform a reset to EMAC or re-initialize the DMA descriptor list and start again. The rest of the DMA channels are not affected by such errors.

## DMA Descriptors

The DMA in the EMAC transfers data based on a linked list of descriptors. The descriptor addresses must be aligned to the 32-bit bus width. The application creates the descriptors in the system memory. The EMAC supports the following two types of descriptors.

- Normal Descriptor: Normal descriptors are used for packet data and to provide control information applicable to the packets to be transmitted.
- Context Descriptor: Context descriptors are used to provide control information applicable to the packet to be transmitted. Each normal descriptor contains two buffers and two address pointers. These buffers enable the EMAC to be compatible with various types of memory management schemes.

NOTE: There is no limit for the number of descriptors that can be used for a single packet.

For basic details that describe the descriptor ring structure, see DMA Controller (EMAC DMA).

## Transmit Descriptor

The DMA in EMAC requires at least one descriptor for a transmit packet. In addition to two buffers, two bytecount buffers, and two address pointers, the transmit descriptor has control fields which can be used to control the MAC operation on per-transmit packet basis. The transmit normal descriptor has two formats: read format and write-back format.

## Transmit Normal Descriptor (Read Format)

The Transmit Normal Descriptor (Read Format) figure shows the read format for a transmit normal descriptor. The following tables describe the read format for the transmit normal descriptors: TDES0, TDES1, TDES2, and TDES3.

Figure 30-13: Transmit Normal Descriptor (Read Format)

<!-- image -->

Table 30-11: TDES0 Normal Descriptor (Read Format)

| Bit   | Name   | Description                                                                                                                                                                                                                 |
|-------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | BUF1AP | Buffer 1 Address Pointer or TSO Header Address Pointer. These bits indicate the physical ad- dress of buffer 1. These bits also indicate the TSO header address pointer when the following bits are set: • TSE bit of TDES3 |

Table 30-11: TDES0 Normal Descriptor (Read Format) (Continued)

| Bit   | Name   | Description       |
|-------|--------|-------------------|
|       |        | • FD bit of TDES3 |

Table 30-12: TDES1 Normal Descriptor (Read Format)

| Bit   | Name   | Description                                                                                                                                                                                   |
|-------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | BUF2AP | Buffer 2 or Buffer 1 Address Pointer. These bits indicate the physical address of buffer 2 when a descriptor ring structure is used. There is no limitation for the buffer address alignment. |

Table 30-13: TDES2 Normal Descriptor (Read Format)

| Bit   | Name      | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|-------|-----------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31    | IOC       | Interrupt on Completion. This bit controls the setting of TI and ETI status bits in the EMAC_DMA[n]_STAT register. When ETIC = 1 and TDES2[LD] = 0, this bit sets the ETI bit. When TDES3[LD] = 1, this bit sets the TI status bit.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 30    | TTSE/TMWD | Transmit Time stamp Enable or External TSO Memory Write Enable. This bit enables the IEEE1588 time stamping for the transmit packet referenced by the descriptor, if the TSE bit is not set. If the TSE bit is set and external TSO memory is enabled, setting this bit disables external TSO memory writing for this packet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 29:16 | B2L       | Buffer 2 Length. The driver sets this field. When set, this field indicates buffer 2 length.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 15:14 | VTIR      | VLAN Tag Insertion or Replacement. These bits request the MAC to perform VLAN tagging or untagging before transmitting the packets. The application must set the CRC pad control bits appropriately when VLAN tag insertion, Replacement, or Deletion is enabled for the pack- et. The following list describes the values of these bits: • 00 = Do not add a VLAN tag • 01 = Remove the VLAN tag from the packets before transmission. This option should be used only with the VLAN packets. • 10 = Insert a VLAN tag with the tag value programmed in the MAC_VLAN_Incl register or context descriptor • 11 = Replace the VLAN tag in packets with the tag value programmed in the MAC_VLAN_Incl register or context descriptor. This option should be used only with the VLAN packets. These bits are valid when the enable SA and VLAN insertion on transmit option is selected while configuring the core. |
| 13:0  | HL or B1L | Header Length or Buffer 1 Length. For header length only bits [9:0] are taken. The size 13:0 is applicable only when interpreting buffer 1 length. If the TCP segmentation offload feature is enabled through the TSE bit of TDES3, this field is equal to the header length. When the TSE bit is set in TDES3, the header length includes the length in bytes from the Ethernet source address until the end of the TCP header. The maxi- mum header length supported for the TSO feature is 1023 bytes. If the TCP segmentation offload feature is not enabled, this field is equal to buffer 1 length                                                                                                                                                                                                                                                                                                         |

Table 30-14: TDES3 Normal Descriptor (Read Format)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|-------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31    | OWN    | Ownership. When set, this bit indicates that the DMAowns the descriptor. When this bit is re- set, it indicates that the application owns the descriptor. The DMAclears this bit either when it completes the transfer of data given in the associated buffers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 30    | CTXT   | Context Type. This bit should be set to 1'b0 for normal descriptor.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 29    | FD     | First Descriptor. When set, this bit indicates that the buffer contains the first segment of a pack- et.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 28    | LD     | Last Descriptor. When this bit is set, it indicates that the buffer contains the last segment of the packet. When this bit is set, the B1L or B2L field should have a non-zero value.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 27:26 | CPC    | Disable CRC. When this bit is set, the EMAC does not append a cyclic redundancy check (CRC) to the end of the transmitted frame. • 00 = CRC and Pad Insertion. The MAC appends the cyclic redundancy check (CRC) at the end of the transmitted packet of length greater than or equal to 60 bytes. The MAC auto- matically appends padding and CRC to a packet with length less than 60 bytes. • 01 = CRC Insertion (Disable Pad Insertion). The MAC appends the CRC at the end of the transmitted packet but it does not append padding. The application should ensure that the padding bytes are present in the packet being transferred from the transmit buffer, that is, the packet being transferred from the transmit buffer is of length greater than or equal to 60 bytes. • 10 = Disable CRC Insertion. The MAC does not append the CRC at the end of the trans- mitted packet. The application should ensure that the padding and CRC bytes are present in the packet being transferred from the transmit buffer. • 11 = CRC Replacement. The MAC replaces the last four bytes of the transmitted packet with recalculated CRC bytes. The application should ensure that the padding and CRC bytes are present in the packet being transferred from the transmit buffer. This field is valid only for the first descriptor. Note: When the TSE bit is set, the MAC ignores this field because the CRC and pad insertion is always done for segmentation. |

Table 30-14: TDES3 Normal Descriptor (Read Format) (Continued)

| Bit   | Name           | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|-------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:23 | SAIC           | SA Insertion Control. When set, these bits request the MAC to add or replace the source address field in the Ethernet packet with the value given in the MAC address 0 register. The application must set the CRC pad control bits appropriately when SA insertion control is enabled for the packet. Bit 25 specifies the MAC address register (1 or 0) value that is used for source address insertion or replacement. Bits [24:23] encodings are as follows • 00 = Do not include the source address • 01 = Include or insert the source address. For reliable transmission, the application must pro- vide frames without source addresses. • 10 = Replace the source address. For reliable transmission, the application must provide frames without source addresses. • 11 = Reserved These bits are valid when the Enable SA and VLAN Insertion on Tx option is selected while con- figuring the core and when the First descriptor (FD) bit (TDES3 [29]) is set. |
| 22:19 | SLOTNUM or THL | SLOTNUM: Slot Number Control bits in AV mode. These bits indicate the slot interval in which the data should be fetched from the corresponding buffers addressed by TDES0 or TDES1. When the transmit descriptor is fetched, the DMAcompares the slot number value in this field with the slot interval maintained in the RSN field in the EMAC_DMA[n]_SFN_CTLSTAT reg- ister. Fetches the data from the buffers only if a value matches. These bits are valid only for the AV channels. THL: TCP/UDP Header Length. If the TSE bit is set, this field contains the length of the TCP/UDP header. The minimum value of this field must be 5 for TCP header. The value must be equal to 2 for UDP header. This field is valid only for the first descriptor                                                                                                                                                                                                               |
| 18    | TSE            | TCP Segmentation Enable. When this bit is set, the DMAperforms the TCP/UDP segmenta- tion or UDP fragmentation for a packet depending on the TSE_MODE[1:0] bit of the EMAC_DMA[n]_TXCTL register. This bit is valid only if the FD bit is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 17:16 | CIC/TPL        | Checksum Insertion Control or TCP Payload Length. These bits control the checksum calculation and insertion. Bit encodings are as follows: • 00 = Checksum insertion disabled. • 01 = Only IP header checksum calculation and insertion are enabled • 10 = IP header checksum and payload checksum calculation and insertion are enabled, but pseudo-header checksum is not calculated in hardware • 11 = IP header checksum and payload checksum calculation and insertion are enabled, and pseudo-header checksum is calculated in hardware When the TSE bit is set, this field contains the upper bits [17:16] of the TCP payload (or IP payload for UDP fragmentation). This allows the TCP/UDP packet length field to be spanned across TDES3[17:0] to provide 256 KB packet length support. This field is valid only for the first descriptor                                                                                                                      |

Table 30-14: TDES3 Normal Descriptor (Read Format) (Continued)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|-------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15    | TPL    | Reserved or TCP Payload Length. When the TSE bit is reset, this bit is reserved. When the TSE bit is set, this is bit 15 of the TCP payload length [17:0]. This field is valid only when the enable TCP segmentation offloading for TCP/IP packets option is selected while configuring the core                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14:0  | FL/TPL | Frame Length or TCP Payload Length. This field is equal to the length of the packet to be trans- mitted in bytes. When the TSE bit is not set, this field is equal to the total length of the packet to be transmitted: Ethernet Header Length + TCP /IP Header Length - Preamble Length - SFD Length + Ethernet Payload Length When the TSE bit is set, this field is equal to the lower 15 bits of the TCP payload length in case of segmentation and IP payload in case of UDP fragmentation. In case of segmentation, this length does not include Ethernet header or TCP/UDP/IP header length. In case of fragmenta- tion, this length does not include Ethernet header and IP header. When DWRR/WFQ algorithm is NOT enabled, value written into this field is not used when TSE = 0. |

## Transmit Normal Descriptor (Write-Back Format)

The write-back format of the transmit descriptor includes time stamp low, time stamp high, OWN, and status bits.

The write-back format is applicable only for the last descriptor of the corresponding packet. The LD bit (TDES3[28]) is set in the descriptor where the DMA writes back the status and time stamp information for the corresponding transmit packet.

The Transmit Normal Descriptor (Write-Back Format) figure illustrates the write-back format of the transmit descriptor.

Figure 30-14: Transmit Normal Descriptor (Write-Back Format)

<!-- image -->

## TDES0 Normal Descriptor (Write-Back Format)

As described in the following table, this format is only applicable to the last descriptor of a packet.

Table 30-15: TDES0 Normal Descriptor (Write-Back Format)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                           |
|-------|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | TTSL   | Transmit Packet Time Stamp Low. The DMAupdates this field with least significant 32 bits of the time stamp captured for the corresponding transmit packet. The DMAwrites the time stamp only if TTSE bit of TDES2 is set in the first descriptor of the packet. This field has the time stamp only if the last segment bit (LS) in the descriptor is set and the time stamp status (TTSS) bit is set. |

## TDES1 Normal Descriptor (Write-Back Format)

As described in the following table, this format is only applicable to the last descriptor of a packet.

Table 30-16: TDES1 Normal Descriptor (Write-Back Format)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                           |
|-------|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | TTSH   | Transmit Packet Time Stamp High. The DMAupdates this field with the most significant 32 bits of the time stamp captured for corresponding transmit packet. The DMAwrites the time stamp only if the TTSE bit of TDES2 is set in the first descriptor of the packet. This field has the time stamp only if the last segment bit (LS) in the descriptor is set and time stamp status (TTSS) bit is set. |

## TDES2 Normal Descriptor (Write-Back Format)

This format is only applicable to the last descriptor of a packet.

Table 30-17: TDES2 Normal Descriptor (Write-Back Format)

| Bit   | Name     |
|-------|----------|
| 31-0  | Reserved |

## TDES3 Normal Descriptor (Write-Back Format)

This format is applicable only to the last descriptor of a packet.

Table 30-18: TDES3 Normal Descriptor (Write-Back Format)

|   Bit | Name   | Description                                                                                                                                                                                                   |
|-------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|    31 | OWN    | Ownership. When this bit is set, it indicates that the EMAC DMAowns the descriptor. The DMAclears this bit when it completes the packet transmission. After the write-back is complete, this bit is set to 0. |
|    30 | CTXT   | Context Type. This bit should be cleared (=0) for normal descriptor.                                                                                                                                          |
|    29 | FD     | First Descriptor. When set, this bit indicates that the buffer contains the first segment of a pack- et.                                                                                                      |
|    28 | LD     | Last Descriptor. This bit is set (=1) for the last descriptor of a packet. The DMAwrites the status fields only in the last descriptor of the packet.                                                         |

Table 30-18: TDES3 Normal Descriptor (Write-Back Format) (Continued)

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|-------|----------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27-24 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 23    | DE       | Descriptor Error. When this bit is set, it indicates that the descriptor content is incorrect. The DMAsets this bit during write-back while closing the descriptor. Descriptor errors include: • Incorrect sequence from the context descriptor. For example, a location after the first descrip- tor for a packet. • All 1s • CTXT is set to 1 and the LD or FD bits are set to 1. NOTE: When a descriptor error occurs due to all 1s or CTXT, LD, and FD bits set to 1, the transmit DMAcloses the transmit descriptor with DE and LD bits set to 1. When the IOC bit in the TDES2 register of the corresponding first descriptor is set to 1, the transmit DMAsets the TI bit in the EMAC_DMA[n]_STAT register. NOTE: Based on the CTXT, LD, and FD bits of the transmit descriptor, the subsequent de- scriptor might be considered as the first descriptor (even if the FD bit is not set) and partial packet is sent. |
| 22-18 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 17    | TTSS     | Transmit Time Stamp Status. This status bit indicates that a time stamp has been captured for the corresponding transmit packet. When this bit is set, the TDES0 and TDES1 registers have time stamp values that were captured for the transmit packet. This field is valid only when the last segment control bit (TDES3 [28]) in a descriptor is set. This bit is valid only when the IEEE1588 time stamping feature is enabled; otherwise, it is reserved.                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 16    | EUE      | ECC Uncorrectable Error Status. Indicates an ECC uncorrectable error in the TSO memory. NOTE: An uncorrectable error in transmit FIFO memory is reported when the packet flush- ed bit (bit 13, FF) = 1. This is because all such packets are flushed by the EMAC module.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 15    | ES       | Error Summary. Indicates the logical OR of the following bits: TDES3[0]: IP Header Error DES3[14]: Jabber Timeout TDES3[13]: Packet Flush TDES3[12]: Payload Checksum Error TDES3[11]: Loss of Carrier TDES3[10]: No Carrier TDES3[9]: Late Collision TDES3[8]: Excessive Collision TDES3[3]: Excessive Deferral TDES3[2]: Underflow Error This bit is also set when the EUE (bit 16) is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 30-18: TDES3 Normal Descriptor (Write-Back Format) (Continued)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|-------|--------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14    | JT     | Jabber Timeout. This bit indicates that the MAC transmitter has experienced a jabber timeout. This bit is set only when the JD bit of the EMAC_MAC_CFG register is not set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 13    | FF     | Packet Flushed. This bit indicates that the DMAor MTL flushed the packet because of a soft- ware flush command given by the CPU.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 12    | PCE    | Payload Checksum Error. This bit indicates that the checksum offload engine had a failure and did not insert any checksum into the encapsulated TCP, UDP, or ICMP payload. This failure is caused by: • insufficient bytes in the payload, as indicated by the payload length field of the IP header. • the MTL starts to forward the packet to the MAC transmitter in store-and-forward mode without the checksum being calculated. This error condition only occurs when the transmit FIFO depth is less than the length of the Ethernet packet being transmitted to avoid dead- lock. In this case the MTL starts forwarding the packet when the FIFO is full, even in store- and-forward mode. • a bus error is detected during packet transfer. When the full checksum offload engine is not enabled, this bit is reserved. |
| 11    | LoC    | Loss of Carrier. This bit indicates that a loss of carrier occurred during packet transmission (that is, the gmii_crs_i signal was inactive for one or more transmit clock periods during packet trans- mission). This is valid only for the packets transmitted without collision and when the MAC op- erates in the half-duplex mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 10    | NC     | No Carrier. This bit indicates that the carrier sense signal form the PHY was not asserted during transmission.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 9     | LC     | Late Collision. This bit indicates that packet transmission was terminated because a collision oc- curred after the collision window (64 byte times including preamble in MII mode and 512 byte times including preamble and carrier extension in GMII mode). This bit is not valid if underflow error is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 8     | EC     | Excessive Collision. This bit indicates that the transmission was terminated after 16 successive collisions while attempting to transmit the current packet. If the DR bit is set in the MAC_Con- figuration register, this bit is set after first collision and the transmission of the packet is termina- ted.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 7-4   | CC     | Collision Count. This 4-bit counter value indicates the number of collisions that occurred before the packet was transmitted. The count is not valid when the EC bit is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 3     | ED     | Excessive Deferral. This bit indicates that the transmission ended because of excessive deferral of over 24,288 bit times (155,680 bits times in 1000 Mbps mode or jumbo packet enabled mode) if the DC bit is set in the MAC_Configuration register. When TBS is enabled in full duplex mode and this bit is set, it indicates that the frame has been dropped after the expiry time has reached.                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 30-18: TDES3 Normal Descriptor (Write-Back Format) (Continued)

|   Bit | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|-------|--------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|     2 | UF     | Underflow Error. This bit indicates that the MAC terminated the packet because the data arrived late from the system memory. The underflow error can occur because of either of the following conditions: • The DMAencountered an empty transmit buffer while transmitting the packet • The application filled the MTL transmit FIFO slower than the MAC transmit rate The transmission process enters the suspended state and sets the underflow bit corresponding to a queue in the EMAC_MTL_ISTAT register. |
|     1 | DB     | Deferred Bit. This bit indicates that the MAC deferred before transmitting because of presence of carrier. This bit is valid only in the half-duplex mode.                                                                                                                                                                                                                                                                                                                                                     |
|     0 | IHE    | IP Header Error. When this bit is set, it indicates that the checksum offload engine detected an IP header error. This bit is valid only when transmit checksum offload is enabled. Otherwise, it is reserved. If the COE detects an IP header error, it still inserts an IPv4 header checksum if the Ethernet type field indicates an IPv4 payload. In full duplex mode, when EST/Qbv is enabled and this bit is set, it indicates the frame drop status due to frame size error or schedule error.           |

## Transmit Context Descriptor

The context descriptor is used to provide the time stamps for one-step time stamp correction, VLAN tag ID for VLAN insertion feature.

The transmit context descriptor can be provided any time before a packet descriptor. The context is valid for the current packet and subsequent packets. The context descriptor is used to provide the time stamps for one-step time stamp correction and VLAN Tag ID for VLAN insertion feature. Write back is done on a context descriptor only to reset the OWN bit.

- NOTE: The VLAN Tag IDs and MSS values, provided by the application in a context descriptor with their corresponding valid bits set, are stored internally by the DMA. When the outer or inner VLAN tag is provided with the valid bit set, the DMA always passes the last valid VLAN tag to the MTL. The application cannot invalidate the valid VLAN tag stored by the DMA. The VLAN tag is inserted or replaced based on the control inputs provided for the packet.

The inner VLAN tag control input is used only for the next packet that immediately follows the context descriptor. The application must provide a context descriptor before the normal descriptor of each packet for which the DMA should use the inner VLAN Tag control input.

Figure 30-15: Transmit Context Descriptor Format

<!-- image -->

Table 30-19: TDES0 Context Descriptor

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                         |
|-------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | TTSL   | Transmit Packet Time Stamp Low. For one-step correction, the driver can provide the lower 32 bits of time stamp in this descriptor word. The DMAuses this value as the low word for doing one-step time stamp correction. This field is valid only if the OSTC and TCMSSV bits of TDES3 context descriptor are set. |

Table 30-20: TDES1 Context Descriptor

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                       |
|-------|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | TTSH   | Transmit Packet Time Stamp High. For one-step correction, the driver can provide the upper 32 bits of time stamp in this descriptor. The DMAuses this value as the high word for doing one- step time stamp correction. This field is valid only if the OSTC and TCMSSV bits of TDES3 context descriptor are set. |

Table 30-21: TDES2 Context Descriptor

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                                                  |
|-------|----------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-16 | IVT      | Inner VLAN Tag. When the IVLTV bit of the TDES3 context descriptor is set and the TCMSSV and OSTC bits of the TDES3 context descriptor are reset, TDES2[31:16] contains the inner VLAN tag that is inserted in the subsequent transmit packets.                                                                                                                              |
| 15-14 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                     |
| 13-0  | MSS      | Maximum Segment Size. When the enable TCP segmentation offloading for TCP/IP packets option is selected, the driver provides maximum segment size in this field. This segment size is used while segmenting the TCP/IP payload. This field is valid only if the TCMSSV bit of the TDES3 context descriptor is set and the OSTC bit of the TDES3 context descriptor is reset. |

Table 30-22: TDES3 Context Descriptor

|   Bit | Name   | Description                                                                                                                                                                                                         |
|-------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|    31 | OWN    | Own Bit. When this bit is set, it indicates that the EMAC DMAowns the descriptor. When this bit is reset, it indicates that the application owns the descriptor. The DMAclears this bit immediately after the read. |
|    30 | CTXT   | Context Type. This bit should be cleared (=0) for normal descriptor.                                                                                                                                                |

Table 30-22: TDES3 Context Descriptor (Continued)

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|-------|----------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29-28 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 27    | OSTC     | One-Step Time stamp Correction Enable. When this bit is set, the DMAperforms a one-step time stamp correction with reference to the time stamp values provided in TDES0 and TDES1.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 26    | TCMSSV   | One-Step Time stamp Correction Input or MSS Valid. When this bit and the OSTC bit are set, it indicates that the time stamp correction input provided in TDES0 and TDES1 is valid. When the OSTC bit is reset and this bit and the TSE bit of TDES3 are set in subsequent nor- mal descriptor, it indicates that the MSS input in TDES2 is valid.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 25-24 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 23    | CDE      | Context Descriptor Error. When this bit is set, it indicates that the descriptor content is incor- rect. The DMAsets this bit during write-back while closing the context descriptor. Context de- scriptor errors can be: • Incorrect sequence from the context descriptor. For example, a location before the first de- scriptor for a packet. • All 1s • CD, LD, and FD bits set to 1. NOTE: When the context descriptor error occurs due to all 1s or the CTXT, LD, and FD bits set to 1, the transmit DMAcloses the transmit descriptor with the DE and LD bits set to 1. When the IOC bit in TDES2 of the corresponding first descriptor is set to 1, transmit DMAsets the TI bit in the DMA_CH#_Status register. NOTE: Based on the CTXT, LD, and FD bits of the transmit descriptor, the subsequent de- scriptor might be considered as the first descriptor (even if the FD bit is not set) and a partial packet is sent. NOTE: This error is categorized as an abnormal event; recovery is only by issuing a software reset (DMA stopping-reconfiguring-restarting recovery mechanism is not suppor- ted). |
| 22-20 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 30-22: TDES3 Context Descriptor (Continued)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|-------|--------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19-18 | IVTIR  | Inner VLAN Tag Insert or Replace. When these bits are set, the MAC is requested to perform inner VLAN tagging or un-tagging before transmitting the packets. If the packet is modified for VLAN tags, the MAC automatically recalculates and replaces the CRC bytes. The following list describes the values of these bits: • 00 = Do not add the inner VLAN tag. • 01 = Remove the inner VLAN tag from the packets before transmission. This option should be used only with the VLAN frames. • 10 = Insert an inner VLAN tag with the tag value programmed in the MAC_In- ner_VLAN_Incl register or context descriptor. • 11 = Replace the inner VLAN tag in packets with the tag value programmed in the MAC_Inner_VLAN_Incl register or context descriptor. This option should be used only with the VLAN frames. These bits are valid when the Enable SA and VLAN Insertion on transmit and Enable Double VLAN Processing options are selected. |
| 17    | IVLTV  | Inner VLAN Tag Valid. When this bit is set, it indicates that the IVT field of TDES2 is valid.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 16    | VLTV   | VLAN Tag Valid. When this bit is set, it indicates that the VT field of TDES3 is valid.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 15-0  | VT     | VLAN Tag. This field contains the VLAN Tag to be inserted or replaced in the packet. This field is used as VLAN Tag only when the VLTI bit of the MAC_VLAN_Incl register is reset.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## Receive Descriptor

The DMA in the EMAC module attempts to read a descriptor only if the tail pointer is different from the base pointer or current pointer. It is recommended to have a descriptor ring with a length that can accommodate at least two complete packets received by the MAC. Otherwise, the performance of the DMA is impacted greatly because of the unavailability of the descriptors. In such situations, the receive FIFO in the MTL becomes full and starts dropping packets.

The following receive descriptors are present:

- Normal descriptors
- Context descriptors

All receive descriptors are prepared by the software and given to the DMA as 'Normal' descriptors with the content as shown in Receive Normal Descriptor (Read Format). The DMA reads this descriptor and after transferring a received packet (or part of) to the buffers indicated by the descriptor, the receive DMA closes the descriptor with the corresponding packet status. The format of this status is given in Receive Normal Descriptor (Write-Back Format).

For some packets, the normal descriptor bits are not enough to write the complete status. For such packets, the receive DMA writes the extended status to the next descriptor (without processing or using the buffers/pointers embedded in that descriptor). The format and content of the descriptor write back is described in Receive Context Descriptor.

## Receive Normal Descriptor (Read Format)

The read format for a receive normal descriptor is made up of a header or buffer 1 address, reserved field, payload or buffer 2 or next descriptor address, a 30-bit reserved filed, OWN bit, and an interrupt bit.

The Receive Normal Descriptor Read Format figure shows the read format for a receive normal descriptor.

Figure 30-16: Receive Normal Descriptor Read Format

<!-- image -->

## RDES0 Normal Descriptor (Read Format)

The following table explains the read format of the RDES0 normal descriptor.

Table 30-23: Receive Descriptor Fields 0 (RDES0)

| Bit   | Name    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|-------|---------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | BUFF1AP | Header or Buffer 1 Address Pointer. These bits indicate the physical address of buffer 1. There are no limit when the SPH bit of the control register of a channel is reset, these bits indicate the physical address of buffer 1. When the SPH bit is set, these bits indicate the physical address of header buffer, where the receive DMAwrites the L2/L3/L4 header bytes of the received packet. The application can program a byte-aligned address for this buffer which means that the LS bits of this field can be non-zero. However, while transferring the start of packet, the DMAperforms a write operation with RDES0[1:0] as zero. However, the packet data is shifted as per actual offset as given by buffer ad- dress pointer. If the address pointer points to a buffer where the middle or last part of the packet is stored, the DMAignores the offset address and writes to the full location as indicated by the data width. |

## RDES1 Normal Descriptor (Read Format)

The following table describes the read format of the RDES1 normal descriptor.

Table 30-24: Receive Descriptor Fields 1 (RDES1)

| Bit   | Name     | Description   |
|-------|----------|---------------|
| 31-0  | Reserved | Reserved      |

## RDES2 Normal Descriptor (Read Format)

The following table describes the read format of the RDES2 normal descriptor.

Table 30-25: Receive Descriptor Fields 2 (RDES2)

| Bit   | Name               | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|-------|--------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | Reserved or BUF1AP | Buffer 2 Address Pointer. These bits indicate the physical address of buffer 2. When the EMAC_DMA[n]_CTL.SPH bit is set, the buffer address pointer must be bus width- aligned, that is, RDES2[3:0, 2:0, or 1:0] = 0 corresponding to 128, 64, or 32 bus width. LSBs are ignored internally. When the EMAC_DMA[n]_CTL.SPH bit is reset, there is no limitations on the RDES2 value. However, the receive DMAuses the LS bits of the pointer address only while transferring the start bytes of a packet. If the BUF2AP is giving the address of a buffer in which the middle or last part of a packet is stored, the DMAignores BUF2AP and writes to the complete location. |

## RDES3 Normal Descriptor (Read Format)

The following table describes the read format of the RDES3 normal descriptor.

Table 30-26: Receive Descriptor Fields 3 (RDES3)

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                |
|-------|----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31    | OWN      | Own Bit. When this bit is set, it indicates that the EMAC module owns the descriptor. When this bit is reset, it indicates that the application owns the descriptor. The DMAclears this bit when either of the following conditions is true: • The DMAcompletes the packet reception • The buffers associated with the descriptor are full |
| 30    | IOC      | Interrupt Enabled on Completion. When this bit is set, an interrupt is issued to the application when the DMAcloses this descriptor.                                                                                                                                                                                                       |
| 29-26 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                   |
| 25    | BUF2V    | Buffer 2 Address Valid. When this bit is set, it indicates to the DMAthat the buffer 2 address speci- fied in RDES2 is valid. The application must set this bit so that the DMAcan use the address, to which the buffer 2 address in RDES2 is pointing, to write received packet data.                                                     |
| 24    | BUF1V    | Buffer 1 Address Valid. When set, this indicates to the DMAthat the buffer 1 address specified in RDES1 is valid. The application must set this value if the address pointed to by Buffer 1 address in RDES1 can be used by the DMAto write received packet data.                                                                          |
| 23-16 | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                   |
| 15-0  | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                   |

## Receive Normal Descriptor (Write-Back Format)

The Receive Normal Descriptor Write-Back Format figure illustrates the write-back format for a receive normal descriptor.

Figure 30-17: Receive Normal Descriptor Write-Back Format

<!-- image -->

## RDES0 Normal Descriptor (Write-Back Format)

The following table describes the write-back format for the RDES0 normal descriptor.

Table 30-27: RDES0 Normal Descriptor (Write-Back Format)

| Bit   | Name   | Description                                                                                                                                                                                           |
|-------|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-16 | IVT    | Inner VLAN Tag. This field contains the Inner VLAN tag of the received packet if the RS0V bit of RDES3 is set. This is valid only when double VLAN tag processing and VLAN tag stripping are enabled. |
| 15-0  | OVT    | Outer VLAN Tag. This field contains the outer VLAN tag of the received packet if the RS0V bit of RDES3 is set.                                                                                        |

## RDES1 Normal Descriptor (Write-Back Format)

The following table describes the write-back format for the RDES1 normal descriptor.

Table 30-28: RDES1 Normal Descriptor (Write-Back Format)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                               |
|-------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-16 | OPC    | OAMSubtype Code, or MAC Control Packet opcode. OAMsubtype code: If bits [18:16] of RDES3 are set to 3'b111, this field contains the OAMsubtype and code fields. MAC Control Packet opcode: The bits [15:8] of RDES3 contains the subtype and bits [7:0] contains the code.                                                                                |
| 15    | TD     | Time Stamp Dropped. This bit indicates that the time stamp was captured for this packet but was dropped in the MTL receive FIFO because of overflow. This bit is available only when the time stamp feature is selected. Otherwise, this bit is reserved.                                                                                                 |
| 14    | TSA    | Time Stamp Available. When time stamp is present, this bit indicates that the time stamp value is available in a context descriptor word 2 (RDES2) and word 1(RDES1). This is val- id only when the last descriptor bit (RDES3 [28]) is set. The context descriptor is written in the next descriptor just after the last normal descriptor for a packet. |

Table 30-28: RDES1 Normal Descriptor (Write-Back Format) (Continued)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|-------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13    | PV     | PTP Version. This bit indicates that the received PTP message has the IEEE 1588 version 2 format. When this bit is reset, it indicates the IEEE 1588 version 1 format. This bit is avail- able only when time stamp feature is selected. Otherwise, this bit is reserved.                                                                                                                                                                                                                                                                                                                                                                                     |
| 12    | PFT    | PTP Packet Type. This bit indicates that the PTP message is sent directly over Ethernet. This bit is available only when time stamp feature is selected. Otherwise, this bit is re- served.                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 11-8  | PMT    | PTP Message Type. These bits are encoded to give the type of the message received: • 0000 = No PTP message received • 0001 = SYNC (all clock types) • 0010 = Follow_Up (all clock types) • 0011 = Delay_Req (all clock types) • 0100 = Delay_Resp (all clock types) • 0101 = Pdelay_Req (in peer-to-peer transparent clock) • 0110 = Pdelay_Resp (in peer-to-peer transparent clock) • 0111 = Pdelay_Resp_Follow_Up (in peer-to-peer transparent clock) • 1000 = Announce • 1001 = Management • 1010 = Signaling • 1011-1110 = Reserved • 1111 = PTP packet with Reserved message type These bits are available only when the time stamp feature is selected. |
| 7     | IPCE   | IP Payload Error. When this bit is set, it indicates either of the following: • The 16-bit IP payload checksum (that is, the TCP, UDP, or ICMP checksum) calcula- ted by the MAC does not match the corresponding checksum field in the received seg- ment. • The TCP, UDP, or ICMP segment length does not match the payload length value in the IP Header field. • The TCP, UDP, or ICMP segment length is less than minimum allowed segment length for TCP, UDP, or ICMP. Bit 15 (ES) of RDES3 is not set when this bit is set.                                                                                                                            |
| 6     | IPCB   | IP Checksum Bypassed. This bit indicates that the checksum offload engine is bypassed. This bit is available when the enable receive TCP/IP checksum check feature is selected.                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 5     | IPV6   | IPv6 Header Present. This bit indicates that an IPV6 header is detected. When the enable split header feature option is selected and the SPH bit of control register of a channel is set, the IPV6 header is available in the header buffer area to which RDES0 is pointing.                                                                                                                                                                                                                                                                                                                                                                                  |

Table 30-28: RDES1 Normal Descriptor (Write-Back Format) (Continued)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|-------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4     | IPV4   | IPV4 Header Present. This bit indicates that an IPV4 header is detected. When the SPH bit of RDES3 is set, the IPV4 header is available in the header buffer area to which RDES0 is pointing.                                                                                                                                                                                                                                                                                                                                                                       |
| 3     | IPHE   | IP Header Error. When this bit is set, it indicates either of the following: • The 16-bit IPv4 header checksum calculated by the MAC does not match the received checksum bytes. • The IP datagram version is not consistent with the Ethernet type value. • Ethernet packet does not have the expected number of IP header bytes. This bit is valid when either bit 5 or bit 4 is set. This bit is available when the enable re- ceive TCP/IP checksum check feature is selected.                                                                                  |
| 2-0   | PT     | Payload Type. These bits indicate the type of payload encapsulated in the IP datagram pro- cessed by the Receive Checksum Offload Engine (COE): • 000: Unknown type or IP/AV payload not processed • 001 = UDP • 010 = TCP • 011 = ICMP • 110 = AV Tagged Data Packet • 111 = AV Tagged Control Packet • 101 = AV Untagged Control Packet • 100 = IGMP if IPV4 Header Present bit is set else DCB (LLDP) Control Packet If the COE does not process the payload of an IP datagram because there is an IP header error or fragmented IP , it sets these bits to 000. |

## RDES2 Normal Descriptor (Write-Back Format)

The following table describes the write-back format for the RDES2 normal descriptor.

Table 30-29: RDES2 Normal Descriptor (Write-Back Format)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|-------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-29 | L3L4FM | Layer 3 and Layer 4 Filter Number Matched. These bits indicate the number of the layer 3 and layer 4 filter that matched the received packet: • 000 = Filter 0 • 001 = Filter 1 • 010 = Filter 2 • 011 = Filter 3 • 100 = Filter 4 • 101 = Filter 5 • 110 = Filter 6 • 111 = Filter 7 This field is valid only when bit 28 or bit 27 is set high. When more than one filter match- es, these bits give the number of lowest filter. Note: This status is not available when flexi- ble RX parser is enabled.                                  |
| 28    | L4FM   | Layer 4 Filter Match. When this bit is set, it indicates that the received packet matches one of the enabled layer 4 port number fields. This status is given only when one of the follow- ing conditions is true: • Layer 3 fields are not enabled and all enabled layer 4 fields match • All enabled layer 3 and layer 4 filter fields match When more than one filter matches, this bit gives the layer 4 filter status of filter indicated by bits[31-29]. Note: This status is not available when the flexible receive parser is enabled |
| 27    | L3FM   | Layer 3 Filter Match. When this bit is set, it indicates that the received packet matches one of the enabled layer 4 port number fields. This status is given only when one of the follow- ing conditions is true: • All enabled layer 3 fields match and all enabled layer 4 fields are bypassed • All enabled filter fields match When more than one filter matches, this bit gives the layer 4 filter status of filter indicated by bits[31-29]. Note: This status is not available when the flexible receive parser is enabled            |
| 26-19 | MADRM  | MAC Address Match or Hash Value. When the HF bit is reset, this field contains the MAC address register number that matched the Destination address of the received packet. This field is valid only if the DAF bit is reset. When the HF bit is set, this field contains the hash value computed by the MAC. A packet passes the hash filter when the bit corresponding to the hash value is set in the hash filter register. Note: This status is not available when the flexible receive parser is enabled                                 |
| 18    | HF     | Hash Filter Status. When this bit is set, it indicates that the packet passed the MAC address hash filter. Bits[26:19] indicate the hash value. Note: This status is not available when the flexible receive parser is enabled.                                                                                                                                                                                                                                                                                                               |

Table 30-29: RDES2 Normal Descriptor (Write-Back Format) (Continued)

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|-------|----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17    | DAF/RXPI | Destination Address Filter Fail. When flexible RX parser is disabled, and this bit is set, it indicates that the packet failed the DA filter in the MAC. When flexible RX parser is enabled, this bit is set to indicate that the packet parsing is in- complete (RXPI) due to ECC error. Note: When this bit is set, ES bit of RDES3 is also set.                                                                                                                                                                       |
| 16    | SAF/RXPD | SA Address Filter Fail. When the flexible receive parser is disabled, and this bit is set, it in- dicates that the packet failed the SA filter in the MAC. When the flexible receive parser is enabled, this bit is set to indicate that the packet is dropped (RXPD) by the parser. Note: When this bit is set, ES bit of RDES3 is also set.                                                                                                                                                                            |
| 15    | OTS      | Outer VLAN Tag Filter Status. This bit is valid for both single and double VLAN tagged frames.                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 14    | ITS      | Inner VLAN Tag Filter Status (ITS). This bit is valid only for double VLAN tagged frames, when double VLAN processing is enabled. For more information, see the Filter Status top- ic.                                                                                                                                                                                                                                                                                                                                   |
| 13-11 | RxParser | Rx Parser • 000 = Rx Parser filter passed • 001 = Packet rejected in Rx Parser • 010 = Rx Parser overflow errors • 011 = Incomplete Rx parsing of the packet • 100 = AF = 1, RF = 1, Bypass • 101 = RA = 1 and L2 filter fails (Rx parser bypass) • 110 = Reserved • 111 = Incomplete parsing due to ECC error                                                                                                                                                                                                           |
| 10    | ARPNR    | ARP Reply Not Generated. When this bit is set, it indicates that the MAC did not generate the ARP reply for received ARP request packet. This bit is set when the MAC is busy trans- mitting ARP reply to earlier ARP request (only one ARP request is processed at a time). This bit is reserved when the Enable IPv4 ARP Offload option is not selected.                                                                                                                                                               |
| 9-0   | HL       | L3/L4 Header Length. This field contains the length of the header of the packet split by the MAC at L3 or L4 header boundary as identified by the MAC receiver. This field is val- id only when the first descriptor bit is set (FD = 1). The header data is written to the buffer 1 address of corresponding descriptor. If header length is zero, this field is not valid. It implies that the MAC did not identify and split the header. This field is valid when the enable split header feature option is selected. |

## RDES3 Normal Descriptor (Write-Back Format)

The following table describes the write-back format for the RDES3 normal descriptor.

Table 30-30: RDES3 Normal Descriptor (Write-Back Format)

|   Bit | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|-------|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|    31 | OWN    | Ownership. When this bit is set, it indicates that the EMAC DMAowns the descriptor. When this bit is reset, it indicates that the application owns the descriptor. The DMAclears this bit when either of the following conditions is true: • The DMAcompletes the packet reception • The buffers associated with the descriptor are full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|    30 | CTXT   | Receive Context Descriptor. When this bit is set, it indicates that the current descriptor is a con- text type descriptor. The DMAwrites 1'b0 to this bit for normal receive descriptor. When CTXT and FD bits are used together, {CTXT, FD} • 00 = Intermediate Descriptor • 01 = First Descriptor • 10 = Reserved • 11 = Descriptor Error (due to all 1s) NOTE: When a descriptor error occurs, the receive DMAcloses the receive descriptor indi- cating descriptor error. This receive descriptor is skipped and the buffer addresses are not used to write the packet data. Receive DMAsets the CDE field of the EMAC_DMA[n]_STAT register but does not the RI field even when IOC field is set, as this is not marked as last receive descriptor for the packet. The subsequent valid receive descriptor is used to write the packet data. |
|    29 | FD     | First Descriptor. When this bit is set, it indicates that this descriptor contains the first buffer of the packet. If the size of the first buffer is 0, the second buffer contains the beginning of the packet. If the size of the second buffer is also 0, the next descriptor contains the beginning of the packet. See the CTXT bit description for details of using the CTXT bit and FD bit together.                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|    28 | LD     | Last Descriptor. When this bit is set, it indicates that the buffers to which this descriptor is point- ing are the last buffers of the packet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|    27 | RS2V   | Receive Status RDES2 Valid. When this bit is set, it indicates that the status in RDES2 is valid and it is written by the DMA. This bit is valid only when the LD bit of RDES3 is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|    26 | RS1V   | Receive Status RDES1 Valid. When this bit is set, it indicates that the status in RDES1 is valid and it is written by the DMA. This bit is valid only when the LD bit of RDES3 is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|    25 | RS0V   | Receive Status RDES0 Valid. When this bit is set, it indicates that the status in RDES0 is valid and it is written by the DMA. This bit is valid only when the LD bit of RDES3 is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|    24 | CE     | CRC Error. When this bit is set, it indicates that a Cyclic Redundancy Check (CRC) Error oc- curred on the received packet. This field is valid only when the LD bit of RDES3 is set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|    23 | GP     | Giant Packet. When this bit is set, it indicates that the packet length exceeds the specified maxi- mum Ethernet size of 1518, 1522, or 2000 bytes (9018 or 9022 bytes if jumbo packet enable is set). Note: Giant packet indicates only the packet length. It does not cause any packet truncation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|    22 | RWT    | Receive Watchdog Timeout When this bit is set, it indicates that the Receive Watchdog Timer has expired while receiving the current packet. The current packet is truncated after watchdog timeout.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 30-30: RDES3 Normal Descriptor (Write-Back Format) (Continued)

| Bit   | Name   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|-------|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21    | OE     | Overflow Error. When this bit is set, it indicates that the received packet is damaged because of buffer overflow in Rx FIFO. Note: This bit is set only when the DMAtransfers a partial packet to the application. This happens only when the Rx FIFO is operating in the threshold mode. In the store-and-forward mode, all partial packets are dropped completely in Rx FIFO.                                                                                                                                                                                                                                               |
| 20    | RE     | Receive Error. When this bit is set, it indicates that the gmii_rxer_i signal is asserted while the gmii_rxdv_i signal is asserted during packet reception. This error also includes carrier extension error in the GMII and half-duplex mode. Error can be of less or no extension, or error (rxd!= 0f) during extension.                                                                                                                                                                                                                                                                                                     |
| 19    | DE     | Dribble Bit Error. When this bit is set, it indicates that the received packet has a non-integer multiple of bytes (odd nibbles). This bit is valid only in the MII Mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 18-16 | LT     | Length/Type Field. This field indicates if the packet received is a length packet or a type packet. The encoding of the 3 bits is as follows: • 000 = The packet is a length packet • 001 = The packet is a type packet • 011 = The packet is a ARP Request packet type • 100 = The packet is a type packet with VLAN Tag • 101 = The packet is a type packet with Double VLAN Tag • 110 = The packet is a MAC Control packet type • 111 = The packet is a OAMpacket type • 010 = Reserved                                                                                                                                     |
| 15    | ES     | Error Summary. When this bit is set, it indicates the logical OR of the following bits: • RDES3[24]: CRC Error • RDES3[19]: Dribble Error • RDES3[20]: Receive Error • RDES3[22]: Watchdog Timeout • RDES3[21]: Overflow Error • RDES3[23]: Giant Packet • RDES2[17]: Destination Address Filter Fail, when flexible RX parser is enabled • RDES2[16]: SA Address Filter Fail, when flexible RX parser is enabled This field is valid only when the LD bit of RDES3 is set.                                                                                                                                                    |
| 14-0  | PL     | Packet Length. These bits indicate the byte length of the received packet that was transferred to system memory (including CRC). This field is valid when the LD bit of RDES3 is set and Overflow Error bits are reset. The packet length also includes the two bytes appended to the Ethernet packet when IP checksum calcula- tion is enabled and the received packet is not a MAC control packet. This field is valid when the LD bit of RDES3 is set. When the last descriptor and error summary bits are not set, this field indicates the accumulated number of bytes that have been transferred for the current packet. |

## Receive Context Descriptor

This descriptor is read-only for the application. Only the DMA can write to this descriptor. The context descriptor provides information about the extended status related to the last received packet. The bit 30 of RDES3 indicates the context type descriptor.

Figure 30-18: Receive Context Descriptor

<!-- image -->

## RDES0 Context Descriptor

The following table describes the format for the RDES0 context descriptor.

Table 30-31: RDES0 Context Descriptor

| Bit   | Name   | Description                                                                                                                                                                                                                                                        |
|-------|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | RTSL   | Receive Packet Time Stamp Low. The DMAupdates this field with least significant 32 bits of the time stamp captured for corresponding receive packet. When this field and the RTSH field of RDES1 show all-one value, the time stamp must be considered as corrupt. |

## RDES1 Context Descriptor

The following table describes the format for the RDES1 context descriptor.

Table 30-32: RDES1 Context Descriptor

| Bit   | Name   | Description                                                                                                                                                                                                                                                         |
|-------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-0  | RTSH   | Receive Packet Time Stamp High. The DMAupdates this field with most significant 32 bits of the time stamp captured for corresponding receive packet. When this field and the RTSL field of RDES0 show all-ones value, the time stamp must be considered as corrupt. |

## RDES2 Context Descriptor

The following table describes the format for the RDES2 context descriptor.

Table 30-33: RDES2 Context Descriptor

| Bit   | Name     | Description   |
|-------|----------|---------------|
| 31-0  | Reserved |               |

## RDES3 Context Descriptor

The following table describes the format for the RDES3 context descriptor.

Table 30-34: RDES3 Context Descriptor

| Bit   | Name     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|-------|----------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31    | OWN      | Ownership. When this bit is set, it indicates that the DMAowns the descriptor. When this bit is reset, it indicates that the application owns the descriptor. The DMAclears this bit when either of the following conditions is true: • The DMAcompletes the packet reception • The buffers associated with the descriptor are full                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 30    | CTXT     | Receive Context Descriptor. When this bit is set, it indicates that the current descriptor is a con- text descriptor. The DMAwrites 10 to this bit for context descriptor. DMAwrites 11 to indicate a descriptor error due to all 1s. When CTXT and DE bits are used together, {CTXT, DE} • 00 = Reserved • 01 = Reserved • 10 = Context Descriptor • 11 = Descriptor Error NOTE: When a descriptor error occurs, the receive DMAcloses the receive descriptor indi- cating a descriptor error. This receive descriptor is skipped and the buffer addresses are not used to write the packet data. The receive DMAsets the EMAC_DMA[n]_STAT.CDE bit but does not set the EMAC_DMA[n]_STAT.RI bit field even when IOC is set, as this is not marked as last receive descriptor for the packet. The subsequent valid receive descriptor is used to write the packet data. |
| 29    | DE       | Descriptor Error. See the CTXT bit description for details of using the DE bit along with the CTXT bit.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 28-0  | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

## Descriptor Structure for Split Header Support

EMAC supports splitting the incoming receive packet header such that the header and the payload are stored in different buffers (buffer-1 and buffer-2) in the system memory. For more details about splitting header on receive packets, see Header-Payload Split.

The Descriptors without Split Header Feature figure shows the descriptor structure without the split header feature.

Figure 30-19: Descriptors without Split Header Feature

<!-- image -->

The Descriptors with Split Header Feature figure shows the descriptor structure with the split header feature.

Figure 30-20: Descriptors with Split Header Feature

<!-- image -->

The DMA writes the header of the received packet by using the header address to which the RDES0 in the first descriptor is pointing (FD bit of RDES3 is set). The DMA writes the payload of the received packet into the buffer address to which the RDES2 is pointing. For subsequent descriptors (FD is set to 0), the address to which RDES0 (Header address) is pointing is not used. The payload is written only to buffers to which the RDES2 (payload address) is pointing.

The EMAC\_MAC\_EXT\_CFG.HDSMS bit field indicates the maximum header size allowed for splitting the header data in the received packet based on the value programmed. When the EMAC\_DMA[n]\_RXCTL2.ARBS bit field is programmed to a non-zero value, it indicates the receive buffer size for buffer-1 and the

EMAC\_DMA[n]\_RXCTL.RBSZ\_X\_0 and EMAC\_DMA[n]\_RXCTL.RBSZ\_13\_Y indicates the receive buffer size for buffer-2. The EMAC\_DMA[n]\_RXCTL2.ARBS field can be set to 0, for backward compatibility.

The value in the EMAC\_MAC\_EXT\_CFG.HDSMS field should be greater than or equal to the Buffer-1 size indicated by the EMAC\_DMA[n]\_RXCTL2.ARBS field for all the channels.

The following table shows how DMA processes a packet based on the packet type.

Table 30-35: Split Header Support Depending on the Packet Type

| Packet Type      | Name                                  | Description                                                                                 |
|------------------|---------------------------------------|---------------------------------------------------------------------------------------------|
| TCP or UDP       | 00 (L3L4 split                        | The DMAwrites the Ethernet header + IP header + TCP or UDP head- er into the header buffer. |
| IP (not TCP/UDP) | 00 (L3L4 split                        | The DMAwrites the Ethernet header + IP header into the header buffer.                       |
| Non-IP           | 00 (L3L4 split                        | The DMAdoes not split the header and payload                                                |
| Any packet       | 01 (L2 split)                         | The DMAwrites the Ethernet header based on split offset (SPLOFST) into the header buffer.   |
| IP               | 10 (Combination of L2 or L3/L4 split) | L3/L4 split                                                                                 |
| Non-IP           | 10 (Combination of L2 or L3/L4 split) | L2 split                                                                                    |
| NA               | 11                                    | Reserved                                                                                    |

The IP header includes IPv4 options in case of IPv4 packet and IPv6 extension headers in case of IPv6 frames. The points at which the header is split are shown in the L3/L4 Header Split Points figure.

Figure 30-21: L3/L4 Header Split Points

<!-- image -->

- NOTE: · The VLAN tag stripping must be set for the split function. For instance, the DMA separates the header and payload of an untagged packet only. So, when a tagged packet is received, then program EMAC such that the VLAN tags are deleted/stripped from the received packets.
- L3/L4 split is applicable for IP packets that are either untagged or VLAN stripped. If VLAN tag is retained in the packet forwarded to the DMA, L3-L4 split is not performed. However, if the EMAC\_SPLM\_OFST\_CFG.SPLM bit field is set to 2, L2 split is performed for VLAN tagged IP packets.

- For AV packets, to specify an alternative L2 split value, use the EMAC\_SPLM\_OFST\_CFG.SAVE and EMAC\_SPLM\_OFST\_CFG.SAVO bit fields.
- L2 Split is not supported for packets with variable preamble.

The following table shows the header length availability in RDES2.

Table 30-36: Header Length Availability in RDES2

|   First Descriptor (FD) Value |   Last Descriptor (LD) Value | Header Length Availability in RDES2                 |
|-------------------------------|------------------------------|-----------------------------------------------------|
|                             0 |                            1 | Header length not available                         |
|                             1 |                            0 | HL[9:0] for all packets                             |
|                             1 |                            1 | For IP packets, HL[9:0] For non-IP packets, HL[9:2] |

The DMA writes the header length in RDES2 of the first receive descriptor (RDES3[29] FD bit is set) for the packet. The packet length is written in RDES3 of the last receive descriptor (RDES3[28] LD bit set).

The buffer length for the payload is set by the driver through the EMAC\_DMA[n]\_RXCTL.RBSZ\_X\_0 and EMAC\_DMA[n]\_RXCTL.RBSZ\_13\_Y fields.

The DMA fills receive buffers fully in all except the last descriptor. The header length is taken to be the value based on the bits programmed in the EMAC\_MAC\_EXT\_CFG.HDSMS field.

## Enhanced Descriptor for Time-Based Scheduling

The time-based scheduling feature needs Enhanced Descriptors (that are 32 Bytes) to be enabled on all the DMA channels that intend to use the feature (by setting the EMAC\_DMA[n]\_TXCTL.EDSE bit).

The structure of the 32-byte descriptor for the context and the normal descriptor in read and write formats are described in the following sections.

## Enhanced Normal Descriptor - Read

Figure 30-22: Enhanced Normal Descriptor - Read (32-bit Mode)

<!-- image -->

Table 30-37: Enhanced Normal Descriptor - Read

| Bit   | Name   | Description                                                                                                                                                                 |
|-------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31-12 | LTV    | Launch Time Valid indicates the Launch Time (LT) and GSN fields present in the descrip- tor are valid. The LTV must be set only if the FD bit of the descriptor is not set. |
| 11-8  | GSN    | GCL slot number associated with the packet                                                                                                                                  |
| 7-0   | LT     | Launch Time associated with the packet.                                                                                                                                     |

For details of the other fields, see T ransmit Normal Descriptor (Read Format).

0

## Enhanced Normal Descriptor - Write

Figure 30-23: Enhanced Normal Descriptor - Write (32-Bit Mode)

<!-- image -->

For details of LTV, GSN, LT fields, see Enhanced Descriptor for Time-Based Scheduling. For details of the other fields, see T ransmit Normal Descriptor (Write-Back Format).

- NOTE: In both the write back formats, no modifications can be done to the extended 16 bytes. The rest of the 16 bytes (TDESC0 to TDESC3) are written back as per the previous 16 bytes Descriptor format.
- NOTE: Fetch time when enabled overrides the AV slot function
- NOTE: When an unaligned new GCL list (with a different CTR) is installed it is recommended not to have traffic during the installation of the new list. Any traffic during the switching of the lists might have unpredictable behavior regarding Fetch, Launch, and Launch expiry as the CTR and BTR values get updated while the frame is being processed.

## Enhanced Context Descriptor (Read)

Figure 30-24: Enhanced Context Descriptor - Read

<!-- image -->

0

For details of the various fields, see T ransmit Context Descriptor.

## Enhanced Context Descriptor (Write)

0

Figure 30-25: Enhanced Context Descriptor (Write)

<!-- image -->

For details of the various fields, see Receive Context Descriptor.

## MAC Transaction Layer (MTL)

The MAC Transaction Layer (MTL) provides the FIFO memory interface to the buffer and regulates the packets between the application system memory and the MAC. It also enables the data to transfer between the application and MAC clock domains. The MTL layer has transmit and receive data paths. The data path for both directions is 32 bits wide and each transmit/receive DMA channel has a dedicated FIFO.

The transmit FIFO size for EMAC0 is 2048 bytes and for EMAC1 is 1024 bytes. The receive FIFO size for EMAC0 is 2048 bytes and for EMAC1 is 512 bytes.

## Transmit Path and Operation

The DMA engine controls all transactions for the transmit path with the application. The DMA pushes the Ethernet packets read from the application or system memory into the corresponding queue. The packet is then popped out and transferred to the MAC when the queue threshold is reached (threshold mode) or the complete packet is in the queue (store-and-forward mode). When EOP is transferred, the status of the transmission is taken from the MAC and transferred back to the DMA.

The queue fill level is indicated to DMA (using PBL and watermark) so that it can initiate a data fetch in required bursts from the application or system memory. The DMA indicates the SOP and EOP as packet delimiters through the Application Transmit Interface (ATI). In configurations with multiple queues, the DMA should also indicate the queue number for which the transaction is being addressed.

The following modes of operation trigger reading the data towards the MAC.

1. Threshold mode. When the number of bytes in the FIFO queue crosses the configured threshold level, (or when the end of packet is written before the threshold is crossed), the data is ready to be popped out and

forwarded to the MAC. The threshold level is configured using the EMAC\_TQ0\_OPMODE.TTC bit field that corresponds to an MTL queue.

2. Store-and-forward mode. In this mode, the MTL pops the packet towards the MAC only when one or more of the following conditions are true:
- A complete packet is stored in the queue
- The transmit FIFO becomes almost full
- The ATI watermark becomes low

The watermark becomes low when the requested queue does not have space to accommodate the requested burst length on the ATI. Therefore, the MTL, when operating in the store-and-forward mode, allows the packet transmission even if the packet length is bigger than the transmit queue size. The application can flush the complete content of the transmit queue using the EMAC\_TQ0\_OPMODE.FTQ bit that corresponds to a MTL queue. This bit is self-clearing and initializes the queue pointers to the default state.

If the EMAC\_TQ0\_OPMODE.FTQ bit is set during a packet transfer from the MTL to the MAC, the MTL stops further transfer because the queue is empty. Therefore, an underflow event occurs at the MAC transmitter. For information about the initialization and transmit operations for the MTL layer, see the following sections.

## Initialization Flow

At reset, the MTL is ready to manage the flow of data to and from the application or DMA, and MAC. For configurations with a single transmit queue, there are no initialization requirements for enabling the MTL. For configurations with multiple transmit queues, the program must initialize the queue size for each of the queues using the EMAC\_TQ0\_OPMODE.TQS bits that correspond to a transmit queue. The program must also initialize the MAC block. The DMA controllers must be individually enabled through their respective registers.

## Transmit DMA Channels and Queues

The number of transmit DMA channels is equal to the transmit queues. This is because the DMA is designed to arbitrate among multiple channels (to fetch the descriptor and packet data from the system memory) in terms of PBLs (Programmable Burst Lengths). However, the transmit queues are packet-based storage memory. Therefore, one to one mapping is required between the DMA channels and transmit queues to maintain the packet-level coherency.

## Transmit Operation with Multiple Packets in the Buffer

The transmit FIFO can be configured to accept up to two packets at a time.

## Retransmission During Collision

While a packet is being transferred from the MTL to the MAC, a collision event can occur on the MAC line interface in half-duplex mode. The MAC indicates a retry attempt to the MTL by giving the status even before the EOP is transferred from the MTL. The MTL then enables the retransmission by popping out the packet again from the queue.

After more than 96 bytes (or 548 bytes in 1000-Mbps mode) are read out towards the MAC, the queue controller frees up that space and makes it available to the application or the DMA to push in more data. This means that the retransmission is not possible after this threshold is crossed or when the MAC indicates a late-collision event.

## Transmit Queue Flush Operation

The MTL allows a transmit Queue to be flushed at any moment using the EMAC\_TQ0\_OPMODE.FTQ bit. The application can read back the same bit (cleared after flush is complete) to check the status of the flush operation.

## Receive Path and Operation

The MTL receive module receives packets from the MAC and pushes them into the receive queue. The status (fill level) of the queue is indicated to the DMA when it crosses the configured receive threshold ( EMAC\_RQ0\_OPMODE.RTC bits of the corresponding MTL queue), or the complete packet is received. The MTL also indicates the fill level of the queue so that the DMA can initiate preconfigured burst transfers towards the SCB interface.

The receive operation occurs as follows:

1. When the MAC receives a packet, it indicates the availability of receive data.
2. The MAC indicates the SOP and EOP delimiters.
3. The MTL accepts the data and pushes it into corresponding receive queue.
4. After the EOP is transferred, the MAC drives the status word which is also pushed into the corresponding receive queue by the MTL.

NOTE: In Threshold (cut-through) mode, the status words are stored after the packet EOP . In store-and-forward mode, the location for the maximum status words are reserved before writing the SOP and the status is written to reserved locations after writing the EOP .

5. If IEEE 1588 time stamp feature is enabled and the 64-bit time stamp is available along with the packet status, it is pushed into the receive queue as a part of the status word. Therefore, two additional locations are taken per packet to store the time stamp in the receive queue.
6. The MTL takes the data out of the FIFO and sends it to the DMA.
- Threshold mode (default). In this mode, the MTL reads the data and indicates its availability to the application or DMA when one of the following occurs:
- Data bytes equal to the threshold amount are written to the receive queue ( EMAC\_RQ0\_OPMODE.RTC bit field value)
- A full packet of data is received into the queue
- Receive FIFO Store-and-forward mode. This mode is configured using EMAC\_RQ0\_OPMODE.RSF bit. The initial receive queue locations are reserved for the status words before writing the SOP . A packet is read out only after it is completely written into the receive queue. In this mode, all error packets are dropped (if configured through the EMAC\_RQ0\_OPMODE.FEP of a queue) such that only valid packets are read and forwarded to the application.

## Multi-Packet Receive Operation

In Threshold mode, the packet status is available immediately after the packet data. In store-and-forward mode, the packet data is available after the packet status. The MTL can store any number of packets into the queue when it is not full. When the MAC receives a packet and the corresponding receive queue is full, the MTL ignores that packet and an overflow event occurs. Additionally, the MTL increments the overflow counter ( EMAC\_RQ0\_MSPKTOF\_CNT.OVFPKTCNT ) of the corresponding queue.

## Receive Error Handling

If the MTL receive queue is full before it receives the EOP data from the MAC, the following happens:

- An overflow is declared
- The whole packet (including the status word) is dropped
- The overflow counter in the DMA ( EMAC\_RQ0\_MSPKTOF\_CNT.OVFPKTCNT bits of corresponding MTL queue) is incremented.

This is true even if the EMAC\_RQ0\_OPMODE.FEP bit of the corresponding MTL queue is set.

If the start address of such a packet has already been transferred to the read controller, the rest of the packet is dropped and a dummy EOP is written to the queue along with the status word with overflow status. The status indicates a partial packet because of overflow. In such packets, the packet length field is invalid. If the MTL receive queue is configured to operate in the store-and-forward mode and the length of the received packet is more than the queue size, overflow occurs, and all such packets are dropped.

The MTL receive control logic can filter error and undersized packets using the EMAC\_RQ0\_OPMODE.FEP and EMAC\_RQ0\_OPMODE.FUP bits of the corresponding MTL queue. If the start address of such a packet has already been transferred to the receive queue read controller, that packet is not filtered. The start address of the packet is transferred to the Read Controller after the packet crosses the receive threshold set by the EMAC\_RQ0\_OPMODE.RTC of the corresponding MTL queue.

## EMAC CORE

The EMAC CORE is the lowest block in the EMAC peripheral and it performs all operations with the external world (PHY chip). It has independent transmit and receive modules. The modules interact with the EMAC Transaction Layer (MTL) at one end and interacts with the PHY chip through the RGMII/RMII/MII interface at the other end. Both modules have several sub blocks which are discussed in subsequent sections.

Transmission is initiated when the MTL pushes in data with the start of packet (SOP). The CORE then transmits to the RGMII/RMII/MII. After the end of packet transfers out, the CORE gives the status of the transmission back to the MTL. The MTL forwards the transmission to the application through DMA.

A receive operation initiates when the EMAC detects an SFD on the RGMII/RMII/MII. The CORE strips the preamble and SFD before proceeding to process the frame. The header fields are checked for the filtering and the FCS field used to verify the CRC for the frame. The frame drops in the core when it fails the address filter.

NOTE: The term CORE (written in capitals) refers to the internal block of Ethernet peripheral. Do not confuse the term with the processor core .

## EMAC CORE Transmission Engine

The following modules constitute the transmission function (transmission engine components) of the EMAC:

- Transmit Bus Interface Module (TBU)
- Transmit Packet Controller Module (TPC)
- Transmit Protocol Engine Module (TPE)
- Transmit Scheduler Module (STX)
- Transmit CRC Generator Module (CTX)
- Transmit Flow Control Module (FTX)

## Transmit Bus Interface Module (TBU)

This module interfaces the transmit path of the EMAC CORE with the MAC Layer FIFO interface. This module outputs the transmit status to the application at the end of normal transmission or collision.

## Transmit Packet Controller Module (TPC)

The Transmit Packet Controller (TPC) module consists of eight registers to hold the data and the last data control received from the TBU.

The register provides a buffer between the application and the T ransmit Protocol Engine (TPE) to regulate data flow.

When the number of bytes received from the application are less than 60 (DA+SA+LT+DATA), the state machine that interfaces with TBU automatically appends zeros to the packet being transmitted. This is done to make the data length exactly 46 bytes to meet the minimum data field requirement of IEEE 802.3. The EMAC module can also be programmed to not append any padding.

The cyclic redundancy check (CRC) for the Frame Check Sequence (FCS) field is calculated before transmission to the TPE module. This value is computed by the CTX module. The TPC module receives the computed CRC and appends it to the data being transmitted to the TPE module. When the MAC is programmed to not append the CRC value to the end of Ethernet packets, the TPC module ignores the computed CRC and transmits only the data received from the TBU module to the TPE module. An exception to this rule is that when the MAC is programmed to append pads for packets (DA+SA+LT+DATA) less than 60 bytes sent by the TBU module, the TPC module always appends the CRC at the end of padded packet.

## Transmit Protocol Engine Module (TPE)

The Transmit Protocol Engine (TPE) module consists of a Transmit State Machine that controls the operation of Ethernet packet transmission. The Transmit State Machine of this module contains the following features to meet the IEEE 802.3/802.3z specification:

- Generates preamble and SFD
- Generates jam pattern in half-duplex mode after normal collision

- Generates carrier extension in the half-duplex (only in the RGMII) mode when packet is smaller than 512 bytes
- Supports packet bursting in the half-duplex (only in the RGMII) mode
- Jabber timeout
- Flow control for half-duplex mode (back pressure)
- Generates transmit packet status
- Contains time stamp snapshot logic for IEEE 1588 support

When the TPC module requests the TPE module for a new packet transmission, the Transmit State Machine sends out the preamble and SFD, followed by the data received. The preamble is defined as 7 bytes of 10101010 pattern and the SFD is defined as 1 byte of 10101011 pattern.

The collision window is defined as 1 slot time (512-bit times for 10/100 Mbps Ethernet and 4096-bit times for 1,000 Mbps Ethernet). The jam pattern generation is applicable only to half-duplex mode, not to full duplex mode.

In MII/RMII mode, if a collision occurs any time from the beginning of the packet to the end of the CRC field, the Transmit State Machine sends a 32-bit jam pattern of 0x55555555 on MII/RMII to inform all other stations that a collision has occurred. If the collision is seen during the preamble transmission phase, the T ransmit state machine completes the transmission of preamble and SFD, and then sends the jam pattern.

In RGMII mode, if a collision occurs any time between the beginning of the packet and the end of the extension field, the T ransmit State Machine sends a 32-bit jam pattern of 0x55555555 on RGMII to inform all other stations of the collision. If the collision is seen during the preamble transmission phase, the T ransmit State Machine completes the transmission of preamble and SFD, and then sends the jam pattern. If a collision occurs during the extension field, the T ransmit State Machine sends a 32-bit jam pattern of 0x1F1F1F1F .

If the collision occurs after the collision window and before the end of the FCS field (or the end of burst if the packet burst mode is enabled), the T ransmit State Machine sends a 32-bit jam pattern and sets the late collision bit in the transmit packet status.

In RGMII half-duplex mode (1000 Mbps), the Transmit State Machine ensures that all valid carrier events exceed a slot time of 4,096-bit times. To accomplish this, any transmit packet shorter than 512 bytes from the TFC module is extended using a carrier extension.

When the packet burst mode is enabled, only the first packet of the burst is carrier extended if it is shorter than 512 bytes. The carrier extension is not applicable for MII/RMII half-duplex and RGMII or MII/RMII full-duplex modes. When the packet burst mode is enabled, the Transmit State Machine transmits a burst of packets (when packets are available from the TFC module) without releasing the carrier of the PHY. To accomplish this, the state machine inserts the carrier extension for a minimum IPG period (96-bit times) between the packets. The Transmit State Machine continues to burst packets when additional packets are available from the TPC module and a burst limit of 8192-byte times is not exceeded. When an additional packet is not available at the end of the IPG period in the middle of the burst, the T ransmit State Machine releases the carrier RGMII.

Packet bursting is applicable only for the RGMII half-duplex mode. It is not applicable in the MII/RMII and RGMII full-duplex modes. In the RGMII half-duplex mode, the size of the first packet in a packet burst should at least be equal to the slot time. If the first packet (including carrier extension) is less than the slot time, the MAC considers all data bytes, received for the first packet and subsequent packets, which end immediately after the slot time is reached as first packet in the burst. This may result in CRC error. However, if the packet burst ends (both RXDV and RXER go low) before the slot time, the subsequent packet is considered as a new packet. This behavior is according the IEEE 802.3.

The TPE module maintains a jabber timer to stop the transmission of Ethernet packets if the TFC module transfers more than 2,048 (default) bytes. The timeout is changed to 10,240 bytes when the jumbo packet is enabled.

The Transmit State Machine uses the deferral mechanism for flow control (backpressure) in the half-duplex mode. When the application requests to stop receiving packets, the T ransmit State Machine sends a JAM pattern of (8'h55) 32 bytes whenever it senses a reception of a packet, provided the transmit flow control is enabled. This results in a collision and the remote station backs off. The application requests the flow control through by setting the EMAC\_Q[i]\_TXFLOW\_CTL.FCB\_BPA bit of the corresponding MTL queue. If the application requests a packet to transmit, it is scheduled and transmitted even when the backpressure is activated. If the backpressure is kept activated for a long time (and more than 16 consecutive collision events occur), the remote stations abort their transmissions because of excessive collisions.

If IEEE 1588 time stamp is enabled for the transmit packet, this block takes a snapshot of the system time when the SFD is put onto the transmit RGMII or MII/RMII bus. The system time source is either an external input or it is internally generated according to the configuration selected.

## Transmit Scheduler Module (STX)

The transmit scheduler is responsible for scheduling the frame transmission on the RGMII or MII/RMII. The two major functions of this module are:

- Maintains the inter-packet gap between two transmitted packets. The STX module maintains an idle period of the configured inter-packet gap ( EMAC\_MAC\_CFG.IPG ) between any two transmitted packets. If packets from the TPC arrive at the TPE module sooner than the configured IPG time, the TPE module waits for the enable signal from the STX module before starting the transmission on GMII or MII. The STX module starts its IPG counter as soon as the carrier signal of RGMII or MII/RMII goes inactive. At the end of the programmed IPG value, the module issues an enable signal to the TPE module in the full-duplex mode.

In the half-duplex mode and when IPG is configured for 96-bit times, the STX module follows the rule of deference specified in the IEEE 802.3, Section 4.2.3.2.1. The module resets its IPG counter if a carrier is detected during the first two-thirds (64-bit times for all IPG values) of the IPG interval. If the carrier is detected during the final one-third of the IPG interval, the STX module continues the IPG count and enables the transmitter after the IPG interval.

- Implements the Truncated Binary Exponential Back-off algorithm in half-duplex mode.

## Transmit CRC Generator Module (CTX)

The MAC Transmit CRC Generator (CTX) module interfaces with the TFC module to generate the CRC for the FCS field of the Ethernet packet.

The TPC module sends the packet data and any necessary padding to the CTX module through an 8-bit interface.

The CTX module calculates the 32-bit CRC for the FCS field of the Ethernet packet. The encoding is defined by the following generating polynomial:

```
G (x) = x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
```

The CTX module gets the byte data of Ethernet packet from the TPC module (DA + SA + LT + DATA + PAD) qualified with a data valid signal. The TPC also indicates to the CTX when to reset the previously calculated CRC and to start the new CRC calculation for the coming packet. The TPC module issues the start command before sending the new packet data for calculation. The calculated CRC is valid on the next clock after the data is received.

In RGMII mode, the data valid signal is valid for every clock, from the first data byte through the last data byte. In MII/RMII mode, this signal is valid every alternate clock.

## Transmit Flow Control Module (FTX)

The Transmit Flow Control (FTX) module generates and transmits the pause packets to the TFC module based on the flow control triggers in full-duplex mode. The TFC module receives the pause packet from the FTX module, appends the calculated CRC, and sends the packet to the TPE module.

## EMAC Core Reception

The following are the functional blocks (reception engine components) in the receive path of the EMAC core.

- Receive Protocol Engine Module (RPE)
- Receive CRC Module (CRX)
- Receive Packet Controller Module (RPC)
- Receive Flow Control Module (FRX)
- Receive Bus Interface Unit Module (RBU)
- Address Filtering Module (AFM)

## Receive Protocol Engine Module (RPE)

The Receive Protocol Engine (RPE) consists of the Receive State Machine which strips the preamble, SFD, and carrier extension of the received Ethernet packet (in half-duplex 1000-Mbps mode). The sequence is as follows:

1. When the RX\_DV signal of GMII or MII becomes active, the Receive State Machine of RPE starts looking for the SFD field (byte 0x5D in GMII mode; 0xD nibble in MII mode). The state machine drops received packets until it detects SFD.
2. When SFD is detected, the state machine begins sending the data of Ethernet packet to the RPC module, beginning with the first byte following the SFD (destination address).

3. If the IEEE 1588 time stamp feature is enabled, the RPE takes a snapshot of the system time at which SFD of any packet is detected on GMII or MII. If this packet is not dropped during MAC filtering, the time stamp is passed to the application. In MII mode, the RPE converts the received nibble data into bytes and forwards the valid packet data to the RFC module.
4. The Receive State Machine of the RPE module decodes the length/type field of the receiving Ethernet packet.

If the length/type field is less than 1,536 and if the MAC is programmed for the Auto CRC/pad stripping ( EMAC\_MAC\_CFG.ACS bit), the state machine sends the data of the packet up to the count specified in the length/type field and starts dropping bytes (including the FCS field). The state machine of the RPE module decodes the length/type field and checks for the length interpretation.

- NOTE: In Audio Video (AV) or Data Center Bridging (DCB) mode when additional receive queues are selected, the packets that are less than or equal to 16 bytes in length after pad stripping, always get dropped inside the MAC receiver. This happens even if the packets have passed the address filter and have no CRC error.

If the length/type field is greater than or equal to 1,536, the RPE module sends all received Ethernet packet data to the RFC module if the CRC stripping for type packet has not been enabled with the EMAC\_MAC\_CFG.CST bit. However, if CRC stripping for type packets is enabled and the Receive Checksum Offload Engine is not enabled, the MAC strips and drops the last 4 bytes of all packets of Ether type before forwarding the packets to the application.

5. By default, the MAC is programmed for watchdog timer to be enabled, that is, packets larger than 2,048 (10,240 if Jumbo Packet is enabled) bytes, (DA + SA + LT + DATA + PAD + FCS) are cut off at the RPE module. In addition, a programmable watchdog timer can be used (bit 16 of the EMAC\_WDT\_TMOUT register) to override the fixed timeout of 2,048 or 10,240 bytes. Programs can disable the watchdog timer using the EMAC\_MAC\_CFG.WD bit. However, even if the watchdog timer is disabled, a packet greater than 32 KB size is cut off and a watchdog timeout status is given.

At the end of every received packet, the RPE module generates received packet status and sends it to the RPC module. Control, missed packet, and filter fail status are added to the receive status in the RPC module.

## Receive CRC Module (CRX)

The receive CRC module checks for any CRC errors in the receiving frame.

This module calculates the 32-bit CRC for the received frame that includes the destination address field through the FCS field (DA+SA+LT+DATA+PAD+FCS). The following generating polynomial defines the encoding.

<!-- formula-not-decoded -->

Irrespective of the auto pad or CRC strip, the CRC module receives the entire frame to compute the CRC check for received frame.

## Receive Packet Controller Module (RPC)

The MAC Receive Packet Controller (RPC) receives the Ethernet packet data and status from the RPE module.

The RPC module consists of a FIFO of parameterized depth (default set to 4 deep and 33-bits wide) and two state machines for writing and reading the FIFO. The FIFO holds the received Ethernet packet data and byte enables, along with a control bit to indicate the last data. The state machines manage the FIFO and provide a packet buffering for the Ethernet packet being received from the RPE module. The following are main functions of the RPC module:

- Converting Data path - converts 8-bit data to 32-bit data to the RBU module
- Packet filtering
- Attaching the calculated IP checksum input form the IPC
- Updating the receive status and forwarding it to RBU

If the EMAC\_MACPKT\_FILT.RA bit is set, the RPC module initiates the data transfer to the RBU module when 4 bytes of Ethernet data are received from the RPE module. At the end of the data transfer, the RPC module sends out the received packet status that includes the packet filter bits (SA Filterfail and DA Filterfail) and status from the RPC module. These bits are generated based on the filter-fail signals from the AFM module. This status bit indicates to the application whether the received packet has passed the filter controls (both address filter and Packet Filter controls from CSR). The RPC module does not drop any packet on its own in this mode.

If the EMAC\_MACPKT\_FILT.RA bit is reset, the RPC module performs packet filtering based on the destination or source address. If the application does not want to receive any bad packets such as runt, CRC error packets, the application still needs to perform another level of filtering. The RPC module waits to receive the first 14 bytes of received data (type field) from the RPE module. Until then, the module does not initiate any transfers to the RBU module. After receiving the destination or source address bytes, the RPC checks the filter-fail signal from the AFM module for an address match. On detecting filter-fail from AFB, the packet is dropped at the RPC module and not transferred to the application.

On a delayed filter response from the AFM (this can only occur if the AFM logic is changed), the RPC module waits until the FIFO is full, and then proceeds with the packet transfer to the RBU module. However, the RPC module still takes the delayed response from the AFM module and if it is a (DA or SA) filter failure, it drops the rest of the packet and sends the receive status word (with zero packet-length, CRC error and runt error bits set) immediately indicating the filter fail. If there is no response from the AFM until EOP is transmitted, the filter fail status in the receive status word is updated accordingly.

## Receive Flow Control Module (FRX)

The Receive Flow Controller (FRX) detects the pause packet being received and pauses the packet transmission for the delay specified within the received pause packet. The FRX module is enabled only in the full-duplex mode.

## Receive Bus Interface Unit Module (RBU)

The Receive Bus Interface Unit (RBU) converts the 32-bit data received from the RPC module into a 32-bit protocol on the Application side.

## Address Filtering Module (AFM)

The Address Filtering Module (AFM) performs the destination and source address checking function on all received packets and reports the address filtering status to the RPC module.

The address checking is done based on different parameters ( EMAC\_MACPKT\_FILT register) chosen by the application. These parameters are inputs to the AFM module as control signals, and the AFM module reports the status of the address filtering based on the combination of these inputs. The AFM module does not filter the receive packets but reports the status of the address filtering (whether to drop the packet or not) to the RFC module. The AFM module also reports address filter status and whether the received packet is a multicast packet or a broadcast packet.

The AFM module probes the 8-bit receive data path between the RPE module and the RFC module and checks the destination and source address field of each incoming packet. In GMII mode, the module takes 8/14 clocks (from the start of packet) to compare the destination/source address of the packet being received. Similarly, in MII mode, the module takes 14/26 clocks (from the start of packet) to compare the destination or source address of the receiving packet. The AFM module gets the physical (MAC) address of the station and the Multicast Hash table from CSR module for address checking. The CSR module provides the packet filter register parameters to AFM.

## Source Address, VLAN, CRC Insertion/Replacement

The following sections provide information on Source Address and VLAN Insertion, Replace, or Delete, Queue/ Channel Based VLAN Tag Insertion on Transmission, and CRC Insertion and Replacement.

## Double VLAN Processing

The EMAC module supports two VLAN tags, namely inner and outer, for processing double VLANs.

This feature is referred called double VLAN tagging where the MAC can process two VLAN tags. With this feature, the EMAC supports:

- Insertion, replacement, or deletion of up to two VLAN tags in the transmit path.
- Packet filtering and stripping based on any one of the two VLAN tags in the receive path. Stripping and providing up to two VLAN tags in the receive path as a part of the receive status.

## Transmit Path

The following table describes the double VLAN features supported by the MAC on the transmit side.

Table 30-38: Double VLAN Processing Features in Transmit Path

| Feature                                 | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|-----------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Support for C-VLAN and S-VLAN Tag types | The inner or outer VLAN tag can be of C-VLAN and S-VLAN type. The VLAN type is specified through the EMAC_VLAN_INCL.CSVL bit and the EMAC_INVLAN_INCL.CSVL bit. The EMAC supports processing of any sequence of outer and inner VLAN tags. NOTE: The EMAC does not support the C-VLAN S-VLAN sequence. The MAC does not check whether the packet provided by the application has a valid sequence of the VLAN tag types or whether the insertion or replacement operation results in an invalid sequence of VLAN tag type. Therefore, the application must provide a correct sequence of VLAN tag types and program the MAC so that the VLAN tag types in the transmitted packet contains the correct se- quence. The application must ensure the following: • The inner tag should not be S-VLAN when the outer C-VLAN tag insertion is enabled. • The outer tag should not be C-VLAN when the inner S-VLAN tag insertion is enabled. • The inner tag cannot be S-VLAN when the outer tag is replaced with C-VLAN. • The outer tag cannot be C-VLAN when the inner tag is replaced with S-VLAN. |
| VLAN Tag deletion                       | Programs can enable the VLAN tag deletion for outer or inner tag using the EMAC_VLAN_INCL.VLC and EMAC_INVLAN_INCL.VLC bit fields, respectively. When VLAN deletion is enabled, the MAC deletes the tag present at the corresponding position. When a packet has only one tag, it is considered as the outer tag. If inner tag deletion is enabled and the packet has only one tag, the MAC does not delete the tag.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| VLAN Tag Insertion or Replacement       | Programs can enable the VLAN tag insertion or replacement for outer or inner tag using the EMAC_VLAN_INCL.VLC EMAC and EMAC_INVLAN_INCL.VLC bit fields, respectively. When VLAN tag insertion or replacement is enabled, the EMAC_VLAN_INCL.VLTI bit and the EMAC_INVLAN_INCL.VLTI bit are used to determine whether the VLAN tag is taken from the register or the control word.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

## Receive Path

The following table describes the features supported by the MAC on the receive side and the corresponding bits in the EMAC\_VLANTAG\_CTL register.

Table 30-39: Double VLAN Processing in Receive Path

| Feature                                               | Description                                                                                                                                                             |
|-------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Outer or inner VLAN tag-based filtering               | The MAC can filter packets based on the outer or inner VLAN tag through the EMAC_VLANTAG_CTL.ERIVLT bit.                                                                |
| C-VLAN or S-VLAN tag-based fil- tering                | The MAC can filter packets based on the C-VLAN or S-VLAN type based on the EMAC_VLANTAG_CTL.ERSVLM bit.                                                                 |
| Outer and Inner VLAN Tag strip- ping                  | The MAC can strip the outer and inner VLAN Tags from received frame based on the EMAC_VLANTAG_CTL.EVLS and EMAC_VLANTAG_CTL.EIVLS bits.                                 |
| 16-bit outer and inner VLAN Tag and Type in Rx status | The MAC can provide the 16-bit outer and inner VLAN tag and type in the Rx status based on the EMAC_VLANTAG_CTL.EVLRXS and EMAC_VLANTAG_CTL.EIVLRXS bits, respectively. |

Table 30-39: Double VLAN Processing in Receive Path (Continued)

| Feature                                               | Description                                                                                                                      |
|-------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------|
| Disabling or skipping checking of outer VLAN Tag type | The MAC can disable or skip checking of outer VLAN tag type to match C-VLAN or S- VLAN based on the EMAC_VLANTAG_CTL.DOVLTC bit. |

## Source Address and VLAN Insert, Replace, or Delete

The EMAC supports inserting or replacing the source address based on the information in the MAC address registers. The MAC can also insert, replace, or delete the VLAN fields (VLAN type and VLAN tag) based on the setting of the EMAC\_VLAN\_INCL.VLTI bit. Programs can enable the SA insert or replace feature for all transmit packets or selective packets. Similarly, programs can enable the VLAN insert, replace, or delete feature for all transmit packets or selective packets.

## Programming Source Address Insert or Replace

The software can use the SA insert or replace feature to instruct the MAC to do the following for transmit packets:

- Insert the content of the MAC address registers in the SA field
- Replace the content of the SA field with the content of the MAC address registers

When SA insert is enabled, the application must ensure that the packets sent to the MAC do not have the SA field. The MAC does not check whether the SA field is present in the transmit packet and it inserts the content of MAC address registers in the SA field. Similarly, when SA replace is enabled, the application must ensure that the SA field is present in the packets sent to the MAC. The MAC replaces the six bytes following the destination address field in the transmit packet with the content of the MAC address registers.

The program can enable the SA insert or replace feature for all transmit packets or selective packets:

- Enable SA insert or replace for all packets: program the EMAC\_MAC\_CFG.SARC bit field
- Enable SA insert or replace for selective packets: program the SA insertion control field (bits [25:23] of TDES3) in the first transmit descriptor of the packet. When bit 25 of TDES3 is set, the SA insertion control field indicates insertion or replacement by the MAC address1 registers. When bit 25 of TDES3 is reset, it indicates insertion or replacement by the MAC address0 registers.

If the MAC address1 registers are not enabled, the MAC address0 registers are used to insert or replace irrespective of the value of the most-significant bit of the SA insertion control field.

## Programming VLAN Insert, Replace, or Delete

The software can use the VLAN insert, replace, or delete feature to instruct the MAC to do the following for transmit packets:

- Delete the VLAN type and VLAN tag fields
- Insert or replace the VLAN type and VLAN tag fields using the EMAC\_VLAN\_INCL.VLTI bit as described in the following table.

Table 30-40: Double VLAN Processing in Receive Path

| Condition                        | Description                                                                                                                                                                                                      |
|----------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| EMAC_VLAN_INCL.VLTI bit is set   | The MAC inserts or replaces the following: • VLAN type field (C-VLAN or S-VLAN as indicated by the EMAC_VLAN_INCL.CSVL bit • VLAN Tag field content of the VT field of transmit context descriptor of the packet |
| EMAC_VLAN_INCL.VLTI bit is reset | The MAC inserts or replaces the following: • VLAN Type field (C-VLAN or S-VLAN as indicated by the EMAC_VLAN_INCL.CSVL bit • VLAN tag field with the EMAC_VLAN_INCL.VLT bit field                                |

When VLAN replace or delete is enabled, the MAC checks if the VLAN type field (0x8100 or 0x88a8) is present after the DA and SA fields in the transmit packet. The replace or delete operation does not occur if the VLAN type field is not detected in two bytes following the DA and SA fields. However, when VLAN insertion is enabled, the MAC does not check for the presence of the VLAN type field in the transmit packet and just inserts the VLAN type and VLAN tag fields.

Programs can enable the VLAN insert, replace, or delete feature for all transmit packets or selective packets:

- Enable VLAN insert, replace, or delete for all packets: program the EMAC\_VLAN\_INCL.VLC and EMAC\_VLAN\_INCL.VLP bit fields
- Enable VLAN insert, replace, or delete for selective packets: program the VTIR field of the TDES2 normal descriptor

## Queue/Channel Based VLAN Tag Insertion on Transmit

The EMAC supports channel/queue based VLAN tag insertion on all transmitted packets. Queue/channel specific VLAN tag registers are accessed using indirect addressing through the EMAC\_VLAN\_INCL register. VLAN type and tag values can be independently programmed for each queue/channel. See Programming Queue/Channel Based VLAN Inclusion Registers for more details.

## CRC Insert and Replace

The software can use the CRC replace feature to instruct the MAC to replace the FCS field in the transmit frame with the CRC computed by the MAC. This feature works on per-frame basis. To enable the CRC replacement feature, program the CRC replacement control field (bit 26 of TDES3) in the first transmit descriptor of the frame.

## Packet Filtering

the Packet Filtering Sequence figure shows the filtering sequence for receive packets. The sequence shown in the fig- ure is valid when all the filters (L2, VLAN, L3, L4) are active. If any of the layer filters are not enabled, that filter is bypassed, and the subsequent filter is applied. A packet that fails any of the filters is discarded. However, the discarded packet can be forwarded to the host based on the register control. For example, when the EMAC\_MACPKT\_FILT.RA bit is set to 1, all discarded packets are forwarded to the host but with its packet status indicating the specific filter-failure. If RA=0, the EMAC\_MACPKT\_FILT.VTFE and

EMAC\_MACPKT\_FILT.IPFE bits of control whether the packets that fail the VLAN filter and Layer 3-4 filter should be discarded or forwarded to the host.

Figure 30-26: Packet Filtering Sequence

<!-- image -->

## Source Address or Destination Address Filtering

The Address Filtering Module of the MAC checks the source address (SA) and destination address (DA) fields of each incoming packet.

## Unicast Destination Address Filtering

The MAC supports 32 MAC addresses for unicast perfect filtering. If perfect filtering is selected (the EMAC\_MACPKT\_FILT.HUC bit is reset), the MAC compares all 48 bits of received unicast address with the programmed MAC address for any match. The default address0 register ( EMAC\_ADDR0\_HI , EMAC\_ADDR0\_LO ) is always enabled.

The address1 through 31 register addresses (both HIGH and LOW) are selected with an individual enable bit. For these addresses, programs can mask each byte during comparison with corresponding received DA byte by setting the corresponding mask byte control bit in the register. This enables group address filtering for the DA.

In hash filtering mode (when the EMAC\_MACPKT\_FILT.HUC bit is set), the MAC performs imperfect filtering for unicast addresses using a hash table (256 bits for EMAC0 and 64 bits for EMAC1). For hash filtering, the MAC uses the upper 6 bits CRC of the received destination address to index the content of the hash table. A value of 00000 selects bit 0 of the selected register, and a value of 11111 selects bit 63 of the hash table register (registers 7-0). When the corresponding bit (indicated by the 6-bit CRC) is set to 1, the unicast packet is considered to have passed the hash filter; otherwise, the packet is considered to have failed the hash filter.

## Multicast Destination Address Filtering

To program the MAC to pass all multicast packets, set the EMAC\_MACPKT\_FILT.PM bit. If the EMAC\_MACPKT\_FILT.PM bit is reset, the MAC performs the filtering for multicast addresses based on the EMAC\_MACPKT\_FILT.HMC bit.

The multicast address is compared with the programmed MAC destination address registers (1-31). Group address filtering is also supported. In hash filtering mode, the MAC performs imperfect filtering using a 64-bit hash table. The MAC uses the upper 6-bits CRC of received multicast address to index the content of the hash table. A value of 000000 selects bit 0 of the selected register and a value of 111111 selects bit 63 of the hash table register. If the corresponding bit is set to 1, the multicast packet is considered to have passed the hash filter. Otherwise, the packet is considered to have failed the hash filter.

## Hash or Perfect Address Filtering

To configure the DA filter to pass a packet when its DA matches either the hash filter or the perfect filter, set the EMAC\_MACPKT\_FILT.HPF bit and the corresponding EMAC\_MACPKT\_FILT.HUC or EMAC\_MACPKT\_FILT.HMC bits. This is applicable to both unicast and multicast packets. If the EMAC\_MACPKT\_FILT.HPF bit is reset, only one of the filters (hash or perfect) is applied to the received packet.

## Broadcast Address Filtering

The MAC does not filter any broadcast packets by default. To program the MAC to reject all broadcast packets, set the EMAC\_MACPKT\_FILT.DBF bit.

## Unicast Source Address Filtering

The MAC can perform perfect filtering based on the source address field of received packets. By default, the MAC compares the SA field with the values programmed in the SA registers. Programs can configure the MAC address registers [1-31] to use SA instead of DA for comparison by setting bit 30 of corresponding register.

The MAC also supports group filtering with SA. Programs can filter a group of addresses by masking one or more bytes of the address. The MAC drops the packets that fail the SA filter if the EMAC\_MACPKT\_FILT.SAF bit is set. Otherwise, the result of the SA filter is given as a status bit in the Receive Status word. When the EMAC\_MACPKT\_FILT.SAF bit is set, the SA filter and DA filter result is AND'ed to decide whether the packet needs to be forwarded. This means that the packet is dropped if either filter fails. The packet is forwarded to the application only if the packet passes both filters in order.

## Inverse Filtering

For DA and SA filtering, programs can invert the filter-match result at the final output by setting the EMAC\_MACPKT\_FILT.DAIF and EMAC\_MACPKT\_FILT.SAIF bits. The EMAC\_MACPKT\_FILT.DAIF bit is applicable for both unicast and multicast DA packets. The result of the unicast or multicast destination address filter is inverted in this mode. Similarly, when the SAIF bit is set, the result of unicast SA filter is reversed.

The following tables summarize the DA and SA filtering based on the type of packets received.

- NOTE: When the EMAC\_MACPKT\_FILT.RA bit is set, all packets are forwarded to the system along with the correct result of the address filtering in the Rx status.

Table 30-41: Destination Address Filtering

| Packet Type   | PR   | HPF   | HUC   | DAIF   | HMC   | PM   | DBF   | DA Filter Operation                                                                            |
|---------------|------|-------|-------|--------|-------|------|-------|------------------------------------------------------------------------------------------------|
| Broad- cast   | 1    | X     | X     | X      | X     | X    | X     | Pass                                                                                           |
| Broad- cast   | 0    | X     | X     | X      | X     | X    | 0     | Pass                                                                                           |
| Broad- cast   | 0    | X     | X     | X      | X     | X    | 1     | Fail                                                                                           |
| Unicast       | 1    | X     | X     | X      | X     | X    | X     | Pass all packets                                                                               |
| Unicast       | 0    | X     | 0     | 0      | X     | X    | X     | Pass on Perfect/Group filter match                                                             |
| Unicast       | 0    | X     | 0     | 1      | X     | X    | X     | Fail on Perfect/Group filter match                                                             |
| Unicast       | 0    | 0     | 1     | 0      | X     | X    | X     | Pass on Hash filter match                                                                      |
| Unicast       | 0    | 0     | 1     | 1      | X     | X    | X     | Fail on Hash filter match                                                                      |
| Unicast       | 0    | 1     | 1     | 0      | X     | X    | X     | Pass on Hash or Perfect/Group filter match                                                     |
| Unicast       | 0    | 1     | 1     | 1      | X     | X    | X     | Fail on Hash or Perfect/Group filter match                                                     |
| Multi- cast   | 1    | X     | X     | X      | X     | X    | X     | Pass all packets                                                                               |
| Multi- cast   | X    | X     | X     | X      | X     | 1    | X     | Pass all packets                                                                               |
| Multi- cast   | 0    | X     | X     | 0      | 0     | 0    | X     | Pass on Perfect/Group filter match and drop Pause packets if EMAC_MACPKT_FILT.PCF = 0x         |
| Multi- cast   | 0    | 0     | X     | 0      | 1     | 0    | X     | Pass on Hash filter match and drop Pause packets if EMAC_MACPKT_FILT.PCF = 0x                  |
| Multi- cast   | 0    | 1     | X     | 0      | 1     | 0    | X     | Pass on Hash or Perfect/Group filter match and drop Pause packets if EMAC_MACPKT_FILT.PCF = 0x |
| Multi- cast   | 0    | X     | X     | 1      | 0     | 0    | X     | Fail on Perfect/Group filter match and drop Pause packets if EMAC_MACPKT_FILT.PCF = 0x         |
| Multi- cast   | 0    | 0     | X     | 1      | 1     | 0    | X     | Fail on Hash filter match and drop Pause packets if EMAC_MACPKT_FILT.PCF = 0x                  |
| Multi- cast   | 0    | 1     | X     | 1      | 1     | 0    | X     | Fail on Hash or Perfect/Group filter match and drop Pause packets if EMAC_MACPKT_FILT.PCF = 0x |

Table 30-42: Source Address Filtering

| Packet Type   |   PR | SAIF   | SAF   | DA Filter Operation                                                            |
|---------------|------|--------|-------|--------------------------------------------------------------------------------|
| Unicast       |    1 | X      | X     | Pass all packets                                                               |
| Unicast       |    0 | 0      | 0     | Pass status on Perfect or Group filter match but do not drop packets that fail |
| Unicast       |    0 | 1      | 0     | Fail status on Perfect or Group filter match but do not drop packet            |
| Unicast       |    0 | 0      | 1     | Pass on Perfect or Group filter match and drop packets that fail               |
| Unicast       |    0 | 1      | 1     | Fail on Perfect or Group filter match and drop packets that fail               |

## VLAN Filtering

The following sections provide information on EMAC VLAN filtering.

## VLAN Tag Perfect Filtering

In VLAN tag perfect filtering, the MAC compares the VLAN tag of received packet and provides the VLAN packet status to the application. Based on the programmed mode, the MAC compares the lower 12 bits or all 16 bits of received VLAN tag to determine the perfect match.

If VLAN tag perfect filtering is enabled, the MAC forwards the VLAN-tagged packets along with VLAN tag match status and drops the VLAN packets that do not match. Programs can enable the inverse matching for VLAN packets by setting the EMAC\_VLANTAG\_CTL.VTIM bit. In addition, programs can enable processing of S-VLAN tagged packets along with the default C-VLAN tagged packets by setting the EMAC\_VLANTAG\_CTL.ESVL bit. The VLAN packet status bit (bit 10 of RDES0) indicates the VLAN tag match status for the matched packets.

NOTE: The source or destination address (if enabled) has precedence over the VLAN tag filters. This means that a packet that fails the source or destination address filter is dropped irrespective of the VLAN tag filter results. By default, the VLAN tag-based perfect filter is available in all configurations.

## VLAN Tag Hash Filtering

The 16-bit VLAN hash table is used for group address filtering based on the VLAN tag. The MAC performs the VLAN hash matching using the EMAC\_VLANTAG\_CTL.VTHM (VLAN tag hash table match enable) bit. The MAC provides VLAN tag hash filtering with a 16-bit hash table.

If the EMAC\_VLANTAG\_CTL.VTHM bit is set, the most significant four bits of CRC-32 of VLAN tag are used to index the content of the EMAC\_VLAN\_HASHTBL register. A value of 1 in the register, corresponding to the index, indicates that the VLAN tag of the packet matched and the packet is forwarded. A value of 0 indicates that VLANtagged packet is dropped.

- NOTE: · The 16 or 12 bits of the VLAN tag are considered for CRC-32 computation based on the EMAC\_VLANTAG\_CTL.ETV bit.
- When the bit is reset, most significant four bits of CRC-32 of VLAN tag are inverted and used to index the content of EMAC\_VLANTAG\_CTL register.
- When the bit is set, most significant four bits of CRC-32 of VLAN tag are directly used to index the content of the EMAC\_VLANTAG\_CTL register.

If the VLAN tag of a packet matches the perfect or hash filter, the packet is dropped. If the VLAN perfect and VLAN hash match are enabled, a packet is considered as matched if either the VLAN hash or the VLAN perfect filter matches. When inverse matching is set, a packet is forwarded only when both perfect and hash filters indicate mismatch.

The VLAN Match Status table shows the different possibilities for VLAN matching and the final VLAN match status. When the EMAC\_MACPKT\_FILT.RA bit is set, all packets are received, and the VLAN match status is indicated in the VF bit of RDES2 normal descriptor (write-back format). When the RA bit is not set and the

EMAC\_MACPKT\_FILT.VTFE bit is set, the packet is dropped if the final VLAN match status is fail. In the following table, X signifies that entry can have any value.

When VLAN VID is programmed to 0 in the EMAC\_VLANTAG\_DAT.VID bit field, all VLAN-tagged packets are considered as perfect matched but the status of the VLAN hash match depends on the EMAC\_VLANTAG\_CTL.VTHM and EMAC\_VLANTAG\_CTL.VTIM bits.

Table 30-43: VLAN Match Status

| VID      | VLAN Perfect Fil- ter Match Result   | VTHM Bit   | VLAN Hash Filter Match Result   | VTIM Bit   | Final VLAN Match Status   |
|----------|--------------------------------------|------------|---------------------------------|------------|---------------------------|
| VID = 0  | Pass                                 | 0          | X                               | X          | Pass                      |
| VID = 0  | Pass                                 | 1          | X                               | 0          | Pass                      |
| VID = 0  | Pass                                 | 1          | Fail                            | 1          | Pass                      |
| VID = 0  | Pass                                 | 1          | Pass                            | 1          | Fail                      |
| VID! = 0 | Pass                                 | X          | X                               | 0          | Pass                      |
| VID! = 0 | Fail                                 | 0          | X                               | 0          | Fail                      |
| VID! = 0 | Fail                                 | 1          | Fail                            | 0          | Fail                      |
| VID! = 0 | Fail                                 | 1          | Pass                            | 0          | Pass                      |
| VID! = 0 | Fail                                 | 0          | X                               | 1          | Pass                      |
| VID! = 0 | Pass                                 | X          | X                               | 1          | Fail                      |
| VID! = 0 | Fail                                 | 1          | Pass                            | 1          | Fail                      |
| VID! = 0 | Fail                                 | 1          | Fail                            | 1          | Pass                      |

## Extended Receive VLAN Filtering and Routing

The MAC receiver can classify the received packets based on a VLAN tag and steer them to a specific Rx DMA channel. The MAC compares the received frame's VLAN tag with all the enabled and relevant filters and provides a filtering result. If any of the perfect filters give a pass result and if the respective filter's DMA channel number is enabled, the frame is routed to that DMA channel.

In addition to filtering, routing can also be done. For more details about routing, see Extended VLAN Based DMA Selection in Dynamic Mapping.

## Comparison Modes

For each VLAN tag filter, the application has the following comparison options:

- It can program the MAC to compare an outer VLAN tag or an inner VLAN tag with the programmed VID.
- It can choose if 12 or 16 bits of the VID field need to be compared.
- Type check can be disabled or enabled for each filter; if enabled, the application can choose if the VID comparison is for SVLAN or CVLAN type frames only. For example, if a filter is enabled for 16-bit comparison,

SVLAN type, and outer VLAN tag, any single or double VLAN tagged frames with outer SVLAN tags are compared with this filter, and a pass or fail result is obtained.

- NOTE: The inner VLAN tag comparison is applicable only if double VLAN tag processing is enabled using the EMAC\_VLANTAG\_CTL.EDVLP bit.

## Filtering

When extended receive VLAN filtering and routing is enabled, the application can enable both perfect and hash filtering. The overall VLAN filter result is based on the perfect filter result and the hash filter result (if enabled). The filter result is passed to the application as part of the status bits.

Perfect filtering is done using the EMAC\_MAC\_VLAN\_TAG\_FILTER[n] registers (these registers are accessed indirectly using the EMAC\_VLANTAG\_DAT and EMAC\_VLANTAG\_CTL registers). For each VLAN tag filter, the MAC compares the relevant VLAN tag ID and gives a result. If any one of the VLAN tag filters gives a match, the frame is considered to have passed the VLAN tag filters. If the frame mismatches all the filters, the frame is considered to have failed the VLAN filter. This behavior is applicable only when the inverse filtering is not enabled in the EMAC\_VLANTAG\_CTL register.

If inverse filtering is enabled and the frame has mismatched all the relevant filters, then it is considered to have passed the VLAN filter. If the frame matches any one of the relevant filters, then it is considered as a fail. If none of the enabled filters can perform a comparison or if none of the filters are enabled, then the frame is bypassed to the application.

The overall filter result and the values programmed in the EMAC\_MACPKT\_FILT.VTFE and EMAC\_MACPKT\_FILT.RA fields determine if the frame must be dropped or forwarded to the application.

- When RA = 1 or VTFE = 0, The MAC forwards the frame, irrespective of the filter result.
- When RA = 0 and VTFE = 1, The MAC forwards the frame only if the VLAN tag filter status is pass. After forwarding a frame to the application, the relevant filter result is indicated through the status bits.

See Programming Extended VLAN Filtering and Routing on Receive.

## VLAN Filter Status

The extended receive VLAN filtering and routing feature provides two status bits to indicate the comparison result of the VLAN tags.

By default, the MAC indicates the VLAN filter status through one bit in the status - VF in RDES2. When extended receive VLAN filtering and routing is enabled, two status bits are used to indicate the comparison result of VLAN tags. The outer VLAN tag filter pass and inner VLAN tag filter pass bits are defined in the following positions in various configurations. The status indicated through these bits is updated in RDES2 as follows:

- Bit 15 - Outer VLAN tag filter status
- Bit 14 - Inner VLAN tag filter status

## Outer VLAN Tag Filter Status (OTS)

The outer VLAN tag filter pass bit provides the following status:

- In perfect filtering, without inverse filtering enabled, if this bit is set, it indicates that the frame's outer VLAN tag has matched one of the VLAN tag filters.
- When this bit is reset, it indicates that the frame's outer VLAN tag has either failed the relevant outer VLAN tag filters or bypassed them.
- When none of the filters are enabled for outer VLAN tag comparison, then this bit is reset.
- When Inverse filtering is enabled and this bit is set, then the frame's VLAN tag has passed all the relevant VLAN tag filters. If it is reset, then it has failed at least one of or bypassed all the filters programmed for outer VLAN tag comparison.
- This bit is valid for both single and double VLAN tagged frames.

## Inner VLAN Tag Filter Status (ITS)

The inner VLAN tag filter pass bit provides the following status:

- In perfect filtering, without inverse filtering enabled, if this bit is set, it indicates that the frame's inner VLAN tag has matched one of the VLAN tag filters.
- When this bit is reset, it indicates that the frame's inner VLAN tag has either failed the relevant inner VLAN tag filters or bypassed them.
- When none of the filters are enabled for inner VLAN tag comparison, then this bit is reset.
- When inverse filtering is enabled and this bit is set, then the frame's VLAN tag has passed all the relevant VLAN tag filters. When it is reset, then it has failed at least one of or bypassed all the filters programmed for inner VLAN tag comparison.
- This bit is valid for only double VLAN tagged frames, when double VLAN processing is enabled.

The application must look at the status bits and the programming to determine if the frame has passed or failed the VLAN filter.

The OTS and ITS Bit Values with At Least 1 Perfect Filter Enabled table and OTS and ITS Bit Values with Only VLAN Hash Filter Enabled table show the possible filter combinations and the corresponding filter results. These tables explain the scenarios when double VLAN processing and hash VLAN filter are enabled in the design.

In the following tables:

- EMAC\_VLANTAG\_CTL.VTIM : VLAN tag inverse match enable - bit 17.
- HFO: Hash filter enabled for outer VLAN tag comparison - bit 25 (VHTM) in the EMAC\_VLANTAG\_CTL register.
- HFI: hash filter enabled for inner VLAN tag comparison - bit 27 (ERIVLT) in the EMAC\_VLANTAG\_CTL register.

- PFO - Perfect filter comparison enabled for outer VLAN tag - Any of the EMAC\_MAC\_VLAN\_TAG\_FIL-TER registers is enabled (bit 16 is set) and programmed for outer VLAN tag comparison (bit 20 is set to 0).
- PFI - Perfect filter comparison enabled for inner VLAN tag - Any of the EMAC\_MAC\_VLAN\_TAG\_FIL-TER registers is enabled (bit 16 is set) and programmed for inner VLAN tag comparison (bit 20 is set to 1).
- OTS - Outer VLAN tag filter status
- ITS - Inner VLAN tag filter status

The following table shows the possible values of status bits (OTS and ITS) when at least one perfect filter is enabled.

Table 30-44: OTS and ITS Bit Values with At Least 1 Perfect Filter Enabled

|   VTIM |   HFO |   HFI |   PFO |   PFI | OTS   | ITS   |
|--------|-------|-------|-------|-------|-------|-------|
|      0 |     0 |     0 |     0 |     1 | 0     | 1/0   |
|      0 |     0 |     0 |     1 |     0 | 1/0   | 0     |
|      0 |     0 |     0 |     1 |     1 | 1/0   | 1/0   |
|      0 |     1 |     0 |     1 |     1 | 1/0   | 1/0   |
|      0 |     1 |     0 |     1 |     0 | 1/0   | 0     |
|      0 |     1 |     0 |     0 |     1 | 1/0   | 1/0   |
|      0 |     0 |     1 |     1 |     1 | 1/0   | 1/0   |
|      0 |     0 |     1 |     1 |     0 | 1/0   | 1/0   |
|      0 |     0 |     1 |     0 |     1 | 0     | 1/0   |
|      1 |     0 |     0 |     0 |     1 | 0     | 1/0   |
|      1 |     0 |     0 |     1 |     0 | 1/0   | 0     |
|      1 |     0 |     0 |     1 |     1 | 1/0   | 1/0   |
|      1 |     1 |     0 |     1 |     1 | 1/0   | 1/0   |
|      1 |     1 |     0 |     1 |     0 | 1/0   | 0     |
|      1 |     1 |     0 |     0 |     1 | 1/0   | 1/0   |
|      1 |     0 |     1 |     1 |     1 | 1/0   | 1/0   |
|      1 |     0 |     1 |     1 |     0 | 1/0   | 1/0   |
|      1 |     0 |     1 |     0 |     1 | 0     | 1/0   |

The following table shows the possible values of status bits (OTS and ITS) when none of the perfect filters are enabled and only the VLAN hash filter is enabled.

Table 30-45: OTS and ITS Bit Values with Only VLAN Hash Filter Enabled

|   VTIM |   HFO |   HFI |   OTS |   ITS |
|--------|-------|-------|-------|-------|
|      0 |     0 |     0 |     0 |     0 |

Table 30-45: OTS and ITS Bit Values with Only VLAN Hash Filter Enabled (Continued)

|   VTIM |   HFO |   HFI | OTS   | ITS   |
|--------|-------|-------|-------|-------|
|      0 |     1 |     0 | 1/0   | 0     |
|      0 |     0 |     1 | 1/0   | 1/0   |
|      1 |     0 |     0 | 1/0   | 0     |
|      1 |     1 |     0 | 1/0   | 0     |
|      1 |     0 |     1 | 1/0   | 1/0   |

- When no perfect filters are enabled, any VLAN packet is considered to have bypassed the perfect filter.
- When VLAN hash filter is enabled for one of the tags, the respective status bit depends on the result of the filter. Status bits are set to 0 when VLAN hash filter is not enabled.
- The value 1/0 for the ITS/OTS field indicates that the final result is dependent on the result of the enabled relevant filter.

Example 1: The second row of the above table indicates that at least one perfect filter is enabled for outer VLAN tag comparison and none of the filters are enabled for Inner VLAN tag comparison. Inverse VLAN filtering is not enabled. The bit OTS is given as 1/0. When the received frame passes at the least one of the enabled Outer VLAN Tag filters, then the bit is set to 1. When the frame does not pass any of the enabled Outer VLAN Tag filters, then the bit is set to 0.

Example 2: Last row of the above table indicates that inverse filtering is enabled, Hash filter and at least one perfect filter is enabled for inner VLAN tag comparison, then when the received frame's inner VLAN tag mismatches with both the hash filter and all the enabled perfect filters, then the frame has the ITS bit set to 1. Otherwise, the bit is set to 0. The OTS bit is set to 0 as the comparison is not performed.

## VLAN Stripping

Each of the VLAN tags has individual control over stripping. The programming options of always strip, never strip, strip on pass and strip on fail are available. Inner or outer VLAN tag stripping is based on the pass or fail results of the individual tag. If a tag is bypassed by all the relevant filters, stripping is not applicable for the tag.

- If strip on pass is enabled for the outer VLAN tag, stripping is done only if the outer VLAN tag has passed the relevant filters. The outer VLAN tag filter result bit is set.
- If strip on fail is enabled for the outer VLAN tag, stripping is done only if the outer VLAN tag has failed relevant filters. The Outer VLAN tag filter result bit is reset.
- If the Outer VLAN tag of the received frame is bypassed by the entire filter (no comparison has been made), the tag is not stripped, though the status bit is still 0.
- As multiple filters are enabled, it is possible that the received VLAN frame matches more than one filters. The VLAN tag's value is not always deterministic from the filter status bits.

- If the application strips the VLAN tag based on the filter result, it might lose the VID. If stripping is enabled for any of the tags, the tag can be put in the status. The application must enable the respective EMAC\_VLANTAG\_CTL.EVLRXS or EMAC\_VLANTAG\_CTL.EIVLRXS bits.

## VLAN Filter Fail Packets Queue

When VLAN filtering is enabled, the VLAN filter fail packets can be routed to a programmable queue ( EMAC\_RXQ\_CTL4.VFFQ ) when the EMAC\_MACPKT\_FILT.RA bit = 1 or EMAC\_MACPKT\_FILT.VTFE = 0 and the enable bit ( EMAC\_RXQ\_CTL4.VFFQE ) for the queue is set.

The packets that pass the VLAN filtering are routed based on the VLAN TAG priority field. The VLAN tag priorities can be assigned to receive queues by programming the PSRQ field in the corresponding EMAC\_RXQ\_CTL2 and EMAC\_RXQ\_CTL3 registers. The packets that fail the VLAN filter are discarded if EMAC\_MACPKT\_FILT.RA =0 or EMAC\_MACPKT\_FILT.VTFE =1. However, when RA=1 or VTFE=0, the bit. If VFFQE=0, the receive queue number is determined

VLAN filter fail packets are still forwarded to the application. In such a scenario, when the VLAN filter fail queue enable ( EMAC\_RXQ\_CTL4.VFFQE ) bit is set, the VLAN filter fail packets are forwarded to the receive queue number programmed in the EMAC\_RXQ\_CTL4.VFFQ by the VLAN priority mapping as per the PSRQ fields.

The Receive Queue Routing Table for Unicast T agged Packets shows the receive queue routing table for Unicast tagged packets, with the DA/SA filter enabled.

In the following table X = don't care; * = when UFFQE is enabled else PSRQ

Table 30-46: Receive Queue Routing Table for Unicast Tagged Packets

| RA   | VFTE   | SA/DA Filter Result   | VLAN Filter Result   | VFFQE   | Queue Routing   |
|------|--------|-----------------------|----------------------|---------|-----------------|
| X    | X      | Pass                  | Pass                 | X       | PSRQ            |
| 0    | 0      | Pass                  | Fail                 | 0       | PSRQ            |
| 0    | 0      | Pass                  | Fail                 | 1       | VFFQ            |
| 0    | X      | Fail                  | X                    | X       | Dropped         |
| 0    | 1      | Pass                  | Fail                 | X       | Dropped         |
| 1    | X      | Fail                  | X                    | 0       | UFFQ*/PSRQ      |
| 1    | X      | Fail                  | X                    | 1       | UFFQ*/PSRQ      |
| 1    | X      | Pass                  | Fail                 | 0       | PSRQ            |
| 1    | X      | Pass                  | Fail                 | 1       | VFFQ            |

## Layer 3 and Layer 4 Frame Filtering

The MAC supports layer 3 and layer 4 based frame filtering. The layer 3 filtering refers to the IP source or destination address filtering in the IPv4 or IPv6 frames whereas layer 4 filtering refers to the source or destination port number filtering in TCP or UDP .

When layer 3 and layer 4 filtering is enabled, the frames are filtered in the following way:

Matched Packets. The MAC forwards the packets that match all enabled fields to the application along with the status. The MAC gives the matched field status only if the EMAC\_MAC\_CFG.IPC bit is set and one of the following conditions is true:

- All enabled layer 3 and layer 4 fields match
- At least one of the enabled fields match and other fields are bypassed or disabled

When multiple layer 3 and layer 4 filters are enabled, any filter match is considered as a match. IWhen more than one filter matches, the MAC provides the status of the lowest filter where filter 0 is the lowest filter and filter 3 is the highest filter. For example, when filter 0 and filter 1 match, the MAC gives the status corresponding to filter 0.

- NOTE: The source or destination address and VLAN tag filters (if enabled) have precedence over layer 3 and layer 4 filter. This means that a packet which fails the source or destination address, or VLAN tag filter is dropped irrespective of the layer 3 and layer 4 filter results.

Unmatched Packets. The MAC drops the packets that do not match any of the enabled fields. Programs can use the inverse match feature to block or drop a packet with specific TCP or UDP over IP fields and forward all other packets. When a packet is dropped, the aborted or partial packets can be dropped in the MTL Rx FIFO. If the Rx FIFO operates in the threshold (cut-through) mode and the threshold is programmed to a small value, such that packet transfer to application starts before the failed layer 3 and layer 4 filter results are available, the application may receive a partial packet with appropriate abort status.

Non-TCP or UDP IP Packets. By default, all non-TCP or UDP IP packets are bypassed from the layer 3 and layer 4 filters. The program can optionally program the MAC to drop all non-TCP or UDP over IP packets.

## Layer 3 Filtering

The EMAC supports perfect matching or inverse matching for IP source address and destination address. The matching compares all bits of the address except the specified lower mask bits.

For IPv6 packets filtering, the program can enable the last four data registers of a register set to contain the 128-bit IP source address or IP destination address. The IP source address or destination address should be programmed in the order defined in the IPv6 specification, that is, the first byte of the IP source address or destination address in the received packet is in the higher byte of the register and the subsequent registers follow the same order.

For IPv4 packet filtering, the program can enable the second and third data registers of a register set to contain the 32-bit IP source address and IP destination address. The remaining two data registers are reserved. The IP source address or destination address should be programmed in the order defined in the IPv4 specification, that is, the first byte of IP source address and destination address in the received packet in the higher byte of the respective register.

## Layer 4 Filtering

The EMAC supports perfect matching or inverse matching for TCP or UDP source and destination port numbers. However, only one type (TCP or UDP) can be programmed at a time. The first data register contains the 16-bit source and destination port numbers of TCP or UDP , that is, the lower 16 bits for source port number and higher 16 bits for destination port number.

The TCP or UDP source and destination port numbers should be programmed in the order defined in the TCP or UDP specification, that is, the first byte of TCP or UDP source and destination port number in the received packet is in the higher byte of the register.

## Layer 3 and Layer 4 Filters Registers

The MAC implements a set of registers for layer 3 and layer 4 based frame filtering. In a register set, there is a control register, such as the EMAC\_L3L4\_CTL[i] register (layer 3 and layer 4 control register), to control the frame filtering. In addition, there are five address registers to program the layer 3 and layer 4 fields to be matched, which are:

- EMAC\_L4\_ADDR[i] (layer 4 Address Register)
- EMAC\_L3\_ADDR0\_REG[i] (layer 3 address 0 register)
- EMAC\_L3\_ADDR1\_REG[i] (layer 3 address 1 register)
- EMAC\_L3\_ADDR2\_REG[i] (layer 3 address 2 register)
- EMAC\_L3\_ADDR3\_REG[i] (layer 3 address 3 register)

## EMAC Station Management Interface (SMI)

The IEEE 802.3 MII station management interface, also known as the MDIO management interface, allows the processor to monitor and control one or more external Ethernet physical-layer transceivers. (Physical-layer transceivers are commonly called PHYs). The management interface physically consists of a 2-wire serial connection composed of the MDC (management data clock) output signal and the MDIO (management data input/output) bidirectional data signal. The IEEE 802.3 MII station management interface applies to both MII/RMII.

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

Table 30-47: Station Management Interface pins

| Station Management Interface Pins   | Pin Description                                                                                         |
|-------------------------------------|---------------------------------------------------------------------------------------------------------|
| MDIO - Management Data I/O          | A periodic clock that runs at a maximum period of 400 ns. Always driven by the EMAC to PHY.             |
| MDC-Management Data Clock           | Data signal driven by EMAC or PHY, depending on write or read access based on EMAC; synchronous to MDC. |

## MDC Clock Frequency

The EMAC uses the EMAC\_MDIO\_ADDR.CR bit field to determine the frequency of MDC as shown in the MDC Clock Frequency Selection table. The clock range selection determines the frequency of the clock relative to the CLKO7 frequency. The table shows the suggested range of CLKO7 frequency applicable for each value of the EMAC\_MDIO\_ADDR.CR field. The programmability based on CLKO7 frequency range ensures that the MDC clock frequency range is within the IEEE specifications of 1.0 MHz to 2.5 MHz. However, the EMAC MDC can also support higher frequencies for PHY devices that support the frequencies.

Table 30-48: MDC Clock Frequency Selection

|   Selection | Programmed Frequency Range   | Frequency of MDC   | Min and Max MDCFreq (Per Specifications)   |
|-------------|------------------------------|--------------------|--------------------------------------------|
|        0000 | 60-100 MHz                   | CLKO7/42           | MIN = 1.43 MHz and MAX = 2.39 MHz          |
|        0001 | 100-150 MHz                  | CLKO7/62           | MIN = 1.61 MHz and MAX = 2.42 MHz          |
|        0010 | 20-35 MHz                    | CLKO7/16           | MIN = 1.25 MHz and MAX = 2.19 MHz          |
|        0011 | 35-60 MHz                    | CLKO7/26           | MIN = 1.35 MHz and MAX = 2.31 MHz          |

The MDIO Frame Parameters table provides MDIO data transfer parameters. The write and read sequences provided in the tables, MDIO Write Data Sequence and MDIO Read Data Sequence , are based on these parameters.

Table 30-49: MDIO Frame Parameters

| Parameter   | Description                                                                      |
|-------------|----------------------------------------------------------------------------------|
| IDLE        | The MDIO line is three-state (noted as Z in sequence); there is no clock on MDC. |
| PREAMBLE    | 32 continuous bits, each of value 1                                              |
| START       | Start of frame is 01                                                             |
| OPCODE      | 10 for read and 01 for write                                                     |
| PHY ADDR    | 5-bit address select for one of 32 PHYs (noted as AAAAA in sequence)             |
| REG ADDR    | Register address in the selected PHY (noted as RRRRR in sequence)                |

Table 30-49: MDIO Frame Parameters (Continued)

| Parameter   | Description                                                                     |
|-------------|---------------------------------------------------------------------------------|
| TA          | Turnaround is Z0 for read and 10 for write (Z = high impedance)                 |
| DATA        | Any 16-bit value. Driven by MAC or PHY based on direction (noted as DDD...DDD). |

Table 30-50: MDIO Write Data Sequence

| IDLE   | PREAMBLE   |   START |   OPCODE | PHY ADDR   | REG ADDR   |   TA | DATA       | IDLE   |
|--------|------------|---------|----------|------------|------------|------|------------|--------|
| Z      | 111...111  |      01 |       01 | AAAAA      | RRRRR      |   10 | DDD... DDD | Z      |

Table 30-51: MDIO Read Data Sequence

| IDLE   | PREAMBLE   |   START |   OPCODE | PHY ADDR   | REG ADDR   | TA   | DATA       | IDLE   |
|--------|------------|---------|----------|------------|------------|------|------------|--------|
| Z      | 111...111  |      01 |       10 | AAAAA      | RRRRR      | Z0   | DDD... DDD | Z      |

## MDIO Frame Structure

Both Clause 22 and Clause 45 MDIO frame structures are supported by the EMAC. The EMAC\_MDIO\_ADDR.C45E bit can be programmed to enable Clause 22 or Clause 45 mode of operation. The MDIO Clause 45 Frame Structure table shows the Clause 45 frame format and the MDIO Clause 22 Frame Structure table shows the Clause 22 frame format. In addition to normal read and write operations, the SMA also supports post-read increment address while operating in Clause 45 mode.

Table 30-52: MDIO Clause 45 Frame Structure

| Field        | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| IDLE         | The MDIO line is in three-state; there is no clock onMDC                                                                                                                                                                                                                                                                                                                                                                                                              |
| PREAMBLE     | 32 continuous bits of value 1                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| START        | Start of packet is 2'b00                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| OPCODE       | 00/01/10/11                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| PHY ADDR     | 5-bit address select for one of 32 PHYs                                                                                                                                                                                                                                                                                                                                                                                                                               |
| DEV ADDR     | 5-bit address select for one of 32 devices                                                                                                                                                                                                                                                                                                                                                                                                                            |
| TA           | Turnaround is Z0 for read and 10 for write (Z = high impedance)                                                                                                                                                                                                                                                                                                                                                                                                       |
| DATA/ADDRESS | 16-bit value. For an address cycle (OPCODE = 2'b00), this frame contains the address of the register to be accessed on the next cycle. For the data cycle of a write frame, this field contains the data that is written to the register. For read or post-read increment address frames, this field contains the contents of the register read from the PHY. • In address and data write cycles, the EMAC drives the MDIO line during the transfer of these 16 bits. |

Table 30-52: MDIO Clause 45 Frame Structure (Continued)

| Field   | Description                                                                                                          |
|---------|----------------------------------------------------------------------------------------------------------------------|
|         | • In read and post-read increment address cycles, the PHY drives the MDIO line during the transfer of these 16 bits. |

Table 30-53: MDIO Clause 22 Frame Structure

| Field    | Description                                                                     |
|----------|---------------------------------------------------------------------------------|
| IDLE     | The MDIO line is in tri-state; there is no clock onMDC                          |
| PREAMBLE | 32 continuous bits of value 1                                                   |
| START    | Start of packet is 2'b01                                                        |
| OPCODE   | 10 for read and 01 for write                                                    |
| PHY ADDR | 5-bit address select for one of 32 PHYs                                         |
| DEV ADDR | 5-bit address to select the register within eachMMD                             |
| TA       | Turnaround is Z0 for read and 10 for write (Z = high impedance)                 |
| DATA     | Any 16-bit value. Driven by MAC or PHY based on direction (noted as DDD...DDD). |

Table 30-54: MDIO Write Data Sequence

| IDLE   | PREAMBLE   |   START |   OPCODE | PHY ADDR   | REG ADDR   |   TA | DATA       | IDLE   |
|--------|------------|---------|----------|------------|------------|------|------------|--------|
| Z      | 111...111  |      01 |       01 | AAAAA      | RRRRR      |   10 | DDD...DD D | Z      |

Table 30-55: MDIO Read Data Sequence

| IDLE   | PREAMBLE   |   START |   OPCODE | PHY ADDR   | REG ADDR   | TA   | DATA       | IDLE   |
|--------|------------|---------|----------|------------|------------|------|------------|--------|
| Z      | 111...111  |      01 |       01 | AAAAA      | RRRRR      | Z0   | DDD...DD D | Z      |

## SMI Write Operation

When programs set the EMAC\_MDIO\_ADDR.GOC\_1 / EMAC\_MDIO\_ADDR.GOC\_0 bit field to 2'b01 and the EMAC\_MDIO\_ADDR.GB bit to 1, the MAC CSR module transfers the PHY address, the register address in PHY, and the write data ( EMAC\_MDIO\_DATA register) to the SMI to initiate a write operation into the PHY registers. At this point, the SMI module starts a write operation on the GMII Management Interface using the management packet format specified in the GMII specifications (as per IEEE 802.3-2002, Section 22.2.4.5).

When the SMA module starts a write operation, the write data packet is transmitted on the MDIO line. The MAC drives the MDIO line for complete duration of the packet. The EMAC\_MDIO\_ADDR.GB bit is set high until the write operation is complete. The CSR ignores the write operations performed to the EMAC\_MDIO\_ADDR register or the EMAC\_MDIO\_DATA register during this period (the EMAC\_MDIO\_ADDR.GB bit is high). When the write operation is complete, the SMI module indicates this to the CSR, and the CSR resets the EMAC\_MDIO\_ADDR.GB bit.

The SMI Write Operation through MDIO/MDC Pins figure shows this operation.

Figure 30-27: SMI Write Operation through MDIO/MDC Pins

<!-- image -->

## SMI Read Operation

When programs set the EMAC\_MDIO\_ADDR.GOC\_1 / EMAC\_MDIO\_ADDR.GOC\_0 bit field to 2'b11 and the EMAC\_MDIO\_ADDR.GB bit to 1, the MAC CSR module transfers the PHY address and the register address in PHY to the SMI to initiate a read operation in the PHY registers. The SMI starts a read operation on the GMII Management Interface using the management packet format specified in the GMII specifications (as per IEEE 802.3-2002, Section 22.2.4.5).

When the SMI starts a read operation on the MDIO, the CSR ignores the write operations to the EMAC\_MDIO\_ADDR or EMAC\_MDIO\_DATA registers during this period (the EMAC\_MDIO\_ADDR.GB bit is high) and the transaction is completed without any error. When the read operation is complete, the SMI indicates this to the CSR. The CSR resets the GB bit and updates the EMAC\_MDIO\_DATA register with the data read from the PHY.

The SMI Read Operation through MDIO/MDC Pins figure shows this operation.

Figure 30-28: SMI Read Operation through MDIO/MDC Pins

<!-- image -->

## Preamble Suppression

The IEEE standard specifies 32-bit preamble (all-ones) for the MDIO frames. The EMAC provides controls to support preamble suppression. EMAC transmits MDIO frames with only 1 preamble bit. The preamble suppression can be enabled by setting the EMAC\_MDIO\_ADDR.PSE bit.

## Trailing Clocks and Back-to-Back Transactions

The EMAC drives MDC clock on the MDC port for duration of the MDIO frame. There is no clock driven during the idle period. The trailing clocks feature can be used if the PHY needs the MDC clock to be active for some cycles after the MDIO frame. The EMAC\_MDIO\_ADDR.NTC bit field allows programming of trailing clocks from 0 to 7.

The EMAC supports back to back transactions which allows start of next MDIO frame even before the trailing clocks are completed for previous MDIO frame. This feature can be enabled by setting the EMAC\_MDIO\_ADDR.BTB bit when trailing clocks feature is also enabled. When back to back transactions is enabled, the GMII busy is cleared immediately after MDIO frame completion. This allows the software to issue the next command, which is executed by EMAC while trailing clocks are still on for the previous MDIO frame. When back to back transactions is not enabled, the GMII busy is cleared after the trailing clocks are completed for MDIO frame.

## Interrupt for MDIO Transaction Completion

The EMAC can generate an interrupt on completion of MDIO read or write transactions. Therefore, the application need not poll the GMII busy field of the EMAC\_MDIO\_ADDR register to know the completion of MDIO commands.

## EMAC Management Counters (MMC)

The EMAC provides a comprehensive set of 32-bit MAC management counters. It uses these counters for gathering statistics on the received and transmitted frames. The MMC sub block also includes

- A control register ( EMAC\_MMC\_CTRL ) for managing the behavior of the counters
- Two 32-bit registers containing interrupts generated ( EMAC\_MMC\_RXINT and EMAC\_MMC\_TXINT )
- Two 32-bit registers containing masks for the interrupt register ( EMAC\_MMC\_RXIMSK and EMAC\_MMC\_TXIMSK )

The MMC receive counters are updated for frames passed by the address filtering sub block in the EMAC CORE. Statistics of frames dropped by the AFM module are not updated unless they are runt frames of less than 6 bytes. (Destination address bytes are not received fully.) The module is also capable of gathering statistics on encapsulated IPv4, IPv6, and TCP , UDP , or ICMP payloads in received Ethernet frames.

The MMC register naming conventions are as follows:

- Tx as a prefix or suffix indicates counters associated with transmission.
- Rx as a prefix or suffix indicates counters associated with reception.
- G as a suffix indicates registers that count good frames only.
- GB as a suffix indicates registers that count frames regardless of whether they are good or bad.

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

- Untagged frame maximum size = 1518
- VLAN frame maximum size = 1522
- Jumbo frame maximum size = 9018
- Jumbo VLAN frame maximum size = 9022

The maximum receive packet size depends on the packet type and control bits (JE, S2KP , GPSLCE and EDVLP), as shown in the Size of the Maximum Receive Packet table.

Table 30-56: Size of the Maximum Receive Packet

|   JE | S2KP   | GPSLCE   | EDVLP   | Untagged Frame maximum size in bytes   | Single VLAN Frame maximum size in bytes   | Double VLAN Frame maximum size in bytes   |
|------|--------|----------|---------|----------------------------------------|-------------------------------------------|-------------------------------------------|
|    1 | X      | X        | 1       | 9018                                   | 9022                                      | 9026                                      |
|    0 | 1      | X        | X       | 2000                                   | 2000                                      | 2000                                      |
|    0 | 0      | 1        | 1       | GPSL                                   | GPSL+4                                    | GPSL+8                                    |
|    0 | 0      | 0        | 1       | 1518                                   | 1522                                      | 1526                                      |
|    1 | X      | X        | 0       | 9018                                   | 9022                                      | 9022                                      |
|    0 | 0      | 1        | 0       | GPSL                                   | GPSL+4                                    | GPSL+4                                    |
|    0 | 0      | 0        | 0       | 1518                                   | 1522                                      | 1522                                      |

The EMAC\_MMC\_CTRL register also contains bits that control preset, freeze and roll-over of counters. The EMAC uses the EMAC\_MMC\_CTRL.RSTONRD bit to enable an auto-reset feature whenever the counters are read. The EMAC uses the EMAC\_MMC\_CTRL.CNTRST bit to reset all the counters.

The MMC can trigger an interrupt when the corresponding bits are enabled in the transmit, receive, and IPC mask registers, and when the counter reaches half or full. The status is also updated in the corresponding interrupt register.

## MMC Receive Interrupt Register

The EMAC\_MMC\_RXINT register maintains the interrupts that are generated when the receive statistic counters reach half their maximum values (0x80000000), and when they cross their maximum values (0xFFFFFFFF). When EMAC\_MMC\_CTRL.CNTSTOPRO bit is set, then interrupts are set, but the counter remains at all ones. The EMAC\_MMC\_RXINT register is a 32-bit wide register. An interrupt bit is cleared when the respective MMC counter that caused the interrupt is read. The least significant byte lane (bits 7:0) of the respective counter must be read to clear the interrupt bit.

## MMC Transmit Interrupt Register

The EMAC\_MMC\_TXINT register maintains the interrupts generated when the transmit statistic counters reach half their maximum values (0x80000000), and when they cross their maximum values (0xFFFFFFFF). When EMAC\_MMC\_CTRL.CNTSTOPRO is set ( = 1), then interrupts are set, but the counter remains at all ones. The EMAC\_MMC\_TXINT register is a 32-bit wide register. An interrupt bit is cleared when the respective MMC counter that caused the interrupt is read. The least significant byte lane (bits 7:0) of the respective counter must be read to clear the interrupt bit.

## MMC Receive Checksum Offload Interrupt Register

The EMAC\_MMC\_IPC\_RXINT register maintains the interrupts generated when receive IPC statistic counters reach half their maximum values (0x80000000), and when they cross their maximum values (0xFFFFFFFF). When the EMAC\_MMC\_CTL.NOROLL is set, then interrupts are set, but the counter remains at all ones. The EMAC\_MMC\_IPC\_RXINT register is 32 bits wide. When the MMC IPC counter that caused the interrupt is read, its corresponding interrupt bit is cleared. The least-significant byte lane (bits 7:0) of the counter must be read to clear the interrupt bit.

## EMAC Precision Time Protocol (PTP) Engine

The following sections describe the precision time protocol (PTP) engine.

## IEEE1588 and the PTP Engine

The Ethernet MAC peripheral includes a PTP engine to assist applications requiring time synchronization. The PTP module is tightly integrated with the EMAC CORE to aid hardware time stamping defined in the IEEE1588 2002/2008 standards. Applications can use accurate hardware time stamps through TCP/IP stacks (if using network layer communication) to exchange time information across devices connected over network. Applications can also use accurate hardware time stamps through Ethernet device drivers (if using MAC layer communication) to exchange time information.

## PTP Engine

For calculation of drift in time between two Ethernet devices, the device records its system time whenever a timing message is sent or received (IEEE 1588 protocol). Due to the indeterministic delay of a software system for a node, the software is unable to capture an accurate time when the message is sent or received. However, the hardware can monitor the signal on the communication media and get an accurate message of arrival and departure time.

The PTP (precision time protocol) module is closely integrated with the EMAC module. It provides hardware assistance to implement both the IEEE 1588-2002 and IEEE 1588-2008 standards on Ethernet (IEEE 802.3). It takes

one input clock signal as its PTP clock and maintains the timing information (called system time ) at the nanosecond level.

The PTP module includes hardware for clock and system time adjustment. The pulse-per-second (PPS) signals physically represent the system time. PPS can be programmed to a fixed frequency or provide flexibility to the signal in terms of pulse width, interval, start, and stop time of the signals. The PTP module can be programmed to trigger an alarm interrupt when system time reaches specified time.

The PTP module can be programmed to detect different types of received frames, capture the system time, and time stamp those frames with the captured system time. Programs can configure any frame so that the PTP module capture the system time when it is transmitted. The PTP module can also capture the system time when an event is detected on the auxiliary snapshot trigger input pins ( EMAC\_PTPAUX\_MCG\_IN[n] ).

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

Figure 30-29: IEEE 1588-2002 PTP Process

<!-- image -->

As shown in the figure, the PTP uses the following process:

1. The requester broadcasts the PTP sync messages to all its nodes. The sync message contains the reference time information of the requester. The time at which this message leaves the system of the requester is t 1 . The requester must capture this time for Ethernet ports, at RMII.
2. The completer receives the Sync message and also captures the exact time, t 2 , using its timing reference.
3. The requester sends a Follow\_up message to the completer, which contains t1 information for later use.
4. The completer sends a Delay\_Req message to the requester, noting the exact time, t 3 , at which this frame leaves the RMII.
5. The requester receives the message, capturing the exact time, t 4 , at which it enters its system.
6. The requester sends the t 4  information to the completer in the Delay\_Resp message.
7. The completer uses the four values of t 1 , t 2 , t 3 , and t 4  to synchronize its local timing reference to the timing reference of the requester.

Most of the PTP implementation occurs in the software above the UDP layer. However, the hardware support must capture the exact time when specific PTP packets enter or leave the Ethernet port at the RMII. Hardware must capture this timing information and return it to the software for the proper implementation of PTP with high accuracy.

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

Figure 30-30: Propagation Delay Calculation between Nodes Supporting P2P Path Correction

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

Figure 30-31: PTP Block Diagram

<!-- image -->

A system time module is present which keeps the time of PTP module. It consists of hardware which can be programmed for time initialization, time correction, and clock correction.

The time stamp module captures the time (provided by the system time module) at various conditions. For example, when the EMAC sends or receives a frame or during the rising edge of the auxiliary snapshot trigger EMAC\_PTPAUX\_MCG\_IN[n] pins. When system time is captured after detection of a frame, the time stamp module automatically includes the time information in the frame descriptor. Time stamping on the detection of a frame can be programmed on a per frame basis.

The PTP clock drives the PTP module. This clock can be selected from three different clock sources.

The Pulse per Second (PPS) module generates a pulse or train of pulse on the PPS output pins, ( EMAC\_PTPPPS[n] ). It is the physical representation of system time. PPS can be fixed (where only frequency varies) or flexible (where width, interval, start time, and stop time can be programmed).

The target time module acts as an alarm for the PTP module. Whenever system time reaches a value equal to programmed target time, the target time trigger interrupt is generated. By appropriate programming, The target time trigger can also start or stop flexible PPS output at specific time.

## PTP Module Clock

The PTP module clock features include Clock Source Selection and Clock Frequency Range .

## Clock Source Selection

The PTP module can take one of these clock sources as its input clock - CLKO7, RMII reference RGMII Rx clock, or PTP external clock.

As shown in the PTP Clock Source Selection table, the PADS\_PCFG0 register [bits 1:0] selects the PTP clock source.

Table 30-57: PTP Clock Source Selection

| PADS_PCFG0 [1:0]   | PTP Clock Source               | Clock Description                       |
|--------------------|--------------------------------|-----------------------------------------|
| 00 - RMII or MII   | ETH0_RXCLK_REFCLK/ ETH1_REFCLK | MII/RMII reference clock                |
| 00 - RGMII         | CLKO7                          | Processor Ethernet clock                |
| 10                 | PTP external clock             | Clock available on EMAC_PTPCLKIN[n] pin |
| 01 or 11           | SCLK0                          | Processor system clock                  |

## Clock Frequency Range

The resolution, or granularity, of the reference time source determines the accuracy of the synchronization. Therefore, a higher PTP clock frequency gives better system performance. The timing constraints achievable for logic operating on the selected PTP clock source limit the maximum PTP clock frequency.

The minimum PTP clock frequency depends on the time required between two consecutive frames. The IEEE specification determines the RMII clock frequency. The minimum PTP clock frequency required for proper operation depends on the operating mode and operating speed of the MAC. See the Minimum PTP Clock Frequency table.

A minimum delay required between two consecutive time stamp captures is 8 clock cycles of RMII and 3 clock cycles of PTP clocks. If the delay between two time stamp captures is less than this delay, the EMAC does not take a time stamp snapshot for the second frame.

## The Minimum PTP Clock Frequency table assumes:

- Minimum Ethernet packet size = 64 bytes
- Mininum IFG = 96 bit times or 12 bytes
- Preamble = 8 bytes
- 3 PTP clock + 4 GMII/MII clock minimum gap between two SFDs (start of frame delimiters)
- 3 PTP clock + 8 RMII clock minimum gap between two SFDs

Table 30-58: Minimum PTP Clock Frequency

| Mode (Full Duplex in all cases)   | Minimum Gap between two SFDs                       | PTP clock                                                    | Comments                                                  |
|-----------------------------------|----------------------------------------------------|--------------------------------------------------------------|-----------------------------------------------------------|
| 10 Mbps RMII                      | 64 bytes of data + 8 bytes of pre- amble + min IFG | 3 PTP clock cycle + 8RMII clock cycle 3360 RMII clock cycles | In RMII @10 Mbps, 1 byte trans- mitted in 40 clock cycles |
| 10 Mbps RMII                      | (2560 + 320 + 480) RMII clock cycles               | 3 PTP clock cycle 3352 RMII clock cycles                     | RMII clock = 50 MHz                                       |
| 10 Mbps RMII                      | 3360 RMII clocks                                   | PTP clock cycle 1117.33 20 ns = 22346 ns                     | 1 RMII clock cycle = (1/50 MHz) = 20 ns                   |
| 10 Mbps RMII                      |                                                    | PTP frequency min = 0.045 MHz                                |                                                           |
| 100 Mbps RMII                     | 64 bytes of data + 8 bytes of pre- amble + min IFG | 3 PTP clock cycle + 8RMII clock cycle 336 RMII clock cycles  | In RMII, 1 byte transmitted in 4 clock cycles             |
| 100 Mbps RMII                     | (256 + 32 + 48) RMII clock cy- cles                | 3 PTP clock cycle 328 RMII clock cycles                      | RMII clock = 50 MHz                                       |
| 100 Mbps RMII                     | 336 RMII clocks                                    | PTP clock cycle 109.33 20 ns = 2186 ns                       | 1 RMII clock cycle = (1/50 MHz) = 20 ns                   |
| 100 Mbps RMII                     |                                                    | PTP frequency min = 0.45 MHz                                 |                                                           |

The minimum PTP clock frequency also depends on the maximum value of the EMAC\_TM\_SUBSEC register. Even at the highest EMAC\_TM\_SUBSEC.SSINC value, the EMAC\_TM\_SEC register value can be incremented every second. Since the EMAC\_TM\_SUBSEC.SSINC is an 8-bit field, the minimum PTP clock frequency allowed is approximately 4 MHz.

## Time Stamp Module

The time stamp module captures time in seconds and nanoseconds maintained as system time. The time stamp module also captures time when specific events occur. Events include detection of a frame transmitted or received

over the EMAC and a rising edge on the EMAC\_PTPAUX\_MCG\_IN[n] pins. The time stamp module does not need to time stamp all of the transmitted or received frames over the EMAC. The PTP module can be programmed to detect specific kinds of frames for time stamping.

## Frame Detection and Time Stamping

The PTP module automatically monitors all received and transmitted IEEE 1588 event messages on the Ethernet. If the module detects an event message, it takes a snapshot of the system time. The PTP module stores the value to the 64-bit fields in transmit or receive descriptor.

The time stamping occurs at the EMAC RMII/RGMII interface when the module sees the start of frame of an event message packet.

## Transmit Path Time Stamping

The MAC captures a time stamp when the Start Packet Delimiter (SFD) of a packet is sent on the MII/RMII/ RGMII interface. The packets for which the time stamps must be captured, can be controlled on per-packet basis. Each transmit packet can be marked to indicate whether a time stamp should be captured for it.

The MAC does not process the transmitted packets to identify the PTP packets. The program needs to specify the packets to capture time stamps.

The program can specify the packets by using the control bits in the transmit descriptor. The MAC returns the time stamp to the software inside the corresponding transmit descriptor, connecting the time stamp automatically to the specific PTP packet. The 64-bit time stamp information is written to the TDES0 and TDES1 fields. The TDES0 field holds the 32 least significant bits of the time stamp.

## Receive Path Time Stamping

The MAC can be programmed to capture the time stamp of all packets received on the GMII or MII interface or to process packets to identify the valid PTP messages. Use the following options of the EMAC\_TM\_CTL register to control the snapshot of the time to be sent to the application:

- Enable snapshot for all packets
- Enable snapshot for IEEE 1588 version 1 or version 2 time stamp
- Enable snapshot for PTP packets transmitted directly over Ethernet or UDP-IP-Ethernet
- Enable time stamp snapshot for the received packet for IPv4 or IPv6
- Enable time stamp snapshot only for event messages (Sync, Delay\_Req, Pdelay\_Req, or Pdelay\_Resp)
- Enable the node to be a main or target and select the snapshot type

This feature controls the type of messages for which snapshots are taken.

Refer to the PTP Frame Type Selections table.

Table 30-59: PTP Frame Type Selections

|   TSENALL (bit 8) | SNAPTYPSEL (bits [17:16])   | TSMSTRENA (bit 15)   | TSEVNTENA (bit 14)   | Frames                                                                                  |
|-------------------|-----------------------------|----------------------|----------------------|-----------------------------------------------------------------------------------------|
|                 1 | X                           | X                    | X                    | All                                                                                     |
|                 0 | 00                          | X                    | 0                    | Sync, Follow_Up, Delay_Req, Delay_Resp                                                  |
|                 0 | 00                          | 0                    | 1                    | Sync                                                                                    |
|                 0 | 00                          | 1                    | 1                    | Delay_Req                                                                               |
|                 0 | 01                          | X                    | 0                    | Sync, Follow_Up, Delay_Req, Delay_Resp, Pdelay_Req, Pdelay_Resp, Pdelay_Resp_ Follow_Up |
|                 0 | 01                          | 0                    | 1                    | Sync, Pdelay_Req, Pdelay_Resp                                                           |
|                 0 | 01                          | 1                    | 1                    | Delay_Req, Pdelay_Req, Pdelay_Resp                                                      |
|                 0 | 10                          | X                    | X                    | Sync, Delay_Req                                                                         |
|                 0 | 11                          | X                    | X                    | Pdelay_Req, Pdelay_Resp                                                                 |

## Time Stamp Correction

According to the IEEE 1588 specification, a time stamp must be captured when the PTP message time stamp point (leading edge of the first bit of the octet immediately following the SFD octet) crosses the boundary between the node and the network.

Because the MAC takes the time stamp at an internal point far from the actual boundary of the node and network, this captured time stamp is corrected/updated for the ingress/egress path latency (including the delay in the PHY layers). Further correction is done for the inaccuracies/errors introduced due to the clock (RGMII Tx, Rx clock). The clock can be different at the capture point when compared with the PTP clock that is used to generate the time. The resultant Clock Domain Crossing (CDC) circuits add error, depending on the clock period of the GMII and PTP clocks.

## Ingress Correction

In the receive side, the time stamp captured at the internal snapshot point is delayed (later in time) as compared to the time at which that packet's SFD bit is received at the port's boundary. Therefore, the captured time stamp must be reduced by the ingress latency and the errors in CDC sampling. This correction value must be determined/calculated by the software and written into the EMAC\_TM\_INGCORR\_NSEC and EMAC\_TM\_INGCORR\_SNSEC registers.

The correction value consists of the following components:

1. External latency in the PHY layer between boundary point and the input of the core. If the PHY is compliant with the IEEE 802.3 Clause 45 MMD registers, it has a register indicating the maximum and minimum ingress latency. The software can read these registers and determine the average ingress latency in the PHY. Alternatively (if the PHY does not support these registers), the ingress latency must be determined from its data sheet or timing characteristics.

2. Internal latency from the input of the core to the internal capture point. The internal ingress latency can be read from the EMAC\_TM\_ING\_LTNCY register. This read-only register gives the latency in scaled nanoseconds format, as defined in IEEE 1588 Clause 5.3.2. The latency differs based on the active PHY interface (RGMII, RMII, so on) and the operating speed. Therefore, the software must read this register after any speed change in the MAC to determine the current internal latency.
3. CDC Synchronization. The CDC synchronization error is approximately equal to two times the clock period of the PTP clock.

Software should add the values determined from these components and write them into the EMAC\_TM\_INGCORR\_NSEC.TSIC and EMAC\_TM\_INGCORR\_SNSEC.TSICSNS bit fields.

- NOTE: The value written to the register must be negative (two's complement), because it must be subtracted from the captured time stamp. The MAC receiver adds the value in this register to the captured time stamp and then gives the resultant value as the time stamp of the received packet.

When the EMAC\_TM\_CTL.TSCTRLSSR bit is set, the nanoseconds field of the captured time stamp is in decimal format with a granularity of 1 ns. So, bit 31 of EMAC\_TM\_INGCORR\_NSEC.TSIC must be set (= 1) (for negative value). Bits[30:0] must be written with "109 - total ingress\_correction\_value[nanosecond part]" represented in binary. For example, if the required correction value is -5 ns, the value is 0xBB9A\_C9FB.

When EMAC\_TM\_CTL.TSCTRLSSR bit is reset, the nanoseconds field of the captured time stamp is in binary format with a granularity of ~0.466 ns. Therefore, bits [30:0] must be written with "231 - total ingress\_correc-tion\_value" represented in binary with bit[31] = 1.

## Egress Correction

In the transmit side, the time stamp captured at the internal snapshot point is earlier (advanced in time) when compared with the time at which that packet's SFD bit is output at the port's boundary. Therefore, the captured time stamp must be compensated by the egress latency and the errors in CDC sampling. Software determines/calculates this correction value and writes it into the EMAC\_TM\_EGCORR\_NSEC and EMAC\_TM\_EGCORR\_SNSEC registers.

The correction value consists of the following components:

1. External latency in the PHY layer between the output of the core and the boundary of the port and the network. If the PHY is compliant to the IEEE 802.3 Clause 45 MMD registers, it has a register indicating the maximum and minimum egress latency. The software can read these registers and determine the average egress latency in the PHY. Otherwise, (if the PHY does not support these registers), the egress latency must be determined from its data sheet or timing characteristics.
2. Internal latency from the internal capture point and the output of the core. This internal egress latency can be read from the EMAC\_TM\_EG\_LTNCY register. This read-only register gives the latency in scaled nanoseconds format as defined in IEEE 1588 Clause 5.3.2. The latency differs based on the active PHY interface (RGMII, RMII, so on) and the operating speed. Therefore, the software must read this register after any speed change in the MAC to determine the current internal latency.

3. CDC synchronization error. The CDC synchronization error value depends on the one-step time stamping mode. When one-step time stamping is enabled, the value = (1 x period of PTP clock + 4 x period of Tx clock).

The PHY Interface Latency table lists the egress and ingress latency for various PHY interfaces.

Table 30-60: PHY Interface Latency

| PHY Interfaces   | PHY Interfaces   |   Egress Latency |   Ingress Latency |
|------------------|------------------|------------------|-------------------|
| RGMII            | 1.0 Gbps         |               12 |                12 |
| RGMII            | 100 Mbps         |               40 |                40 |
| RGMII            | 10 Mbps          |              400 |               400 |
| RMII             | 100 Mbps         |               40 |                40 |
| RMII             | 10 Mbps          |              400 |               400 |

## PTP Processing and Control

When the EMAC module receives a frame, frame detection and time stamping are based on some of the PTP fields in the frame. The PTP Message Format (IEEE 1588-2008) table shows the common message header for the PTP messages. This format is derived from the IEEE standard 1588-2008. When the EMAC module sends a PTP frame, the frame follows this format.

When a frame is received, the PTP module compares these fields with standard values and finds out the type of PTP frame and other information (for example, PTP version, IP version, and others). The module then updates the related fields in RDES4. When a frame is transmitted, programs must ensure that all the fields are appropriate. The PTP module on the other end of a communication must correctly detect and decode the frame.

Table 30-61: PTP Message Format (IEEE 1588-2008)

| Bits               | Bits               | Bits               | Bits               | Bits               | Bits               | Bits               | Bits               |   Octets |   Offset |
|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|----------|----------|
| 7                  | 6                  | 5                  | 4                  | 3                  | 2                  | 1                  | 0                  |          |          |
| transportSpecific  | transportSpecific  | transportSpecific  | transportSpecific  | messageType        | messageType        | messageType        | messageType        |        1 |        0 |
| Reserved           | Reserved           | Reserved           | Reserved           | versionPTP         | versionPTP         | versionPTP         | versionPTP         |        1 |        1 |
| messageLength      | messageLength      | messageLength      | messageLength      | messageLength      | messageLength      | messageLength      | messageLength      |        2 |        2 |
| domainNumber       | domainNumber       | domainNumber       | domainNumber       | domainNumber       | domainNumber       | domainNumber       | domainNumber       |        1 |        4 |
| Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           |        1 |        5 |
| flagField          | flagField          | flagField          | flagField          | flagField          | flagField          | flagField          | flagField          |        2 |        6 |
| correctionField    | correctionField    | correctionField    | correctionField    | correctionField    | correctionField    | correctionField    | correctionField    |        8 |        8 |
| Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           |        4 |       16 |
| sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity |       10 |       20 |
| sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         |        2 |       30 |

Table 30-61: PTP Message Format (IEEE 1588-2008)  (Continued)

| Bits                                                                                                                 |   Octets |   Offset |
|----------------------------------------------------------------------------------------------------------------------|----------|----------|
| controlField (used in version 1. For version 2, messageType field is used for detecting dif- ferent message types. ) |        1 |       32 |
| logMessageInterva                                                                                                    |        1 |       33 |

There are some fields in the Ethernet payload that can be used to detect the PTP packet type and control the snapshot taken. These fields are different for the following PTP frames:

- PTP Frames Over IPv4
- PTP Frames Over IPv6
- PTP Frames Over Ethernet

For these PTP frames, EMAC does not consider the PTP version 1 messages as valid when the frame consists of peer delay multicast address as destination address (DA).

## PTP Frame Over IPv4

The IPv4-UDP PTP Frame Fields Required for Control and Status table provides information about the fields that are matched to control snapshot for the PTP packets. The packets are sent over UDP over IPv4 for IEEE 1588 version 1 and 2. The octet positions for the tagged frames are offset by 4. The positions are based on IEEE 15882008 standards and the message format. The format is defined in the PTP Message Format (IEEE 1588-2008) table in the PTP Processing and Control section.

Table 30-62: IPv4-UDP PTP Frame Fields Required for Control and Status

| Field Matched                              | Octet Position   | Matched Value                                             | Description                                                                              |
|--------------------------------------------|------------------|-----------------------------------------------------------|------------------------------------------------------------------------------------------|
| MAC Frame type                             | 12, 13           | 0x0800                                                    | IPv4 datagram                                                                            |
| IP Version and Header Length               | 14               | 0x45                                                      | IP version is IPv4                                                                       |
| Layer 4 Protocol                           | 23               | 0x11                                                      | UDP                                                                                      |
| IP Multicast Address (IEEE 1588 Version 1) | 30, 31, 32, 33   | 0xE0,0x00, 0x01,0x81 (or 0x82 or 0x83 or 0x84)            | Multicast IPv4 addresses allowed 224.0.1.129 224.0.1.130 224.0.1.131 224.0.1.132         |
| IP Multicast Address (IEEE 1588 Version 2) | 30, 31, 32, 33   | 0xE0, 0x00, 0x01, 0x81 (Hex) 0xE0, 0x00, 0x00, 0x6B (Hex) | PTP-Primary multicast address: 224.0.1.129 PTP-Peer delay multicast address: 224.0.0.107 |

Table 30-62: IPv4-UDP PTP Frame Fields Required for Control and Status (Continued)

| Field Matched                           | Octet Position   | Matched Value                            | Description                                                                                                                                                                |
|-----------------------------------------|------------------|------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| UDP Destination Port                    | 36, 37           | 0x013F 0x0140                            | 0x013F - PTP Event Messages. These are SYNC, Delay_Req (IEEE 1588 version 1 and 2) or Pdelay_Req, Pdelay_Resp (IEEE 1588 version 2 only). 0x0140 - PTP general messages    |
| PTP Control Field (IEEE ver- sion 1)    | 74               | 0x00/0x01/0x02/ 0x03/0x04                | 0x00 - SYNC 0x01 - Delay_Req 0x02 - Follow_Up 0x03 - Delay_Resp 0x04 - Management                                                                                          |
| PTP Message Type Field (IEEE version 2) | 42 (nibble)      | 0x0/0x1/0x2/0x3/0x8/0x9/0xA/0xB/ 0xC/0xD | 0x0 - SYNC 0x1 - Delay_Req 0x2 - Pdelay_Req 0x3 - Pdelay_Resp 0x8 - Follow_Up 0x9 - Delay_Resp 0xA - Pdelay_Resp_Follow_Up 0xB - Announce 0xC - Signaling 0xD - Management |
| PTP Version                             | 43 (nibble)      | 0x1 or 0x2                               | 0x1 - Supports PTP version 1 0x2 - Supports PTP version 2                                                                                                                  |

## PTP Frame Over IPv6

The IPv6-UDP PTP Frame Fields Required for Control And Status table provides information about the fields that are matched to control the snapshots for the PTP packets. The packets are sent over UDP over IPv6 for IEEE 1588 version 1 and 2. The octet positions for the tagged frames are offset by 4. The positions are based on IEEE 15882008 standards and the message format defined in PTP Message Format (IEEE 1588-2008).

Table 30-63: IPv6-UDP PTP Frame Fields Required for Control and Status

| Field Matched    | Octet Position                                         | Matched Value   | Description        |
|------------------|--------------------------------------------------------|-----------------|--------------------|
| MAC Frame type   | 12, 13                                                 | 0x86DD          | IP datagram        |
| IP Version       | 14 (bits [7:4])                                        | 0x06            | IP version is IPv6 |
| Layer 4 Protocol | 20 (IPv6 extension header not defined for PTP packets) | 0x11            | UDP                |

Table 30-63: IPv6-UDP PTP Frame Fields Required for Control and Status (Continued)

| Field Matched                           | Octet Position                                                    | Matched Value                                                | Description                                                                                                                                                                |
|-----------------------------------------|-------------------------------------------------------------------|--------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PTP Multicast Address                   | 38-53                                                             | FF0x:0:0:0:0:0:0:0:0:181 (Hex) FF02:0:0:0:0:0:0:0:0:6B (Hex) | PTP - primary multicast address: FF0x: 0:0:0:0:0:0:0:0:181 (Hex) PTP - Peer delay multicast address: FF02:0:0:0:0:0:0:0:0:6B (Hex)                                         |
| UDP Destination Port                    | 56, 57 (IPv6 exten- sion header not de- fined for PTP pack- ets)  | 0x013F, 0x0140                                               | 0x013F - PTP event messages 0x0140 - PTP general messages                                                                                                                  |
| PTP Control Field (IEEE 1588 version 1) | 93 (IPv6 extension header not defined for PTP packets)            | 0x00/0x01/0x02/ 0x03/0x04                                    | 0x00 - SYNC 0x01 - Delay_Req 0x02 - Follow_Up 0x03 - Delay_Resp 0x04 - Management (version1)                                                                               |
| PTP Message Type Field (IEEE version 2) | 74 (nibble) (IPv6 ex- tension header not defined for PTP packets) | 0x0/0x1/0x2/0x3/0x8/0x9/0xA/0xB/ 0xC/0xD                     | 0x0 - SYNC 0x1 - Delay_Req 0x2 - Pdelay_Req 0x3 - Pdelay_Resp 0x8 - Follow_Up 0x9 - Delay_Resp 0xA - Pdelay_Resp_Follow_Up 0xB - Announce 0xC - Signaling 0xD - Management |
| PTP Version                             | 75 (nibble)                                                       | 0x1 or 0x2                                                   | 0x1 - Supports PTP version 1 0x2 - Supports PTP version 2                                                                                                                  |

## PTP Frame Over Ethernet

Refer to the Ethernet PTP Frame Fields Required for Control and Status table. The table provides information about the fields that are matched to control the snapshots for the PTP packets sent over Ethernet for IEEE 1588 version 1 and 2. The octet positions for the tagged frames are offset by 4. The positions are based on IEEE 15882008 standards and the message format defined in the table.

Table 30-64: Ethernet PTP Frame Fields Required for Control and Status

| Field Matched                                                                                                                                                      | Octet Position   | Matched value                             | Description                                                                                                                                                                |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|-------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| MAC Destination Multicast Address (The address match of destination address (DA) programmed in MAC address 0 is used when the EMAC_TM_CTL. TSENMACADDR bit is set) | 0-5              | 01-1B-19-00-00-00 01-80-C2-00-00-0E       | All PTP messages can use any of the fol- lowing multicast addresses: 01-1B-19-00-00-00 01-80-C2-00-00-0E                                                                   |
| MAC Frame Type                                                                                                                                                     | 12, 13           | 0x88F7                                    | PTP Ethernet frame                                                                                                                                                         |
| PTP control field (IEEE Ver- sion 1)                                                                                                                               | 45               | 0x00/0x01/0x02/ 0x03/0x04                 | 0x00 - SYNC 0x01 - Delay_Req 0x02 - Follow_Up 0x03 - Delay_Resp 0x04 - Management                                                                                          |
| PTP Message Type Field (IEEE version 2)                                                                                                                            | 14 (nibble)      | 0x0/0x1/0x2/0x3/0x8/0x9/0x A/0xB/ 0xC/0xD | 0x0 - SYNC 0x1 - Delay_Req 0x2 - Pdelay_Req 0x3 - Pdelay_Resp 0x8 - Follow_Up 0x9 - Delay_Resp 0xA - Pdelay_Resp_Follow_Up 0xB - Announce 0xC - Signaling 0xD - Management |
| PTP Version                                                                                                                                                        | 15(nibble)       | 0x1 or 0x2                                | 0x1 - Supports PTP version 1 0x2 - Supports PTP version 2                                                                                                                  |

## Auxiliary Time Stamp Snapshot

The auxiliary snapshot feature stores snapshots of the system time whenever a rising edge is detected on the EMAC\_PTPAUX\_MCG\_IN[n] pins.

The PTP stores 64 bits of the captured time stamp in a 16-deep FIFO. Only 64 bits of the time stamp are stored in the FIFO. The upper 16 bits of seconds can be read from the EMAC\_TM\_HISEC register when it is present. When a snapshot is stored, the PTP indicates this event to the EMAC with the auxiliary snapshot interrupt. The EMAC\_TM\_STAT.AUXTSTRIG bit is set (=1). The value of the snapshot is read through the EMAC\_TM\_AUXTS\_SEC and EMAC\_TM\_AUXTS\_NSEC registers. If the FIFO becomes full and an external trigger to take the snapshot is asserted, then the snapshot trigger missed ( EMAC\_TM\_STAT.ATSSTM ) bit is set (=1). The latest snapshot is not written to the FIFO when it is full.

When a host reads the 64-bit time stamp from the FIFO through the EMAC\_TM\_AUXTS\_SEC and EMAC\_TM\_AUXTS\_NSEC registers, the space becomes available to store the next snapshot.

- NOTE: A space in the FIFO is created whenever the EMAC\_TM\_AUXTS\_SEC register is read. Therefore, read the EMAC\_TM\_AUXTS\_NSEC register before reading the EMAC\_TM\_AUXTS\_SEC register.

The program can clear the FIFO by setting the EMAC\_TM\_AUX\_CTL.ATSFC bit. When multiple snapshots are present in the FIFO, the EMAC\_TM\_STAT.ATSNS bits indicate the count.

- NOTE: The minimum gap between two events on the EMAC\_PTPAUX\_MCG\_IN[n] pin must be 4 cycles of PTP\_CLK + 3 cycles of CLKO7). Otherwise, the logic misses the rising edge of the trigger.

## System Time

To get a snapshot of the time, the EMAC requires a reference time in 64-bit format as defined in the IEEE 1588 specification. The PTP module maintains 80-bit time, known as system time. The PTP clock updates system time.

The 80-bit timing reference is split into the following registers:

- EMAC\_TM\_NSEC -32-bit nanoseconds register which provides time in nanoseconds
- EMAC\_TM\_SEC -32-bit seconds register which provides time in seconds
- EMAC\_TM\_HISEC -16-bit high seconds register which provides time beyond the seconds register. The IEEE 1588 standard does not include this register. Its use is application specific.

The 64-bit system time (seconds and nanoseconds) is the source for taking time stamps for Ethernet frames being transmitted or received at the RMII.

Since the PTP clock frequency does not correspond to a 1 ns period, the EMAC\_TM\_NSEC register is incremented with a value equal to the PTP clock period for every PTP clock cycle. The function uses the EMAC\_TM\_SUBSEC register. The EMAC\_TM\_NSEC value is incremented with the value programmed in EMAC\_TM\_SUBSEC register every PTP clock cycle.

Whenever the EMAC\_TM\_SEC register overflows from 0xFFFFFFFF to 0x00000000, the seconds overflow interrupt is triggered. The EMAC uses the EMAC\_TM\_STAT.TSSOVF bit to indicate the event. After a seconds overflow, the EMAC\_TM\_HISEC register increments by one.

The system time module supports the following types of rollover modes for the EMAC\_TM\_NSEC register.

- Digital rollover mode. The maximum value in the nanoseconds field is 0x3B9AC9FF , that is, 10 9  nanoseconds. After it reaches this value, the EMAC\_TM\_SEC register increments and the EMAC\_TM\_NSEC register restarts counting from zero. Accuracy in digital rollover mode it is 1 ns per bit.
- Binary rollover mode. The nanoseconds field rolls over and increments the seconds field after the value reaches 0x7FFFFFFF. Accuracy in binary rollover mode is ~0.466 ns per bit.

## System Time Adjustment

The following sections describe the process for system time adjustment.

## System Time Initialization

System time can be initialized with 64-bit time when the PTP module is enabled. The initial value is written to the EMAC\_TM\_SECUPDT and EMAC\_TM\_NSECUPDT system time update registers. The system time counter is written with the value in the registers when EMAC\_TM\_CTL.TSINIT =1.

## Coarse Correction Method

If the complete system time has an offset based on the system time of the requester, the coarse correction method can correct it. The time offset value is written to the EMAC\_TM\_SECUPDT and EMAC\_TM\_NSECUPDT registers. The offset value is then added to or subtracted from the system time when the EMAC\_TM\_CTL.TSUPDT bit is set (=1). Use the EMAC\_TM\_NSECUPDT.ADDSUB bit to choose addition or subtraction. System time correction occurs in one clock cycle using the coarse correction method.

NOTE: During subtraction, the EMAC\_TM\_SECUPDT register value must be less than the value of the EMAC\_TM\_SEC register. Check the value prior to subtracting using coarse correction.

## Fine Correction Method

If a target PTP clock frequency has a drift based on the requester PTP clock (as defined in IEEE 1588), it can be corrected using the fine correction method. Using this method, system time is corrected over a period (unlike coarse correction where it happens in one clock cycle). This correction helps maintain linear time and does not introduce drastic changes (or a large jitter) in the reference time between PTP sync message intervals.

Using this method, an accumulator sums the contents of the EMAC\_TM\_ADDEND register. The System Time Update, Fine Correction Method figure shows the method. The arithmetic-carry that the accumulator generates acts as a pulse to increment the system time counter. The accumulator and the addend are 32-bit registers. Here, the accumulator acts as a high-precision frequency divider.

Figure 30-32: System Time Update, Fine Correction Method

<!-- image -->

## Calculating Addend Value

The example in this section uses the fine correction process for system time.

In this example, the requester clock runs at 50 MHz and the completer clock has drifted to 66 MHz. The goal is to adjust the completer system time to 50 MHz, so that the completer PTP module synchronizes with the requester. Using the figure in Fine Correction Method, the nanoseconds increment signal runs at 50 MHz. The nanoseconds increment is the carry from accumulator register. The addend value increments the carry from accumulator at the rate of the completer clock (66 MHz).

The accumulator overflows and generates a carry every N addend value, so N × Addend = 2 32 .

The accumulator increments at 66 MHz. This addition brings the carry to 50 MHz N = 66/50 = 1.32.

Hence, the addend = 2 32 /1.32 = 0xC1F07C1F .

Therefore, if the addend is programmed with 0xC1F07C1F, the completer system time runs at 50 MHz, which synchronizes with the requester.

In the Fine Correction Method figure, the sub second increment is the value programmed in the EMAC\_TM\_SUBSEC register. It increments the EMAC\_TM\_NSEC register according to the frequency of the nanoseconds increment signal.

In the example, the sub second increment is 20 (for digital rollover) or 43 (for binary rollover). This addition increments the EMAC\_TM\_NSEC register by 20 ns (1/50 MHz).

The software must calculate the drift in frequency and update the EMAC\_TM\_ADDEND register accordingly.

- NOTE: The PTP reference clock is the clock at which the system time is updated. When the EMAC\_TM\_CTL.TSCFUPDT bit is cleared (= 0), this clock equals the PTP clock. Using fine correction, the PTP reference clock is generated on the nanoseconds increment signal at which the system time updates.

## Target Time Trigger (Alarm)

The PTP module provides an alarm function by triggering an alarm at a preset time. It sets the EMAC\_TM\_STAT.TSTARGT0 bit when the system time matches the value of the EMAC\_PPS[n]\_TGTM\_SEC and EMAC\_PPS[n]\_TGTM\_NSEC registers. This trigger can generate an interrupt and command the flexible PPS module to start or stop PPS output, depending on value programmed in the EMAC\_PPS\_CTL.TRGTMODSEL0 field.

The trigger is enabled by programming the EMAC\_PPS[n]\_TGTM\_SEC and EMAC\_PPS[n]\_TGTM\_SEC registers. Once an alarm has occurred, if the PTP needs another alarm, the software must:

- Clear the status bit,
- Reprogram the EMAC\_PPS[n]\_TGTM\_SEC and EMAC\_PPS[n]\_TGTM\_NSEC registers to a future value, and
- If the time programmed in the target time registers has elapsed, indicate a target time programming error by setting the EMAC\_TM\_STAT.TSTRGTERR0 bit.

The alarm time is represented in absolute units, not relative units. For example, if the software must generate an alarm after 10 seconds, it must read the current system time value. Then, the software must add the number corresponding to 10 seconds, and write the result back to the target time registers.

## Pulse-Per-Second (PPS)

Pulse-per-second (PPS) is a physical representation of system time. It consists of a single pulse or train of pulses. The PTP uses PPS for extra synchronization or to monitor the synchronization performance between clocks. With proper configuration, the PTP module can generate PPS signals that are output on the EMAC\_PTPPPS[n] pins. The PTP supports two kinds of PPS output: fixed and flexible.

## Fixed Pulse-Per-Second Output

The EMAC module supports a fixed pulse-per-second (PPS) output that indicates 1 second intervals (default). Change the frequency of the PPS output by configuring the EMAC\_PPS\_CTL.PPSCTRL\_PPSCMD bits. The default value for these bits is 0000, which configures a 1 Hz signal with a pulse width equal to the period of the PTP clock.

The PPS Output Frequencies table shows various PPS output frequencies.

Table 30-65: PPS Output Frequencies

| PPSCTL Bit Setting   | Binary Rollover   | Digital Rollover   |
|----------------------|-------------------|--------------------|
| 0001                 | 2 Hz              | 1 Hz               |
| 0010                 | 4 Hz              | 2 Hz               |
| 0011                 | 8 Hz              | 4 Hz               |
| ...                  | ...               | ...                |
| 1111                 | 32.768 kHz        | 16.384 kHz         |

In binary rollover mode, the PPS output (ETH)\_PTPPS0) has a duty cycle of 50% with these frequencies.

In digital rollover mode, the PPS output frequency is an average number. The actual clock is a different frequency that is synchronized every second. PPS output pulses have different periods and duty cycles. This behavior is because of the non-linear toggling of the bits in digital rollover mode. For example:

- When EMAC\_PPS\_CTL.PPSCTRL\_PPSCMD = 0001, the PPS (1 Hz) has a low period of 537 ms and a high period of 463 ms.
- When EMAC\_PPS\_CTL.PPSCTRL\_PPSCMD = 0010, the PPS (2 Hz) is a sequence of:
- One clock of 50 % duty cycle and 537 ms period
- Second clock of 463 ms period (268 ms low and 195 ms high)
- When EMAC\_PPS\_CTL.PPSCTRL\_PPSCMD = 0011, the PPS (4 Hz) is a sequence of:
- Three clocks of 50 % duty cycle and 268 ms period
- Fourth clock of 195 ms period (134 ms low and 61 ms high)

## Flexible Pulse-Per-Second Output

The EMAC module also provides the flexibility to program the start or stop time, width, and interval of the pulse generated on the PPS output. Enable this feature, called flexible PPS, by setting the EMAC\_PPS\_CTL.PPSEN0 bit.

## The flexible PPS output options are:

- Program the start point of the single pulse and start and stop points of the pulse train in terms of system time. The target time registers program the start and stop time.
- Program the stop time in advance. Programs can configure the stop time before the actual start time has elapsed.
- Program the width, between the rising edge and corresponding falling edge of the PPS signal output, in terms of number of units of subsecond increment. This value is configured in the EMAC\_TM\_SUBSEC register.

- Program the interval, between the rising edges of PPS signal, in terms of number of units of subsecond increment. This value is configured in the EMAC\_TM\_SUBSEC register.
- Cancel the programmed PPS start or stop request.
- Indicate an error if the start or stop time programmed has already elapsed.

## PPS Start or Stop Time

Start time can initially be programmed in the target time registers. If necessary, the start or stop time can be programmed again, but, only after the earlier programmed value is synchronized to the PTP clock domain. The EMAC\_PPS[n]\_TGTM\_NSEC.TRGTBUSY bit indicates the status of synchronization. Programs can configure the start or stop time in advance, even before the earlier stop or start time has elapsed.

Program the start or stop time with advanced system time to ensure proper PPS signal output. If the application programs a start or stop time that has already elapsed, the EMAC module sets the EMAC\_TM\_STAT.TSTRGTERR0 bit indicating the error. If enabled, the EMAC module also sets the target time trigger (alarm) interrupt event. The application can cancel the start or stop request only when the corresponding start or stop time has not elapsed. If the time has elapsed, the cancel command has no effect.

## PPS Width and Interval

The PPS width and interval are programmed in terms of granularity of system time, that is, the number of the units of subsecond increment value. For example, with the PTP reference clock of 50 MHz, a PPS pulse width of 40 ns, and an interval of 100 ns, program the width and interval to 2 and 5, respectively.

Use a faster PTP reference clock to achieve smaller granularity. Before commanding to trigger a pulse or pulse train on the PPS output, programs must configure or update the interval and width of the PPS signal output.

## PPS Command

When the PPS module has a flexible PPS output configuration, the PTP can use the

EMAC\_PPS\_CTL.PPSCTRL\_PPSCMD bits to command the PPS module to use any of the flexible PPS features.

Programming these bits with a non-zero value instructs the PPS module to initiate an event. Once the command transfers or synchronizes to the PTP clock domain, these bits clear automatically. Software must ensure that these bits are programmed only when they are all-zero.

The Flexible PPS Output Commands table describes the different commands and their functions.

Table 30-66: Flexible PPS Output Commands

|   PPSCTL (Bits 3-0) | Command            | Description                                                                                                                                          |
|---------------------|--------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
|                0000 | No Command         | No Command                                                                                                                                           |
|                0001 | Start Single Pulse | Generates a single pulse rising at the start point defined in the target time registers and of the duration defined in the EMAC_PPS[n]_WID register. |

Table 30-66: Flexible PPS Output Commands (Continued)

| PPSCTL (Bits 3-0)   | Command                        | Description                                                                                                                                                                                                                                                                                                                                                                                               |
|---------------------|--------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0010                | Start Pulse Train              | Generates a train of pulses rising at the start time configured in the target time registers and of the duration configured in the EMAC_PPS[n]_WID register. The train of pul- ses repeats at the interval configured in the EMAC_PPS[n]_INTVL register. By de- fault, the PPS pulse train is runs freely unless shalted by a Stop Pulse Train at Time command or a Stop Pulse Train Immediately command. |
| 0011                | Cancel Start                   | Cancels the Start Single Pulse and Start Pulse Train commands when the system time has not crossed the programmed start time.                                                                                                                                                                                                                                                                             |
| 0100                | Stop Pulse Train at Time       | Stops the train of pulses initiated by the command for start pulse train after the time programmed in the target time registers elapses.                                                                                                                                                                                                                                                                  |
| 0101                | Stop Pulse Train Im- mediately | Immediately stops the train of pulses initiated by the command for start pulse train.                                                                                                                                                                                                                                                                                                                     |
| 0110                | Cancel Stop Pulse Train        | Cancels the stop pulse train at time command when the programmed stop time has not elapsed. The PPS pulse train switches to free-running on the successful execution of this command.                                                                                                                                                                                                                     |
| 0111-1111           | Reserved                       | Reserved                                                                                                                                                                                                                                                                                                                                                                                                  |

## PTP Interrupts

Set the EMAC\_MAC\_IEN.TSIE bit to enable interrupts from the PTP module. The EMAC module uses the EMAC\_MAC\_ISTAT.TSIS bit to indicate the status of the interrupt. The PTP supports the following three types of interrupts.

## Auxiliary Snapshot Trigger

When an external event occurs on the EMAC\_PTPAUX\_MCG\_IN[n] pins and a time stamp snapshot occurs, an auxiliary snapshot interrupt is triggered. The EMAC module uses the EMAC\_TM\_STAT.AUXTSTRIG bit to indicate the interrupt.

## Target Time Reached

This interrupt is triggered when the system time becomes equal to the value written in the EMAC\_PPS[n]\_TGTM\_SEC and EMAC\_PPS[n]\_TGTM\_NSEC registers. This interrupt can be used as an alarm and is indicated on the EMAC\_TM\_STAT.TSTARGT0 bit.

## System Time Seconds Register Overflow

The system time seconds register overflow interrupt is triggered when the EMAC\_TM\_SEC register overflows from 0xFFFF FFFF to 0x0000 0000. This interrupt is indicated on the EMAC\_TM\_STAT.TSSOVF bit. As soon as EMAC\_TM\_SEC register overflows, the EMAC\_TM\_HISEC register increments by one.

## Media Clock Generation and Recovery (MCGR)

The EMAC module supports dedicated hardware for media clock recovery. This hardware generates a reference clock, which is multiplied to get the desired media clock. The hardware clock multiplier is external to the EMAC.

The media clock generation and recovery process is mainly hardware-based and involves insignificant software.

NOTE: The PPS and MCGR modes are mutually exclusive in that all the instances selected in the configuration must operate either in PPS mode or in MCGR mode. Mixed operation mode, where a few instances operate in PPS mode and others in MCGR mode, is not supported. Therefore, program all the instances to the same mode, irrespective of whether they are used or not.

The media clock generation and recovery support involve a dedicated external hardware and a thin layer of software. The dedicated hardware generates the reference clock (for example, 6 kHz or 8 kHz for AVB class A stream) which is then multiplied to get the desired media clock (for example an audio MCLK of 12.288 MHz).

The media clock recovery process involves host software extracting the presentation time from the 1722 frames and programming them into a CSR register, which is read by the EMAC to generate a reference clock. When the timer expires, the previous value is either toggled or a pulse is sent. The output at the recovered clock pin is generally a wave of 750 Hz ranging up to 8 kHz depending on the application that the clock is synced with, at the other end of the network.

As shown in the Media Clock Generation and Recovery Block Diagram , the time stamp array and the DMA module are external to the EMAC block.

Figure 30-33: Media Clock Generation and Recovery Block Diagram

<!-- image -->

For the operation of the media clock generation and recovery, the EMAC supports:

- A 32-bit presentation time nanosecond counter that wraps at the full 32-bit value (and not after one second) to match up with the 1722 presentation time format.
- A mechanism to handshake with an external application (such as DMA) to program the presentation time into the CSR register. The hardware compares the programmed time stamp value with the presentation time and generates a wave based on the programmable settings.

## Presentation Time Counter

The presentation time counter follows the 1722 presentation time format. In this format, time is represented as a 32-bit counter in nanoseconds that does a binary rollover on reaching the maximum value of 32'hFFFF\_FFFF . This counter is another view of the PTP system time (1588 timer). For a given PTP , system time (PTP[63:32] represents time in seconds; PTP[31:0] represents time in nanoseconds. There exists a corresponding presentation time (32-bit value in nanosecond). The presentation time computed is referred to as the Current Presentation Time (CPT).

The CPT can be viewed as derived from 64-bit PTP system time as:

PTPNS [63:0] = (PTP[63:32] ✕ 32'd1,000,000,000) + PTP[31:0]

where,

PTPNS is the PTP system time converted into a 64-bit nanosecond value

CPT [31:0] = PTPNS [31:0]

Media clock recovery is possible when the system time is internally generated in the EMAC module. CPT is computed in the same way as the internal PTP system time generation. Because CPT and system time values are in nanoseconds, and must be synchronous, the increment cycle of both the timers are same.

The timer is updated at the same instance; however, the updated value could be different. The software must compute a separate 32-bit update value in nanoseconds for the CPT update. A CPT sampled at different edges of a triggering input results in the generation of media clock time stamps which are inserted in 1722-based AVBTP packets to recover the clock at the destination.

- NOTE: · The value of CPT can be obtained by reading the EMAC\_PRSNTM\_NSEC register.
- The EMAC module supports both fine grain and coarse grain correction modes.
- Only the digital rollover mode must be used for the PTP time. The CPT also uses the same increment value and CPT requires the time to be in nanoseconds.

## Comparator Modules

As shown in Figure 30-33 Media Clock Generation and Recovery Block Diagram, the EMAC module supports four comparator modules to handle multiple media clocks that might be required.

The comparator modules handshake with the application to:

- Program the time stamps into the EMAC\_PPS[n]\_TGTM\_SEC register when EMAC\_TM\_CTL.PTGE =1.
- Set the presentation time control bits of the instance for recovery mode

The time stamps received from the application are referred as Target Presentation Time (TPT).

The MCGR and PPS modes are mutually exclusive. When the EMAC\_PPS\_CTL.MCGREN3 through EMAC\_PPS\_CTL.MCGREN0 bits are set (= 1), the corresponding PPS instance[n] operates in MCGR mode. Otherwise, by default, the PPS instance[n] operates in PPS mode.

The comparator module sends up to two requests to the application for TPT write, when EMAC\_TM\_CTL.PTGE = 1, and the presentation time control bits of the instance are configured for MCGR mode. Subsequent requests are generated each time a presentation time match occurs. (CPT transitions from a value less than TPT, to a value greater than or equal to CPT). The first request is asserted when a specific comparator instance is configured to MCGR mode with a non-zero presentation time control; an additional request is made when the first data is received into the comparator. This sequence allows the application to write the next TPT value while the EMAC module processes the previous TPT value for a match.

The TPT read from the EMAC\_PPS[n]\_TGTM\_SEC register is considered as a future time. A toggle/pulse is generated in the next cycle when a match is detected. When a match is detected, the EMAC\_PTPAUX\_MCG\_IN[n] trigger requester request for that comparator is asserted (to obtain next TPT) until the corresponding application acknowledgment is set.

The presentation control bits (for example, EMAC\_PPS\_CTL.PPSCMD1 ) determine the shape of the generated waveform. Also, these bits can be programmed to either toggle or generate a high/low pulse for one PTP clock cycle, upon match.

## Clock Recovery

A match is when the free-running CPT value matches the received TPT value. An output signal, EMAC\_PTPPPS[n] , is asserted (toggle, low pulse, or high pulse) based on the programmed presentation control value.

## Clock Generation

Based on the presentation control value programmed in the EMAC\_PPS\_CTL register, the comparator captures the presentation time and programs it into the EMAC\_PPS[n]\_TGTM\_SEC register. A request is raised to the application to read the captured time stamp. Until the acknowledgment that the read operation is complete is received from the application, no new time stamps are captured.

## Media Clock Generation and Recovery Flow

The Media Clock Generation and Recovery Flow figure shows the media clock generation and recovery flow.

For both media clock generation and recovery, set the EMAC\_TM\_CTL.PTGE field for CPT generation and set the EMAC\_PPS\_CTL.MCGREN3 through EMAC\_PPS\_CTL.MCGREN0 field for the corresponding instance to enable MCGR mode.

For media clock generation, the generated presentation time is sampled at positive, negative or at both the edges of trigger inputs ( EMAC\_PTPAUX\_MCG\_IN[n] ) based on the mode programmed in the EMAC\_PPS\_CTL register. The sampled value is captured in the EMAC\_PPS[n]\_TGTM\_SEC registers. After these registers are updated with the presentation time, the EMAC asserts the EMAC\_PTPAUX\_MCG\_IN[n] trigger to request the application to read the presentation time. The application acknowledges that it has read the captured time stamp by asserting the corresponding EMAC\_PTPAUX\_MCG\_IN[n] trigger completer.

For media clock recovery, the PPS instance asserts the EMAC\_PTPAUX\_MCG\_IN[n] trigger to tell the application to program the target time seconds registers with the TPT value and acknowledge the corresponding request using the EMAC\_PTPAUX\_MCG\_IN[n] trigger completer. When the application programs the target time seconds registers with the TPT value, the particular comparator compares the programmed TPT value with the CPT and

generates the waveform on EMAC\_PTPPPS[n] based on the presentation time control value programmed in the EMAC\_PPS\_CTL register.

- NOTE: · The consecutive triggers to sample the presentation time should always be asserted after a few PTP clock cycles to allow for synchronization delays. (This is not an issue when the input trigger maximum frequency is 8 kHz and the PTP clock is running at least at 1 MHz).
- DMA acknowledgment can be both posted as well as non-posted, in the media clock recovery mode.

Figure 30-34: Media Clock Generation and Recovery Flow

<!-- image -->

## PTP Time Stamp Offload Function

The PTP time stamp offload function enables the automatic generation of specific PTP packets when the MAC is working as a specific node in the PTP network. These packets may be generated periodically or triggered by the host software. In other modes, this feature can parse the incoming PTP packets on the receiver, and automatically generate and respond to the required PTP packets. This functionality helps in offloading certain functions of a PTP node with better accuracy and lower latency of responses.

Based on the programmed mode, the MAC generates PTP Ethernet messages periodically or from the application or based on reception of a particular PTP message.

The PTP Message Generation Criteria table indicates the PTP message generation criteria. Note that all other programming combinations are invalid for the PTP offload feature.

Table 30-67: PTP Message Generation Criteria

| Programming ( EMAC_TM_CTL )   | Programming ( EMAC_TM_CTL )   | Programming ( EMAC_TM_CTL )   | Mode                             | Criteria for Genera- tion of PTP Messag- es   | PTP Message Type Generated   |
|-------------------------------|-------------------------------|-------------------------------|----------------------------------|-----------------------------------------------|------------------------------|
| SNAPTYPSEL                    | TSMSTRENA                     | TSEVNTENA                     | Mode                             | Criteria for Genera- tion of PTP Messag- es   | PTP Message Type Generated   |
| 2'b00                         | 0                             | 1                             | Ordinary or boun- dary completer | SYNC message re- ception                      | Delay_Req                    |
| 2'b00                         | 1                             | 1                             | Ordinary or boun- dary requester | Periodic or on trigger from application       | SYNC                         |
| 2'b00                         | 1                             | 1                             | Ordinary or boun- dary requester | Delay_Req message reception                   | Delay_Resp                   |
| 2'b01                         | 0                             | 1                             | Transparent complet- er          | Periodic or on trigger from application       | Pdelay_Req                   |
| 2'b01                         | 0                             | 1                             | Transparent complet- er          | Pdelay_Req message reception                  | Pdelay_Resp                  |
| 2'b01                         | 0                             | 1                             | Transparent complet- er          | SYNC message re- ception                      | Delay_Req                    |
| 2'b01                         | 1                             | 1                             | Transparent requester            | Periodic or on trigger from application       | Pdelay_Req                   |
| 2'b01                         | 1                             | 1                             |                                  | Pdelay_Req message reception                  | Pdelay_Resp                  |
| 2'b01                         | 1                             | 1                             |                                  | Periodic or on trigger from application       | SYNC                         |
| 2'b01                         | 1                             | 1                             |                                  | Delay_Req message reception                   | Delay_Resp                   |
| 2'b11                         | X                             | X                             | Peer-to-peer trans- parent       | Periodic or on trigger from application       | Pdelay_Req                   |
| 2'b11                         | X                             | X                             | Peer-to-peer trans- parent       | Pdelay_Req message reception                  | Pdelay_Resp                  |

NOTE: Clocks that support the peer delay mechanism must not generate delay request/delay response messages, according to the IEEE 1588-2008 specification. However, the EMAC supports this for flexibility, with a programmable control bit ( EMAC\_PTO\_CTL.DRRDIS ). Programs can use this bit to control the response generation for delay request/delay response message. For example, in transparent completer mode, delay request is generated in response to received sync only when the bit is reset.

For example, when the MAC is set as an ordinary or boundary completer clock in the PTP network, it can respond to the reception of SYNC messages with an automatic generation and transmission of the corresponding Delay\_Req

message. Similarly, various other modes of operation are explained in the PTP Message Generation Criteria table. The MAC supports the multicast communication model for the generation of SYNC and Pdelay\_Req PTP messages. For instance, the destination address field of the generated PTP over Ethernet packet is the defined special multicast addresses (0x011B19000000 for all except peer delay mechanism messages and 0x0180C200000E for peer delay mechanism messages).

When the MAC responds to received SYNC, Delay\_Req, and Pdelay\_Req PTP messages with a special multicast destination address, it also uses the corresponding special multicast address in the DA field of the automatically generated Delay\_Req, Delay\_Resp, and Pdelay\_Resp PTP messages, respectively.

When the MAC responds to received SYNC, Delay\_Req, and Pdelay\_Req PTP messages with a unicast destination address, it takes the SA field of the received packets and makes them as the DA field of the automatically generated Delay\_Req, Delay\_Resp, and Pdelay\_Resp PTP messages, respectively.

At the same time, all received PTP messages are forwarded to the application along with the receive status, indicating whether the response was generated by the MAC and whether it satisfies the packet filtering logic of the MAC receiver.

When the MAC automatically generates a PdelayReq or responds with a Delay\_Req, the egress time stamp of these two PTP messages are provided in the transmit TS status ( EMAC\_TM\_STAT ) register; an interrupt is generated.

In addition to matching messageType and versionPTP fields for basic PTP over Ethernet message detection, the following fields are matched to qualify the received PTP message type:

1. The domainNumber field is checked for a match against the value programmed in the CSR.
2. The twoStepFlag in flagField field is checked for one step indication (1'b0).

The transportSpecific field is checked for the default PTP over Ethernet (4'h0) or 802.1AS mode (4'h1), when enabled.

## Packet Generation

When enabled, the MAC can automatically generate PTP packets. The Common PTP Message Header Fields table describes the fields of specific PTP packets generated. The format and content of the packets are described in the following sections.

Table 30-68: Common PTP Message Header Fields

| Bits              | Bits              | Bits              | Bits              | Bits          | Bits          | Bits          | Bits          |        |        |
|-------------------|-------------------|-------------------|-------------------|---------------|---------------|---------------|---------------|--------|--------|
| 7                 | 6                 | 5                 | 4                 | 3             | 2             | 1             | 0             | Octets | Offset |
| transportSpecific | transportSpecific | transportSpecific | transportSpecific | messageType   | messageType   | messageType   | messageType   | 1      | 0      |
| Reserved          | Reserved          | Reserved          | Reserved          | versionPTP    | versionPTP    | versionPTP    | versionPTP    | 1      | 1      |
| messageLength     | messageLength     | messageLength     | messageLength     | messageLength | messageLength | messageLength | messageLength | 2      | 2      |
| domainNumber      | domainNumber      | domainNumber      | domainNumber      | domainNumber  | domainNumber  | domainNumber  | domainNumber  | 1      | 4      |
| Reserved          | Reserved          | Reserved          | Reserved          | Reserved      | Reserved      | Reserved      | Reserved      | 1      | 5      |

Table 30-68: Common PTP Message Header Fields (Continued)

| Bits               | Bits               | Bits               | Bits               | Bits               | Bits               | Bits               | Bits               |        |        |
|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|--------------------|--------|--------|
| 7                  | 6                  | 5                  | 4                  | 3                  | 2                  | 1                  | 0                  | Octets | Offset |
| flagField          | flagField          | flagField          | flagField          | flagField          | flagField          | flagField          | flagField          | 2      | 6      |
| correctionField    | correctionField    | correctionField    | correctionField    | correctionField    | correctionField    | correctionField    | correctionField    | 8      | 8      |
| Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | Reserved           | 4      | 16     |
| sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | sourcePortIdentity | 10     | 20     |
| sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | 2      | 30     |
| sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | sequenceId         | 2      | 30     |
| controlField       | controlField       | controlField       | controlField       | controlField       | controlField       | controlField       | controlField       | 1      | 32     |
| logMessageInterval | logMessageInterval | logMessageInterval | logMessageInterval | logMessageInterval | logMessageInterval | logMessageInterval | logMessageInterval | 1      | 33     |

## messageType

The following encoded values are used for PTP message types:

- SYNC - 4'h0
- Delay\_Req - 4'h1
- Pdelay\_Req - 4'h2
- Pdelay\_Resp - 4'h3
- Delay\_Resp - 4'h9

## transportSpecific

The following transport protocol encoding is used:

- Default PTP over Ethernet - 4'h0
- 802.1AS mode - 4'h1

## versionPTP

It is always configured to 2 because PTP version 2 is supported.

## domainNumber

This contains the value from the EMAC\_PTO\_CTL register.

## flagField

The following values are used:

- alternateMasterFlag (Octet 0 bit 0) - 1b0 for SYNC and Delay\_Resp
- twoStepFlag (Octet 0 bit 1) - 1'b0 for SYNC and Pdelay\_Resp
- unicastFlag (Octet 0 bit 2) - 1'b0 for Multicast Address, 1'b1 for Unicast Address

## correctionField

For more information, see the Table 30-69 MAC Transmit PTP Modes table.

## sourcePortIdentity

This field takes the value programmed in the EMAC\_SRCPRT\_IDNTY0 -EMAC\_SRCPRT\_IDNTY2 registers.

## sequenceId

Pdelay\_Resp and Delay\_Resp use the same sequenceId field from received Pdelay\_Req and Delay\_Req PTP messages. For SYNC/Delay\_Req, Pdelay\_Req, a separate sequenceId counter is maintained. These sequenceId counters get incremented by 1 every time the corresponding message is generated and transmitted.

## controlField

The following encoded values are used for controlField:

- SYNC - 8'h00
- Delay\_Req - 8'h01
- Pdelay\_Req - 8'h02
- Pdelay\_Resp - 8'h05
- Delay\_Resp - 8'h03

## logMessageInterval

- SYNC - This contains logSyncInterval from the corresponding EMAC\_LOG\_MSG\_INTVL register.
- Delay\_Resp - This contains the sum of DRSYNCR and logSyncInterval value taken from the EMAC\_LOG\_MSG\_INTVL register for a multicast PTP message and 8'h7F for unicast PTP message.
- Delay\_Req, Pdelay\_Req and Pdelay\_Resp - 8'h7F where logSyncInterval = log2 (Mean Value of Interval in seconds)

The MAC supports values of -15 to 15 for logSyncInterval fields, which translates to a range from 32.768 microsecond (2-15) to 215 second. For a given value of log sync interval (N), the time interval between two SYNC packets is given by the following:

- 2(30+N) ns, when N is negative (-1 to -15)
- 2N seconds, when N is positive (0 to 15)

For example:

- When logSyncInterval is programmed to 1, the interval is 21; therefore, the SYNC message is sent once every 2 seconds.
- When logSyncInterval is programmed to -1, the interval is 2-1 = 0.536 seconds; therefore, the SYNC message is sent once every 536 milliseconds. The value is 0.536 seconds, because 2-30 = 1 ns.
- When logSyncInterval is programmed to -5, the interval is 2-5 = 33.55 ms; therefore, the SYNC message is sent once every 33.55 milliseconds.

NOTE: The MAC uses the PTP system time to generate the intervals for periodic packet transmission. For negative values of log message interval programmed, the generated period may deviate from the value given by the equation 2(30+N), because of the non-binary nature of the nanoseconds field of the system time.

## PTP Message Specific Fields

The PTP includes the following fields for messaging:

## messageLength

There is no suffix supported; therefore, this field contains the length of the PTP message that includes 34 byte PTP common header and the body specific to the message type.

For SYNC and Delay\_Req packets, this field contains 44, whereas for Delay\_Resp, Pdelay\_Req and Pdelay\_Resp, it contains 54.

## originTimestamp

This field is the captured egress time stamp for SYNC, Delay\_Req, and Pdelay\_Req PTP messages receiveTimestamp. For Delay\_Resp PTP message, this field is the ingress time stamp of the corresponding received Delay\_Req PTP message.

## requestingPortIdentity

For Delay\_Resp and Pdelay\_Resp PTP messages, this field is the sourcePortIdentity field taken from the corresponding received Delay\_Req and Pdelay\_Req PTP messages.

## requestReceiptTimestamp

For the Pdelay\_Resp PTP message, this field = 0.

## One Step Time Stamp

The EMAC supports the one step time stamp feature. When the feature is enabled, the MAC identifies the offset in the packet and inserts the time stamp received from the application at that offset.

Depending upon the type of message and its mode, the MAC updates the following fields of transmit PTP packets:

- correctionField in the PTP header of messages.

## · originTimestamp in SYNC, Delay\_Req, and Pdelay\_Req messages

The MAC Transmit PTP Modes table shows how the PTP mode is selected for the one step time stamp operation based on the configuration of EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA , and EMAC\_TM\_CTL.TSEVNTENA bits. The bits are updated for the incoming PTP packets based on the message type in that mode, during the one step time stamping operation.

Table 30-69: MAC Transmit PTP Modes

| Programming   |           |           |                                  | Per Packet Control *1   | Per Packet Control *1   | Per Packet Control *1   | Messages Pro- cessed on Tx                                                           |
|---------------|-----------|-----------|----------------------------------|-------------------------|-------------------------|-------------------------|--------------------------------------------------------------------------------------|
| SNAPTYPSE     | TSMSTRENA | TSEVNTENA | Mode                             | TTSE *2                 | OSTC *3                 | TTS *4                  |                                                                                      |
| X             | X         | X         | N/A                              | 1                       | X                       | X                       | Time stamp is captured and returned to ap- plication                                 |
| X             | X         | X         | N/A                              | X                       | 0                       | X                       | OST operation is not per- formed (PTP packet is not modified)                        |
| 2'b00         | X         | 0         | End-to-end transparent           | 0                       | 1                       | Ingress TS              | Sync (correc- tion field for residence time and In- gress Asym cor)                  |
| 2'b00         | X         | 0         | End-to-end transparent           | 0                       | 1                       | Ingress TS              | Delay_Req (correction field for resi- dence time and Egress Asym Cor                 |
| 2'b00         | 0         | 1         | Ordinary or boundary completer   | 1                       | 1                       | X                       | Delay_Req (originTimes- tamp field) Delay_Req (correction field for Egress Asym Cor) |
| 2'b00         | 1         | 1         | Ordinary or boundary re- quester | 0                       | 1                       | X                       | Sync (originTi- mestamp field) sync (correc- tion field for sub nanosec- ond cor)    |

Table 30-69: MAC Transmit PTP Modes (Continued)

| Programming   | Programming   | Programming   |                                                              | Per Packet Control *1   | Per Packet Control *1   | Per Packet Control *1   | Messages Pro- cessed on Tx                                               |
|---------------|---------------|---------------|--------------------------------------------------------------|-------------------------|-------------------------|-------------------------|--------------------------------------------------------------------------|
| SNAPTYPSE     | TSMSTRENA     | TSEVNTENA     |                                                              | TTSE *2                 | OSTC *3                 | TTS *4                  | Messages Pro- cessed on Tx                                               |
| 2'b01         | X             | 0             | End-to-end transparent with support for peer delay mechanism | 0                       | 1                       | Ingress TS              | Sync (correc- tion field for residence time and Ingress Asym Cor)        |
| 2'b01         | X             | 0             | End-to-end transparent with support for peer delay mechanism | 0                       | 1                       | Ingress TS              | Pdelay_Req (correction field for resi- dence time and Egress Asym Cor)   |
| 2'b01         | X             | 0             | End-to-end transparent with support for peer delay mechanism | 0                       | 1                       | Ingress TS              | Pdelay_Resp (correction field for resi- dence time and Ingress Asym Cor) |

Table 30-69: MAC Transmit PTP Modes (Continued)

| Programming   | Programming   | Programming   |                                                                                              | Per Packet Control *1   | Per Packet Control *1   | Per Packet Control *1     | Messages Pro- cessed on Tx                                                                                                              |
|---------------|---------------|---------------|----------------------------------------------------------------------------------------------|-------------------------|-------------------------|---------------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| SNAPTYPSE     | TSMSTRENA     | TSEVNTENA     | Mode                                                                                         | TTSE *2                 | OSTC *3                 | TTS *4                    |                                                                                                                                         |
| 2'b01         | 0             | 1             | Ordinary or boundary slave with support for peer delay mechanism or peer-to-peer transparent | 0                       | 1                       | Ingress TS                | Sync (correc- tion field for residence time and Ingress Asym Cor) (applicable on- ly for peer-to- peer transpar- ent clock oper- ation) |
| 2'b01         | 0             | 1             | Ordinary or boundary slave with support for peer delay mechanism or peer-to-peer transparent | 1                       | 1                       | X                         | Delay_Req (originTimes- tamp field) Delay_Req (correction field for Egress Asym Cor)                                                    |
| 2'b01         | 0             | 1             | Ordinary or boundary slave with support for peer delay mechanism or peer-to-peer transparent | 1                       | 1                       | X                         | Pdelay_Req (originTimes- tamp field) Pdelay_Req (correction field for Egress Asym Cor)                                                  |
| 2'b01         | 0             | 1             | Ordinary or boundary slave with support for peer delay mechanism or peer-to-peer transparent | 0                       | 1                       | Ingress TS for Pdelay_Req | Pdelay_Resp (correction field for turn- around time and Ingress Asym Cor)                                                               |

Table 30-69: MAC Transmit PTP Modes (Continued)

| Programming   |           |           |                                                                        | Per Packet Control *1   | Per Packet Control *1   | Per Packet Control *1     | Messages Pro- cessed on Tx                                                             |
|---------------|-----------|-----------|------------------------------------------------------------------------|-------------------------|-------------------------|---------------------------|----------------------------------------------------------------------------------------|
| SNAPTYPSE     | TSMSTRENA | TSEVNTENA | Mode                                                                   | TTSE *2                 | OSTC *3                 | TTS *4                    |                                                                                        |
| 2'b01         | 1         | 1         | Ordinary or boundary re- questor with support for peer delay mechanism | 0                       | 1                       | X                         | Sync (originTi- mestamp field) sync (correc- tion field for sub nanosec- ond cor)      |
| 2'b01         | 1         | 1         | Ordinary or boundary re- questor with support for peer delay mechanism | 1                       | 1                       | X                         | Pdelay_Req (originTimes- tamp field) Pdelay_Req (correction field for Egress Asym Cor) |
| 2'b01         | 1         | 1         | Ordinary or boundary re- questor with support for peer delay mechanism | 0                       | 1                       | Ingress TS for Pdelay_Req | Pdelay_Resp (correction field for turn- around time and Ingress Asym Cor)              |
| 2'b10         | X         | X         | End-to-end transparent                                                 | 0                       | 1                       | Ingress TS                | Sync (correc- tion field for residence time and ingress Asym cor)                      |
| 2'b10         | X         | X         | End-to-end transparent                                                 | 0                       | 1                       | Ingress TS                | Delay_Req (correction field for resi- dence time and Egress Asym Cor)                  |

Table 30-69: MAC Transmit PTP Modes (Continued)

| Programming   | Programming   | Programming   |                          | Per Packet Control *1   | Per Packet Control *1   | Per Packet Control *1     | Messages Pro- cessed on Tx                                                             |
|---------------|---------------|---------------|--------------------------|-------------------------|-------------------------|---------------------------|----------------------------------------------------------------------------------------|
| SNAPTYPSE     | TSMSTRENA     | TSEVNTENA     |                          | TTSE *2                 | OSTC *3                 | TTS *4                    | Messages Pro- cessed on Tx                                                             |
| 2'b11         | X             | X             | Peer-to-peer transparent | 0                       | 1                       | Ingress TS                | Sync (correc- tion field for residence time and Ingress Asym Cor)                      |
| 2'b11         | X             | X             | Peer-to-peer transparent | 1                       | 1                       | X                         | Pdelay_Req (originTimes- tamp field) Pdelay_Req (correction field for Egress Asym Cor) |
| 2'b11         | X             | X             |                          | 0                       | 1                       | Ingress TS for Pdelay_Req | Pdelay_Resp (correction field for turn- around time and Ingress Asym Cor)              |

- *1 The per packet control values provided here are the recommended settings used by devices in typical PTP operation, for the programmed mode.
- *2 TTSE represents TTSE bit of transmit descriptor. The TTSE function is independent of the OST function and the programmed operation mode for OST. The MAC captures and returns the time stamp when the TTSE bit is set.
- *3 OSTC represents OSTC bit of transmit descriptor.
- *4 TTS represents the time stamp value provided in the TTSH, TTSL fields of transmit descriptor.

NOTE: Residence time/ turnaround time is calculated as the difference between the captured time stamp (egress time stamp) and the ingress time stamp. When the sub nanosecond feature is enabled, the residence time calculation includes sub nanosecond accuracy. Clocks supporting peer delay mechanism do not use delay request or response; however, it is included in OST for flexibility.

## Enabling One Step Time Stamp

Programs can enable the one step time stamp feature for a packet by setting bit 20 (OSTC) in the transmit descriptor. For updating the correction field in certain PTP packets, the ingress time stamp must be given in the TSSL and TSSH fields. When the added IEEE 1588 higher word register option is selected, the time stamp is 80 bits. It also contains the content of the higher 16 bits of the EMAC\_TM\_HISEC register.

## One Step Time Stamping for PTP Over UDP

The EMAC supports one step time stamp operation for PTP messages sent over UDP/IPv4 and UDP/IPv6. Because origin time stamp and/or correction field (based on the PTP message type and the programmed PTP mode) in the

PTP message are updated as part of the one-step time stamp operation, it also requires update to the checksum field of the PTP message sent over UDP/IP .

## Checksum Update for One-Step Timestamping for PTP Over UDP

In the EMAC, checksum is updated for a packet, in the MTL before the packet is sent to the MAC where the PTP message is updated for one step time stamp operation. PTP requires the time stamp to be captured while the packet (SFD of the packet) is on the line and one step time stamp requires the origin time stamp and/or correction field to be updated in the PTP header while the packet is being transmitted on the line. Because the PTP message is present in the UDP payload, it is not possible to update the checksum field in the UDP header for modifications made to the PTP message, because it is already transmitted by then. Instead, the following options are supported that are in line with specification mentioned in Annex D and E of IEEE 1588-2008:

- When PTP message is sent over UDP/IPv4, the UDP checksum may be set to 0. The application must set the UDP checksum to 0 and set CIC to 0 or 1 while providing the packet to the MAC and the MAC does not update checksum for changes in origin time stamp or correction field.
- When PTP message is sent over UDP/IPv6, a transmitting node extends the UDP payload of all PTP messages by 2 octets beyond the end of the PTP message. The contents of the UDP checksum field or the final 2 octets of the UDP payload might be modified to ensure that the UDP checksum remains intact after modification of PTP fields. The application must add the two extra bytes at the end of the PTP message while providing the packet to the MAC and the MAC updates these two bytes to keep the checksum correct.

A programmable option ( EMAC\_TM\_CTL.CSC field) supports UDP with a non-zero checksum for IPv4. When set, the two bytes at the end of the PTP message are updated to keep the UDP checksum correct as with PTP over UDP/IPv6. The application must add two bytes at the end of the PTP message sent to the EMAC and the EMAC updates these two bytes to keep the checksum correct.

Updating the pad bytes to keep the checksum correct requires the old value of correction field and/or origin time stamp in the packet. To get better latency and area, the application can set the origin time stamp field to zero, for PTP message in particular modes in which EMAC overwrites the origin time stamp. For example, EMAC operating in ordinary/boundary requester mode overwrites the origin time stamp field of the sync message, when OSTC is set. Because the sync message is originated by the port operating in ordinary/boundary requester mode, it is possible for the application generating the message to set the origin time stamp field to zero.

The transmit checksum engine is automatically enabled when this feature is selected. The EMAC uses the transmit checksum engine to identify the start of a PTP message in the UDP/IP packet. The checksum engine has the capability to detect errors in the header (L3) and payload (L4). These errors are not computed for some type of packets, which is indicated by "No" in the Transmit Checksum Offload Engine Functions for Different Packet Types table in the Synopsis DesignWare® Cores Ethernet Quality-of-Service Databook .

For one step, operation the packet is free of those errors and have "Yes/Not applicable" for the corresponding packet type in the Transmit Checksum Offload Engine Functions for Different Packet Types table. If an error is reported by the checksum engine or there is a "No" in For PTP messages sent over UDP/IP , no update is performed on the PTP message. See 'Transmit Checksum Offload Engine' of the Synopsis DesignWare® Cores Ethernet Quality-of-Service Databook for more details about the errors detected by transmit checksum engine.

- NOTE: · The packet is sent with specific IP address and UDP port numbers and does not have IPv4 options/ IPv6 extension headers. Applications must take care of these requirements before giving the packet to the EMAC. EMAC does not validate them. As PTP is identified by specific value in the UDP destination port and as EMAC does not investigate these fields, it assumes an IP packet with OSTC set as a PTP packet, and it is the responsibility of the application to set the OSTC bit only for IP packets which contains PTP messages.
- For PTP over UPD/IPv6 and PTP over UDP/IPv4 with CSC set, the EMAC does not verify the presence of the two pad bytes, instead it overwrites the two bytes after the PTP message.
- When the enable one step time stamp for PTP over UDP/IP feature is selected, the enable transmit TCP/IP checksum offload feature is automatically enabled, and this is enabled only for queue 0 in multiple transmit queue configurations. If the application requires OST support for PTP messages sent over UDP/IP for packets sent from queues other than 0, transmit checksum engine must be manually enabled for those transmit queues. If checksum engine is not enabled, no updates are done for PTP messages sent over UDP/IP even though OSTC is set.
- One step time stamp operation is not supported for tunneled packets

## Multiple Channels and Queues Support

The EMAC supports up to eight queues and channels on transmit and receive paths. The EMAC Block Diagram and Interfaces figure shows the architecture of EMAC with multiple queues and channels.

## Multiple Queues Support in the Transmit Path

The EMAC supports up to eight queues and channels on transmit and receive paths.

- NOTE: EMAC0 supports eight transmit DMA channels and queues. EMAC1 supports one transmit DMA channel and queue.

The number of transmit DMA channels is equal to the number of transmit queues and is direct one-to-one mapping. This enables the interleaving of packet/data transfer of multiple transmit DMA on the host/system bus and each transmit queue is reserved for the corresponding transmit DMA. Therefore, the integrity of packet transfer by each transmit DMA is maintained in the transmit queue and enables efficient use of the host bus for data transfers among multiple transmit DMAs without depending on the packet length, transmit buffer availability and so on.

When two transmit DMAs can transfer data into the same transmit queue, then a transmit DMA starts a packet transfer, the other transmit DMA cannot transfer data to the same transmit queue unless the previous packet transfer is complete. This means that the second DMA remains idle until the first packet transferred is complete, which is effectively a one-to-one mapping of the transmit DMA to the transmit queue.

This section describes how multiple queues and channels are operated in the EMAC.

## Fixed Priority Scheme

The fixed priority scheme does not necessarily cause 'starving' of lower priority channels for the SCB bus because of the capabilities of the SCB protocol and characteristics of the transmit DMA, described as follows:

- Read command requests from a transmit DMA for a burst of data transfers is driven on a separate SCB channel and is executed in one clock cycle itself. The actual data transfer transfers start on a separate SCB channel only after the request is accepted and it takes multiple clock cycles to complete with relatively large read access latencies.
- SCB requester is free to drive out subsequent requests even before the previous data transfers are completed.
- Transmit DMA requests for another data transfer only after the data transfer for its previous request is completed and transferred to the MTL TxQ, and space is available in TxQ to take in the next request.
- The EMAC can be programmed to control the maximum number of outstanding requests, requested burst sizes by DMA, and the burst sizes of each data transfer on the SCB with the help of the EMAC\_DMA\_SYSBMODE register.

When all transmit DMAs place a request simultaneously and are serviced by the fixed priority algorithm, all the requests are placed on the SCB bus in a window of 32 clocks (two clocks per SCB request, two SCB burst requests per transmit DMA request). The data transfer of the first request takes 32 clock cycles + the read-access latencies (when there are tens of cycles). Therefor the transmit DMA arbiter is already IDLE and has completed servicing all the transmit DMA requests before this data transfer completes.

Effectively, all the transmit DMA channels get equal access to SCB bus and the corresponding TxQs are filled up at the same rate. The data is read out from the TxQs as per the traffic class scheduler in the MTL. At a steady state (assuming that SCB bandwidth available to fetch data from system memory is always more than the operating line rate), the bandwidth for each channel is controlled by the priorities and settings of the traffic scheduler in the MTL.

## Multiple Queues in the Receive Path

The EMAC supports up to eight receive channels. The number of receive channels is independent of the receive queues.

NOTE: EMAC0 supports eight receive DMA channels and queues. EMAC1 supports 1 receive DMA channel and queue.

The EMAC supports independent selection of the number of receive DMA channels and the number of receive queues in the MTL receive buffer. Therefore, the packets from any receive queue can be serviced by any of the receive DMA. As all the packets of receive queues are stored in a shared RXFIFO buffer in the MTL, the receive scheduler selects the receive queue whose packet data is to be transferred to the destination receive DMA, which then forwards the packet data to the host memory. The sequence of events in time are as follows:

1. Each receive DMA indicates whether it is ready to transfer data, that is, when the receive DMA has fetched the descriptor and has a receiver buffer ready, to accept the packet data. Each receive DMA also indicates the number of words that it can transfer, based on the space available in the receive buffer and the value programmed in the EMAC\_DMA[n]\_CTL.PBLX8 bit field.
2. The MTL receive scheduler matches the number of bytes/packets available for transfer at the top of all receive queues with the corresponding receive DMA ready status and the number of bytes requested by it.
3. If there are multiple successful matches, the scheduler selects the receive queue to be serviced, based on the arbitration schemes (strict priority or weighted strict priority) decided by the EMAC\_MTL\_OPMODE.RAA bit

field. In the weighted strict priority scheme, the weights of each receive queue is set in the EMAC\_RQ[n]\_CTL.RXQ\_WEGT bit field.

4. The MTL receive controller fetches the data of the selected receive queue from the shared RXFIFO memory and starts the transfer of the requested PBL number of bytes to the DMA block or until the end of packet and the corresponding receive status is transferred (whichever is first).
5. After completing the current request, the scheduler repeats the process by starting a new arbitration cycle. However, it selects the same queues queue for servicing, if the end of packet is not transferred and the EMAC\_RQ[n]\_CTL.RXQ\_FRM\_ARBIT bits are 1, for the receive queue.

## Receive Queue to DMA Mapping

The packets in the MTL receive queues can be routed to any one of the multiple DMA channels by programming the following registers:

- EMAC\_RXQ\_DMA\_MAP0 register (for queues 0, 1, 2 and 3)
- EMAC\_RXQ\_DMA\_MAP1 register (for queues 4, 5, 6 and 7)

The following types of receive queue to DMA mapping is possible EMAC\_DMA[n]\_IEN.RSE through programming:

- Static mapping
- Dynamic (per packet) mapping

## Static Mapping

In static mapping mode, all packets of a receive queue are connected to a specific DMA channel. For example, all the packets from receive queue 0 can be routed to a DMA channel by programming the EMAC\_RXQ\_DMA\_MAP0.Q0MDMACH and EMAC\_RXQ\_DMA\_MAP0.Q0DDMACH bit fields.

Similarly, packets from other receive queues can be routed to any DMA channel by programming register fields corresponding to each queue.

## Dynamic Mapping

In Dynamic (per packet) mapping mode, the destination DMA channel is chosen by the MAC core receiver for each packet.

In this mode, the destination DMA channel of a packet being read from a receive queue is not constant but decided independently for each packet. For example, if the EMAC\_RXQ\_DMA\_MAP0.Q1MDMACH bit is set, the static mapping is disabled for receive queue 1 and the value in the EMAC\_RXQ\_DMA\_MAP0.Q1MDMACH bit is ignored. The destination DMA channel is chosen by the MAC receiver for each packet, depending on the following in decreasing order of priority.

1. L3-L4 filter Based DMA Selection. When the enable layer 3 and layer 4 packet filter feature is present and enabled in the program configuration, the TCP/UDP and IP header fields of the received packet are matched against the corresponding values programmed and enabled for comparison in the EMAC\_L3L4\_CTL[i] register. If the match is successful, the DMA channel number programmed in the
2. EMAC\_L3L4\_CTL[i].DMCHN bit field is selected as the destination DMA channel number provided the

EMAC\_L3L4\_CTL[i].DMCHEN bit of the same register is set. If none of the L3-L4 registers give a comparison match, the EMAC proceeds to the next step.

2. Extended VLAN Based DMA Selection. Extended routing is applicable only if the VLAN filter has passed. Routing is only based on perfect filter result. Each perfect filter has a programmable DMA channel enable and a DMA channel number field. Routing is applicable for a filter when the DMA channel enable bit is set.

The frame is routed to the smallest matching filter's DMA channel provided it is enabled. If that filter's DMA channel number is not enabled, the frame gets routed to channel 0. For example, if a frame's VLAN tag matches filters 7, 3, and 1. Then the MAC checks if filter 1's DMA channel number is enabled through programming. If yes, the frame is routed to the programmed value; otherwise it is routed to channel 0. When the inverse filter is enabled, the frame is routed to the least mismatched filter's DMA channel number provided it is enabled. If the DMA channel enable bit is not set, then the frame is routed based on DA based addressing or to channel 0.

If the hash filter is also enabled, it is used to determine the filter result only. Routing still depends on the enabled perfect filters. If none of the perfect filters are enabled or if all of them are bypassed, the VLAN filter based routing is not performed. The frame is routed through DA based addressing or to channel 0. If all the perfect filters give a fail result and the hash filter has passed, the VLAN filter result is a pass. However, routing is based on DA based addressing or to channel 0. The behavior is similar when inverse filtering is enabled.

3. Ethernet DA-Based DMA Selection. The DA address of the received packet is compared against the programmed DA values in MAC address registers. If the address matches any of the programmed values, the corresponding EMAC\_ADDR[i]\_HI.DCS bit field (when enabled) determines the destination DMA channel number. If none of the previous operations can make a successful match/decision, then the packet is routed to DMA channel 0 by default.

## Broadcast/Multicast Packet Duplication

The EMAC supports the Broadcast/Multicast Packet Duplication feature to send the received broadcast/multicast packets to multiple DMA channels.

In a virtual system that supports multiple guest OS's (for instance, each OS owning a DMA channel), where each DMA could act as an independent Ethernet node, it may be required to route the received multicast/broadcast packets to all/multiple DMA channels. This is implemented by using the MTL RX FIFO read controller to read the packet from the same queue multiple times and route the same packet to the specified multiple DMA channels.

To maintain backward compatibility, through software control, the DMA channel selection ( EMAC\_ADDR[i]\_HI.DCS ) field are redefined to denote multiple DMA channels.

The EMAC\_ADDR[i]\_HI.DCS bit field determines the DMA channel number to which the received packet (that matches the MAC address present in that register) must be routed.

The EMAC\_ADDR[i]\_HI.DCS is a programmable field that supports more than one DMA channel number to be selected.

The EMAC\_ADDR[i]\_HI.DCS field can hold any one of the following values:

- Binary value of the channel number when the EMAC\_MAC\_EXT\_CFG.PDC bit is set to 0

- One-hot value of the channel number when the EMAC\_MAC\_EXT\_CFG.PDC bit is set to 1

Use DA based routing to enable the multicast/broadcast packets to route to multiple DMA channels. Packet duplication is not supported in other DMA channel routing mechanisms such as extended VLAN, or L3-L4 based routing. In other words if the destination DMA channel is decided by the VLAN or L3-L4 filter result, it is routed only to the specific channel even if the DA of the packet matches the multicast address in the EMAC\_ADDR[i]\_HI / EMAC\_ADDR[i]\_LO registers.

The packet duplication feature is supported only on the highest MTL queue configured. Therefore, packet duplication for broadcast/multicast packets requires that the EMAC\_RXQ\_CTL1.MCBCQ bit field is programmed to the highest RxQ present in the configuration.

In packet duplication, the MTL read controller sequentially transfers the same packet to all the required receive DMA channels. This packet is removed from the MTL FIFO only after the packet transfer completes. Therefore, this feature is suitable for smaller packet sizes or very low bandwidth traffic. This ensures that the MTL RxQ does not overflow due to the long period when the packet remains in the queue during the packet duplication.

NOTE: When more than one bit is set for the DMA channel number, a packet that is not routed to the highest receive queue is routed to only one DMA channel as indicated by the lowest bit set in the EMAC\_ADDR[i]\_HI.DCS bit field.

## Assigning Tag Priorities to Transmit and Receive Queues

The EMAC supports assigning tag priorities to transmit and receive queues through programming registers.

The VLAN tag priorities can be assigned to transmit queues by programming the EMAC\_TXQ\_PRTY\_MAP0.PSTQ0 through EMAC\_TXQ\_PRTY\_MAP1.PSTQ7 bit fields in the following registers:

- EMAC\_TXQ\_PRTY\_MAP0 (for transmit queues 0 to 3)
- EMAC\_TXQ\_PRTY\_MAP1 (for transmit queues 4 to 7)

For example, if the program needs to assign a VLAN tag priority of 3 to transmit queue 0, set bit [3] in the EMAC\_TXQ\_PRTY\_MAP0.PSTQ0 bit field.

The same VLAN tag priority can be set for multiple transmit queues. The transmit packet association to transmit queue is governed by the application, so the MAC does not route the packet based on transmit queue priority mapping; it is only used for Pause Flow Control (PFC). The settings in the PSTQ[n] field determines the transmit queues blocked for transmission when the PFC packet is received with corresponding VLAN tag priorities enabled.

The VLAN tag priorities can be assigned to receive queues by programming the EMAC\_RXQ\_CTL2.PSRQ0 through EMAC\_RXQ\_CTL3.PSRQ7 bit fields. The bit corresponding to the VLAN tag priority can be set in the PSRQ field for assigning that priority to the receive queue. For example, to assign a VLAN tag priority of 3 to receive queue 0, set bit [3] in the EMAC\_RXQ\_CTL2.PSRQ0 bit field. The VLAN tag priority assigned to the receive queue must be unique, that is, more than one receive queue cannot be assigned the same VLAN tag priority. However, more than one VLAN tag priority can be assigned to same receive queue.

The settings in the PSRQ bit field is used for VLAN tagged receive packet routing to receive queues as well as for PFC based transmit flow control. The received VLAN tagged receive packet is routed to the receive queue that has the VLAN tag priority match. In PFC based transmit flow control, the PSRQ bit field corresponding to a particular receive queue is used for enabling VLAN tag priorities in the PFC packet transmitted when corresponding receive queue threshold levels are reached.

## Distribution of Receive Packets from MAC to Receive Queues

The EMAC provides multiple receive queues to support classification and distribution of ingress packets. The MAC receiver pushes the ingress packets to the receive queues based on packet type (tagged, untagged, multicast, PTP , AVB, DCB, and so on), tag priorities, MAC packet filter results, preemptable or express packet, and so on. The distribution is controlled by the settings of the EMAC\_RXQ\_CTL0 through EMAC\_RXQ\_CTL4 and EMAC\_MACPKT\_FILT register fields.

EMAC additionally supports distribution of ingress packets based on the user-defined type field of the ingress packets when the EMAC\_RXQ\_CTL1.TBRQE bit is enabled. Up to 8 types for matching are programmed in the MAC\_TMRQ\_Regs(n) registers which are accessible through the EMAC\_INDR\_ACC\_CTL and EMAC\_INDR\_ACC\_DAT registers. Each MAC\_TMRQ\_Regs(n) contains

- TYP field: contains 16-bit type value of the packet that needs to be compared with the received Ethernet packet. This field is valid when programmed to a value greater than or equal to 0x0600.
- TMRQ field: contains the receive queue number to which ingress packet is pushed when its type field matches TYP.
- PFEX field: This bit indicates whether the TYP is applicable for express packets or preemptable packets.

Based on the packet filter settings, the MAC receiver decides to either drop or forward the ingress packet. However, the results of layer 3 and layer 4 filtering, mentioned in Layer 3 and Layer 4 Frame Filtering are not considered for the distribution of ingress packets into the receive queues. Only the results of DA/SA/VLAN filters mentioned in Source Address or Destination Address Filtering and VLAN Filtering are applicable.

Filtering of the broadcast packets is exclusively controlled by the EMAC\_MACPKT\_FILT.DBF bit. The filtering of multicast packets is controlled by the EMAC\_MACPKT\_FILT.PM bit, in addition to the DA/SA match.

By default, packets that fail any of the DA/SA filters, are dropped by the MAC receiver. However, packets that fail the DA/SA/VLAN filter can still be forwarded to the application when the EMAC\_MACPKT\_FILT.RA bit is 1. Even when the RA field is 0, VLAN filter fail packets can be forwarded if the EMAC\_MACPKT\_FILT.VTFE bit is 0.

The ingress packets go through the following operations in the decreasing order of priority.

RA = 0: Ingress packets are dropped when any one of the following conditions is true.

- DA/SA filter fail
- VLAN filter fail and EMAC\_MACPKT\_FILT.VTFE bit is 1

EMAC\_MACPKT\_FILT.RA = 1: Ingress packets that fail the DA/SA filter are also forwarded to the receive queue as mentioned in the Routing DA/SA Failed Packets to Receive Queue table.

Table 30-70: Routing DA/SA Failed Packets to Receive Queue

| Packet Type         | TSN Type    | Destination Receive Queue Determined By                                          |
|---------------------|-------------|----------------------------------------------------------------------------------|
| Broadcast/Multicast | Express     | if MFFQE = 1, MFFQ; else if VFFQE = 1, VFFQ; (only for tagged packets) else RxQ0 |
| Unicast             | Express     | if MFFQE = 1, MFFQ; else if VFFQE = 1, VFFQ; (only for tagged packets) else RxQ0 |
| All                 | Preemptable | FPRQ field                                                                       |

EMAC\_MACPKT\_FILT.RA = 1 and EMAC\_MACPKT\_FILT.VTFE = 0: Ingress tagged packets that pass the DA/SA filter but fail the VLAN filter, are forwarded to the receive queue as mentioned in the Routing DA/SA Pass But VLAN Filter Fail Packets to Receive Queue table.

Table 30-71: Routing DA/SA Pass But VLAN Filter Fail Packets to Receive Queue

| TSN Packet Type   | Destination Receive Queue Determined By   |
|-------------------|-------------------------------------------|
| Express           | if VFFQE = 1, VFFQ; else RxQ0             |
| Preemptable       | RQ field                                  |

Ingress packets that pass the DA/SA/VLAN filters are forwarded to the receive queue. The Priority Routing of Filter Pass Packets When OMCBCQ = 0 table shows the routing priority in the decreasing order when the EMAC\_RXQ\_CTL1.OMCBCQ bit field is 0.

Table 30-72: Priority Routing of Filter Pass Packets When OMCBCQ = 0

| Packet Type                 | TSN Type   | Tagged   | Destination Receive Queue Determined By                   |
|-----------------------------|------------|----------|-----------------------------------------------------------|
| Broadcast/Multicast         | Express    | Yes      | if MCBCQEN = 1, MCBCQ; else if TBRQE = 1, TBRQ; else PSRQ |
| Broadcast/Multicast         | Express    | No       | if MCBCQEN = 1, MCBCQ; else if TBRQE = 1, TBRQ; else UPQ  |
| Unicast AVTP Control        | Express    | No       | AVCPQ                                                     |
| Unicast AVTP Control        | Express    | Yes      | if TACPQE = 1, AVCPQ; else PSRQ                           |
| Unicast PTP over Ether- net | Express    | No       | PTPQ                                                      |
| Unicast PTP over Ether- net | Express    | Yes      | PTPQ or PSRQ based on the value of TPQC                   |
| Unicast DCB/LLDP Control    | Express    | No       | DCBPQ                                                     |

Table 30-72: Priority Routing of Filter Pass Packets When OMCBCQ = 0 (Continued)

| Packet Type       | TSN Type    | Tagged   | Destination Receive Queue Determined By   |
|-------------------|-------------|----------|-------------------------------------------|
| Remaining Unicast | Express     | No       | If TBRQE = 1, TBRQ; else UPQ              |
|                   |             | Yes      | If TBRQE = 1, TBRQ; else PSRQ             |
| All Packets       | Preemptable | No       | If TBRQE = 1, TBRQ; else FPRQ             |
|                   |             | Yes      | If TBRQE = 1, TBRQ; else PSRQ             |

The Priority Routing of Filter Pass Packets When OMCBCQ = 1 table shows the routing priority in the decreasing order when the EMAC\_RXQ\_CTL1.OMCBCQ bit field is 1.

Table 30-73: Priority Routing of Filter Pass Packets When OMCBCQ = 1

| Packet Type                    | TSN Type    | Tagged   | Destination Receive Queue Determined By                   |
|--------------------------------|-------------|----------|-----------------------------------------------------------|
| All AVTP Control               | Express     | No       | AVCPQ                                                     |
| All AVTP Control               | Express     | Yes      | if TACPQE = 1, AVCPQ; else PSRQ                           |
| All PTP over Ethernet          | Express     | No       | PTPPQ                                                     |
| All PTP over Ethernet          | Express     | Yes      | PTPQ or PSRQ based on TPQC                                |
| Remaining Broadcast/ Multicast | Express     | Yes      | if MCBCQEN = 1, MCBCQ; else if TBRQE = 1, TBRQ; else PSRQ |
| Remaining Broadcast/ Multicast | Express     | No       | if MCBCQEN = 1, MCBCQ; else if TBRQE = 1, TBRQ; else UPQ  |
| Unicast DCB/LLDP Control       | Express     | No       | DCBPQ                                                     |
| Remaining Unicast              | Express     | No       | If TBRQE = 1, TBRQ; else UPQ                              |
| Remaining Unicast              | Express     | Yes      | If TBRQE = 1, TBRQ; else PSRQ                             |
| All Packets                    | Preemptable | No       | If TBRQE = 1, TBRQ; else FPRQ                             |
| All Packets                    | Preemptable | Yes      | If TBRQE = 1, TBRQ; else PSRQ                             |

NOTE: If the RxQ pointed to by the various fields in tables above is not enabled in the EMAC\_RXQ\_CTL0 register as per the packet type, the packet is written to

- RxQ0 for express packets
- FPRQ field for preemptable packets
- If RxQ0/FPRQ is not enabled, the packet is dropped.

If the EMAC\_TM\_CTL.AV8021ASMEN bit is 1, the MAC receiver checks only for untagged PTP packets. Tagged packets with a PTP payload are considered as remaining (Unicast/Broadcast/Multicast) tagged packets in the above tables.

## Traffic Shaper

Traffic shaping is the process of distributing frames/packets evenly in time to smooth out network traffic. This reduces traffic bursts which can overwhelm the network. The following sections describe how to implement and manage this feature.

## Data Center Bridging

The data center bridging feature supports three types of algorithms: weighted round robin, deficit weighted round robin, and weighted fair queuing.

## Weighted Round Robin

In the Weighted Round Robin algorithm used in the Data Center Bridging feature, all queues are serviced in roundrobin order based on weight setting.

In Weighted Round Robin (WRR) algorithm, each queue is assigned a weight based on the percentage of configured bandwidth. All queues are serviced in the round-robin order according to the weight settings. This algorithm is less complex and requires less hardware as compared to other algorithms. Large packets get more bandwidth in this algorithm.

50% BANDWIDTH

Figure 30-35: Weighted Round Robin Example

<!-- image -->

In the Weighted Round Robin Example figure, Queue 2, Queue 1, and Queue 0 are configured for 50%, 25%, and 25% bandwidth, respectively. The corresponding weights are two, one, and one. Before the WRR scheduler is started, Queue 2 has four packets; Queue 1 has three packets; and Queue 0 has three packets. The packets are transmitted in the following order:

Table 30-74: Weighted Round Robin Sequence Example

| Packet Sequence   | Queue Number   |
|-------------------|----------------|
| Packet 1          | Queue 2        |
| Packet 2          | Queue 2        |
| Packet 3          | Queue 1        |
| Packet 4          | Queue 0        |

Table 30-74: Weighted Round Robin Sequence Example (Continued)

| Packet Sequence   | Queue Number   |
|-------------------|----------------|
| Packet 5          | Queue 2        |
| Packet 6          | Queue 2        |
| Packet 7          | Queue 1        |
| Packet 8          | Queue 0        |
| Packet 9          | Queue 1        |
| Packet 10         | Queue 0        |

## Deficit Weighted Round Robin

When the Data Center Bridging feature is supported, programs can use the deficit weighted round robin algorithm to assign a weight based on the percentage of configured bandwidth. A deficit counter holds the credit for transmission.

In Deficit Weighted Round Robin (DWRR) algorithm, each queue is assigned a weight based on the percentage of configured bandwidth. A deficit counter holds the credit for transmission. The quantum value, proportional to the bandwidth, is added to the deficit counter every time a queue is scanned. The packet in the queue is selected for transmission only if the credit in the deficit counter is less than or equal to the size of the packet available at the head of the queue. A selected queue is serviced until the outstanding credit is lesser than the packet size at the head of the queue during a particular queue scan cycle. After this, the next lower priority queue is serviced. When the packets in a queue are over, the credit becomes zero for that queue scan cycle. This algorithm is less complex, and it provides more fairness in scheduling as compared to other algorithms.

Figure 30-36: Deficit Weighted Round Robin Example

<!-- image -->

In the Deficit Weighted Round Robin Example figure, Queue 2, Queue 1, and Queue 0 are configured for 50%, 25%, and 25% bandwidth, respectively. The corresponding quantum values are 1000, 500, and 500. Before the DWRR scheduler is started, each queue has three packets. The Deficit Weighted Round Robin table shows the queues scan

cycle number and the packet size at the head of queues, deficit counter value before and after packet in selected queue is transmitted, and queue scheduled for transmission.

Table 30-75: Deficit Weighted Round Robin

| Queues Scan Cycle Number   | Packet Size at the Head of Queue   | Packet Size at the Head of Queue   | Packet Size at the Head of Queue   | Deficit Counter Value Before the Packet is Transmitted in Selected Queue   | Deficit Counter Value Before the Packet is Transmitted in Selected Queue   | Deficit Counter Value Before the Packet is Transmitted in Selected Queue   | Deficit Counter Value after the Packet is Transmitted in Selected Queue   | Deficit Counter Value after the Packet is Transmitted in Selected Queue   | Deficit Counter Value after the Packet is Transmitted in Selected Queue   | Queue Scheduled for Trans- mission   |
|----------------------------|------------------------------------|------------------------------------|------------------------------------|----------------------------------------------------------------------------|----------------------------------------------------------------------------|----------------------------------------------------------------------------|---------------------------------------------------------------------------|---------------------------------------------------------------------------|---------------------------------------------------------------------------|--------------------------------------|
| Queue                      | Q2                                 | Q1                                 | Q0                                 | Q2                                                                         | Q1                                                                         | Q0                                                                         | Q2                                                                        | Q1                                                                        | Q0                                                                        |                                      |
| 1                          | 600                                | 400                                | 600                                | 1000                                                                       | 500                                                                        | 500                                                                        | 400                                                                       | 500                                                                       | 500                                                                       | Q2                                   |
| 1                          | 300                                | 400                                | 600                                | 400                                                                        | 500                                                                        | 500                                                                        | 100                                                                       | 500                                                                       | 500                                                                       | Q2                                   |
| 1                          | 400                                | 400                                | 600                                | 100                                                                        | 500                                                                        | 500                                                                        | 100                                                                       | 100                                                                       | 500                                                                       | Q1                                   |
| 1                          | 400                                | 300                                | 600                                | 100                                                                        | 100                                                                        | 500                                                                        | 100                                                                       | 100                                                                       | 500                                                                       | None                                 |
| 2                          | 400                                | 300                                | 600                                | 1100                                                                       | 600                                                                        | 1000                                                                       | 700                                                                       | 600                                                                       | 1000                                                                      | Q2                                   |
| 2                          | N/A                                | 300                                | 600                                | 0                                                                          | 600                                                                        | 1000                                                                       | 0                                                                         | 300                                                                       | 1000                                                                      | Q1                                   |
| 2                          | N/A                                | 400                                | 600                                | 0                                                                          | 300                                                                        | 1000                                                                       | 0                                                                         | 300                                                                       | 400                                                                       | Q0                                   |
| 2                          | N/A                                | 400                                | 300                                | 0                                                                          | 300                                                                        | 400                                                                        | 0                                                                         | 300                                                                       | 100                                                                       | Q0                                   |
| 2                          | N/A                                | 400                                | 400                                | 0                                                                          | 300                                                                        | 100                                                                        | 0                                                                         | 300                                                                       | 100                                                                       | None                                 |
| 3                          | N/A                                | 400                                | 400                                | 0                                                                          | 800                                                                        | 600                                                                        | 0                                                                         | 400                                                                       | 600                                                                       | Q1                                   |
| 3                          | N/A                                | N/A                                | 400                                | 0                                                                          | N/A                                                                        | 600                                                                        | 0                                                                         | 0                                                                         | 200                                                                       | Q0                                   |
| 3                          | N/A                                | N/A                                | N/A                                | 0                                                                          | N/A                                                                        | N/A                                                                        | 0                                                                         | 0                                                                         | 0                                                                         | None                                 |

## Weighted Fair Queuing

In the Weighted Fair Queuing (WFQ) algorithm used in the Data Center Bridging feature, each queue is assigned a weight based on the percentage of configured bandwidth. The algorithm computes the finish time of packets at the head of the queues based on the packet size and weights for the queue. The packets with earliest finish time is scheduled or transmission first. The algorithm is more complex and less scalable as compared to other algorithms.

Figure 30-37: Weighted Fair Queuing Example

<!-- image -->

In the Weighted Fair Queuing Example figure, Queue 2, Queue 1, and Queue 0 are configured for 50%, 25%, and 25% bandwidth, respectively. The corresponding weights are 2, 1, and 1. Before the WFQ scheduler is started, each queue has three packets. The Weighted Fair Queuing Algorithm Example Values table shows the packet size at the head of the queue, computed finish time, and the queue scheduled for transmission.

Table 30-76: Weighted Fair Queuing Algorithm Example Values

| Packet size at the Head of the Queue   | Packet size at the Head of the Queue   | Packet size at the Head of the Queue   | Computed Finish Time in Clock Cycles (packet size or weights)   | Computed Finish Time in Clock Cycles (packet size or weights)   | Computed Finish Time in Clock Cycles (packet size or weights)   | Queue Scheduled for Transmission   |
|----------------------------------------|----------------------------------------|----------------------------------------|-----------------------------------------------------------------|-----------------------------------------------------------------|-----------------------------------------------------------------|------------------------------------|
| Q2                                     | Q1                                     | Q0                                     | Q2                                                              | Q1                                                              | Q0                                                              |                                    |
| 30                                     | 70                                     | 135                                    | 15                                                              | 70                                                              | 135                                                             | Q2                                 |
| 50                                     | 70                                     | 135                                    | 25                                                              | 70                                                              | 135                                                             | Q2                                 |
| 150                                    | 70                                     | 135                                    | 75                                                              | 70                                                              | 135                                                             | Q1                                 |
| 150                                    | 110                                    | 135                                    | 75                                                              | 110                                                             | 135                                                             | Q2                                 |
| N/A                                    | 110                                    | 135                                    | N/A                                                             | 110                                                             | 135                                                             | Q1                                 |
| N/A                                    | 145                                    | 135                                    | N/A                                                             | 145                                                             | 135                                                             | Q0                                 |
| N/A                                    | 145                                    | 155                                    | N/A                                                             | 145                                                             | 155                                                             | Q1                                 |
| N/A                                    |                                        | 155                                    | N/A                                                             | N/A                                                             | 155                                                             | Q0                                 |
| N/A                                    |                                        | 190                                    | N/A                                                             | N/A                                                             | 190                                                             | Q0                                 |

## Audio Video Bridging

The EMAC supports the AV data transfer in 100 Mbps and 1000 Mbps modes. The AV feature enables transmission of time-sensitive traffic over bridged local area networks (LANs). The following standards define various aspects of the AV feature implementation:

- IEEE 802.1Qav-2009: Allows the bridges to provide time-sensitive and loss-sensitive real-time audio video data transmission (AV traffic). It specifies the priority regeneration and controlled bandwidth queue draining algorithms that are used in bridges and AV traffic sources.
- IEEE 802.1Qat-2009: Allows the network resources to be reserved for specific traffic streams traversing a bridged local area network.
- IEEE 802.1AS-2011: Specifies the protocol and procedures used to ensure that the synchronization requirements are met for time-sensitive applications such as audio and video across bridged and virtual-bridged LANs consisting of LAN media where the transmission delays are fixed and symmetrical. For example, IEEE 802.3 full-duplex links include the maintenance of synchronized time during normal operation followed by addition, removal, or failure of network components and network reconfiguration.

While the AVB standards were originally designed for audio and video over standard Ethernet, extending these to control streams additionally benefits the industrial and automotive sectors. Time Sensitive Networking (TSN) is a set of standards developed by the Time Sensitive Networking Task Group with the intent to extend the future AVB enhancements to other sectors that could benefit.

Bridges are increasingly used to interconnect devices that support scheduled applications (for example, industrial automation, process control and vehicle control). The IEEE 802.1Qbv-2015 (Enhancements to Scheduling Traffic) provides performance assurances of latency and delivery variation to enable these applications in an engineered LAN while maintaining the existing guarantees for the credit-based shaper and best-effort traffic.

The IEEE 802.3br (Interspersing Traffic) and IEEE 802.1Qbu (Frame Preemption) enables the suspension of a large preemptable packet being transmitted by the MAC layer to allow one or more express packets to be transmitted before the transmission of the preemptable packet is resumed. This provides the capability to schedule express traffic packets with minimal delays/latencies or at predictable times with efficient utilization of the line bandwidth.

## Transmit Path Functions

Transmit paths of queue 0 and the highest enabled queue are enabled by default.

The transmit path of queue 0 supports the strict priority algorithm, and it is used for best-effort traffic. For a queue, the strict priority algorithm determines that a packet is available for transmission if the queue contains one or more packets. When the threshold mode for the MTL Tx FIFO is enabled, the strict priority algorithm determines that a packet is available for transmission if the queue contains a partial packet of a size equal to the programmed threshold limit.

The transmit paths of additional queues support traffic management by using the credit-based shaper algorithm. For a queue, the credit-based shaper algorithm determines that a queue is available for transmission if the following conditions are true:

- The queue contains one or more packets
- The credit for the queue is positive as per the algorithm

The credit-based shaper algorithm can be disabled for all queues or lower-priority queues. For example, the program can either disable the credit-based shaper algorithm for queue 1 and queue 2 or only for queue 1. The program

should not disable the credit-based shaper algorithm for queue 2 and enable it for queue 1. When the credit-based shaper algorithm for a queue is disabled, the channel uses the default strict priority algorithm.

Each transmit DMA has a separate descriptor chain for fetching the transmit data. The transmit channel that gets the access to the system bus depends on the DMA arbiter.

The transmit path has a shared FIFO (MTL layer) for each queue. The data fetched by the DMA is put in the respective part of the FIFO. The MTL transmit queue scheduler controls which part of the FIFO data is transmitted by the MAC. If the credit-based shaper algorithm is enabled for a queue, the corresponding queue is selected for transmission if the following conditions are true:

- If the packet is available in the channel and has a positive or zero credit
- If the higher priority queue has no packet waiting in the FIFO

If the credit-based shaper algorithm is disabled for all queues, the packet transmitted from a queue is selected based on the priority scheme described in the following table.

Table 30-77: Weight for DMA Channels

|   EMAC_DMA[n]_TXCTL.TCW Field |   Transmit Channel Weight |
|-------------------------------|---------------------------|
|                           000 |                         1 |
|                           001 |                         2 |
|                           010 |                         3 |
|                           011 |                         4 |
|                           100 |                         5 |
|                           101 |                         6 |
|                           110 |                         7 |
|                           111 |                         8 |

## Receive Path Functions

The receive path of Queue 0 is enabled by default. All traffic is received on this channel. Programs can enable the receive paths of additional queues. By enabling the receive paths of multiple channels, the received data is demultiplexed and sends the packets into separate receive channels.

To differentiate between the AV and non-AV traffic, the MAC provides a status that indicates if it is an AV packet and its corresponding VLAN priority tag value. This status is updated in the extended status field of the receive descriptor as explained in Receive Descriptor.

All received packets with the EtherType field of 0x22F0 are detected as AV packets. The AV packets can be of the following two types:

- AV data packets: The AV data packets are always tagged. The tagged AV data packets are received based on the programmed priority value. To specify the channel to which an AV packet with a given priority must be sent, program bits[15:8] in the EMAC\_Q[i]\_TXFLOW\_CTL register of the corresponding queue.

- AV control packets: The AV control packets can be either tagged or untagged. The untagged AV control packets are received on Queue 0, by default. To receive these packets on any other queue, program EMAC\_RXQ\_CTL1.AVCPQ bit field. Like the AV data packets, the tagged AV control packets are received based on the programmed priority value.

In addition to the AV packets, the untagged PTP packets on any queue ca also be received. By default, the PTP packets (tagged or untagged) are received on queue 0. To receive these packets on any other queue, program the EMAC\_RXQ\_CTL1.PTPQ bits.

## Credit-Based Shaper Algorithm

The MTL Queue Scheduler uses the credit-based shaper algorithm to arbitrate the AV traffic in all queues and the legacy Ethernet traffic in queue 0. The additional queues can be programmed to use the credit-based shaper algorithm.

The following sections provide information for implementing the credit-based shaper algorithm:

- Credit value
- idleSlopeCredit and sendSlopeCredit values
- Bandwidth status

## Credit Value

The credit value is part of the credit-based sharper algorithm used by the MTL queue scheduler. The credit value is accumulated every transmit clock cycle, that is, 40 ns for 100 Mbps and 8 ns for 1000 Mbps. The credit to add or subtract per cycle can be fractional based on the required idleSlope and sendSlope values, as described in the following table.

Table 30-78: Credit Value per Transmit Cycle Example

| Mode     | Values                                                                                                                                      | Description                                                                                                                                                                                                                                    |
|----------|---------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 100 Mbps | portTransmitRate = 100 Mbps idleSlope = 70 Mbps (assuming 70% bandwidth re- served for a higher priority traffic class) sendSlope = 30 Mbps | credit = 2.8 bits accumulates per cycle (40 ns) for the higher priority traffic class when best-effort packet is being transmitted. credit = 1.2 bits drains per cycle (40 ns) when higher priority traffic class packet is being transmitted. |

The DMA stores the queue traffic in the respective part of the Tx FIFO based on the slot number in the transmit descriptor (if enabled) or the bandwidth availability on the application bus.

The credit for a queue builds up only when the packet is available, but it cannot be transmitted because the MAC is sending a packet from another queue. The EMAC supports another mode in which the credit can build up in advance for a queue in which no packet is available in respective part of the FIFO. This enables sending a burst of high priority traffic in a queue as soon as the data is available. You can enable this mode is enabled using the EMAC\_TQ[n]\_ETS\_CTL.AVALG bit of corresponding queues.

When reset, the accumulated credit parameter in the credit-based shaper algorithm is set to zero if there is positive credit, and there is no packet to transmit in a queue. The credit does not accumulate when there is no packet

waiting in a queue and other queues are transmitting. When set, the accumulated credit parameter in the creditbased shaper algorithm is not reset to zero if there is positive credit and no packet to transmit in a queue. The credit accumulates even when there is no packet waiting in a queue and other queues are transmitting.

## idleSlopeCredit and sendSlopeCredit Values

The software must program the idleSlopeCredit and sendSlopeCredit values. The programmed values should be the credit accumulated or drained per clock cycle scaled by 1024, such as, 2.8 x 1024 = 2867 and 1.2 x 1024 = 1229. In addition, the software must program the hiCredit and loCredit values, scaled by 1024, to adjust for scaling of the idleSlopeCredit and sendSlopeCredit values. This means that if computed hiCredit and loCredit values are 12,000 bits and 3,036 bits respectively, the values programmed in the hiCredit and loCredit registers of the corresponding channel are 12000 x 1024 bits and two's complement of 3036 x1024, respectively.

## Bandwidth Status

The hardware maintains the status of the actual bandwidth consumed by each higher priority queue in the EMAC\_TQ[n]\_ETS\_STAT registers. This enables the software to estimate the average bandwidth consumed by numerically higher traffic classes as compared to the reserved bandwidth.

The EMAC\_TQ[n]\_ETS\_STAT registers show the average number of bits transmitted during the previous programmed slot interval (1, 2, 4, 8, or 16 slots of 125 µs) in a queue. The status register is updated even if the creditbased shaper algorithm is not enabled for a queue. The number of slots over which the average bits transmitted per slot are computed is programmed in the EMAC\_TQ[n]\_ETS\_CTL.SLC bit field of the respective queue. For example, when programming two slots, the average bits are computed over slot numbers 0-1, 2-3, 4-5, and so on.

The value programmed in the idleSlopeCredit register of a queue is proportional to the bandwidth reserved for the queue. The software can allocate any bandwidth that is not used by the higher priority queue to the reserved bandwidth of the lower priority queue.

A lower priority queue that uses the credit-based shaper algorithm, cannot use the unused reserved bandwidth of any higher priority queue that is using the credit-based shaper algorithm. However, a lower priority queue that uses the strict-priority algorithm, can use the unused reserved bandwidth of any higher priority queue that uses the creditbased shaper algorithm. For example, queue 1 and queue 2 use the credit-based shaper algorithm (with reserved bandwidth of 50% and 25%, respectively) and queue 0 uses the strict-priority algorithm. If queue 1 uses only 40% of reserved bandwidth, the remaining 10% is used by queue 0. Queue 2 cannot exceed the reserved bandwidth of 25%.

## Slot Number Function

The slot number function is used to schedule the data fetch from the system memory by the DMA. This feature is useful when the source AV data needs to be transmitted at specific intervals.

The slot number at which the DMA fetches the data from system memory is programmed in the Transmit Descriptor Word 3 (TDES3) register. This 4-bit field allows the application to schedule data up to 16 slots; a programmable slot interval that ranges from 1µs to 4096 µs with a granularity of 1µs. This field is applicable only for the AV channels.

When the DMA fetches a transmit descriptor, it compares the slot number of the transmit descriptor with the internally generated reference slot interval. The slot interval is a counter that is updated based on a programmable slot interval (with a range of from 1µs to 4096 µs) of the IEEE 1588 system time. In addition, the slot interval counter is initialized to zero when the seconds field of the system time is incremented, that is, the sub-second counter rolls over. The DMA fetches the data only if it matches the current slot or the next slot. The DMA remains in the descriptor fetch state until there is a match.

To enable the DMA to fetch the data only if it matches the current slot or the next two slots, program the EMAC\_DMA[n]\_SFN\_CTLSTAT.ASC bit of the corresponding DMA channel.

NOTE: If the slot number in the descriptor is less than the reference slot number, the DMA takes it as a future slot.

The program also enables the check for slot number by setting EMAC\_DMA[n]\_SFN\_CTLSTAT.ASC bit. When this check is not enabled, the packets are fetched immediately after the descriptor is read. In addition, the EMAC\_DMA[n]\_SFN\_CTLSTAT.RSN bits indicate the value of the reference slot number in DMA.

## Queue Modes

Programs can enable a transmit queue for generic DCB and AV traffic. When the program selects multiple transmit queues without selecting the DCB feature, the queues are enabled for generic queuing using the WRR or WSP algorithms.

When the program selects the enable data center bridging feature, DCB is enabled for all selected transmit queues and queuing is based on the WRR, WSP , DWRR, or WFQ algorithms. To enable a transmit queue for AV, select the enable audio video bridging option and then the enable support for AV in transmit Queue # option. The queuing is based on the CBS or SP algorithms.

The program can enable a receive queue for generic, DCB, or AV traffic. A particular queue enabled for generic DCB or AV based routing is determined by the RXQ#EN bit field ( EMAC\_RXQ\_CTL0 register) of the corresponding queue. The following sections explain how receive queues are enabled based on the selected features.

## Multiple Receive Queues without the DCB Feature

When this mode is selected, all queues are enabled for generic queuing based on the VLAN tag priority. The VLAN tag priority should match the PSRQ field of the EMAC\_RXQ\_CTL2 and EMAC\_RXQ\_CTL3 registers. By default, untagged packets are routed to the receive queue specified in the EMAC\_RXQ\_CTL1.UPQ bit field. Queue 0 is the default value of the UPQ field. The program can override the default value with any other value for the UPQ field. The receive packets can also be routed to a particular DMA channel based on the EMAC\_ADDR[i]\_HI.DCS bit field.

## Multiple Receive Queues with DCB Feature

When this mode is selected, DCB is enabled for all selected receive queues. The queuing is done based on the VLAN tag priority for DCB data packets. The VLAN tag priority should match the PSRQ field of the EMAC\_RXQ\_CTL2 and EMAC\_RXQ\_CTL3 registers. The receive packets can also be routed to a particular DMA

channel based on the EMAC\_ADDR[i]\_HI.DCS bit field. The DCB control packets are routed based on the EMAC\_RXQ\_CTL1.DCBCPQ bit field.

## Multiple Receive Queues with AV Feature

When this mode is selected, AV is enabled for all selected receive queues. The queuing is done based on the VLAN tag priority for DCB data packets. The VLAN tag priority should match the PSRQ field of the EMAC\_RXQ\_CTL2 and EMAC\_RXQ\_CTL3 registers. The receive packets can also be routed to a particular DMA channel based on the EMAC\_ADDR[i]\_HI.DCS bit field. The AV control packets (tagged or untagged) are routed based on the EMAC\_RXQ\_CTL1.AVCPQ bit field. The PTP over Ethernet packets are routed based on the EMAC\_RXQ\_CTL1.PTPQ bit field.

## Queue Priorities

The transmit queue priorities are programmed using the EMAC\_TXQ\_PRTY\_MAP0 and EMAC\_TXQ\_PRTY\_MAP1 registers. The receive queue priorities are programmed using the EMAC\_RXFLOW\_CTL register.

The priority of a receive queue is programmed using the corresponding field of the EMAC\_RXQ\_CTL2 and EMAC\_RXQ\_CTL3 registers. Assign the priority in the following order:

1. AV queue (high priority)
2. DCB queue (medium priority)
3. Best-effort queue (low priority)

The software should put packets with correct priorities in the respective programmed queue on transmit side. The MAC uses the programmed priorities for blocking the transmit queues when a PFC packet is received. If a single queue is selected for multiple priorities and PFC is enabled, the entire queue is paused if one or more priorities in the queue are paused.

## TCP/IP Offloading Features

Communication protocols such as TCP and UDP implement checksum fields, which help determine the integrity of data transmitted over a network. The most widespread use of Ethernet is to encapsulate TCP and UDP over IP datagrams. Therefore, the MAC has an Checksum Offload Engine (COE) to support checksum calculation and insertion in the transmit path, and error detection in the receive path.

The TCP Segmentation Offload (TSO) engine is useful in offloading the TCP segmentation functions to the hardware.

This section contains the following sub-sections:

- Transmit Checksum Offload Engine
- Receive Checksum Offload Engine
- TCP/IP Segmentation Offload (TSO) Engine

- UDP/IPv4 Fragment Offload (UFO) Engine
- IPv4 ARP Offload Engine

## Transmit Checksum Offload Engine

The MAC has a Checksum Offload Engine (COE) to support checksum calculation and insertion in the transmit path where the software can offload the checksum insertion task to the hardware. In the transmit path the MAC calculates the checksum and inserts it in the transmit packet. This feature reduces the load on the software and can improve the overall throughput of the system. The checksum engine can be controlled for each packet by setting the CIC bits (TDES3 Bits[17:16]).

NOTE: The checksum for TCP , UDP , or ICMP is calculated over a complete packet, and then inserted into its corresponding header field. Because of this requirement, when this function is enabled, the transmit FIFO automatically operates in the store-and-forward mode even if the EMAC is configured for threshold (cutthrough) mode.

NOTE: The application must ensure that the transmit FIFO is deep enough to store a complete packet before that packet is transferred to the MAC transmitter. The reason being that when space is not available to accept the programmed burst length of data, then the MTL transmit FIFO starts reading to avoid deadlock. In such a case, the COE fails as the start of the packet header is read out before the payload checksum can be calculated and inserted. Therefore, the program must enable the checksum insertion only in the packets that are less than the number of bytes, given by the following equation:

Packet size &lt; TxQSize() - (PBL + 7) x 4,

Where, TxQSize is indicated by the EMAC\_TQ[n]\_OPMODE.TQS bit field and PBL by the EMAC\_DMA[n]\_TXCTL.TXPBL bit field.

## IP Header Checksum Engine

In IPv4 datagrams, the integrity of the header fields is indicated by the 16-bit header checksum field (the eleventh and twelfth bytes of the IPv4 datagram). The COE detects an IPv4 datagram when the type field of IPv4 datagrams. The integrity of the header fields is indicated by the 16-bit header checksum field (the eleventh and twelfth bytes of the IPv4 datagram). The COE detects an IPv4 datagram when the type field of Ethernet packet has the value 0x0800 and the version field of an IP datagram has the value 0x4. The checksum field of the input packet is ignored during calculation and replaced with the calculated value.

NOTE: IPv6 headers do not have a checksum field. Therefore, the COE does not modify the IPv6 header fields.

The result of this IP header checksum calculation is indicated by the IP header error status bit in the transmit status (bit 0 in RDES3 Normal Descriptor (Write-Back Format)). This status bit is set whenever the values of the Ethernet Type field and the Version field of IP header are not consistent, or when the Ethernet packet does not have enough data, as indicated by the IP header Length field. In other words, this bit is set when an IP header error is asserted under the following circumstances:

- For IPv4 datagrams:
- The received Ethernet type is 0x0800, but the version field of IP header is not equal to 0x4.

- The IPv4 header length field indicates a value less than 0x5 (20 bytes).
- The total packet length is less than the value given in the IPv4 header length field.
- For IPv6 datagrams:
- The Ethernet type is 0x86dd but the IP header version field is not equal to 0x6.
- The packet ends before the IPv6 header (40 bytes) or extension header (as given in the corresponding header length field in an extension header) is completely received.

## TCP/UDP/ICMP Checksum Engine

The TCP/UDP/ICMP Checksum Engine processes the IPv4 or IPv6 header (including extension headers) and determines whether the encapsulated payload is TCP , UDP , or ICMP . The checksum is calculated for the TCP , UDP , or ICMP payload and inserted into its corresponding field in the header. The transmit COE can work in the following two modes:

- The TCP , UDP , or ICMPv6 pseudo-header is not included in the checksum calculation and is assumed to be present in the checksum field of the input packet. This engine includes the checksum field in the checksum calculation, and then replaces the checksum field with the final calculated checksum.
- The engine ignores the checksum field, includes the TCP , UDP , or ICMPv6 pseudo-header data into the checksum calculation, and overwrites the checksum field with the final calculated value.

NOTE: For ICMP-over-IPv4 packets, the checksum field in the ICMP packet must always be 16'h0000 in both modes, because pseudo-headers are not defined for such packets. If it does not equal 16'h0000, an incorrect checksum may be inserted into the packet.

The result of this operation is indicated by the payload checksum error status bit in the transmit status vector (bit 12 of TDES3). This engine sets the payload checksum error status bit when it detects that the packet has been forwarded to the MAC transmitter engine in the store-and-forward mode without the end of packet (EOP) being written to the FIFO, or when the packet ends before the number of bytes indicated by the payload length field in the IP Header is received. When the packet is longer than the indicated payload length, the COE ignores them as stuff bytes, and no error is reported. When this engine detects the first type of error, it does not modify the TCP , UDP , or ICMP header. For the second error type, it still inserts the calculated checksum into the corresponding header field.

The following table describes the functions supported by T ransmit Checksum Offload engine based on the packet type. When the MAC does not insert the checksum, it is indicated as 'No' in the table.

Table 30-79: Transmit Checksum Offload Engine Functions for Different Packet Types

| Packet Type                                                                                                                                                                                                        | Hardware IP Header Checksum Insertion   | Hardware TCP/UDP Checksum Insertion   |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------|---------------------------------------|
| Non-IPv4 or IPv6 packet                                                                                                                                                                                            | No                                      | No                                    |
| IPv4packet is greater than 1,522 bytes (2,000 bytes when IEEE 802.3ad support for 2K packets is enabled in MAC) but less than or equal to the frame size constraint specified in Transmit Checksum Offload Engine. | Yes                                     | Yes                                   |

Table 30-79: Transmit Checksum Offload Engine Functions for Different Packet Types (Continued)

| Packet Type                                                                                                                                                                                                                                                                                                                                                        | Hardware IP Header Checksum Insertion                                                                                   | Hardware TCP/UDP Checksum Insertion   |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------|---------------------------------------|
| IPv6packet is greater than 1,522 bytes (2,000 bytes when IEEE 802.3ad support for 2K packets is enabled in MAC) but less than or equal to the frame size constraint specified in Transmit Checksum Offload Engine.                                                                                                                                                 | Not Applicable                                                                                                          | Yes                                   |
| IPv4 with TCP, UDP, or ICMP                                                                                                                                                                                                                                                                                                                                        | Yes                                                                                                                     | Yes                                   |
| IPv4 packet has IP options (IP header is longer than 20 bytes)                                                                                                                                                                                                                                                                                                     | Yes                                                                                                                     | Yes                                   |
| Packet is an IPv4 fragment                                                                                                                                                                                                                                                                                                                                         | Yes                                                                                                                     | No                                    |
| IPv6packet with the following next header fields in main or extension headers: • Hop-by-hop options (in IPv6 main header) • Hop-by-hop options (in IPv6 extension header) • Destinationsoptions • Routing (with segment left = 0) • Routing (with segment left > 0) • TCP, UDP, orICMP • Authentication • Any other next header field in main or extension headers | Not Applicable Not Applicable Not Applicable Not Applicable Not Applicable Not Applicable Not Applicable Not Applicable | Yes No Yes No No Yes No No            |
| IPv4 packet has TCP header with Options fields                                                                                                                                                                                                                                                                                                                     | Yes                                                                                                                     | Yes                                   |
| IPv4 Tunnels: • IPv4 packet in an IPv4 tunnel • IPv6 packet in an IPv4 tunnel                                                                                                                                                                                                                                                                                      | Yes (IPv4 tunnel header) Yes (IPv4 tunnel header)                                                                       | No No                                 |
| IPv6 Tunnels: • IPv4 packet in an IPv6 tunnel • IPv6 packet in an IPv6 tunnel                                                                                                                                                                                                                                                                                      | Not applicable Not applicable                                                                                           | No No                                 |
| IPv4 packet has 802.3ac tag (with C-VLAN tag or S-VLAN tag when enabled).                                                                                                                                                                                                                                                                                          | Yes                                                                                                                     | Yes                                   |
| IPv6 packet has 802.3ac tag (with C-VLAN tag or S-VLAN tag when enabled).                                                                                                                                                                                                                                                                                          | Not applicable                                                                                                          | Yes                                   |
| IPv4 frames with security features (such as encapsulated security pay- load)                                                                                                                                                                                                                                                                                       | Yes                                                                                                                     | No                                    |
| IPv6 frames with security features (such as encapsulated security pay- load)                                                                                                                                                                                                                                                                                       | Not applicable                                                                                                          | No                                    |

## Receive Checksum Offload Engine

The EMAC provides the Checksum Offload Engine that is used to detect any error in an IPv4 or IPv6 packet in the receive path. The MAC verifies the checksum field of the received packet with the internally calculated checksum

and provides the status. This engine also identifies a TCP , UDP , or ICMP payload in received IP packets and calculates the checksum of such payloads appropriately.

Here, both IPv4 and IPv6 packet in the received Ethernet packets are detected and processed for data integrity. The MAC receiver identifies IPv4 or IPv6 packets by checking for value 0x0800 or 0x86DD, respectively, in the type field of the received Ethernet packet. This identification is applicable to single VLAN-tagged packets. It is also applicable to double VLAN-tagged packets when the enable double VLAN processing option is selected and the EMAC\_VLANTAG\_CTL.EDVLP bit is set.

The receive COE calculates the IPv4 header checksums and checks that they match the received IPv4 header checksums. The result of this operation (pass or fail) is given to the RFC module for insertion into the receive status word. The IP Header Error bit is set for any mismatch between the indicated payload type (Ethernet Type field) and the IP header version, or when the received packet does not have enough bytes, as indicated by the Length field of the IPv4 header (or when fewer than 20 bytes are available in an IPv4 or IPv6 header).

Packets with TCP/IP errors (header or payload) are dropped in MTL when the EMAC\_RQ0\_OPMODE.DIS\_TCP\_EF bit is reset and the EMAC\_RQ0\_OPMODE.FEP bit is set.

This engine also identifies a TCP , UDP , or ICMP payload in the received IP datagrams (IPv4 or IPv6) and calculates the checksum of such payloads properly, as defined in the TCP , UDP , or ICMP specifications. This engine includes the TCP , UDP , or ICMPv6 pseudo-header bytes for checksum calculation and checks whether the received checksum field matches the calculated value. The result of this operation is given as a payload checksum error bit in the receive status word. This status bit is also set if the length of the TCP , UDP , or ICMP payload does not match the expected payload length given in the IP header.

The following table describes the functions supported by the receive COE based on the packet type. When the payload of an IP packet is not processed (indicated as "No" in the table), the information (whether the checksum engine is bypassed or not) is given in the receive status.

NOTE: The MAC does not append any payload checksum bytes to the received Ethernet packets.

Table 30-80: Transmit Checksum Offload Engine Functions for Different Packet Types

| Packet Type                                                                                                     | Hardware IP Header Checksum Checking   | Hardware TCP/UDP Checksum Checking   |
|-----------------------------------------------------------------------------------------------------------------|----------------------------------------|--------------------------------------|
| Non-IPv4 or IPv6 packet                                                                                         | No                                     | No                                   |
| IPv4packet is greater than 1,522 bytes (2,000 bytes when IEEE 802.3ad support for 2K packets is enabled in MAC) | Yes                                    | Yes                                  |
| IPv6packet is greater than 1,522 bytes (2,000 bytes when IEEE 802.3ad Support for 2K Packets is enabled in MAC) | Not Applicable                         | Yes                                  |
| IPv4 with TCP, UDP, or ICMP                                                                                     | Yes                                    | Yes                                  |
| IPv4 header's protocol field contains a protocol other than TCP, UDP, or ICMP                                   | Yes                                    | No                                   |
| IPv4 packet has IP options (IP header is longer than 20 bytes)                                                  | Yes                                    | Yes                                  |

Table 30-80: Transmit Checksum Offload Engine Functions for Different Packet Types (Continued)

| Packet Type                                                                                                                                                                                                                                                                                                                                     | Hardware IP Header Checksum Checking                                                                     | Hardware TCP/UDP Checksum Checking   |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------|--------------------------------------|
| Packet is an IPv4 fragment                                                                                                                                                                                                                                                                                                                      | Yes                                                                                                      | No                                   |
| IPv6 packet with the following next header fields in main or extension headers: • Hop-by-hop options (in IPv6 mainheader) • Hop-by-hop options (in IPv6 extensionheader) • Destinations options • Routing (with segment left 0) • Routing (with segment left >0) • TCP, UDP, or ICMP • Any other next header field in main or extension headers | Not Applicable Not Applicable Not Applicable Not Applicable Not Applicable Not Applicable Not Applicable | Yes No Yes Yes No Yes No             |
| IPv4 packet has TCP header with Options fields                                                                                                                                                                                                                                                                                                  | Yes                                                                                                      | Yes                                  |
| IPv4 Tunnels: • IPv4 packet in an IPv4 tunnel • IPv6 packet in an IPv4 tunnel                                                                                                                                                                                                                                                                   | Yes (IPv4 tunnel header) Yes (IPv4 tunnel header)                                                        | No No                                |
| IPv6 Tunnels: • IPv4 packet in an IPv6 tunnel • IPv6 packet in an IPv6 tunnel                                                                                                                                                                                                                                                                   | Not applicable Not applicable                                                                            | No No                                |
| IPv4 packet has 802.3ac tag (with C-VLAN tag or S-VLAN tag when enabled).                                                                                                                                                                                                                                                                       | Yes                                                                                                      | Yes                                  |
| IPv6 packet has 802.3ac tag (with C-VLAN Tag or S-VLAN Tag when enabled).                                                                                                                                                                                                                                                                       | Not applicable                                                                                           | Yes                                  |
| IPv4 frames with security features (such as encapsulated security pay- load)                                                                                                                                                                                                                                                                    | Yes                                                                                                      | No                                   |
| IPv6 frames with security features (such as encapsulated security pay- load)                                                                                                                                                                                                                                                                    | Not applicable                                                                                           | No                                   |

## Header-Payload Split

The MAC can identify the boundary between header and payload of the received packet and store them into separate buffers. This feature is useful for multiple reasons:

- Header buffers can be lin faster memory/cache while payload buffers are in slower DRAM memory. This enables the software stack to process the headers faster.
- Buffers used for payload can be directly forwarded to the application layer without the need to copy the payload into application buffer.

- When the Large Receive Offload (LRO) function is implemented in the software/driver layer, it is easy to link the buffers of multiple and contiguous payload data and form a bigger packet with a new header. This reduces the number of packets forwarded to the upper layer and thus improves system software performance.

The EMAC supports multiple methods and packet types for header-payload splitting. This is controlled by the setting the EMAC\_SPLM\_OFST\_CFG.SPLM bit field and described in the following table.

Table 30-81: Split Header Support Depending on the Packet Type

| Packet Type              | SPLM                                  | Description                                                                                                                                                                      |
|--------------------------|---------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| TCP or UDP packet        | 00 (L3/L4 split)                      | The DMAwrites the Ethernet header + IP header + TCP or UDP head- er into the header buffer.                                                                                      |
| IP packet (not TCP/ UDP) | 00 (L3/L4 split)                      | The DMAwrites the Ethernet header + IP header into the header buffer.                                                                                                            |
| Non-IP packet            | 00 (L3/L4 split)                      | The DMAdoes not split the header and payload                                                                                                                                     |
| Any packet               | 01 (L2 split)                         | The DMAwrites the Ethernet header • For AV type packets using AV type Split Offset (SAVO) when split AV Enable (SAVE) field is set to 1. • For non-AV type packets using SPLOFST |
| IP packet                | 10 (combination of L2 or L3/L4 split) | L3/L4 split                                                                                                                                                                      |
| Non-IP packet            | 10 (combination of L2 or L3/L4 split) | L2 split                                                                                                                                                                         |
| N/A                      | 11                                    | Reserved                                                                                                                                                                         |

- NOTE: · L3/L4 split is applicable for IP packets that are either untagged or VLAN stripped. If VLAN tag is retained in the packet forwarded to the DMA, L3-L4 split is not performed. When SPLM field is set to 2, L2 Split is performed for IP packets which are VLAN tagged.
- For AV packets, program the EMAC\_SPLM\_OFST\_CFG.SAVE and EMAC\_SPLM\_OFST\_CFG.SAVO bit fields to specify an alternative L2 split value.

The IP header includes IPv4 options in case of a IPv4 packet, and IPv6 extension headers in case of IPv6 frames. The points at which the L3/L4 header is split are shown in the following figure.

Figure 30-38: L3/L4 Header Split Points

The Figure 30-39 L2 Header Split figure shows the L2 split boundary. In this mode, the header payload boundary is determined for received packets at a fixed length offset starting from the length/type field, as indicated by the EMAC\_SPLM\_OFST\_CFG.SPLOFST bit field. This mode is applicable for all received packets when EMAC\_SPLM\_OFST\_CFG.SPLM = 01 and only for non-IP packets when SPLM = 10.

The following table shows the split offset for various packet types, when SPLOFST field is set to 2.

Table 30-82: Split Offset Calculation Example: SPLOFST = 2

| Packet Type                                           | Split Offset (From the Beginning of LT Field)   |
|-------------------------------------------------------|-------------------------------------------------|
| Untagged packet                                       | DA + SA + LT = 14                               |
| Single VLAN tagged packet with VTAG stripped          | DA + SA + LT = 14                               |
| Double VLAN tagged packet with both VTAGs stripped    | DA + SA + LT = 14                               |
| Double VLAN tagged packet with both VTAGs stripped    | DA + SA + VTAG + LT = 18                        |
| Double VLAN tagged packet with only one VTAG stripped | DA + SA + VTAG + LT = 18                        |
| Double VLAN tagged packet with no stripping           | DA + SA +2 x VTAG + LT = 22                     |

The split offset reference is always from the beginning of the LT field, whether the VLAN tag is present or stripped. For more details about the descriptor structure for the split header feature, see

Descriptor Structure for Split Header Support

.

<!-- image -->

Figure 30-39: L2 Header Split

For programming details see the Programming Header Payload Split Receive section.

## TCP/IP Segmentation Offload (TSO) Engine

The TCP Segmentation Offload (TSO) engine is useful in offloading the TCP segmentation functions to the hardware.

It also supports UDP Segmentation Offload (USO) in which the UDP payload is segmented in the hardware. There are no sequencing/ordering controls available or updated in the segments, so it can be used in point to point applications in which out of order packets are not expected by the receiver. The description and flow of TSO mentioned in this section is same as USO, any deviation is provided as notes.

In the TCP segmentation offload (TSO) feature, the DMA splits a large TCP packet into multiple small packets and passes these packets to the MTL as shown in the TCP Segmentation Offload Overview figure.

Figure 30-40: TCP Segmentation Offload Overview

<!-- image -->

## DMA Operation with TSO Feature

The TCP Segmentation Offload Flow figure shows the TSO flow.

<!-- image -->

Figure 30-41: TCP Segmentation Offload Flow

For the TSO feature, the transmit DMA operation is as follows:

1. The application sets up the transmit descriptor (TDES0-TDES3) and sets the Own bit (TDES3[31]) after setting up the corresponding data buffer(s) with Ethernet Packet data.
2. The application increases the offset value of the descriptor tail pointer of the DMA transmit channel.
3. While in the Run state, the DMA fetches the next available descriptor and does one of the following:

<!-- image -->

- a. If the descriptor is a context descriptor and the context is not between the first and last descriptors of a packet, the DMA stores the context values.
- b. If the descriptor is a context descriptor and the context is between the first and last descriptors of a packet, the DMA closes the context descriptor indicating a context descriptor error (TDES3[23]) and fetches the next descriptor.
- c. If the descriptor is a normal descriptor, the DMA checks for the TSE bit. If the TSE bit is not set, the DMA continues with the default mode of operation or OSF operation (if enabled).
4. The DMA calculates the number of segments from the TCP payload length (TDES3[17:0]) and the MSS value.
5. The DMA goes through channel arbitration to fetch the data buffers. The DMA fetches the header and payload separately.
6. For the first segment, the DMA fetches the header from the system memory and stores it in the TSO memory (when the length of header is not greater than the TSO memory size). If the current segmented packet is not the first segment, the DMA fetches the header from the local TSO memory. Otherwise, it fetches the header buffer in system memory again, as done for the first segment. In such cases (header not available is TSO memory), the DMA does not close the first descriptor containing the header buffer, until the header for last segment is fetched.
7. The required fields in the header bytes are modified/updated as per the segmentation requirements and written into the corresponding MTL TxQ.
8. The DMA then takes the payload buffer pointer, fetches the MSS number of payload bytes from the system memory and directly pushes it into the MTL TxQ. In case the buffer(s) in the descriptor do not have enough data for the MSS payload (except for the last segment), the DMA closes this descriptor.
9. The DMA jumps to Step 3 and repeats the process until the last segment is written into the TxQ.
10. The DMA closes the last descriptor and the first descriptor (containing the header buffer when it is not stored in TSO memory) and then moves on to the next packet transfer.

The DMA repeats all these steps if more descriptors are available. When no descriptor is available, the DMA enters the suspend state.

- NOTE: · The TSO memory size on ADSP-SC59x processor is 256 bytes per channel.
- The TSO engine determines whether to perform TSO or USO operation based on the THL field (TCP Header Length) in TDES3 of first normal transmit descriptor for the packet. The value of 2 indicates USO and any value greater than or equal to 5 indicates TSO.

The DMA repeats all these steps if more descriptors are available. When no descriptor is available, the DMA enters the suspend state.

## TCP/IP Header Fields

While segmenting a TCP packet, the DMA automatically updates the TCP/IP header fields. The following table describes how the TCP and IP headers are updated.

Table 30-83: TSO: TCP and IP Header Fields

| Packet Sequence    | TCP Header                                                                                                                                                                                                                       | IP Header                                                                                                                                                                                                                                                                                           |
|--------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| First packet       | 1. The sequence number is not updated. The value provided in the header is used. 2. The TCP checksum is calculated again. 3. If set, the FIN and PSH flags are cleared.                                                          | IPv4 Header: • Total Length = MSS + TCP Header Length + IP Header Length • Identification field is not modified. It is sent as per the header provided by the software • IPv4 Header Checksum is recalculated. IPv6 Header: Payload Length = MSS + TCP Header Length + IPExtension Header Length    |
| Subsequent packets | 1. The sequence number is updated. The MSS value is added to the sequence number val- ue of previous segment. 2. If set, the FIN and PSH flags are cleared. 3. The TCP checksum is calculated again.                             | IPv4 Header: • Total Length = MSS + TCP Header Length + IP Header Length • Identification field = Previous Identification Field + 1 • IPv4 Header Checksum is recalculated. IPv6 Header: Payload Length = MSS + TCP Header Length + IPExtension Header Length                                       |
| Last packet        | 1. The sequence number is updated. The MSS value is added to the sequence number val- ue of previous segment. 2. If FIN and PSH flags were set in original header, these flags are set. 3. The TCP checksum is calculated again. | IPv4 Header: • Total Length = Remaining Payload + TCP Head- er Length + IP Header Length • Identification Field = Previous Identification Field + 1 • IPv4 Header Checksum is recalculated. IPv6 Header: Payload Length = Remaining Payload Length + TCP Header Length + IP Extension Header Length |

NOTE: In case of USO, the engine updates only the following fields in the IP-UDP headers:

- Total length, identification field, and header checksum field in IPv4 header
- Payload length field in IPv6 header
- Length and checksum field in UDP header

## Header and Payload Fields of Segmented Packets

After segmentation, the split packets use the same header as the parent TCP packet for header fields other than the ones described in the Table 30-83 TSO: TCP and IP Header Fields table. The Header and Payload Fields of Segmented Packets figure shows how the same header is used for the header fields of segmented packets.

The application must create the header in buffer 1 of the first descriptor of the packet to be segmented and provide the header length in TDES2 of the first descriptor (FD = 1). When the FD bit is set, the DMA reads the header from the header buffer to which the TDES0 is pointing. Buffer 2 of the first descriptor can be used for payload and TDES0 and TDES1 of subsequent descriptors. For subsequent descriptors (FD = 0), the address to which the TDES0 and TDES1 are pointing is treated as payload buffer address of the same packet.

Figure 30-42: Header and Payload Fields of Segmented Packets

<!-- image -->

## Context Descriptor Sequence

Programs can use the context descriptor to provide the MSS value for segmentation. The application must provide the context descriptor before the normal descriptor to be used for the corresponding TCP packet. If the application wants to provide a new MSS, it must create the context descriptor in the descriptor list before the first normal descriptor of the packet to be segmented with the new MSS value. The MSS value in the context descriptor is valid only if the TCMSSV bit of TDES3 in context descriptor is set and the OSTC bit is reset.

When the application provides a context descriptor with a valid MSS value, the DMA internally stores the MSS value and uses this MSS value for all subsequent packets that have the TSO Enabled through the TSE bit of TDES3 normal descriptor.

If the application places a context descriptor in the middle of a packet (between the first and last descriptors of a packet), the DMA does the following:

1. The DMA ignores the context and closes the descriptor
2. The DMA indicates the error in descriptor status
3. The DMA generates an interrupt if the EMAC\_DMA[n]\_IEN.CDEE bit is set corresponding to a DMA channel

The application can read the interrupt status through the CDE bit of the status register corresponding to a DMA channel.

## Building the Descriptor and the Packet for the TSO Feature

Set the TSE bit of TDES3 of the first normal descriptor. If the TSE bit is set in TDES3 for a non TCP/IP packet, the DMA behavior is unpredictable.

The application must program the length of the TCP packet payload in TDES3[17:0] and the TCP header in TDES3[22:19]. The maximum length of TCP packet payload that can be segmented is 256 KB.

- NOTE: The TDES3[22:19] field is used as slot number in case of AV configurations. If both TSO and AV are enabled for the same DMA transmit channel (the EMAC\_DMA[n]\_TXCTL.TSE bit is set), then slot checking for that DMA transmit channel must not be enabled (do not set the EMAC\_DMA[n]\_SFN\_CTLSTAT.ESC bit).

The header of the packet including the Ethernet header, L3 header and L4 header should be provided in buffer 1 of the first normal descriptor of the TSO packet. Only buffer 1 of the first normal descriptor of a packet enabled for TSO is taken as the buffer containing the header.

The TCP payload can begin from buffer 2 of the first normal descriptor and continue to buffer 1 and buffer 2 of second normal descriptor and subsequent descriptors.

The TCP payload may span across multiple buffers and multiple descriptors. The size of buffers containing the TCP payload should add up to and be equal to the TCP payload length provided in TDES3[17:0] of the first normal descriptor.

The MAC always calculates and appends CRC and inserts padding (if required) for all packets segmented by the DMA. If the TSE bit of TDES3 is enabled, the CRC PAD control (CPC) field of TDES3 is reserved. To determine the size of a TCP packet after segmentation, the DMA uses the maximum segment size (MSS) provided by the application through context descriptor. The DMA segments only those packets which have payload size greater than MSS. The application must provide the MSS by either programming the EMAC\_DMA[n]\_CTL.MSS value or by providing a context descriptor. The DMA uses the last programmed value of MSS or the last MSS value provided through context (whichever is provided later).

The header length plus the MSS size (which is equal to the size of each TCP segment) should not exceed 16383 bytes, otherwise the MAC transmitter truncates the packet after 16383 bytes causing a CRC error.

The header length plus MSS size plus programmed PBL value in the EMAC\_DMA[n]\_TXCTL register should be lesser than the Tx Queue size programmed in the EMAC\_TQ0\_OPMODE.TQS bit field. It is recommended that MSS + header is equal to half the programmed transmit queue size.

The DMA also supports segmentation of VLAN tagged TCP/IP frames, so if the TCP packet has a VLAN tag, then the same tag is used for all the segments irrespective of the VLAN tag type provided (C-VLAN or S-VLAN). The VLAN tag insert/replace control bits are used for all segments.

If the double VLAN feature is selected, then the DMA passes both the tags for all segments irrespective of the VLAN tag types provided (C-VLAN or S-VLAN). The VLAN tag insert/replace control bits for both tags is applicable for all segments.

If the double VLAN feature is not selected, then the application must not set the TSE bit in TDES3 for a TCP/IP packet with two tags. The DMA behavior in this scenario is unpredictable.

If the TSE bit is set in TDES3 for the packet and TCP header length provided is less than 5 (meaning an invalid TCP header because it is less than 20 bytes), the DMA does not perform segmentation, instead it transmits the entire packet as a single packet. In this scenario, the CRC pad control bits are forced by DMA to 2'b00 (MAC does CRC and padding) and checksum insertion control bits are forced to 2'b11 (hardware does the checksum calculation for both header and payload).

See the Programming TSO section for more details on how to program the TSO engine.

## UDP/IPv4 Fragment Offload (UFO) Engine

The EMAC supports breaking a large UDP packet into smaller and multiple packets for transmission, using IPv4 Fragmentation. Each IPv4 fragment contains the following header or payload information:

- Ethernet Header
- IP Header
- UDP Header (only in the first fragment)
- UDP Payload
- NOTE: · Fragmentation is supported only for UDP over IP . Fragmentation is not supported for TCP packets.
- Fragmentation of UDP is supported only over IPv4. This support is not available over IPv6.

The UDP Fragment Offload (UFO) Engine is an add-on function of the TSO engine. Therefore, both UFO and TSO functions can be enabled on the same transmit DMA channel. In this mode, the UDP packets are fragmented and TCP packets are segmented. If the UFO function is not enabled, then both TCP packets and UDP packets are segmented into smaller packets as described in TCP/IP Segmentation Offload (TSO) Engine.

The UFO breaks down a large UDP packet into smaller packets using IPv4 fragmentation. In this mode, the MSS field in the context Descriptor (TDES2[17:0]) and the EMAC\_DMA[n]\_CTL.MSS bit are interpreted as Maximum Fragmentation Size (MFS). This is the maximum size of the payload after the IPv4 header in each fragment. When UFO is enabled, the MSS field must be programmed in multiples of 8 bytes because the IP fragments offset is defined in terms of 8 bytes.

The UFO functions in two modes:

- UFO with valid UDP checksum
- UFO with no UDP checksum

## UFO with Valid UDP Checksum

In this mode, the UFO engine calculates the UDP payload checksum and inserts it in the respective UDP header field. However, as the checksum must be calculated over the complete payload ending in the last fragment and then inserted in the UDP header encapsulated in the first IPv4 fragment, the fragments are not transmitted in order. The first IPv4 fragment containing the UDP header is sent last. The Fragmentation Flow With Checksum figure shows this sequence.

Figure 30-43: Fragmentation Flow With Checksum

<!-- image -->

The following table shows the details of the IP/UDP Header updates performed in each fragment by the UFO Engine in this mode.

Table 30-84: Details of IPv4 Fragmentation with UDP Checksum

| Packet Sequence                              | UDP Header                          | IPv4 Header                                                                                                                                                                    | Comments                                                                                   |
|----------------------------------------------|-------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------|
| First fragment (This seg- ment is sent last) | Calculate the UDP head- er checksum | 1. Total length = 32 bytes + IPv4 header length 2. DF = 0 (do not fragment) 3. MF = 1 4. Fragment offset = 0 5. Update the IPv4 header check- sum                              | The header (and 32 bytes IPv4 fragment payload) data is stored in TSO memory in this step. |
| Second fragment                              | NA (No UDP header)                  | 1. Total length = MFS + IPv4 header length 2. DF = 0 (do not fragment) 3. MF = 1 4. Fragment offset = previous frag- ment offset + (32/8) 5. Update the IPv4 header check- sum | The header data is read from the TSO memory. Only the Ethernet + IPv4 header is read.      |

Table 30-84: Details of IPv4 Fragmentation with UDP Checksum (Continued)

| Packet Sequence                                | UDP Header         | IPv4 Header                                                                                                                                                                                     | Comments                                                                              |
|------------------------------------------------|--------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------|
| Intermediate (non-first and non-last) fragment | NA (No UDP header) | 1. Total length = MFS + IPv4 header length 2. DF = 0 (do not fragment) 3. MF = 1 4. Fragment offset = previous frag- ment offset + (MFS/8) 5. Update the IPv4 header check- sum                 | The header data is read from the TSO memory. Only the Ethernet + IPv4 header is read. |
| Last fragment                                  | NA (No UDP header) | 1. Total length = remaining pay- load + IPv4 header Length 2. DF = 0 (do not Fragment) 3. MF = 0 4. Fragment offset = previous frag- ment offset + (MFS/8) 5. Update the IPv4 header check- sum | The header data is read from the TSO memory. Only the Ethernet + IPv4 header is read. |

## UFO with no UDP Checksum

In this mode, the UDP payload checksum is not calculated and an all-zero value is inserted in the corresponding checksum field of the UDP header and all the IP fragment packets are transmitted in sequence. the Fragmentation Flow Without UDP Checksum figure shows the sequence of fragmented packets created and transmitted by the UFO engine.

Figure 30-44: Fragmentation Flow Without UDP Checksum

<!-- image -->

UDP Packet

The following table shows the details of IPv4/UDP header updates performed in each fragment of the UFO engine, in this mode.

Table 30-85: Details of IPv4 Fragmentation without UDP Checksum

| Fragment Sequence                              | UDP Header                                   | IPv4 Header                                                                                                                                                                                     | Comments                                                                              |
|------------------------------------------------|----------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------|
| First fragment                                 | Replace the UDP check- sum field with all 0s | 1. Total length = MFS (MSS field) + IPv4 header length 2. DF = 0 (do not fragment) 3. MF = 1 4. Fragment offset = 0 5. Update the IPv4 header checksum                                          | The header data is stored in the TSO memory in this step.                             |
| Intermediate (non-first and non-last) fragment | NA (No UDP header)                           | 1. Total length = MFS (MSS field) + IPv4 header length 2. DF = 0 (do not fragment) 3. MF = 1 4. Fragment offset = previous fragment offset + (MSS/8) 5. Update the IPv4 header checksum         | The header data is read from the TSO memory. Only the Ethernet + IPv4 header is read. |
| Last fragment                                  | NA (No UDP header)                           | 1. Total length = remaining payload + IPv4 header length 2. DF = 0 (do not fragment) 3. MF = 0 4. New fragment offset = previous frag- ment offset + (MSS/8) 5. Update the IPv4 header checksum | The header data is read from the TSO memory. Only the Ethernet + IPv4 header is read. |

## Segmentation Versus Fragmentation

The following table shows the differences between UDP packet fragmentation performed by the UFO and UDP segmentation performed by the TSO engine.

Table 30-86: Segmentation Versus Fragmentation

| Segmentation                                                                                                  | Fragmentation                                                                        |
|---------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------|
| The UDP header is replicated for each segment. The length field in the UDP header is updated for each segment | The UDP header is not updated; however, checksum field in the UDP header is updated. |
| MSS field indicates the size of the maximum segment after the L4(TCP/UDP) header                              | MFS field indicates the size of the fragment after the L3 (IPv4) header              |
| The TCP packet is broken down into multiple chunks by keeping L2+L3+L4 header                                 | The UDP packet is broken down into multiple chunks by keeping the L2 + L3 header     |

## IPv4 ARP Offload Engine

The EMAC supports the Address Recognition Protocol (ARP) offload for IPv4 packets. This feature allows the processing of the IPv4 ARP request packet in the receive path and generating corresponding ARP response packet in the

transmit path. EMAC generates the ARP reply packets for appropriate ARP request packets. The ARP packet for IPv4 is a L2 layer packet with length/type of 0x0806.

The ARP offloading process is as follows:

1. The MAC receiver gets an ARP request if the target protocol address of request matches the IPv4 address programmed in the EMAC\_ARP\_ADDR register of the MAC.
2. The MAC generates an ARP reply packet.
3. The MAC copies the sender hardware address field in the ARP request to the following fields:
- DA field of the Ethernet packet header
- Target hardware address field of the ARP reply packet
4. The MAC copies the Sender Protocol Address field in the ARP request to the target protocol address field in the ARP reply packet.
5. The MAC places its MAC address in the following fields:
- SA field of the Ethernet packet header
- Sender Hardware Address field of the ARP reply packet
6. The MAC copies the target protocol address field in the ARP request to the sender protocol address field in the ARP reply packet.
7. The MAC sets the opcode field in ARP reply packet to 2 indicating ARP reply.
8. The MAC recalculates the CRC and performs padding for generated ARP reply packet.
9. The MAC transmitter sends the ARP reply.

The MAC processes only one ARP request at a time. It does not store the fields of multiple ARP requests. If the MAC receives an ARP request when it is already processing an earlier ARP request, the MAC does not generate the ARP reply for new ARP request. The MAC forwards the new ARP request packet to application with ARP reply not generated (bit 34) status bit set. However, in power-down mode, if the MAC receives an ARP request when it is already processing an earlier ARP request, the MAC drops the new ARP request.

If the disable CRC check bit of the MAC extension configuration bit is set, then the MAC does not check for valid CRC of a ARP request packet. It can generate an ARP response packet if the other conditions are valid.

- NOTE: When the received ARP request is less than 64 bytes packet length, EMAC does not send a ARP response. It is treated as normal packet and forwarded to the application based on the EMAC filter settings.

## Energy Efficient Ethernet (EEE) Mode

EEE is an operational mode that enables the IEEE 802.3 Media Access Control (MAC) sub layer along with a family of physical layers to operate in the Low-Power Idle (LPI) mode. The EEE operational mode supports the IEEE 802.3 MAC operation at 100 Mbps, 1000 Mbps, and 10 Gbps. The EMAC supports the IEEE 802.3az-2010 for EEE.

The LPI mode allows power saving by switching off the parts of the communication device functionality when there is no data to be transmitted and received. The systems on both sides of the link can disable some functions to save power during the periods of low-link use. The MAC controls whether the system should enter or exit the LPI mode and communicates this to the PHY.

The EEE specifies the capabilities negotiation methods that the link partners can use to determine whether EEE is supported, and then select the set of parameters that are common to both devices.

## Transmit Path Functions

The transmit path functions include tasks that the MAC must perform to make the PHY to enter the LPI state.

In the transmit path, the software must set the EMAC\_LPI\_CTLSTAT.LPIEN bit to indicate to the MAC to stop transmission and initiate the LPI protocol. The MAC completes the transmission in progress, generates its transmission status, and starts transmitting the LPI pattern instead of the IDLE pattern if the link status has been up continuously for a period specified in the EMAC\_LPI\_TMRSCTL.LST bit field. The PHY link status ( EMAC\_LPI\_CTLSTAT.PLS bit) indicates the link status of the PHY.

To make the PHY enter the LPI state, the MAC performs the following tasks:

1. Deasserts TXEN
2. Asserts TX\_ER
3. Sets EMAC\_TXD[n] [3:0] to 0x1 (for 100 Mbps) or TXD[7:0] to 0x01 (for 1,000 Mbps)
4. Updates the status ( EMAC\_LPI\_CTLSTAT.TLPIEN bit) and generates an interrupt.

To bring the PHY out of the LPI state, that is, when the software resets the TLPIEN bit, the MAC performs the following tasks:

1. Stops transmitting the LPI pattern and starts transmitting the IDLE pattern
2. Starts the LPI TW TIMER. The MAC cannot start the transmission until the wake-up time specified for the PHY expires. The auto-negotiated wake-up interval is programmed in the EMAC\_LPI\_TMRSCTL.TWT bit field.
3. Updates the LPI exit status ( EMAC\_LPI\_CTLSTAT.TLPIEX bit) and generates an interrupt

The LPI Transitions (Transmit) figure shows the behavior of the TXEN, TX\_ER, and EMAC\_TXD[n] [3:0] signals during the LPI mode transitions.

- NOTE: · Gating of EMAC\_TXCLK is not supported on EMAC.
- If the MAC is in the LPI mode and the application issues a soft or hard reset, the MAC transmitter comes out of the LPI mode.

Figure 30-45: LPI Transitions (Transmit)

<!-- image -->

## Automated Entry/Exit of LPI Mode

The MAC transmitter can be programmed to enter and exit LPI IDLE mode automatically based on whether it is IDLE for a specific period or has a packet to transfer. These modes are enabled and controlled by the EMAC\_LPI\_CTLSTAT register.

When the EMAC\_LPI\_CTLSTAT.LPITXA and EMAC\_LPI\_CTLSTAT.LPIEN are set, the MAC transmitter enters LPI IDLE state when the MAC transmit path (including the MTL layers and DMA layers) are idle. The MAC transmitter exits the LPI IDLE state and clear the LPIEN bit as soon as any of functions in the transmit path (DMA, MTL or MAC) becomes non-idle due to initiation of a packet transfer.

In addition, when ( EMAC\_LPI\_CTLSTAT.LPIATE ) is also set, the MAC transmitter enters LPI IDLE state only when the Transmit path remains in idle state (no activity) for the period indicated by the value in the EMAC\_LPI\_ENTR\_TMR register. In this mode also, the MAC transmitter exits the LPI IDLE state as soon as any of the functions becomes non-idle. However, the LPIEN bit is not cleared but remains active so that reentry to LPI IDLE state is possible without any software intervention when the MAC becomes idle again.

When both the EMAC\_LPI\_CTLSTAT.LPIATE and EMAC\_LPI\_CTLSTAT.LPITXA bits are cleared, the program can directly control the entry and exit of LPI IDLE state by programming the LPIEN bit.

## Receive Path Functions

The receive path functions include the tasks that the PHY and MAC must perform when the PHY receives signals from the link partner to exit the LPI state.

In the receive path, when the PHY receives the signals from the link partner to enter into the LPI state, the PHY and MAC perform the following tasks:

1. The PHY asserts EMAC\_RXERR
2. The PHY sets EMAC\_RXD[n] [7:0] to 0x01
3. The PHY deasserts RX\_DV

4. The MAC updates the EMAC\_LPI\_CTLSTAT.RLPIEN bit and immediately generates an interrupt
2. NOTE: · The PHY maintains the same state of the EMAC\_RXERR , EMAC\_RXD[n] , and RX\_DV signals for the entire duration during which it remains in the LPI state.
- If the LPI pattern is detected for a very short duration (that is, less than two cycles of receive clock), the MAC does not enter the receive LPI mode.
- If the duration between end of the current receive LPI pattern and start of the next receive LPI pattern, is very short (that is, less than two cycles of receive clock), then the MAC exits and again enters the receive LPI mode. The MAC does not give the receive LPI exit and entry interrupts

When the PHY receives signals from the link partner to exit the LPI state, the PHY and MAC perform the following tasks:

1. The PHY deasserts EMAC\_RXERR and returns to a normal inter-packet state.
2. The MAC updates the EMAC\_LPI\_CTLSTAT.RLPIEX bit and generates an interrupt immediately.

The following figure shows the behavior of the EMAC\_RXERR , RX\_DV, and EMAC\_RXD[n] [3:0] signals during the LPI mode transitions.

Figure 30-46: LPI Transitions (Receive)

<!-- image -->

- NOTE: · If the RX\_CLK\_stoppable bit (in the PHY register written through MDIO) is asserted when the PHY is indicating LPI to the MAC, the PHY may halt the RX\_CLK at any time more than nine clock cycles after the start of the LPI state as shown in the LPI Transitions (Receive) figure.
- If the MAC is in the LPI mode and the application issues a soft or hard reset, the MAC receiver comes out of the LPI mode during reset. If the LPI pattern is still received after the reset is de-asserted, the MAC receiver again enters the LPI state.
- If the receive clock is stopped in the RX LPI mode, the application should not write to the CSR registers that are being synchronized to the receive clock domain.
- When the PHY sends the LPI pattern and if the EMAC\_MAC\_CFG.RE bit is set, the MAC automatically enters the LPI state. There is no other software control to prevent the MAC from entering the LPI state.

## LPI Timers

The transmitter maintains the LPI LS TIMER, LPI TW TIMER, and LPI AUTO ENTRY TIMER timers. The following sections describe the LPI timers that are loaded with the respective values from the EMAC\_LPI\_TMRSCTL and EMAC\_LPI\_ENTR\_TMR registers:

## LPI LS Timer

The LPI LS TIMER counts, in milliseconds, the time expired since the link status is up. Programs can enable the monitoring of the EMAC\_PHYIF\_CTLSTAT.LUD bit by setting the EMAC\_LPI\_CTLSTAT.PLSEN bit.

The link status is indicated to the MAC by the EMAC\_PHYIF\_CTLSTAT.LNKSTS bit or the value programmed by the software in the EMAC\_LPI\_CTLSTAT.PLS bit. If the link status is not available in the EMAC\_PHYIF\_CTLSTAT , the software can get the PHY link status by reading the PHY register and accordingly update the PLS bit.

This timer is cleared every time the link goes down. It starts to increment when the link is up again and continues to increment until the value of the timer becomes equal to the terminal count. Once the terminal count is reached, the timer remains at the same value as long as the link is up. The terminal count is the value programmed in the EMAC\_LPI\_TMRSCTL.LST bits. The RGMII interface does not assert the LPI pattern unless the terminal count is reached. This ensures a minimum time for which no LPI pattern is asserted after a link is established with the remote station. This period is defined as 1 second in the IEEE 802.3-az-2010. The LPI LS TIMER is 10-bit wide. Therefore, the software can program up to 1023 milliseconds.

## LPI TW Timer

The LPI TW TIMER counts, in microseconds, the time expired since the de-assertion of LPI. The terminal count is programmed using the EMAC\_LPI\_TMRSCTL.TWT bit field. The terminal count of the timer is the value of resolved transmit TW that is the auto-negotiated time after which the MAC can resume the normal transmit operation. After exiting the LPI mode, the MAC resumes its normal operation after the TW timer reaches the terminal count. The MAC supports the LPI TW TIMER in units of microseconds. The LPI TW TIMER is 16 bits wide. Therefore, the software can program up to 65535 µs.

## LPI Auto Entry Timer

This timer counts in steps of eight microseconds, the time for which the MAC transmit path must remain in idle state (no activity), before the MAC transmitter enters the LPI IDLE state and starts transmitting the LPI pattern. This timer is enabled when the EMAC\_LPI\_CTLSTAT.LPIATE bit is set.

NOTE: Program the EMAC\_LPI\_CTLSTAT.PLS bit to 1'b0 before switching between the GMII and MII modes. This resets the internal timers. If the mode is changed after the LPI LS TIMER or LPI TW TIMER starts, the change in the transmit clock frequency can result in incorrect timeout

## Low Power Interrupt (LPI)

The MAC generates the LPI interrupt when the transmit or receive side enters or exits the LPI state. The LPI interrupt can be cleared by reading the EMAC\_LPI\_CTLSTAT register.

For detailed programming guidelines see Programming Energy Efficient Ethernet.

## Time-Sensitive Networking

The Time-Sensitive network (TSN) is a set of standards developed by the IEEE 802.1 work group. The standards define mechanisms for the time-sensitive transmission of data over the Ethernet.

This section contains the following topics:

- Enhancements to Scheduled Traffic (EST)
- Frame Preemption (FPE)
- Time-Based Scheduling

## Enhancements to Scheduled Traffic (EST)

The IEEE 802.1Qbv-2015 specification defines the schedule for each of the queues on every egress port which makes the implementation aware of traffic arrival schedule. This information can be used to block the lower priority traffic from transmission in this time window/slot. This ensures that scheduled traffic is forwarded from sender to receiver through all the network nodes with a deterministic delay. The Time Aware Shaper Implementation figure shows the scheduled traffic windows for periodic priority data.

Figure 30-47: Time Aware Shaper Implementation

<!-- image -->

To achieve a low latency, ensure there are no interfering frames during the scheduled windows that are reserved for high priority traffic. The use of scheduled traffic imposes limitations while starting a transmission.

Refer to the Implementing a Guard Band figure. IWhenan interfering frame begins transmission just before the start of a reserved period, transmission can be extended into the reserved window, and potentially interfere with higher priority traffic. Therefore, a guard band whose size is equal to the largest possible interfering frame is required before the window starts.

Figure 30-48: Implementing a Guard Band

<!-- image -->

A larger guard band equates to a less efficient use of the network bandwidth. However, the implementation of IEEE802.1Qbu (frame preemption) addresses this inefficiency. Frame preemption breaks the interfering frame into smaller fragments. Therefore, the guard band must only be as large as the largest possible interfering fragment (instead of the largest possible interfering frame).

During the guard band, only the frames that can complete the transmission of the entire frame before the next gate close event are permitted. This ensures that the high priority traffic can always start at the beginning of the specified window .

## Definitions

The following terms are frequently used in the EST support:

- Gate Control List - The list in the hardware memory that holds the gate controls and the associated time intervals.
- Gate Controls - For a given schedule (row in gate control list), there is a gate open (O) and a gate close (C) state associated with each traffic control (TC). Gate controls are the set of O or C values, whose width is same as the configured TCs. For example, CCOCOOCO means TC7 = C, TC6 = C, TC5 = O and so forth.
- Time Interval - Time interval (in nano seconds). It is a 16-, 20-, or 24-bit configured field in the gate control list that indicates the time for which the associated gate controls are valid.
- Base Time Register - Each gate control list is associated with a 64-bit base time register that holds the start time (in PTP format) for the list.

## Transmit Scheduling Updates

To support EST, the following updates are required for transmit scheduling:

- Implementation of the Gate Control List (GCL)
- Enforcing gate controls in the scheduler
- Accounting for gate open duty cycle in the computation of idleSlope (CBS)

## Gate Control List (GCL) Implementation

The Gate Close and Open Events figure shows a block diagram based on the IEEE 802.1Qbv specification. It illustrates how the gate control list governs the gate close (C) and open (O) events based on the schedule provided for each event. GCL includes a time interval and gate control.

Time Interval - Defines the time in nano seconds for which the gate controls are valid. It should be applied before reading the next gate control from the list. It has a configurable width of 16, 20 or 24 to represent a maximum of 64 us, 1 ms, or 16 ms schedule intervals, respectively. The GLC implementation supports a left shift of a time interval up to 7 bits to apply a multiplication factor from 1 to 128 ns (in steps of 2 n ). The maximum value (post shifting) of this field must be 999,999,999 ns.

Gate Control - Defines the open event (represented by a logic 1) or the close event (represented by logic 0) state for the gate of each TC.

Figure 30-49: Gate Close and Open Events

<!-- image -->

The implementation of GCL consists of the following gate control lists:

- Hardware Owned List (HWOL) - a list for hardware access
- Software Owned List (SWOL) - a list for software access

The access to these lists is mutually exclusive. Hardware configures the ownership to the list in the EMAC\_EST\_STAT register. The External Memory Used for Holding the GCL table provides the implementation details of GCL.

Table 30-87: External Memory Used for Holding the GCL

| Gate Control (up to 8 Bits)   | Time Interval (ns) 16, 20 or 24 Bits   |             |
|-------------------------------|----------------------------------------|-------------|
| OOCCCCCC                      | T0                                     | HWOLor SWOL |
| OOOOCCCC                      | T1                                     | HWOLor SWOL |
| CCCCOOCC                      | T2                                     | HWOLor SWOL |
| CCCCCCOO                      | T3                                     | HWOLor SWOL |
| …                             | …                                      | HWOLor SWOL |
| …                             | …                                      | HWOLor SWOL |
| OCOCOCOO                      |                                        | HWOLor SWOL |
| OOOOCCCC                      | T last                                 | HWOLor SWOL |
| OOOOCCCC                      | T0                                     | SWOL orHWOL |
| OOOOCCCC                      | T1                                     | SWOL orHWOL |
| OOCCCCCC                      | T2                                     | SWOL orHWOL |
| OOOOCCCC                      | T3                                     | SWOL orHWOL |
| …                             | …                                      | SWOL orHWOL |
| …                             | …                                      | SWOL orHWOL |
| OCOCCCCC                      |                                        | SWOL orHWOL |
| OOCCCCCC                      | T last                                 | SWOL orHWOL |

GCL uses the following registers (one for each GCL). These registers are implemented through indirect addressing using the EMAC\_EST\_GCL\_CTL and EMAC\_EST\_GCL\_DAT registers.

Base Time Register (BTR) - A 64-bit register that holds the start time to execute the GCL. The format of the BTR is the same as the PTP format (the upper 32-bits holds time in seconds and the lower 32 bits hold time in nano seconds). Once the execution of a given list begins, the implementation can update the value in the BTR to indicate the next list execution time.

Cycle Time Register (CTR) - A 40-bit register that holds the time at which the execution of the GCL should be repeated. The cycle time register consists of an 8-bit value in seconds, and a 32-bit value in nano seconds (similar to the PTP time format with a truncated seconds register).

For a given gate control list:

Start time = Base Time + N * Cycle Time where N is an integer value representing the iteration number starting with 0 for first iteration

If the gate control list execution takes longer than the cycle time, the list is truncated at the cycle time and the subsequent loop begins at cycle time.

Time Extension Register (TER) - An m-bit (where m = configured time interval width + 7) register that holds the amount of time (in nano seconds) that the current gate control list can be extended before switching to the new gate control list. This is useful to avoid small fragments of the current list before switching to a new list.

List Length Register (LLR) - An n-bit gate control register (when n is 7, 8, 9, 10, or 11 for a GCL configured depth of 64, 128, 256, 512, or 1024, respectively) that holds the integer value of the length of the GCL (the number of valid rows in each GCL). The processing of the GCL stops after the number of rows read equals the LLR value.

## Transmission Gating Implementation

A bridge or an end station can be enhanced to allow transmission from each TC that is yet to be scheduled relative to a known timescale. To achieve this, a transmission gate is associated with each TC; the state of the transmission gate determines whether the queued frames can be selected for transmission.

For a TC, the transmission gate can be in one of the following states:

1. Open-queued frames are selected for transmission in accordance with the definition of the transmission selection algorithm associated with the TC
2. Closed-queued frames are not selected for transmission

A frame on a traffic class queue is not available for transmission when the transmission gate is in the closed state or when there is not enough time to transmit the entire frame before the next gate-close event associated with the queue.

The implementation has visibility into the current schedule of gate controls and the immediate next schedule of the gate controls. So, the maximum gate open period does not exceed the sum of the two time intervals. A frame is selected for transmission only when the gate is currently open and the duration of gate open interval is greater than the time taken to transfer the entire frame.

The implementation must have the frame size before the transmission to avoid transmission overruns and schedule only frames that can complete. To implement transmission gating:

- Program the EMAC\_EST\_EXT\_CTL.OVHD bit field with a fixed overhead of the IPG or EIPG, preamble, and scheduler delay
- Enable the automatic prediction of variable overhead due to the offloads for a packet in the MAC transmitter. Consider the following while programming the EST-related fields for correct scheduling of packets:
- Speeds: 1Gpbs/100 Mbps
- PTOV in ns: 6 PTP clock cycles
- CTOV in ns: 3 PTP + 3 transmit clock cycles
- Per packet slot interval in ns: (packet size + overhead + IPG or EIPG + preamble) * (time to transmit 1 byte)
- Overhead, scheduler delay in bytes is calculated as:

17 Tx + (6 + X + Y + Z) CLKO7 clock cycles

where

- If Tx COE or TSO is selected, X = 4 If TBS or OST is selected, Y = 4 If VLAN tag insertion is enabled, Z = 2

The implementation compensates for the data transfer delays from the buffer to the line by offsetting the current time with all the relevant delays (provided by the EMAC\_EST\_CTL.CTOV field). This configuration ensures that the schedule provided is always accurately implemented at the line.

- NOTE: · An overhead is added to the packet size to account for the scheduler delay, IPG or EIPG, and preamble overheads on the line. Therefore, for a 128 byte frame to be transmitted, the gate open window must be able to accommodate at least 128 overhead bytes.
- In 100 Mbps mode of operation, the rounding down error is about 0.4%. For example, the gate open duration must be at least 1035 bytes for transmitting 1000 byte frames at a speed of 100 Mbps (31 bytes for scheduler delay, line overheads for two-port SRAM configurations, and 4 bytes for rounding down error)
- The minimum time interval for GCL entry must be equal to the time required to transmit fragment size + scheduler delay + IPG/EIPG + preamble overheads

## Idle Slope Computation

When EST is enabled, credit is accumulated only when the gate is open; therefore, the effective data rate of the idle slope must be increased to reflect the duty cycle for the transmission gate associated with the queue.

The idle slope is computed based on GateOpenTime and OperCycleTime values. Program the idleSlope registers (implemented one per CBS-enabled TC) based on the following equation. The existing MTL register has sufficient bit width to accommodate the new values for idleSlope.

idleSlope = (operIdleSlope(N) * OperCycle/GateOpenTime)

## Operational Details of GCL

Set (=1) the switch to the software list using the EMAC\_EST\_CTL.SSWL bit. Hardware can subsequently access the programmed gate control list. The first configuration of gate controls are applied when the current time is equal to the value in the BTR. It is held for the programmed time interval value.

To avoid transmission overruns, one additional gate control event is read ahead from the list. This operation permits the GCL to determine the next gate close events (if any) for the TCs that are open.

The scheduling is based on the gate open state and time interval of only the current and subsequent schedule. An internal accumulator is used to add the time intervals when gate controls are applied. The BTR and accumulator hold the time at which the next set of gate controls will be applied.

Figure 30-50: GCL Example - Base Time and Cycle Time

<!-- image -->

| GATE CONTROL   |   TIME INTERVAL |
|----------------|-----------------|
| OOCCCCCC       |            1000 |
| OOOOCCCC       |             500 |
| CCCCOOCC       |            2200 |
| CCCCCCOO       |             300 |

Refer to the GCL Example - Base Time and Cycle Time figure. The GCL is read progressively from the first row adhering to the schedule. The read operations continue until the list length (from LLR register) is reached and the execution of the list restarts at BTR + CTR time. At this point, the BTR is incremented by value in CTR to mark the beginning of a new cycle. In the absence of any gate controls, all gates are deemed to be in the open state during the execution of the list.

In cases where the execution time of the list is greater than the cycle time, the list is truncated and the next iteration starts when the current time equals BTR + CTR.

Table 30-88: GCL Execution Example - BTR and CTR

|   Current Time | Gate Control Applied   |   Accumulator Value (ns) |   BTR (with updates) |
|----------------|------------------------|--------------------------|----------------------|
|          14200 | OOCCCCCC               |                     1000 |                14200 |
|          15200 | OOOOCCCC               |                     1500 |                14200 |
|          15700 | CCCCOOCC               |                     3700 |                14200 |
|          17200 | CCCCCCOO               |                     4000 |                14200 |
|          18200 | OOOOOOOO               |                        0 |                20200 |
|          20200 | OOCCCCCC               |                     1000 |                20200 |
|          21200 | OOOOCCCC               |                     1500 |                20200 |

Refer to the GCL Execution Example - BTR and CTR table. The execution starts at 14200 and the first set of gate controls OOCCCCCC are applied immediately. When the time interval is 1000 ns, the next set of gate controls are applied at 14200 (BTR) + 1000 (Accumulator) = 15200 ns. Since there are no gate controls available after the execution of the last gate control and before the next iteration of the loop, the gates are deemed to be in open state during that period as depicted at time 18200.

Refer to the GCL Example - Base Time and Cycle Time, Execution Time &gt; Cycle Time figure. Because the list execution takes longer than the allocated cycle time, the list is truncated.

| GATE CONTROL   | TIME INTERVAL   |
|----------------|-----------------|
| OOCCCCCC       | 1000            |
| OOOOCCCC       | 500             |
| CCCCOOCC       | 2200 1500       |
| CCCCCCOO       | 300             |

Figure 30-51: GCL Example - Base Time and Cycle Time, Execution Time &gt; Cycle Time

<!-- image -->

Refer to the GCL Execution Example - BTR and CTR , Execution Time &gt; Cycle Time table. The list is started from the BTR + CTR. While applying the third set of gate controls, the BTR + Cycle Time (17200) &lt; BTR + Accumulator (17900). Therefore, the list is truncated and execution switches to a new iteration at 17200.

Table 30-89: GCL Execution Example - BTR and CTR, Execution Time &gt; Cycle Time

|   Current Time | Gate Control Applied   |   Accumulator Value (ns) |   BTR (with updates) |
|----------------|------------------------|--------------------------|----------------------|
|          14200 | OOCCCCCC               |                     1000 |                14200 |
|          15200 | OOOOCCCC               |                     1500 |                14200 |
|          15700 | CCCCOOCC               |                     3700 |                14200 |
|          17200 | OOCCCCCC               |                     1000 |                17200 |
|          18200 | OOOOCCCC               |                     1500 |                17200 |
|          18700 | CCCCOOCC               |                     3700 |                17200 |

## Installing a New GCL

When a new software programmed GCL is available and must be executed at the new BTR value, the switch to the new GCL can happen as a:

- New base time aligned with current schedule
- New base time unaligned with current schedule

## New Base Time Aligned with Current Schedule

If the cycle time for the new gating cycle is the same as the cycle time for the current gating cycle, and if the BTR chosen for the new gating cycle is an integer multiple of the current cycle time (+ current BTR), the new and old gating cycle are aligned. This would be the ideal case; it allows the new gating cycle to be installed and executed with no timing issues. The implementation completes the execution of an iteration of the current list, and switches to the new list at the beginning of the base time listed in the new list.

When (New BaseTime ≥ Current Time), then ConfigChangeTime = New BaseTime, Else If (New BaseTime &lt; Current Time):

1. Set the BTRError.
2. ConfigChangeTime = (New BaseTime + N* New CycleTime). where N is the smallest integer for which the relationship ConfigChangeTime ≥ CurrentTime and (N =&lt; 8) = TRUE.

When N &gt; 8, the hardware cannot auto recover and the loop count value in BTR error reporting is set to 1111. The software must reprogram the new base time.

The Switching to a New List Example - Aligned figure illustrates the installation of a new GCL that is aligned with the existing configuration. In the example, after the sixth iteration of the first GCL, the BTR values of the old and new GCL are equal. At that point, the new GCL is processed as a natural extension to the existing GCL.

Figure 30-52: Switching to a New List Example - Aligned

<!-- image -->

Table 30-90: GCL Execution Example - BTR and CTR

|   Current Time | Gate Control Applied   |   Accumulator Value |   BTR (with updates) |
|----------------|------------------------|---------------------|----------------------|
|          44200 | OOCCCCCC               |                1000 |                44200 |
|          45200 | OOOOCCCC               |                1500 |                44200 |
|          45700 | CCCCOOCC               |                3700 |                44200 |
|          47900 | CCCCCCOO               |                4000 |                44200 |
|          48200 | OOOOOOOO               |                   0 |                50200 |
|          50200 | CCCOCCCC               |                1000 |                50200 |
|          51200 | CCCCOCCC               |                3000 |                50200 |
|          53200 | OOOOOOOO               |                   0 |                56200 |

## New Base Time Unaligned with Current Schedule

If a new cycle time differs from current cycle time or the new base time is in the future and not an integer multiple of the current cycle time, the old and new cycles are unaligned. When new base time is reached (when the new configuration is installed and starts to execute), the last old cycle is normally truncated to start the first new cycle. This could be undesirable if it results in a very short last old cycle. Arguably, it would be better to extend the old cycle by that small amount, rather than start a short cycle. The cycle time extension register (related to the current configuration list) allows this extension of the last old cycle. If the last complete old cycle ends normally in less than current Cycle Time Extension (TER) ns before the new base time, then the last complete cycle (before new base time is reached) is extended so that it ends at the new base time.

The Switching to a New List Example - Unaligned figure shows how the installation of a new GCL that is unaligned with the current schedule truncates the current list.

Figure 30-53: Switching to a New List Example - Unaligned

<!-- image -->

At the end of the fifth iteration the Current time + Cycle time extension (TER) &lt; New BTR. So, the sixth iteration of current configuration is started. During the sixth iteration of the current list when the new BTR value is smaller than the next schedule in the current list, it switches to the new list.

Table 30-91: Extending to New List by Truncating the Current List

|   Current Time | Gate Control Applied   |   Accumulator Value |   BTR (with updates) |
|----------------|------------------------|---------------------|----------------------|
|          44200 | OOCCCCCC               |                1000 |                44200 |
|          45200 | OOOOCCCC               |                1500 |                44200 |
|          45700 | CCCCOOCC               |                3700 |                44200 |
|          47500 | CCCOCCCC               |                4000 |                44200 |
|          48500 | CCCCOCCC               |                   0 |                50200 |
|          50500 | OOOOOOOO               |                1000 |                50200 |

The Switching to a New List Example - Extended figure shows where the current configuration list is extended (instead of starting a new iteration). The extension time of 800 ns is less than the allowed cycle extension time (TER) of 1000 ns.

Figure 30-54: Switching to a New List Example - Extended

<!-- image -->

Table 30-92: Switching to New List by Extending the Current List

|   Current Time | Gate Control Applied   |   Accumulator Value |   BTR (with updates) |
|----------------|------------------------|---------------------|----------------------|
|          38200 | OOCCCCCC               |                1000 |                38200 |
|          39200 | OOOOCCCC               |                1500 |                38200 |
|          39700 | CCCCOOCC               |                3700 |                38200 |
|          41900 | CCCCCCOO               |                4000 |                38200 |
|          42200 | OOOOOOOO               |                   0 |                44200 |
|          45000 | CCCOCCCC               |                1000 |                45000 |
|          46000 | CCCCOCCC               |                3000 |                45000 |
|          48000 | OOOOOOOO               |                   0 |                51000 |

## Impact of Transmit Scheduling Algorithms on EST

When EST is used in isolation, the gate control list manages the final open/close state of the queues along with the checks carried out by the transmission selection algorithm in MTL. Because the gate controls operate on a predefined repetitive schedule, use the strict priority or Credit Based Shaper (CBS) scheduling algorithms.

Other algorithms such as the Weighted Round Robin (WRR), Deficit Weighted Round Robin (DWRR) and Weighted Fair Queuing (WFQ) implement masking of the queues based on the current winning queue. The algorithm is applied only among the group of queues that open simultaneously. To ensure queues whose gates are open get priority, these algorithms are modified to treat gate open queues and gate closed queues as separate groups giving priority to gate open queues.

For example, consider four queues (Q3, Q2, Q1, Q0) with weights 4:3:2:1; Q3 and Q2 are in open state in slot one slot, while Q1 and Q0 are in open state in another slot. In this case, the scheduler works as follows:

1. In the first slot, the Q3 and Q2 are serviced in the ratio of 4:3 for the duration that the slot is open.
2. In the second slot, the queues Q1 and Q0 are serviced in the ratio of 2:1.
3. Fresh arbitration is started every time a slot is opened.

The traffic does not get distributed in the intended ratio of 4:3:2:1, but rather as two groups with different ratios and only for the duration of the slot when the gates are open continuously.

## Frame Preemption (FPE)

The IEEE 802.1Qbv-2015 specification defines the schedule for each queue on every egress port that makes the implementation aware of the traffic arrival schedule. This information can be used to block the lower priority traffic from transmission in this time window. Scheduled traffic is forwarded from the sender to the receiver through all the network nodes with a deterministic delay.

To achieve a low latency, ensure that there are no interfering frames during the scheduled windows that are reserved for high priority traffic. The use of scheduled traffic imposes limitations while starting a transmission.

If an interfering frame begins transmission just before the start of a reserved time period, it can extend transmission into the reserved window, and potentially interfere with higher priority traffic. Therefore, a guard band whose size is equal to the largest possible interfering frame is required before the window starts.

A larger guard band equates to a less efficient use of network bandwidth. However, with the implementation of IEEE802.1Qbu (frame preemption), this issue is addressed. Frame preemption breaks the interfering frame into smaller fragments. Therefore, the guard band needs to be only as large as the largest possible interfering fragment instead of the largest possible interfering frame.

During the guard band, only the frames that can complete the transmission of the entire frame before the next gate close event are permitted. This ensures that the high priority traffic can always start at the beginning of the window reserved for it.

NOTE: Internal loopback is not supported for preempted frames.

Preemption allows one or more higher priority (express) frames to interrupt the transmission of a lower priority (preemptable) frame; the preemptable frame transmission is resumed and completed after the express frame transmission is complete. To support frame preemption, the following abstractions of the MAC are used:

- A preemptable MAC, called pMAC, which carries the preemptable traffic
- An express MAC, called eMAC, which carries the express traffic

In the implementation, only the parts of the MAC that hold the state during preemption are replicated and represented as pMAC and eMAC. The MAC uses the following ways to halt the transmission of preemtable traffic in the presence of express traffic:

- The MTL scheduler interrupts the preemptable traffic that is currently being transmitted. When the preemption capability is active, the MAC interrupts the transmission and reception of preemptable frames. A preempted fragment can be followed by zero or more express frames before the continuation fragments. The preemptable frame can be fragmented any number of times. However, the minimum final and non-final fragment length criterion must be met.
- Interleaving of more than one preemptable packet is not permitted. IWhen a preemptable packet is fragmented by an express packet, another preemptable packet cannot be transferred until all the remaining fragments of the first preempted packet are transferred.
- The MTL scheduler prevents starting the transmission of preemptable traffic. When the preemption capability is inactive, the pMAC entity is disabled and only express traffic is transmitted or received.

NOTE: Queue 0 can be an express queue when FPE is enabled, and EST is disabled.

## Enabling Frame Preemption

Enable the frame preemption feature by setting the EMAC\_FPE\_CTLSTAT.EFPE bit.

## GCL Modification to Support FPE

In the EST only configuration, the GCL entry has up to 24 bits of time interval, and up to 8 high order bits representing the gate open and close state requirements as shown in the Gate Control table.

Table 30-93: Gate Control - FPE Disabled

| Gate Control (up to 8 Bits)   | Time Interval (ns) 16, 20 or 24 Bits   |
|-------------------------------|----------------------------------------|
| OOCCCCCC                      | T0                                     |
| OOOOCCCC                      | T1                                     |
| CCCCOOCC                      | T2                                     |
| CCCCCCOO                      | T3                                     |
| …                             | …                                      |
| …                             | …                                      |
| OCOCOCOO                      | T126                                   |
| OOOOCCCC                      | T127                                   |

EST only supports the SetGate operation, which implies that the gates are set to either open or close at a given time interval.

However, when both FPE and EST are enabled, the GCL also supports Set-and-Hold-MAC and Set-and-ReleaseMAC operations, in addition to the SetGate operation. To enable the support of hold and release operations, the format of the GCL is slightly changed while configuring and enabling the FPE. The first queue (Q0) is always programmed to carry preemption traffic. Therefore, it is always open. The bit corresponding to Q0 is renamed as hold and release MAC control.

The TX queues whose packets are preemptable are indicated by the EMAC\_FPE\_CTRLSTS.PEC field. The GCL bit of the corresponding queue is ignored and considered as always open. So, even if the software writes a 0 (C), it is ignored for such queues.

Table 30-94: Gate Control, FPE Enabled

| Gate Control (up to 7 Bits)   |   Release/Hold Indication | Time Interval (ns) 16, 20 or 24 Bits   |
|-------------------------------|---------------------------|----------------------------------------|
| CCCOOOO                       |                         0 | T0                                     |
| CCCCOOO                       |                         0 | T1                                     |
| OCCCOOO                       |                         1 | T2                                     |
| COCCOOO                       |                         1 | T3                                     |
| CCOCOOO                       |                         1 | T4                                     |
| CCCOOOO                       |                         0 | T5                                     |
| CCCCOOO                       |                         0 | T6                                     |

When the hold and release bit transitions from a 0 to 1, a Set-and-Hold-MAC operation is performed. This operation indicates the halting of the preemptable traffic. A hold request is sent to MTL in advance (the time interval is given in the EMAC\_FPE\_ADV.HADV field). When the hold and release bit transitions from a 1 to 0, a Set-andRelease-MAC operation is performed. This operation marks the resumption of the preemptable traffic. A release request is sent to MTL in advance (the time interval is given in the EMAC\_FPE\_ADV.RADV field). The preemptable traffic is blocked for the duration that the hold and release bit is set to a 1 in the GCL.

NOTE: The EMAC\_FPE\_ADV.HADV field is the advance time to initiate the hold state. Ideally, it should be programmed to a value of minimum fragment size. The size should be smaller than the time interval (minimum fragment size + IPG + preamble overheads in nanoseconds).

## Impact of Preemption on CBS

In CBS, the definition of transmit is:

- True for the duration of frame transmission from the queue.
- False when frame transmission from the queue is complete.

When the CBS algorithm is used along with frame preemption, the value of transmit is true only while the MAC is transmitting the frame. If the frame transmission is delayed or interrupted, the value of transmit is false until transmission of the frame commences or is resumed. For example, a preemptable frame transmission is interrupted to allow the transmission of an express frame from a different queue. Or the start of express frame is delayed because a preemptable frame is being transmitted)

The value of transmit is false during the transmission of any overhead that is a consequence of frame preemption. For example, an additional frame overhead (mCRC, fragment count) that is added to the preemptable frame.

At any given time, if there are no frames in the queue, and the value of transmit is false, and credit is a positive value. The credit value is set to zero if there is no preemptable frame from the queue for which transmission is in-progress but has been interrupted.

## mPacket Format

When the preemption capability is active, the MAC sends mPackets to the PHY. An mPacket can be one of the following:

1. An express packet
2. A preemptable packet
3. An initial fragment of a preemptable packet
4. A continuation fragment of a preemptable packet

The mPacket Format figure shows the format of the mPacket. It contains an express packet, a complete preemptable packet, or the initial fragment of a preemptable packet. Format B shows the format of an mPacket containing a continuation fragment of a packet.

<!-- image -->

Figure 30-55: mPacket Format

## Preamble

The preamble in the mPacket shown in mPacket Format contains seven octets. The preamble in the mPacket shown in mPacket Format (Format B) contains six octets. Each octet contains the value of 0x55 (transmitted in order from left to right 10101010).

## Start mPacket Delimiter (SMD)

The value of the SMD indicates whether the mPacket contains an express packet, the start of a preemptable packet (initial fragment or complete packet), or any of continuation fragments of a preemptable packet. The SMD Values table shows the valid SMD values for different mPacket types.

Table 30-95: SMD Values

| mPacket Type             | Notation   | Frame Count   | Value   |
|--------------------------|------------|---------------|---------|
| Verify packet            | SMD-V      | N/A           | 0x07    |
| Respond packet           | SMD-R      | N/A           | 0x19    |
| Express packet           | SMD-E      | N/A           | 0xD5    |
| Preemptable packet start | SMD-S0     | 0             | 0xE6    |
| Preemptable packet start | SMD-S1     | 1             | 0x4C    |
| Preemptable packet start | SMD-S2     | 2             | 0x7F    |
| Preemptable packet start | SMD-S3     | 3             | 0xB3    |
| Continuation fragment    | SMD-C0     | 0             | 0x61    |
| Continuation fragment    | SMD-C1     | 1             | 0x52    |
| Continuation fragment    | SMD-C2     | 2             | 0x9E    |
| Continuation fragment    | SMD-C3     | 3             | 0x2A    |

## frag\_count

A frag\_count is a modulo 4 counter that increments for each continuation fragment of the preemptable packet. The frag\_count protects against mPacket reassembly errors by enabling the detection of the loss of up to three packet fragments.

The frag\_count field is present only in mPackets with SMD-C notation (continuation fragment). The frag\_count is zero in the first continuation fragment of each preemptable packet. The frag\_count Values table shows value associated with the frag\_count.

Table 30-96: frag\_count Values

|   frag_count | Value   |
|--------------|---------|
|            0 | 0xE6    |
|            1 | 0x4C    |
|            2 | 0x7F    |
|            3 | 0xB3    |

## mData

The contents of the packet from the MAC. It includes the first byte after the SFD to the last byte before the FCS. The contents are sent in the mData fields of one or more mPackets for that frame. The minimum size of the mData field is 60 bytes.

## CRC

The CRC field contains a Cyclic Redundancy Check (CRC) and has an indication of the final mPacket of a frame. In the final mPacket of a frame, the CRC field contains the last four octets of the MAC frame (the FCS field).

For other mPackets, the CRC field contains an mCRC value. The mCRC is calculated on the octets of the packet from the first octet of the frame (the octet following the SFD of preemption frames) to the last octet of the packet transmitted in that mPacket. The check by performing an XOR of the calculated 32 bit CRC value of the fragment and a value of 0x0000FFFF.

## Summary of Packet Formats

- Express packet - 7 bytes of preamble, SMD-E, data, and CRC
- Complete preemptable packet - 7 bytes of preamble, current preemptable packet SMD, data, CRC
- Initial fragment (non-final) of preemptable packet- 7 bytes of preamble, current preemptable packet SMD, data, mCRC
- Continuation fragments (non-final) of preemptable packet - 6 bytes of preamble, current preemptable continuation fragment SMD, current preemptable continuation fragment FC, data, mCRC
- Final fragment of preemptable packet - 6 bytes of preamble, current preemptable continuation fragment SMD, current preemptable continuation fragment FC, data, CRC

When frame preemption is enabled, the MAC receiver communicates the exceptions related to received preempted fragments (incorrect fragment sequencing, missed fragment, and so on) to MTL layer by reporting it as a CRC error. Software should not set (=1) the EMAC\_MAC\_EXT\_CFG.DCRCC bit. Otherwise, the MTL layer might forward the unintended preempted fragments to the application.

Table 30-97: Current and Previous SMD Values

| Previous Preemptable Packet SMD   | Current Preemptable Packet SMD   |
|-----------------------------------|----------------------------------|
| SMD-S0                            | SMD-S1                           |
| SMD-S1                            | SMD-S2                           |
| SMD-S2                            | SMD-S3                           |
| SMD-S3                            | SMD-S0                           |

Table 30-98: Current and Previous SMD Values

| Previous Preemptable Fragment SMD   | Previous Preemptable Fragment FC   | Current Preemptable Continua- tion Fragment SMD   | Current Preemptable Continua- tion Fragment FC   |
|-------------------------------------|------------------------------------|---------------------------------------------------|--------------------------------------------------|
| SMD-S0                              | N/A                                | SMD-C0                                            | FC0                                              |
| SMD-S1                              | N/A                                | SMD-C1                                            | FC0                                              |
| SMD-S2                              | N/A                                | SMD-C2                                            | FC0                                              |

Table 30-98: Current and Previous SMD Values (Continued)

| Previous Preemptable Fragment SMD   | Previous Preemptable Fragment FC   | Current Preemptable Continua- tion Fragment SMD   | Current Preemptable Continua- tion Fragment FC   |
|-------------------------------------|------------------------------------|---------------------------------------------------|--------------------------------------------------|
| SMD-S3                              | N/A                                | SMD-C3                                            | FC0                                              |
| SMD-C0                              | FC0                                | SMD-C0                                            | FC1                                              |
| SMD-C0                              | FC1                                | SMD-C0                                            | FC2                                              |
| SMD-C0                              | FC2                                | SMD-C0                                            | FC3                                              |
| SMD-C0                              | FC3                                | SMD-C0                                            | FC0                                              |
| SMD-C1                              | FC0                                | SMD-C1                                            | FC1                                              |
| SMD-C1                              | FC1                                | SMD-C1                                            | FC2                                              |
| SMD-C1                              | FC2                                | SMD-C1                                            | FC3                                              |
| SMD-C1                              | FC3                                | SMD-C1                                            | FC0                                              |
| SMD-C2                              | FC0                                | SMD-C2                                            | FC1                                              |
| SMD-C2                              | FC1                                | SMD-C2                                            | FC2                                              |
| SMD-C2                              | FC2                                | SMD-C2                                            | FC3                                              |
| SMD-C2                              | FC3                                | SMD-C2                                            | FC0                                              |
| SMD-C3                              | FC0                                | SMD-C3                                            | FC1                                              |
| SMD-C3                              | FC1                                | SMD-C3                                            | FC2                                              |
| SMD-C3                              | FC2                                | SMD-C3                                            | FC3                                              |
| SMD-C3                              | FC3                                | SMD-C3                                            | FC0                                              |

## Transmit Preemption

To support preemption, the MAC should have more than one transmit queue with at least one queue designated as express queue (and one queue designated as preemption queue). When FPE is enabled

( EMAC\_FPE\_CTLSTAT.EFPE =1), the MTL preempts a preemptable frame. When a hold request is asserted (EST/Qbv configured and enabled), express frames are available for transmission. The frame is present in the MTL FIFO and qualified for arbitration after ensuring that the minimum mPacket mData field size is met.

Therefore, preemption occurs only when at least 60 bytes of the preemptable frame have been transmitted and at least 64 bytes (including the frame CRC) remain.

The earliest starting position of preemption is controlled by the EMAC\_FPE\_CTRLSTS.AFSZ bit. Preemption does not occur until at least 64 x (1 + AFSZ) - 4 bytes of the preemptable frame have been sent.

When preemption occurs, all of the preemptable queues are blocked; only the express queues can arbitrate (if more than one express queue has traffic) and transmit.

The continuation fragment of the preempted frame is the first frame to be transmitted after the release request is asserted (EST/Qbv configured and enabled) and all the express traffic transmission completes.

NOTE: All the PTP packets should be transmitted as express packets.

MTL communicates the following frame-type information to the MAC using a 2-bit preemption control signal on the MTI interface (qualified with SoF and EoF). Refer to the MTI Preemption Control Values table.

- Express frame
- Preemption frame (full or fragment)
- Continuation fragment (non-final or final)

Table 30-99: MTI Preemption Control Values

| Qualifier   |   Preemption Control Values | Frame Types           |
|-------------|-----------------------------|-----------------------|
| SoF         |                          00 | Start express         |
| SoF         |                          01 | Start preemption      |
| SoF         |                          10 | Continuation fragment |
| SoF         |                          11 | Reserved              |
| EoF         |                          00 | End of frame          |
| EoF         |                          01 | End of fragment       |

The MTL must wait for the previous fragment status to be received before resuming the continuation fragments of a preempted frame.

## MAC Transmit Preemption

The MAC supports preemption by implementing the functionality needed to generate the mPackets. The functionality is based on the preemption control value received on the MTI interface (qualified with SoF and EoF). The MAC determines the frame type (see the MTI Preemption Control Values table) and generates mPackets accordingly. When the preemption capability is active, the MAC replaces the SFD of a preemption packet with an SMD-S value. A 2-bit rolling frame count is encoded in the SMD-S value. The SMD-E value is the same as the SFD value. So, the SFD of an express packet does not need to be replaced.

## Transmit Fragment Status

The MAC sends the transmit fragment status to indicate a successful transmission of fragmented mPackets. To indicate a fragment status, bit 31 of the mti\_txstatus\_o field is set (=1) and all other values in the status field are zero.

For a transmission error (for example, an underflow or jabber), the frame status is sent (and not a fragment status) with an error indication along with all other relevant status fields. For a receiving an error status, the MTL drops the remaining fragments and does not send any more continuation fragments.

## Receive Preemption

When FPE is enabled, the MAC receiver passes the incoming packets and differentiates between express packets and preemptable packets. An SMD containing an SMD-E indicates an express packet; an SMD containing an SMD-S indicates the first mPacket of a preemptable packet.

If an mPacket containing an SMD-S is received when the MAC has not completely received the previous preempted packet, MAC sets a CRC error status for the previously received partial packet.

When an SMD-S is detected, the MAC records the frame count indicated by the SMD and then begins sending data on the MRI interface.

The MAC checks the last four bytes of the mPacket. If the last four bytes of the mPacket do not match CRC, it indicates the end of the packet with or without a CRC error (CRC check result). If the last four octets of the mPacket match, it indicates that the packet was preempted.

An SMD containing an SMD-C indicates an mPacket that continues the data for a preempted packet. Upon receiving an SMD value of SMD-C, the MAC checks the following:

1. A preempted packet is in progress
2. The frame count indicated by the SMD matches the frame count of the packet in progress
3. The frag\_count value indicates the next fragment count

If any of these checks fail, the mPacket is discarded and MAC sets a CRC error status for the partially received packet.

If all the checks pass, the next fragment count is incremented modulo 4.

When a packet is preempted, the MAC saves the state of the partially received packet (filter check status, time stamp, length fields and so on) and can process any received express packets before the continuation fragment is received.

The MAC receiver sends a dummy status for all of the mPacket fragments that are successfully received. It sends the Rx status with the final fragment. If an error is detected during any of the fragments, the Rx status is sent, and the fragment is marked as final fragment. All subsequent continuation fragments received for this packet are dropped in the MAC.

The MAC communicates the following frame type information to the MTL using a 2-bit preemption control signal on the MRI interface. Refer to the MRI Preemption Control Values table.

1. Express frame
2. Preemption frame (full or fragment)
3. Continuation fragment (non-final or final)

Table 30-100: MRI Preemption Control Values

| Qualifier   |   Preemption Control Values | Frame Types           |
|-------------|-----------------------------|-----------------------|
| SoF         |                          00 | Start express         |
| SoF         |                          01 | Start preemption      |
| SoF         |                          10 | Continuation fragment |
| SoF         |                          11 | Reserved              |

Table 30-100: MRI Preemption Control Values (Continued)

| Qualifier   |   Preemption Control Values | Frame Types     |
|-------------|-----------------------------|-----------------|
| EoF         |                          00 | End of frame    |
| EoF         |                          01 | End of fragment |

## Data Alignment

When a received frame cannot be fragmented on any byte boundary, the MAC retains the unaligned bytes of data in the previous fragment and resends them with the next fragment. See the MAC Data Alignment figure.

Figure 30-56: MAC Data Alignment

<!-- image -->

The MTL can choose to ignore the partial data (based on byte enable value) that comes with the end of the fragment and instead use the data width aligned data received in the first beat of the next continuation fragment.

## MTL Receive Preemption

The MTL Rx must have at least two receive queues to support FPE function because the preemptable packets and the express packets must be routed to separate queues. The EMAC\_RXQ\_CTL0 registers control the destination Rx queue of a received packet. Program these registers such that preemptable traffic and express traffic are not routed to the same receive queue. The queue mapping for tagged packets is based on VLAN user-priority field. So, the priority of preemptable and express packets are mutually exclusive. Packets of a certain priority (traffic class) are either express or preemptable, but not both.

For the preemptable frames, in addition to the Priority Selected in Rx Queue (PSRQ) routing, a programmable Frame Preemption Residue Queue (FPRQ) is supported. This feature routes all other preemption packets received (untagged, SA/ DA or VLAN filter fails forwarded due to RA being set or VTFE being reset). Refer to the MTI Preemption Routing table.

Table 30-101: MRI Preemption Routing

| Preemption Packet Type                                                                                        | Queue Routing   |
|---------------------------------------------------------------------------------------------------------------|-----------------|
| AV tagged packets passing the SA/DA and VLAN filters                                                          | PSRQ*           |
| DCB/generic tagged packets passing the SA/DA and VLAN fil- ters                                               | PSRQ**          |
| Tagged packets failing the SA/DA and VLAN filters without setting the receive all (RA = 0) or setting VTFE =1 | Dropped         |
| All other packets or when RA = 1                                                                              | FPRQ            |

The express frames use queue 0 as a default queue. In case of filter failures, queue 0 must not be used for receiving preemption frames.

When a fragment is received, it is pushed into its destination receive queue. When this packet is preempted, and is followed by an express packet, it is pushed/stored in a different receive queue. The MTL saves the context of the preempted frame, and processes one or more express frames. When the continuation fragment arrives, it restores the saved context and pushes the remainder of the fragmented frame into its receive queue.

## MTL Receive Arbitration

On the ARI interface, data is fetched from the MTL receive FIFO based on the arbitration selected in the EMAC\_MTL\_OPMODE register. Frame-based arbitration can be used only when all the MTL preemption queues operate in store-and-forward mode. Otherwise, there is a loss of bandwidth on the ARI interface because all of the fragments of a preemptable packet are not available. Therefore, express packets received between the fragments are blocked until all of the fragments are received and transferred; this defeats the purpose of express traffic.

When operating in either threshold (cut through) or store-and-forward modes, PBL-based arbitration is recommended over frame-based arbitration. For PBL-based arbitration, the watermark check is always performed. The arbitration/transfer of data is in chunks of PBL size of data (similar to the concepts of fragments). Therefore, the express queue packets are blocked for less time and the ARI interface transfers the data without a loss of efficiency.

## Frames Support in Offload Engines

When frame preemption is enabled, all the packets processed by the offload engines (ARP , PTO) and PTP , PAUSE, and PFC packets must be received as express/normal packets. These offload engine logic and functions do not recognize preemption frames.

## Verify and Respond mPackets

When FPE function is present, the MAC can receive and detect the verify and respond mPackets, even when FPE is not enabled by software. When the MAC detects valid verify and respond mPackets, it notifies the software by configuring the EMAC\_FPE\_CTLSTAT.RVER and EMAC\_FPE\_CTLSTAT.RRSP fields, respectively. Optionally, an interrupt can be generated. These packets have empty (all-zero) data payload; they are dropped inside the MAC and not forwarded to the MRI.

Software can configure the EMAC\_FPE\_CTLSTAT.SRSP and EMAC\_FPE\_CTLSTAT.SVER fields to request the MAC to transmit verify and respond mPackets, respectively. Upon successful transmission of these frames, the MAC clears the EMAC\_FPE\_CTLSTAT.SRSP / EMAC\_FPE\_CTLSTAT.SVER bit and sets the EMAC\_FPE\_CTLSTAT.TVER / EMAC\_FPE\_CTLSTAT.TRSP bits. Optionally, an interrupt can be generated when these events occur.

## NOTE: 1. For preemption frames received with a CRC error, ignore the following status fields:

- Packet length
- Dribble error indication
- Runt frame indication
2. When FPE is configured, the EMAC\_RQ0\_OPMODE.FUP (RXQ(n)) bit should not be set.
3. When FPE is configured, do not disable CRC checking upon receive. Internally, FPE error fragments are identified using CRC checks. For details, see the EMAC\_MAC\_EXT\_CFG.DCRCC field.
4. Express and preemption traffic must be routed to different DMAs.

## MMC Counter and Interrupt Registers

The following MMC counters and associated interrupt registers are instantiated/present in the MAC when the frame preemption feature and any one of the existing MMC counters are selected during RTL configuration.

Table 30-102: MMC Counters and Interrupt Registers

| Frame Assembly Error Counter   | Description                                                                                                                                     | Associated MMCCounter   |
|--------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------|
| Frame assembly error counter   | A 32-bit counter that provides the number of MAC frames with reassembly errors on the receiver, due to a mismatch in the frag- ment count value | EMAC_RXPKT_ASERR_CNT    |
| Frame SMD error counter        | A 32-bit counter that provides the number of MAC frames rejected due to arriving with an SMD-C when there was no preceding preempted frame      | EMAC_RXPKT_SMDERR_CNT   |
| Frame assembly OK counter      | A 32-bit counter that provides the number of MAC frames that were successfully reas- sembled                                                    | EMAC_RXPKT_ASOK_CNT     |
| MAC Rx fragment counter        | A 32-bit counter that provides the number of additional mPackets received due to pre- emption.                                                  | EMAC_RXFPE_FRGMNT_CNT   |
| MAC Tx fragment counter        | A 32-bit counter that provides the number of additional mPackets transmitted due to preemption.                                                 | EMAC_TXFPE_FRGMNT_CNT   |

Table 30-102: MMC Counters and Interrupt Registers (Continued)

| Frame Assembly Error Counter   | Description                                                                               | Associated MMCCounter   |
|--------------------------------|-------------------------------------------------------------------------------------------|-------------------------|
| Hold request counter           | A 32-bit counter that maintains the count of the number of hold requests given to the MAC | EMAC_TXHLDRQ_CNT        |

## Additional Registers Associated with MMC Interrupts

Following are the additional registers associated with MMC interrupts for the MMC error counters:

- EMAC\_MMC\_FPE\_TXINT
- EMAC\_MMC\_FPE\_TXIMSK
- EMAC\_MMC\_FPE\_RXINT
- EMAC\_MMC\_FPE\_RXIMSK

## Time-Based Scheduling

The time-based scheduling feature is suitable for traffic whose periodicity and rate are predictable. This feature is optional. To improve the quality-of-service of such traffic:

- The transmit DMA fetches the packet from the host memory for transmission at designated time. This method helps the software to set up the transmit descriptors before the packet is ready/available. It reduces the overhead on the software and avoids constant monitoring of the time. It prepares the descriptors for the targeted transmission time of the packet.
- The MAC transmits the packet only at the designated/pre-determined time even when the packets are fetched in advance. This method helps in maintaining a constant transmission rate that can be consumed by the receiver station; therefore, it avoids congestion and excessive buffering in the network.

The time-based scheduling feature supports fetching and launching an Ethernet packet at (or after) a pre-determined time. The time-based scheduling is supported only in the following modes:

- Full duplex mode
- Link speed is 100 Mbps or higher
- NOTE: · Do not enable time-based scheduling for the channel for which TSO feature is enabled.
- In EST enabled configurations, the EMAC\_TBS\_CTRL.ESTM bit impacts the time-based scheduling feature only when the DUT processes the GCL list.
- Slot number is meaningful only after GCL is active. So, start the GCL list before enabling the TBS and EST features. If GCL is not started, all gates are deemed open. If packets do not make progress, the Tx queue creates back pressure.

## TBS Definitions

The following definitions apply to time-based scheduling:

- Launch time - the time beyond which MTL can schedule the packet for transmission
- Fetch time - the time beyond which the Tx DMA can schedule a packet-fetch from the host memory
- Expiry time - the time beyond which the packet is dropped by MTL

## Launch Time

The launch time is specified in the enhanced normal descriptor in the DMA configurations. For more details, see Enhanced Descriptor for Time-Based Scheduling. The launch time can also be specified as a control word for each packet. When the launch time is specified in the DMA configuration, it is valid only for the specific frame that is scheduled for transmission. The launch time is reset after the transmission of one frame.

The formats of the launch time include:

- Normal/absolute format
- EST/offset format

In normal/absolute format, the launch time is an absolute time value at which the packet is launched for transmission. The launch time is interpreted in the normal/absolute format when EMAC\_TBS\_CTRL.ESTM = 0.

The launch time is a 32-bit value, where the most-significant 8-bits represent the time in seconds and the rest 24 bits represent the time in 256 ns. The launch time is compared against the IEEE 1588 based system/PTP time (bits [39:8]) and rolls over after 256 seconds.

The maximum value of the lower 32-bits of system time is 999,999,999 decimal (0x3B9AC9FF). It wraps to zero when reaching this value (representing a full second). Therefore, the maximum value of the lower 24 bits of the launch time (after multiplying by 256) must be 0x3B9AC9.

Because the maximum value of launch time is 256 seconds:

- Launch time is greater than current system time when its value is between system time[39:8] and system time[39:8] + 128 s
- Launch time is less than current system time when its value is between system time[39:8] + 128 s and system time[39:8] + 256 s. This computation is modulo 256.

In EST/offset format, the launch time is an offset value relative to the time indicated by the BTR of the GCL list provided in the GCL Slot Number (GSN). The value in the BTR is always updated to the start time of the current loop of the GCL. For each packet, the GSN value and the launch time value are specified in the enhanced transmit descriptor.

The launch time offset is a 32-bit value; the upper 8-bits represent the time in seconds and the rest 24 bits represent the time in 256 ns, which is added to the BTR value corresponding to GSN. The value of the launch time offset must be smaller than the value of the cycle time (specified in the CTR register that is implemented when EST is enabled). If the CTR is greater than or equal to 1s, the maximum value of the lower-24 bits of launch time offset should be 999,999,999 decimal (0x3B9AC9FF).

In EST/offset format, the launch time is a 64-bit value, which is interpreted as:

Launch time = Launch GSN BTR[63:0] + (Launch time offset[31:0] &lt;&lt; 8), which is compared with the system time [63:0].

where:

GSN BTR - the base time value at which the launch GSN loop started execution.

GSN - A modulo 16 count of the GCL loop count. The count is incremented for every new GCL loop; installation of new GCL list does not impact the count. The current GCL slot number can be obtained by reading the EMAC\_EST\_STAT.CGSN field. The maximum value of GSN is 15. Therefore, GSN values between the EMAC\_EST\_STAT.CGSN field and CGSN+8 represent current or future slots; all other GSN values are interpreted as elapsed slots or past slots. So, for the correct interpretation of time, GSN value must be between EMAC\_EST\_STAT.CGSN and CGSN+8 for the correct interpretation of time.

## Launch Expiry Time

In the normal/absolute format, when EMAC\_TBS\_CTRL.LEOV = 1, the EMAC\_TBS\_CTRL.LEOS bit determines the maximum amount of time a frame is eligible for launch, starting from the time the frame becomes eligible for launch.

The launch expiry offset is a 24-bit value defined in 256 ns units, with a maximum possible value of 999,999,999 ns (0x3B9AC9FF).

Launch expiry time = (launch time[39:8] + EMAC\_TBS\_CTRL.LEOS [32:8]) * 256 ns

The packet with a specific launch time is considered eligible for transmission when the launch time is less than the system time and (if EMAC\_TBS\_CTRL.LEOV = 1) the system time is less than the launch expiry time.

When the system time is greater than the launch expiry time the frame is categorized as expired and is dropped from the MTL FIFO.

- NOTE: 1. For correct interpretation and meaningful operation, the fetch, launch, and launch expiry time should never be set to a value larger than current system time + 128 s; such a value is interpreted as time that has already elapsed.
2. In full duplex mode, the frames dropped from the MTL FIFO have an error summary (bit 15) and Excessive Deferral (bit 3) of the TxStatus set.

In the EST/offset format, when EMAC\_TBS\_CTRL.LEOV = 1, the launch expiry GSN offset ( EMAC\_TBS\_CTRL.LEGOS ) and launch expiry offset ( EMAC\_TBS\_CTRL.LEOS bits determine the maximum amount of time a frame remains eligible for launch, starting from the time the frame becomes eligible for launch.

The launch expiry offset is computed as:

Launch expiry offset = ( EMAC\_TBS\_CTRL.LEGOS : EMAC\_TBS\_CTRL.LEOS )

The launch expiry offset is a 24-bit value defined in the units of 256 ns, with a maximum possible value of the smaller of 999,999,999 ns or CTR-1 ns. The EMAC\_TBS\_CTRL.LEGOS field holds the GSN offset (multiples of CTR time). The EMAC\_TBS\_CTRL.LEOS field holds maximum value of CTR (sub-CTR values) in ns.

The launch expiry GSN is computed as:

```
Launch expiry GSN = (launch GSN + EMAC_TBS_CTRL.LEGOS + CCMA) where,
```

CCMA - the CTR carry due to modulo addition. This value is 1 if ((launch time offset + EMAC\_TBS\_CTRL.LEOS ) &lt;&lt; 8) ≥ CTR.

- When CCMA = 1, launch expiry offset = (launch time offset+ EMAC\_TBS\_CTRL.LEOS ) - CTR
- When CCMA = 0, launch expiry offset = (launch time offset + EMAC\_TBS\_CTRL.LEOS ).

Launch expiry time = launch expiry GSN BTR[63:0] + launch expiry time offset

- When EMAC\_TBS\_CTRL.LEOV = 0, the launch expiry time is not checked.
- When EMAC\_TBS\_CTRL.LEOV =1, and
- The system time is greater than the launch expiry time, the frame is dropped from MTL FIFO. The frame is considered as expired.
- The launch time is smaller than the system time and launch expiry time is greater than system time, the frame is considered eligible for launch.

## NOTE:

## Fetch Time

Software can indicate the fetch time for each packet ( EMAC\_DMA\_TBS\_CTL0.FTOV =1). When EMAC\_DMA\_TBS\_CTL0.FTOV = 0, the fetch time offset is not valid and the DMA fetches packets without any time constraints. The fetch time accounts for all possible delays in the DMA fetch operation and ensures that the frame is present in the MTL FIFO before the launch time.

In normal/absolute mode, fetch time is derived/calculated by reducing the time specified in the EMAC\_DMA\_TBS\_CTL0.FTOS field from the given launch time. In a normal mode of operation, the fetch launch time is computed as:

Fetch time[39:8] = (launch time[39:8] -EMAC\_DMA\_TBS\_CTL0.FTOS [31:8])

The fetch time is 32 bits. It is compared against system time[39:8] to determine the eligible for fetching the frame:

- The fetch time is defined as greater than system time when the fetch time is in the range of system time[39:8] and system time[39:8] + 128 s. The frame is considered ineligible for fetch.
- The fetch time is defined as smaller than the system time when the fetch time is in the range of system time[39:8] + 128 s and system time[39:8] + 256 s. The frame is considered as eligible for fetch.
1. The max value of EMAC\_TBS\_CTRL.LEGOS is 7. This implies that when EMAC\_TBS\_CTRL.LEOV = 1, the frame has a maximum lifetime of &lt; 8 GCL loop iterations after it becomes eligible for launch.
2. The slot number of the first GCL list executed each time after EST is enabled, is zero.

This computation is modulo 256 .

In the EST/offset mode, the EMAC\_DMA\_TBS\_CTL0.FGOS field provides the slot number offset to be deducted from the launch GSN. In this case, the EMAC\_DMA\_TBS\_CTL0.FTOS value should be the smaller of 999,999,999 ns or CTR-1 ns.

When (launch time offset ≥ EMAC\_DMA\_TBS\_CTL0.FTOS ):

- Fetch time offset = ((launch time offset -EMAC\_DMA\_TBS\_CTL0.FTOS ) x 256 ns)
- CBFS (CTR borrow for fetch subtraction) = 0

When (launch time offset &lt; EMAC\_DMA\_TBS\_CTL0.FTOS ):

- Fetch time offset = CTR + ((launch time offset -EMAC\_DMA\_TBS\_CTL0.FTOS ) x 256 ns)
- CBFS = 1

The fetch GSN is computed as follows:

Fetch GSN = launch GSN -EMAC\_DMA\_TBS\_CTL0.FGOS - CBFS

Fetch time = Fetch GSN BTR[63:0] + Fetch time offset

The frame is marked eligible for DMA fetch when the fetch time is smaller than the system time.

The max value of EMAC\_DMA\_TBS\_CTL0.FGOS is 7. This implies that when EMAC\_DMA\_TBS\_CTL0.FTOV =1, the frame can be fetched at a maximum of &lt;8 GCL loop iterations before it becomes eligible for launch.

## DMA Operations Sequence

When EMAC\_DMA\_TBS\_CTL0.FTOV = 1, the following DMA operations sequence occurs:

1. Fetch the first enhanced normal descriptor. (FD is set).
2. If LTV is set and fetch enabled, compute the fetch time based on the launch time. Wait for the system time to be greater than fetch time.
3. Read the frame (data) from the host memory and transfer to MTL FIFO.
4. Close the normal descriptor.
5. Fetch the next normal descriptor (if the previous descriptor was not the last).
6. Repeat steps 4 to 6, until the last descriptor of the frame (LD is set).

After the last descriptor of a frame, the next enhanced normal descriptor should be programmed with a new launch time and with the LTV bit set. Otherwise, the subsequent frames are processed without any time restrictions.

## Control Word

Time-based scheduling ensures that a packet is allowed to arbitrate for transmission only when the system time is greater than the launch time.

- Launch time and GSN use the same field that holds the 64-bit time stamp value for One Step Timestamping Correction (OSTC). When the LTV bit is set and OSTC bit is not set (bit 19 of the first control word), the time stamp field is interpreted as the launch time. Launch time uses the fields meant for OSTC time stamp[39:8]; GSN uses the OSTC time stamp[43:40] field.
- Bit 31 of control word 0 is the equivalent of the LTV field. The last two Control D words provide the launch time and GST.
- The last two control D words are the time stamp for OSTC when bit 19 of control word 0 (OSTC) is set.

When the MTL TxFIFO read controller reads the first control word, it checks bit 31 to determine if the frame has a valid launch time. If bit 31 is set, it reads the subsequent control words to get the launch time.

## Impact on Transmission Selection Algorithm

The time-based scheduler does not directly influence the transmission selection algorithm. However, it has an indirect influence as it determines whether a queue is eligible to participate in the scheduling.

When a frame of a specific queue has a valid launch time, the time-based scheduler ensures that the frame participates in transmit scheduling (TRC scheduler) only after the launch time has elapsed.

The gating done by time-base scheduler is only for the scheduling (picking frame for transmission). It might not alter any other characteristics and behavior of the frame. For example, the frame waiting for the launch time to lapse continues to gain credits in CBS. The CBS credits are not lost because the frame is marked as available, but not ready. Strict priority or CBS are the recommended transmit scheduling algorithms for the TBS implementation. Scheduling for TBS-enabled queues is more predictable.

The Time-Based Scheduling Flow figure shows the time-based scheduling flow.

Figure 30-57: Time-Based Scheduling Flow

<!-- image -->

## Flow Control

This section describes the flow control feature of EMAC for transmit and receive paths.

## Transmit Flow Control

The transmit flow control involves transmitting pause packets in full-duplex mode and backpressure in half-duplex mode to control the flow of packets from the remote end.

## Flow Control in Full-Duplex Mode

In full-duplex mode, the EMAC uses one of the following packet types for flow control:

- IEEE 802.3x pause packets
- Priority flow control (PFC) packets

The EMAC\_RXFLOW\_CTL.PFCE bit determines whether PFC packets or IEEE 802.3x pause control packets are used for flow control. If the PFCE bit is set, the MAC sends the PFC packets.

## Pause Packet Structure

The Pause Packet Fields table describes the fields of a pause packet.

Table 30-103: Pause Packet Fields

| Field              | Description                                                                       |
|--------------------|-----------------------------------------------------------------------------------|
| DA                 | Contains the special multicast address                                            |
| SA                 | Contains the MAC address 0                                                        |
| Type               | Contains 8808                                                                     |
| MAC Control Opcode | Contains 0001 for IEEE 802.3x Pause Control packets; 0101 for PFC packets         |
| PT                 | Contains Pause time specified in the PT field of the MAC_Q#_Tx_Flow_Ctrl register |

## Pause Packet Control

When the EMAC\_Q[i]\_TXFLOW\_CTL.FCB\_BPA bit is set, the MAC generates and transmits a single pause packet. If the FCB bit is set again after the pause packet transmission is complete, the MAC sends another pause packet irrespective of whether the pause time is complete or not. To extend the pause or terminate the pause prior to the time specified in the previously transmitted pause packet, the application should program the pause time register with appropriate value and then again set the FCB\_BPA bit.

Similarly, when the flow control signal from FIFO layer (mti\_flowctrl\_i) is asserted, the MAC generates and transmits a single pause packet. If the mti\_flowctrl\_i signal remains asserted at a configurable number of slot times before the pause time runs out, the MAC transmits a second pause packet. This process is repeated if the mti\_flowctrl\_i signal remains active. If the mti\_flowctrl\_i signal goes inactive prior to the sampling time, the MAC transmits a pause packet with zero pause time (if the bit is set to 0) to indicate to the remote end that the receive buffer is ready to receive new data packets. For PFC packets, the priority, pause time, and other controls can be specified for a queue in the EMAC\_Q[i]\_TXFLOW\_CTL register. The MAC supports independent flow control for each receive queue. There is one trigger input for each receive queue. Therefore, when multiple triggers come simultaneously, the MAC sends one PFC packet for each trigger. Based on the hardware trigger or software trigger through respective queues, the MAC sends a PFC packet with programmed pause time and priority. If multiple priorities are programmed in same receive queue, multiple priorities are set for the PFC packet with same pause time values. A separate pause timer is available for each receive queue.

## Flow Control in Half-Duplex Mode

In half-duplex mode, the MAC uses the deferral mechanism for the flow control (backpressure). When the application requests to stop receiving packets, the MAC sends a JAM pattern of 32 bytes when it senses a packet reception, provided the transmit flow control is enabled. This results in a collision and the remote station backs off. If the application requests a packet to be transmitted, it is scheduled and transmitted even when the backpressure is activated. If the backpressure is kept activated for a long time (and more than 16 consecutive collision events occur), the remote stations abort the transmission because of excessive collisions.

The Transmit MAC Flow Control table describes the flow control in the transmit path for queue 0 based on the setting of the following bits:

- EMAC\_RQ0\_OPMODE.EHFC

- EMAC\_Q[i]\_TXFLOW\_CTL.TFE
- EMAC\_MAC\_CFG.DM

Flow control is similar for all queues.

Table 30-104: Transmit MAC Flow Control

| EHFC   |   TFE | DM   | Description                                                                                                                                                                                                                                                                                   |
|--------|-------|------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| x      |     0 | x    | The MAC transmitter does not perform the flow control or backpressure operation.                                                                                                                                                                                                              |
| 0      |     1 | 0    | The MAC transmitter performs back-pressure when bit 0 of the EMAC_Q[i]_TXFLOW_CTL register is set or the sideband signal sbd_flowctrl_i = 1.                                                                                                                                                  |
| 1      |     1 | 0    | The MAC transmitter performs back-pressure when bit 0 of the EMAC_Q[i]_TXFLOW_CTL register is set or the sideband signal sbd_flowctrl_i is 1. In addition, the MAC Tx performs back-pressure when the Rx queue level crosses the threshold set by bits[10:8] of the EMAC_RQ0_OPMODE register. |
| 0      |     1 | 1    | The MAC transmitter sends the pause packet when bit 0 of the EMAC_Q[i]_TXFLOW_CTL register is set or the sideband signal sbd_flowctrl_i = 1.                                                                                                                                                  |
| 1      |     1 | 1    | The MAC transmitter sends the pause packet when bit 0 of the EMAC_Q[i]_TXFLOW_CTL register is set or the sideband signal sbd_flowctrl_i = 1. In addition, the MAC Tx sends a pause packet when the Rx queue level crosses the threshold set by bits[10:8] of the EMAC_RQ0_OPMODE register.    |

## Enabling Transmit Flow Control

To independently enable transmit flow control for each receive queue, set the EMAC\_Q[i]\_TXFLOW\_CTL.TFE bit.

## Triggering Transmit Flow Control

Transmit flow control involves transmitting pause packets in full-duplex mode and backpressure in half-duplex mode to control the flow of packets from the remote end. The application can request the MAC to send a pause packet or initiate back-pressure by using either of the following methods:

- Software Trigger: In this method, the application sets the EMAC\_Q[i]\_TXFLOW\_CTL.FCB\_BPA bit.
- Hardware Trigger: In this method, the application triggers the flow control by asserting the mti\_flowctrl\_i signal based on the receive queue threshold.
- Receive Queue Threshold: The flow control operation of the MAC is enabled when the EMAC\_RQ0\_OPMODE.EHFC bit is set. The flow control signal to the MAC is asserted when the fill level of the receive queue crosses the threshold configured in the EMAC\_RQ0\_OPMODE.RFA bit field. This flow control signal is de-asserted when the fill-level of the queue is lower than the threshold configured in the EMAC\_RQ0\_OPMODE.RFD field.

The hardware flow control generated based on the receive queue threshold crossing condition is applicable only when the receive queue size is 4,096 bytes or more.

NOTE: The RFA sets the threshold at which the flow control is triggered and the MAC schedules to send a PAUSE frame to be sent by the local transmitter.

The remote transmitter can continue to send packets until it receives this PAUSE packet. So, the RXQ needs space to take in this data even after the flow-control is triggered, to avoid overflow and loss of packets.

The max-space required that is calculated theoretically can be 2 max-sized packets + a little more when the read from the RXFIFO is not occurring during this whole time.

Theoretically, this makes RFA = 4 or more, which implies that RxQ must have at least a size of 4KB.

Practically, the system works with RFA = 2 (Full - 2KB, with little probability of overflow). When the application has a FIFO size of less than 4K, with RFA = 2, the flow control is triggered with 1000B in the RxQ, which is less than maximum packet size.

The EMAC\_RQ0\_OPMODE.RFA bit field is used for activating the flow control. These bits control the threshold (fill-level of Rx queue) at which the flow control is activated.

For example, if the RxQ size is 2K and the program sets the EMAC\_RQ0\_OPMODE.RFA bit field to 1K; as soon as 1K is received, flow control is activated. If the program sends a packet of 1500 bytes (which is allowed as per spec), flow-control is triggered even before a complete packet is received. This reduces the throughput considerably.

When the RxQ size is 4K, the program can set the RFA to 2K, and the whole packet can be received.

NOTE: Receive pause packets should have a frame size of 64 bytes.

## Receive Flow Control

Receive flow control is implemented by the MAC based on the bit value of the respective register, and the destination address and different fields of the received packet.

The Receive MAC Flow Control table describes the flow control in the receive path based on the setting of the EMAC\_RXFLOW\_CTL.RFE and EMAC\_MAC\_CFG.DM bits.

Table 30-105: Receive MAC Flow Control

|   RFE | DM   | Description                                                                                                           |
|-------|------|-----------------------------------------------------------------------------------------------------------------------|
|     0 | X    | The MAC receiver does not detect the received pause packets                                                           |
|     1 | 0    | The MAC receiver does not detect the received pause packets but recog- nizes such packets as control packets.         |
|     1 | 1    | The MAC receiver detects or processes the pause packets and responds to such packets by stopping the MAC transmitter. |

For EMAC0, the program can enable the PFC packet detection by setting the EMAC\_RXFLOW\_CTL.PFCE bit. If PFC packet detection is enabled, the MTL transmit queue corresponding to the received priority is blocked when the PFC packet is received. If PFC packet detection is not enabled in configurations with multiple queues and the RFE bit is enabled, the MAC transmitter is blocked when the 802.3x pause packet is received.

The following list describes the receive flow control.

1. The MAC checks the destination address of the received pause packet for either of the following:
- Multicast destination address: The DA matches the unique multicast address specified for the control packet (48'h0180C2000001).
- Unicast destination address: The DA matches the content of the MAC address register 0 and the EMAC\_RXFLOW\_CTL.UP bit. If the UP bit is set, the MAC processes pause packets with a unicast destination address in addition to the unique multicast address.
2. The MAC decodes the following fields of the received packet:
- Type field: This field is checked for 16'h8808.
- Opcode field: This field is checked for 16'h0001 (pause packet) or 16'h0101 (PFC packet).
- Pause Time or Pause Time Vector field: The pause time (for pause packet) is captured to determine the time for which the transmitter needs to be blocked. The Pause Time Vector field (for a PFC packet) is captured to determine the time for which the MTL transmit queue corresponding to the received priority needs to be blocked.
- Priority Enable Vector field: This field is valid only for PFC packets. It is captured to determine the MTL transmit queue corresponding to the received priority.
3. If the byte count of the status indicates 64 bytes and there is no CRC error, the MAC transmitter does one of the following:
- For 802.3x Pause packets, the MAC pauses the transmission of any data packet for the duration of the decoded pause time value multiplied by the slot time (64-byte times).
- For PFC packets, the MAC blocks the transmit queues corresponding to the priority. Based on the priorities assigned to the transmit queues in the EMAC\_TXQ\_PRTY\_MAP0 and EMAC\_TXQ\_PRTY\_MAP1 registers, the MAC asserts the respective bit in the mti\_disable\_txq\_o signal and loads the pause timer corresponding to the priority. The transmit queue corresponding to the priority is blocked until the pause timer expires.
- In addition to the PFC-based flow control operation, when the PFCE bit is set, all bits of the mti\_disa-ble\_txq\_o signal are asserted while in LPI mode so that the ETS or CBS scheduler can start over after exiting the LPI mode.
4. The MAC transfers the received control packet to the application based on the setting of the EMAC\_MACPKT\_FILT.PCF bit field.

If subsequent pause or PFC packets are received before the earlier pause time expires, the MAC updates the pause timer with new value.

## Loopback Mode

The MAC supports Loopback of transmitted packets to its receiver.

## Guidelines for Using Loopback Mode

The following are some guidelines for using the loopback mode:

- Enable loopback only with the full-duplex mode. In half-duplex mode, the carrier sense signal (crs) or collision (col) signal inputs get sampled which may result in issues such as packet dropping.
- If the loopback mode is enabled without connecting a PHY chip (for example, in a FPGA setup), the program should externally generate the transmit and receive clocks and provide these clocks to the MAC.
- Do not loop back big packets. Big packets may get corrupted in the loopback FIFO.

The transmit and receive clocks can have an asynchronous timing relationship. Therefore, an asynchronous FIFO is used to make the loopback path of the EMAC\_TXD[n] data to the receive path. The asynchronous FIFO is 10-bits (6-bits in 10/100 Mbps mode) wide to accommodate EMAC\_TXD[n] , TXEN, and TX\_ER. The depth of the FIFO is five in 1000 Mbps mode and nine in 10/100 Mbps mode. The FIFO is free running to write on the write clock TX\_CLK and read on every read clock RX\_CLK.

At the start of each packet read out of the FIFO, the write and read pointers are re-initialized to have an offset of 2 (4 in 10/100 Mbps mode). This avoids overflow or underflow during a packet transfer. This also ensures that the overflow or underflow occurs only during the IPG period between the packets. The FIFO depth of five or nine is sufficient to prevent data corruption for packet sizes up to 9,022 bytes with a difference of 200 ppm between MII/ RMII/RGMII transmit and receive clock frequencies. Therefore, bigger packets should not be looped back because they may become corrupt in this loopback FIFO.

At the end of every received packet, the Receive Protocol Engine module generates received packet status and sends it to the Receive Packet Controller module. The control, missed packet, and filter fail status are added to the Receive status in the Receive Packet Controller module. The MAC does not process ARP packets that are looped back.

## Enabling Loopback Mode

To enable this feature, program the EMAC\_MAC\_CFG.LM bit.

Programs can enable loopback for all PHY interfaces. The data is always looped back through an internal asynchronous FIFO on to the internal receive MII/RMII/RGMII interface, irrespective of which PHY interface is selected. The loopback data is also passed through the corresponding transmit PHY interface block, on to the Ethernet line.

- NOTE: · During loopback, the data/packet is reflected on TXD[n] signal.
- Preemption is not supported in loopback mode.