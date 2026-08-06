# Ethernet Media Access Controller (EMAC) — EMAC Features

<!-- source: 032_Ethernet_Media_Access_Controller_EMAC_EMAC_Features.pdf | original pages 1858–1863 -->

## 30   Ethernet Media Access Controller (EMAC)

The EMAC peripheral in the processor enables network connectivity to applications through an Ethernet interface.

The module is fully compliant to the following standards:

- IEEE 802.3-2015 for Physical Layer Specifications.
- IEEE 1588-2008 for Precision Clock Synchronization Protocol for Networked Measurement and Control Systems.
- IEEE 802.1AS-2011 and 802.1-Qav-2009 for Forwarding and Queuing Enhancements for Time-Sensitive Streams or Audio Video (AV) traffic.
- IEEE 802.1AS-Rev/D4.0 for Timing and Synchronization for Time-Sensitive Applications in Bridged Local Area Networks.
- IEEE 802.3az-2010 for Energy Efficient Ethernet (EEE).
- IEEE 802.1Qbv-2015, 802.1Qbu-2016, and 802.1AS-Rev D5.0 for Time-Sensitive Networking (TSN).
- IEEE 802.1Qaz-2011 and 802.1Qbb-2011 for Data Center Bridging (DCB).
- Reduced Gigabit Media Independent Interface (RGMII), Revision 2.6, HP/Marvell.
- Reduced Media Independent Interface Specification, Revision 1.2, RMII Consortium.
- NOTE: Copyright © 2020 Synopsys, Inc.; portions of this chapter are included with permission from Synopsys, Inc.
- NOTE: References to MII in this document are applicable to both MII and RMII; references to GMII are applicable to RGMII.

## EMAC Features

The EMAC features include the following:

## MAC Tx and Rx Features

- 10/100/1000 Mbps data transfer rates with external PHY interfaced through RGMII/RMII/MII

- Support for both full-duplex and half-duplex operations
- CSMA/CD protocol for half-duplex operation
- IEEE 802.3x flow control for full-duplex and using backpressure support for half-duplex

## MAC Transmit Features

- Automatic CRC and pad generation controllable on a per-frame basis
- Programmable packet length to support Standard or Jumbo Ethernet packets with up to 16 KB of size
- Programmable Inter-Frame Gap (IFG)
- Source address field insertion or replacement, and VLAN insertion, replacement, and deletion in transmitted frames with per-frame control
- Insertion, replacement, or deletion of up to two VLAN tags
- Option to transmit packets with reduced preamble size in full-duplex mode
- Insert, replace, or delete queue/channel based VLAN tags
- Frame preemption for MAC Tx

## MAC Receive Features

- Automatic pad and CRC stripping options
- Preamble and SFD deletion
- Programmable watchdog timeout limit
- Flexible address filtering modes
- Additional packet filtering (VLAN tag based, layer 3, layer 4)
- IEEE 802.1Q VLAN tag detection
- Frame preemption for MAC Rx

## MAC Transaction Layer (MTL) Features

- Eight queues each on the transmit and receive path
- Programmable threshold capability for each queue
- Programmable burst length

## MTL Transmit Features

- Store-and-forward mechanism or threshold mode (cut-through) for transmission to the MAC

- Programmable queue size in configurations with multiple queues. Each queue size can be programmed in terms of 256 bytes
- Automatic retransmission of collision packets in half-duplex mode
- Discard packets on late collision, excessive collisions, excessive deferral, and underrun conditions with appropriate status
- Option to calculate and insert IPv4 header checksum and TCP , UDP , or ICMP checksum
- Programmable interrupt options for different operational conditions
- Statistics by generating pulses for packets dropped (because of underflow) in the Tx FIFO
- Statistics related to bandwidth consumption by each queue of up to 16 blocks over a 125 us period
- Optional packet-level control for:
- VLAN tag insertion or replacement
- Ethernet source address insertion
- Layer 3/layer 4 checksum insertion control
- One-step time stamp
- Time stamp control
- CRC and pad control
- Following scheduling algorithms in configurations with multiple queues:
- Weighted Round Robin (WRR)
- (When Data Center Bridging is enabled) Deficit Weighted Round Robin (DWRR)
- (When Data Center Bridging is enabled) Weighted Fair Queuing (WFQ)
- Strict Priority (SP)
- (When Audio-Video Bridging is enabled) Credit-based Shaper (CBS)
- (When TSN is enabled), Enhancement to Scheduled Traffic (EST)
- (When TSN is enabled), Time Based Scheduling (TBS)Option to support dropping of Tx status to improve the transmit throughput

## MTL Receive Features

- Programmable Rx queue threshold in threshold (or cut through) mode
- Option to filter all error packets on reception and not forward them to the application in the store-and-forward mode
- Option to forward the undersized good packets

- Statistics by generating pulses for packets dropped (because of overflow) in the Rx FIFO
- Automatic generation of pause packet control or backpressure signal to the MAC based on the Rx queue fill level
- Arbitration among queues when multiple queues are present. The following arbitration schemes are supported:
- Weighted Round Robin (WRR)
- Weighted Strict priority (WSP)
- Strict Priority (SP)
- Option to replicate received multicast packets for transfer by multiple Rx DMA channel
- Option to have a programmable lookup table based flexible parser for filtering and steering the Rx packets
- Address Recognition Protocol (ARP) offload for IPv4 packets

## DMA Block Features

- Multichannel transmit and receive engines (eight transmit and eight receive channels)
- Separate DMA channel in the transmit path for each queue in MFL
- Single or multiple DMA channels for any number of queues in MFL receive path
- Dual-buffer (ring) descriptor support
- Comprehensive status reporting for normal operation and transfers with errors
- Individual programmable burst length for Tx DMA and Rx DMA engines
- Programmable interrupt options for different operational conditions
- Per-packet transmit or receive complete interrupt control
- Round-robin or fixed-priority arbitration between the receive and transmit engines
- Fixed-priority, weighted-strict-priority or weighted-round-robin arbitration for data read requests from multiple transmit DMA engines
- Start and stop modes
- Support for TCP Segmentation Offload (TSO) and UDP Fragmentation Offload (UFO) with selectable number of Tx DMA channels
- Routing of received packets to the DMA channels based on the DA or VLAN priority in multichannel DMA configurations
- Option to split the packet header (layer 3 and layer 4) and payload in different buffers
- Time-sensitive conditional packet fetching from system memory by comparing the slot time or IEEE 1588 time information provided in the descriptor (useful for AV applications)

- Programmable control for transmit descriptor posted writes to improve the throughput
- Separate channels or queues for AV data transfer in 100 Mbps and 1000 Mbps modes
- Up to eight queues on the receive paths for AV traffic and seven queues on the transmit path for AV traffic
- IEEE 802.1-Qav specified credit-based shaper (CBS) algorithm for transmit channels
- Single Tx FIFO and Rx FIFO (MFL) for all selected queues
- Programmable slot interval with range from 1µs to 4096 µs and granularity of 1µs

## Data Center Bridging Features

- Separate channels or queues for DCB data transfer in 100 Mbps and 1000 Mbps modes
- Up to eight queues on the transmit and receive paths for DCB traffic
- The following Tx queues scheduling mechanisms meet the compliance specification in IEEE 802.1-Qaz Enhanced Transmission Selection (ETS) algorithm:
- Weighted Round Robin (WRR)
- Deficit Weighted Round Robin (DWRR)
- Weighted Fair Queuing (WFQ)
- Strict Priority (SP)
- Common memory for all selected Tx or Rx queues
- Priority-based Flow Control (PFC) on individual queues configured for DCB
- Programmable control to route received VLAN tagged packets to channels or queues

## Time Sensitive Networking Features

- IEEE 802.1Qbv-2015, Enhancements to Scheduling Traffic
- IEEE802.1Qbu/802.3br, Frame preemption and Interspersing Express Traffic

## Precision Time Protocol (PTP) Features

- Ethernet frame time stamping as described in IEEE 1588-2002 and IEEE 1588-2008. The transmit or receive status of each frame include 64-bit time stamps
- Hardware assisted time stamping capable of up to 8 ns resolution
- Automatic detection of PTP messages through Ethernet, IPv4, and IPv6 packets
- Four programmable PPS outputs that physically represent PTP system time
- Auxiliary snapshot to time stamp external events - four auxiliary input pins available

- PTP time stamp offload support
- Support for one-step time stamp

Table 30-1: EMAC0/EMAC1 Feature Differences

| Feature                                                 | EMAC0              | EMAC0            | EMAC1       | EMAC1       |
|---------------------------------------------------------|--------------------|------------------|-------------|-------------|
| Feature                                                 | SC594              | SC598            | SC594       | SC598       |
| Speed                                                   | 10/100/1000 Mbps   | 10/100/1000 Mbps | 10/100 Mbps | 10/100 Mbps |
| Line Interface                                          | MII, RMII, RGMII   | MII, RMII, RGMII | RMII        | RMII        |
| Double VLAN Processing                                  | No                 | Yes              | No          | No          |
| Queue/Channel-based VLAN Tag Insertion on Tx            | No                 | Yes              | No          | No          |
| Additional MAC Address Registers                        | 1                  | 31               | 1           | 1           |
| Address Filter Hash Table Size                          | 64 bits            | 256 bits         | 64 bits     | 64 bits     |
| Extended RX VLAN Tag-based Fil- ters                    | 0                  | 4                | 0           | 4           |
| Layer 3 and Layer 4 Filters                             | 1                  | 8                | 1           | 1           |
| Multicast Packet Duplication Sup- port                  | No                 | Yes              | No          | No          |
| Number of Tx/Rx Queues and DMAChannels                  | 3                  | 8                | 1           | 1           |
| Data Center Bridging (DCB)                              | No                 | Yes              | No          | No          |
| Audio Video Bridging (AVB)                              | Yes                | Yes              | No          | No          |
| Additional AVB DMAchannels                              | 2                  | 7                | N/A         | N/A         |
| Precision Time Protocol (PTP)                           | Yes                | Yes              | No          | No          |
| PTP - Number of Auxiliary snap- shots                   | 4 with 4 deep FIFO | 4 with 16 deep   | N/A         | N/A         |
| PTP - Timestamp Offloading (Au- to Send PTP frames)     | No                 | Yes              | N/A         | N/A         |
| PTP - One Step Timestamp Fea- ture                      | No                 | Yes              | N/A         | N/A         |
| TSN-EST (Enhancements to Scheduling Traffic) (802.1Qbv) | No                 | Yes              | No          | No          |
| Frame Preemption Support (802.3br, 802.1Qbu)            | No                 | Yes              | No          | No          |
| Time Based Scheduling (launch time)                     | No                 | Yes              | No          | No          |