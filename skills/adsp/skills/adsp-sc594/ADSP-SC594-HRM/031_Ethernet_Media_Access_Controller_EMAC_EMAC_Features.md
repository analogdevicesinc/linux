# Ethernet Media Access Controller (EMAC) — EMAC Features

<!-- source: 031_Ethernet_Media_Access_Controller_EMAC_EMAC_Features.pdf | original pages 1645–1646 -->

## 29   Ethernet Media Access Controller (EMAC)

The EMAC peripheral in the processor enables network connectivity to applications through an Ethernet interface. The module is fully compliant to the following standards:

NOTE: On the processor, EMAC0 is 10/100/1000 BaseT-compliant and supports AVB (Audio Video Bridging) standards.

- Carrier Sense Multiple Access With Collision Detection (CSMA/CD) Access Method and Physical Layer Specifications, Standard 802.3-2005, Institute of Electrical and Electronics Engineers (IEEE).
- Standard for a Precision Clock Synchronization Protocol for Networked Measurement and Control Systems, Standard 1588-2008, Institute of Electrical and Electronics Engineers (IEEE).
- Reduced Media Independent Interface Specification, Revision 1.2, RMII Consortium.
- Reduced Gigabit Media Independent Interface (RGMII), Revision 2.6, HP/Marvell.
- Timing and Synchronization for Time-Sensitive Applications in Bridged LANs, Standard 802.1AS-2011, Institute of Electrical and Electronics Engineers (IEEE).
- Forwarding and Queuing Enhancements for Time-Sensitive Streams, Standard 802.1Qav-2009, Institute of Electrical and Electronics Engineers (IEEE).
- The EMAC supports IEEE 803.3az-2010 standard for Energy Efficient Ethernet. This feature enables the Media Access Control (MAC) sublayer along with a family of physical layers to operate in the Low-Power Idle (LPI) mode.

NOTE: Copyright © 2010 Synopsys, Inc.; portions of this chapter are included with permission from Synopsys, Inc.

## EMAC Features

The Ethernet features include the following:

- Supports 10/100 Mbps data transfer rates with external PHY interfaced through RMII
- Full-duplex and half-duplex support for Ethernet
- Dedicated DMA controller with independent read write channels

- Supports dual-buffer (ring) or linked-list (chained) descriptor chaining
- Direct interface with the system crossbar bus
- Provides support for CSMA/CD protocol for half-duplex operation
- IEEE 802.3x flow control for full-duplex and half-duplex
- Automatic network monitoring statistics with management counters
- Flexible address filtering options for uni-cast, multi-cast, and broadcast addresses
- Support for promiscuous mode in reception
- Supports IEEE 802.1Q VLAN tag detection
- VLAN tag-based frame filtering - Perfect match and hash-based filtering
- Supports programmable Inter-frame Gap (IFG)
- Checksum offload engine for checking IPv4 header checksum and TCP/UDP/ICMP checksum encapsulated in IPv4 or IPv6 datagrams
- Station management interface for PHY device configuration and management
- Automatic CRC and pad generation controllable on a per-frame basis
- CRC replacement, source address field insertion or replacement, and VLAN insertion, replacement, and deletion in transmitted frames with per-frame control
- Layer 3 and layer 4-based frame filtering - TCP or UDP over IPv4 or IPv6
- Supports Gigabit data transfer rates with external PHY interfaced through RGMII
- Ethernet frame timestamping as described in IEEE 1588-2002 and IEEE 1588-2008. The transmit or receive status of each frame include 64-bit timestamps
- Hardware assisted time stamping capable of up to 8-ns resolution
- Automatic detection of PTP messages through Ethernet, IPv4, and IPv6 packets
- Four programmable PPS outputs that physically represent PTP system time
- Auxiliary snapshot to timestamp external events
- Four auxiliary input pins available
- Two separate channels (channel 1 and channel 2) or queues for transmission and reception of time-sensitive traffic. Channel 0 is available by default and carries the legacy best-effort Ethernet traffic on the transmit side.
- IEEE 802.1-Qav specified credit-based shaper (CBS) algorithm for channel 1 and channel 2
- Slot number function to schedule the data fetching by DMA from the system memory for channel 1 and channel 2