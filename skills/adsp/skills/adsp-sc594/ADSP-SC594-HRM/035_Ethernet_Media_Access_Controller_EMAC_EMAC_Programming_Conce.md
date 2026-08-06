# Ethernet Media Access Controller (EMAC) — EMAC Programming Concepts

<!-- source: 035_Ethernet_Media_Access_Controller_EMAC_EMAC_Programming_Conce.pdf | original pages 1754–1756 -->

2. Output the PPS waveform by configuring the EMAC\_TM\_PPSCTL.PPSCTL0 bits and binary or digital rollover using the EMAC\_TM\_CTL.TSCTRLSSR bit. See Fixed Pulse-Per-Second Output.

## Programming Flexible Pulse-Per-Second Output

Use the following procedure to program flexible PPS output.

1. Enable the PTP module by setting the EMAC\_TM\_CTL.TSENA bit.
2. Set the EMAC\_TM\_PPSCTL.PPSEN bit to enable flexible PPS output.
3. Program the EMAC\_TM\_PPSCTL.TRGTMODSEL0 bits with 11 or 10 (for target time trigger interrupt).
4. Program the start time value when the PPS output starts using the EMAC\_TM\_PPS0TGTM and EMAC\_TM\_PPS0NTGTM registers. Ensure that the EMAC\_TM\_PPS0NTGTM.TSTRBUSY bit is reset before programming the target time registers again.
5. Program the period of the PPS signal output using the EMAC\_TM\_PPS0INTVL register for pulse train output. Program the width of the PPS signal output in the EMAC\_TM\_PPS0WIDTH register for single pulse or pulse train output.
6. Ensure that the EMAC\_TM\_PPSCTL.PPSCTL0 bits are cleared. Then, program the bits to 0001 to start single pulse, or to 0010 to start pulse train at programmed start time (Step 4).
7. ADDITIONAL INFORMATION: The PPS pulse train is free-running unless stopped by a STOP pulse train at time command ( EMAC\_TM\_PPSCTL.PPSCTL0 = 0100) or STOP pulse train immediately command EMAC\_TM\_PPSCTL.PPSCTL0 = 0101).
7. The start of pulse generation can be canceled by giving the cancel start command ( EMAC\_TM\_PPSCTL.PPSCTL0 = 0011) before the programmed start time (Step 4) elapses.
8. Program the stop time value when the PPS output must stop using the EMAC\_TM\_PPS0TGTM and EMAC\_TM\_PPS0NTGTM registers. Ensure that the EMAC\_TM\_PPS0NTGTM.TSTRBUSY bit is reset before programming the target time registers again.
9. Ensure that the EMAC\_TM\_PPSCTL.PPSCTL0 bits are cleared. Then, program the bits to 0100. This programming stops the train of pulses on PPS signal output after the programmed stop time (Step 8) elapses.
11. ADDITIONAL INFORMATION: The pulse train can be stopped immediately by giving the STOP pulse train immediately command ( EMAC\_TM\_PPSCTL.PPSCTL0 = 0101). Program the EMAC\_TM\_PPSCTL.PPSCTL0 bits to 0110 before the programmed stop time (Step 8) elapses to cancel the stop pulse train command (given in Step 9).

## EMAC Programming Concepts

The following sections provide basic information and guidelines to help with programming the EMAC module.

## IEEE 802.3 Ethernet Packet Structure

The IEEE 802.3 Frame Structure table provides typical frame format of an Ethernet packet. Refer to the IEEE standards for detailed information on Ethernet packets and their format.

Table 29-52: IEEE 802.3 Frame Structure

| Parameter   | Description                                                                                                                                                                                                                                                                                           |   Position in Ether- net Packet | Total Bytes   |
|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------|---------------|
| PREAMBLE    | This parameter is a 56-bit (7-byte) pattern of alternating 1 and 0 bits (#10101010), which allows devices on the network to detect a new incoming frame for synchronization.                                                                                                                          |                               1 | 7             |
| SFD         | The SFD (#10101011) is a 1-byte pattern designed to break the preamble pattern, and signal the start of the actual frame.                                                                                                                                                                             |                               2 | 1             |
| DA          | 48-bit destination address. This parameter can be a unicast, multicast, or broadcast address.                                                                                                                                                                                                         |                               3 | 6             |
| SA          | 48-bit long source address, typically a unicast, multicast, or broadcast address.                                                                                                                                                                                                                     |                               4 | 6             |
| LT          | Typically this field is the length, in terms of the number of bytes, and can be anywhere between 0-1500. When the value is greater than or equal to 0x0600, this field also indicates the type of special payload carried by the frame. Examples include 0x8808 for flow control and 0x0800 for IPv4. |                               5 | 2             |
| DATA        | Actual application data payload, usually between 0-1500.                                                                                                                                                                                                                                              |                               6 | 0-1500        |
| PAD         | This field compensates for data frames that are shorter than 64 bytes long, not including the preamble.                                                                                                                                                                                               |                               7 | 0-46          |
| FCS         | The frame check sequence is a 32-bit cyclic redundancy check that detects corrupted data within the entire frame. This parameter is generated from a CRC-32 polynomial code (CRC-32-IEEE): G (x) = x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1.                    |                               8 | 4             |

## Frame Size Statistics for Application Software

Table 29-53: Ethernet Frame Size Statistics

| Frame size statistics        | VLAN-specific change Comments                                                                       |                                                                             |
|------------------------------|-----------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------|
| Information bytes/Header     | 4 byte 802.1Q header inserted after source address and before Type/LAN in 802.3 packets = 22 bytes. | 6 x 2 + 2 + 4 = 18 bytes (DA+SA+LT+FCS)                                     |
| Minimum Frame Size (typical) | If DATA is NULL, 42-byte padding makes 64 bytes (42 +22)                                            | 64 bytes. If DATA is NULL, 46-byte padding makes 64 bytes (46 +18)          |
| Maximum Frame Size (typical) | 1522 bytes                                                                                          | 1518 bytes (1500 bytes DATA and 18-bytes header)                            |
| Jumbo Frame Size             | 9022 bytes                                                                                          | Typical industry standard. Ethernet jumbo frame size treated as 9018 bytes. |

## Software Visualization of Programmable Packet Size

The Visualization of Programmable Packet Size table provides the byte sizes of packets with various configurations.

Table 29-54: Visualization of Programmable Packet Size

| Size in Bytes   | Comments                                                                          |
|-----------------|-----------------------------------------------------------------------------------|
| 16384           | Receive watchdog and transmit jabber disabled, jumbo frames enabled.              |
| 10240           | Receive watchdog and transmit jabber disabled, jumbo frames disabled.             |
| 2048            | Receive watchdog and transmit jabber enabled.                                     |
| 1518            | Typical max size of Ethernet frame. Receive watchdog and transmit jabber enabled. |
| 64              | Typical minimum size of Ethernet frame.                                           |
| < 64            | Runt frames requiring Zero-PAD.                                                   |

## Ethernet Packet Structure in C

The following is an example for Ethernet packet structure in the C language.

```
typedef struct ETHER_PACKET { char  dst_addr[6];             //destination address char  src_addr[6];             //source address char  length[2];               //length of actual data char  data[DATA_SIZE];         //application data char  fdlimit[DELIMIT_SIZE];   //32-bit delimit (if manual appending) char  fcs[4];                  //crc frame checksum, used by RX buffer. } ETHER_PACKET;
```

## DMA Descriptor Implementation in C

The following code is a simple implementation of descriptors in ring and chain model in C language. Typically 4 WORDs (32-bit) are used for descriptors. Using checksum offload or the PTP engine requires 8 WORDs. Only high-level common functions across transmit and receive descriptors are considered here.

```
/* DMA Ring Descriptor */ typedef struct EMAC_DMADESC_RING { unsigned int        Status;     //TDES0 OR RDES0 unsigned int        ControlDesc;  //TDES1 OR RDES1 unsigned int        StartAddr1;   //TDES2 OR RDES2 unsigned int        StartAddr2;   //TDES3 OR RDES3 #ifdef CHECKSUM_OFFLOAD struct EMAC_EXT_STAT      ExtendedStat; #endif } EMAC_DMADESC_RING; /* DMA Chain Descriptor */ typedef struct EMAC_DMADESC_CHAIN
```