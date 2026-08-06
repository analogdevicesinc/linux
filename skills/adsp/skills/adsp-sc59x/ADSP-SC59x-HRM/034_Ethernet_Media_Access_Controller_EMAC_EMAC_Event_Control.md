# Ethernet Media Access Controller (EMAC) — EMAC Event Control

<!-- source: 034_Ethernet_Media_Access_Controller_EMAC_EMAC_Event_Control.pdf | original pages 2072–2075 -->

## Endian Support

The EMAC DMA supports both little endian and big endian modes. The following table illustrates the order of bytes on 32-bit data bus for little endian and big endian modes, respectively.

Table 30-106: Endian Support

| Transfer Size        | Address OffsetA[1:0]   | Data[31:24]        | Data[23:16]        | Data[15:8]         | Data[7:0]          |
|----------------------|------------------------|--------------------|--------------------|--------------------|--------------------|
| Little-Endian Mode   | Little-Endian Mode     | Little-Endian Mode | Little-Endian Mode | Little-Endian Mode | Little-Endian Mode |
| Always word transfer | 0                      | MSB                | MSB - 1            | LSB + 1            | LSB                |
| Big Endian Mode      | Big Endian Mode        | Big Endian Mode    | Big Endian Mode    | Big Endian Mode    | Big Endian Mode    |
| Always word transfer | 0                      | LSB                | LSB + 1            | MSB - 1            | MSB                |

The endian mode can be programmed using the PADS\_PCFG0.EMAC0\_ENDIANNESS /

PADS\_PCFG0.EMAC1\_ENDIANNESS registers. This bit should be set for big-endian mode and cleared for little endian mode. The reset value of this bit is '1' (big endian mode).

## EMAC Event Control

The EMAC has registered with the system event controller (SEC) module. Various interrupt sources within the EMAC peripheral are shared through this interrupt line. Refer to the SEC chapter for details on how interrupts work in this product and how to configure them.

## EMAC Interrupt Signals

Interrupts from the EMAC are triggered from the EMAC DMA layer MTL, or the EMAC CORE (MAC) layer. The following table shows the EMAC interrupt signals.

Table 30-107: EMAC Interrupt Signals

| Interrupt Type   | Description                                                       |
|------------------|-------------------------------------------------------------------|
| EMAC_STAT        | Combined interrupt signal includes MAC, MTL, and DMAinter- rupts. |
| EMAC_MAC         | Interrupts from MAC and MTL                                       |
| EMAC_DMA[n]_TX   | DMATransmit Per Channel Interrupts                                |
| EMAC_DMA[n]_RX   | DMAReceive Per Channel Interrupts                                 |

Interrupts can be generated because of various events in the EMAC. These events are captured in status registers, and interrupt enables are provided for each source of an interrupt such that the interrupt signal ( EMAC\_STAT ) is asserted for an event only when the corresponding interrupt enable is set.

On EMAC, the interrupt status and corresponding enable registers are organized in an hierarchical manner so that it is easier for software to traverse and identify the source of an interrupt event quickly. When EMAC\_STAT is asserted, the EMAC\_DMA\_ISTAT register is the first level that indicates the major blocks for the interrupt event source.

This register is read-only, and it contains bits corresponding to each DMA channel (transmit and receive pair), the MTL, and the MAC. The software application must then read one (or more) of the following registers corresponding to the bits that are set:

- EMAC\_MAC\_ISTAT
- EMAC\_MTL\_ISTAT
- EMAC\_DMA[n]\_STAT

## Interrupts from the MAC

Interrupts can be generated from the MAC because of various events in the MAC Receiver, Transmitter, or the modules/functions such as RMON counters, EEE and so on. The MAC interrupts are of level type, that is, the interrupt remains asserted (high) until it is cleared by the application or software. The EMAC\_MAC\_ISTAT register describes the events that can cause an interrupt from the MAC. The MAC interrupts are disabled by default. Each event can assert the interrupt on the EMAC\_STAT or EMAC\_MAC signals when the corresponding bit is set in the EMAC\_MAC\_IEN register.

The interrupt register bits only indicate the block from which the event is reported. The program must read the corresponding status registers and other registers to clear the interrupt. For example, when set high, bit 0 ( EMAC\_MAC\_ISTAT.RGSMIIIS ) indicates that the link status on the RGMII, SGMII, or SMII interface has changed. The program must read the EMAC\_PHYIF\_CTLSTAT register to clear this interrupt event.

- NOTE: By default, the MAC interrupt status bits are cleared when the register that contains the source of the interrupt is read. If the EMAC\_CSR\_SW\_CTL.RCWE bit is programmed to 1, the MAC interrupt status bits are cleared when the bit that contains the source of the interrupt is explicitly written to 1.
- NOTE: Both the EMAC\_STAT and the EMAC\_MAC interrupt signals are generated for interrupts from the MAC. The EMAC\_MAC interrupt is always generated one cycle ahead of the EMAC\_STAT interrupt. If both the EMAC\_STAT and EMAC\_MAC interrupts are enabled:
- For GIC/ARM, only the EMAC\_MAC interrupt service routine is called
- For SEC/SHARC, both the EMAC\_STAT and EMAC\_MAC interrupt services routines is called

## Interrupts from the MTL

The EMAC can generate interrupts because of events in the MAC layer or in the MTL modules. The EMAC\_STAT output is the interrupt signal, which is a level signal (asserted until the interrupt source is read and cleared). In the MTL, the interrupts are mainly related to exception events in the TxQ or RxQ in the transmit and receive paths, respectively. The interrupt status is captured and organized in a hierarchical manner to identify the root cause quickly.

The EMAC\_MTL\_ISTAT register identifies the top-level modules that can cause the interrupt to be asserted. Bits[7:0] identify 8 Queues (transmit or receive). They are read-only bits, and the application should read the corresponding EMAC\_Q[n]\_INT\_CTLSTAT register to identify the exact cause and set the corresponding bits to 1'b1

to clear the interrupt event. The assertion of EMAC\_MAC due to these events is enabled by the corresponding enable bits in the EMAC\_Q[n]\_INT\_CTLSTAT register.

## Interrupts from DMA

The EMAC\_STAT interrupt is a level signal and gets de-asserted only when all the enabled interrupt events are cleared in their respective status registers and correspondingly all the bits in the EMAC\_DMA\_ISTAT register are cleared.

The EMAC\_DMA[n]\_STAT register captures all the interrupt events of that transmit DMA and receive DMA channel pair. The EMAC\_DMA[n]\_IEN register contains the corresponding enable bits for each interrupt event. There are two groups of interrupts in the DMA channel, normal and abnormal. They are indicated by the EMAC\_DMA[n]\_STAT.NIS and EMAC\_DMA[n]\_STAT.AIS bits. The normal group is for events that happen during the normal transfer of packets ( EMAC\_DMA[n]\_STAT.TI , EMAC\_DMA[n]\_STAT.RI , EMAC\_DMA[n]\_STAT.TBU ) while the abnormal interrupt events are for error events. Interrupt events are cleared by writing 1'b1 to the corresponding bit position. When all the enabled interrupt events are cleared (including the NIS and AIS), the interrupt source from the DMA channel is cleared and the corresponding bit in the EMAC\_DMA\_ISTAT register is also cleared.

Interrupts are not queued. If the same interrupt event occurs again before the driver responds to the previous one, no additional interrupts are generated. For example, the EMAC\_DMA[n]\_STAT.RI bit indicates that one or more packets were transferred to the application buffer. The driver must scan all descriptors, from the last recorded position to the first one, owned by the DMA to determine how many packets are received.

An interrupt is generated only once for multiple events. The driver must scan the EMAC\_DMA\_ISTAT register for the cause of the interrupt and clear the source in the respective status register. The EMAC\_STAT is cleared only when all the bits of the EMAC\_DMA\_ISTAT register are cleared.

## Periodic Scheduling of Transmit and Receive Interrupt

To improve the throughput and performance, EMAC supports interrupt timer and transmit descriptor to generate interrupts periodically, instead of every DMA transfer.

It is not preferable to generate interrupts for every packet transferred by DMA ( EMAC\_DMA[n]\_STAT.RI and EMAC\_DMA[n]\_STAT.TI ) for system throughput performance reasons. The EMAC gives the flexibility to schedule the interrupt at regular intervals using two methods:

- Set the interrupt on completion bit in transmit descriptor (TDES2[31]) once for every 'required' number of packets to be transmitted.
- Similarly, set the IOC (see RDES3[30]) bit only at some specific intervals of receive descriptors. This way, whenever a received packet transfer to system memory is complete, and any of the descriptors used for that packet transfer has the IOC bit set, only then is the event is generated.

In addition, an interrupt timer ( EMAC\_DMA[n]\_RXINTWDTMR ) is given for flexible control and periodic scheduling of receive interrupts. When this interrupt timer is programmed with a nonzero value, it gets activated as soon as the receive DMA completes a transfer of a received packet to system memory without asserting the Receive Interrupt because the corresponding interrupt of completion IOC bit (see Table 30-26 Receive Descriptor Fields 3

(RDES3)) is not set. When this timer runs out as per the programmed value, the bit is set, and the interrupt is asserted when the corresponding EMAC\_DMA[n]\_IEN.RIE bit is enabled. The timer is stopped and cleared before it expires, iwhen the RI bit is set for a packet transfer whose descriptor's IOC was set. The timer is reactivated automatically after the next packet transfer is complete without the RI event being generated.

## Per Channel Transfer Complete Interrupt

The transmit transfer complete interrupt (TI) and receive transfer complete interrupt (RI) are reflected in the EMAC\_DMA[n]\_STAT register. The EMAC\_DMA[n]\_STAT.TI bit is set whenever the transmit DMA channel closes the transmit descriptor in which the IOC (interrupt on completion - TDES2[31]) bit is set. Similarly, the EMAC\_DMA[n]\_STAT.RI bit is set whenever the receive DMA channel closes the receive descriptor with LD bit set and in any of the descriptors used for transferring that packet, IOC (interrupt enable on completion RDES3[30]) bit is set.

The common EMAC\_STAT output signal is asserted for the transfer complete interrupts only when the corresponding interrupts are enabled in the EMAC\_DMA[n]\_IEN register.

EMAC also supports the following per channel transfer complete interrupt signals.

- EMAC\_DMA[n]\_TX (transmit per channel interrupts)
- EMAC\_DMA[n]\_RX (receive per channel interrupts)

The behavior of the RI/TI/ EMAC\_DMA[n]\_TX / EMAC\_DMA[n]\_RX changes depending on the settings of the EMAC\_DMA\_MODE.INTM bit field [17:16]. The following explains the transfer complete interrupt behavior.

Table 30-108: Transfer Complete Interrupt Behavior

|   S. No. |   Interrupt Mode (INTM) | Behavior of EMAC_DMA[n]_TX and EMAC_DMA[n]_RX                                                                                                                                                                                                                                                                                                          | Behavior of EMAC_STAT                                                                                                                                                                                                                                                                                                                                  |
|----------|-------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|        1 |                       0 | A pulse is asserted on these output signals when corresponding TX/RX transfer com- plete event (for which IOC descriptor bits are enabled) is detected, irrespective of the corresponding interrupt status.                                                                                                                                            | The TI/RI status signals are set whenever the 'Transfer complete' event is detected. The bits get cleared whenever the software driver writes '1' to these bits. The EMAC_STAT is asserted whenever the corresponding inter- rupts are also enabled in the EMAC_DMA[n]_IEN register. The EMAC_DMA[n]_STAT.NIS status bit is asserted for RI/TI events. |
|        2 |                       1 | These signals reflect the value of correspond- ing EMAC_DMA[n]_STAT.TI / EMAC_DMA[n]_STAT.TI bits when the corresponding interrupt enable is set. There- fore, they are level signals and are cleared by the application by writing 1'b1 to the RI/TI status bits. This signal is not asserted when the corresponding interrupt enable bit is not set. | The EMAC_STAT signal and EMAC_DMA[n]_STAT.NIS status bit are not asserted for RI/TI events.                                                                                                                                                                                                                                                            |