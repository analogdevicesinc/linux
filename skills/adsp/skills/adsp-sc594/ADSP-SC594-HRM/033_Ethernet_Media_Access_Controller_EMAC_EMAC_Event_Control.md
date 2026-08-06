# Ethernet Media Access Controller (EMAC) — EMAC Event Control

<!-- source: 033_Ethernet_Media_Access_Controller_EMAC_EMAC_Event_Control.pdf | original pages 1744–1746 -->

The LPI LS timer counts, in milliseconds, the time expired since the link status is up. Programs can enable the monitoring of Bit 3 EMAC\_ISTAT.RGMIIIS (link status) of Register 54 (SGMII/RGMII/SMII Control and Status Register) by setting the EMAC\_LPI\_CTLSTAT.PLSEN bit.

The link status is indicated to the MAC by Bit 3 (link status) of Register 54 (RGMII Control and Status Register) or the value programmed by the software in the EMAC\_LPI\_CTLSTAT.PLS bit. If the link status is not available in Register 54 (RGMII Control and Status Register), the software should get the PHY link status by reading the PHY register and accordingly update the EMAC\_LPI\_CTLSTAT.PLS bit.

This timer is cleared every time the link goes down. It starts to increment when the link is up again and continues to increment until the value of the timer becomes equal to the terminal count. When the terminal count is reached, the timer remains at the same value as long as the link is up. The terminal count is the value programmed in the EMAC\_LPI\_TMRSCTL.LST bit field. The GMII interface does not assert the LPI pattern unless the terminal count is reached. This ensures a minimum time for which no LPI pattern is asserted after a link is established with the remote station. This period is defined as 1 second in the IEEE standard 802.3-az, version D2.0. The LPI LS timer is 10-bit wide. Therefore, the software can program up to 1023 milliseconds

## LPI TW TIMER

The LPI TW timer counts, in microseconds, the time expired since the de-assertion of LPI. The terminal should be programmed using the EMAC\_LPI\_TMRSCTL.TWT bit. The terminal count of the timer is the value of resolved Transmit TW that is the auto-negotiated time after which the MAC can resume the normal transmit operation. After exiting the LPI mode, the MAC resumes its normal operation after the TW timer reaches the terminal count.

The MAC supports the LPI TW timer in units of microsecond. The LPI TW timer is 16-bit wide. Therefore, the software can program up to 65535 us.

NOTE: Program the EMAC\_LPI\_CTLSTAT.PLS bit to 1'b0 before switching between the GMII and MII modes. This resets the internal timers. If the mode is changed after the LPI LS timer or LPI TW timer starts, the change in the TX clock frequency can result in incorrect timeout.

## LPI Interrupt

The MAC generates the LPI interrupt when the Tx or Rx side enters or exits the LPI state. The interrupt is asserted when the LPI interrupt status is set. The LPI interrupt can be cleared by reading the EMAC\_LPI\_CTLSTAT register.

## EMAC Event Control

The EMAC has dedicated interrupt signals registered with the system event controller (SEC) module. Various interrupt sources within the EMAC peripheral are shared through this interrupt line. Refer to the System Event Controller (SEC) and Generic Interrupt Controller (GIC) chapter for details on how interrupts work in this product and how to configure them.

## EMAC Interrupt Signals

Interrupts from the EMAC are triggered from the EMAC DMA layer or the EMAC CORE layer. Interrupts are triggered from EMAC DMA when a particular status bit is set in the EMAC\_DMA0\_STAT register. An interrupt line is asserted only when the corresponding bits are enabled in the DMA interrupt enable register. Similarly, interrupts are triggered from the EMAC CORE when a particular MMC status bit, RGMII link status bit, LPI, or PTP status bit is set in the interrupt status register.

An interrupt line is asserted only when the corresponding bits are enabled in the MMC mask registers for MMC counters or the interrupt mask register for PTP . DMA status register also reflects the MMC interrupt status. The following lists show the two groups of interrupts in the DMA status register.

NIS - Normal Interrupt source summary:

- Transmit Interrupt
- Transmit Buffer Unavailable
- Receive Interrupt
- Early Receive Interrupt

AIS - Abnormal Interrupt source summary:

- Transmit Process Stopped
- Transmit Jabber Timeout
- Receive FIFO Overflow
- Transmit Underflow
- Receive Buffer Unavailable
- Receive Process Stopped
- Receive Watchdog Timeout
- Early Transmit Interrupt
- Fatal Bus Error

As an enhancement on ADSP-SC59x processors, to minimize overall interrupt latency, there are two new types of interrupts in addition to the legacy EMAC\_STAT interrupt. These new interrupts are called EMAC\_MAC and EMAC\_DMA interrupts. Each DMA channel contains its own individual EMAC\_DMA interrupt.

The interrupts routed to EMAC\_MAC are as follows: GLPII - GMAC LPI Interrupt

- TTI - Timestamp Trigger Interrupt
- MCI - MAC MMC Interrupt
- MCI - MAC MMC Interrupt

The interrupts routed to EMAC\_DMA (3 EMAC\_DMA interrupts for EMAC0 and 1 for EMAC1) are as follows:

## Normal Interrupt Sources (NIS):

- TI - Transmit Interrupt
- TU - Transmit Buffer Unavailable
- RI - Receive Interrupt
- ERI - Early receive Interrupt

## Abnormal Interrupt Sources (AIS):

- TPS - Transmit Process Stopped
- TJT - Transmit Jabber Timeout
- UNF - Transmit Buffer Underflow
- ETI - Early Transmit Interrupt
- OVF - Receive Buffer Overflow
- RU - Receive Buffer Unavailable
- RPS - Receive Process Stopped
- RWT - Receive Watchdog Timeout
- FBI - Fatal Bus Error

The EMAC generates an interrupt only once for simultaneous, multiple events. The driver must read the EMAC\_DMA0\_STAT register for the cause of the interrupt. It can generate a new interrupt once the driver has cleared the appropriate bit in DMA status register.

For example, the controller generates a receive interrupt ( EMAC\_DMA0\_STAT.RI bit) and the driver begins reading the EMAC\_DMA0\_STAT register. Next, a receive buffer unavailable interrupt ( EMAC\_DMA0\_STAT.RU bit) occurs. The driver clears the EMAC\_DMA0\_STAT.RI bit but the internal interrupt signal is not deasserted, because of the active or pending EMAC\_DMA0\_STAT.RU interrupt. The driver must scan all of the descriptors, from the last recorded position to the first one owned by the DMA, to know which descriptor has asserted the interrupt.

Interrupts are cleared by writing a 1 to the corresponding bit position in the EMAC\_DMA0\_STAT register. When all the enabled interrupts within a group are cleared, the corresponding summary bit is cleared.

An interrupt delay timer provides (receive interrupt watchdog timer register) flexible control of the receive interrupt.

When the interrupt timer is programmed with a non-zero value, it is activated as soon as the RxDMA transfers a received frame to system memory. The transfer occurs without asserting the receive interrupt. This interrupt is not enabled in the corresponding receive descriptor ( RDES1 [31] in the receive DMA descriptors).

When this timer runs out (per the programmed value), the EMAC\_DMA0\_STAT.RI bit is set. The interrupt is asserted when the corresponding EMAC\_DMA0\_STAT.RI bit is enabled in the interrupt enable register. The timer