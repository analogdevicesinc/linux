# Ethernet Media Access Controller (EMAC) — ADSP-SC59x EMAC Register Descriptions — Transmit VLAN Packets Good Counter Register

<!-- source: 304_Ethernet_Media_Access_Controller_EMAC_ADSP-SC59x_EMAC_Regist.pdf | original pages 2596–2596 -->

## Transmit VLAN Packets Good Counter Register

The EMAC\_TX\_VLAN\_PCNT\_G register provides the number of good VLAN packets transmitted by the EMAC module.

Figure 30-324: EMAC\_TX\_VLAN\_PCNT\_G Register Diagram

<!-- image -->

Table 30-379: EMAC\_TX\_VLAN\_PCNT\_G Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | TXVLANG    | Tx VLAN Packets Good. The EMAC_TX_VLAN_PCNT_G.TXVLANG field provides the number of good VLAN packets transmitted. |