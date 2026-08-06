# Ethernet Media Access Controller (EMAC) — EMAC Programming Model

<!-- source: 035_Ethernet_Media_Access_Controller_EMAC_EMAC_Programming_Model.pdf | original pages 2076–2098 -->

Table 30-108: Transfer Complete Interrupt Behavior (Continued)

|   S. No. |   Interrupt Mode (INTM) | Behavior of EMAC_DMA[n]_TX and EMAC_DMA[n]_RX                                                                                                                                                                                                                                                                                                                                                                    | Behavior of EMAC_STAT                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|----------|-------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|        3 |                       2 | In this mode, RI/TI interrupts are queued. These signals reflect the value of correspond- ing EMAC_DMA[n]_STAT.TI / EMAC_DMA[n]_STAT.TI bits when the corresponding interrupt enable is set. They are level signals and cleared by software by writing 1'b1 to the RI/TI status bits. How- ever, it is set again if another TI/RI event(s) is detected before the TI/RI bits are cleared for the previous event. | The RI/TI status bits are set whenever the Transfer Complete event is detected and gets reset whenever software driver clears those bits by writing 1. However, if another trans- fer complete event is detected before it is cleared (serviced) by the software, then EMAC automatically set these status bits again. However, the EMAC_STAT signal is not generated based on TI/RI. The EMAC_DMA[n]_STAT.NIS status bit is not asserted for RI/TI events. |

## EMAC Triggers

EMAC0 contains:

- 12 trigger requesters - 8 for receive DMA channels ( EMAC\_DMA[n]\_RX ) and 4 for MCGR DMA request (EMAC0\_MCGR[n])
- 20 trigger completers - 8 for receive DMA channels ( EMAC\_DMA[n]\_RX\_STOP ), 8 for transmit DMA channels ( EMAC\_DMA[n]\_TX ) and 4 for MCGR DMA Ack (EMAC0\_MCGR[n]\_ACK)

## EMAC1 contains:

- 1 trigger requester - for Rx DMA (EMAC1\_DMA0\_RX)
- 2 trigger completers - 1 for Rx DMA (EMAC1\_DMA0\_RX\_STOP) and 1 for Tx DMA (EMAC1\_DMA0\_TX)
- NOTE: · Each of the DMA trigger completers at EMAC interface is controller by 2 trigger signals from the TRU - 1 start trigger, 1 stop trigger. When the TRU gives a start trigger, the trigger completer at EMAC interface is asserted. When the TRU gives a stop trigger, the trigger completer at EMAC interface is de-asserted.
- An error condition occurs when the TRU generates a 2nd start trigger that occurs before a stop trigger or a 2nd stop trigger before a start trigger. This error condition does not affect the trigger signal at EMAC. It is just to check the status and is stored as a W1C status bit in ECO\_REG9 of the MISC REG block.

## EMAC Programming Model

This section provides the programming model and programming procedures for the Ethernet MAC peripheral.

## Initializing DMA

Use the following procedure to initialize DMA.

1. Perform a software reset by setting the EMAC\_DMA\_MODE.SWR bit. This action resets all of the EMAC internal registers and logic.
2. Wait for the completion of the reset process by polling the EMAC\_DMA\_MODE.SWR bit which is only cleared (automatically) after the reset operation completes.
3. Program the following fields to initialize the EMAC\_DMA\_SYSBMODE register:
- a. EMAC\_DMA\_SYSBMODE.AAL b. Fixed burst or undefined burst ( EMAC\_DMA\_SYSBMODE.FB ) c. Maximum SCB outstanding request limit ( EMAC\_DMA\_SYSBMODE.WR\_OSR\_LMT and EMAC\_DMA\_SYSBMODE.RD\_OSR\_LMT for both write and read)
- d. If fixed length value is enabled ( EMAC\_DMA\_SYSBMODE.FB bit is set), select the maximum burst length possible on the SCB bus ( EMAC\_DMA\_SYSBMODE.BLEN16 , EMAC\_DMA\_SYSBMODE.BLEN8 and EMAC\_DMA\_SYSBMODE.BLEN4 bits).
4. Create a descriptor list for transmit and receive. In addition, ensure that the descriptors are owned by DMA (set bit 31 of descriptor TDES3/RDES3). For more information about descriptors, see DMA Descriptors.
5. Program the transmit and receive ring length registers ( EMAC\_DMA[n]\_TXDSC\_RLEN and EMAC\_DMA[n]\_RXCTL2.RDRL ). The ring length programmed must be at least four.
6. Initialize the receive and transmit descriptor list address with the base address of the transmit and receive descriptor ( EMAC\_DMA[n]\_TXDSC\_ADDR and EMAC\_DMA[n]\_RXDSC\_ADDR ). Also, program transmit and receive tail pointer registers indicating to the DMA about the available descriptors ( EMAC\_DMA[n]\_TXDSC\_TLPTR and EMAC\_DMA[n]\_RXDSC\_TLPTR ).
7. Program the settings of the following registers for the parameters like maximum burst-length (PBL) initiated by DMA, descriptor skip lengths, OSP in case of TxDMA, RBSZ in case of RxDMA, and so on:
- a. EMAC\_DMA[n]\_CTL
8. Enable the interrupts by programming the EMAC\_DMA[n]\_IEN register.
9. Start the receive and transmit DMAs by setting the EMAC\_DMA[n]\_RXCTL.SR and the EMAC\_DMA[n]\_TXCTL.ST bits.
10. Repeat steps 4 to 9 for all the transmit DMA and receive DMA channels selected in the hardware.

```
b. EMAC_DMA[n]_TXCTL c. EMAC_DMA[n]_RXCTL
```

## Initializing the MTL Registers

The Transaction layer (MTL) registers must be initialized to establish the transmit and receive operating modes and commands. Use the following steps to initialize the MTL registers:

1. Program the transmit scheduling ( EMAC\_MTL\_OPMODE.SCHALG ) and receive arbitration algorithm ( EMAC\_MTL\_OPMODE.RAA ) fields to initialize the MTL operation in case of multiple transmit and receive queues.
2. Program the receive queue to DMA mapping in the EMAC\_RXQ\_DMA\_MAP0 and EMAC\_RXQ\_DMA\_MAP1 registers.
3. Program the following fields to initialize the mode of operation in the EMAC\_TQ0\_OPMODE register.
- a. Transmit store and forward ( EMAC\_TQ0\_OPMODE.TSF ) or transmit threshold control ( EMAC\_TQ0\_OPMODE.TTC ) in case of threshold mode.
- b. Transmit queue enable ( EMAC\_TQ0\_OPMODE.TXQEN ) to value 2'b10 to enable transmit queue0.
- c. Transmit queue size ( EMAC\_TQ0\_OPMODE.TQS ).
4. Program the following fields to initialize the mode of operation in the EMAC\_RQ0\_OPMODE register:
- a. Receive store and forward ( EMAC\_RQ0\_OPMODE.RSF ) or EMAC\_RQ0\_OPMODE.RTC in case of threshold mode.
- b. Flow control activation and de-activation thresholds for MTL receive FIFO ( EMAC\_RQ0\_OPMODE.RFA and EMAC\_RQ0\_OPMODE.RFD ).
- c. Error Packet and undersized good packet forwarding enable ( EMAC\_RQ0\_OPMODE.FEP and EMAC\_RQ0\_OPMODE.FUP ).
- d. Receive queue size ( EMAC\_RQ0\_OPMODE.RQS ).
5. Repeat previous two steps for all MTL transmit and receive queues selected in the configuration.

## Initializing the MAC

The MAC configuration registers establish the operating mode of the MAC. These registers must be initialized before initializing the DMA.

The following MAC initialization operations can be performed after DMA initialization. If the MAC initialization is completed before the DMA is configured, enable the MAC receiver (the last step in the following sequence) only after the DMA is active. Otherwise, received frames fill the receive FIFO and overflow.

1. Provide the required number of MAC address registers: EMAC\_ADDR0\_HI and EMAC\_ADDR0\_LO .
2. Program the following fields to set the appropriate filters for the incoming frames in the packet filter register:
- a. Receive all ( EMAC\_MACPKT\_FILT.RA )

```
b. Promiscuous mode ( EMAC_MACPKT_FILT.PR ) c. Hash or perfect filter ( EMAC_MACPKT_FILT.HPF ) d. Unicast, multicast, broadcast, and control frames filter settings
```

3. Program the following fields for proper flow control in the flow control register:
- a. Pause time and other pause frame control bits ( EMAC\_Q[i]\_TXFLOW\_CTL.PT ,
3. EMAC\_Q[i]\_TXFLOW\_CTL.PLT )
4. Program the EMAC\_MAC\_IEN register, as required, and if applicable, for the configuration.
5. Program the appropriate fields in the EMAC\_MAC\_CFG register. For example: Inter-packet gap ( EMAC\_MAC\_CFG.IPG ) while transmission and jabber disable ( EMAC\_MAC\_CFG.JD ).
6. Set bit 0 ( EMAC\_MAC\_CFG.RE ) and 1 ( EMAC\_MAC\_CFG.TE ) to start the MAC transmitter and receiver.
7. To support jumbo transmit/receive packets, configure the MAC configuration register as follows:
- a. Set the EMAC\_MAC\_CFG.JE field to 1
- b. Set the EMAC\_MAC\_CFG.JD and EMAC\_MAC\_CFG.WD fields to 1 to avoid giant packet error reporting
- c. Set the EMAC\_MAC\_CFG.GPSLCE field to 1
- d. Set the EMAC\_MAC\_EXT\_CFG.GPSL field to a value &gt; 9026
8. To support transmit/receive packets up to 16K, configure the MAC configuration register as follows:
- a. Set the EMAC\_MAC\_CFG.JD and EMAC\_MAC\_CFG.WD fields to 1 to avoid giant packet error reporting
- b. Set the EMAC\_MAC\_CFG.GPSLCE field to 1
- c. Set the EMAC\_MAC\_EXT\_CFG.GPSL field to 16383

```
b. Transmit flow control bits ( EMAC_Q[i]_TXFLOW_CTL.TFE ) c. Flow control busy ( EMAC_Q[i]_TXFLOW_CTL.FCB_BPA )
```

## Performing Normal Transmit and Receive Operations

During normal operation of the EMAC, normal and transmit interrupts are read, descriptors polled, the DMA is suspended (when it does not own descriptors), and the values of the current host transmitter or receiver descriptor pointers are read for debugging.

For normal operations, use the following steps:

1. For normal transmit and receive interrupts, read the interrupt status. Then, poll the descriptors, reading the status of the descriptor owned by the host (either transmit or receive).

2. Set appropriate values for the descriptors, ensuring that transmit and receive descriptors are owned by the DMA to resume the transmission and reception of data.

ADDITIONAL INFORMATION: When the descriptors are not DMA (or no descriptor is available), the DMA goes into the SUSPEND state. The transmission or reception can be resumed by freeing the descriptors and writing the descriptor tail pointer to the transmit/receive tail pointer registers ( EMAC\_DMA[n]\_TXDSC\_TLPTR and EMAC\_DMA[n]\_RXDSC\_TLPTR ).

3. The values of the current host transmitter or receiver descriptor address pointer can be read for the debug process ( EMAC\_DMA[n]\_TXDSC\_CUR and EMAC\_DMA[n]\_RXDSC\_CUR ).
4. The values of the current host transmit buffer address pointer and receive buffer address pointer can be read for the debug process ( EMAC\_DMA[n]\_TXBUF\_CUR and EMAC\_DMA[n]\_RXBUF\_CUR ).

## Stopping and Starting Transmission

Use the following procedure to stop and start EMAC transfers.

1. Disable the MAC transmitter and receiver by clearing the EMAC\_MAC\_CFG.RE and EMAC\_MAC\_CFG.TE bits.
2. Ensure that both the transmit and receive queues are empty ( EMAC\_TQ0\_DBG.TXQSTS and EMAC\_RQ0\_DBG.RXQSTS is 0).
3. Disable the receive DMA (whenapplicable), after ensuring that the data in the Rx FIFO is transferred to the system memory (by reading the appropriate bits of EMAC\_RQ0\_DBG register, EMAC\_RQ0\_DBG.PRXQ = 0 and EMAC\_RQ0\_DBG.RXQSTS = 00).
4. To restart the operation, first start the DMAs, and then enable the MAC transmitter and receiver. ADDITIONAL INFORMATION: Do not change the configuration (such as duplex mode, speed, port, or loop back) when the MAC is actively transmitting or receiving. These parameters are changed by software only when the MAC transmitter and receiver are not active. Similarly, do not change the DMA-related configuration when transmit and receive DMA are active.

## Switching to New Descriptor List in Receive DMA

Switching to a new descriptor list is different in the receive DMA compared to the transmit DMA. Switching to a new descriptor list is permitted when the receive DMA is in SUSPEND state, as clarified by the following points:

- Generally, receive DMA prepares the descriptors in advance.
- If the receive DMA goes to the SUSPEND state due to the descriptors not being available, a major failure occurs (software is not able to free the filled-up descriptors/buffers). If this issue is not rectified immediately, frames are lost because of a receive FIFO overflow. Therefore, the software is allowed to create a new descriptor list and program the receive DMA to start using it immediately, without going into the STOP state.

## Multi-Channel Multi-Queuing

## Transmit

Use the following procedure for a transmit operation.

1. Program the transmit queue size in the EMAC\_TQ0\_OPMODE.TQS bit field. Based on the value programmed in this bit field, the size of the queue is determined. In the transmit operation, the number of channels is equal to the number of the queues. Due to this reason, the channel-to-queue mapping is fixed.
2. For a queue to be used, enable the queue using the EMAC\_TQ0\_OPMODE.TXQEN bit. Enable the EMAC\_DMA[n]\_TXCTL.ST bit and the corresponding EMAC\_TQ0\_OPMODE.TXQEN bit.
3. Program the scheduling method in the EMAC\_MTL\_OPMODE.SCHALG bit field.
4. Program the EMAC\_TQ0\_QTMWGT register for DCB queues as per the selected algorithm. For CBS algorithms in AVB queues, the EMAC\_TQ[n]\_ETS\_CTL , EMAC\_TQ[n]\_SSCRDT , EMAC\_TQ[n]\_HICRDT and EMAC\_TQ[n]\_LOCRDT registers also need to be programmed as required.
5. If DCB is enabled and the PFC function is required, program the EMAC\_TXQ\_PRTY\_MAP0 register to assign a fixed priority to the queue. This priority is used for determining if the corresponding queue should stop transmitting a packet based on the received PFC packet.

## Receive

Use the following procedure for the receive operation.

1. Program the receive queue size in the EMAC\_RQ0\_OPMODE.RQS bit field. Based on the value programmed in this bit field, the size of the queue is determined.
2. Enable the receive queues 0 to 7 in the EMAC\_RXQ\_CTL0.RXQ0EN to EMAC\_RXQ\_CTL0.RXQ7EN bit fields for AV or DCB. In DMA configurations, enable the EMAC\_DMA[n]\_RXCTL.SR bit of statically or dynamically mapped EMAC\_DMA[n]\_RXCTL register and the corresponding EMAC\_RXQ\_CTL0.RXQ0EN to EMAC\_RXQ\_CTL0.RXQ7EN bits.
3. The MAC routes the receive packets to the receive queues based on following packet types:
- AV PTP packets - based on the programming of the EMAC\_RXQ\_CTL1.PTPQ bit.
- AV untagged control packets - based on the programming of the EMAC\_RXQ\_CTL1.AVCPQ bit.
- Data Center Bridging (DCB) related Link Layer Discovery Protocol (LLDP) packets. Program the EMAC\_RXQ\_CTL1.DCBCPQ bits to indicate to the MAC which queue receives the DCB packets.
- VLAN Tag Priority field in VLAN tagged packets - program the PSRQ7-0 bits in the EMAC\_RXQ\_CTL2 and EMAC\_RXQ\_CTL3 registers for the routing of tagged packets based on the USP (user priority) field of the received packets to the receive queues 0 to 7.
- The AV tagged control and data packets are also routed based on PSRQ field of the EMAC\_RXQ\_CTL2 and EMAC\_RXQ\_CTL3 registers. The priorities set in PSRQ7-0 should be unique.

4. If multiple receive DMA channels are enabled, the following programming is required for proper arbitration and mapping:
- a. Program the EMAC\_MTL\_OPMODE.RAA bit field to select the arbitration algorithm to decide which RxQ is read out from the RxFIFO memory.
- b. Program the EMAC\_RQ0\_CTL register to decide the weights and the packet arbitration for each RxQ.
- c. If static mapping is programmed in the EMAC\_RXQ\_DMA\_MAP0 / EMAC\_RXQ\_DMA\_MAP1 registers (the DDMACH bits are reset to 0), the RXQx2DMA and others need to be programmed to select the channel for which each queue is mapped.
- d. Set the EMAC\_RXQ\_DMA\_MAP0 DDMACH bits to select dynamic mapping of packets in each receive queue.
- e. In dynamic channel mapping, the routing of a packet to a specific receive DMA channel is decided by the value of the DCS field in the lowest MAC address register (for example EMAC\_ADDR0\_HI.DCS ).

## Recovering from DMA Channel Failure

When the DMA channel issues a bus error, follow these steps to recover from the failure.

## Recovering from the Receive DMA Channel Failure

Follow these steps a bus error occurs in the receive DMA channel:

1. Set the EMAC\_DMA[n]\_RXCTL.RPF bit to 1. This flushes all the packets one after the other. This step is optional. However, setting this bit prevents HOL (head-of-line) blocking in the receive queues when packets sent to the receive DMA are stopped due to bus error.
2. Re-program the specific registers of the DMA channel.
3. Start the DMA channel.

## Recovering from the Transmit DMA Channel Failure

Follow these steps if a bus error occurs in the transmit DMA channel:

1. Stop the specific DMA channel, even if it is in an active state.
2. Flush the corresponding MTL queue.
3. Re-program the specific registers of the DMA channel.
4. Start the DMA channel.

NOTE: Reprogramming the DMA channel registers might not always be successful in recovering from a bus error. If EMAC is not fully functional after reprogramming the DMA, as a workaround, issue a soft reset to recover from the bus error.

## Disabling the Receive Queue

Follow these steps to disable specific receive queues

1. Disable the receiver by setting the EMAC\_MAC\_CFG.RE bit to 0.
2. Read the debug register (ensure that the EMAC\_MAC\_DBG.RPESTS and EMAC\_MAC\_DBG.RFCFCSTS fields are 0) and the EMAC\_RQ0\_DBG registers of all the receive queues.
3. When the EMAC\_RQ0\_DBG registers are read as 0, disable the intended receive queue by programming the corresponding fields in the EMAC\_RXQ\_CTL0 register.
4. Enable the receiver by setting the EMAC\_MAC\_CFG.RE bit to 1.

## IEEE 1588 Time Stamping (PTP)

The following sections provide programming information for using for IEEE 1588 Time Stamping.

## Initialization Guidelines for System Time Generation

Programs can enable the time stamp feature by setting the EMAC\_TM\_CTL.TSENA bit. However, it is essential that the time stamp counter be initialized after this bit is set. Complete the following steps during EMAC initialization:

1. Mask the time stamp trigger interrupt by clearing the EMAC\_MAC\_IEN.TSIE bit.
2. Set the EMAC\_TM\_CTL.TSENA bit 0 to enable time stamping.
3. Program the EMAC\_TM\_SUBSEC register based on the PTP clock frequency.
4. If the Fine Correction method is required, program the EMAC\_TM\_ADDEND register and set the EMAC\_TM\_CTL.TSADDREG bit.
5. Poll the EMAC\_TM\_CTL register until the EMAC\_TM\_CTL.TSADDREG bit is cleared.
6. Program the EMAC\_TM\_CTL.TSCFUPDT bit to select the Fine Update method (if required).
7. Program the EMAC\_TM\_SECUPDT and EMAC\_TM\_NSECUPDT registers with the appropriate time value.
8. Set the EMAC\_TM\_CTL.TSINIT bit.

ADDITIONAL INFORMATION: The time stamp counter starts operation as soon as it is initialized with the value written in the time stamp update registers. If one-step time stamping is enabled:

- a. To enable one-step time stamping, program bit 27 of the TDES3 Context Descriptor.
- b. Program the EMAC\_TM\_INGASYM\_CORR and EMAC\_TM\_EGASYM\_CORR registers to update the correction field in PDelay\_Req PTP messages.
9. Enable the MAC receiver and transmitter for proper time stamping.

If a time stamp operation is disabled by clearing the EMAC\_TM\_CTL.TSENA bit, repeat all these steps to restart the time stamp operation.

## System Time Correction

The following sections provide programming information for system time correction.

## Coarse Correction Method

To synchronize or update the system time in one process (coarse correction method), complete the following steps:

1. Set the offset (positive or negative) in the timestamp update registers ( EMAC\_TM\_SECUPDT and EMAC\_TM\_NSECUPDT ).
2. Set the EMAC\_TM\_CTL.TSUPDT bit. The value in the time stamp update registers is added to or subtracted from the system time when the TSUPDT bit is cleared.

## Fine Correction Method

To synchronize or update the system time to reduce system-time jitter (fine correction method), complete the following steps:

1. With the help of the algorithm explained in Fine Correction Method, calculate the rate by which to make the system time increments slower or faster.
2. Update the EMAC\_TM\_ADDEND register with the new value and set the EMAC\_TM\_CTL.TSADDREG bit.
3. Wait for the time for the new value of the Addend register to be active. You can do this by enabling the Timestamp Trigger interrupt after the system time reaches the target value.
4. Program the required target time in the EMAC\_PPS[n]\_TGTM\_SEC and EMAC\_PPS[n]\_TGTM\_NSEC registers.
5. Enable the time stamp interrupt using the EMAC\_MAC\_IEN.TSIE bit.
6. Set bit 4 in the EMAC\_TM\_CTL register.
7. When this trigger causes an interrupt, read the EMAC\_MAC\_ISTAT register.
8. Reprogram the EMAC\_MAC\_ISTAT register with the old value and set the EMAC\_TM\_CTL.TSADDREG bit again.

## PTP Offloading

The following sections provide programming information for using PTP offload.

## Automatic Periodic Generation of PTP Sync Messages

Follow these steps to enable automatic periodic generation of PTP sync messages:

1. Program the EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA , and EMAC\_TM\_CTL.TSEVNTENA bit fields to zero, one, and one respectively, to configure the node as ordinary or boundary requester (one, one, and one for transparent requester).
2. Program the EMAC\_PTO\_CTL.PTOEN bit and EMAC\_PTO\_CTL.DN bit field to enable the PTP offload feature and domain number to send in egress PTP Sync message.
3. Program the EMAC\_PTO\_CTL.ASYNCEN bit to enable periodic generation of PTP Sync messages.
4. Program the 80-bit Source Port Identity in the EMAC\_SRCPRT\_IDNTY0 through EMAC\_SRCPRT\_IDNTY2 registers to send in egress PTP Sync message.
5. Program the EMAC\_LOG\_MSG\_INTVL.LSI bit field to program the periodicity of the PTP Sync messages. For example, value of 1 corresponds to 21 which translates to PTP Sync message every 2 seconds and value of 0xFF (twos complement of -1) corresponds to 2-1 which translates to PTP Sync message every 0.536 seconds.
6. Program the EMAC\_MAC\_IEN.TSIE bit to enable generation of time stamp interrupt.
7. Wait for a EMAC\_STAT or EMAC\_MAC interrupt generated when the EMAC\_TM\_STAT.TXTSSIS bit is set, which indicates that the time stamp for PTP Sync message is captured in the EMAC\_TXTMSTAT\_SEC and EMAC\_TXTMSTAT\_NSEC registers.

## Periodic Generation of PTP Pdelay\_Req Messages

Follow these steps to enable automatic periodic generation of PTP Pdelay\_Req messages.

1. Program the EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA , and EMAC\_TM\_CTL.TSEVNTENA bit fields to 1, 0, and 1 respectively to configure the node as transparent completer (1, 1, and 1 for transparent requester OR 3, X, and X for Peer-to-Peer T ransparent).
2. Program the EMAC\_PTO\_CTL.PTOEN bit and EMAC\_PTO\_CTL.DN bit field to enable PTP Offload feature and domain Number to send in egress PTP Pdelay\_Req message.
3. Program the EMAC\_PTO\_CTL.APDREQEN bit to enable periodic generation of PTP Pdelay\_Req messages.
4. Program the 80-bit Source Port Identity in the EMAC\_SRCPRT\_IDNTY0 through EMAC\_SRCPRT\_IDNTY2 registers to send in egress PTP Pdelay\_Req message.
5. Program the EMAC\_LOG\_MSG\_INTVL.LMPDRI bit field to program the periodicity of the PTP Pde-lay\_Req messages. For example, value of 1 corresponds to 21 which translates to PTP Pdelay\_Req message every 2 seconds and value of 0xFF (twos complement of -1) corresponds to 2-1 which translates to PTP Pde-lay\_Req message every 0.536 seconds.
6. Program the EMAC\_MAC\_IEN.TSIE bit to enable generation of Timestamp Interrupt.
7. Wait for EMAC\_STAT or EMAC\_MAC interrupt generated on setting of the EMAC\_TM\_STAT.TXTSSIS bit is set, which indicates that the time stamp for PTP Sync message is captured in the EMAC\_TXTMSTAT\_SEC and EMAC\_TXTMSTAT\_NSEC registers.

## Response Generation - Ordinary or Boundary Requester Mode

Follow these steps to enable generation of PTP response messages for Ordinary or Boundary Requester mode (periodic PTP sync messages generated and PTP Delay\_Resp message generated in response to PTP Delay\_Req message).

1. Program the EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA and EMAC\_TM\_CTL.TSEVNTENA bit fields to zero, one, and one respectively.
2. Program the EMAC\_PTO\_CTL.PTOEN bit and EMAC\_PTO\_CTL.DN field to enable the PTP offload feature and domain number to match with ingress PTP Delay\_Req message and send in egress PTP Delay\_Resp message.
3. Program the 80-bit source port identity in the EMAC\_SRCPRT\_IDNTY0 through EMAC\_SRCPRT\_IDNTY2 registers to match with ingress PTP Delay\_Req message and send in egress PTP Delay\_Resp message.
4. Program the EMAC\_LOG\_MSG\_INTVL.DRSYNCR and EMAC\_LOG\_MSG\_INTVL.LSI bit fields. The sum of both these fields is updated in the logMinDelayReqInterval field of the PTP Delay\_Resp message.

## Response Generation - Ordinary or Boundary Completer Mode

Follow these steps to enable generation of PTP response messages for Ordinary or Boundary completer mode (PTP Delay\_Req message generated in response to PTP Sync message).

1. Program the EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA and EMAC\_TM\_CTL.TSEVNTENA bit fields to zero, zero, and one respectively.
2. Program the EMAC\_PTO\_CTL.PTOEN bit and EMAC\_PTO\_CTL.DN bit field to enable PTP Offload feature and domain Number to match with ingress PTP Sync message and send in egress PTP Delay\_Req message.
3. Program the 80-bit source port identity in the EMAC\_SRCPRT\_IDNTY0 through EMAC\_SRCPRT\_IDNTY2 registers to match with ingress PTP Sync message and send in egress PTP De-lay\_Req message.
4. Program the EMAC\_LOG\_MSG\_INTVL.DRSYNCR bit field to indicate one PTP Delay\_Req message is generated in response to how many received PTP Sync messages.

## Response Generation - Transparent Completer Mode

Follow these steps to enable generation of PTP response messages for transparent completer mode (PTP Delay\_Req message generated in response to PTP Sync message, PTP Pdelay\_Resp message generated in response to PTP Pde-lay\_Req message and Periodic PTP Pdelay\_Req messages generated).

1. Program the EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA and EMAC\_TM\_CTL.TSEVNTENA bit fields to one, zero, and one respectively.

2. Program the EMAC\_PTO\_CTL.PTOEN bit and EMAC\_PTO\_CTL.DN bit field to enable the PTP offload feature and domain number to match with ingress PTP Sync or Pdelay\_Req message and send in egress PTP Delay\_Req or Pdelay\_Resp or Pdelay\_Req message.
3. Program the 80-bit source port identity in the EMAC\_SRCPRT\_IDNTY0 through EMAC\_SRCPRT\_IDNTY2 registers to match with ingress PTP Sync or Pdelay\_Req message and send in egress PTP Delay\_Req or Pdelay\_Resp or Pdelay\_Req message.
4. Program the EMAC\_LOG\_MSG\_INTVL.DRSYNCR bit field and EMAC\_LOG\_MSG\_INTVL.LMPDRI bit fields to indicate one PTP Delay\_Req message is generated in response to how many received PTP Sync messages and periodicity of the PTP Pdelay\_Req messages.
5. Program the EMAC\_MAC\_IEN.TSIE bit to enable generation of the timestamp interrupt.
6. Wait for the EMAC\_STAT or EMAC\_MAC interrupt that is generated when the EMAC\_TM\_STAT.TXTSSIS bit is set. This indicates that the timestamp for PTP Sync message is captured in the EMAC\_TXTMSTAT\_SEC and EMAC\_TXTMSTAT\_NSEC registers for egress PTP Pdelay\_Req and Pdelay\_Resp messages.

## Response Generation - Transparent Requester Mode

Follow these steps to enable generation of PTP response messages for transparent requester mode (PTP Delay\_Resp message generated in response to PTP Delay\_Req message, PTP Pdelay\_Resp message generated in response to PTP Pdelay\_Req message and Periodic PTP Pdelay\_Req or Sync messages generated).

1. Program the EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA and EMAC\_TM\_CTL.TSEVNTENA bit fields to one, one, and 1one respectively.
2. Program the EMAC\_PTO\_CTL.PTOEN bit and EMAC\_PTO\_CTL.DN bit field to enable the PTP offload feature and domain number to match with ingress PTP Delay\_Req or Pdelay\_Req message and send in egress PTP Delay\_Resp or Pdelay\_Resp or Pdelay\_Req or Sync message.
3. Program the 80-bit source port identity in the EMAC\_SRCPRT\_IDNTY0 through EMAC\_SRCPRT\_IDNTY2 registers to match with ingress PTP Delay\_Req or Pdelay\_Req message and send in egress PTP Delay\_Resp or Pdelay\_Resp or Pdelay\_Req or Sync message.
4. Program the EMAC\_LOG\_MSG\_INTVL.DRSYNCR , EMAC\_LOG\_MSG\_INTVL.LSI and EMAC\_LOG\_MSG\_INTVL.LMPDRI bit fields. The sum of EMAC\_LOG\_MSG\_INTVL.DRSYNCR and EMAC\_LOG\_MSG\_INTVL.LSI is updated in logMinDelayReqInterval field of PTP Delay\_Resp message and periodicity of the PTP Sync or Pdelay\_Req messages.
5. Program the EMAC\_MAC\_IEN.TSIE bit to enable generation of the timestamp interrupt.
6. Wait for the EMAC\_STAT or EMAC\_MAC interrupt that is generated when the EMAC\_TM\_STAT.TXTSSIS bit is set. This indicates that the timestamp for PTP Sync message is captured in the EMAC\_TXTMSTAT\_SEC and EMAC\_TXTMSTAT\_NSEC registers for egress PTP Sync, Pdelay\_Req and Pdelay\_Resp messages.

## Response Generation - Peer-to-Peer Transparent Mode

Follow these steps to enable generation of PTP response messages for Peer-to-Peer T ransparent mode (PTP Pde-lay\_Resp message generated in response to PTP Pdelay\_Req message and Periodic PTP Pdelay\_Req messages generated).

1. Program the EMAC\_TM\_CTL.SNAPTYPSEL , EMAC\_TM\_CTL.TSMSTRENA and EMAC\_TM\_CTL.TSEVNTENA bit fields to 3, X, and X respectively.
2. Program the EMAC\_PTO\_CTL.PTOEN bit and EMAC\_PTO\_CTL.DN bit field to enable the PTP offload feature and domain number to match with ingress PTP Pdelay\_Req message and send in egress PTP Pde-lay\_Resp message.
3. Program the 80-bit source port identity in the EMAC\_SRCPRT\_IDNTY0 through EMAC\_SRCPRT\_IDNTY2 registers to match with ingress PTP Pdelay\_Req message and send in egress PTP Pdelay\_Resp message.
4. Program the EMAC\_LOG\_MSG\_INTVL.LMPDRI bit field to indicate periodicity of the PTP Pdelay\_Req messages.
5. Program the EMAC\_MAC\_IEN.TSIE bit to enable generation of the timestamp interrupt.
6. Wait for the EMAC\_STAT or EMAC\_MAC interrupt that is generated when the EMAC\_TM\_STAT.TXTSSIS bit is set. This indicates that the timestamp for PTP Sync message is captured in the EMAC\_TXTMSTAT\_SEC and EMAC\_TXTMSTAT\_NSEC registers for egress PTP Pdelay\_Req and Pdelay\_Resp messages.

## Using the Audio Video Feature

The following sections provide procedures to program audio video features.

## Initializing the DMA in Audio Video Feature

1. Provide a software reset to reset all internal registers and logic ( EMAC\_DMA\_MODE.SWR bit).
2. Wait for the completion of the reset process. Poll the EMAC\_DMA\_MODE.SWR bit, which is cleared only after the reset operation is completed.
3. Program the fields to initialize the DMA register by setting the values in the EMAC\_DMA\_MODE register.
4. Create a proper descriptor list for transmit and receive. In addition, ensure that the DMA owns the transmit and receive descriptors.
5. ADDITIONAL INFORMATION: When OSF mode is used, at least two transmit descriptors are required. For more information about descriptors, see the DMA Descriptors section.
5. Ensure that the software creates three or more different transmit or receive descriptors in the list before reusing any of the descriptors.

6. Program the transmit and receive ring length registers ( EMAC\_DMA[n]\_TXDSC\_RLEN and EMAC\_DMA[n]\_RXCTL2.RDRL ). The programmed ring length must be at least 4.
7. Initialize receive and transmit descriptor list address with the base address of the transmit and receive descriptor ( EMAC\_DMA[n]\_TXDSC\_ADDR and EMAC\_DMA[n]\_RXDSC\_ADDR ). In addition, program the transmit and receive tail pointer registers that indicate to the DMA the available descriptors ( EMAC\_DMA[n]\_TXDSC\_TLPTR and EMAC\_DMA[n]\_RXDSC\_TLPTR ).
8. Program the following fields to initialize the mode of operation in the EMAC\_TQ0\_OPMODE register:
- a. Transmit store and forward ( EMAC\_TQ0\_OPMODE.TSF )
- b. Transmit threshold control ( EMAC\_TQ0\_OPMODE.TTC )
- c. Transmit queue enable ( EMAC\_TQ0\_OPMODE.TXQEN ) to value 2'b10 to enable transmit queue0
- d. Transmit queue size ( EMAC\_TQ0\_OPMODE.TQS )
9. Enable the interrupts by programming the EMAC\_DMA[n]\_IEN register.
10. Repeat steps 4 through 9 for all additional channels of AV feature.
11. Program the EMAC\_TQ[n]\_ETS\_CTL register, idleSlope ( EMAC\_TQ0\_QTMWGT ), sendSlope ( EMAC\_TQ[n]\_SSCRDT , hiCredit ( EMAC\_TQ[n]\_HICRDT ), and loCredit ( EMAC\_TQ[n]\_LOCRDT ) registers of the AVQueues.
12. Start the receive and transmit DMA by setting the EMAC\_DMA[n]\_TXCTL.ST bit and the EMAC\_DMA[n]\_RXCTL.SR bit.

## Enabling Slot Number Checking

The program enables the ability to report the average bits that are transmitted in a slot. The EMAC\_TQ0\_ETS\_STAT register of the additional AV channels provides information about the average bits that are transmitted in a slot. The software can asynchronously read this register to retrieve information about the average bits transmitted per slot. Complete the following steps to enable average bits per slot reporting.

1. Enable time stamping by following the steps described in the 'Initialization Guidelines for System Time Generation."
2. Program the EMAC\_TQ[n]\_ETS\_CTL.SLC bits [6:4] of a channel with the number of slots over which the average transmitted bits per slot need to be computed.
3. ADDITIONAL INFORMATION: The frequency of this interrupt depends on the value programmed in this step. For example, when the value 0 is used in the EMAC\_TQ[n]\_ETS\_CTL.SLC field, the interrupt is generated at every 125 microseconds. When not required, disable this interrupt to stop the interrupt flooding.
3. Enable the EMAC\_Q0\_INT\_CTLSTAT.ABPSIE bit of a channel to generate the average bits per slot interrupt.
4. Read the EMAC\_TQ0\_ETS\_STAT.ABS bits [16:0] of a channel on each interrupt.

ADDITIONAL INFORMATION: The software can read the EMAC\_TQ0\_ETS\_STAT.ABS bits in polling mode even if the EMAC\_Q0\_INT\_CTLSTAT.ABPSIE bit is not enabled. When high, bit 1 ( EMAC\_Q0\_INT\_CTLSTAT.ABPSIS ) indicates that a new value is updated in the ABS field.

## Disabling Flow Control for AV Enabled Queues

## Receive Flow

Program the EMAC\_RQ0\_OPMODE.EHFC (enable hardware flow control) bit of the corresponding receive queue's operation mode register to 0.

## Transmit Flow

Program the EMAC\_TXQ\_PRTY\_MAP0.PSTQ0 through EMAC\_TXQ\_PRTY\_MAP1.PSTQ7 bit fields corresponding to the AV enabled transmit queue (using the EMAC\_TXQ\_PRTY\_MAP0 and EMAC\_TXQ\_PRTY\_MAP1 registers) to 0.

## Programming Energy Efficient Ethernet

The following sections provide procedures to program energy efficient Ethernet.

## Entering and Exiting the Transmit LPI Mode

EEE enables the IEEE 802.3 Media Access Control (MAC) sub layer along with a family of Physical layers to oper- ate in the Low-Power Idle (LPI) mode. In the transmit path, the software must set the EMAC\_LPI\_CTLSTAT.LPIEN bit to indicate to the MAC to stop transmission and initiate the LPI protocol.

Follow these steps for entering/existing the transmit LPI Mode:

1. Read the PHY register through the MDIO interface, check if the remote end has the EEE capability, and then negotiate the timer values.
2. Program the PHY registers through the MDIO interface (including the RX\_CLK\_stoppable bit that indicates to the PHY whether to stop receive clock in LPI mode.)
3. Program the EMAC\_LPI\_TMRSCTL.LST and EMAC\_LPI\_TMRSCTL.TWT bit fields.
4. Read the link status of the PHY chip by using the MDIO interface and update the EMAC\_LPI\_CTLSTAT.PLS bit accordingly. This update should be done whenever the link status in the PHY chip changes.
5. Program the EMAC\_ONEUS\_TIC\_CNT register as per the frequency of SCLK0.
6. Program the EMAC\_LPI\_ENTR\_TMR.LPIET bit field with the IDLE time for which the MAC should wait before entering the LPI state on its own.
7. Set the EMAC\_LPI\_CTLSTAT.LPIATE and EMAC\_LPI\_CTLSTAT.LPITXA to enable the auto-entry into LPI and auto-exit of MAC from LPI state.
8. Set the EMAC\_LPI\_CTLSTAT.LPIEN bit to make the MAC transmitter enter the LPI state.

The MAC enters the LPI mode after completing all scheduled packets and remains IDLE for the time indicated by the EMAC\_LPI\_ENTR\_TMR.LPIET bit field. It sets the EMAC\_LPI\_ENTR\_TMR.LPIET bit [0] after entry to the LPI state.

When a packet is scheduled for transmission (when the transmit DMA comes out of the IDLE state), the MAC transmitter exits the LPI state automatically. It waits for TWT time before setting the EMAC\_LPI\_CTLSTAT.TLPIEX interrupt status bit and then resumes the packet transmission.

9. The MAC transmitter re-enters the LPI state if it remains IDLE for EMAC\_LPI\_ENTR\_TMR.LPIET time and sets the EMAC\_LPI\_CTLSTAT.LPIEN bit and the entry-exit cycle continues.
10. Reset the EMAC\_LPI\_CTLSTAT.TLPIEX bit in case the application wants to over-ride the auto-entry/exit modes and make the MAC transmitter exit the LPI state directly.

To ensure MAC enter the LPI state only after completion of transmission of all the queued frames in the Tx FIFO, set the EMAC\_LPI\_CTLSTAT.LPITXA bit [19].

## Programming Flexible Pulse-Per-Second Output

The following sections provide procedures to program Flexible Pulse-Per-Second Output.

## Generating Single Pulse on PPS

To program single pulse on PPS, use the following procedure.

1. Program 11 or 10 (for interrupt) using the EMAC\_PPS\_CTL.TRGTMODSEL0 bits [6:5]. This instructs the MAC to use the target time registers ( EMAC\_PPS[n]\_TGTM\_SEC and EMAC\_PPS[n]\_TGTM\_NSEC ) for the start time of PPS signal output.
2. Program the start time value in the target time registers ( EMAC\_PPS[n]\_TGTM\_SEC and EMAC\_PPS[n]\_TGTM\_NSEC ).
3. Program the width of the PPS signal output in the EMAC\_PPS[n]\_WID register.
4. Program the EMAC\_PPS\_CTL.PPSCMD1 bits [3:0] to 0001. This instructs the MAC to generate a single pulse on the PPS signal output at the time programmed in the target time registers.

## Generating Next Pulse on PPS

When the PPSCMD is executed (PPSCMD bits = 0), the program can cancel the pulse generation by giving the cancel start command (PPSCMD = 0011) before the programmed start time elapses. The behavior of the next pulse can be programmed in advance. To program the next pulse, follow this procedure.

1. Program the start time for the next pulse in the target yime registers. This time should be more than the time at which the falling edge occurs for the previous pulse.
2. Program the width of the next PPS signal output in the EMAC\_PPS[n]\_WID register.

3. Program the EMAC\_PPS\_CTL.PPSCTRL\_PPSCMD bits [3:0], PPSCMD to generate a single pulse after the time at which the previous pulse is de-asserted. This instructs the MAC to generate single pulse on the PPS signal output, at the time programmed in target time registers. When this command executes before the previous pulse becomes low, then the new command overwrites the previous command and the EMAC may generate only one extended pulse.

## Generating a Pulse Train on PPS

The EMAC\_PPS\_CTL.TRGTMODSEL0 bits program the target time registers to do any one of the following:

- Generate only interrupts.
- Generate interrupts and the PPS start and stop time.
- Generate only PPS start and stop time.

Complete the following steps to program the target time registers to generate only interrupt event.

1. Program the EMAC\_PPS\_CTL.TRGTMODSEL0 bits = 00 (for interrupt). This instructs the MAC to use the target time registers for target time interrupt.
2. Program a target time value in the target time registers. This instructs the MAC to generate an interrupt when the target time elapses. If the mode select bits [6:5] are changed (for example, to control the PPS), then the interrupt generation is over-written with the new mode and new programmed target time register value.

The TSTARGTERR[n] interrupt is generated when the programmed target time is smaller (that is, time in the past) compared to system time in the EMAC\_TM\_SEC and EMAC\_TM\_NSEC registers. Therefore, to avoid unwanted TSTARGTERR[n] interrupt, the correct writing order is as follows:

```
1. EMAC_PPS[n]_TGTM_NSEC 2. EMAC_PPS[n]_TGTM_SEC 3. EMAC_PPS[n]_INTVL 4. EMAC_PPS[n]_WID 5. EMAC_PPS_CTL.PPSCTRL_PPSCMD and EMAC_PPS_CTL.PPSEN0
```

## Programming TSO

The TCP Segmentation Offload (TSO) engine is used to offload the TCP segmentation functions to the hardware. To program the TSO, set the TSE bit to enable TCP packet segmentation, and program descriptor fields to enable TSO for the current packet. Use the following steps to program TSO.

1. Program the EMAC\_DMA[n]\_TXCTL.TSE bit to enable TCP packet segmentation in that DMA channel.
2. In addition to the normal transfer descriptor setting, the following descriptor fields must be programmed to enable TSO for the current packet:

- a. Enable the TSE in bit 18 of the TDES3 register
- b. Program the length of the un-segmented TCP/IP packet payload in bits [17:0] of the TDES3 register and the TCP header in bits [22:19] of TDES3.
- c. Program the maximum size of the segment using the EMAC\_DMA[n]\_CTL.MSS bits or MSS in the context descriptor. If the MSS field is programmed in both the DMA channel control register and in the context descriptor, the latest software programmed sequence is considered.
3. The header of the unsegmented TCP/IP packet should be in buffer 1 of the first descriptor and this buffer must not hold any payload bytes. The payload is allocated to buffer 2 and the buffers of the subsequent descriptors.

ADDITIONAL INFORMATION: If the TSE bit is enabled in the TDES3 register for a non-TCP-IP packet, the result is unpredictable.

## Programming UFO

EMAC supports fragmentation of UDP packets into smaller IPv4 fragments. Enable the IP fragmentation by programming the EMAC\_DMA[n]\_TXCTL.TSE\_MODE bits [14:13] as follows:

- Set to 2'b01 to enable fragmentation of UDP over IPv4 with checksum
- Set to 2'b10 to enable fragmentation of UDP over IPv4 without checksum

NOTE: In both the cases, if the EMAC\_DMA[n]\_TXCTL.TSE bit [12] is set to 1 for a TCP packet, DMA enables TCP segmentation; fragmentation is not enabled, because fragmentation is supported only for UDP packets.

## Software Guidelines

When the software creates Transmit Descriptor of the UDP/IPv4 packet that is to be fragmented, the software must:

1. Set the desired fragment size (MFS) in the MSS field of TDES2 of the Context Descriptor. The MFS must be set only once, unless the MFS needs to be changed (for TCP packet targeted for segmentation). The MFS must be a multiple of 8 bytes for IP fragmentation.
2. Set the TSE bit of TDES3 of normal descriptor.
3. Program the THL = 2 in TDES3 of normal descriptor.
4. Program the correct value in the TPL field of TDES3 of normal descriptor. This value should be equal to length of UDP Header + UDP payload.
5. NOTE: · Enable fragmentation only for IPv4; not for IPv6.
- Set the DF flag (do not fragment) in the IPv4 header packet to 0.
- Set the fragmentation offset in the IPv4 header packet to 0.

When the input packet is UDP over IPv6, do not enable IP fragmentation. Set the TSE bit to 0 in the TDES3 of normal descriptor and let the software perform the fragmentation.

## Programming Header Payload Split Receive

The Header-Payload Split support can be enabled by

1. Setting the EMAC\_DMA[n]\_CTL.SPH bit field.
2. Selecting the Header-Payload split option by setting the EMAC\_SPLM\_OFST\_CFG.SPLM bit field.

The DMA can process the header and payload of the received packets separately.

- NOTE: · In case of L3/L4 header-payload split, if the IPv4 header length or IPv6 payload length fields are corrupt, the split might not always happen at the correct boundary.
- When the EMAC\_DMA[n]\_CTL.SPH bit field is set to 1, buffer 1 of only the first descriptor is used with buffer size indicated by the EMAC\_MAC\_EXT\_CFG.HDSMS bit field.
- Buffer 2 of the first descriptor and subsequent descriptors are used with buffer size indicated by the EMAC\_DMA[n]\_RXCTL.RBSZ\_13\_Y / EMAC\_DMA[n]\_RXCTL.RBSZ\_X\_0 bit fields.
- When the HL field of the first descriptor (RDES2) is zero, it indicates packet is not split into header and payload.
- When the HL field of the first descriptor (RDES2) is non-zero, it indicates that the packet is split into header and payload.

## Programming VLAN Filtering on Receive

Use the following steps to program VLAN filtering on receive:

1. Program the EMAC\_VLANTAG\_CTL register as follows to select the filtering method.
- a. EMAC\_VLANTAG\_CTL.ETV - Enable 12-bit VLAN tag comparison or 16-bit VLAN tag comparison.
- b. EMAC\_VLANTAG\_CTL.VTHM - VLAN tag hash table match enable.
- c. EMAC\_VLANTAG\_CTL.ERIVLT - Enable inner VLAN tag or outer VLAN tag (to enable the inner or outer VLAN tag filtering, enable double VLAN processing by setting the EMAC\_VLANTAG\_CTL.EDVLP bit)
- d. EMAC\_VLANTAG\_CTL.ERSVLM - Enable receive S-VLAN Match or C-VLAN match (for S-VLAN processing to be enabled, set EMAC\_VLANTAG\_CTL.ESVL ESVL)
- e. EMAC\_VLANTAG\_CTL.DOVLTC - Ignores VLAN type for tag match
- f. EMAC\_VLANTAG\_CTL.VTIM - Enable VLAN tag inverse match instead of the normal VLAN tag matching

2. Program the EMAC\_VLANTAG\_CTL.ETV bit field to select 12-bit or 16-bit VLAN tag.
3. If hash filtering of VLAN tag is enabled, program the EMAC\_VLAN\_HASHTBL register.

ADDITIONAL INFORMATION: When the EMAC\_VLANTAG\_CTL.ETV bit is reset, upper 4 bits of the calculated CRC-32 of VLAN tag are inverted and used to index the content of the EMAC\_VLAN\_HASHTBL register. When the ETV bit is set, upper 4 bits of calculated CRC-32 of VLAN tag are used to index the content of the EMAC\_VLAN\_HASHTBL register. For example, when the EMAC\_VLANTAG\_CTL.ETV bit is set, a hash value of 4b'1000 selects Bit 8 of the VLAN hash table. When the ETV bit is reset a hash value of 4'b1000 selects bit 7 of the VLAN hash table.

## Programming Extended VLAN Filtering and Routing on Receive

For the indirect access of the per VLAN tag registers, follow these steps:

Write

1. Write the required data into the EMAC\_VLANTAG\_DAT register.
2. Program the EMAC\_VLANTAG\_CTL.OFS bit field with the required filter register's offset and command type to the EMAC\_VLANTAG\_CTL.CT bit field. For a write command, set this bit to 0.
3. Write 1 to the EMAC\_VLANTAG\_CTL.OB bit field and wait until the bit is reset before performing the next write. This is to ensure that the appropriate VLAN tag filter register has been programmed.

## Read

1. Program the EMAC\_VLANTAG\_CTL.OFS bit field with the required register's offset and command type to the EMAC\_VLANTAG\_CTL.CT bit field. For a read command, set this bit to 1.
2. Write 1 to the EMAC\_VLANTAG\_CTL.OB bit field and wait until the bit is reset. The appropriate VLAN tag filter register's value is available in the EMAC\_VLANTAG\_DAT register.

## Programming Queue/Channel Based VLAN Inclusion Registers

The following sections provide the procedures for reading and writing the Queue/Channel based VLAN inclusion register.

## Writing to the Indirect Registers

To write to the queue/channel based VLAN inclusion register, use the following steps.

1. Set the EMAC\_VLAN\_INCL.CBTI bit to 1, to enable queue/channel based VLAN tag insertion on all transmitted packets.

ADDITIONAL INFORMATION: This bit must be set before any indirect access to the queue/channel specific MAC\_VLAN\_INCL[n] register.

2. Set the EMAC\_VLAN\_INCL.RDWR bit field to 1, to indicate a write access.

3. Program the VLAN tag and VLAN type to be inserted in packets from a particular queue/channel using the EMAC\_VLAN\_INCL.VLT and EMAC\_VLAN\_INCL.CSVL bit fields. The corresponding offset address in the EMAC\_VLAN\_INCL.ADDR field (0 for queue/channel 0, 1 for queue/channel 1, and so on) must be set.
4. Set the EMAC\_VLAN\_INCL.RDWR bit to 1 to indicate write access. ADDITIONAL INFORMATION: The write to byte 0 (byte 3 in Big Endian mode) of the EMAC\_VLAN\_INCL register initiates access to indirect access MAC\_VLAN\_INCL[n] register.
5. The EMAC\_VLAN\_INCL.BUSY bit is set by EMAC to indicate the progress of access to indirect access MAC\_VLAN\_INCL[n] register. On completion of the access, the EMAC\_VLAN\_INCL.BUSY bit is cleared. The application must not attempt subsequent accesses to the MAC\_VLAN\_INCL[n] register when this bit is one.
6. Repeat step 3 through 5 to insert VLAN tag and VLAN type in packets from the remaining queues/channels. The application must ensure that the required VLAN tag and VLAN type for all the queues/channels are programmed; otherwise unintended VLAN tag and VLAN type might be inserted.

## Reading the Indirect Registers

To read the queue/channel based VLAN inclusion register, complete the following steps:

1. Set the EMAC\_VLAN\_INCL.CBTI bit to 1, to enable queue/channel based VLAN tag insertion on all the transmitted packets. This bit must be set before indirect access to the queue/channel specific MAC\_VLAN\_INCL[n] register.
2. Program the read offset address in the EMAC\_VLAN\_INCL.ADDR ADDR field (0 for queue/channel 0, 1 for queue/channel 1, and so on). Set the EMAC\_VLAN\_INCL.RDWR bit to 0, to indicate read access.
3. The EMAC\_VLAN\_INCL.BUSY field is set by EMAC to indicate the progress of access to the indirect access MAC\_VLAN\_INCL[n] register. On completion of the access, the BUSY field is cleared. The bits [15:0] and bit [19] of the EMAC\_VLAN\_INCL register contains the VLAN tag and VLAN type respectively, from the corresponding queue/channel/offset address.
4. Repeat step 2 and step 3 to read VLAN tag and VLAN type from the remaining queues/channels.

## Programming EST

Program the gate control values and time intervals in the Software Owned Gate Control List (SWOL) along with the other EST related registers to appropriate values. The following sub-sections provide step by step details for programming the GCL and the other EST related registers.

## GCL and GCL Linked Registers

Follow these steps to program the Gate Control List (GCL) and the four other registers implemented per GCL.

The GCL and the four other GCL-linked registers are accessed through indirect addressing using the EMAC\_EST\_GCL\_CTL and EMAC\_EST\_GCL\_DAT registers. The EMAC\_EST\_STAT.SWOL bit field indicates if GCL0 or GCL1 is owned by software.

1. Write the 32-bit write data to EMAC\_EST\_GCL\_DAT register. Then program the EMAC\_EST\_GCL\_CTL register to write the write address and other control information.

ADDITIONAL INFORMATION: In the EMAC\_EST\_GCL\_DAT register, write data consists of up to 8 bits (configurable) of gate controls and up to 24 bits (configurable) of time interval. Gate close is indicated by programming a 0 and gate open is indicated by programming a 1. For a 4-TC and 20-bit time interval configuration, the data width is 24 bits and the remaining 8 bits are reserved/read-only. Write the data in the following format: {8'h0, TC3, TC2, TC1, TC0, 20-bit Time Interval} where TCx = 0 or 1.

2. Program the EMAC\_EST\_GCL\_CTL.SRWO bit to 1 (to start a write op) and program the address and R/W fields appropriately.
3. Poll and check for the EMAC\_EST\_GCL\_CTL.SRWO bit to be cleared by hardware to indicate the completion of the previous operation before initiating a new R/W operation using the same indirect addressing mode.
4. Repeat steps 3, 4, 5 until the programming of the GCL is completed.
5. Using the indirect addressing method, program the BTR, CTR, TER and LLR registers. Set the EMAC\_EST\_GCL\_CTL.GCRR bit appropriately. This bit interprets the address field as belonging to these registers (instead of the GCL).
6. After programming of the GCL and the related registers, program the EMAC\_EST\_CTL register to allow hardware to own and process the GCL.

ADDITIONAL INFORMATION: When the list length (as indicated in LLR) is 1, the associated time interval should be smaller than the value of the cycle time register. Otherwise, an error is reported (as detailed in the Error handling section) as a single set of gate controls add no value in the TSN context.

- ADDITIONAL INFORMATION: The time unit in all the GCL related registers is seconds and nanoseconds. In cases where internally generated PTP System Time is used, the nano-seconds field must be programmed to use the digital rollover mode ( EMAC\_TM\_CTL.TSCTRLSSR bit field must be set to 1).

## EST Registers

After completing the steps described in GCL and GCL Linked Registers, program the EMAC\_EST\_CTL register.

1. Set the current time offset value ( EMAC\_EST\_CTL.CTOV and time internal left shift fields ( EMAC\_EST\_CTL.TILS appropriately. Also, set the enable EST ( EMAC\_EST\_CTL.EEST ) and switch to SWOL ( EMAC\_EST\_CTL.SSWL ) bits.

This enables EMAC to own and process the new GCL and switch to the new GCL at the BTR value. If enabled, EMAC generates an interrupt when the switch to the new list occurs.

- Software must address any other interrupts received during the hardware execution of the GCL
2. Set the EMAC\_EST\_CTL.SSWL bit field to handoff to the controller. The EMAC\_EST\_CTL.SSWL is reset/cleared by the controller when it successfully switches to the new list.

The controller also flips the EMAC\_EST\_CTL.SSWL bit to indicate the new GCL that the software owns. To

install a new GCL, program the GCL it owns (indicated by SSWL bit) as described in GCL and GCL Linked Registers.

3. Program the EMAC\_EST\_CTL register as described. Ensure that the new BTR is set to an appropriate value to avoid BTR error that might need software intervention in some cases.

ADDITIONAL INFORMATION: To avoid transmission overruns, the packet length (frame size) information should always be available. Therefore, in the DMA configurations, program the packet length in the first descriptor of every transmit frame. Similarly, in the MTL configuration, provide the packet length in the control word.

## Programming Launch Time in Time-Based Scheduling

The launch time is programmed in the enhanced normal transmit descriptors in DMA configurations and is driven as a control word in MTL configurations as follows.

The OSTC and launch time features are mutually exclusive and should not be used together. If a context descriptor is received with a valid OSTC value immediately before receiving a first normal descriptor with the LTV bit set, then the LTV is ignored.

## Programming Media Clock Generation and Recovery

The following sections provide procedures for programming the media clock.

## Media Clock Generation

1. Program the appropriate presentation time control (supported generation modes "1001-1011") to the EMAC\_PPS\_CTL.PPSCTRL\_PPSCMD (for 0th instance) and EMAC\_PPS\_CTL.PPSCMD1 through EMAC\_PPS\_CTL.PPSCMD3 bits. Program all the EMAC\_PPS\_CTL.MCGREN0 through EMAC\_PPS\_CTL.MCGREN3 bits to enable MCGR mode.
2. ADDITIONAL INFORMATION: The application must drive the appropriate trigger signal to the corresponding EMAC\_PTPAUX\_MCG\_IN[n] pin, for the instances enabled for media clock generation.
2. Based on the programmed mode, MCGR instance[n] captures the time stamp and programs it in the EMAC\_PPS[n]\_TGTM\_SEC register. Then, the trigger requester event EMAC\_MCGR[n] is generated.

For every request generated by the MCGR instance n, EMAC reads the corresponding EMAC\_PPS[n]\_TGTM\_SEC register and acknowledges the read request by accepting the corresponding completer trigger request.

## Media Clock Recovery

1. Enable the Current Presentation Time (CPT) counter by setting the EMAC\_TM\_CTL.PTGE bit field. In addition to programming the initialization values for the system time, update the EMAC\_PRSNTM\_UPDT bits with the equivalent presentation time initial value. This is a requirement for the EMAC\_TM\_CTL.TSINIT bit field to be set.