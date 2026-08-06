## 41   Public Key Interrupt Controller (PKIC)

The Public Key Accelerator (PKA) and the True Random Number Generator (TRNG) share a common interrupt controller, the Public Key Interrupt Controller (PKIC). The host processor configures the PKIC to generate interrupts when certain operations are complete, or interrupts are caused by errors.

## PKIC Functional Description

The main purpose and function of the PKIC is to capture the interrupts from different sources, either from the PKA or the TRNG and combine them to one interrupt output. The interrupt controller is managed using the following register groups:

- Control for polarity, edge, and level detection and enabling of individual interrupts
- Acknowledgment (to clear edge detected interrupts)
- Status:
- A raw source status register after edge detection, if edge selected.
- A status register after masking with the interrupt enable control bits.

## ADSP-2184x PKIC Register List

The Public Key Processor Interrupt Controller (PKIC) provides security-related features. A set of registers governs PKIC operations. For more information on PKIC functionality, see the PKIC register descriptions.

Table 41-1: ADSP-2184x PKIC Register List

| Name         | Description               |
|--------------|---------------------------|
| PKIC_ACK     | Acknowledge Register      |
| PKIC_EN_CLR  | Enable Clear Register     |
| PKIC_EN_CTL  | Enable Control Register   |
| PKIC_EN_SET  | Enable Set Register       |
| PKIC_EN_STAT | Enabled Status Register   |
| PKIC_POL_CTL | Polarity Control Register |

Table 41-1: ADSP-2184x PKIC Register List (Continued)

| Name          | Description           |
|---------------|-----------------------|
| PKIC_RAW_STAT | Raw Status Register   |
| PKIC_TYPE_CTL | Type Control Register |

## ADSP-2184x PKIC Interrupt List

Table 41-2: ADSP-2184x PKIC Interrupt List

|   Interrupt ID | Name      | Description     | Sensitivity   | DMA Channel   |
|----------------|-----------|-----------------|---------------|---------------|
|            225 | PKIC0_IRQ | PKIC0 Interrupt | Level         |               |

## PKIC Programming Model

The following sections provide information on how to program the PKIC.

## Enabling/Disabling and Status

The PKIC\_EN\_STAT register provides the mask to which interrupt source is enabled. There are two status registers, PKIC\_RAW\_STAT and PKIC\_EN\_STAT . They allow the host processor to read the status of the interrupt source before and after the mask is applied.

## Level or Edge

All of the interrupt sources are level or edge events. The PKIC\_TYPE\_CTL register configures each interrupt to either level or edge. The PKIC\_POL\_CTL register controls the polarity of the signal.

These interrupts are latched at both status registers in case edge detection is selected. The edge detectors are reset by clearing the interrupts using the PKIC\_ACK registers.

## PKIC Programming Concepts

The following concepts help with proper programming for the PKIC module.

## Interrupt Handling

When an interrupt is triggered, the handler must first examine this module, the PKIC to determine what triggered the interrupt. By reading the PKIC\_EN\_STAT , the bits that are set are the pending interrupts of the ones that were not masked. After determining the source of the interrupt, the appropriate action must be taken to service the interrupt from the corresponding module, the PKA, or the TRNG. After handling the interrupt in a particular module, the corresponding interrupt must be acknowledged and cleared in the PKIC to allow further interrupts.

While handling an interrupt, any events that would cause another interrupt would happen without triggering another interrupt.

## Overlapping Registers

There are two sets of overlapping registers in the PKIC. The PKIC\_EN\_STAT and PKIC\_ACK registers share one address. If read, the register tells which enabled interrupts are pending. If written to (W1C), the interrupt is acknowledged and cleared. The PKIC\_RAW\_STAT and PKIC\_EN\_SET registers are another pair that share the address. When read, the register tells which interrupts are pending and if written to will enabled certain interrupts. This register cannot be used to disable any interrupts.

## ADSP-2184x PKIC Register Descriptions

Public Key Processor Interrupt Controller (PKIC) contains the following registers.

Table 41-3: ADSP-2184x PKIC Register List

| Name          | Description               |
|---------------|---------------------------|
| PKIC_ACK      | Acknowledge Register      |
| PKIC_EN_CLR   | Enable Clear Register     |
| PKIC_EN_CTL   | Enable Control Register   |
| PKIC_EN_SET   | Enable Set Register       |
| PKIC_EN_STAT  | Enabled Status Register   |
| PKIC_POL_CTL  | Polarity Control Register |
| PKIC_RAW_STAT | Raw Status Register       |
| PKIC_TYPE_CTL | Type Control Register     |

## Acknowledge Register

The PKIC\_ACK register is used to acknowledge the interrupt and clear the corresponding interrupt bit in the status register.

Figure 41-1: PKIC\_ACK Register Diagram

![Image](44_Public_Key_Interrupt_Controller_(PKIC)_artifacts/image_000000_7d426838740291748cb1e729ca2e2f4f37aa2d14e5304bd8f18c6d21c55939bb.png)

Table 41-4: PKIC\_ACK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (RX/W)           | SLERRINT   | Completer Error IRQ. The PKIC_ACK.SLERRINT bit is the acknowledge bit for the completer error interrupt. When set =1 the PKIC_ACK.SLERRINT bit acknowledges the interrupt signal and clears the status bit (and is cleared automatically). 0 Do not acknowledge interrupt and clear status bit 1 Acknowledge interrupt and clear status bit |
| 3 (RX/W)           | TRNGINT    | TRNG IRQ. The PKIC_ACK.TRNGINT bit is the acknowledge bit for the TRNG interrupt. When set =1 the PKIC_ACK.TRNGINT bit acknowledges the interrupt signal and clears the status bit (and is cleared automatically).                                                                                                                          |
| 1 (RX/W)           | PKAINT1    | PKA Completion IRQ. The PKIC_ACK.PKAINT1 bit is the acknowledge bit for the PKA completion interrupt. When set =1 the PKIC_ACK.PKAINT1 bit acknowledges the interrupt signal and clears the status bit (and is cleared automatically).                                                                                                      |

## Enable Clear Register

The PKIC\_EN\_CLR register allows the user to disable certain interrupts without enabling others. The disabled interrupts are also reflected in PKIC\_EN\_CTL register.

Figure 41-2: PKIC\_EN\_CLR Register Diagram

![Image](44_Public_Key_Interrupt_Controller_(PKIC)_artifacts/image_000001_8d2dbc67df33c27691dd02d7875f795f0729f5f961c2598343b44e3cbb66d346.png)

Table 41-5: PKIC\_EN\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (RX/W)           | SLERRINT   | Completer Error IRQ. The PKIC_EN_CLR.SLERRINT bit is the individual disable for the completer error interrupt. When set =1 this bit clears/resets the corresponding bit in the PKIC_EN_CTL register to 0 (disables the interrupt). This bit is cleared automatically.              | Completer Error IRQ. The PKIC_EN_CLR.SLERRINT bit is the individual disable for the completer error interrupt. When set =1 this bit clears/resets the corresponding bit in the PKIC_EN_CTL register to 0 (disables the interrupt). This bit is cleared automatically.              |
| 5 (RX/W)           | SLERRINT   | 0                                                                                                                                                                                                                                                                                  | No action                                                                                                                                                                                                                                                                          |
| 5 (RX/W)           | SLERRINT   | 1                                                                                                                                                                                                                                                                                  | Clear/reset corresponding CTL bit                                                                                                                                                                                                                                                  |
| 3 (RX/W)           | TRNGINT    | TRNG IRQ. The PKIC_EN_CLR.TRNGINT bit is the individual disable for the TRNG inter- rupt. When set =1 this bit clears/resets the corresponding bit in the PKIC_EN_CTL register to 0 (disables the interrupt). This bit is cleared automatically. 0= no effect.                     | TRNG IRQ. The PKIC_EN_CLR.TRNGINT bit is the individual disable for the TRNG inter- rupt. When set =1 this bit clears/resets the corresponding bit in the PKIC_EN_CTL register to 0 (disables the interrupt). This bit is cleared automatically. 0= no effect.                     |
| 3 (RX/W)           | TRNGINT    | 0                                                                                                                                                                                                                                                                                  | No action                                                                                                                                                                                                                                                                          |
| 3 (RX/W)           | TRNGINT    | 1                                                                                                                                                                                                                                                                                  | Clear/reset corresponding CTL bit                                                                                                                                                                                                                                                  |
| 1 (RX/W)           | PKAINT1    | PKA Completion IRQ. The PKIC_EN_CLR.PKAINT1 bit is the individual disable for the PKA Com- pletion interrupt. When set =1 this bit clears/resets the corresponding bit in the PKIC_EN_CTL register to 0 (disables the interrupt). This bit is cleared automatically. 0= no effect. | PKA Completion IRQ. The PKIC_EN_CLR.PKAINT1 bit is the individual disable for the PKA Com- pletion interrupt. When set =1 this bit clears/resets the corresponding bit in the PKIC_EN_CTL register to 0 (disables the interrupt). This bit is cleared automatically. 0= no effect. |
| 1 (RX/W)           | PKAINT1    | 0                                                                                                                                                                                                                                                                                  | No action                                                                                                                                                                                                                                                                          |
| 1 (RX/W)           | PKAINT1    | 1                                                                                                                                                                                                                                                                                  | Clear/reset corresponding CTL bit                                                                                                                                                                                                                                                  |

## Enable Control Register

The PKIC\_EN\_CTL register provides individual enable bits for the interrupt sources.

Figure 41-3: PKIC\_EN\_CTL Register Diagram

![Image](44_Public_Key_Interrupt_Controller_(PKIC)_artifacts/image_000002_607aa3c36b8b4464a7ec2f1ba6731fdd7853aaa7f7101c801dc745d36019a60f.png)

Table 41-6: PKIC\_EN\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | SLERRINT   | Completer Error IRQ. The PKIC_EN_CTL.SLERRINT bit enables control for the completer error inter- rupt.                                                        |
| 3 (R/W)            | TRNGINT    | TRNG IRQ. The PKIC_EN_CTL.TRNGINT bit enables control for the TRNG interrupt. 0 Disable interrupt                                                             |
| 1 (R/W)            | PKAINT1    | 1 Enable interrupt PKA Completion IRQ. The PKIC_EN_CTL.PKAINT1 bit enables control for the PKA completion inter- rupt. 0 Disable interrupt 1 Enable interrupt |

## Enable Set Register

The PKIC\_EN\_SET register allows the user to only set certain interrupt sources without disabling any others. The enabled interrupts are reflected in PKIC\_EN\_CTL register.

Figure 41-4: PKIC\_EN\_SET Register Diagram

![Image](44_Public_Key_Interrupt_Controller_(PKIC)_artifacts/image_000003_8d2dbc67df33c27691dd02d7875f795f0729f5f961c2598343b44e3cbb66d346.png)

Table 41-7: PKIC\_EN\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (RX/W)           | SLERRINT   | Completer Error IRQ. The PKIC_EN_SET.SLERRINT bit is the individual enable for the completer error interrupt. If =1, sets the corresponding bit in the PKIC_EN_CTL register to 1 (enables interrupt). This bit is cleared automatically. 0=no effect                 |
| 3 (RX/W)           | TRNGINT    | TRNG IRQ. The PKIC_EN_SET.TRNGINT bit is the individual enable for the TRNG inter- rupt. If =1, sets the corresponding bit in the PKIC_EN_CTL register to 1 (enables interrupt). This bit is cleared automatically.                                                  |
| 1 (RX/W)           | PKAINT1    | PKA Completion IRQ. The PKIC_EN_SET.PKAINT1 bit is the individual enable for the PKA Completion interrupt. If =1, sets the corresponding bit in the PKIC_EN_CTL register to 1 (enables interrupt). This bit is cleared automatically. 0 No effect 1 Enable interrupt |

## Enabled Status Register

The PKIC\_EN\_STAT register is used to tell the status of the interrupts after the gating with the PKIC\_EN\_CTL register.

Figure 41-5: PKIC\_EN\_STAT Register Diagram

![Image](44_Public_Key_Interrupt_Controller_(PKIC)_artifacts/image_000004_ed4bcf5bd18f016ce0ec1710f35fe8c98bf651ab2d69f2531c119115f35099b1.png)

Table 41-8: PKIC\_EN\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/NW)           | SLERRINT   | Completer Error IRQ. The PKIC_EN_STAT.SLERRINT bit provides the status of the completer error interrupt (after masking from the PKIC_EN_CTL register).                      |
| 3 (R/NW)           | TRNGINT    | TRNG IRQ. The PKIC_EN_STAT.TRNGINT bit provides the status of the TRNG interrupt (after masking from the PKIC_EN_CTL register).                                             |
| 1 (R/NW)           | PKAINT1    | PKA Completion IRQ. The PKIC_EN_STAT.PKAINT1 bit provides the status of the PKA Completion interrupt (after masking from the PKIC_EN_CTL register). 0 Interrupt is inactive |
| 1 (R/NW)           |            |                                                                                                                                                                             |

## Polarity Control Register

The PKIC\_POL\_CTL register is used to configure the signal polarity for each individual interrupt. During the initialization phase of the PKA the host processor must set each interrupt in this register to (high level/rising edge or low level/falling edge).

Figure 41-6: PKIC\_POL\_CTL Register Diagram

![Image](44_Public_Key_Interrupt_Controller_(PKIC)_artifacts/image_000005_7f132ef0a0cf24324bb3c09e917631227753a991137b1568a45587e5da624d8c.png)

Table 41-9: PKIC\_POL\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | SLERRINT   | Completer Error IRQ. The PKIC_POL_CTL.SLERRINT bit provides polarity control for the completer error interrupt.                   |
| 3 (R/W)            | TRNGINT    | TRNG IRQ. The PKIC_POL_CTL.TRNGINT bit provides polarity control for the TRNG inter- rupt.                                        |
| 1 (R/W)            | PKAINT1    | PKA Completion IRQ. The PKIC_POL_CTL.PKAINT1 bit provides polarity control for PKA completion interrupt. 0 Low level/falling edge |

## Raw Status Register

The PKIC\_RAW\_STAT register reflects the status of the individual interrupts before masking with the PKIC\_EN\_CTL register.

Figure 41-7: PKIC\_RAW\_STAT Register Diagram

![Image](44_Public_Key_Interrupt_Controller_(PKIC)_artifacts/image_000006_09f252418dfa719396f734fb4226c73eb54ff6e24ffff7da38f8d0779311493c.png)

Table 41-10: PKIC\_RAW\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/NW)           | SLERRINT   | Completer Error IRQ. The PKIC_RAW_STAT.SLERRINT bit provides the raw status of the completer error interrupt.                                              |
| 3 (R/NW)           | TRNGINT    | TRNG IRQ. The PKIC_RAW_STAT.TRNGINT bit provides the raw status of the TRNG inter- rupt where 1=pending and 0=inactive.                                    |
| 1 (R/NW)           | PKAINT1    | PKA Completion IRQ. The PKIC_RAW_STAT.PKAINT1 bit provides raw status of the PKA completion interrupt where 1=pending and 0=inactive. 0 Inactive interrupt |
| 1 (R/NW)           |            |                                                                                                                                                            |

## Type Control Register

The PKIC\_TYPE\_CTL register is used to configure the signal type for each individual interrupt. During the initialization phase of the PKA the host processor must set each interrupt in this register to level or edge.

Figure 41-8: PKIC\_TYPE\_CTL Register Diagram

![Image](44_Public_Key_Interrupt_Controller_(PKIC)_artifacts/image_000007_4fed1c6bd8f679dda8d40947939f76a2e8ecda53792ed71031b121f3e2abf1c3.png)

Table 41-11: PKIC\_TYPE\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | SLERRINT   | Completer Error IRQ. The PKIC_TYPE_CTL.SLERRINT bit provides signal type control for the com- pleter error interrupt.    |
| 3 (R/W)            | TRNGINT    | TRNG IRQ. The PKIC_TYPE_CTL.TRNGINT bit provides signal type control for the TRNG interrupt.                             |
| 1 (R/W)            | PKAINT1    | PKA Completion IRQ. The PKIC_TYPE_CTL.PKAINT1 bit provides signal type control for the PKA completion interrupt. 0 Level |