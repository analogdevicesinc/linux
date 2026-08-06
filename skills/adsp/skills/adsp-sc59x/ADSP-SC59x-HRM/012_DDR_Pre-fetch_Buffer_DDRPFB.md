# DDR Pre-fetch Buffer (DDRPFB)

<!-- source: 012_DDR_Pre-fetch_Buffer_DDRPFB.pdf | original pages 481–503 -->

## 10   DDR Pre-fetch Buffer (DDRPFB)

The DDR Pre-fetch buffer is designed to reduce the latency for fetching instructions or reading data from DDR memory. The Pre-fetch buffer works in the background to reduce the latency of a subsequent data or instruction access by pre-fetching the next data for next address (using the principle of locality). The DDR pre-fetch buffer is integrated in the DDR path and uses the following features.

- SMART pre-fetching decision which is a direct access or pre-fetch based on the pattern seen in the previous read transactions issued
- ID based direct access where in a multi-controller scenario with more than four controllers avoids a slow controller (for example the SPORT) to flush out a buffer line that is being used by a fast requester (for example HSMDMA)
- Range based return zero feature

The Pre-fetch Buffer Block Diagram figure shows the functional blocks within the DDRPFB.

Figure 10-1: Pre-fetch Buffer Block Diagram

<!-- image -->

## DDRPFB Features

The DDRPFB supports the following features:

- Smart Pre-fetch
- Direct DMC access
- Return Zero
- Controller ID based direct access
- Range based pre-fetch

## Smart Pre-fetch

Smart pre-fetch is used for the worst case scenario where the controller fetches data at non continuous addresses which results no hits in the DDRPFB. To avoid fetching excess data for such non continuous read transactions, smart pre-fetching choses to start to fetch and/or pre-fetch requests from the DDRPFB to the DMC or gives direct access for such read requests. The decision is made based on the history of read transactions received at the input of the prefetch buffer. The DDRPFB\_CTL1.SMART\_FEATURE\_EN bit is used to enable this feature.

## Data Invalidation

When the DDRPFB\_CTL0.DATA\_INVALIDATION bit is set (=1), the complete pre-fetch buffer is invalidated. Clear this bit manually after setting it to make use of pre-fetch buffer.

## Direct Access

In direct access, there are no pre-fetch accesses and the accesses are forwarded to the DMC directly. There is a one cycle latency while the pre-fetch buffer decides whether the access is direct or not. The read data from the DMC is directly forwarded to the controller without any latency impact and is not stored in the buffer.

## Return Zero

This feature allows the program to configure a range of addresses range in L3 memory to always returns zero. To enable this feature, set (=1) the DDRPFB\_CTL0.RETURN\_ZERO bit. The address range which require all zeros to be read is configured using the DDRPFB\_ZERO\_START.VAL and DDRPFB\_ZERO\_END.VAL registers. All the requests which that fall between the start and end addresses (that are aligned to 256 bytes) are returned with all zeros.

## Controller ID Based Direct Access

This section explains how to configure a specific SCB controller for direct access or DDR pre-fetch access. Set the DDRPFB\_CTL1.RQSTR\_ID\_FEATURE\_EN bit = 1 to use this feature. The following registers and bits are used to manage ID based direct access.

- The SCB controller ID is the ID of the SCB requester that is requesting data from the DDR. For ID information, see .

- The DDRPFB\_RQSTR\_ID\_EN.VAL bit field has four bits. It can take all 16 combinations. If this bit is cleared (=0) the corresponding SCB requester has direct access.
- The DDRPFB\_RQSTR0\_ID , DDRPFB\_RQSTR1\_ID , DDRPFB\_RQSTR2\_ID and DDRPFB\_RQSTR3\_ID registers are four IDs that are preprogrammed with the SCB ID. The configuration of these registers determine whether the DDRPFB gives a normal PFB enabled access or a direct access (if the corresponding DDRPFB\_RQSTR\_ID\_EN.VAL bit is set). If the SCB ID of the requester matches with the DDRPFB\_MSTRx\_ID, then the DDR pre-fetch operation is enabled.
- The DDRPFB\_RQSTR0\_ID\_MSK , DDRPFB\_RQSTR1\_ID\_MSK , DDRPFB\_RQSTR2\_ID\_MSK and DDRPFB\_RQSTR3\_ID\_MSK : When the DDRPFB\_RQSTR\_ID\_EN bit is set for some DDRPFB\_MSTRx\_ID, then the PFB enabled access is provided for a partial ID match of the SCB ID with the corresponding preprogrammed DDRPFB\_MSTRx\_ID. Clearing a bit masks the corresponding bit and the masked bits are not checked for a match.

The DDRPFB Access Configurations table shows the register configurations for different access types.

Table 10-1: DDRPFB Access Configurations

| SCB Requester ID   |   DDRPFB_MST R_ID_EN[0] | DDRPFB_MST R0_ID   | DDRPFB_MST R0_ID_MASK   | Access Type                                                                                                                                                                                                                                                                                                                                          |
|--------------------|-------------------------|--------------------|-------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| XXXX               |                       0 | XXXX               | XXXX                    | Direct access Irrespective of SCB requester ID, DDRPFB_RQSTR0_ID and DDRPFB_RQSTR0_ID_MSK , direct access is given to all the requests.                                                                                                                                                                                                              |
| MMMM               |                       1 | NNNN               | 0000                    | DDRPFB enabled access DDRPFB does not check any bits of the SCB requester ID if it matches with DDRPFB_RQSTR0_ID . Therefore, DDRPFB performs prefetch operation for all IDs. This is an example where DDR prefetch is enabled, though none of the bits of SCB ID matches with DDRPFB_RQSTR0_ID .                                                    |
| MMXX               |                       1 | MMXX               | 1100                    | DDRPFB enabled access DDRPFB looks for only two MSBs from SCB ID matching the two MSBs of DDRPFB_RQSTR0_ID . If they match, the DDRPFB performs a prefetch operation. Otherwise, direct ac- cess is given to such requests. In this example the two MSBs matching with the two MSBs of DDRPFB_RQSTR0_ID con- figures a normal DDRPFB enabled access. |
| MMXX               |                       1 | NNXX               | 1100                    | Direct access The PFB looks for only the two MSBs from the SCB ID that match with the two MSBs of DDRPFB_RQSTR0_ID . If they match, the PFB performs a normal operation. Otherwise, di- rect access is given to such requests. In this example, since the two MSBs do not match with the two MSBs of DDRPFB_RQSTR0_ID , it is a direct access.       |

## Range Based Pre-fetch

Range based prefetch allows an entire address range to be divided into 16 address ranges to selectively disable the pre-fetch buffer (while pre-fetching is enabled) for this address range using the DDRPFB\_RANGE\_SELECT register. For example, the Range Based Pre-fetch table shows that if DDRPFB\_RANGE\_SELECT [9] = 0, then the PFB is disabled for address range 90000000 to 9FFFFFFF .

The DDRPFB\_CTL1.RANGE\_SHIFT bit field is used to configure pre-fetching 16 sections of the address map based on the most significant nibble in the input address. In the example DDRPFB\_CTL1.RANGE\_SHIFT is 28.

Table 10-2: Range Based Pre-fetch

|   Sr. No. |                                                                        | PFB Operation          |
|-----------|------------------------------------------------------------------------|------------------------|
|         1 | DDRPFB_RANGE_SELECT [ARADDR_IN[31:28]]=1&& DDRPFB_CTL0.PREFETCH_EN = 1 | PFB enabled operation  |
|         2 | DDRPFB_RANGE_SELECT [ARADDR_IN[31:28]]=0&& DDRPFB_CTL0.PREFETCH_EN = 1 | PFB disabled operation |
|         3 | DDRPFB_RANGE_SELECT [ARADDR_IN[31:28]]=X&& DDRPFB_CTL0.PREFETCH_EN = 0 | PFB disabled operation |

Table 10-3: Range Based Address Range Selections

| Input bit        | Address range for Pre-fetch   |
|------------------|-------------------------------|
| RANGE_SELECT[0]  | 00000000 - 0FFFFFFF           |
| RANGE_SELECT[1]  | 10000000 - 1FFFFFFF           |
| RANGE_SELECT[2]  | 20000000 - 2FFFFFFF           |
| RANGE_SELECT[3]  | 30000000 - 3FFFFFFF           |
| RANGE_SELECT[4]  | 40000000 - 4FFFFFFF           |
| RANGE_SELECT[5]  | 50000000 - 5FFFFFFF           |
| RANGE_SELECT[6]  | 60000000 - 6FFFFFFF           |
| RANGE_SELECT[7]  | 70000000 - 7FFFFFFF           |
| RANGE_SELECT[8]  | 80000000 - 8FFFFFFF           |
| RANGE_SELECT[9]  | 90000000 - 9FFFFFFF           |
| RANGE_SELECT[10] | A0000000 - AFFFFFFF           |
| RANGE_SELECT[11] | B0000000 - BFFFFFFF           |
| RANGE_SELECT[12] | C0000000 - CFFFFFFF           |
| RANGE_SELECT[13] | D0000000 - DFFFFFFF           |
| RANGE_SELECT[14] | E0000000 - EFFFFFFF           |
| RANGE_SELECT[15] | F0000000 - FFFFFFFF           |

## Programming Notes

The following guidelines should be noted when using range based pre-fetch.

- DDRPFB\_RANGE\_SELECT.VAL is used to change the prefetch range
- When DDRPFB\_RANGE\_SELECT.VAL = 5'bN, then ARADDR\_IN[N+3:N] are used for range selection (here, N is programmable number)
- When DDRPFB\_CTL1.RANGE\_SHIFT = 28, logic behavior is same as above table
- When DDRPFB\_CTL1.RANGE\_SHIFT = 26, then ARADDR\_IN[29:26] are used for range selection

For finer control of a small range the configurations in the following table can be used.

|   Sr. No. |                                                                                      | PFB Operation          |
|-----------|--------------------------------------------------------------------------------------|------------------------|
|         1 | DDRPFB_RANGE_SELECT [ARADDR_IN[N+3:N]]=1&& DDRPFB_CTL0.PREFETCH_EN = 1               | PFB enabled operation  |
|         2 | DDRPFB_RANGE_SELECT [ARADDR_IN[N+3:N]]=0&& DDRPFB_CTL0.PREFETCH_EN = 1               | PFB disabled operation |
|         3 | DDRPFB_RANGE_SELECT [ARADDR_IN[N+3:N]] = X(don't care)&& DDRPFB_CTL0.PREFETCH_EN = 0 | PFB disabled operation |

## ADSP-SC59x DDRPFB Register List

PFB

Table 10-4: ADSP-SC59x DDRPFB Register List

| Name                 | Description                  |
|----------------------|------------------------------|
| DDRPFB_CTL0          | Control Register 0           |
| DDRPFB_CTL1          | Control Register 1           |
| DDRPFB_RANGE_SELECT  | Range Select Register        |
| DDRPFB_RQSTR0_ID     | Requester ID 0 Register      |
| DDRPFB_RQSTR0_ID_MSK | Requester ID 0 Mask Register |
| DDRPFB_RQSTR1_ID     | Requester ID 1 Register      |
| DDRPFB_RQSTR1_ID_MSK | Requester ID 1 Mask Register |
| DDRPFB_RQSTR2_ID     | Requester ID 2 Register      |
| DDRPFB_RQSTR2_ID_MSK | Requester ID 2 Mask Register |
| DDRPFB_RQSTR3_ID     | Requester ID 3 Register      |
| DDRPFB_RQSTR3_ID_MSK | Requester ID 3 Mask Register |
| DDRPFB_RQSTR_ID_EN   | Requester ID Enable Register |

Table 10-4: ADSP-SC59x DDRPFB Register List (Continued)

| Name              | Description                           |
|-------------------|---------------------------------------|
| DDRPFB_STS0       | Prefetch Buffer First Status Register |
| DDRPFB_ZERO_END   | Return Zero End Address Register      |
| DDRPFB_ZERO_START | Return Zero Start Address Register    |

## DDRPFB Programming Model

The following sections describe how to program the DDRPFB.

## Enabling the Pre-fetch Buffer and PFB Idle State

Set the DDRPFB\_CTL0.PREFETCH\_EN bit (=1) to enable the DDR pre-fetch buffer.

When the DDRPFB\_STS0.PFB\_IDLE bit = 1 there are no pending transactions between the DDRPFB and the DMC. When this bit = 0, there is at least one pending transaction between the DDRPFB and the DMC.

## General Programming Guidelines

The following are few guidelines which need to be followed for proper functioning of DDR PFB.

- The DDRPFB\_STS0.PFB\_IDLE indicates whether the DDRPFB is IDLE or not and can be polled to add delay to allow pending requests from the PFB to completer get completed.
- To issue a DMC reset during operation, the following steps are used to maintain data coherence and prevent system hangs.
1. Block all requesters that are currently accessing DDR
2. Check that the PFB is in the IDLE state by checking DDRPFB\_STS0.PFB\_IDLE
3. Issue a DMC reset
4. Invalidate the PFB
5. Unblock all requesters that need access to DDR
- During DDR initialization the PFB should be disabled. If the program needs to have the PFB enabled during DDR initialization, perform a PFB invalidation after DDR initialization is completed.
- The maximum possible value of DDRPFB\_CTL1.RANGE\_SHIFT is 'b11100. Any value that is more than 'b11100 has no significance because the address bus width is 32-bits.

## Guidelines For Achieving Maximum Performance

To achieve maximum performance, please note the following:

- Each DDR PFB line can hold 256 bytes of data. To achieve maximum performance the starting address of sequential memory accesses should be 256 bytes aligned

- Enable DDRPFB for requesters that support burst mode
- Burst mode with a burse size of 128 bits provide good performance compared to other combinations of burst size less than 128 bits
- Use the Requester ID based direct access feature for the SPORTs and IIR when there are more than four requesters acting at a time.
- Multiple requesters can be combined as a one requester when they are fetching data in interleaved fashion. This is accomplished by placing data in DDR in such a way that (for example), address A is for requester 1, A+x for requester 2, A+2x for again requester 1 and A +3x for requester 2. So in this example for requester 1 addresses A + 2nx are used and for requester 2 addresses A + (2n+1)x are used, where n = 0,1,2, and so on.
- Avoiding wrap access - Data defined in source code can be placed in memory in such a way that all requests from requester A55 is cache line aligned so that all requests from cache onwards are incremental accesses.
- The decision should be made on what is most suitable with respect to performance out of the various DDRPFB configurations, including DDRPFB disabled. In most cases PFB = enabled and the smart feature = enabled has better performance. For few use-cases PFB = enabled, smart feature disabled work best.

## ADSP-SC59x DDRPFB Register Descriptions

Prefetch buffer mmr registers (DDRPFB) contains the following registers.

Table 10-5: ADSP-SC59x DDRPFB Register List

| Name                 | Description                           |
|----------------------|---------------------------------------|
| DDRPFB_CTL0          | Control Register 0                    |
| DDRPFB_CTL1          | Control Register 1                    |
| DDRPFB_RANGE_SELECT  | Range Select Register                 |
| DDRPFB_RQSTR0_ID     | Requester ID 0 Register               |
| DDRPFB_RQSTR0_ID_MSK | Requester ID 0 Mask Register          |
| DDRPFB_RQSTR1_ID     | Requester ID 1 Register               |
| DDRPFB_RQSTR1_ID_MSK | Requester ID 1 Mask Register          |
| DDRPFB_RQSTR2_ID     | Requester ID 2 Register               |
| DDRPFB_RQSTR2_ID_MSK | Requester ID 2 Mask Register          |
| DDRPFB_RQSTR3_ID     | Requester ID 3 Register               |
| DDRPFB_RQSTR3_ID_MSK | Requester ID 3 Mask Register          |
| DDRPFB_RQSTR_ID_EN   | Requester ID Enable Register          |
| DDRPFB_STS0          | Prefetch Buffer First Status Register |
| DDRPFB_ZERO_END      | Return Zero End Address Register      |
| DDRPFB_ZERO_START    | Return Zero Start Address Register    |

## Control Register 0

The DDRPFB\_CTL0 register disables prefetching for up to sixteen address ranges. Bits[31:16] bits control the range selection for the instruction cache, and bits [15:0] control the range selection for the data cache.

Figure 10-2: DDRPFB\_CTL0 Register Diagram

<!-- image -->

Table 10-6: DDRPFB\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                       |
|--------------------|-------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DATA_INVALIDATION | Data Invalid. When the DDRPFB_CTL0.DATA_INVALIDATION bit is set the prefetch buffer is invalid. The program must clear this bit manually.                                                                                                                                                                                     | Data Invalid. When the DDRPFB_CTL0.DATA_INVALIDATION bit is set the prefetch buffer is invalid. The program must clear this bit manually.                                                                                                                                                                                     |
| 4 (R/W)            | DATA_INVALIDATION | 0                                                                                                                                                                                                                                                                                                                             | Buffer valid                                                                                                                                                                                                                                                                                                                  |
| 3 (R/W)            | DIRECT_ACCESS_EN  | Direct Access Enable. When the DDRPFB_CTL0.DIRECT_ACCESS_EN bit is set (=1) the prefetch buf- fer passes requests as is from a requester to a completer. No excess data is fetched and stored in prefetch buffer. When the DDRPFB_CTL0.DIRECT_ACCESS_EN bit is cleared (=0) the prefetch buffer behaves as if it is disabled. | Direct Access Enable. When the DDRPFB_CTL0.DIRECT_ACCESS_EN bit is set (=1) the prefetch buf- fer passes requests as is from a requester to a completer. No excess data is fetched and stored in prefetch buffer. When the DDRPFB_CTL0.DIRECT_ACCESS_EN bit is cleared (=0) the prefetch buffer behaves as if it is disabled. |
| 3 (R/W)            | DIRECT_ACCESS_EN  | 0                                                                                                                                                                                                                                                                                                                             | Direct access disabled                                                                                                                                                                                                                                                                                                        |
| 3 (R/W)            | DIRECT_ACCESS_EN  | 1                                                                                                                                                                                                                                                                                                                             | Direct access enabled                                                                                                                                                                                                                                                                                                         |
| 2 (R/W)            | RETURN_ZERO       | Return Zero Data. When the DDRPFB_CTL0.RETURN_ZERO bit is set, the prefetch buffer returns zero data for the address range configured in the DDRPFB_ZERO_START and DDRPFB_ZERO_END registers. The addresses in the DDRPFB_ZERO_START and DDRPFB_ZERO_END registers should be 128 byte aligned.                                | Return Zero Data. When the DDRPFB_CTL0.RETURN_ZERO bit is set, the prefetch buffer returns zero data for the address range configured in the DDRPFB_ZERO_START and DDRPFB_ZERO_END registers. The addresses in the DDRPFB_ZERO_START and DDRPFB_ZERO_END registers should be 128 byte aligned.                                |
| 2 (R/W)            | RETURN_ZERO       | 0                                                                                                                                                                                                                                                                                                                             | Return zero disabled                                                                                                                                                                                                                                                                                                          |
| 2 (R/W)            | RETURN_ZERO       | 1                                                                                                                                                                                                                                                                                                                             | Return zero data in selected address range                                                                                                                                                                                                                                                                                    |

Table 10-6: DDRPFB\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                          |
|--------------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | PREFETCH_EN | Prefetch Enable. When the DDRPFB_CTL0.PREFETCH_EN bit is set (=1), the prefetch buffer is en- abled. Otherwise, the prefetch buffer is disabled. |
| 0 (R/W)            | PREFETCH_EN | 0 Prefetch disabled                                                                                                                              |
| 0 (R/W)            | PREFETCH_EN | 1 Prefetch enabled                                                                                                                               |

## Control Register 1

The DDRPFB\_CTL1 register enables smart prefetch, range shifting, and direct access.

Figure 10-3: DDRPFB\_CTL1 Register Diagram

<!-- image -->

Table 10-7: DDRPFB\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                                                                                                                                                                                                                                                                                        |
|--------------------|-----------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | SMART_FEATURE_EN      | Smart Feature Enable. Setting the DDRPFB_CTL1.SMART_FEATURE_EN bit enables the SMART fea- ture.                                                                                                                                                                                                                                                |
| 7:3 (R/W)          | RANGE_SHIFT           | Range Shift. The DDRPFB_CTL1.RANGE_SHIFT bit field determines which bits of the read ad- dress (that appear at the input of prefetch buffer) are used for the range select opera- tion. By default this is the four MSBs are used. When this bit field is programmed to 5hN then [N+3:N] address bits are used for the range select operation. |
| 2 (R/W)            | RQSTR_ID_FEA- TURE_EN | ID Based Direct Access Enable. The DDRPFB_CTL1.RQSTR_ID_FEATURE_EN bit, when set (=1) enables the ID based direct access feature. 0 ID based direct access disabled                                                                                                                                                                            |
| 2 (R/W)            | RQSTR_ID_FEA- TURE_EN | 1 ID based direct access enabled                                                                                                                                                                                                                                                                                                               |
| 2 (R/W)            | RQSTR_ID_FEA- TURE_EN |                                                                                                                                                                                                                                                                                                                                                |

## Range Select Register

The DDRPFB\_RANGE\_SELECT register determines whether the prefetch buffer is enabled or disabled in the specified address range.

Figure 10-4: DDRPFB\_RANGE\_SELECT Register Diagram

<!-- image -->

Table 10-8: DDRPFB\_RANGE\_SELECT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VAL        | Range select bit selection. Each bit in this bit field determines prefetch buffer should be enabled or disabled in that address range. This bit field is used in conjunction with RANGE_SHIFT bit field. |

## Requester ID 0 Register

The DDRPFB\_RQSTR0\_ID register is used to configure the first requester ID for controller ID based direct access operations.

Figure 10-5: DDRPFB\_RQSTR0\_ID Register Diagram

<!-- image -->

Table 10-9: DDRPFB\_RQSTR0\_ID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VAL        | Requester Zero ID. The DDRPFB_RQSTR0_ID.VAL bit field identifies the first requester ID to be con- sidered for ID based direct access. |

## Requester ID 0 Mask Register

The DDRPFB\_RQSTR0\_ID\_MSK register is used to mask the first requester ID for controller ID based direct access operations. The masked bits are not used in direct access operations.

Figure 10-6: DDRPFB\_RQSTR0\_ID\_MSK Register Diagram

<!-- image -->

Table 10-10: DDRPFB\_RQSTR0\_ID\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VAL        | Requester Zero ID Mask. The DDRPFB_RQSTR0_ID_MSK.VAL bit field masks the first requester ID to ex- clude from ID based direct access. |

## Requester ID 1 Register

The DDRPFB\_RQSTR1\_ID register is used to configure the first requester ID for controller ID based direct access operations.

Figure 10-7: DDRPFB\_RQSTR1\_ID Register Diagram

<!-- image -->

Table 10-11: DDRPFB\_RQSTR1\_ID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VAL        | Requester One ID. The DDRPFB_RQSTR1_ID.VAL bit field identifies the second requester ID to be considered for ID based direct access. |

## Requester ID 1 Mask Register

The DDRPFB\_RQSTR1\_ID\_MSK register is used to mask the second requester ID for controller ID based direct access operations. The masked bits are not used in direct access operations.

Figure 10-8: DDRPFB\_RQSTR1\_ID\_MSK Register Diagram

<!-- image -->

Table 10-12: DDRPFB\_RQSTR1\_ID\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VAL        | Requester One ID Mask. The DDRPFB_RQSTR1_ID_MSK.VAL bit field masks the second requester ID to exclude from ID based direct access. |

## Requester ID 2 Register

The DDRPFB\_RQSTR2\_ID register is used to configure the third requester ID for controller ID based direct access operations.

Figure 10-9: DDRPFB\_RQSTR2\_ID Register Diagram

<!-- image -->

Table 10-13: DDRPFB\_RQSTR2\_ID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VAL        | Requester Two ID. The DDRPFB_RQSTR2_ID.VAL bit field identifies the third requester ID to be considered for ID based direct access. |

## Requester ID 2 Mask Register

The DDRPFB\_RQSTR2\_ID\_MSK register is used to mask the third requester ID for controller ID based direct access operations. The masked bits are not used in direct access operations.

Figure 10-10: DDRPFB\_RQSTR2\_ID\_MSK Register Diagram

<!-- image -->

Table 10-14: DDRPFB\_RQSTR2\_ID\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VAL        | Requester Two ID Mask. The DDRPFB_RQSTR2_ID_MSK.VAL bit field masks the third requester ID to ex- clude from ID based direct access. |

## Requester ID 3 Register

The DDRPFB\_RQSTR3\_ID register is used to configure the fourth requester ID for controller ID based direct access operations.

Figure 10-11: DDRPFB\_RQSTR3\_ID Register Diagram

<!-- image -->

Table 10-15: DDRPFB\_RQSTR3\_ID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VAL        | Requester Three ID. The DDRPFB_RQSTR3_ID.VAL bit field identifies the fourth requester ID to be considered for ID based direct access. |

## Requester ID 3 Mask Register

The DDRPFB\_RQSTR3\_ID\_MSK register is used to mask the fourth requester ID for controller ID based direct access operations. The masked bits are not used in direct access operations.

Figure 10-12: DDRPFB\_RQSTR3\_ID\_MSK Register Diagram

<!-- image -->

Table 10-16: DDRPFB\_RQSTR3\_ID\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | VAL        | Requester Three ID Mask. The DDRPFB_RQSTR3_ID_MSK.VAL bit field masks the fourth requester ID to exclude from ID based direct access. |

## Requester ID Enable Register

The DDRPFB\_RQSTR\_ID\_EN register is used to configure controller based direct access. As many as four controller IDs can be programmed for direct access. For more information, see "Controller ID Based Direct Access".

Figure 10-13: DDRPFB\_RQSTR\_ID\_EN Register Diagram

<!-- image -->

Table 10-17: DDRPFB\_RQSTR\_ID\_EN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | VAL        | Requester ID Enable. The DDRPFB_RQSTR_ID_EN.VAL bits are used to configure controller based di- rect access. As many as four controller IDs can be programmed for direct access. For more information, see "Controller ID Based Direct Access". |

## Prefetch Buffer First Status Register

The DDRPFB\_STS0 register indicates whether all fetch and prefetch operations from the PFB to the DDR are complete.

Figure 10-14: DDRPFB\_STS0 Register Diagram

<!-- image -->

Table 10-18: DDRPFB\_STS0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------|
|                    | 1 PFB_IDLE | Prefetch Idle. The DDRPFB_STS0.PFB_IDLE bit, when set (=1), indicates all fetch and prefetch |
| (R/NW)             |            | operations from the PFB to the DDR are complete.                                             |

## Return Zero End Address Register

The DDRPFB\_ZERO\_END register is used with the DDRPFB\_ZERO\_START register to configure an address range in L3 memory to always returns zero.

Figure 10-15: DDRPFB\_ZERO\_END Register Diagram

<!-- image -->

Table 10-19: DDRPFB\_ZERO\_END Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VAL        | Return Zero End Address. The DDRPFB_ZERO_END.VAL bit field is used to configure the end address (that are aligned to 256 bytes) that are returned with all zeros. |

## Return Zero Start Address Register

The DDRPFB\_ZERO\_START register is used with the DDRPFB\_ZERO\_END register to configure a range of addresses in L3 memory to always returns zero.

Figure 10-16: DDRPFB\_ZERO\_START Register Diagram

<!-- image -->

Table 10-20: DDRPFB\_ZERO\_START Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VAL        | Return Zero Start Address. The DDRPFB_ZERO_START.VAL bit field is used to configure the start address (that are aligned to 256 bytes) that are returned with all zeros. |