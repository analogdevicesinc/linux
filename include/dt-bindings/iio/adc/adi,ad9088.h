/*
 * Driver for AD9088 and similar high-speed MxFEs
 *
 * Copyright 2022 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#ifndef _DT_BINDINGS_IIO_ADC_AD9088_H
#define _DT_BINDINGS_IIO_ADC_AD9088_H


 /*
  * JESD204-FSM defines
  */
#define DEFRAMER_LINK_A0_TX 0
#define DEFRAMER_LINK_A1_TX 1
#define DEFRAMER_LINK_B0_TX 2
#define DEFRAMER_LINK_B1_TX 3
#define FRAMER_LINK_A0_RX 4
#define FRAMER_LINK_A1_RX 5
#define FRAMER_LINK_B0_RX 6
#define FRAMER_LINK_B1_RX 7

#endif
