/**
 * \file mykonos_user.h
 *
 * \brief Contains macro definitions and global structure declarations for mykonos_user.c
 *
 * Mykonos API version: 1.2.05.3475
 */

#ifndef _MYKONOSPROFILES_H_
#define _MYKONOSPROFILES_H_

#ifdef __cplusplus
extern "C" {
#endif

/* turns verbose messaging on */
#define MYKONOS_VERBOSE 0
#define MYK_ENABLE_SPIWRITEARRAY 0

/* 3 Bytes per SPI transaction * 341 transactions = ~1024 byte buffer size */
/* Minimum MYK_SPIWRITEARRAY_BUFFERSIZE = 27 */
#define MYK_SPIWRITEARRAY_BUFFERSIZE 300
/*
 *****************************************
 * Rx, ObsRx, and Sniffer gain tables
 ******************************************
 */
#define MAX_GAIN_TABLE_INDEX			255

#define START_RX_GAIN_INDEX 			255
#define MAX_RX_GAIN_TABLE_NUMINDEXES	(START_RX_GAIN_INDEX + 1)
#define MIN_RX_GAIN_TABLE_INDEX			0

#define START_ORX_GAIN_INDEX 			47
#define MAX_ORX_GAIN_TABLE_NUMINDEXES  	(START_ORX_GAIN_INDEX + 1)
#define MIN_ORX_GAIN_TABLE_INDEX		(MAX_GAIN_TABLE_INDEX - START_ORX_GAIN_INDEX)

#define START_SNRX_GAIN_INDEX 			127
#define MAX_SNRX_GAIN_TABLE_NUMINDEXES	(START_SNRX_GAIN_INDEX + 1)
#define MIN_SNRX_GAIN_TABLE_INDEX		(MAX_GAIN_TABLE_INDEX - START_SNRX_GAIN_INDEX)

#define START_LOOPBACK_GAIN_INDEX           47
#define MAX_LOOPBACK_GAIN_TABLE_NUMINDEXES  (START_LOOPBACK_GAIN_INDEX + 1)
#define MIN_LOOPBACK_GAIN_TABLE_INDEX       (MAX_GAIN_TABLE_INDEX - START_SNRX_GAIN_INDEX)

extern uint8_t RxGainTable [61][4];
extern uint8_t ORxGainTable [19][4];
extern uint8_t SnRxGainTable [53][4];


/*
 ********************************************
 * Rx, Sniffer, ObsRx and Tx Profiles limits
 ********************************************
 */
#define MIN_RX_IQRATE_KHZ   15000
#define MAX_RX_IQRATE_KHZ   320000

#define MIN_TX_IQRATE_KHZ   15000
#define MAX_TX_IQRATE_KHZ   492000

#define MIN_SNIFFER_RFBW_HZ 5000000
#define MAX_SNIFFER_RFBW_HZ 20000000

#define MIN_ORX_RFBW_HZ     5000000
#define MAX_ORX_RFBW_HZ     250000000

#define MIN_RX_RFBW_HZ      5000000
#define MAX_RX_RFBW_HZ      100000000

#define MIN_TX_RFBW_HZ      20000000
#define MAX_TX_RFBW_HZ      250000000

#ifdef __cplusplus
}
#endif

#endif
