/* Copyright 2013-2015 The MathWorks, Inc. */
#ifndef _MWADMA_IOCTL_H_
#define _MWADMA_IOCTL_H_

#define MAX_DMASIZE 1024*1024*32*2 /* 32 MB per channel */

struct mw_axidma_params {
    char           *virt;
    dma_addr_t     phys;
    size_t         offset;
    size_t         counter;
    size_t         size;
    size_t         bytes_per_ring;
    size_t         desc_length;
    size_t         total_rings;
};

enum SIGNAL_TRANSFER {
    SIGNAL_TRANSFER_COMPLETE = 1,
    SIGNAL_OFF,
    SIGNAL_DATAFLOW,
    SIGNAL_BURST_COMPLETE,
    MAX_SIGNAL_TRANSFER
};

enum TX_QUEUE_ERROR {
    TX_ERROR_QUNDERFLOW = 0,
    TX_ERROR_QLOW,
    TX_ERROR_QPRIME,
    TX_ERROR_QFULL,
    TX_ERROR_QOVERFLOW,
    MAX_TX_QUEUE_ERROR
};

enum TX_WATERMARK {
    TX_WATERMARK_QUNDERFLOW = 0,
    TX_WATERMARK_QLOW,
    TX_WATERMARK_QPRIME,
    TX_WATERMARK_QFULL = 20,
    TX_WATERMARK_QOVERFLOW = 256,
    MAX_TX_WATERMARK
};

#define MWADMA_MAGIC 'Q'
#define MWADMA_IOC_RESET            _IO(MWADMA_MAGIC, 0)
#define MWADMA_GET_PROPERTIES       _IO(MWADMA_MAGIC, 1)
#define MWADMA_TEST_LOOPBACK        _IO(MWADMA_MAGIC, 18)

#define MWADMA_SETUP_RX_CHANNEL     _IO(MWADMA_MAGIC, 2)
#define MWADMA_RX_SINGLE            _IO(MWADMA_MAGIC, 4) 
#define MWADMA_RX_BURST             _IO(MWADMA_MAGIC, 6) 
#define MWADMA_RX_CONTINUOUS        _IO(MWADMA_MAGIC, 8)
#define MWADMA_RX_GET_NEXT_INDEX    _IO(MWADMA_MAGIC, 10)
#define MWADMA_RX_STOP              _IO(MWADMA_MAGIC, 12)
#define MWADMA_RX_GET_ERROR         _IO(MWADMA_MAGIC, 14)
#define MWADMA_FREE_RX_CHANNEL      _IO(MWADMA_MAGIC, 16)

#define MWADMA_SETUP_TX_CHANNEL     _IO(MWADMA_MAGIC, 3)
#define MWADMA_TX_ENQUEUE           _IO(MWADMA_MAGIC, 5) 
#define MWADMA_TX_SINGLE            _IO(MWADMA_MAGIC, 7) 
#define MWADMA_TX_BURST             _IO(MWADMA_MAGIC, 9) 
#define MWADMA_TX_CONTINUOUS        _IO(MWADMA_MAGIC, 11) 
#define MWADMA_TX_GET_ERROR         _IO(MWADMA_MAGIC, 13)
#define MWADMA_TX_STOP              _IO(MWADMA_MAGIC, 15)
#define MWADMA_TX_GET_NEXT_INDEX    _IO(MWADMA_MAGIC, 17)
#define MWADMA_FREE_TX_CHANNEL      _IO(MWADMA_MAGIC, 19)


#endif /*_MWADMA_IOCTL_H_ */

/*EOF*/
