/* Copyright 2013, The MathWorks, Inc */
#ifndef _MWIPCORE_IOCTL_H_
#define _MWIPCORE_IOCTL_H_


struct raw_access_data 
{
    unsigned int offset_addr;
    unsigned int data_length; /*In bytes*/
    void *data_buf;
};


#define MWIPCORE_DMA_MAGIC 'A'
#define MWIPCORE_DMA_IOC_RESET _IO(MWIPCORE_DMA_MAGIC, 0)
#define MWIPCORE_REGISTER_READ _IOWR(MWIPCORE_DMA_MAGIC, 1, struct raw_access_data *)
#define MWIPCORE_REGISTER_WRITE _IOWR(MWIPCORE_DMA_MAGIC, 2, struct raw_access_data *)
#define MWIPCORE_DMA_SET_LENGTH _IOW(MWIPCORE_DMA_MAGIC, 3, unsigned long)
#define MWIPCORE_DMA_GET_LENGTH _IOR(MWIPCORE_DMA_MAGIC, 4, unsigned long)
#define MWIPCORE_DMA_START _IO(MWIPCORE_DMA_MAGIC, 5)
#define MWIPCORE_DMA_STOP _IO(MWIPCORE_DMA_MAGIC, 6)
#define MWIPCORE_SET_INTERRUPT_OFFSET _IOR(MWIPCORE_DMA_MAGIC, 7, unsigned long)

#define MAX_DMA_TRANSFER_SIZE (4*1024*1024)
#define DEFAULT_FIFO_UNITS 4*1024  /* one page*/
#define DEFAULT_BYTES_PER_UNIT 1



#endif /*_MWIPCORE_IOCTL_H_ */

/*EOF*/
