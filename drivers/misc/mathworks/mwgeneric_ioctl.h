/* Copyright 2013, The MathWorks, Inc */
#ifndef _MWGENERIC_IOCTL_H_
#define _MWGENERIC_IOCTL_H_

#define MWGENERIC_PARAM_NAME_LEN 		32

struct mwgeneric_reg_info 
{
	size_t			size;
	void			*phys;
};

struct mwgeneric_param_info 
{
    char 			name[MWGENERIC_PARAM_NAME_LEN];
    size_t			size;
	void 			*buf;
};

struct mwgeneric_dma_info
{
	size_t			size;
	void			*phys;
};


#define MWGENERIC_IOCTL_MAGIC 			'B'
#define MWGENERIC_GET_PARAM 			_IOWR(MWGENERIC_IOCTL_MAGIC, 0, struct mwgeneric_param_info *)
#define MWGENERIC_DMA_INFO 				_IOWR(MWGENERIC_IOCTL_MAGIC, 1, struct mwgeneric_dma_info *)
#define MWGENERIC_REG_INFO 				_IOWR(MWGENERIC_IOCTL_MAGIC, 2, struct mwgeneric_reg_info *)

#define MWGENERIC_DMA_OFFS				1024*4

#endif /*_MWGENERIC_IOCTL_H_ */

/*EOF*/
