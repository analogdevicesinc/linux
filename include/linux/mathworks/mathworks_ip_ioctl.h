/*
 * MathWorks IP Common Functionality
 *
 * Copyright 2013-2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#ifndef _MATHWORKS_IP_IOCTL_H_
#define _MATHWORKS_IP_IOCTL_H_

#define MATHWORKS_IP_PARAM_LEN 		32

struct mathworks_ip_reg_info 
{
	size_t			size;
	void			*phys;
};

struct mathworks_ip_param_info 
{
    char 			name[MATHWORKS_IP_PARAM_LEN];
    size_t			size;
	void 			*buf;
};

struct mathworks_ip_dma_info
{
	size_t			size;
	void			*phys;
};


#define MATHWORKS_IP_IOCTL_MAGIC 			'B'
#define MATHWORKS_IP_GET_PARAM 				_IOWR(MATHWORKS_IP_IOCTL_MAGIC, 0, struct mwgeneric_param_info *)
#define MATHWORKS_IP_DMA_INFO 				_IOWR(MATHWORKS_IP_IOCTL_MAGIC, 1, struct mwgeneric_dma_info *)
#define MATHWORKS_IP_REG_INFO 				_IOWR(MATHWORKS_IP_IOCTL_MAGIC, 2, struct mwgeneric_reg_info *)

#define MATHWORKS_IP_DMA_OFFS				1024*4

#endif /*_MATHWORKS_IP_IOCTL_H_ */

/*EOF*/
