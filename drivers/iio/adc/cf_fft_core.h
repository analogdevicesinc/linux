/*
 * ADI-FFT Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/
 */

#ifndef ADI_FFT_H_
#define ADI_FFT_H_

int fft_calculate(dma_addr_t src, dma_addr_t dest,
		  unsigned int size, unsigned irsel);

#endif /* ADI_FFT_H_ */
