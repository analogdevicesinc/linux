/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Driver for AD9083 and similar high-speed Analog-to-Digital converters
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#ifndef _DT_BINDINGS_IIO_ADC_AD9083_H
#define _DT_BINDINGS_IIO_ADC_AD9083_H

#define AD9083_CIC_DEC_4 0				/*!< Decimation by 4 */
#define AD9083_CIC_DEC_8 1				/*!< Decimation by 8 */
#define AD9083_CIC_DEC_16 2				/*!< Decimation by 16 */

#define AD9083_J_DEC_1 0				/*!< Bypass */
#define AD9083_J_DEC_4 1				/*!< Decimation by 4 */
#define AD9083_J_DEC_8 2				/*!< Decimation by 8 */
#define AD9083_J_DEC_16 3				/*!< Decimation by 16 */
#define AD9083_J_DEC_12 6				/*!< Decimation by 12 */
#define AD9083_J_DEC_24 7				/*!< Decimation by 24 */
#define AD9083_J_DEC_10 9				/*!< Decimation by 10 */
#define AD9083_J_DEC_20 10				/*!< Decimation by 20 */
#define AD9083_J_DEC_30 14				/*!< Decimation by 30 */
#define AD9083_J_DEC_40 11				/*!< Decimation by 40 */
#define AD9083_J_DEC_60 15				/*!< Decimation by 60 */

#define AD9083_DATAPATH_ADC_CIC 1			/*!< ADC -> CIC -> output */
#define AD9083_DATAPATH_ADC_CIC_NCO_J 2			/*!< ADC -> CIC -> NCO -> J -> output */
#define AD9083_DATAPATH_ADC_CIC_J 3			/*!< ADC -> CIC -> J -> output */
#define AD9083_DATAPATH_ADC_J 4				/*!< ADC -> J -> output */
#define AD9083_DATAPATH_ADC_CIC_NCO_G 5			/*!< ADC -> CIC -> NCO -> G -> output */
#define AD9083_DATAPATH_ADC_CIC_NCO_G_H 6		/*!< ADC -> CIC -> NCO -> G -> H output */

#endif
