/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device-tree bindings for the Analog Devices ADSY1100 Apollo SOM
 * "Nyx" RF front-end controller.
 *
 * Copyright 2026 Analog Devices Inc.
 */
#ifndef _DT_BINDINGS_IIO_LOGIC_ADI_ADSY1100_NYX_H
#define _DT_BINDINGS_IIO_LOGIC_ADI_ADSY1100_NYX_H

/*
 * Filter band selection used with the adi,filter-mode property of the
 * per-channel dac@N / adc@N subnodes.
 */
#define ADSY1100_NYX_FILTER_LOW_BAND	0
#define ADSY1100_NYX_FILTER_THRU	1
#define ADSY1100_NYX_FILTER_X_BAND	2
#define ADSY1100_NYX_FILTER_KU_BAND	3

#endif /* _DT_BINDINGS_IIO_LOGIC_ADI_ADSY1100_NYX_H */
