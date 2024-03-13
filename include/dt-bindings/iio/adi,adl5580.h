/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _DT_BINDINGS_ADI_ADL5580_H
#define _DT_BINDINGS_ADI_ADL5580_H

/*
 * adi,prg-itrm:	Enumerates Input Common Mode Voltage.
 *			Register: GEN_CTL0
 *			BitField: [3:2] PRG_ITRM_1P8V
 */
#define ADL5580_IN_CM_1P45V 0x00
#define ADL5580_IN_CM_1P60V 0x01
#define ADL5580_IN_CM_1P75V 0x02 /* Default */
#define ADL5580_IN_CM_1P90V 0x03

/*
 * adi,prg-otrm:	Enumerates Output Common Mode Voltage.
 *			Register: GEN_CTL0
 *			BitField: [7:6] PRG_OTRM_1P8V
 */
#define ADL5580_OUT_CM_0P4V 0x00
#define ADL5580_OUT_CM_0P5V 0x01 /* Default */
#define ADL5580_OUT_CM_0P6V 0x02
#define ADL5580_OUT_CM_0P7V 0x03

/*
 * adi,ms-itrm:		Enumerates Input Termination Mode.
 *			Register: GEN_CTL0
 *			BitField: [1:0] MS_ITRM_1P8V
 */
#define ADL5580_IN_TERM_MODE0 0x00	/*!< Mode 0: Internal VCM disabled, VCMI pin disconnected. Default. */
#define ADL5580_IN_TERM_MODE1 0x01	/*!< Mode 1: Internal VCM enabled, VCMI pin disconnected. */
#define ADL5580_IN_TERM_MODE2 0x02	/*!< Mode 2: Internal VCM enabled, VCMI export. */
#define ADL5580_IN_TERM_MODE3 0x03	/*!< Mode 3: Internal VCM disabled, VCMI import. */

/*
 * adi,ms-otrm:		Enumerates Output Termination Mode.
 *			Register: GEN_CTL0
 *			BitField: [5:4] MS_OTRM_1P8V
 */
#define ADL5580_OUT_TERM_MODE0 0x00	/*!< Mode 0: Internal VCM disabled, VCMO pin disconnected. */
#define ADL5580_OUT_TERM_MODE1 0x01	/*!< Mode 1: Internal VCM enabled, VCMO pin disconnected. */
#define ADL5580_OUT_TERM_MODE2 0x02	/*!< Mode 2: Internal VCM enabled, VCMO export. */
#define ADL5580_OUT_TERM_MODE3 0x03	/*!< Mode 3: Internal VCM disabled, VCMO import. Default. */

#endif /* _DT_BINDINGS_ADI_ADL5580_H */
