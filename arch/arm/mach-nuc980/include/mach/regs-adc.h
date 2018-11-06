/*
 * arch/arm/mach-nuc980/include/mach/regs-adc.h
 *
 * Copyright (c) 2017 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#ifndef __ASM_ARCH_REGS_ADC_H
#define __ASM_ARCH_REGS_ADC_H

#define ADC_BA			NUC980_VA_ADC	/* ADC Control */

#define REG_ADC_CTL		(ADC_BA + 0x000)	/* ADC Control  */
#define REG_ADC_CONF		(ADC_BA + 0x004)	/* ADC Configure  */
#define REG_ADC_IER		(ADC_BA + 0x008)	/* ADC Interrupt Enable Register */
#define REG_ADC_ISR		(ADC_BA + 0x00C)	/* ADC Interrupt Status Register */
#define REG_ADC_WKISR		(ADC_BA + 0x010)	/* ADC Wake Up Interrupt Status Register */
#define REG_ADC_XYDATA		(ADC_BA + 0x020)	/* ADC Touch X,Y Position Data  */
#define REG_ADC_ZDATA		(ADC_BA + 0x024)	/* ADC Touch Z Pressure Data  */
#define REG_ADC_DATA		(ADC_BA + 0x028)	/* ADC Normal Conversion Data  */
#define REG_ADC_VBADATA		(ADC_BA + 0x02C)	/* ADC Battery Detection Data  */
#define REG_ADC_KPDATA		(ADC_BA + 0x030)	/* ADC Battery Detection Data  */
#define REG_ADC_SELFDATA	(ADC_BA + 0x034)	/* ADC Self-Test Data  */

#endif /*  __ASM_ARCH_REGS_CAP_H */
