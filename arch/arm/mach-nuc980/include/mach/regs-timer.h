/*
 * arch/arm/mach-nuc980/include/mach/regs-timer.h
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

#ifndef __ASM_ARCH_REGS_TIMER_H
#define __ASM_ARCH_REGS_TIMER_H

#define TIMER0			NUC980_VA_TIMER01
#define TIMER1			(NUC980_VA_TIMER01 + 0x100)
#define TIMER2			NUC980_VA_TIMER23
#define TIMER3			(NUC980_VA_TIMER23 + 0x100)
#define TIMER4			NUC980_VA_TIMER45
#define TIMER5			(NUC980_VA_TIMER45 + 0x100)

#define REG_TIMER_CTL(x)	(void __iomem *)((x) + 0x00)
#define REG_TIMER_PRECNT(x)	(void __iomem *)((x) + 0x04)
#define REG_TIMER_CMPR(x)	(void __iomem *)((x) + 0x08)
#define REG_TIMER_IER(x)	(void __iomem *)((x) + 0x0C)
#define REG_TIMER_ISR(x)	(void __iomem *)((x) + 0x10)
#define REG_TIMER_DR(x)		(void __iomem *)((x) + 0x14)
#define REG_TIMER_TCAP(x)	(void __iomem *)((x) + 0x18)
#define REG_TIMER_ECTL(x)	(void __iomem *)((x) + 0x20)

#endif /*  __ASM_ARCH_REGS_TIMER_H */
