/*
 * arch/arm/mach-nuc980/include/mach/regs-clock.h
 *
 * Copyright (c) 2017 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_REGS_AIC_H
#define __ASM_ARCH_REGS_AIC_H

/* Advance Interrupt Controller (AIC) Registers */
#define		AIC_BA		NUC980_VA_IRQ	/* Interrupt Controller */
#define		REG_AIC_SRCCTL0		(AIC_BA+0x00)	/* Source control register 0 */
#define		REG_AIC_SRCCTL1		(AIC_BA+0x04)	/* Source control register 1 */
#define		REG_AIC_SRCCTL2		(AIC_BA+0x08)	/* Source control register 2 */
#define		REG_AIC_SRCCTL3		(AIC_BA+0x0C)	/* Source control register 3 */
#define		REG_AIC_SRCCTL4		(AIC_BA+0x10)	/* Source control register 4 */
#define		REG_AIC_SRCCTL5		(AIC_BA+0x14)	/* Source control register 5 */
#define		REG_AIC_SRCCTL6		(AIC_BA+0x18)	/* Source control register 6 */
#define		REG_AIC_SRCCTL7		(AIC_BA+0x1C)	/* Source control register 7 */
#define		REG_AIC_SRCCTL8		(AIC_BA+0x20)	/* Source control register 8 */
#define		REG_AIC_SRCCTL9		(AIC_BA+0x24)	/* Source control register 9 */
#define		REG_AIC_SRCCTL10	(AIC_BA+0x28)	/* Source control register 10 */
#define		REG_AIC_SRCCTL11	(AIC_BA+0x2C)	/* Source control register 11 */
#define		REG_AIC_SRCCTL12	(AIC_BA+0x30)	/* Source control register 12 */
#define		REG_AIC_SRCCTL13	(AIC_BA+0x34)	/* Source control register 13 */
#define		REG_AIC_SRCCTL14	(AIC_BA+0x38)	/* Source control register 14 */
#define		REG_AIC_SRCCTL15	(AIC_BA+0x3C)	/* Source control register 15 */
#define		REG_AIC_IRQNUM		(AIC_BA+0x120)	/* IRQ source number register */
#define		REG_AIC_FIQNUM		(AIC_BA+0x124)	/* FIQ source number register */
#define		REG_AIC_INTMSK0		(AIC_BA+0x128)	/* Interrupt mask register 0 */
#define		REG_AIC_INTMSK1		(AIC_BA+0x12C)	/* Interrupt mask register 1 */
#define		REG_AIC_INTEN0		(AIC_BA+0x130)	/* Mask enable command register 0 */
#define		REG_AIC_INTEN1		(AIC_BA+0x134)	/* Mask enable command register 1 */
#define		REG_AIC_INTDIS0		(AIC_BA+0x138)	/* Mask disable command register 0 */
#define		REG_AIC_INTDIS1		(AIC_BA+0x13C)	/* Mask disable command register 1 */
#define		REG_AIC_EOIS		(AIC_BA+0x150)	/* End of IRQ service command register */
#define		REG_AIC_EOFS		(AIC_BA+0x154)	/* End of FIQ service command register */


#define		AIC_IRQNUM		0x120
#endif /*  __ASM_ARCH_REGS_AIC_H */
