/*
 * arch/arm/mach-nuc980/include/mach/regs-pwm1.h
 *
 * Copyright (c) 2018 Nuvoton Technology Corp.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_REGS_PWM1_H
#define __ASM_ARCH_REGS_PWM1_H

/* PWM Registers */

#define PWM1_BA			NUC980_VA_PWM1
#define REG_PWM_PPR		(PWM1_BA+0x00)
#define REG_PWM_CSR		(PWM1_BA+0x04)
#define REG_PWM_PCR		(PWM1_BA+0x08)
#define REG_PWM_CNR0		(PWM1_BA+0x0C)
#define REG_PWM_CMR0		(PWM1_BA+0x10)
#define REG_PWM_PDR0		(PWM1_BA+0x14)
#define REG_PWM_CNR1		(PWM1_BA+0x18)
#define REG_PWM_CMR1		(PWM1_BA+0x1C)
#define REG_PWM_PDR1		(PWM1_BA+0x20)
#define REG_PWM_CNR2		(PWM1_BA+0x24)
#define REG_PWM_CMR2		(PWM1_BA+0x28)
#define REG_PWM_PDR2		(PWM1_BA+0x2C)
#define REG_PWM_CNR3		(PWM1_BA+0x30)
#define REG_PWM_CMR3		(PWM1_BA+0x34)
#define REG_PWM_PDR3		(PWM1_BA+0x38)
#define REG_PWM_PIER		(PWM1_BA+0x3C)
#define REG_PWM_PIIR		(PWM1_BA+0x40)

#endif /*  __ASM_ARCH_REGS_PWM1_H */
