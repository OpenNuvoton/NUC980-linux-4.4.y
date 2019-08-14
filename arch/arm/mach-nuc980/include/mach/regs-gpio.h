/*
 * arch/arm/mach-nuc980/include/mach/regs-gpio.h
 *
 * Copyright (c) 2017 Nuvoton Technology Corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_REGS_GPIO_H
#define __ASM_ARCH_REGS_GPIO_H

/* Global control registers */
#define GPIO_BA     NUC980_VA_GPIO

#define REG_GPIOA_MODE      (GPIO_BA+0x000)
#define REG_GPIOA_DINOFF    (GPIO_BA+0x004)
#define REG_GPIOA_DOUT      (GPIO_BA+0x008)
#define REG_GPIOA_DATMSK    (GPIO_BA+0x00C)
#define REG_GPIOA_PIN       (GPIO_BA+0x010)
#define REG_GPIOA_DBEN      (GPIO_BA+0x014)
#define REG_GPIOA_INTTYPE (GPIO_BA+0x018)
#define REG_GPIOA_INTEN     (GPIO_BA+0x01C)
#define REG_GPIOA_INTSRC    (GPIO_BA+0x020)
#define REG_GPIOA_SMTEN     (GPIO_BA+0x024)
#define REG_GPIOA_SLEWCTL (GPIO_BA+0x028)
#define REG_GPIOA_PUSEL     (GPIO_BA+0x030)

#define REG_GPIOB_MODE      (GPIO_BA+0x040)
#define REG_GPIOB_DINOFF    (GPIO_BA+0x044)
#define REG_GPIOB_DOUT      (GPIO_BA+0x048)
#define REG_GPIOB_DATMSK    (GPIO_BA+0x04C)
#define REG_GPIOB_PIN       (GPIO_BA+0x050)
#define REG_GPIOB_DBEN      (GPIO_BA+0x054)
#define REG_GPIOB_INTTYPE (GPIO_BA+0x058)
#define REG_GPIOB_INTEN     (GPIO_BA+0x05C)
#define REG_GPIOB_INTSRC    (GPIO_BA+0x060)
#define REG_GPIOB_SMTEN     (GPIO_BA+0x064)
#define REG_GPIOB_SLEWCTL (GPIO_BA+0x068)
#define REG_GPIOB_PUSEL     (GPIO_BA+0x070)

#define REG_GPIOC_MODE      (GPIO_BA+0x080)
#define REG_GPIOC_DINOFF    (GPIO_BA+0x084)
#define REG_GPIOC_DOUT      (GPIO_BA+0x088)
#define REG_GPIOC_DATMSK    (GPIO_BA+0x08C)
#define REG_GPIOC_PIN       (GPIO_BA+0x090)
#define REG_GPIOC_DBEN      (GPIO_BA+0x094)
#define REG_GPIOC_INTTYPE (GPIO_BA+0x098)
#define REG_GPIOC_INTEN     (GPIO_BA+0x09C)
#define REG_GPIOC_INTSRC    (GPIO_BA+0x0A0)
#define REG_GPIOC_SMTEN     (GPIO_BA+0x0A4)
#define REG_GPIOC_SLEWCTL (GPIO_BA+0x0A8)
#define REG_GPIOC_PUSEL     (GPIO_BA+0x0B0)

#define REG_GPIOD_MODE      (GPIO_BA+0x0C0)
#define REG_GPIOD_DINOFF    (GPIO_BA+0x0C4)
#define REG_GPIOD_DOUT      (GPIO_BA+0x0C8)
#define REG_GPIOD_DATMSK    (GPIO_BA+0x0CC)
#define REG_GPIOD_PIN       (GPIO_BA+0x0D0)
#define REG_GPIOD_DBEN      (GPIO_BA+0x0D4)
#define REG_GPIOD_INTTYPE (GPIO_BA+0x0D8)
#define REG_GPIOD_INTEN     (GPIO_BA+0x0DC)
#define REG_GPIOD_INTSRC    (GPIO_BA+0x0E0)
#define REG_GPIOD_SMTEN     (GPIO_BA+0x0E4)
#define REG_GPIOD_SLEWCTL (GPIO_BA+0x0E8)
#define REG_GPIOD_PUSEL     (GPIO_BA+0x0F0)

#define REG_GPIOE_MODE      (GPIO_BA+0x100)
#define REG_GPIOE_DINOFF    (GPIO_BA+0x104)
#define REG_GPIOE_DOUT      (GPIO_BA+0x108)
#define REG_GPIOE_DATMSK    (GPIO_BA+0x10C)
#define REG_GPIOE_PIN       (GPIO_BA+0x110)
#define REG_GPIOE_DBEN      (GPIO_BA+0x114)
#define REG_GPIOE_INTTYPE (GPIO_BA+0x118)
#define REG_GPIOE_INTEN     (GPIO_BA+0x11C)
#define REG_GPIOE_INTSRC    (GPIO_BA+0x120)
#define REG_GPIOE_SMTEN     (GPIO_BA+0x124)
#define REG_GPIOE_SLEWCTL (GPIO_BA+0x128)
#define REG_GPIOE_PUSEL     (GPIO_BA+0x130)

#define REG_GPIOF_MODE      (GPIO_BA+0x140)
#define REG_GPIOF_DINOFF    (GPIO_BA+0x144)
#define REG_GPIOF_DOUT      (GPIO_BA+0x148)
#define REG_GPIOF_DATMSK    (GPIO_BA+0x14C)
#define REG_GPIOF_PIN       (GPIO_BA+0x150)
#define REG_GPIOF_DBEN      (GPIO_BA+0x154)
#define REG_GPIOF_INTTYPE (GPIO_BA+0x158)
#define REG_GPIOF_INTEN     (GPIO_BA+0x15C)
#define REG_GPIOF_INTSRC    (GPIO_BA+0x160)
#define REG_GPIOF_SMTEN     (GPIO_BA+0x164)
#define REG_GPIOF_SLEWCTL (GPIO_BA+0x168)
#define REG_GPIOF_PUSEL     (GPIO_BA+0x170)

#define REG_GPIOG_MODE      (GPIO_BA+0x180)
#define REG_GPIOG_DINOFF    (GPIO_BA+0x184)
#define REG_GPIOG_DOUT      (GPIO_BA+0x188)
#define REG_GPIOG_DATMSK    (GPIO_BA+0x18C)
#define REG_GPIOG_PIN       (GPIO_BA+0x190)
#define REG_GPIOG_DBEN      (GPIO_BA+0x194)
#define REG_GPIOG_INTTYPE (GPIO_BA+0x198)
#define REG_GPIOG_INTEN     (GPIO_BA+0x19C)
#define REG_GPIOG_INTSRC    (GPIO_BA+0x1A0)
#define REG_GPIOG_SMTEN     (GPIO_BA+0x1A4)
#define REG_GPIOG_SLEWCTL (GPIO_BA+0x1A8)
#define REG_GPIOG_PUSEL     (GPIO_BA+0x1B0)

#define REG_GPIO_DBNCECON   (GPIO_BA+0x440)

#define GPIO_PIN_DATA_BASE (GPIO_BA+0x800)
/* Define GPIO Pin Data Input/Output. It could be used to control each I/O pin by pin address mapping. */
#define GPIO_PIN_DATA(port, pin)    (*((volatile uint32_t *)((GPIO_PIN_DATA_BASE+(0x40*(port))) + ((pin)<<2))))


#define GPIO_OFFSET 0x20
#define DRIVER_NAME "nuc980-gpio"
#define NUMGPIO 0x20 * 7    //(PortA~PortG)

#endif /*  __ASM_ARCH_REGS_GPIO_H */
