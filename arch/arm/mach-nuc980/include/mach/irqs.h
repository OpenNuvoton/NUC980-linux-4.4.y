/*
 * arch/arm/mach-nuc980/include/mach/irqs.h
 *
 * Copyright (c) 2017 Nuvoton technology corporation
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

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/*
 * we keep the first set of CPU IRQs out of the range of
 * the ISA space, so that the PC104 has them to itself
 * and we don't end up having to do horrible things to the
 * standard ISA drivers....
 *
 */

#define NUC980_IRQ(x)	(x)

/* Main cpu interrupts */

#define IRQ_WDT		NUC980_IRQ(1)
#define IRQ_WWDT	NUC980_IRQ(2)
#define IRQ_LVD		NUC980_IRQ(3)
#define IRQ_EXT0	NUC980_IRQ(4)
#define IRQ_EXT1	NUC980_IRQ(5)
#define IRQ_EXT2	NUC980_IRQ(6)
#define IRQ_EXT3	NUC980_IRQ(7)
#define IRQ_GPA		NUC980_IRQ(8)
#define IRQ_GPB		NUC980_IRQ(9)
#define IRQ_GPC		NUC980_IRQ(10)
#define IRQ_GPD		NUC980_IRQ(11)
#define IRQ_I2S		NUC980_IRQ(12)

#define IRQ_CAP0	NUC980_IRQ(14)
#define IRQ_RTC		NUC980_IRQ(15)
#define IRQ_TIMER0	NUC980_IRQ(16)
#define IRQ_TIMER1	NUC980_IRQ(17)
#define IRQ_ADC		NUC980_IRQ(18)
#define IRQ_EMC0RX	NUC980_IRQ(19)
#define IRQ_EMC1RX	NUC980_IRQ(20)
#define IRQ_EMC0TX	NUC980_IRQ(21)
#define IRQ_EMC1TX	NUC980_IRQ(22)
#define IRQ_EHCI	NUC980_IRQ(23)
#define IRQ_OHCI	NUC980_IRQ(24)
#define IRQ_PDMA0	NUC980_IRQ(25)
#define IRQ_PDMA1	NUC980_IRQ(26)
#define IRQ_SDH		NUC980_IRQ(27)
#define IRQ_FMI		NUC980_IRQ(28)
#define IRQ_UDC		NUC980_IRQ(29)
#define IRQ_TIMER2	NUC980_IRQ(30)
#define IRQ_TIMER3	NUC980_IRQ(31)
#define IRQ_TIMER4	NUC980_IRQ(32)
#define IRQ_CAP1	NUC980_IRQ(33)
#define IRQ_TIMER5	NUC980_IRQ(34)
#define IRQ_CRYPTO	NUC980_IRQ(35)
#define IRQ_UART0	NUC980_IRQ(36)
#define IRQ_UART1	NUC980_IRQ(37)
#define IRQ_UART2	NUC980_IRQ(38)
#define IRQ_UART4	NUC980_IRQ(39)
#define IRQ_UART6	NUC980_IRQ(40)
#define IRQ_UART8	NUC980_IRQ(41)
#define IRQ_CAN3	NUC980_IRQ(42)
#define IRQ_UART3	NUC980_IRQ(43)
#define IRQ_UART5	NUC980_IRQ(44)
#define IRQ_UART7	NUC980_IRQ(45)
#define IRQ_UART9	NUC980_IRQ(46)
#define IRQ_I2C2	NUC980_IRQ(47)
#define IRQ_I2C3	NUC980_IRQ(48)
#define IRQ_GPE		NUC980_IRQ(49)
#define IRQ_SPI2	NUC980_IRQ(50)
#define IRQ_SPI0	NUC980_IRQ(51)
#define IRQ_SPI1	NUC980_IRQ(52)
#define IRQ_I2C0	NUC980_IRQ(53)
#define IRQ_I2C1	NUC980_IRQ(54)
#define IRQ_SMC0	NUC980_IRQ(55)
#define IRQ_SMC1	NUC980_IRQ(56)
#define IRQ_GPF		NUC980_IRQ(57)
#define IRQ_CAN0	NUC980_IRQ(58)
#define IRQ_CAN1	NUC980_IRQ(59)
#define IRQ_PWM0	NUC980_IRQ(60)
#define IRQ_PWM1	NUC980_IRQ(61)
#define IRQ_CAN2	NUC980_IRQ(62)
#define IRQ_GPG		NUC980_IRQ(63)

#ifndef CONFIG_GPIO_NUC980
#define SPARE_IRQS	(64)
#define NR_IRQS		(IRQ_GPG + SPARE_IRQS + 1)

#else
#define EXT0_BASE 100
#define IRQ_EXT0_A0           NUC980_IRQ(EXT0_BASE + 0)
#define IRQ_EXT1_A1           NUC980_IRQ(EXT0_BASE + 1)
#define IRQ_EXT2_D0           NUC980_IRQ(EXT0_BASE + 2)
#define IRQ_EXT3_D1           NUC980_IRQ(EXT0_BASE + 3)

#define IRQ_EXT0_A13          NUC980_IRQ(EXT0_BASE + 4)
#define IRQ_EXT1_A14          NUC980_IRQ(EXT0_BASE + 5)
#define IRQ_EXT2_E10          NUC980_IRQ(EXT0_BASE + 6)
#define IRQ_EXT3_E12          NUC980_IRQ(EXT0_BASE + 7)

#define IRQ_EXT2_B3           NUC980_IRQ(EXT0_BASE + 8)
#define IRQ_EXT2_B13          NUC980_IRQ(EXT0_BASE + 9)
#define IRQ_EXT3_G15          NUC980_IRQ(EXT0_BASE +10)

#define IRQ_GPIO_START	NUC980_IRQ(NUC980_IRQ(0x100))
#define IRQ_GPIO_END		NUC980_IRQ(NUC980_IRQ(0x100 + 0xE0))

#define SPARE_IRQS		(64)

#define NR_IRQS			(IRQ_GPIO_END + SPARE_IRQS +1)

#endif

#endif /* __ASM_ARCH_IRQ_H */
