/*
 *  Nuvoton NUC980 GPIO API definitions
 *
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 */

#ifndef __ASM_MACH_NUC980_GPIO_H
#define __ASM_MACH_NUC980_GPIO_H

#define ARCH_NR_GPIOS		512
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include <asm-generic/gpio.h>

#define	NUC980_PA0	(0x00 + 0)
#define	NUC980_PA1	(0x00 + 1)
#define	NUC980_PA2	(0x00 + 2)
#define	NUC980_PA3	(0x00 + 3)
#define	NUC980_PA4	(0x00 + 4)
#define	NUC980_PA5	(0x00 + 5)
#define	NUC980_PA6	(0x00 + 6)
#define	NUC980_PA7	(0x00 + 7)
#define	NUC980_PA8	(0x00 + 8)
#define	NUC980_PA9	(0x00 + 9)
#define	NUC980_PA10	(0x00 + 10)
#define	NUC980_PA11	(0x00 + 11)
#define	NUC980_PA12	(0x00 + 12)
#define	NUC980_PA13	(0x00 + 13)
#define	NUC980_PA14	(0x00 + 14)
#define	NUC980_PA15	(0x00 + 15)

#define	NUC980_PB0	(0x20 + 0)
#define	NUC980_PB1	(0x20 + 1)
#define	NUC980_PB2	(0x20 + 2)
#define	NUC980_PB3	(0x20 + 3)
#define	NUC980_PB4	(0x20 + 4)
#define	NUC980_PB5	(0x20 + 5)
#define	NUC980_PB6	(0x20 + 6)
#define	NUC980_PB7	(0x20 + 7)
#define	NUC980_PB8	(0x20 + 8)
#define	NUC980_PB9	(0x20 + 9)
#define	NUC980_PB10	(0x20 + 10)
#define	NUC980_PB11	(0x20 + 11)
#define	NUC980_PB12	(0x20 + 12)
#define	NUC980_PB13	(0x20 + 13)
#define	NUC980_PB14	(0x20 + 14)
#define	NUC980_PB15	(0x20 + 15)

#define	NUC980_PC0	(0x40 + 0)
#define	NUC980_PC1	(0x40 + 1)
#define	NUC980_PC2	(0x40 + 2)
#define	NUC980_PC3	(0x40 + 3)
#define	NUC980_PC4	(0x40 + 4)
#define	NUC980_PC5	(0x40 + 5)
#define	NUC980_PC6	(0x40 + 6)
#define	NUC980_PC7	(0x40 + 7)
#define	NUC980_PC8	(0x40 + 8)
#define	NUC980_PC9	(0x40 + 9)
#define	NUC980_PC10	(0x40 + 10)
#define	NUC980_PC11	(0x40 + 11)
#define	NUC980_PC12	(0x40 + 12)
#define	NUC980_PC13	(0x40 + 13)
#define	NUC980_PC14	(0x40 + 14)
#define	NUC980_PC15	(0x40 + 15)

#define	NUC980_PD0	(0x60 + 0)
#define	NUC980_PD1	(0x60 + 1)
#define	NUC980_PD2	(0x60 + 2)
#define	NUC980_PD3	(0x60 + 3)
#define	NUC980_PD4	(0x60 + 4)
#define	NUC980_PD5	(0x60 + 5)
#define	NUC980_PD6	(0x60 + 6)
#define	NUC980_PD7	(0x60 + 7)
#define	NUC980_PD8	(0x60 + 8)
#define	NUC980_PD9	(0x60 + 9)
#define	NUC980_PD10	(0x60 + 10)
#define	NUC980_PD11	(0x60 + 11)
#define	NUC980_PD12	(0x60 + 12)
#define	NUC980_PD13	(0x60 + 13)
#define	NUC980_PD14	(0x60 + 14)
#define	NUC980_PD15	(0x60 + 15)

#define	NUC980_PE0	(0x80 + 0)
#define	NUC980_PE1	(0x80 + 1)
#define	NUC980_PE2	(0x80 + 2)
#define	NUC980_PE3	(0x80 + 3)
#define	NUC980_PE4	(0x80 + 4)
#define	NUC980_PE5	(0x80 + 5)
#define	NUC980_PE6	(0x80 + 6)
#define	NUC980_PE7	(0x80 + 7)
#define	NUC980_PE8	(0x80 + 8)
#define	NUC980_PE9	(0x80 + 9)
#define	NUC980_PE10	(0x80 + 10)
#define	NUC980_PE11	(0x80 + 11)
#define	NUC980_PE12	(0x80 + 12)
#define	NUC980_PE13	(0x80 + 13)
#define	NUC980_PE14	(0x80 + 14)
#define	NUC980_PE15	(0x80 + 15)

#define	NUC980_PF0	(0xA0 + 0)
#define	NUC980_PF1	(0xA0 + 1)
#define	NUC980_PF2	(0xA0 + 2)
#define	NUC980_PF3	(0xA0 + 3)
#define	NUC980_PF4	(0xA0 + 4)
#define	NUC980_PF5	(0xA0 + 5)
#define	NUC980_PF6	(0xA0 + 6)
#define	NUC980_PF7	(0xA0 + 7)
#define	NUC980_PF8	(0xA0 + 8)
#define	NUC980_PF9	(0xA0 + 9)
#define	NUC980_PF10	(0xA0 + 10)
#define	NUC980_PF11	(0xA0 + 11)
#define	NUC980_PF12	(0xA0 + 12)
#define	NUC980_PF13	(0xA0 + 13)
#define	NUC980_PF14	(0xA0 + 14)
#define	NUC980_PF15	(0xA0 + 15)

#define	NUC980_PG0	(0xC0 + 0)
#define	NUC980_PG1	(0xC0 + 1)
#define	NUC980_PG2	(0xC0 + 2)
#define	NUC980_PG3	(0xC0 + 3)
#define	NUC980_PG4	(0xC0 + 4)
#define	NUC980_PG5	(0xC0 + 5)
#define	NUC980_PG6	(0xC0 + 6)
#define	NUC980_PG7	(0xC0 + 7)
#define	NUC980_PG8	(0xC0 + 8)
#define	NUC980_PG9	(0xC0 + 9)
#define	NUC980_PG10	(0xC0 + 10)
#define	NUC980_PG11	(0xC0 + 11)
#define	NUC980_PG12	(0xC0 + 12)
#define	NUC980_PG13	(0xC0 + 13)
#define	NUC980_PG14	(0xC0 + 14)
#define	NUC980_PG15	(0xC0 + 15)

typedef struct nuc980_eint_pins{
	u32	pin;
	irq_handler_t handler;
        u32   trigger;
        char *name;
}eint_wakeup_pins;



#endif /* __ASM_MACH_NUC980_GPIO_H*/
