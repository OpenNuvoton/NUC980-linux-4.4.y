/*
 * linux/arch/arm/mach-nuc980/irq.c
 *
 * Copyright (c) 2017 Nuvoton technology corporation
 * All rights reserved.
 *
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/regs-aic.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <asm/gpio.h>

#if 0
#define ENTRY()                 printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()                 printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif

static void nuc980_irq_mask(struct irq_data *d)
{
	if (d->irq < 32)
		__raw_writel(1 << (d->irq), REG_AIC_INTDIS0);
	else
		__raw_writel(1 << (d->irq - 32), REG_AIC_INTDIS1);
}

static void nuc980_irq_ack(struct irq_data *d)
{
	__raw_writel(0x01, REG_AIC_EOIS);
}

static void nuc980_irq_unmask(struct irq_data *d)
{
	if (d->irq < 32)
		__raw_writel(1 << (d->irq), REG_AIC_INTEN0);
	else
		__raw_writel(1 << (d->irq - 32), REG_AIC_INTEN1);
}

static int nuc980_irq_set_wake(struct irq_data *d, unsigned int on)
{
	return 0;
}

static struct irq_chip nuc980_irq_chip = {
	.irq_disable = nuc980_irq_mask,
	.irq_enable = nuc980_irq_unmask,
	.irq_ack = nuc980_irq_ack,
	.irq_mask = nuc980_irq_mask,
	.irq_unmask = nuc980_irq_unmask,
	.irq_set_wake = nuc980_irq_set_wake,
};

#if defined(CONFIG_GPIO_NUC980)

static const unsigned int Port[7] = {
	(unsigned int)REG_GPIOA_MODE,
	(unsigned int)REG_GPIOB_MODE,
	(unsigned int)REG_GPIOC_MODE,
	(unsigned int)REG_GPIOD_MODE,
	(unsigned int)REG_GPIOE_MODE,
	(unsigned int)REG_GPIOF_MODE,
	(unsigned int)REG_GPIOG_MODE
};

static unsigned int gpio_type[8];
static unsigned int gpio_inten[8];

static void nuc980_irq_gpio_mask(struct irq_data *d)
{
	unsigned int port,num;
	ENTRY();
	port =(d->irq-IRQ_GPIO_START)/GPIO_OFFSET;
	num  =(d->irq-IRQ_GPIO_START)%GPIO_OFFSET;
	__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x1C)) &~(0x1<<(num+16)),(volatile unsigned int *)(Port[port]+0x1C));
	__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x1C)) &~(0x1<<num),(volatile unsigned int *)(Port[port]+0x1C));
	LEAVE();
}

static void nuc980_irq_gpio_ack(struct irq_data *d)
{
	ENTRY();
	__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}

static void nuc980_irq_gpio_unmask(struct irq_data *d)
{
	unsigned int port,num,tmp;
	ENTRY();
	port =(d->irq-IRQ_GPIO_START)/GPIO_OFFSET;
	num  =(d->irq-IRQ_GPIO_START)%GPIO_OFFSET;
	tmp = gpio_type[port] & (0x1<<(num));
	__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x18))|tmp,(volatile unsigned int *)(Port[port]+0x18));
	tmp = gpio_inten[port] & (0x1<<(num+16));
	__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x1C))|tmp,(volatile unsigned int *)(Port[port]+0x1C));
	tmp = gpio_inten[port] & (0x1<<num);
	__raw_writel(__raw_readl((volatile unsigned int *)(Port[port]+0x1C))|tmp,(volatile unsigned int *)(Port[port]+0x1C));
	LEAVE();

}

static int nuc980_irq_gpio_type(struct irq_data *d, unsigned int type)
{
	unsigned int port, num;
	ENTRY();
	port = (d->irq - IRQ_GPIO_START) / GPIO_OFFSET;
	num = (d->irq - IRQ_GPIO_START) % GPIO_OFFSET;

	if (type == IRQ_TYPE_PROBE) {
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x18)) & ~(0x1 << num),
		             (volatile unsigned int *)(Port[port] + 0x18));
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x1C)) | (0x1 << num) | ((0x1<<num)<<16),
		             (volatile unsigned int *)(Port[port] + 0x1C));
		gpio_type[port] &= ~(0x1<<num);
		gpio_inten[port] |= (0x1<<num);
		gpio_inten[port] |= (0x1<<(num+16));
		return 0;
	}
	if (type & IRQ_TYPE_LEVEL_MASK) {
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x18)) | (0x1 << num),
		             (volatile unsigned int *)(Port[port] + 0x18));
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x1C)) & ~((0x1 << num)|((0x1<<num)<<16)),
		             (volatile unsigned int *)(Port[port] + 0x1C));
		gpio_type[port] |= (0x1<<num);
		gpio_inten[port] &= ~(0x1<<num);
		gpio_inten[port] &= ~(0x1<<(num+16));
		if (type == IRQ_TYPE_LEVEL_HIGH) {
			__raw_writel(__raw_readl
			             ((volatile unsigned int *)(Port[port] +
			                                        0x1C)) | ((0x1 <<
			                                                num)<<16),
			             (volatile unsigned int *)(Port[port] +
			                                       0x1C));
			gpio_inten[port] |=(0x1<<(num+16));
			return 0;
		}
		if (type == IRQ_TYPE_LEVEL_LOW) {
			__raw_writel(__raw_readl
			             ((volatile unsigned int *)(Port[port] +
			                                        0x1C)) | (0x1 <<
			                                                num),
			             (volatile unsigned int *)(Port[port] +
			                                       0x1C));
			gpio_inten[port] |=(0x1<<num);
			return 0;
		}
	} else {
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x18)) & ~(0x1 << num),
		             (volatile unsigned int *)(Port[port] + 0x18));
		gpio_type[port] &= ~(0x1<<num);
		if (type & IRQ_TYPE_EDGE_RISING) {
			__raw_writel(__raw_readl
			             ((volatile unsigned int *)(Port[port] +
			                                        0x1C)) | ((0x1 <<
			                                                num)<<16),
			             (volatile unsigned int *)(Port[port] +
			                                       0x1C));
			gpio_inten[port] |=(0x1<<(num+16));
		} else {
			__raw_writel(__raw_readl
			             ((volatile unsigned int *)(Port[port] +
			                                        0x1C)) & ~((0x1<<
			                                                num)<<16),
			             (volatile unsigned int *)(Port[port] +
			                                       0x1C));
			gpio_inten[port] &= ~(0x1<<(num+16));
		}
		if (type & IRQ_TYPE_EDGE_FALLING) {
			__raw_writel(__raw_readl
			             ((volatile unsigned int *)(Port[port] +
			                                        0x1C)) | (0x1 <<
			                                                num),
			             (volatile unsigned int *)(Port[port] +
			                                       0x1C));
			gpio_inten[port] |= (0x1<<num);
		} else {
			__raw_writel(__raw_readl
			             ((volatile unsigned int *)(Port[port] +
			                                        0x1C)) & ~(0x1
			                                                <<
			                                                num),
			             (volatile unsigned int *)(Port[port] +
			                                       0x1C));
			gpio_inten[port] &= ~(0x1<<num);
		}
	}
	return 0;
}

static struct irq_chip nuc980_irq_gpio = {
	.name = "GPIO-IRQ",
	.irq_disable = nuc980_irq_gpio_mask,
	.irq_enable = nuc980_irq_gpio_unmask,
	.irq_ack = nuc980_irq_gpio_ack,
	.irq_mask = nuc980_irq_gpio_mask,
	.irq_unmask = nuc980_irq_gpio_unmask,
	.irq_set_type = nuc980_irq_gpio_type,
	.irq_set_wake = nuc980_irq_set_wake,
};

static void nuc980_irq_demux_intgroupA(struct irq_desc *desc)
{
	unsigned int j, isr;
	unsigned int flag=0;
	ENTRY();
	isr = __raw_readl(REG_GPIOA_INTSRC);
	__raw_writel(isr,REG_GPIOA_INTSRC);
	for (j = 0; j < 16; j++) {
		if (isr  & 0x1) {
			flag=1;
			generic_handle_irq(IRQ_GPIO_START +
			                   0 * 0x20 + j);
		}
		isr = isr >> 1;
	}
	if(flag==0)
		__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}
static void nuc980_irq_demux_intgroupB(struct irq_desc *desc)
{
	unsigned int j, isr;
	unsigned int flag=0;
	ENTRY();
	isr = __raw_readl(REG_GPIOB_INTSRC);
	__raw_writel(isr,REG_GPIOB_INTSRC);
	for (j = 0; j < 16; j++) {
		if (isr  & 0x1) {
			flag=1;
			generic_handle_irq(IRQ_GPIO_START +
			                   1 * 0x20 + j);
		}
		isr = isr >> 1;
	}
	if(flag==0)
		__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}
static void nuc980_irq_demux_intgroupC(struct irq_desc *desc)
{
	unsigned int j, isr;
	unsigned int flag=0;
	ENTRY();
	isr = __raw_readl(REG_GPIOC_INTSRC);
	__raw_writel(isr,REG_GPIOC_INTSRC);
	for (j = 0; j < 16; j++) {
		if (isr  & 0x1) {
			flag=1;
			generic_handle_irq(IRQ_GPIO_START +
			                   2 * 0x20 + j);
		}
		isr = isr >> 1;
	}
	if(flag==0)
		__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}
static void nuc980_irq_demux_intgroupD(struct irq_desc *desc)
{
	unsigned int j, isr;
	unsigned int flag=0;
	ENTRY();
	isr = __raw_readl(REG_GPIOD_INTSRC);
	__raw_writel(isr,REG_GPIOD_INTSRC);
	for (j = 0; j < 16; j++) {
		if (isr  & 0x1) {
			flag=1;
			generic_handle_irq(IRQ_GPIO_START +
			                   3 * 0x20 + j);
		}
		isr = isr >> 1;
	}
	if(flag==0)
		__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}
static void nuc980_irq_demux_intgroupE(struct irq_desc *desc)
{
	unsigned int j, isr;
	unsigned int flag=0;
	ENTRY();
	isr = __raw_readl(REG_GPIOE_INTSRC);
	__raw_writel(isr,REG_GPIOE_INTSRC);
	for (j = 0; j < 16; j++) {
		if (isr  & 0x1) {
			flag=1;
			generic_handle_irq(IRQ_GPIO_START +
			                   4 * 0x20 + j);
		}
		isr = isr >> 1;
	}
	if(flag==0)
		__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}
static void nuc980_irq_demux_intgroupF(struct irq_desc *desc)
{
	unsigned int j, isr;
	unsigned int flag=0;
	ENTRY();
	isr = __raw_readl(REG_GPIOF_INTSRC);
	__raw_writel(isr,REG_GPIOF_INTSRC);
	for (j = 0; j < 16; j++) {
		if (isr  & 0x1) {
			flag=1;
			generic_handle_irq(IRQ_GPIO_START +
			                   5 * 0x20 + j);
		}
		isr = isr >> 1;
	}
	if(flag==0)
		__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}
static void nuc980_irq_demux_intgroupG(struct irq_desc *desc)
{
	unsigned int j, isr;
	unsigned int flag=0;
	ENTRY();
	isr = __raw_readl(REG_GPIOG_INTSRC);
	__raw_writel(isr,REG_GPIOG_INTSRC);
	for (j = 0; j < 16; j++) {
		if (isr  & 0x1) {
			flag=1;
			generic_handle_irq(IRQ_GPIO_START +
			                   6 * 0x20 + j);
		}
		isr = isr >> 1;
	}
	if(flag==0)
		__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}

//------------------------------------------------------------------------------

static const unsigned int EXT[11]= {
	(unsigned int)NUC980_PA0,
	(unsigned int)NUC980_PA1,
	(unsigned int)NUC980_PD0,
	(unsigned int)NUC980_PD1,
	(unsigned int)NUC980_PA13,
	(unsigned int)NUC980_PA14,
	(unsigned int)NUC980_PE10,
	(unsigned int)NUC980_PE12,
	(unsigned int)NUC980_PB3,
	(unsigned int)NUC980_PB13,
	(unsigned int)NUC980_PG15,
};

static void nuc980_irq_ext_mask(struct irq_data *d)
{
	ENTRY();
	if(d->irq==IRQ_EXT0_A0 || d->irq==IRQ_EXT0_A13)
		__raw_writel(1<<IRQ_EXT0, REG_AIC_INTDIS0);
	else if(d->irq==IRQ_EXT1_A1 || d->irq==IRQ_EXT1_A14)
		__raw_writel(1 <<IRQ_EXT1, REG_AIC_INTDIS0);
	else if(d->irq==IRQ_EXT2_D0 || d->irq==IRQ_EXT2_E10 || IRQ_EXT2_B3 || IRQ_EXT2_B13)
		__raw_writel(1 <<IRQ_EXT2, REG_AIC_INTDIS0);
	else if(d->irq==IRQ_EXT3_D1 || d->irq==IRQ_EXT3_E12 || IRQ_EXT3_G15)
		__raw_writel(1 <<IRQ_EXT3, REG_AIC_INTDIS0);
	LEAVE();
}

static void nuc980_irq_ext_ack(struct irq_data *d)
{
	ENTRY();
	__raw_writel(0x01, REG_AIC_EOIS);
	LEAVE();
}

static void nuc980_irq_ext_unmask(struct irq_data *d)
{
	ENTRY();
	if(d->irq==IRQ_EXT0_A0 || d->irq==IRQ_EXT0_A13)
		__raw_writel(1<<IRQ_EXT0, REG_AIC_INTEN0);
	else if(d->irq==IRQ_EXT1_A1 || d->irq==IRQ_EXT1_A14)
		__raw_writel(1 <<IRQ_EXT1, REG_AIC_INTEN0);
	else if(d->irq==IRQ_EXT2_D0 || d->irq==IRQ_EXT2_E10 || IRQ_EXT2_B3 || IRQ_EXT2_B13)
		__raw_writel(1 <<IRQ_EXT2, REG_AIC_INTEN0);
	else if(d->irq==IRQ_EXT3_D1 || d->irq==IRQ_EXT3_E12 || IRQ_EXT3_G15)
		__raw_writel(1 <<IRQ_EXT3, REG_AIC_INTEN0);
	LEAVE();
}

static int nuc980_irq_ext_type(struct irq_data *d, unsigned int type)
{
	unsigned int port, num;
	ENTRY();
	port = (EXT[d->irq - EXT0_BASE]) / GPIO_OFFSET;
	num = (EXT[d->irq - EXT0_BASE]) % GPIO_OFFSET;
	if (type == IRQ_TYPE_PROBE) {
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x1C)) | (0x1 << (num+16)),
		             (volatile unsigned int *)(Port[port] + 0x1C));
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x1C)) | (0x1 << num),
		             (volatile unsigned int *)(Port[port] + 0x1C));
		return 0;
	}

	if (type & IRQ_TYPE_EDGE_RISING) {
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x1C)) | (0x1 << (num+16)),
		             (volatile unsigned int *)(Port[port] + 0x1C));
	} else
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x1C)) & ~(0x1 << (num+16)),
		             (volatile unsigned int *)(Port[port] + 0x1C));

	if (type & IRQ_TYPE_EDGE_FALLING) {
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x1C)) | (0x1 << num),
		             (volatile unsigned int *)(Port[port] + 0x1C));
	} else
		__raw_writel(__raw_readl
		             ((volatile unsigned int *)(Port[port] +
		                                        0x1C)) & ~(0x1 << num),
		             (volatile unsigned int *)(Port[port] + 0x1C));
	LEAVE();
	return 0;
}

static struct irq_chip nuc980_irq_ext = {
	.name = "EXT-IRQ",
	.irq_disable = nuc980_irq_ext_mask,
	.irq_enable = nuc980_irq_ext_unmask,
	.irq_ack = nuc980_irq_ext_ack,
	.irq_mask = nuc980_irq_ext_mask,
	.irq_unmask = nuc980_irq_ext_unmask,
	.irq_set_type = nuc980_irq_ext_type,
	.irq_set_wake = nuc980_irq_set_wake,
};

static void nuc980_irq_demux_intgroup2(struct irq_desc *desc)
{
	unsigned int port0, num0, port1, num1;
	unsigned int port2, num2, port3, num3;
	unsigned int irq = irq_desc_get_irq(desc);
	ENTRY();
	port0 = EXT[irq - 4] / GPIO_OFFSET;
	num0 = EXT[irq - 4] % GPIO_OFFSET;
	port1 = EXT[irq - 4 + 4] / GPIO_OFFSET;
	num1 = EXT[irq - 4 + 4] % GPIO_OFFSET;
	switch (irq) {
	case IRQ_EXT0:
		if (__raw_readl((volatile unsigned int *)(Port[port0] + 0x20)) &
		    (1 << num0)) {
			generic_handle_irq(IRQ_EXT0_A0);
			__raw_writel(0x1 << num0,
			             (volatile unsigned int *)(Port[port0] +
			                                       0x20));
		} else if (__raw_readl
		           ((volatile unsigned int *)(Port[port1] +
		                                      0x20)) & (1 << num1)) {
			generic_handle_irq(IRQ_EXT0_A13);
			__raw_writel(0x1 << num1,
			             (volatile unsigned int *)(Port[port1] +
			                                       0x20));
		} else
			__raw_writel(0x01, REG_AIC_EOIS);
		break;
	case IRQ_EXT1:
		if (__raw_readl((volatile unsigned int *)(Port[port0] + 0x20)) &
		    (1 << num0)) {
			generic_handle_irq(IRQ_EXT1_A1);
			__raw_writel(0x1 << num0,
			             (volatile unsigned int *)(Port[port0] +
			                                       0x20));
		} else if (__raw_readl
		           ((volatile unsigned int *)(Port[port1] +
		                                      0x20)) & (1 << num1)) {
			generic_handle_irq(IRQ_EXT1_A14);
			__raw_writel(0x20 << num1,
			             (volatile unsigned int *)(Port[port1] +
			                                       0x20));
		} else
			__raw_writel(0x01, REG_AIC_EOIS);
		break;
	case IRQ_EXT2:
		port2 = EXT[8] / GPIO_OFFSET;
		num2 = EXT[8] % GPIO_OFFSET;
		port3 = EXT[9] / GPIO_OFFSET;
		num3 = EXT[9] % GPIO_OFFSET;
		if (__raw_readl((volatile unsigned int *)(Port[port0] + 0x20)) &
		    (1 << num0)) {
			generic_handle_irq(IRQ_EXT2_D0);
			__raw_writel(0x1 << num0,
			             (volatile unsigned int *)(Port[port0] +
			                                       0x20));
		} else if (__raw_readl
		           ((volatile unsigned int *)(Port[port1] +
		                                      0x20)) & (1 << num1)) {
			generic_handle_irq(IRQ_EXT2_E10);
			__raw_writel(0x1 << num1,
			             (volatile unsigned int *)(Port[port1] +
			                                       0x20));
		} else if (__raw_readl
		           ((volatile unsigned int *)(Port[port2] +
		                                      0x20)) & (1 << num2)) {
			generic_handle_irq(IRQ_EXT2_B3);
			__raw_writel(0x1 << num2,
			             (volatile unsigned int *)(Port[port2] +
			                                       0x20));
		} else if (__raw_readl
		           ((volatile unsigned int *)(Port[port3] +
		                                      0x20)) & (1 << num3)) {
			generic_handle_irq(IRQ_EXT2_B13);
			__raw_writel(0x1 << num3,
			             (volatile unsigned int *)(Port[port3] +
			                                       0x20));
		} else
			__raw_writel(0x01, REG_AIC_EOIS);
		break;
	case IRQ_EXT3:
		port2 = EXT[10] / GPIO_OFFSET;
		num2 = EXT[10] % GPIO_OFFSET;
		if (__raw_readl((volatile unsigned int *)(Port[port0] + 0x20)) &
		    (1 << num0)) {
			generic_handle_irq(IRQ_EXT3_D1);
			__raw_writel(0x1 << num0,
			             (volatile unsigned int *)(Port[port0] +
			                                       0x20));
		} else if (__raw_readl
		           ((volatile unsigned int *)(Port[port1] +
		                                      0x20)) & (1 << num1)) {
			generic_handle_irq(IRQ_EXT3_E12);
			__raw_writel(0x1 << num1,
			             (volatile unsigned int *)(Port[port1] +
			                                       0x20));
		} else  if (__raw_readl
		            ((volatile unsigned int *)(Port[port2] +
		                                       0x20)) & (1 << num2)) {
			generic_handle_irq(IRQ_EXT3_G15);
			__raw_writel(0x1 << num2,
			             (volatile unsigned int *)(Port[port2] +
			                                       0x20));
		} else
			__raw_writel(0x01, REG_AIC_EOIS);
		break;
	}
	LEAVE();
}
#endif

void __init nuc980_init_irq(void)
{
#if !defined(CONFIG_USE_OF)

	int irqno;

	__raw_writel(0xFFFFFFFC, REG_AIC_INTDIS0);
	__raw_writel(0xFFFFFFFF, REG_AIC_INTDIS1);

	for (irqno = IRQ_WDT; irqno < NR_IRQS - SPARE_IRQS; irqno++) {
		irq_set_chip_and_handler(irqno, &nuc980_irq_chip,
		                         handle_level_irq);
		irq_clear_status_flags(irqno, IRQ_NOREQUEST);
	}

#if defined(CONFIG_GPIO_NUC980)
	ENTRY();
	/*
	 * Install handler for GPIO edge detect interrupts
	 */
	irq_set_chip(IRQ_GPA, &nuc980_irq_chip);
	irq_set_chip(IRQ_GPB, &nuc980_irq_chip);
	irq_set_chip(IRQ_GPC, &nuc980_irq_chip);
	irq_set_chip(IRQ_GPD, &nuc980_irq_chip);
	irq_set_chip(IRQ_GPE, &nuc980_irq_chip);
	irq_set_chip(IRQ_GPF, &nuc980_irq_chip);
	irq_set_chip(IRQ_GPG, &nuc980_irq_chip);
	irq_set_chained_handler(IRQ_GPA, nuc980_irq_demux_intgroupA);
	irq_set_chained_handler(IRQ_GPB, nuc980_irq_demux_intgroupB);
	irq_set_chained_handler(IRQ_GPC, nuc980_irq_demux_intgroupC);
	irq_set_chained_handler(IRQ_GPD, nuc980_irq_demux_intgroupD);
	irq_set_chained_handler(IRQ_GPE, nuc980_irq_demux_intgroupE);
	irq_set_chained_handler(IRQ_GPF, nuc980_irq_demux_intgroupF);
	irq_set_chained_handler(IRQ_GPG, nuc980_irq_demux_intgroupG);


	for (irqno = IRQ_GPIO_START; irqno < IRQ_GPIO_END; irqno++) {
		irq_set_chip_and_handler(irqno, &nuc980_irq_gpio,
		                         handle_level_irq);
		irq_clear_status_flags(irqno, IRQ_NOREQUEST);
	}

	/*
	 * Install handler for GPIO external interrupts
	 */
	for (irqno = IRQ_EXT0; irqno <= IRQ_EXT3; irqno++) {
		//printk("registering irq %d (extended nuc980 irq)\n", irqno);
		irq_set_chip(irqno, &nuc980_irq_chip);
		irq_set_chained_handler(irqno, nuc980_irq_demux_intgroup2);
	}

	for (irqno = IRQ_EXT0_A0; irqno <= IRQ_EXT3_G15; irqno++) {
		irq_set_chip_and_handler(irqno, &nuc980_irq_ext,
		                         handle_level_irq);
		irq_clear_status_flags(irqno, IRQ_NOREQUEST);
	}
	LEAVE();
#endif
#endif
}

#ifdef CONFIG_USE_OF

static struct irq_domain *nuc980_aic_domain;

static int nuc980_aic_irq_map(struct irq_domain *h, unsigned int virq,
                              irq_hw_number_t hw)
{
	//printk("nuc980_aic_irq_map: %d %d\n", virq, (int)hw);

	if ((IRQ_WDT <= hw) && (hw < NR_IRQS - SPARE_IRQS)) {
		irq_set_chip_and_handler(virq, &nuc980_irq_chip,
		                         handle_level_irq);
		irq_clear_status_flags(virq, IRQ_NOREQUEST);

#if defined(CONFIG_GPIO_NUC980)

		{
			int irqno;
			irq_set_chip(IRQ_GPA, &nuc980_irq_chip);
			irq_set_chip(IRQ_GPB, &nuc980_irq_chip);
			irq_set_chip(IRQ_GPC, &nuc980_irq_chip);
			irq_set_chip(IRQ_GPD, &nuc980_irq_chip);
			irq_set_chip(IRQ_GPE, &nuc980_irq_chip);
			irq_set_chip(IRQ_GPF, &nuc980_irq_chip);
			irq_set_chip(IRQ_GPG, &nuc980_irq_chip);
			irq_set_chained_handler(IRQ_GPA, nuc980_irq_demux_intgroupA);
			irq_set_chained_handler(IRQ_GPB, nuc980_irq_demux_intgroupB);
			irq_set_chained_handler(IRQ_GPC, nuc980_irq_demux_intgroupC);
			irq_set_chained_handler(IRQ_GPD, nuc980_irq_demux_intgroupD);
			irq_set_chained_handler(IRQ_GPE, nuc980_irq_demux_intgroupE);
			irq_set_chained_handler(IRQ_GPF, nuc980_irq_demux_intgroupF);
			irq_set_chained_handler(IRQ_GPG, nuc980_irq_demux_intgroupG);
			for (irqno = IRQ_GPIO_START; irqno < IRQ_GPIO_END; irqno++) {
				irq_set_chip_and_handler(irqno, &nuc980_irq_gpio,
				                         handle_level_irq);
				irq_clear_status_flags(irqno, IRQ_NOREQUEST);
			}

			/*
			 * Install handler for GPIO external interrupts
			 */
			for (irqno = IRQ_EXT0; irqno <= IRQ_EXT3; irqno++) {
				//printk("registering irq %d (extended nuc980 irq)\n", irqno);
				irq_set_chip(irqno, &nuc980_irq_chip);
				irq_set_chained_handler(irqno, nuc980_irq_demux_intgroup2);
			}

			for (irqno = IRQ_EXT0_A0; irqno <= IRQ_EXT3_G15; irqno++) {
				irq_set_chip_and_handler(irqno, &nuc980_irq_ext,
				                         handle_level_irq);
				irq_clear_status_flags(irqno, IRQ_NOREQUEST);
			}
		}
#endif

	}

	else
		return -EINVAL;

	return 0;
}

static int nuc980_aic_irq_domain_xlate(struct irq_domain *d,
                                       struct device_node *ctrlr,
                                       const u32 * intspec,
                                       unsigned int intsize,
                                       irq_hw_number_t * out_hwirq,
                                       unsigned int *out_type)
{
	if (WARN_ON(intsize < 2))
		return -EINVAL;
	if (WARN_ON(intspec[0] >= NR_IRQS))
		return -EINVAL;

	*out_hwirq = intspec[0];
	*out_type = IRQ_TYPE_NONE;

	return 0;
}

static struct irq_domain_ops nuc980_aic_irq_ops = {
	.map = nuc980_aic_irq_map,
	.xlate = nuc980_aic_irq_domain_xlate,
};

int __init nuc980_of_init_irq(struct device_node *node,
                              struct device_node *parent)
{
	nuc980_aic_domain = irq_domain_add_linear(node, SPARE_IRQS,
	                    &nuc980_aic_irq_ops, NULL);
	if (!nuc980_aic_domain)
		panic("Failed to add irq domain!!\n");

	irq_set_default_host(nuc980_aic_domain);

	__raw_writel(0xFFFFFFFC, REG_AIC_INTDIS0);
	__raw_writel(0xFFFFFFFF, REG_AIC_INTDIS1);

	irq_set_chip_and_handler(IRQ_TIMER4, &nuc980_irq_chip, handle_level_irq);
	irq_clear_status_flags(IRQ_TIMER4, IRQ_NOREQUEST);

	return 0;
}

#endif
