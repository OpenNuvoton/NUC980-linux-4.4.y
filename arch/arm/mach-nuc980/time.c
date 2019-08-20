/*
 * linux/arch/arm/mach-nuc980/time.c
 *
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clkdev.h>
#include <linux/sched_clock.h>

#include <asm/mach-types.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <mach/mfp.h>
#include <mach/map.h>
#include <mach/regs-timer.h>
#include <mach/hardware.h>
#include <mach/regs-clock.h>
#include <mach/regs-aic.h>
#include <mach/regs-serial.h>
#include <mach/regs-gcr.h>

#define PERIOD		(0x1 << 4)
#define ONESHOT		(0x0)
#define COUNTEN		(0x1)

#define TICKS_PER_SEC	100
#define PRESCALE	0x63	/* Divider = prescale + 1 */

#define	TDR_SHIFT	24
#define	TDR_MASK	((1 << TDR_SHIFT) - 1)

static unsigned int timer4_load;

static inline void timer_shutdown(struct clock_event_device *evt)
{
	/* disable timer */
	__raw_writel(0x00, REG_TIMER_CTL(TIMER4));
}

static int nuc980_shutdown(struct clock_event_device *evt)
{
	timer_shutdown(evt);

	return 0;
}

int nuc980_set_periodic(struct clock_event_device *clk)
{
	unsigned int val;

	val = __raw_readl(REG_TIMER_CTL(TIMER4)) & ~(0x03 << 4);
	__raw_writel(timer4_load, REG_TIMER_CMPR(TIMER4));
	val |= (PERIOD | COUNTEN);
	__raw_writel(val, REG_TIMER_CTL(TIMER4));
	return 0;
}

int nuc980_set_oneshot(struct clock_event_device *clk)
{
	unsigned int val;

	val = __raw_readl(REG_TIMER_CTL(TIMER4)) & ~(0x03 << 4);
	val |= (ONESHOT | COUNTEN);
	__raw_writel(val, REG_TIMER_CTL(TIMER4));
	return 0;
}

static int nuc980_clockevent_setnextevent(unsigned long evt,
					  struct clock_event_device *clk)
{
	__raw_writel(0, REG_TIMER_CTL(TIMER4));
	__raw_writel(evt, REG_TIMER_CMPR(TIMER4));
	while(__raw_readl(REG_TIMER_DR(TIMER4)) != 0);
	__raw_writel(__raw_readl(REG_TIMER_CTL(TIMER4)) | COUNTEN,
		     REG_TIMER_CTL(TIMER4));

	return 0;
}

#ifdef CONFIG_PM
static int tmr4_msk;
static void nuc980_clockevent_suspend(struct clock_event_device *clk)
{
	unsigned long flags;

	local_irq_save(flags);
	if (__raw_readl(REG_AIC_INTMSK1) & (1 << 0)) {
		tmr4_msk = 1;
		__raw_writel(0x1, REG_AIC_INTDIS1);
	} else
		tmr4_msk = 0;

	local_irq_restore(flags);

}

static void nuc980_clockevent_resume(struct clock_event_device *clk)
{
	unsigned long flags;

	local_irq_save(flags);
	if (tmr4_msk == 1)
		__raw_writel(0x1, REG_AIC_INTEN1);
	local_irq_restore(flags);
}
#endif
static struct clock_event_device nuc980_clockevent_device = {
	.name = "nuc980-timer4",
	.shift = 24,
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
        .set_state_shutdown = nuc980_shutdown,
	.set_state_oneshot = nuc980_set_oneshot,
	.set_state_periodic = nuc980_set_periodic,
	.tick_resume = nuc980_shutdown,
	.set_next_event = nuc980_clockevent_setnextevent,
#ifdef CONFIG_PM
	.suspend = nuc980_clockevent_suspend,
	.resume = nuc980_clockevent_resume,
#endif
	.rating = 300,
};

/*IRQ handler for the timer*/
static irqreturn_t nuc980_timer4_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &nuc980_clockevent_device;

	__raw_writel(1, REG_TIMER_ISR(TIMER4));	/* clear interrupt */
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction nuc980_timer4_irq = {
	.name = "nuc980-timer4",
	.flags = IRQF_TIMER | IRQF_IRQPOLL,
	.handler = nuc980_timer4_interrupt,
};

static void __init nuc980_clockevents_init(void)
{
	unsigned int rate;
	struct clk *clk = clk_get(NULL, "timer4");
	struct clk *eclk = clk_get(NULL, "timer4_eclk");

	BUG_ON(IS_ERR(clk));
	BUG_ON(IS_ERR(eclk));

	clk_prepare(clk);
	clk_enable(clk);
	clk_prepare(eclk);
	clk_enable(eclk);

	__raw_writel(0, REG_TIMER_CTL(TIMER4));
	rate = clk_get_rate(eclk);

	timer4_load = (rate / TICKS_PER_SEC);

	__raw_writel(1, REG_TIMER_ISR(TIMER4));
	__raw_writel(0, REG_TIMER_PRECNT(TIMER4));
	__raw_writel(1, REG_TIMER_IER(TIMER4));
	setup_irq(IRQ_TIMER4, &nuc980_timer4_irq);

	nuc980_clockevent_device.cpumask = cpumask_of(0);

	clockevents_config_and_register(&nuc980_clockevent_device, rate, 12, 0xffffff);
}

static u64 read_sched_clock(void)
{

	return __raw_readl(REG_TIMER_DR(TIMER5));
}

static void __init nuc980_clocksource_init(void)
{
	unsigned int rate = 0;
	struct clk *clk = clk_get(NULL, "timer5");
	struct clk *eclk = clk_get(NULL, "timer5_eclk");

	BUG_ON(IS_ERR(clk));
	BUG_ON(IS_ERR(eclk));

	clk_prepare(clk);
	clk_enable(clk);
	clk_prepare(eclk);
	clk_enable(eclk);

	__raw_writel(0x00, REG_TIMER_CTL(TIMER5));

	rate = clk_get_rate(eclk) / (PRESCALE + 1);

	__raw_writel(0xffffffff, REG_TIMER_CMPR(TIMER5));
	__raw_writel(PRESCALE, REG_TIMER_PRECNT(TIMER5));

	__raw_writel(COUNTEN | PERIOD, REG_TIMER_CTL(TIMER5));

	clocksource_mmio_init(REG_TIMER_DR(TIMER5),
		"nuc980-timer5", rate, 200, 24, clocksource_mmio_readl_up);
	sched_clock_register(read_sched_clock, 24, rate);
}

void __init nuc980_setup_default_serial_console(void)
{
	struct clk *clk = clk_get(NULL, "uart0");

	BUG_ON(IS_ERR(clk));

	clk_prepare(clk);
	clk_enable(clk);

	/* GPF11, GPF12 */
	nuc980_mfp_set_port_f(11, 0x1);
	nuc980_mfp_set_port_f(12, 0x1);
}

extern int nuc980_init_clocks(void);
void __init nuc980_timer_init(void)
{
	nuc980_init_clocks();
	nuc980_setup_default_serial_console();
	nuc980_clocksource_init();
	nuc980_clockevents_init();
}
