/* linux/driver/char/nuc980-etimer.c
 *
 * Copyright (c) 2018 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
#include <mach/regs-timer.h>
#include <mach/nuc980-timer.h>

#define ETIMER_CH	4
#define ETIMER_OPMODE_NONE		0
//#define ETIMER_OPMODE_ONESHOT		1
#define ETIMER_OPMODE_PERIODIC	2
//#define ETIMER_OPMODE_CONTINUOUS	3
#define ETIMER_OPMODE_TOGGLE		4
#define ETIMER_OPMODE_TRIGGER_COUNTING	5
#define ETIMER_OPMODE_FREE_COUNTING	6
#define ETIMER_OPMODE_EVENT_COUNTING 7

#define ETIMER_CTL_TWAKE_EN		0x00000004
#define ETIMER_CTL_ETMR_EN		0x00000001
#define ETIMER_CTL_ONESHOT		0x00000000
#define ETIMER_CTL_PERIODIC		0x00000010
#define ETIMER_CTL_TOGGLE		0x00000020
#define ETIMER_CTL_CONTINUOUS	0x00000030
#define ETIMER_CTL_EVENT_COUNTING	0x00001000
#define ETIMER_CTL_TCAP_EN		0x00010000
#define ETIMER_CTL_FREE_COUNTING    0x00000000
#define ETIMER_CTL_TRIGGER_COUNTING	0x00100000
#define ETIMER_IER_TCAP_IE		0x00000002


#define ETIMER_TRIGGER_COUNTING 	(ETIMER_CTL_TRIGGER_COUNTING |\
                                     ETIMER_CTL_TCAP_EN |\
                                     ETIMER_CTL_PERIODIC |\
                                     ETIMER_CTL_ETMR_EN)

#define ETIMER_FREE_COUNTING 		(ETIMER_CTL_FREE_COUNTING |\
                                     ETIMER_CTL_TCAP_EN |\
                                     ETIMER_CTL_PERIODIC |\
                                     ETIMER_CTL_ETMR_EN)

#define ETIMER_TOGGLE			(ETIMER_CTL_TOGGLE | ETIMER_CTL_ETMR_EN)

#define ETIMER_EVENT_COUNTER	(ETIMER_CTL_EVENT_COUNTING | ETIMER_CTL_ETMR_EN)

struct nuc980_etimer {
	spinlock_t lock;
	struct pinctrl *pinctrl;
	struct clk *clk;
	struct clk *eclk;
	wait_queue_head_t wq;
	int minor;	// dynamic minor num, so we need this to distinguish between channels
	u32 cap;	// latest capture data
	u32 cnt;	// latest timer up-counter value
	int irq;	// interrupt number
	u8 ch;		// timer channel. 0~3
	u8 mode;	// Current OP mode. Counter, free counting, trigger counting...
	u8 occupied;	// device opened
	u8 update;	// new capture data available
};


static struct nuc980_etimer *etmr[ETIMER_CH];
#ifdef CONFIG_NUC980_TIMER_WKUP
static uint8_t gu8_ch;
#endif
static uint32_t gu32_cnt;

static irqreturn_t nuc980_etimer_interrupt(int irq, void *dev_id)
{
	struct nuc980_etimer *t = (struct nuc980_etimer *)dev_id;
	static int cnt = 0;
	static uint32_t t0, t1;
	void __iomem *TMRBaseAddr = TIMER0;
	unsigned long flag = 0;

	int ch = t->ch;

	switch (ch) {
	case 0:
		TMRBaseAddr = TIMER0;
		break;
	case 1:
		TMRBaseAddr = TIMER1;
		break;
	case 2:
		TMRBaseAddr = TIMER2;
		break;
	case 3:
		TMRBaseAddr = TIMER3;
		break;
	}
	spin_lock(&t->lock);

	flag = __raw_readl(REG_TIMER_ISR(TMRBaseAddr));
	if( flag & 0x10 ) {
		__raw_writel((0x100 << ch), REG_WKUPSSR0); // clear system wake up source flag
		__raw_writel(0x10, REG_TIMER_ISR(TMRBaseAddr));  // clear Timer Wake-up Status
	}

	if(flag & 0x1) {
		t->cnt = gu32_cnt++;
		__raw_writel(__raw_readl(REG_TIMER_ISR(TMRBaseAddr)) & 0x1, REG_TIMER_ISR(TMRBaseAddr)); // clear Timer Interrupt Status
		t->update = 1;
	}

	if(flag & 0x2) {
		if(t->mode == ETIMER_OPMODE_FREE_COUNTING) {
			if(cnt == 0) {
				/* Gets the Timer capture data */
				t0 =  __raw_readl(REG_TIMER_TCAP(TMRBaseAddr));
				cnt++;

			} else if(cnt == 1) {
				/* Gets the Timer capture data */
				t1 =  __raw_readl(REG_TIMER_TCAP(TMRBaseAddr));
				cnt++;

				if(t0 > t1) {
					/* over run, drop this data and do nothing */

				} else {
					/* Display the measured input frequency */
					t->cap =  12000000 / (t1 - t0);
					t->update = 1;

				}
			} else {
				cnt = 0;
			}

		} else {
			t->cap = __raw_readl(REG_TIMER_TCAP(TMRBaseAddr));
			t->update = 1;
		}

		__raw_writel(__raw_readl(REG_TIMER_ISR(TMRBaseAddr)) & 0x2, REG_TIMER_ISR(TMRBaseAddr)); // clear Timer capture Interrupt Status

	}

	wake_up_interruptible(&t->wq);
	spin_unlock(&t->lock);

	return IRQ_HANDLED;
}

static void etimer_SwitchClkSrc(int flag, struct nuc980_etimer *t)
{
	struct clk *clkmux, *clklxt;
	struct clk ;
	int ch;
	void __iomem *TMRBaseAddr = TIMER0;

	clkmux = clk_get(NULL, "timer0_eclk_mux");

	ch = t->ch;
	switch (ch) {
	case 0:
		TMRBaseAddr = TIMER0;
		break;
	case 1:
		TMRBaseAddr = TIMER1;
		break;
	case 2:
		TMRBaseAddr = TIMER2;
		break;
	case 3:
		TMRBaseAddr = TIMER3;
		break;
	}

	if(flag==1) {
		clklxt = clk_get(NULL, "xin32k");
		// timer clock is 32kHz, set prescaler to 1 - 1.
		__raw_writel(0, REG_TIMER_PRECNT(TMRBaseAddr));
		pr_debug("ch = %d, xin32k \n", ch);
	} else {
		clklxt = clk_get(NULL, "xin");
		// timer clock is 12MHz, set prescaler to 12 - 1.
		__raw_writel(11, REG_TIMER_PRECNT(TMRBaseAddr));
		pr_debug("ch = %d, xin12M \n", ch);
	}

	if (IS_ERR(clklxt)) {
		pr_debug("failed to get 32k clk\n");
		return;
	}
	if(ch == 0) {
		clkmux = clk_get(NULL, "timer0_eclk_mux");
	} else if (ch == 1) {
		clkmux = clk_get(NULL, "timer1_eclk_mux");
	} else if (ch == 2) {
		clkmux = clk_get(NULL, "timer2_eclk_mux");
	} else if (ch == 3) {
		clkmux = clk_get(NULL, "timer3_eclk_mux");
	}
	if (IS_ERR(clkmux)) {
		pr_debug("failed to get etimer clock mux\n");
		return;
	}
	clk_set_parent(clkmux, clklxt);

	if(ch == 0) {
		etmr[ch]->clk = clk_get(NULL, "timer0");
		etmr[ch]->eclk = clk_get(NULL, "timer0_eclk");
	} else if(ch == 1) {
		etmr[ch]->clk = clk_get(NULL, "timer1");
		etmr[ch]->eclk = clk_get(NULL, "timer1_eclk");
	} else if(ch == 2) {
		etmr[ch]->clk = clk_get(NULL, "timer2");
		etmr[ch]->eclk = clk_get(NULL, "timer2_eclk");
	} else if(ch == 3) {
		etmr[ch]->clk = clk_get(NULL, "timer3");
		etmr[ch]->eclk = clk_get(NULL, "timer3_eclk");
	}


	if (IS_ERR(etmr[ch]->clk)) {
		printk("failed to get etmr clock\n");
		return;
	}


	if (IS_ERR(etmr[ch]->eclk)) {
		printk("failed to get etmr eclock\n");
		return;
	}

	clk_prepare(etmr[ch]->clk);
	clk_enable(etmr[ch]->clk);
	clk_prepare(etmr[ch]->eclk);
	clk_enable(etmr[ch]->eclk);
}


static void stop_timer(struct nuc980_etimer *t)
{
	unsigned long flag;
	void __iomem *TMRBaseAddr = TIMER0;

	switch (t->ch) {
	case 0:
		TMRBaseAddr = TIMER0;
		break;
	case 1:
		TMRBaseAddr = TIMER1;
		break;
	case 2:
		TMRBaseAddr = TIMER2;
		break;
	case 3:
		TMRBaseAddr = TIMER3;
		break;
	}

	spin_lock_irqsave(&t->lock, flag);
	// stop timer
	__raw_writel(0, REG_TIMER_CTL(TMRBaseAddr));
	// disable interrupt
	__raw_writel(0, REG_TIMER_IER(TMRBaseAddr));
	// clear interrupt flag if any
	__raw_writel(0xFFFFFFFF, REG_TIMER_ISR(TMRBaseAddr));
	t->mode = ETIMER_OPMODE_NONE;
	t->update = 0;
	spin_unlock_irqrestore(&t->lock, flag);
}

static ssize_t etimer_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned long flag;
	struct nuc980_etimer *t = (struct nuc980_etimer *)filp->private_data;
	int ret = 0;

	spin_lock_irqsave(&t->lock, flag);
	if(t->mode != ETIMER_OPMODE_TRIGGER_COUNTING &&
	   t->mode != ETIMER_OPMODE_FREE_COUNTING    &&
	   t->mode != ETIMER_OPMODE_PERIODIC         &&
	   t->mode != ETIMER_OPMODE_EVENT_COUNTING) {
		ret = -EPERM;

		goto out;
	}

	if(t->update) {

		if(t->mode == ETIMER_OPMODE_TRIGGER_COUNTING ||
		   t->mode == ETIMER_OPMODE_FREE_COUNTING) {

			if(copy_to_user(buf, &t->cap, sizeof(unsigned int)))
				ret = -EFAULT;
			else
				ret = 4;	// size of int.
		} else if(t->mode == ETIMER_OPMODE_PERIODIC ||
		          t->mode == ETIMER_OPMODE_EVENT_COUNTING) {
			if(copy_to_user(buf, &t->cnt, sizeof(unsigned int)))
				ret = -EFAULT;
			else
				ret = 4;	// size of int.
		}
		t->update = 0;

		goto out;
	} else {
		spin_unlock_irqrestore(&t->lock, flag);
		wait_event_interruptible(t->wq, t->update != 0);
		if(t->mode == ETIMER_OPMODE_TRIGGER_COUNTING ||
		   t->mode == ETIMER_OPMODE_FREE_COUNTING) {

			if(copy_to_user(buf, &t->cap, sizeof(unsigned int)))
				ret = -EFAULT;
			else
				ret = 4;	// size of int.
		} else if(t->mode == ETIMER_OPMODE_PERIODIC ||
		          t->mode == ETIMER_OPMODE_EVENT_COUNTING) {
			if(copy_to_user(buf, &t->cnt, sizeof(unsigned int)))
				ret = -EFAULT;
			else
				ret = 4;	// size of int.
		}
		t->update = 0;

		return ret;
	}

out:
	spin_unlock_irqrestore(&t->lock, flag);

	return ret;
}


static int etimer_release(struct inode *inode, struct file *filp)
{
	struct nuc980_etimer *t = (struct nuc980_etimer *)filp->private_data;
	int ch = t->ch;
	unsigned long flag;

	stop_timer(t);

	// free irq
	free_irq(etmr[ch]->irq, etmr[ch]);
	// disable clk
	clk_disable(etmr[ch]->clk);
	clk_disable(etmr[ch]->eclk);
	clk_put(etmr[ch]->clk);
	clk_put(etmr[ch]->eclk);

	spin_lock_irqsave(&etmr[ch]->lock, flag);
	etmr[ch]->occupied = 0;
	spin_unlock_irqrestore(&etmr[ch]->lock, flag);
	filp->private_data = NULL;

	return(0);
}

static int etimer_open(struct inode *inode, struct file *filp)
{
	int i, ret, ch = 0;
	unsigned long flag;
	struct clk *clkmux, *clkhxt;

	for(i = 0; i < ETIMER_CH; i++)
		if(MINOR(inode->i_rdev) == etmr[i]->minor) {
			ch = i;
			break;
		}

	spin_lock_irqsave(&etmr[ch]->lock, flag);
	if(etmr[ch]->occupied) {
		spin_unlock_irqrestore(&etmr[ch]->lock, flag);
		pr_debug("-EBUSY error\n");
		return -EBUSY;
	}

	etmr[ch]->occupied = 1;
	spin_unlock_irqrestore(&etmr[ch]->lock, flag);

	if (request_irq(etmr[ch]->irq, nuc980_etimer_interrupt,
	                IRQF_NO_SUSPEND, "nuc980-timer", etmr[ch])) {
		pr_debug("register irq failed %d\n", etmr[ch]->irq);
		ret = -EAGAIN;
		goto out2;
	}

	filp->private_data = etmr[ch];

	// configure engine clock
	clkhxt = clk_get(NULL, "xin");
	if (IS_ERR(clkhxt)) {
		pr_debug("failed to get xin clk\n");
		ret = PTR_ERR(clkhxt);
		goto out1;
	}
	if(ch == 0) {
		clkmux = clk_get(NULL, "timer0_eclk_mux");
	} else if (ch == 1) {
		clkmux = clk_get(NULL, "timer1_eclk_mux");
	} else if (ch == 2) {
		clkmux = clk_get(NULL, "timer2_eclk_mux");
	} else if (ch == 3) {
		clkmux = clk_get(NULL, "timer3_eclk_mux");
	}
	if (IS_ERR(clkmux)) {
		pr_debug("failed to get etimer clock mux\n");
		ret = PTR_ERR(clkmux);
		goto out1;
	}
	clk_set_parent(clkmux, clkhxt);

	if(ch == 0) {
		etmr[ch]->clk = clk_get(NULL, "timer0");
		etmr[ch]->eclk = clk_get(NULL, "timer0_eclk");
	} else if(ch == 1) {
		etmr[ch]->clk = clk_get(NULL, "timer1");
		etmr[ch]->eclk = clk_get(NULL, "timer1_eclk");
	} else if(ch == 2) {
		etmr[ch]->clk = clk_get(NULL, "timer2");
		etmr[ch]->eclk = clk_get(NULL, "timer2_eclk");
	} else if(ch == 3) {
		etmr[ch]->clk = clk_get(NULL, "timer3");
		etmr[ch]->eclk = clk_get(NULL, "timer3_eclk");
	}

	if (IS_ERR(etmr[ch]->clk)) {
		pr_debug("failed to get etmr clock\n");
		ret = PTR_ERR(etmr[ch]->clk);
		goto out1;
	}


	if (IS_ERR(etmr[ch]->eclk)) {
		pr_debug("failed to get etmr eclock\n");
		ret = PTR_ERR(etmr[ch]->eclk);
		goto out1;
	}

	clk_prepare(etmr[ch]->clk);
	clk_enable(etmr[ch]->clk);
	clk_prepare(etmr[ch]->eclk);
	clk_enable(etmr[ch]->eclk);

	return 0;


out1:

	free_irq(etmr[ch]->irq, etmr[ch]);
out2:
	spin_lock_irqsave(&etmr[ch]->lock, flag);
	etmr[ch]->occupied = 0;
	spin_unlock_irqrestore(&etmr[ch]->lock, flag);

	return ret;

}

static long etimer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flag;
	struct nuc980_etimer *t = (struct nuc980_etimer *)filp->private_data;
	int ch = t->ch;
	int unsigned param;
	u32 clksrc;
	void __iomem *TMRBaseAddr = TIMER0;
#ifndef CONFIG_USE_OF
	struct pinctrl_state *s;
	int ret;
#endif

	// stop timer before we do any change
	stop_timer(t);

	// init time-out counts
	gu32_cnt = 1;
	t->cnt = gu32_cnt;

	// check clock source
	clksrc = __raw_readl(REG_CLK_DIV8);

	switch (ch) {
	case 0:
		TMRBaseAddr = TIMER0;
		break;
	case 1:
		TMRBaseAddr = TIMER1;
		break;
	case 2:
		TMRBaseAddr = TIMER2;
		break;
	case 3:
		TMRBaseAddr = TIMER3;
		break;
	}

	switch(cmd) {
	case TMR_IOC_CLKLXT:
		etimer_SwitchClkSrc(1, t);
		break;

	case TMR_IOC_CLKHXT:
		etimer_SwitchClkSrc(0, t);
		break;

	case TMR_IOC_STOP:
		//timer stopped
		break;

	case TMR_IOC_PERIODIC:

		if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

		// compare register is 24-bit width
		if(param > 0xFFFFFF)
			return -EPERM;

		spin_lock_irqsave(&t->lock, flag);

		// timer clock is 12MHz, set prescaler to 12 - 1.
		__raw_writel(11, REG_TIMER_PRECNT(TMRBaseAddr));
		__raw_writel(param, REG_TIMER_CMPR(TMRBaseAddr));

		// enable timeout interrupt
		__raw_writel(0x1, REG_TIMER_IER(TMRBaseAddr));
		__raw_writel(ETIMER_CTL_PERIODIC | ETIMER_CTL_ETMR_EN, REG_TIMER_CTL(TMRBaseAddr));

		t->mode = ETIMER_OPMODE_PERIODIC;
		spin_unlock_irqrestore(&t->lock, flag);

		break;

#ifdef CONFIG_NUC980_TIMER_WKUP
	case TMR_IOC_PERIODIC_FOR_WKUP:

		if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

		// compare register is 24-bit width
		if(param > 0xFFFFFF)
			return -EPERM;

		// check clock, power down mode using 32kHz
		clksrc = __raw_readl(REG_CLK_DIV8);
		if ( ((clksrc >> ((16 + (ch * 2)))) & 0x3) != 0x3 ) {
			printk("Power down mode clock need to switch to 32k.\n");
			return -1;
		}
		// gu8_wkflag = 0;
		gu8_ch = ch;
		spin_lock_irqsave(&t->lock, flag);
		__raw_writel(param, REG_TIMER_CMPR(TMRBaseAddr));

		// enable timeout interrupt
		__raw_writel(0x1, REG_TIMER_IER(TMRBaseAddr));
		__raw_writel(ETIMER_CTL_PERIODIC  | ETIMER_CTL_ETMR_EN, REG_TIMER_CTL(TMRBaseAddr));


		t->mode = ETIMER_OPMODE_PERIODIC;
		spin_unlock_irqrestore(&t->lock, flag);

		break;
#endif

	case TMR_IOC_TOGGLE:
		// get output duty in us
		if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;
		// divide by 2 because a duty cycle is high + low
		param >>= 1;
		// compare register is 24-bit width
		if(param > 0xFFFFFF)
			return -EPERM;

#ifndef CONFIG_USE_OF
		// set pin function
		if(t->ch == 0) {
#if defined (CONFIG_NUC980_TIMER0_TGL_PB3)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-tgl-PB3");
#elif defined (CONFIG_NUC980_TIMER0_TGL_PC0)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-tgl-PC0");
#elif defined (CONFIG_NUC980_TIMER0_TGL_PB9)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-tgl-PB9");
#elif defined (CONFIG_NUC980_TIMER0_TGL_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 1) {
#if defined (CONFIG_NUC980_TIMER1_TGL_PA14)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-tgl-PA14");
#elif defined (CONFIG_NUC980_TIMER1_TGL_PD0)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-tgl-PD0");
#elif defined (CONFIG_NUC980_TIMER1_TGL_PG11)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-tgl-PG11");
#elif defined (CONFIG_NUC980_TIMER1_TGL_PF8)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-tgl-PF8");
#elif defined (CONFIG_NUC980_TIMER1_TGL_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 2) {
#if defined (CONFIG_NUC980_TIMER2_TGL_PA10)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-tgl-PA10");
#elif defined (CONFIG_NUC980_TIMER2_TGL_PB12)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-tgl-PB12");
#elif defined (CONFIG_NUC980_TIMER2_TGL_PD12)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-tgl-PD12");
#elif defined (CONFIG_NUC980_TIMER2_TGL_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 3) {
#if defined (CONFIG_NUC980_TIMER3_TGL_PA8)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-tgl-PA8");
#elif defined (CONFIG_NUC980_TIMER3_TGL_PD14)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-tgl-PD14");
#elif defined (CONFIG_NUC980_TIMER3_TGL_NONE)
			return -EPERM;
#endif
		}

		if (IS_ERR(s)) {
			pr_debug("pinctrl_lookup_state err\n");
			return -EPERM;
		}

		if((ret = pinctrl_select_state(t->pinctrl, s)) < 0) {
			pr_debug("pinctrl_select_state err\n");
			return ret;
		}
#endif

		spin_lock_irqsave(&t->lock, flag);
		// timer clock is 12MHz, set prescaler to 12 - 1.
		__raw_writel(11, REG_TIMER_PRECNT(TMRBaseAddr));
		__raw_writel(param, REG_TIMER_CMPR(TMRBaseAddr));
		__raw_writel(ETIMER_TOGGLE, REG_TIMER_CTL(TMRBaseAddr));
		t->mode = ETIMER_OPMODE_TOGGLE;
		pr_debug("Toggle REG_TIMER_CTL(%d)=0x%08x\n",ch, __raw_readl(REG_TIMER_CTL(TMRBaseAddr)));
		spin_unlock_irqrestore(&t->lock, flag);

		break;

	case TMR_IOC_EVENT_COUNTING:

		// get capture setting
		if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

#ifndef CONFIG_USE_OF
		// set pin function
		if(t->ch == 0) {
#if defined (CONFIG_NUC980_TIMER0_ECNT_PA0)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-ecnt-PA0");
#elif defined (CONFIG_NUC980_TIMER0_ECNT_PD6)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-ecnt-PD6");
#elif defined (CONFIG_NUC980_TIMER0_ECNT_PF0)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-ecnt-PF0");
#elif defined (CONFIG_NUC980_TIMER0_ECNT_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 1) {
#if defined (CONFIG_NUC980_TIMER1_ECNT_PA1)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-ecnt-PA1");
#elif defined (CONFIG_NUC980_TIMER1_ECNT_PD7)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-ecnt-PD7");
#elif defined (CONFIG_NUC980_TIMER1_ECNT_PF1)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-ecnt-PF1");
#elif defined (CONFIG_NUC980_TIMER1_ECNT_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 2) {
#if defined (CONFIG_NUC980_TIMER2_ECNT_PA2)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-ecnt-PA2");
#elif defined (CONFIG_NUC980_TIMER2_ECNT_PD8)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-ecnt-PD8");
#elif defined (CONFIG_NUC980_TIMER2_ECNT_PF2)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-ecnt-PF2");
#elif defined (CONFIG_NUC980_TIMER2_ECNT_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 3) {
#if defined (CONFIG_NUC980_TIMER3_ECNT_PA3)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-ecnt-PA3");
#elif defined (CONFIG_NUC980_TIMER3_ECNT_PD9)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-ecnt-PD9");
#elif defined (CONFIG_NUC980_TIMER3_ECNT_PF3)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-ecnt-PF3");
#elif defined (CONFIG_NUC980_TIMER3_ECNT_NONE)
			return -EPERM;
#endif
		}

		if (IS_ERR(s)) {
			pr_debug("pinctrl_lookup_state err\n");
			return -EPERM;
		}
		if((ret = pinctrl_select_state(t->pinctrl, s)) < 0) {
			pr_debug("pinctrl_select_state err\n");
			return ret;
		}
#endif

		spin_lock_irqsave(&t->lock, flag);
		// timer clock is 12MHz, set prescaler to 12 - 1.
		__raw_writel(0, REG_TIMER_PRECNT(TMRBaseAddr));
		__raw_writel(param, REG_TIMER_CMPR(TMRBaseAddr));
		// enable timeout interrupt
		__raw_writel(0x1, REG_TIMER_IER(TMRBaseAddr));
		__raw_writel(ETIMER_EVENT_COUNTER | ETIMER_CTL_PERIODIC | TMR_EXTCNT_EDGE_FF, REG_TIMER_CTL(TMRBaseAddr));
		t->mode = ETIMER_OPMODE_EVENT_COUNTING;
		spin_unlock_irqrestore(&t->lock, flag);

		break;

	case TMR_IOC_FREE_COUNTING:

		// get capture setting
		if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

#ifndef CONFIG_USE_OF
		// set pin function
		if(t->ch == 0) {
#if defined (CONFIG_NUC980_TIMER0_CAP_PB1)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PB1");
#elif defined (CONFIG_NUC980_TIMER0_CAP_PB8)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PB8");
#elif defined (CONFIG_NUC980_TIMER0_CAP_PB10)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PB10");
#elif defined (CONFIG_NUC980_TIMER0_CAP_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 1) {
#if defined (CONFIG_NUC980_TIMER1_CAP_PA13)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PA13");
#elif defined (CONFIG_NUC980_TIMER1_CAP_PD1)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PD1");
#elif defined (CONFIG_NUC980_TIMER1_CAP_PG12)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PG12");
#elif defined (CONFIG_NUC980_TIMER1_CAP_PF9)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PF9");
#elif defined (CONFIG_NUC980_TIMER1_CAP_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 2) {
#if defined (CONFIG_NUC980_TIMER2_CAP_PA9)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PA9");
#elif defined (CONFIG_NUC980_TIMER2_CAP_PB11)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PB11");
#elif defined (CONFIG_NUC980_TIMER2_CAP_PD13)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PD13");
#elif defined (CONFIG_NUC980_TIMER2_CAP_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 3) {
#if defined (CONFIG_NUC980_TIMER3_CAP_PA7)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-cap-PA7");
#elif defined (CONFIG_NUC980_TIMER3_CAP_PD15)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-cap-PD15");
#elif defined (CONFIG_NUC980_TIMER3_CAP_NONE)
			return -EPERM;
#endif
		}

		if (IS_ERR(s)) {
			pr_debug("pinctrl_lookup_state err\n");
			return -EPERM;
		}
		if((ret = pinctrl_select_state(t->pinctrl, s)) < 0) {
			pr_debug("pinctrl_select_state err\n");
			return ret;
		}
#endif

		spin_lock_irqsave(&t->lock, flag);
		// timer clock is 12MHz.
		__raw_writel(0, REG_TIMER_PRECNT(TMRBaseAddr));
		__raw_writel(0xFFFFFF, REG_TIMER_CMPR(TMRBaseAddr));
		// enable capture interrupt
		__raw_writel(ETIMER_IER_TCAP_IE, REG_TIMER_IER(TMRBaseAddr));
		__raw_writel(param | ETIMER_FREE_COUNTING, REG_TIMER_CTL(TMRBaseAddr));

		t->mode = ETIMER_OPMODE_FREE_COUNTING;
		spin_unlock_irqrestore(&t->lock, flag);

		break;

	case TMR_IOC_TRIGGER_COUNTING:
		// get capture setting
		if(copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;


#ifndef CONFIG_USE_OF
		// set pin function
		if(t->ch == 0) {
#if defined (CONFIG_NUC980_TIMER0_CAP_PB1)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PB1");
#elif defined (CONFIG_NUC980_TIMER0_CAP_PB8)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PB8");
#elif defined (CONFIG_NUC980_TIMER0_CAP_PB10)
			s = pinctrl_lookup_state(t->pinctrl, "etimer0-cap-PB10");
#elif defined (CONFIG_NUC980_TIMER0_CAP_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 1) {
#if defined (CONFIG_NUC980_TIMER1_CAP_PA13)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PA13");
#elif defined (CONFIG_NUC980_TIMER1_CAP_PD1)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PD1");
#elif defined (CONFIG_NUC980_TIMER1_CAP_PG12)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PG12");
#elif defined (CONFIG_NUC980_TIMER1_CAP_PF9)
			s = pinctrl_lookup_state(t->pinctrl, "etimer1-cap-PF9");
#elif defined (CONFIG_NUC980_TIMER1_CAP_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 2) {
#if defined (CONFIG_NUC980_TIMER2_CAP_PA9)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PA9");
#elif defined (CONFIG_NUC980_TIMER2_CAP_PB11)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PB11");
#elif defined (CONFIG_NUC980_TIMER2_CAP_PD13)
			s = pinctrl_lookup_state(t->pinctrl, "etimer2-cap-PD13");
#elif defined (CONFIG_NUC980_TIMER2_CAP_NONE)
			return -EPERM;
#endif
		} else if(t->ch == 3) {
#if defined (CONFIG_NUC980_TIMER3_CAP_PA7)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-cap-PA7");
#elif defined (CONFIG_NUC980_TIMER3_CAP_PD15)
			s = pinctrl_lookup_state(t->pinctrl, "etimer3-cap-PD15");
#elif defined (CONFIG_NUC980_TIMER3_CAP_NONE)
			return -EPERM;
#endif
		}

		if (IS_ERR(s)) {
			pr_debug("pinctrl_lookup_state err\n");
			return -EPERM;
		}
		if((ret = pinctrl_select_state(t->pinctrl, s)) < 0) {
			pr_debug("pinctrl_select_state err\n");
			return ret;
		}
#endif

		spin_lock_irqsave(&t->lock, flag);
		// timer clock is 12MHz.
		__raw_writel(11, REG_TIMER_PRECNT(TMRBaseAddr));
		__raw_writel(0xFFFFFF, REG_TIMER_CMPR(TMRBaseAddr));
		// enable capture interrupt
		__raw_writel(ETIMER_IER_TCAP_IE, REG_TIMER_IER(TMRBaseAddr));
		__raw_writel((u32)(param | ETIMER_TRIGGER_COUNTING), REG_TIMER_CTL(TMRBaseAddr));
		t->mode = ETIMER_OPMODE_TRIGGER_COUNTING;
		spin_unlock_irqrestore(&t->lock, flag);
		break;

	default:
		return -ENOTTY;


	}
	return 0;
}

static unsigned int etimer_poll(struct file *filp, poll_table *wait)
{
	struct nuc980_etimer *t = (struct nuc980_etimer *)filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &t->wq, wait);
	if(t->update)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

struct file_operations etimer_fops = {
	.owner		= THIS_MODULE,
	.open		= etimer_open,
	.release	= etimer_release,
	.read		= etimer_read,
	.unlocked_ioctl	= etimer_ioctl,
	.poll		= etimer_poll,
};

static struct miscdevice etimer_dev[] = {
	[0] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer0",
		.fops = &etimer_fops,
	},
	[1] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer1",
		.fops = &etimer_fops,
	},
	[2] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer2",
		.fops = &etimer_fops,
	},
	[3] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer3",
		.fops = &etimer_fops,
	},

};

static int nuc980_etimer_probe(struct platform_device *pdev)
{
	int ch = pdev->id;

	//printk("nuc980_etimer_probe    %s - pdev = %s  \n", __func__, pdev->name);

#ifdef CONFIG_USE_OF
	struct pinctrl *pinctrl;
	u32   val32[2];

	//printk("nuc980_etimer_probe    %s - pdev = %s  \n", __func__, pdev->name);

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		return PTR_ERR(pinctrl);
	}

	if (of_property_read_u32_array(pdev->dev.of_node, "port-number", val32, 1) != 0) {
		printk("%s can not get port-number!\n", __func__);
		return -EINVAL;
	}

	ch = val32[0];

#endif

	//printk("etimer %d  \n\n", ch);

	etmr[ch] = devm_kzalloc(&pdev->dev, sizeof(struct nuc980_etimer), GFP_KERNEL);
	if (etmr[ch] == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for etimer device\n");
		return -ENOMEM;
	}


	misc_register(&etimer_dev[ch]);

	etmr[ch]->pinctrl = devm_pinctrl_get(&pdev->dev);
	etmr[ch]->minor = MINOR(etimer_dev[ch].minor);
	etmr[ch]->ch = ch;
	spin_lock_init(&etmr[ch]->lock);

#ifdef CONFIG_USE_OF
	etmr[ch]->irq = platform_get_irq(pdev, 0);
#else
	etmr[ch]->irq = platform_get_irq(pdev, ch);
#endif

	//printk("etimer%d(pdev->id=%d), platform_get_irq - %d\n", ch, pdev->id, etmr[ch]->irq);

	init_waitqueue_head(&etmr[ch]->wq);


	platform_set_drvdata(pdev, etmr[ch]);


	return(0);
}

static int nuc980_etimer_remove(struct platform_device *pdev)
{
	struct nuc980_etimer *t = platform_get_drvdata(pdev);
	int ch = t->ch;

	misc_deregister(&etimer_dev[ch]);


	return 0;
}


#ifdef CONFIG_NUC980_TIMER_WKUP
static int nuc980_etimer_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct nuc980_etimer *t = platform_get_drvdata(pdev);

	void __iomem *TMRBaseAddr = TIMER0;

	switch (t->ch) {
	case 0:
		TMRBaseAddr = TIMER0;
		break;
	case 1:
		TMRBaseAddr = TIMER1;
		break;
	case 2:
		TMRBaseAddr = TIMER2;
		break;
	case 3:
		TMRBaseAddr = TIMER3;
		break;
	}

	//pr_debug("******* nuc980_etimer_suspend pdev->id = %d, gu8_ch= %d, t->ch =%d \n", pdev->id, gu8_ch,  t->ch);
	if(t->ch == gu8_ch) {
		__raw_writel(__raw_readl(REG_WKUPSER0)| (0x100<<(gu8_ch)),REG_WKUPSER0);

		__raw_writel(ETIMER_CTL_PERIODIC | ETIMER_CTL_ETMR_EN | ETIMER_CTL_TWAKE_EN, REG_TIMER_CTL(TMRBaseAddr));


		enable_irq_wake(t->irq);
	}

	return 0;
}

static int nuc980_etimer_resume(struct platform_device *pdev)
{
	struct nuc980_etimer *t = platform_get_drvdata(pdev);
	void __iomem *TMRBaseAddr = TIMER0;

	switch (t->ch) {
	case 0:
		TMRBaseAddr = TIMER0;
		break;
	case 1:
		TMRBaseAddr = TIMER1;
		break;
	case 2:
		TMRBaseAddr = TIMER2;
		break;
	case 3:
		TMRBaseAddr = TIMER3;
		break;
	}

	//pr_debug("======== nuc980_etimer_resume pdev->id = %d, gu8_ch= %d, t->ch =%d \n", pdev->id, gu8_ch,  t->ch);
	if(t->ch == gu8_ch) {
		__raw_writel(__raw_readl(REG_WKUPSER0)& ~(0x100<<(gu8_ch)),REG_WKUPSER0);

		__raw_writel(ETIMER_CTL_PERIODIC | ETIMER_CTL_ETMR_EN, REG_TIMER_CTL(TMRBaseAddr));

		disable_irq_wake(t->irq);
	}

	return 0;
}

#else
#define nuc980_etimer_suspend 	NULL
#define nuc980_etimer_resume	NULL
#endif

static const struct of_device_id nuc980_etimer_of_match[] = {
	{ .compatible = "nuvoton,nuc980-timer" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_etimer_of_match);

static struct platform_driver nuc980_etimer_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "nuc980-timer",
		.of_match_table = of_match_ptr(nuc980_etimer_of_match),
	},
	.probe		= nuc980_etimer_probe,
	.remove		= nuc980_etimer_remove,
	.suspend	= nuc980_etimer_suspend,
	.resume		= nuc980_etimer_resume,
};


module_platform_driver(nuc980_etimer_driver);



MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_ALIAS("platform:nuc980-timer");
MODULE_LICENSE("GPL");
