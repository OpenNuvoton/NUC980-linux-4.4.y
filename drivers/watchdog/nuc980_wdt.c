/*
 * Copyright (c) 2018 Nuvoton Technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/of.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-wdt.h>

#define TOUTSEL			(0x07 << 8)     /* wdt interval selection */
#define WDTEN			(0x01 << 7)	/* wdt enable*/
#define INTEN			(0x01 << 6)
#define WKF			(0x01 << 5)
#define WKEN			(0x01 << 4)
#define IF			(0x01 << 3)
#define RSTF			(0x01 << 2)	/* wdt reset flag */
#define RSTEN			(0x01 << 1)	/* wdt reset enable */
/*
 * Assumming 32k crystal is configured as the watchdog clock source,
 * the time out interval can be calculated via following formula:
 * TOUTSEL		real time interval (formula)
 * 0x05		((2^ 14 + 1024) * (32k crystal freq))seconds = 0.53 sec
 * 0x06		((2^ 16 + 1024) * (32k crystal freq))seconds = 2.03 sec
 * 0x07		((2^ 18 + 1024) * (32k crystal freq))seconds = 8.03 sec
 */
#define WDT_HW_TIMEOUT		0x05

#define RESET_COUNTER		0x00005AA5

#ifdef CONFIG_NUC980_WDT_WKUP
static int heartbeat = 8;
#else
static int heartbeat = 11;
#endif
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeats in seconds. "
	"(default = " __MODULE_STRING(WDT_HEARTBEAT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
	"(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct nuc980_wdt {
	struct resource		*res;
	struct clk		*clk;
	struct clk		*eclk;
	struct platform_device	*pdev;
};

static struct nuc980_wdt *nuc980_wdt;

// UnLock register write protect
static void Unlock_RegWriteProtect(void)
{
    do {
        __raw_writel(0x59, REG_WRPRTR);
        __raw_writel(0x16, REG_WRPRTR);
        __raw_writel(0x88, REG_WRPRTR);
    //wait for write-protection disabled indicator raised
    } while(!(__raw_readl(REG_WRPRTR) & 1));
}

// Lock register write protect
static void Lock_RegWriteProtect(void)
{
    __raw_writel(0x0, REG_WRPRTR);
}

static int nuc980wdt_ping(struct watchdog_device *wdd)
{
	__raw_writel(RESET_COUNTER, REG_WDT_RSTCNT);
	return 0;
}


static int nuc980wdt_start(struct watchdog_device *wdd)
{
	unsigned int val = RSTEN | WDTEN;
	unsigned long flags;


#ifdef CONFIG_NUC980_WDT_WKUP
	val |= INTEN;
	val |= WKEN;
#endif

	if(wdd->timeout < 2) {
		val |= 0x5 << 8;
#ifdef CONFIG_NUC980_WDT_WKUP
	} else if (wdd->timeout < 8) {
#else
	} else if (wdd->timeout < 11) {
#endif
		val |= 0x6 << 8;
	} else {
		val |= 0x7 << 8;
	}

	local_irq_save(flags);
	Unlock_RegWriteProtect();
	__raw_writel(val, REG_WDT_CTL);
	Lock_RegWriteProtect();
	local_irq_restore(flags);
	__raw_writel(RESET_COUNTER, REG_WDT_RSTCNT);

	return 0;
}

static int nuc980wdt_stop(struct watchdog_device *wdd)
{
	unsigned long flags;

	pr_warn("Stopping WDT is probably not a good idea\n");

	local_irq_save(flags);
	Unlock_RegWriteProtect();
	__raw_writel(0, REG_WDT_CTL);
	Lock_RegWriteProtect();
	local_irq_restore(flags);
	return 0;
}


static int nuc980wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	unsigned int val;
	unsigned long flags;


	val = __raw_readl(REG_WDT_CTL);
	val &= ~TOUTSEL;
	if(timeout < 2) {
		val |= 0x5 << 8;
#ifdef CONFIG_NUC980_WDT_WKUP
	} else if (timeout < 8) {
#else
	} else if (timeout < 11) {
#endif
		val |= 0x6 << 8;
	} else {
		val |= 0x7 << 8;
	}

	local_irq_save(flags);
	Unlock_RegWriteProtect();
	__raw_writel(val, REG_WDT_CTL);
	Lock_RegWriteProtect();
	local_irq_restore(flags);

	return 0;
}

static const struct watchdog_info nuc980wdt_info = {
	.identity	= "nuc980 watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static struct watchdog_ops nuc980wdt_ops = {
	.owner = THIS_MODULE,
	.start = nuc980wdt_start,
	.stop = nuc980wdt_stop,
	.ping = nuc980wdt_ping,
	.set_timeout = nuc980wdt_set_timeout,
};

static struct watchdog_device nuc980_wdd = {
	.status = WATCHDOG_NOWAYOUT_INIT_STATUS,
	.info = &nuc980wdt_info,
	.ops = &nuc980wdt_ops,
};

#ifdef CONFIG_NUC980_WDT_WKUP
static irqreturn_t nuc980_wdt_interrupt(int irq, void *dev_id)
{
	__raw_writel(RESET_COUNTER, REG_WDT_RSTCNT);

	Unlock_RegWriteProtect();
	if (__raw_readl(REG_WDT_CTL) & IF) {
		__raw_writel(__raw_readl(REG_WDT_CTL) | IF, REG_WDT_CTL);
	}
	Lock_RegWriteProtect();

	return IRQ_HANDLED;
};
#endif

static int nuc980wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct clk *clkmux, *clksrc;

	nuc980_wdt = devm_kzalloc(&pdev->dev, sizeof(struct nuc980_wdt), GFP_KERNEL);
	if (!nuc980_wdt)
		return -ENOMEM;

	nuc980_wdt->pdev = pdev;

	nuc980_wdt->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (nuc980_wdt->res == NULL) {
		dev_err(&pdev->dev, "no memory resource specified\n");
		return -ENOENT;
	}

	if (!devm_request_mem_region(&pdev->dev, nuc980_wdt->res->start,
				resource_size(nuc980_wdt->res), pdev->name)) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		return -ENOENT;
	}

	clkmux = clk_get(NULL, "wdt_eclk_mux");
	if (IS_ERR(clkmux)) {
		dev_err(&pdev->dev, "failed to get watchdog clock mux\n");
		ret = PTR_ERR(clkmux);
		return ret;
	}
#ifdef CONFIG_NUC980_WDT_WKUP
	/* Need to select xin32k to support WDT wake up */
	clksrc = clk_get(NULL, "xin32k");
#else
	clksrc = clk_get(NULL, "xin512_div");
#endif
	if (IS_ERR(clksrc)) {
#ifdef CONFIG_NUC980_WDT_WKUP
		dev_err(&pdev->dev, "failed to get xin32k clk\n");
#else
		dev_err(&pdev->dev, "failed to get xin/512 clk\n");
#endif
		ret = PTR_ERR(clksrc);
		return ret;
	}


	clk_set_parent(clkmux, clksrc);

	nuc980_wdt->clk = clk_get(NULL, "wdt");
	if (IS_ERR(nuc980_wdt->clk)) {
		dev_err(&pdev->dev, "failed to get watchdog clock\n");
		ret = PTR_ERR(nuc980_wdt->clk);
		return ret;
	}

	clk_prepare(nuc980_wdt->clk);
	clk_enable(nuc980_wdt->clk);

	nuc980_wdt->eclk = clk_get(NULL, "wdt_eclk");
	if (IS_ERR(nuc980_wdt->eclk)) {
		dev_err(&pdev->dev, "failed to get watchdog eclock\n");
		ret = PTR_ERR(nuc980_wdt->eclk);
		return ret;
	}

	clk_prepare(nuc980_wdt->eclk);
	clk_enable(nuc980_wdt->eclk);
#ifdef CONFIG_NUC980_WDT_WKUP
	nuc980_wdd.timeout = 8;		// default time out = 2 sec
	nuc980_wdd.min_timeout = 1;	// min time out = 1 sec
	nuc980_wdd.max_timeout = 8;	// max time out = 8 sec
#else
	nuc980_wdd.timeout = 11;	// default time out = 11.2 sec
	nuc980_wdd.min_timeout = 1;	// min time out = 1.4 sec
	nuc980_wdd.max_timeout = 11;	// max time out = 11.2 sec
#endif
	watchdog_init_timeout(&nuc980_wdd, heartbeat, &pdev->dev);
	watchdog_set_nowayout(&nuc980_wdd, nowayout);

	nuc980_wdd.bootstatus = __raw_readl(REG_RSTSTS) & (1 << 5) ? WDIOF_CARDRESET : 0;

	ret = watchdog_register_device(&nuc980_wdd);
	if (ret) {
		dev_err(&pdev->dev, "err register watchdog device\n");
		clk_disable(nuc980_wdt->clk);
		clk_put(nuc980_wdt->clk);
		return ret;
	}


	return 0;
}

static int nuc980wdt_remove(struct platform_device *pdev)
{
	watchdog_unregister_device(&nuc980_wdd);

	clk_disable(nuc980_wdt->eclk);
	clk_put(nuc980_wdt->eclk);
	clk_disable(nuc980_wdt->clk);
	clk_put(nuc980_wdt->clk);

	return 0;
}

static void nuc980wdt_shutdown(struct platform_device *pdev)
{
	nuc980wdt_stop(&nuc980_wdd);
}

#ifdef CONFIG_NUC980_WDT_WKUP
static u32 reg_save;
static int nuc980wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	unsigned long flags;

	reg_save = __raw_readl(REG_WDT_CTL);

	local_irq_save(flags);
	Unlock_RegWriteProtect();
	__raw_writel(__raw_readl(REG_WDT_CTL) & ~RSTEN, REG_WDT_CTL); //Disable WDT reset
	Lock_RegWriteProtect();
	local_irq_restore(flags);

	__raw_writel( (1 << 0) | __raw_readl(REG_WKUPSER0), REG_WKUPSER0); //Enable System WDT wakeup
	if (request_irq(IRQ_WDT, nuc980_wdt_interrupt, IRQF_NO_SUSPEND, "nuc980wdt", NULL)) {
		return -EBUSY;
	}
	enable_irq_wake(IRQ_WDT);

	return 0;
}

static int nuc980wdt_resume(struct platform_device *dev)
{
	unsigned long flags;

	local_irq_save(flags);
	Unlock_RegWriteProtect();
	__raw_writel(reg_save, REG_WDT_CTL);
	Lock_RegWriteProtect();
	local_irq_restore(flags);

	__raw_writel(RESET_COUNTER, REG_WDT_RSTCNT);
	disable_irq_wake(IRQ_WDT);
	free_irq(IRQ_WDT, NULL);

	return 0;
}

#else
#define nuc980wdt_suspend NULL
#define nuc980wdt_resume  NULL
#endif /* CONFIG_NUC980_WDT_WKUP */

static const struct of_device_id nuc980_wdt_of_match[] = {
	{ .compatible = "nuvoton,nuc980-wdt" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_wdt_of_match);

static struct platform_driver nuc980wdt_driver = {
	.probe		= nuc980wdt_probe,
	.remove		= nuc980wdt_remove,
	.shutdown	= nuc980wdt_shutdown,
	.suspend	= nuc980wdt_suspend,
	.resume		= nuc980wdt_resume,
	.driver		= {
		.name	= "nuc980-wdt",
		.of_match_table = of_match_ptr(nuc980_wdt_of_match),
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(nuc980wdt_driver);

MODULE_DESCRIPTION("Watchdog driver for NUC980");
MODULE_LICENSE("GPL");
