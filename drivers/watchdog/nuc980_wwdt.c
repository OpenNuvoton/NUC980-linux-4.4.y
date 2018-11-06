/*
 * Copyright (c) 2018 Nuvoton technology corporation.
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
#include <mach/regs-wwdt.h>

#define RELOAD_WORD	0x00005AA5

/*
 *  Select WWDT clock source from 12M/512.
 *  Here we set compare window to 32, and prescale to 1024 after init.
 *
 *  So WWDT time out every 2048 * 32 * (512/12000000) = 2.79 second,
 *  And software has another 2.79 second window period to reload
 *  WWDT counter by writing RELOAD_WORD to REG_WWDT_RLD register.
 */
#define WWDT_CONFIG	0x00200F01

struct nuc980_wwdt {
	struct resource		*res;
	struct platform_device	*pdev;
};

static struct nuc980_wwdt *nuc980_wwdt;

static int nuc980wwdt_ping(struct watchdog_device *wdd)
{
	__raw_writel(RELOAD_WORD, REG_WWDT_RLD);
	return 0;
}

static int nuc980wwdt_start(struct watchdog_device *wdd)
{
	__raw_writel(WWDT_CONFIG, REG_WWDT_CR);
	return 0;
}

/*
 *  This function always return error, we define it here simply because stop() is mandatory operation.
 *  Due to the fact that WWDT register can only be programmed once, so there is NO WAY OUT!!!
 */
static int nuc980wwdt_stop(struct watchdog_device *wdd)
{

	return -EBUSY;
}

static unsigned int nuc980wwdt_get_timeleft(struct watchdog_device *wdd)
{
	unsigned int time_left;

	time_left = __raw_readl(REG_WWDT_CVR) / 32;

	return time_left;
}

static const struct watchdog_info nuc980wwdt_info = {
	.identity	= "nuc980 window watchdog",
	.options	= WDIOF_KEEPALIVEPING,
};

static struct watchdog_ops nuc980wwdt_ops = {
	.owner 	= THIS_MODULE,
	.start 	= nuc980wwdt_start,
	.stop 	= nuc980wwdt_stop,
	.ping 	= nuc980wwdt_ping,
	.get_timeleft = nuc980wwdt_get_timeleft,
};

static struct watchdog_device nuc980_wdd = {
	.status	= WATCHDOG_NOWAYOUT_INIT_STATUS,
	.info 	= &nuc980wwdt_info,
	.ops 	= &nuc980wwdt_ops,
	.timeout = 2,
};


static int nuc980wwdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct clk *clk, *eclk, *clkmux, *clksrc;

	nuc980_wwdt = devm_kzalloc(&pdev->dev, sizeof(struct nuc980_wwdt), GFP_KERNEL);
	if (!nuc980_wwdt)
		return -ENOMEM;

	nuc980_wwdt->pdev = pdev;

	nuc980_wwdt->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (nuc980_wwdt->res == NULL) {
		dev_err(&pdev->dev, "no memory resource specified\n");
		return -ENOENT;
	}

	if (!devm_request_mem_region(&pdev->dev, nuc980_wwdt->res->start,
				resource_size(nuc980_wwdt->res), pdev->name)) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		return -ENOENT;
	}


	clkmux = clk_get(NULL, "wwdt_eclk_mux");
	if (IS_ERR(clkmux)) {
		dev_err(&pdev->dev, "failed to get wwdt clock mux\n");
		ret = PTR_ERR(clkmux);
		return ret;
	}

	clksrc = clk_get(NULL, "xin512_div");
	if (IS_ERR(clksrc)) {
		dev_err(&pdev->dev, "failed to get xin/512 clk\n");
		ret = PTR_ERR(clksrc);
		return ret;
	}


	clk_set_parent(clkmux, clksrc);

	clk = clk_get(NULL, "wwdt");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get wwdt clock\n");
		ret = PTR_ERR(clk);
		return ret;
	}

	clk_prepare(clk);
	clk_enable(clk);

	eclk = clk_get(NULL, "wwdt_eclk");
	if (IS_ERR(eclk)) {
		dev_err(&pdev->dev, "failed to get wwdt clock\n");
		ret = PTR_ERR(eclk);
		return ret;
	}

	clk_prepare(eclk);
	clk_enable(eclk);

	ret = watchdog_register_device(&nuc980_wdd);
	if (ret) {
		dev_err(&pdev->dev, "err register window watchdog device\n");
		return ret;
	}

	return 0;

}

static int nuc980wwdt_remove(struct platform_device *pdev)
{

	watchdog_unregister_device(&nuc980_wdd);
	// There's no way out~~~~
	/* You can check-out any time you like
	   But you can never leave! */

	return 0;
}

#ifdef CONFIG_PM
static int nuc980wwdt_suspend(struct platform_device *dev, pm_message_t state)
{
        return 0;
}

static int nuc980wwdt_resume(struct platform_device *dev)
{
        return 0;
}

#else
#define nuc980wwdt_suspend NULL
#define nuc980wwdt_resume  NULL
#endif /* CONFIG_PM */

static const struct of_device_id nuc980_wwdt_of_match[] = {
	{ .compatible = "nuvoton,nuc980-wwdt" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_wwdt_of_match);


static struct platform_driver nuc980wwdt_driver = {
	.probe		= nuc980wwdt_probe,
	.remove		= nuc980wwdt_remove,
	.suspend        = nuc980wwdt_suspend,
        .resume         = nuc980wwdt_resume,
	.driver		= {
		.name	= "nuc980-wwdt",
		.of_match_table = of_match_ptr(nuc980_wwdt_of_match),
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(nuc980wwdt_driver);

MODULE_DESCRIPTION("NUC980 Window Watchdog Timer Driver");
MODULE_LICENSE("GPL");
