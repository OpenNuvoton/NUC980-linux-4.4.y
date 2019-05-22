/*
 * linux/driver/usb/host/ohci-nuc980.c
 *
 * Copyright (c) 2017 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/of.h>
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <linux/clk.h>

#include <mach/map.h>


#ifdef CONFIG_USE_OF
static int  of_pm_vbus_off;
static int  of_mfp_setting;
#endif


/**
 * usb_hcd_ppc_soc_probe - initialize On-Chip HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
static int usb_hcd_nuc980_probe(const struct hc_driver *driver,
                                struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd;
	struct ohci_hcd *ohci ;
#ifdef CONFIG_USE_OF
	u32   val32[2];
#endif
	struct pinctrl *p=NULL;

	printk("usb_hcd_nuc980_probe, id = %d, name: %s, %d\n", pdev->id, dev_name(&pdev->dev), (int)p);

	/*------------------------------------------------------------*/
	/*  USBH Lite initialization                                  */
	/*------------------------------------------------------------*/
#ifdef CONFIG_USB_NUC980_USBH_LITE0
	if (pdev->id == 1) {
#if defined(CONFIG_LITE0_PB4_PB6)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite0_pb4_pb6");
#elif defined(CONFIG_LITE0_PB5_PB7)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite0_pb5_pb7");
#elif defined(CONFIG_LITE0_PB10_PB9)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite0_pb10_pb9");
#elif defined(CONFIG_LITE0_PD15_PD14)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite0_pd15_pd15");
#endif
		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve usbh_lite0 pin\n");
			retval = PTR_ERR(p);
		}
	}
#endif   // CONFIG_USB_NUC980_USBH_LITE0

#ifdef CONFIG_USB_NUC980_USBH_LITE1
	if (pdev->id == 2) {
#if defined(CONFIG_LITE1_PE1_PE0)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite1_pe1_pe0");
#elif defined(CONFIG_LITE1_PF1_PF0)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite1_pf1_pf0");
#endif
		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve usbh_lite1 pin\n");
			retval = PTR_ERR(p);
		}
	}
#endif  // CONFIG_USB_NUC980_USBH_LITE1

#ifdef CONFIG_USB_NUC980_USBH_LITE2
	if (pdev->id == 3) {
#if defined(CONFIG_LITE2_PE3_PE2)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite2_pe3_pe2");
#elif defined(CONFIG_LITE2_PF3_PF2)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite2_pf3_pf2");
#endif
		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve usbh_lite2 pin\n");
			retval = PTR_ERR(p);
		}
	}
#endif  // CONFIG_USB_NUC980_USBH_LITE2

#ifdef CONFIG_USB_NUC980_USBH_LITE3
	if (pdev->id == 4) {
#if defined(CONFIG_LITE3_PE5_PE4)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite3_pe5_pe4");
#elif defined(CONFIG_LITE3_PF3_PF2)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite3_pf5_pf4");
#endif
		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve usbh_lite3 pin\n");
			retval = PTR_ERR(p);
		}
	}
#endif  // CONFIG_USB_NUC980_USBH_LITE3

#ifdef CONFIG_USB_NUC980_USBH_LITE4
	if (pdev->id == 5) {
#if defined(CONFIG_LITE4_PE7_PE6)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite4_pe7_pe6");
#elif defined(CONFIG_LITE4_PF7_PF6)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite4_pf7_pf6");
#elif defined(CONFIG_LITE4_PG10_PA15)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite4_pg10_pa15");
#elif defined(CONFIG_LITE4_PB13_PF6)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite4_pb13_pf6");
#endif
		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve usbh_lite4 pin\n");
			retval = PTR_ERR(p);
		}
	}
#endif  // CONFIG_USB_NUC980_USBH_LITE4

#ifdef CONFIG_USB_NUC980_USBH_LITE5
	if (pdev->id == 6) {
#if defined(CONFIG_LITE5_PE9_PE8)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite5_pe9_pe8");
#elif defined(CONFIG_LITE5_PF9_PF8)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite5_pf9_pf8");
#elif defined(CONFIG_LITE5_PA14_PA13)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite5_pa14_pa13");
#elif defined(CONFIG_LITE5_PB12_PB11)
		p = devm_pinctrl_get_select(&pdev->dev, "usbh_lite5_pb12_pb11");
#endif
		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve usbh_lite5 pin\n");
			retval = PTR_ERR(p);
		}
	}
#endif  // CONFIG_USB_NUC980_USBH_LITE5


#ifdef CONFIG_USE_OF

	devm_pinctrl_get_select_default(&pdev->dev);

	if ((__raw_readl(REG_MFP_GPE_H) & 0x000F0000) == 0x00010000)
		of_mfp_setting = 1;
	else
		of_mfp_setting = 0;

	//printk("of_mfp_setting = %d\n", of_mfp_setting);

	if (of_property_read_u32_array(pdev->dev.of_node, "ov_active", val32, 1) == 0) {
		// printk("Over-current active level %s...\n", val32[0] ? "high" : "low");
		if (val32[0]) {
			/* set over-current active high */
			__raw_writel(__raw_readl(NUC980_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC980_VA_OHCI+0x204));
		} else {
			/* set over-current active low */
			__raw_writel(__raw_readl(NUC980_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC980_VA_OHCI+0x204));
		}
	}

	if (of_property_read_u32_array(pdev->dev.of_node, "pm_vbus_off", val32, 1) == 0) {
		if (val32[0])
			of_pm_vbus_off = 1;
		else
			of_pm_vbus_off = 0;
	} else {
		of_pm_vbus_off = 0;
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

#else

#if !defined(CONFIG_USB_NUC980_EHCI)

	/* multi-function pin select */
#if defined(CONFIG_NUC980_USBH_PWREN_OVC_ON_)
	p = devm_pinctrl_get_select(&pdev->dev, "usbh_pwren_ovc_on");
	if (IS_ERR(p)) {
		dev_err(&pdev->dev, "unable to reserve USBH_PWREN and USB_OVC pins\n");
		retval = PTR_ERR(p);
		/* set over-current active high */
		__raw_writel(__raw_readl(NUC980_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC980_VA_OHCI+0x204));
	} else {
		/* set over-current active low */
		__raw_writel(__raw_readl(NUC980_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC980_VA_OHCI+0x204));
	}

#elif defined(CONFIG_NUC980_USBH_PWREN_ON_)

	p = devm_pinctrl_get_select(&pdev->dev, "usbh_pwren_on");
	if (IS_ERR(p)) {
		dev_err(&pdev->dev, "unable to reserve USBH_PWREN pin\n");
		retval = PTR_ERR(p);
	}
	/* set over-current active high */
	__raw_writel(__raw_readl(NUC980_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC980_VA_OHCI+0x204));

#elif defined(CONFIG_NUC980_USBH_OVC_ON_)
	p = devm_pinctrl_get_select(&pdev->dev, "usbh_ovc_on");
	if (IS_ERR(p)) {
		dev_err(&pdev->dev, "unable to reserve USB_OVC pin\n");
		retval = PTR_ERR(p);
		/* set over-current active high */
		__raw_writel(__raw_readl(NUC980_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC980_VA_OHCI+0x204));
	} else {
		/* set over-current active low */
		__raw_writel(__raw_readl(NUC980_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC980_VA_OHCI+0x204));
	}
#elif defined(CONFIG_NUC980_USBH_PWREN_OVC_OFF_)

	/* set over-current active high */
	__raw_writel(__raw_readl(NUC980_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC980_VA_OHCI+0x204));

#endif

#endif   // !CONFIG_USB_NUC980_EHCI

#endif   // CONFIG_USE_OF

#ifdef CONFIG_USE_OF
	if (strcmp(pdev->name, "b0017000.usbh_ohci") != 0)
		return 0;         /* return here if is USBH Lite device */
#else
	if (pdev->id != 0)
		return 0;         /* return here if is USBH Lite device */
#endif

	clk_prepare(clk_get(NULL, "usb_eclk"));
	clk_enable(clk_get(NULL, "usb_eclk"));

	/* enable USB Host clock */
	clk_prepare(clk_get(NULL, "usbh_hclk"));
	clk_enable(clk_get(NULL, "usbh_hclk"));

	/* enable PHY 0/1 */
	__raw_writel(0x160, (volatile void __iomem *)(NUC980_VA_EHCI+0xC4));
	__raw_writel(0x520, (volatile void __iomem *)(NUC980_VA_EHCI+0xC8));

	hcd = usb_create_hcd(driver, &pdev->dev, "nuc980-ohci");
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("ohci probe request_mem_region failed");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ohci error mapping memory\n");
		retval = -ENOMEM;
		goto err2;
	}

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);

	//printk("Port status: 0x%x, 0x%x\n", __raw_readl((volatile void __iomem *)(NUC980_VA_EHCI+0x64)), __raw_readl((volatile void __iomem *)(NUC980_VA_EHCI+0x68)));
	//printk("Port status: 0x%x, 0x%x\n", __raw_readl((volatile void __iomem *)(NUC980_VA_OHCI+0x54)), __raw_readl((volatile void __iomem *)(NUC980_VA_OHCI+0x58)));

	if (retval == 0)
		return retval;

	pr_debug("Removing nuc980 OHCI USB Controller\n");

	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:

	usb_put_hcd(hcd);
	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_ppc_soc_remove - shutdown processing for On-Chip HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_ppc_soc_probe().
 * It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_nuc980_remove(struct usb_hcd *hcd,
                                  struct platform_device *dev)
{
	usb_remove_hcd(hcd);

	//pr_debug("stopping PPC-SOC USB Controller\n");

	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}


static int ohci_nuc980_start (struct usb_hcd *hcd)
{
	struct ohci_hcd *ohci = hcd_to_ohci (hcd);
	int ret;

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run (ohci)) < 0) {
		printk("can't start %s", hcd->self.bus_name);
		ohci_stop (hcd);
		return ret;
	}

	return 0;
}


static const struct hc_driver ohci_nuc980_hc_driver = {
	.description =      hcd_name,
	.product_desc =     "Nuvoton NUC980 OHCI Host Controller",
	.hcd_priv_size =    sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =          ohci_irq,
	.flags =        HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =        ohci_nuc980_start,
	.stop =         ohci_stop,
	.shutdown =      ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =      ohci_urb_enqueue,
	.urb_dequeue =      ohci_urb_dequeue,
	.endpoint_disable = ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =  ohci_hub_status_data,
	.hub_control =      ohci_hub_control,
#ifdef  CONFIG_PM
	.bus_suspend =      ohci_bus_suspend,
	.bus_resume =       ohci_bus_resume,
#endif
	.start_port_reset = ohci_start_port_reset,
};


static int ohci_hcd_nuc980_drv_probe(struct platform_device *pdev)
{
	int ret;

	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_nuc980_probe(&ohci_nuc980_hc_driver, pdev);
	return ret;
}

static int ohci_hcd_nuc980_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_nuc980_remove(hcd, pdev);
	return 0;
}


#if defined(CONFIG_PM)

static int ohci_nuc980_pm_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	bool do_wakeup = device_may_wakeup(dev);
	struct platform_device  *pdev = to_platform_device(dev);
	int  ret;

#ifdef CONFIG_USE_OF
	if (strcmp(pdev->name, "b0017000.usbh_ohci") != 0)
		return 0;         /* return here if is USBH Lite device */
#else
	if (pdev->id != 0)
		return 0;         /* return here if is USBH Lite device */
#endif

	ret = ohci_suspend(hcd, do_wakeup);

	/* Suspend PHY0 and PHY1; this will turn off PHY power. */
	__raw_writel(0x60, NUC980_VA_EHCI+0xC4);
	__raw_writel(0x20, NUC980_VA_EHCI+0xC8);

#if !defined(CONFIG_USB_NUC980_EHCI)
#ifdef CONFIG_USE_OF
	if (of_pm_vbus_off) {
		if (of_mfp_setting == 1) {
			__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1<<12), REG_GPIOE_DOUT);     // PE.12 output low
			__raw_writel(((__raw_readl(REG_GPIOE_MODE) & 0xFCFFFFFF) | (1<<24)), REG_GPIOE_MODE);   // PE.12 output mode
			__raw_writel(__raw_readl(REG_MFP_GPE_H) & 0xFFF0FFFF, REG_MFP_GPE_H);     // PE.12 GPIO mode
		}
	}
#else   /* !CONFIG_USE_OF */

#if defined(CONFIG_USB_NUC980_PM_VBUS_OFF) && defined(CONFIG_NUC980_USBH_PWREN_ON_)
	/* turn off port power */
	__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1<<12), REG_GPIOE_DOUT);     // PE.12 output low
	__raw_writel(((__raw_readl(REG_GPIOE_MODE) & 0xFCFFFFFF) | (1<<24)), REG_GPIOE_MODE);   // PE.12 output mode
	__raw_writel(__raw_readl(REG_MFP_GPE_H) & 0xFFF0FFFF, REG_MFP_GPE_H);     // PE.12 GPIO mode
#endif

#endif  /* end of CONFIG_USE_OFF */
#endif  /* end of !CONFIG_USB_NUC980_EHCI */

	return ret;
}

static int ohci_nuc980_pm_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct platform_device  *pdev = to_platform_device(dev);

#ifdef CONFIG_USE_OF
	if (strcmp(pdev->name, "b0017000.usbh_ohci") != 0)
		return 0;         /* return here if is USBH Lite device */
#else
	if (pdev->id != 0)
		return 0;         /* return here if is USBH Lite device */
#endif

#if !defined(CONFIG_USB_NUC980_EHCI)
#ifdef CONFIG_USE_OF
	if (of_pm_vbus_off) {
		if (of_mfp_setting == 1) {
			__raw_writel(__raw_readl(REG_MFP_GPE_H) | 0x00010000, REG_MFP_GPE_H);       // PE.12 for USBH_PWREN
		}
	}
#else  /* !CONFIG_USE_OF */

#if defined(CONFIG_USB_NUC980_PM_VBUS_OFF) && defined(CONFIG_NUC980_USBH_PWREN_ON_)
	__raw_writel(__raw_readl(REG_MFP_GPE_H) | 0x00010000, REG_MFP_GPE_H);       // PE.12 for USBH_PWREN
#endif

#endif  /* end of CONFIG_USE_OF */
#endif  /* !CONFIG_USB_NUC980_EHCI */

	/* re-enable PHY0 and PHY1 */
	__raw_writel(0x160, NUC980_VA_EHCI+0xC4);
	__raw_writel(0x520, NUC980_VA_EHCI+0xC8);

	ohci_resume(hcd, false);

	return 0;
}
#else
#define ohci_nuc980_pm_suspend  NULL
#define ohci_nuc980_pm_resume   NULL
#endif

static const struct dev_pm_ops ohci_nuc980_dev_pm_ops = {
	.suspend         = ohci_nuc980_pm_suspend,
	.resume          = ohci_nuc980_pm_resume,
};


static const struct of_device_id nuc980_ohci_of_match[] = {
	{ .compatible = "nuvoton,nuc980-ohci" },
	{ .compatible = "nuvoton,nuc980-usbh-lite0" },
	{ .compatible = "nuvoton,nuc980-usbh-lite1" },
	{ .compatible = "nuvoton,nuc980-usbh-lite2" },
	{ .compatible = "nuvoton,nuc980-usbh-lite3" },
	{ .compatible = "nuvoton,nuc980-usbh-lite4" },
	{ .compatible = "nuvoton,nuc980-usbh-lite5" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_ohci_of_match);


static struct platform_driver ohci_hcd_nuc980_driver = {
	.probe      = ohci_hcd_nuc980_drv_probe,
	.remove     = ohci_hcd_nuc980_drv_remove,

	.driver     = {
		.name   = "nuc980-ohci",
		.pm     = &ohci_nuc980_dev_pm_ops,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_ohci_of_match),
	},
};
