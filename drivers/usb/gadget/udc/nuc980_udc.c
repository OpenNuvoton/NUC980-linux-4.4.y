/*
 * linux/drivers/usb/gadget/udc/nuc980_udc.c
 *
 * Nuvoton NUC980 MCU on-chip full speed USB device controllers
 *
 * Copyright (C) 2018 Nuvoton Technology Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/proc_fs.h>
#include <linux/prefetch.h>
#include <linux/usb/ch9.h>
#include <linux/of.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <asm/byteorder.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <mach/regs-usbd.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include "nuc980_udc.h"

//#define pr_devel printk

#define DRIVER_DESC     "NUVOTON USB Device Controller Gadget"
#define DRIVER_VERSION  "16 March 2018"
#define DRIVER_AUTHOR   "shirley <clyu2@nuvoton.com>"

#define USBD_TIMEOUT	(10000)
u32 volatile usb_vaddr, usb_paddr;

static const char gadget_name [] = "nuc980-usbdev";
static const char driver_desc [] = DRIVER_DESC;

static const struct {
	const char *name;
	const struct usb_ep_caps caps;
} ep_info[] = {
#define EP_INFO(_name, _caps) \
	{ \
		.name = _name, \
		.caps = _caps, \
	}

	EP_INFO("ep0",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_CONTROL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep1",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep2",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep3",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep4",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep5",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep6",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep7",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep8",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep9",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep10",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep11",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep12",
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ALL, USB_EP_CAPS_DIR_ALL)),

#undef EP_INFO
};

#define ep0name		ep_info[0].name

#define EP0_FIFO_SIZE           64
#define EP_FIFO_SIZE            512

static void udc_isr_reset(struct nuc980_udc *udc);
static void udc_isr_dma(struct nuc980_udc *udc);
static void udc_isr_ctrl_pkt(struct nuc980_udc *udc);
static void udc_isr_update_dev(struct nuc980_udc *udc);
static void nuc980_udc_enable(struct nuc980_udc *udc);
static void nuc980_udc_disable(struct nuc980_udc *udc);


static void done(struct nuc980_ep *ep, struct nuc980_request *req, int status)
{
	struct nuc980_udc *udc = ep->dev;

	list_del_init(&req->queue); //del req->queue from ep->queue

	if (list_empty(&ep->queue))
	{
		if (ep->index)
			__raw_writel(0, udc->base + REG_USBD_EPA_EPINTEN + 0x28*(ep->index-1));
	}
	else
	{
		__raw_writel(ep->irq_enb, udc->base + REG_USBD_EPA_EPINTEN + 0x28*(ep->index-1));
	}

	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);
	req->req.complete(&ep->ep, &req->req);
}

static void nuke (struct nuc980_udc *udc, struct nuc980_ep *ep, int status)
{
	while (!list_empty (&ep->queue))
	{
		struct nuc980_request *req;
		req = list_entry(ep->queue.next, struct nuc980_request, queue);
		done(ep, req, status);
	}
}


/*
 *  write_packet
 */
static inline int write_packet(struct nuc980_ep *ep, struct nuc980_request *req)
{
	struct nuc980_udc *udc = ep->dev;
	unsigned total, len;
	u8  *buf;
	u32 i;
	unsigned int volatile timeout;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);
	total = req->req.length - req->req.actual;
	if (ep->ep.maxpacket < total)
		len = ep->ep.maxpacket;
	else
		len = total;

	if (ep->ep_num == 0)
	{
		for (i=0; i<len; i++)
		{
			__raw_writeb( *buf++ & 0xff, udc->base + REG_USBD_CEPDAT);
		}
		__raw_writel(len, udc->base + REG_USBD_CEPTXCNT);
		req->req.actual += len;
	}
	else
	{
		usb_gadget_map_request(&udc->gadget, &req->req, ep->ep_dir);
		buf = req->req.buf + req->req.actual;

		if (len == 0)
		{
			__raw_writel(USB_EP_RSPCTL_ZEROLEN, udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));
		}
		else
		{
			memcpy((char *)usb_vaddr, (char *)buf, len);
			__raw_writel((__raw_readl(udc->base + REG_USBD_DMACTL)&0xe0) | 0x110 | ep->ep_num,
						 udc->base + REG_USBD_DMACTL);// bulk in, write
			__raw_writel(0, udc->base + REG_USBD_EPA_EPINTEN + (0x28* (ep->index-1)));
			__raw_writel((USBD_BUSINTEN_DMADONEIEN | USBD_BUSINTEN_RSTIEN | USBD_BUSINTEN_SUSPENDIEN | USBD_BUSINTEN_VBUSDETIEN), udc->base + REG_USBD_BUSINTEN);
			//__raw_writel((u32)(req->req.dma + req->req.actual), udc->base + REG_USBD_DMAADDR);
			__raw_writel((u32)usb_paddr, udc->base + REG_USBD_DMAADDR);//Tell DMA the memory physcal address
			__raw_writel(len, udc->base + REG_USBD_DMACNT);
			__raw_writel(0x20, udc->base + REG_USBD_BUSINTSTS);

			__raw_writel(__raw_readl(udc->base + REG_USBD_DMACTL)|0x00000020, udc->base + REG_USBD_DMACTL);
			timeout = 0;
			while (!(__raw_readl(udc->base + REG_USBD_BUSINTSTS) & 0x20))
			{
				if (!(__raw_readl(udc->base + REG_USBD_PHYCTL) &  0x80000000))			/* Exit when USB Un-Plug */
				{
					//printk("unplug-1  0x%x\n", __raw_readl(udc->base + REG_USBD_PHYCTL));
					break;
				}
				if (timeout > USBD_TIMEOUT)
				{
					__raw_writel(0x80, udc->base + REG_USBD_DMACTL);
					__raw_writel(0x00, udc->base + REG_USBD_DMACTL);
					__raw_writel(__raw_readl(udc->base + REG_USBD_CEPCTL)|USB_CEPCTL_FLUSH, udc->base + REG_USBD_CEPCTL);
					__raw_writel(USB_EP_RSPCTL_FLUSH|USB_EP_RSPCTL_TOGGLE, udc->base + REG_USBD_EPA_EPRSPCTL + 0x28*(ep->index-1));
					//printk("tout-1 0x%x, 0x%x\n", __raw_readl(udc->base + REG_USBD_DMACTL), __raw_readl(udc->base + REG_USBD_PHYCTL));
					break;
				}
				timeout++;
			}
			__raw_writel(0x20, udc->base + REG_USBD_BUSINTSTS);
		}
		__raw_writel((__raw_readl(udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1)) & 0x16)|USB_EP_RSPCTL_SHORTTXEN, udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));
		__raw_writel(len, udc->base + REG_USBD_EPA_EPTXCNT+0x28*(ep->index-1));
		req->req.actual += len;
	}

	return len;
}

/*
 *  write_fifo
 */
// return:  0 = still running, 1 = completed, negative = errno
static int write_fifo(struct nuc980_ep *ep, struct nuc980_request *req)
{
	u32 len;

	len = write_packet(ep, req);

	/* last packet is often short (sometimes a zlp) */

	if (req->req.length == req->req.actual/* && !req->req.zero*/)
	{
		done(ep, req, 0);
		return 1;
	}
	else
	{
		return 0;
	}
}

static inline int read_packet(struct nuc980_ep *ep,u8 *buf, struct nuc980_request *req, u16 cnt)
{
	struct nuc980_udc *udc = ep->dev;
	unsigned int data, i;
	unsigned int volatile timeout;

	if (ep->ep_num == 0)
	{ //ctrl pipe don't use DMA

		for (i=0; i<cnt; i++)
		{
			data = __raw_readb(udc->base + REG_USBD_CEPDAT);
			*buf++ = data & 0xFF;
		}
		req->req.actual += cnt;
	}
	else
	{
		usb_gadget_map_request(&udc->gadget, &req->req, ep->ep_dir);

		__raw_writel((__raw_readl(udc->base + REG_USBD_DMACTL) & 0xe0)|ep->ep_num, udc->base + REG_USBD_DMACTL);   //read
//		__raw_writel((u32)(req->req.dma + req->req.actual), udc->base + REG_USBD_DMAADDR);
		__raw_writel((u32)usb_paddr, udc->base + REG_USBD_DMAADDR);
		__raw_writel(cnt, udc->base + REG_USBD_DMACNT);
		__raw_writel(0x20, udc->base + REG_USBD_BUSINTSTS);
		__raw_writel(__raw_readl(udc->base + REG_USBD_DMACTL)|0x00000020, udc->base + REG_USBD_DMACTL);
		timeout = 0;
		while (!(__raw_readl(udc->base + REG_USBD_BUSINTSTS) & 0x20))
		{
			if (!(__raw_readl(udc->base + REG_USBD_PHYCTL) &  0x80000000))			/* Exit when USB Un-Plug */
			{
				//printk("unplug-2  0x%x\n", __raw_readl(udc->base + REG_USBD_PHYCTL));
				break;
			}
			if (timeout > USBD_TIMEOUT)
			{
				__raw_writel(0x80, udc->base + REG_USBD_DMACTL);
				__raw_writel(0x00, udc->base + REG_USBD_DMACTL);
				__raw_writel(__raw_readl(udc->base + REG_USBD_CEPCTL)|USB_CEPCTL_FLUSH, udc->base + REG_USBD_CEPCTL);
				__raw_writel(USB_EP_RSPCTL_FLUSH|USB_EP_RSPCTL_TOGGLE, udc->base + REG_USBD_EPA_EPRSPCTL + 0x28*(ep->index-1));
				//printk("tout-2  0x%x, 0x%x\n", __raw_readl(udc->base + REG_USBD_DMACTL), __raw_readl(udc->base + REG_USBD_PHYCTL));
				break;
			}
			timeout++;
		}
		__raw_writel(0x20, udc->base + REG_USBD_BUSINTSTS);
		memcpy((char *)buf, (char *)usb_vaddr, cnt);
		req->req.actual += cnt;
	}

	return cnt;
}

// return:  0 = still running, 1 = queue empty, negative = errno
static int read_fifo(struct nuc980_ep *ep, struct nuc980_request *req, u16 cnt)
{
	u8 *buf;
	unsigned bufferspace;
	int is_last=1;
	int fifo_count = 0;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;

	if (cnt > ep->ep.maxpacket)
		cnt = ep->ep.maxpacket;
	if (cnt > bufferspace) {
		pr_devel("%s buffer overflow\n", ep->ep.name);
		req->req.status = -EOVERFLOW;
		cnt = bufferspace;
	}
	fifo_count = read_packet(ep, buf, req, cnt);

	if (req->req.length == req->req.actual)
		done(ep, req, 0);
	else if (fifo_count && fifo_count < ep->ep.maxpacket)
	{
		done(ep, req, 0);
		/* overflowed this request?  flush extra data */
//		if (req->req.length != req->req.actual)
//		{
//			pr_devel("%s(): EOVERFLOW set\n", __FUNCTION__);
//			if (req->req.short_not_ok)
//				req->req.status = -EOVERFLOW;   //device read less then host write
//		}
	}
	else
		is_last = 0;

	return is_last;
}


void paser_usb_irq(struct nuc980_udc *udc, int irq)
{
	__raw_writel(irq, udc->base + REG_USBD_BUSINTSTS);
	if (irq & USBD_BUSINTSTS_RSTIF)
	{
		udc_isr_reset(udc);
	}

	if (irq & USBD_BUSINTSTS_RESUMEIF)
	{
		__raw_writel((USBD_BUSINTEN_RSTIEN|USBD_BUSINTEN_SUSPENDIEN|USBD_BUSINTEN_VBUSDETIEN), udc->base + REG_USBD_BUSINTEN);
	}

	if (irq & USBD_BUSINTSTS_SUSPENDIF)
	{
		__raw_writel((USBD_BUSINTEN_RSTIEN|USBD_BUSINTEN_RESUMEIEN|USBD_BUSINTEN_VBUSDETIEN), udc->base + REG_USBD_BUSINTEN);
	}

	if (irq & USBD_BUSINTSTS_HISPDIF)
	{
		udc->gadget.speed = USB_SPEED_HIGH;
		udc->usb_address = 0;       //zero
		__raw_writel(USBD_CEPINTEN_SETUPPKIEN, udc->base + REG_USBD_CEPINTEN);
	}

	if (irq & USBD_BUSINTSTS_DMADONEIF)
	{
		udc_isr_dma(udc);
	}

	if (irq & USBD_BUSINTSTS_VBUSDETIF)
	{
		if (__raw_readl(udc->base + REG_USBD_PHYCTL) & USBD_PHYCTL_VBUSDET)
		{
//printk("plug-in\n");
			__raw_writel(__raw_readl(udc->base + REG_USBD_CEPCTL)|USB_CEPCTL_FLUSH, udc->base + REG_USBD_CEPCTL);
			nuc980_udc_enable(udc);
		}
		else
		{
//printk("plug-out\n");
			nuc980_udc_disable(udc);
			nuke(udc, &udc->ep[0], -ESHUTDOWN);
		}
	}

	return ;
}

void paser_irq_cep(struct nuc980_udc *udc, u32 irq)
{
	struct nuc980_ep *ep = &udc->ep[0];
	struct nuc980_request *req;
	int is_last = 1;
	unsigned int volatile timeout;

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct nuc980_request, queue);

	if (irq & USBD_CEPINTSTS_SETUPPKIF)
	{
		udc->ep0state = EP0_IDLE;
		udc->setup_ret = 0;
		udc_isr_ctrl_pkt(udc);
	}

	if (irq & USBD_CEPINTSTS_RXPKIF)
	{
		if (udc->ep0state == EP0_OUT_DATA_PHASE)
		{
			if (req)
				is_last = read_fifo(ep, req, __raw_readl(udc->base + REG_USBD_CEPDATCNT));

			__raw_writel(USBD_CEPINTSTS_STSDONEIF, udc->base + REG_USBD_CEPINTSTS);
			if (!is_last)
			{
				__raw_writel(USBD_CEPINTEN_SETUPPKIEN|USBD_CEPINTEN_RXPKIEN|USBD_CEPINTEN_STSDONEIEN, udc->base + REG_USBD_CEPINTEN);
			}
			else 
			{
				__raw_writel(USB_CEPCTL_NAKCLR, udc->base + REG_USBD_CEPCTL);
				__raw_writel(USBD_CEPINTEN_SETUPPKIEN|USBD_CEPINTEN_STSDONEIEN, udc->base + REG_USBD_CEPINTEN);
				udc->ep0state = EP0_END_XFER;
			}
		}
	}

	if (irq & USBD_CEPINTSTS_INTKIF)
	{
		if (udc->ep0state == EP0_IN_DATA_PHASE)
		{
			timeout = 0;
			while (1)
			{
				if (__raw_readl(udc->base + REG_USBD_CEPINTSTS) & 0x1000)	/* buffer empty */
					break;
				if (timeout > USBD_TIMEOUT)
				{
					printk("timeout!!\n");
					return;
				}
				timeout++;
			}
			if (req)
				is_last = write_fifo(ep,req);

			if (!is_last)
			{
				__raw_writel(USBD_CEPINTEN_SETUPPKIEN|USBD_CEPINTEN_STSDONEIEN|USBD_CEPINTEN_TXPKIEN|USBD_CEPINTEN_INTKIEN, udc->base + REG_USBD_CEPINTEN);
			}
			else 
			{
				if (udc->setup_ret >= 0)
					__raw_writel(USB_CEPCTL_NAKCLR, udc->base + REG_USBD_CEPCTL);
				__raw_writel(USBD_CEPINTEN_STSDONEIEN|USBD_CEPINTEN_TXPKIEN|USBD_CEPINTEN_SETUPPKIEN, udc->base + REG_USBD_CEPINTEN);

				if (udc->setup_ret < 0)
					udc->ep0state=EP0_IDLE;
				else if (udc->ep0state != EP0_IDLE)
					udc->ep0state=EP0_END_XFER;
			}
		}
	}

	if (irq & USBD_CEPINTSTS_STSDONEIF)
	{
		__raw_writel(USBD_CEPINTEN_SETUPPKIEN, udc->base + REG_USBD_CEPINTEN);
		udc_isr_update_dev(udc);
		udc->ep0state=EP0_IDLE;
		udc->setup_ret = 0;
	}
}


void paser_irq_nep(struct nuc980_ep *ep, u32 irq)
{
	struct nuc980_udc *udc = ep->dev;
	struct nuc980_request *req;
	unsigned int volatile timeout;

	if (list_empty(&ep->queue))
	{
		pr_devel("nep->queue is empty\n");
		req = 0;
	}
	else
	{
		req = list_entry(ep->queue.next, struct nuc980_request, queue);
	}

	if (irq & USBD_EPINTSTS_INTKIF)
	{
		__raw_writel(USBD_EPINTSTS_INTKIF, udc->base + REG_USBD_EPA_EPINTSTS + 0x28*(ep->index-1));
		if (ep->ep_type == USB_EP_CFG_TYPE_BULK)
		{
			if (__raw_readl(udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1)) & USB_EP_RSPCTL_SHORTTXEN)
			{
				pr_devel("send last packet\n");
				return;
			}
		}

		if (req == NULL)
		{
			__raw_writel(0, udc->base + REG_USBD_EPA_EPINTEN + 0x28*(ep->index-1));
			return;
		}

		timeout = 0;
		while (__raw_readl(udc->base + REG_USBD_DMACTL) & 0x20) //wait DMA complete
		{
			if (!(__raw_readl(udc->base + REG_USBD_PHYCTL) & USBD_PHYCTL_VBUSDET))
			{
				//printk("unplug-3 0x%x\n", __raw_readl(udc->base + REG_USBD_PHYCTL));
				return;
			}
			if (timeout > USBD_TIMEOUT)
			{
				__raw_writel(0x80, udc->base + REG_USBD_DMACTL);
				__raw_writel(0x00, udc->base + REG_USBD_DMACTL);
				__raw_writel(__raw_readl(udc->base + REG_USBD_CEPCTL)|USB_CEPCTL_FLUSH, udc->base + REG_USBD_CEPCTL);
				__raw_writel(USB_EP_RSPCTL_FLUSH|USB_EP_RSPCTL_TOGGLE, udc->base + REG_USBD_EPA_EPRSPCTL + 0x28*(ep->index-1));
				//printk("tout-3 0x%x, 0x%x\n", __raw_readl(udc->base + REG_USBD_DMACTL), __raw_readl(udc->base + REG_USBD_PHYCTL));
				return;
			}
			timeout++;
		}
		if (!write_fifo(ep, req))
			__raw_writel(0x40, udc->base + REG_USBD_EPA_EPINTEN + 0x28*(ep->index-1));
	}

	if (irq & USBD_EPINTSTS_TXPKIF)
	{
		__raw_writel(USBD_EPINTSTS_TXPKIF, udc->base + REG_USBD_EPA_EPINTSTS + 0x28*(ep->index-1));
	}

	if ((irq & USBD_EPINTSTS_RXPKIF) || (irq & USBD_EPINTSTS_SHORTRXIF))
	{
		__raw_writel(USBD_EPINTSTS_RXPKIF|USBD_EPINTSTS_SHORTRXIF, udc->base + REG_USBD_EPA_EPINTSTS + 0x28*(ep->index-1));
		if (req == NULL)
		{
			__raw_writel(0, udc->base + REG_USBD_EPA_EPINTEN + 0x28*(ep->index-1));
			__raw_writel(irq, udc->base + REG_USBD_EPA_EPINTSTS + 0x28*(ep->index-1));
			return;
		}
		read_fifo(ep, req, __raw_readl(udc->base + REG_USBD_EPA_EPDATCNT + 0x28*(ep->index-1)));
	}

	return ;
}


/*
 *      nuc980_udc_irq - interrupt handler
 */
static irqreturn_t nuc980_udc_irq(int irq, void *_dev)
{
	struct nuc980_udc *udc;
	u32 volatile IrqStL, IrqSt;
	int j;

	udc = (struct nuc980_udc *)(_dev);

	IrqStL = __raw_readl(udc->base + REG_USBD_GINTSTS) & __raw_readl(udc->base + REG_USBD_GINTEN);
	if (!IrqStL)
	{
		pr_err("Not our interrupt !\n");
		return IRQ_HANDLED;
	}

	if (IrqStL & USBD_GINTSTS_USBIF)
	{
		IrqSt = __raw_readl(udc->base + REG_USBD_BUSINTSTS) & __raw_readl(udc->base + REG_USBD_BUSINTEN);
		__raw_writel(IrqSt, udc->base + REG_USBD_BUSINTSTS);
		if (IrqSt && udc->driver)
			paser_usb_irq(udc, IrqSt);
	}

	if (IrqStL & USBD_GINTSTS_CEPIF)
	{
		IrqSt = __raw_readl(udc->base + REG_USBD_CEPINTSTS) & __raw_readl(udc->base + REG_USBD_CEPINTEN);
		__raw_writel(IrqSt, udc->base + REG_USBD_CEPINTSTS);
		if (IrqSt && udc->driver)
			paser_irq_cep(udc, IrqSt);
	}

	if (IrqStL & USBD_GINTSTS_EPIF)
	{
		IrqStL >>= 2;
		for (j = 0; j < NUC980_ENDPOINTS-1; j++)
		{
			if (IrqStL & (1 << j))
			{
				//in-token and out token interrupt can deal with one only
				IrqSt = __raw_readl(udc->base + REG_USBD_EPA_EPINTSTS+0x28*j) & __raw_readl(udc->base + REG_USBD_EPA_EPINTEN+0x28*j);
				if (IrqSt && udc->driver)
					paser_irq_nep(&udc->ep[j+1], IrqSt);
			}
		}
	}

	return IRQ_HANDLED;
}


static s32 sram_data[13][2] = {{0,0x40}};

//0-3F for Ctrl pipe
s32 get_sram_base(struct nuc980_udc *udc, u32 max)
{
	int i, cnt = 1, j;
	s32 start, end;

	for (i = 1; i < NUC980_ENDPOINTS; i++)
	{
		struct nuc980_ep *ep = &udc->ep[i];

		start = __raw_readl(udc->base + REG_USBD_EPA_EPBUFSTART+0x28*(ep->index-1));
		end = __raw_readl(udc->base + REG_USBD_EPA_EPBUFEND+0x28*(ep->index-1));
		if (end - start > 0)
		{
				sram_data[cnt][0] = start;
				sram_data[cnt][1] = end + 1;
				cnt++;
		}
	}
	if (cnt == 1)
		return 0x40;
	//sorting from small to big
	j= 1;
	while ((j<cnt))
	{
		for (i=0; i<cnt -j; i++)
		{
			if (sram_data[i][0]>sram_data[i+1][0])
			{
				start = sram_data[i][0];
				end = sram_data[i][1];
				sram_data[i][0] = sram_data[i+1][0];
				sram_data[i][1] = sram_data[i+1][1];
				sram_data[i+1][0] = start;
				sram_data[i+1][1] = end;
			}
		}
		j++;
	}

	for (i = 0; i< cnt-1; i++)
	{
		if (sram_data[i+1][0] - sram_data[i][1] >= max)
			return sram_data[i][1];
	}

	if (0x1000 - sram_data[cnt-1][1] >= max)
		return sram_data[cnt-1][1];

	return -ENOBUFS;
}

/*
 *  nuc980_ep_enable
 */
static int nuc980_ep_enable (struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct nuc980_udc *udc;
	struct nuc980_ep *ep;
	u32 max, tmp;
	unsigned long flags;
	u32 int_en_reg;
	s32 sram_addr;

	ep = container_of (_ep, struct nuc980_ep, ep);
	if (!_ep || !desc || _ep->name == ep0name || desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	udc = ep->dev;

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	max = usb_endpoint_maxp(desc);

	spin_lock_irqsave (&udc->lock, flags);
	_ep->maxpacket = max & 0x7ff;

	ep->ep.desc = desc;
	ep->bEndpointAddress = desc->bEndpointAddress;

	/* set max packet */
	if (ep->index != 0)
	{
		__raw_writel(max, udc->base + REG_USBD_EPA_EPMPS + 0x28*(ep->index-1));
		ep->ep.maxpacket = max;

		sram_addr = get_sram_base(udc, max);

		if (sram_addr < 0)
			return sram_addr;

		__raw_writel(sram_addr, udc->base + REG_USBD_EPA_EPBUFSTART+0x28*(ep->index-1));
		sram_addr = sram_addr + max;
		__raw_writel(sram_addr-1, udc->base + REG_USBD_EPA_EPBUFEND+0x28*(ep->index-1));
	}

	/* set type, direction, address; reset fifo counters */
	if (ep->index != 0)
	{
		ep->ep_num = desc->bEndpointAddress & ~USB_DIR_IN;
		ep->ep_dir = desc->bEndpointAddress &0x80 ? 1 : 0;
		ep->ep_type = ep->ep.desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
		if (ep->ep_type == USB_ENDPOINT_XFER_ISOC)
		{
			ep->ep_type = USB_EP_CFG_TYPE_ISO;
			ep->ep_mode = USB_EP_RSPCTL_MODE_AUTO;
		}
		else if (ep->ep_type == USB_ENDPOINT_XFER_BULK)
		{
			ep->ep_type = USB_EP_CFG_TYPE_BULK;
			ep->ep_mode = USB_EP_RSPCTL_MODE_AUTO;
		}
		else if (ep->ep_type == USB_ENDPOINT_XFER_INT)
		{
			ep->ep_type = USB_EP_CFG_TYPE_INT;
			ep->ep_mode = USB_EP_RSPCTL_MODE_MANUAL;
		}
		__raw_writel(USB_EP_RSPCTL_FLUSH|USB_EP_RSPCTL_TOGGLE, udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));
		__raw_writel(ep->ep_num<<4|ep->ep_dir<<3|ep->ep_type|USB_EP_CFG_VALID,
					 udc->base + REG_USBD_EPA_EPCFG+0x28*(ep->index-1));
		__raw_writel(ep->ep_mode, udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));

		/* enable irqs */
		int_en_reg = __raw_readl(udc->base + REG_USBD_GINTEN);
		__raw_writel(int_en_reg | (1<<(ep->index+1)), udc->base + REG_USBD_GINTEN);
		udc->irq_enbl = __raw_readl(udc->base + REG_USBD_GINTEN);

		if (ep->ep_type == USB_EP_CFG_TYPE_BULK)
		{
			if (ep->ep_dir)//IN
				ep->irq_enb = 0x40;
			else
			{
				ep->irq_enb = 0x1010;
			}
		}
		else if (ep->ep_type == USB_EP_CFG_TYPE_INT)
		{
			if (ep->ep_dir)//IN
				ep->irq_enb = 0x40;
			else
				ep->irq_enb = 0x10;
		}
		else if (ep->ep_type == USB_EP_CFG_TYPE_ISO)
		{
			if (ep->ep_dir)//IN
				ep->irq_enb = 0x40;
			else
				ep->irq_enb = 0x20;
		}
	}

	/* print some debug message */
	tmp = desc->bEndpointAddress;
	pr_devel ("enable %s(%d) ep%x%s-blk max %02x\n",
			_ep->name,ep->ep_num, tmp, desc->bEndpointAddress & USB_DIR_IN ? "in" : "out", max);

	spin_unlock_irqrestore (&udc->lock, flags);
	return 0;
}

/*
 * nuc980_ep_disable
 */
static int nuc980_ep_disable (struct usb_ep *_ep)
{
	struct nuc980_ep *ep = container_of(_ep, struct nuc980_ep, ep);
	unsigned long flags;

	if (!_ep || !ep->ep.desc)
		return -EINVAL;

	spin_lock_irqsave(&ep->dev->lock, flags);
	ep->ep.desc = 0;

	__raw_writel(0, ep->dev->base + REG_USBD_EPA_EPCFG+0x28*(ep->index-1));
	__raw_writel(0, ep->dev->base + REG_USBD_EPA_EPINTEN + 0x28*(ep->index-1));

	nuke (ep->dev, ep, -ESHUTDOWN);

	__raw_writel(0, ep->dev->base + REG_USBD_EPA_EPBUFSTART+0x28*(ep->index-1));
	__raw_writel(0, ep->dev->base + REG_USBD_EPA_EPBUFEND+0x28*(ep->index-1));

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	pr_devel("%s disabled\n", _ep->name);
	return 0;
}

/*
 * nuc980_alloc_request
 */
static struct usb_request *nuc980_alloc_request (struct usb_ep *_ep, gfp_t mem_flags)
{
	struct nuc980_ep *ep;
	struct nuc980_request *req;

	ep = container_of (_ep, struct nuc980_ep, ep);
	if (!_ep)
		return 0;

	req = kmalloc (sizeof *req, mem_flags);
	if (!req)
		return 0;
	memset (req, 0, sizeof *req);
	INIT_LIST_HEAD (&req->queue);

	return &req->req;
}

/*
 * nuc980_free_request
 */
static void nuc980_free_request (struct usb_ep *_ep, struct usb_request *_req)
{
	struct nuc980_ep *ep;
	struct nuc980_request *req;

	ep = container_of (_ep, struct nuc980_ep, ep);
	if (!ep || !_req || (!ep->ep.desc && _ep->name != ep0name))
		return;

	req = container_of (_req, struct nuc980_request, req);
	list_del_init(&req->queue);

	WARN_ON (!list_empty (&req->queue));
	kfree (req);
}


/*
 *  nuc980_queue
 */
static int nuc980_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct nuc980_request *req;
	struct nuc980_ep *ep;
	struct nuc980_udc *udc;
	unsigned long flags;

	ep = container_of(_ep, struct nuc980_ep, ep);
	if (unlikely (!_ep || (!ep->ep.desc && ep->ep.name != ep0name)))
	{
		pr_err("nuc980_queue: invalid args\n");
		local_irq_restore(flags);
		return -EINVAL;
	}

	udc = ep->dev;
	if (unlikely (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN))
	{
		pr_err("nuc980_queue: speed =%d\n",udc->gadget.speed);
		return -ESHUTDOWN;
	}

	local_irq_save(flags);

	req = container_of(_req, struct nuc980_request, req);

	if (unlikely (!_req || !_req->complete || !_req->buf || !list_empty(&req->queue)))
	{
		local_irq_restore(flags);
		return -EINVAL;
	}

	/* iso is always one packet per request, that's the only way
	 * we can report per-packet status.  that also helps with dma.
	 */
	if (ep->ep.desc)
	{ //clyu
		if (unlikely (ep->ep.desc->bmAttributes == USB_ENDPOINT_XFER_ISOC
						&& req->req.length > usb_endpoint_maxp(ep->ep.desc)))
		{
			local_irq_restore(flags);
			return -EMSGSIZE;
		}
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* for ep0 IN without premature status, zlp is required and
	 * writing EOP starts the status stage (OUT).
	 */
	if (unlikely(ep->ep_num == 0 && ep->ep_dir))
		_req->zero = 1;

	/* pio or dma irq handler advances the queue. */
	if (likely (req != 0))
		list_add_tail(&req->queue, &ep->queue);

	if (ep->index==0)
	{ //delayed status
        if ((req->req.length != 0) && (udc->ep0state == EP0_END_XFER))
        {
            udc->ep0state = EP0_IN_DATA_PHASE;
            __raw_writel(0x0a, udc->base + REG_USBD_CEPINTEN);
		}
		if ((udc->setup_ret > 1000) || ((req->req.length==0) && (udc->ep0state == EP0_OUT_DATA_PHASE)))
		{
			__raw_writel(USB_CEPCTL_NAKCLR, udc->base + REG_USBD_CEPCTL);
			__raw_writel(0x402, udc->base + REG_USBD_CEPINTEN);
			done(ep, req, 0);
			__raw_writel(USB_CEPCTL_FLUSH, udc->base + REG_USBD_CEPCTL);
		}
	}
	else if (ep->index > 0)
	{
		__raw_writel(ep->irq_enb, udc->base + REG_USBD_EPA_EPINTEN + 0x28*(ep->index-1));
	}

	local_irq_restore(flags);
	return 0;
}

/*
 *  nuc980_dequeue
 */
static int nuc980_dequeue (struct usb_ep *_ep, struct usb_request *_req)
{
	struct nuc980_ep *ep;
	struct nuc980_udc *udc;
	int retval = -EINVAL;
	unsigned long flags;
	struct nuc980_request *req;

	if (!_ep || !_req)
		return retval;
	ep = container_of (_ep, struct nuc980_ep, ep);
	udc = ep->dev;

	if (!udc->driver)
		return -ESHUTDOWN;

	spin_lock_irqsave (&udc->lock, flags);
	list_for_each_entry(req, &ep->queue, queue)
	{
		if (&req->req == _req)
		{
			list_del_init (&req->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}
	spin_unlock_irqrestore (&udc->lock, flags);
	pr_devel("dequeue: %d, req %p\n", retval,  &req->req);
	if (retval == 0)
	{
		pr_devel( "dequeued req %p from %s, len %d buf %p\n", req, _ep->name, _req->length, _req->buf);
		_req->complete (_ep, _req);
		done(ep, req, -ECONNRESET);
	}

	return retval;
}


/*
 * nuc980_set_halt
 */
static int nuc980_set_halt (struct usb_ep *_ep, int value)
{
	struct nuc980_ep *ep = container_of(_ep, struct nuc980_ep, ep);
	struct nuc980_udc *udc = ep->dev;
	unsigned long flags;

	if (!_ep || (ep->ep_type == USB_EP_CFG_TYPE_ISO))
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	/* Halting an IN endpoint should fail if queue is not empty */
	if (value && ep->ep_dir && !list_empty(&ep->queue)) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EAGAIN;
	}

	if (value == 1)
	{
		/* stall */
		if (ep->ep_num == 0)
		{
			__raw_writel(USB_CEPCTL_STALL, udc->base + REG_USBD_CEPCTL);
			udc->ep0state = EP0_STALL;
		}
		else
		{
			__raw_writel((__raw_readl(udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1)) & 0xf7) | USB_EP_RSPCTL_HALT,
						udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));
		}
	}
	else
	{
		/* End stall */
		if (ep->ep_num != 0)
		{
			__raw_writel(USB_EP_RSPCTL_TOGGLE, udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));
		}
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

/*
 * Return the total number of bytes currently stored in the internal buffers of
 * the endpoint.
 */
static int nuc980_fifo_status(struct usb_ep *_ep)
{
	struct nuc980_ep *ep;
	struct nuc980_udc *udc;
	u32 bytes = 0;

	if (!_ep)
		return -ENODEV;

	ep = container_of(_ep, struct nuc980_ep, ep);
	udc = ep->dev;

	if (ep->ep_num == 0)
	{
		bytes = __raw_readl(udc->base + REG_USBD_CEPDATCNT) & 0xffff;
	}
	else
	{
		bytes = __raw_readl(udc->base + REG_USBD_EPA_EPDATCNT+0x28*(ep->index-1)) & 0xffff;
	}

	return bytes;
}


/* Empty data from internal buffers of an endpoint. */
static void nuc980_fifo_flush(struct usb_ep *_ep)
{
	struct nuc980_ep *ep;
	struct nuc980_udc *udc;
	unsigned long flags;

	if (!_ep)
		return;

	ep = container_of(_ep, struct nuc980_ep, ep);
	udc = ep->dev;
	pr_devel("EP: flush fifo %s\n", ep->ep.name);

	spin_lock_irqsave(&udc->lock, flags);

	if (ep->ep_num == 0)
	{
		__raw_writel(USB_CEPCTL_FLUSH, udc->base + REG_USBD_CEPCTL);
	}
	else
	{
		__raw_writel(USB_EP_RSPCTL_FLUSH, udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));
	}

	spin_unlock_irqrestore(&udc->lock, flags);
}



static const struct usb_ep_ops nuc980_ep_ops =
{
	.enable         = nuc980_ep_enable,
	.disable        = nuc980_ep_disable,

	.alloc_request  = nuc980_alloc_request,
	.free_request   = nuc980_free_request,

	.queue          = nuc980_queue,
	.dequeue        = nuc980_dequeue,

	.set_halt       = nuc980_set_halt,
	.fifo_status	= nuc980_fifo_status,
	.fifo_flush		= nuc980_fifo_flush,
};

/*
 *  nuc980_g_get_frame
 */
static int nuc980_g_get_frame (struct usb_gadget *_gadget)
{
	struct nuc980_udc *udc = to_nuc980_udc(_gadget);
	int tmp;
	tmp = __raw_readl(udc->base + REG_USBD_FRAMECNT);
	return tmp & 0xffff;
}

/*
 *  nuc980_wakeup
 */
static int nuc980_wakeup (struct usb_gadget *_gadget)
{
	return 0;
}

/*
 *  nuc980_set_selfpowered
 */
static int nuc980_set_selfpowered (struct usb_gadget *_gadget, int value)
{
	return 0;
}

static int nuc980_pullup (struct usb_gadget *g, int is_on)
{
	struct nuc980_udc *udc = to_nuc980_udc(g);
//printk("pullup\n");

	if (is_on)
		nuc980_udc_enable(udc);
	else {
		if (udc->gadget.speed != USB_SPEED_UNKNOWN) {
			if (udc->driver && udc->driver->disconnect)
				udc->driver->disconnect(&udc->gadget);
		}
		nuc980_udc_disable(udc);
	}
	return 0;
}

static int nuc980_udc_start(struct usb_gadget *g, struct usb_gadget_driver *driver);
static int nuc980_udc_stop(struct usb_gadget *g, struct usb_gadget_driver *driver);

static const struct usb_gadget_ops nuc980_ops =
{
	.get_frame          = nuc980_g_get_frame,
	.wakeup             = nuc980_wakeup,
	.set_selfpowered    = nuc980_set_selfpowered,
	.pullup             = nuc980_pullup,
	.udc_start          = nuc980_udc_start,
	.udc_stop           = nuc980_udc_stop,
};


static void nuc980_udc_enable(struct nuc980_udc *udc)
{
//    udc->gadget.speed = USB_SPEED_HIGH;
//printk("enable: 0x%x\n", __raw_readl(udc->base + REG_USBD_PHYCTL));
	__raw_writel(__raw_readl(udc->base + REG_USBD_PHYCTL) | 0x100, udc->base + REG_USBD_PHYCTL);
}

static void nuc980_udc_disable(struct nuc980_udc *udc)
{
	unsigned int i;
//printk("disable: 0x%x\n", __raw_readl(udc->base + REG_USBD_PHYCTL));
	__raw_writel(0, udc->base + REG_USBD_CEPINTEN);
	__raw_writel(0xffff, udc->base + REG_USBD_CEPINTSTS);
	__raw_writel(__raw_readl(udc->base + REG_USBD_CEPCTL)|USB_CEPCTL_FLUSH, udc->base + REG_USBD_CEPCTL);
	for (i = 1; i < NUC980_ENDPOINTS; i++)
		__raw_writel(USB_EP_RSPCTL_FLUSH|USB_EP_RSPCTL_TOGGLE, udc->base + REG_USBD_EPA_EPRSPCTL + 0x28*(i-1));

	__raw_writel(__raw_readl(udc->base + REG_USBD_PHYCTL) & ~0x100, udc->base + REG_USBD_PHYCTL);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
}

/*
 *  nuc980_udc_start
 */
static int nuc980_udc_start(struct usb_gadget *g, struct usb_gadget_driver *driver)
{
	struct nuc980_udc *udc = to_nuc980_udc(g);

	pr_devel("nuc980_udc_start() '%s'\n", driver->driver.name);

	udc->gadget.name = gadget_name;
	udc->gadget.ops = &nuc980_ops;
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->driver = driver;

	udc->usb_address = 0;
	/*
	 * configure USB controller
	 */
	__raw_writel(0x03, udc->base + REG_USBD_GINTEN);    /* enable usb, cep interrupt */
	__raw_writel((USBD_BUSINTEN_RESUMEIEN | USBD_BUSINTEN_RSTIEN | USBD_BUSINTEN_VBUSDETIEN), udc->base + REG_USBD_BUSINTEN);
	__raw_writel(0, udc->base + REG_USBD_FADDR);
	__raw_writel(0x402, udc->base + REG_USBD_CEPINTEN);


	nuc980_udc_enable(udc);
	return 0;
}

/*
 *  nuc980_udc_stop
 */
static int nuc980_udc_stop(struct usb_gadget *g, struct usb_gadget_driver *driver)
{
	struct nuc980_udc *udc = to_nuc980_udc(g);
	unsigned int volatile i;

	udc->driver = 0;

	pr_devel("device_release_driver\n");

	/* clear/disable all interrupts */
	__raw_writel(0, udc->base + REG_USBD_BUSINTEN);
	__raw_writel(0xffff, udc->base + REG_USBD_BUSINTSTS);

	__raw_writel(0, udc->base + REG_USBD_CEPINTEN);
	__raw_writel(0xffff, udc->base + REG_USBD_CEPINTSTS);

	for (i = 0; i < NUC980_ENDPOINTS-1; i++)
	{ //6 endpoints
		__raw_writel(0, udc->base + REG_USBD_EPA_EPINTEN + 0x28 * i);
		__raw_writel(0xffff, udc->base + REG_USBD_EPA_EPINTSTS + 0x28 * i);
	}

	nuc980_udc_disable(udc);
	return 0;
}

static void udc_isr_reset(struct nuc980_udc *udc)
{
	int i;

	nuke(udc, &udc->ep[0], -ECONNRESET);

	udc->usb_address = 0;
	udc->usb_less_mps = 0;

	//reset DMA
	__raw_writel(0x80, udc->base + REG_USBD_DMACTL);
	__raw_writel(0x00, udc->base + REG_USBD_DMACTL);

	pr_devel("speed:%x\n", __raw_readl(udc->base + REG_USBD_OPER));
	if (__raw_readl(udc->base + REG_USBD_OPER) & 0x04)
		udc->gadget.speed = USB_SPEED_HIGH;
	else
		udc->gadget.speed = USB_SPEED_FULL;

	__raw_writel(__raw_readl(udc->base + REG_USBD_CEPCTL)|USB_CEPCTL_FLUSH, udc->base + REG_USBD_CEPCTL);
	for (i = 1; i < NUC980_ENDPOINTS; i++)
		__raw_writel(USB_EP_RSPCTL_FLUSH|USB_EP_RSPCTL_TOGGLE, udc->base + REG_USBD_EPA_EPRSPCTL + 0x28*(i-1));

	__raw_writel(0, udc->base + REG_USBD_FADDR);
	__raw_writel(USBD_CEPINTEN_SETUPPKIEN, udc->base + REG_USBD_CEPINTEN);
}

static void udc_isr_dma(struct nuc980_udc *udc)
{
	struct nuc980_request *req;
	struct nuc980_ep *ep;
	u32 datacnt_reg;

	if (!udc->usb_dma_trigger)
	{
		pr_devel("DMA not trigger, intr?\n");
		return;
	}

	ep = &udc->ep[udc->usb_dma_owner];
	datacnt_reg = (u32)(REG_USBD_EPA_EPDATCNT+0x28*(ep->index-1));

	if (udc->usb_dma_dir == Ep_In)
		__raw_writel(USBD_EPINTSTS_INTKIF, udc->base + REG_USBD_EPA_EPINTSTS + 0x28*(ep->index-1));

	udc->usb_dma_trigger = 0;
	if (list_empty(&ep->queue)) {
		pr_devel("DMA ep->queue is empty\n");
		req = 0;
		__raw_writel(udc->irq_enbl, udc->base + REG_USBD_GINTEN);
		return;
	}
	else {
		req = list_entry(ep->queue.next, struct nuc980_request, queue);
	}

	if (req) {
		if (ep->ep_type == USB_EP_CFG_TYPE_BULK) {
			if (udc->usb_less_mps == 1) {
				__raw_writel((__raw_readl(udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1)) & 0x16)|USB_EP_RSPCTL_SHORTTXEN, udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));
				udc->usb_less_mps = 0;
			}
		}
		else if (ep->ep_type == USB_EP_CFG_TYPE_INT) {
			__raw_writel(udc->usb_dma_cnt, udc->base + REG_USBD_EPA_EPTXCNT+0x28*(ep->index-1));
		}
		else if (ep->ep_type == USB_EP_CFG_TYPE_ISO) {
			if (udc->usb_less_mps == 1) {
				__raw_writel((__raw_readl(udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1)) & 0x16)|USB_EP_RSPCTL_SHORTTXEN, udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));
				udc->usb_less_mps = 0;
			}
		}
		req->req.actual += udc->usb_dma_cnt;
		if ((req->req.length == req->req.actual) || udc->usb_dma_cnt < ep->ep.maxpacket) {
			__raw_writel(udc->irq_enbl, udc->base + REG_USBD_GINTEN);
			if ((ep->ep_type == USB_EP_CFG_TYPE_BULK) && (ep->ep_dir == 0) && (udc->usb_dma_cnt < ep->ep.maxpacket)) {
				if (ep->buffer_disabled) {
					__raw_writel((__raw_readl(udc->base + REG_USBD_EPA_EPRSPCTL + 0x28*(ep->index-1))) & 0x16,
								 udc->base + REG_USBD_EPA_EPRSPCTL + 0x28*(ep->index-1));//enable buffer
					__raw_writel((__raw_readl(udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1)) & 0x16) | 0x80,
								 udc->base + REG_USBD_EPA_EPRSPCTL+0x28*(ep->index-1));//disable buffer when short packet
				}
			}
			done(ep, req, 0);

			return;
		}
	}

	if (udc->usb_dma_dir == Ep_Out) {
		if (udc->usb_dma_trigger_next) {
			udc->usb_dma_trigger_next = 0;
			pr_devel("dma out\n");
			read_fifo(ep, req, 0);
		}
	}
	else if (udc->usb_dma_dir == Ep_In) {
		if (udc->usb_less_mps == 1)
			udc->usb_less_mps = 0;
		if (udc->usb_dma_trigger_next) {
			udc->usb_dma_trigger_next = 0;
			pr_devel("dma in\n");
			write_fifo(ep, req);
		}
	}
}


static void udc_isr_ctrl_pkt(struct nuc980_udc *udc)
{
	struct nuc980_ep *ep = &udc->ep[0];
	struct usb_ctrlrequest crq;
	struct nuc980_request *req;
	int ret;

	if (list_empty(&ep->queue)) {
		pr_devel("ctrl ep->queue is empty\n");
		req = 0;
	}
	else {
		req = list_entry(ep->queue.next, struct nuc980_request, queue);
	}

	crq.bRequestType = (u8)__raw_readl(udc->base + REG_USBD_SETUP1_0) & 0xff;
	crq.bRequest = (u8)(__raw_readl(udc->base + REG_USBD_SETUP1_0) >> 8) & 0xff;
	crq.wValue = (u16)__raw_readl(udc->base + REG_USBD_SETUP3_2);
	crq.wIndex = (u16)__raw_readl(udc->base + REG_USBD_SETUP5_4);
	crq.wLength = (u16)__raw_readl(udc->base + REG_USBD_SETUP7_6);
	udc->crq = crq;

	switch (udc->ep0state) {
		case EP0_IDLE:
			if (crq.bRequest == USB_REQ_SET_ADDRESS) {
				udc->usb_address = crq.wValue;
			}

			if (crq.bRequestType & USB_DIR_IN) {
				udc->ep0state = EP0_IN_DATA_PHASE;
				__raw_writel(USBD_CEPINTEN_SETUPPKIEN|USBD_CEPINTEN_INTKIEN, udc->base + REG_USBD_CEPINTEN);
			}
			else {
				udc->ep0state = EP0_OUT_DATA_PHASE;
				__raw_writel(USBD_CEPINTEN_SETUPPKIEN|USBD_CEPINTEN_RXPKIEN, udc->base + REG_USBD_CEPINTEN);
			}

			if (udc->gadget.speed == USB_SPEED_FULL)
				udelay(5);

            ret = udc->driver->setup(&udc->gadget, &crq);
            udc->setup_ret = ret;
            if ((ret < 0) || (crq.bRequest == USB_REQ_SET_ADDRESS)) {
				__raw_writel(USBD_CEPINTSTS_STSDONEIF, udc->base + REG_USBD_CEPINTSTS);
				__raw_writel(USB_CEPCTL_NAKCLR, udc->base + REG_USBD_CEPCTL);
				__raw_writel(USBD_CEPINTEN_SETUPPKIEN|USBD_CEPINTEN_STSDONEIEN, udc->base + REG_USBD_CEPINTEN);
			}
			else if (ret > 1000) {
				pr_devel("DELAYED_STATUS:%p\n", req);
				udc->ep0state = EP0_END_XFER;
				__raw_writel(USBD_CEPINTEN_SETUPPKIEN, udc->base + REG_USBD_CEPINTEN);
			}
			break;

		case EP0_STALL:
			break;

		default:
			;
	}
}

void udc_isr_update_dev(struct nuc980_udc *udc)
{
	struct usb_ctrlrequest *pcrq = &udc->crq;

	//update this device for set requests
	switch (pcrq->bRequest)
	{
		case USB_REQ_SET_ADDRESS:
			__raw_writel(udc->usb_address, udc->base + REG_USBD_FADDR);
			break;

		case USB_REQ_SET_CONFIGURATION:
			break;

		case USB_REQ_SET_INTERFACE:
			break;

		case USB_REQ_SET_FEATURE:
			if ((pcrq->bRequestType & 0x3) == 0x0) 	/* Receipent is Device */
			{
				if ((pcrq->wValue & 0x3) == 0x2)
				{
					 __raw_writel(pcrq->wIndex >> 8, udc->base + REG_USBD_TEST);
				}
			}
			break;

		case USB_REQ_CLEAR_FEATURE:
			break;

		default:
			;
	}//switch end
	return;
}


static void USB_Init(struct nuc980_udc *udc)
{
	int i, j;

	udc->usb_address = 0;
	/*
	 * configure USB controller
	 */
	__raw_writel(0x03, udc->base + REG_USBD_GINTEN);    /* enable usb, cep interrupt */
	__raw_writel((USBD_BUSINTEN_RESUMEIEN | USBD_BUSINTEN_RSTIEN | USBD_BUSINTEN_VBUSDETIEN), udc->base + REG_USBD_BUSINTEN);

	__raw_writel(0, udc->base + REG_USBD_FADDR);
	__raw_writel((USBD_CEPINTEN_SETUPPKIEN | USBD_CEPINTEN_STSDONEIEN), udc->base + REG_USBD_CEPINTEN);

	for (j = 0; j < NUC980_ENDPOINTS; j++)
	{
		udc->ep[j].ep_num = 0xff;
		udc->ep[j].ep_dir = 0xff;
		udc->ep[j].ep_type = 0xff;
	}

	/* setup endpoint information */
	INIT_LIST_HEAD (&udc->gadget.ep_list);
	for (i = 0; i < NUC980_ENDPOINTS; i++)
	{
		struct nuc980_ep *ep = &udc->ep[i];

		if (!ep_info[i].name)
			break;
		ep->index = i;
		ep->ep.name = ep_info[i].name;
		ep->ep.caps = ep_info[i].caps;
		ep->ep.ops = &nuc980_ep_ops;
		list_add_tail (&ep->ep.ep_list, &udc->gadget.ep_list);

		/* maxpacket differs between ep0 and others ep */
		if (!i)
		{
			ep->ep_num = 0;
			ep->ep.maxpacket = EP0_FIFO_SIZE;
			usb_ep_set_maxpacket_limit(&ep->ep, EP0_FIFO_SIZE);
			__raw_writel(0x00000000, udc->base + REG_USBD_CEPBUFSTART);
			__raw_writel(0x0000003f, udc->base + REG_USBD_CEPBUFEND);
		}
		else
		{
			ep->ep.maxpacket = EP_FIFO_SIZE;
			usb_ep_set_maxpacket_limit(&ep->ep, EP_FIFO_SIZE);
			__raw_writel(0, udc->base + REG_USBD_EPA_EPBUFSTART + 0x28*(ep->index-1));
			__raw_writel(0, udc->base + REG_USBD_EPA_EPBUFEND + 0x28*(ep->index-1));
		}
		ep->dev = udc;
		ep->ep.desc = 0;
		INIT_LIST_HEAD (&ep->queue);
	}
	udc->gadget.ep0 = &udc->ep[0].ep;
	list_del_init (&udc->ep[0].ep.ep_list);
}

/*
 *  probe - binds to the platform device
 */
static int nuc980_udc_probe(struct platform_device *pdev)
{
	struct nuc980_udc *udc;
	struct device *dev = &pdev->dev;
	struct pinctrl *p;
	int error;

	pr_devel("nuc980_udc_probe...\n");
	dev_dbg(dev, "%s()\n", __func__);

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

#ifdef CONFIG_OF

        p = devm_pinctrl_get_select_default(&pdev->dev);
        if (IS_ERR(p)) {
            return PTR_ERR(p);
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
	p = devm_pinctrl_get_select(&pdev->dev, "usbd-vbusvld");
	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		error = PTR_ERR(p);
	}
#endif

	udc->pdev = pdev;

	udc->clk = clk_get(NULL, "usbd_hclk");
	if (IS_ERR(udc->clk))
	{
		error = -ENODEV;
		dev_dbg(&pdev->dev, "no udc_clk?\n");
		goto fail1;
	}

	clk_prepare(udc->clk);
	clk_enable(udc->clk);       /* Enable the peripheral clock */

	udc->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (udc->res == NULL)
	{
		dev_err(dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto fail1;
	}

	if (!request_mem_region(udc->res->start, resource_size(udc->res), pdev->name))
	{
		dev_err(dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto fail1;
	}

	udc->base = ioremap(udc->res->start, resource_size(udc->res));
	if (udc->base == NULL)
	{
		dev_err(dev, "failed to remap I/O memory\n");
		error = -ENXIO;
		goto fail1;
	}

	udc->gadget.dev.parent = dev;
	platform_set_drvdata (pdev, udc);

	spin_lock_init (&udc->lock);

	__raw_writel(__raw_readl(udc->base + REG_USBD_PHYCTL) | 0x200, udc->base + REG_USBD_PHYCTL);
	// FIXME: is it possible to loop forever?
	while (1)
	{
		__raw_writel(0x20, udc->base + REG_USBD_EPA_EPMPS);
		if (__raw_readl(udc->base + REG_USBD_EPA_EPMPS) == 0x20)
			break;
	}

	/* initial gadget structure */
	udc->gadget.ops = &nuc980_ops;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->ep0state = EP0_IDLE;
	udc->gadget.name = dev_name(dev);

	USB_Init(udc);

	udc->irq = platform_get_irq(pdev, 0);
	if (udc->irq < 0)
	{
		dev_err(dev, "Failed to get irq\n");
		error = -ENXIO;
		goto fail2;
	}
	error = request_irq(udc->irq, nuc980_udc_irq, 0, gadget_name, udc);
	if (error != 0)
	{
		dev_err(dev, "request_irq() failed\n");
		goto fail2;
	}

	usb_vaddr = (u32)dma_alloc_writecombine(NULL, 512, (u32 *)&usb_paddr, GFP_KERNEL);
	error = usb_add_gadget_udc(dev, &udc->gadget);
	if (error)
		goto fail3;

	pr_devel("nuc980_udc_probe done.\n");
	return 0;
fail3:
	free_irq(udc->irq, udc);
fail2:
	iounmap(udc->base);
fail1:
	return error;
}

/*
 *  nuc980_udc_remove
 */
static int nuc980_udc_remove(struct platform_device *pdev)
{
	struct nuc980_udc *udc = platform_get_drvdata (pdev);

	dev_dbg(&pdev->dev, "%s()\n", __func__);

	usb_del_gadget_udc(&udc->gadget);
	free_irq(udc->irq, udc);
	iounmap(udc->base);

	__raw_writel(__raw_readl(udc->base + REG_USBD_PHYCTL) & ~0x200,
				 udc->base + REG_USBD_PHYCTL);    // phy suspend
	clk_disable(udc->clk);

	return 0;
}

#ifdef CONFIG_PM
static int nuc980_udc_suspend (struct platform_device *pdev, pm_message_t state)
{
	__raw_writel(__raw_readl(udc->base + REG_USBD_PHYCTL) & ~0x200, udc->base + REG_USBD_PHYCTL);    // phy suspend
	return 0;
}

static int nuc980_udc_resume (struct platform_device *pdev)
{
	unsigned int reg = __raw_readl(udc->base + REG_USBD_EPA_EPMPS);

	__raw_writel(__raw_readl(udc->base + REG_USBD_PHYCTL) | 0x200, udc->base + REG_USBD_PHYCTL);
	while (1)
	{
		__raw_writel(0x20, udc->base + REG_USBD_EPA_EPMPS);
		if (__raw_readl(udc->base + REG_USBD_EPA_EPMPS) == 0x20)
		{
			__raw_writel(reg, udc->base + REG_USBD_EPA_EPMPS);
			break;
		}
	}

	return 0;
}
#else
#define nuc980_udc_suspend     NULL
#define nuc980_udc_resume      NULL
#endif

static const struct of_device_id nuc980_usbd_of_match[] = {
	{ .compatible = "nuvoton,nuc980-usbdev" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_usbd_of_match);

static struct platform_driver nuc980_udc_driver =
{
	.probe      = nuc980_udc_probe,
	.remove     = nuc980_udc_remove,
	.suspend    = nuc980_udc_suspend,
	.resume     = nuc980_udc_resume,
	.driver     = {
			.owner  = THIS_MODULE,
			.name   = (char *) "nuc980-usbdev",
		        .of_match_table = of_match_ptr(nuc980_usbd_of_match),
	},
};

//insmod g_mass_storage.ko file=/dev/mmcblk0p1 stall=0 removable=1

module_platform_driver(nuc980_udc_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
