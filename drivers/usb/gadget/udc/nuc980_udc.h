/* linux/include/asm-arm/arch-nuc980/nuc980_reg.h
 *
 * Copyright (c) 2018 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2018/03/16     add this file for nuvoton nuc980 MCU ip REG.
 */
#ifndef _NUC980_UDC_H
#define _NUC980_UDC_H

#define NUC980_ENDPOINTS    13

struct nuc980_ep {
    struct list_head    queue;
    struct nuc980_udc   *dev;
    struct usb_ep       ep;
    u8                  index;
    u8                  buffer_disabled;
    u8                  bEndpointAddress;//w/ direction

    u8                  ep_mode;//auto/manual/fly
    u8                  ep_num;//no direction ep address
    u8                  ep_dir;//0 OUT, 1 IN
    u8                  ep_type;//bulk/in/iso
    u32 irq_enb;
};


struct nuc980_request {
    struct list_head      queue;      /* ep's requests */
    struct usb_request    req;
    u32                   dma_mapped;
};

enum ep0_state {
    EP0_IDLE,
    EP0_IN_DATA_PHASE,
    EP0_OUT_DATA_PHASE,
    EP0_END_XFER,
    EP0_STALL,
};


struct nuc980_udc {
    spinlock_t                  lock;
    struct nuc980_ep            ep[NUC980_ENDPOINTS];
    struct usb_gadget           gadget;
    struct usb_gadget_driver    *driver;
    struct platform_device      *pdev;

    struct clk                  *clk;
    struct resource             *res;
    void __iomem                *base;
    int                         irq;

    enum ep0_state              ep0state;

    u8                          usb_devstate;
    u8                          usb_address;

    u8                          usb_dma_dir;
    u8                          usb_dma_trigger;//bool. dma triggered
    u8                          usb_dma_trigger_next;//need trigger again
    u8                          usb_less_mps;
    u32                         usb_dma_cnt;//one dma transfer count
    u32                         usb_dma_loop;//for short packet only;dma loop, each loop 32byte;
    u32                         usb_dma_owner;

    struct usb_ctrlrequest      crq;
    s32                         setup_ret;

    u32                         irq_enbl;
};

#define to_nuc980_udc(g)        (container_of((g), struct nuc980_udc, gadget))

#endif
