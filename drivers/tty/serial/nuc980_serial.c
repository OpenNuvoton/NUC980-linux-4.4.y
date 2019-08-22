/*
 *  linux/drivers/tty/serial/nuc980_serial.c
 *
 *  NUC980 serial driver
 *
 *
 *  Copyright (C) 2018 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#if defined(CONFIG_SERIAL_NUC980_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/serial.h>

#include <mach/map.h>
#include <mach/regs-serial.h>
#include <mach/regs-gcr.h>
#include <mach/mfp.h>
#include <mach/regs-pdma.h>
#include <mach/sram.h>
#include <linux/platform_data/dma-nuc980.h>


#include "nuc980_serial.h"

//#define USING_SRAM
#define UART_NR 10
#define UART_RX_BUF_SIZE 128 //bytes
#define UART_TX_MAX_BUF_SIZE 128 //bytes

//#define CONFIG_USE_DDR 1

static struct uart_driver nuc980serial_reg;

struct clk      *clk;

struct uart_nuc980_port {
	struct uart_port    port;

	unsigned short      capabilities;   /* port capabilities */
	unsigned char       ier;
	unsigned char       lcr;
	unsigned char       mcr;
	unsigned char       mcr_mask;  /* mask of user bits */
	unsigned char       mcr_force; /* mask of forced bits */

	struct serial_rs485 rs485; /* rs485 settings */

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	struct nuc980_ip_rx_dma dma_rx;
	struct nuc980_ip_tx_dma dma_tx;
	struct nuc980_mem_alloc src_mem_p;
	struct nuc980_mem_alloc dest_mem_p;
	struct nuc980_dma_done   dma_slave_done;

	unsigned char PDMA_UARTx_TX;
	unsigned char PDMA_UARTx_RX;

	struct nuc980_dma_done   dma_Rx_done;
	struct nuc980_dma_done   dma_Tx_done;

	unsigned int tx_dma_len;

	unsigned char uart_pdma_enable_flag;
	unsigned char Tx_pdma_busy_flag;
#endif

	/*
	* We provide a per-port pm hook.
	*/
	void    (*pm)(struct uart_port *port, unsigned int state, unsigned int old);
};

static struct uart_nuc980_port nuc980serial_ports[UART_NR];


static inline void __stop_tx(struct uart_nuc980_port *p);

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
static void nuc980_prepare_RX_dma(struct uart_nuc980_port *p);
static void nuc980_prepare_TX_dma(struct uart_nuc980_port *p);
#endif

static inline struct uart_nuc980_port *
to_nuc980_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct uart_nuc980_port, port);
}

static inline unsigned int serial_in(struct uart_nuc980_port *p, int offset)
{
	return(__raw_readl(p->port.membase + offset));
}

static inline void serial_out(struct uart_nuc980_port *p, int offset, int value)
{
	__raw_writel(value, p->port.membase + offset);
}

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)

static void nuc980_Rx_dma_callback(void *arg)
{
	struct nuc980_dma_done *done = arg;
	struct uart_nuc980_port *p = (struct uart_nuc980_port *)done->callback_param;
	struct tty_port		*tty_port = &p->port.state->port;
	int count;
	int copied_count = 0;

	if(done->timeout==1)
		count = ((p->dest_mem_p.size/2) -(done->remain +1));
	else
		count = (p->dest_mem_p.size/2);

	spin_lock(&p->port.lock);

	if(done->base_addr==1)
		copied_count = tty_insert_flip_string(tty_port, ((unsigned char *)p->dest_mem_p.vir_addr), count);
	else
		copied_count = tty_insert_flip_string(tty_port, ((unsigned char *)p->dest_mem_p.vir_addr+(p->dest_mem_p.size/2)), count);

	if(copied_count != count) {
		printk("\n Rx overrun: dropping %zu bytes \n", (count - copied_count));
	}

	p->port.icount.rx +=copied_count;

	tty_flip_buffer_push(tty_port);

	spin_unlock(&p->port.lock);

	if(done->timeout==1) {
		nuc980_prepare_RX_dma(p);
		//Trigger Rx dma again
		serial_out(p, UART_REG_IER, (serial_in(p, UART_REG_IER)|RXPDMAEN));
	}

	return;
}

static void nuc980_Tx_dma_callback(void *arg)
{
	struct nuc980_dma_done *done = arg;
	struct uart_nuc980_port *p = (struct uart_nuc980_port *)done->callback_param;
	struct circ_buf *xmit = &p->port.state->xmit;

	spin_lock(&p->port.lock);

	p->port.icount.tx += p->tx_dma_len;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&p->port);

	if (!uart_circ_empty(xmit) && !uart_tx_stopped(&p->port)) {
		p->Tx_pdma_busy_flag = 1;
		nuc980_prepare_TX_dma(p);
		// Trigger Tx dma again
		serial_out(p, UART_REG_IER, (serial_in(p, UART_REG_IER)| TXPDMAEN));
	} else {
		p->Tx_pdma_busy_flag = 0;
	}

	spin_unlock(&p->port.lock);
}

static void set_pdma_flag(struct uart_nuc980_port *p, int id)
{
#if defined(CONFIG_ENABLE_UART1_PDMA) || defined(CONFIG_USE_OF)
	if(id == 1) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART1_RX;
		p->PDMA_UARTx_TX = PDMA_UART1_TX;
	}
#endif

#if defined(CONFIG_ENABLE_UART2_PDMA) || defined(CONFIG_USE_OF)
	if(id == 2) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART2_RX;
		p->PDMA_UARTx_TX = PDMA_UART2_TX;
	}
#endif

#if defined(CONFIG_ENABLE_UART3_PDMA) || defined(CONFIG_USE_OF)
	if(id == 3) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART3_RX;
		p->PDMA_UARTx_TX = PDMA_UART3_TX;
	}
#endif

#if defined(CONFIG_ENABLE_UART4_PDMA) || defined(CONFIG_USE_OF)
	if(id == 4) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART4_RX;
		p->PDMA_UARTx_TX = PDMA_UART4_TX;
	}
#endif

#if defined(CONFIG_ENABLE_UART5_PDMA) || defined(CONFIG_USE_OF)
	if(id == 5) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART5_RX;
		p->PDMA_UARTx_TX = PDMA_UART5_TX;
	}
#endif

#if defined(CONFIG_ENABLE_UART6_PDMA) || defined(CONFIG_USE_OF)
	if(id == 6) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART6_RX;
		p->PDMA_UARTx_TX = PDMA_UART6_TX;
	}
#endif

#if defined(CONFIG_ENABLE_UART7_PDMA) || defined(CONFIG_USE_OF)
	if(id == 7) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART7_RX;
		p->PDMA_UARTx_TX = PDMA_UART7_TX;
	}
#endif

#if defined(CONFIG_ENABLE_UART8_PDMA) || defined(CONFIG_USE_OF)
	if(id == 8) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART8_RX;
		p->PDMA_UARTx_TX = PDMA_UART8_TX;
	}
#endif

#if defined(CONFIG_ENABLE_UART9_PDMA) || defined(CONFIG_USE_OF)
	if(id == 9) {
		p->uart_pdma_enable_flag = 1;
		p->PDMA_UARTx_RX = PDMA_UART9_RX;
		p->PDMA_UARTx_TX = PDMA_UART9_TX;
	}
#endif
}

static void nuc980_prepare_RX_dma(struct uart_nuc980_port *p)
{
	struct nuc980_dma_config dma_crx;
	struct nuc980_ip_rx_dma *pdma_rx = &(p->dma_rx);
	dma_cookie_t cookie;

	serial_out(p, UART_REG_IER, (serial_in(p, UART_REG_IER)&~ RXPDMAEN));

	if(p->dest_mem_p.size == 0) {
		// use DDR
#ifndef USING_SRAM
		//p->dest_mem_p.size = 256;
		p->dest_mem_p.size = 4096*2;
		p->dest_mem_p.vir_addr = (unsigned int)dma_alloc_writecombine(NULL,
		                         PAGE_ALIGN(p->dest_mem_p.size),
		                         &(p->dest_mem_p.phy_addr),
		                         GFP_KERNEL);
#else
		p->dest_mem_p.size = 1020; //set to 1Kbytes
		p->dest_mem_p.vir_addr =(u32)sram_alloc(p->dest_mem_p.size, &(p->dest_mem_p.phy_addr));
#endif
	}

	pdma_rx->slave_config.src_addr = (unsigned int)(p->port.membase - 0x40000000);
	pdma_rx->slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	pdma_rx->slave_config.src_maxburst = 1;
	pdma_rx->slave_config.direction = DMA_DEV_TO_MEM;
	pdma_rx->slave_config.device_fc = false;
	dmaengine_slave_config(pdma_rx->chan_rx,&(pdma_rx->slave_config));

	sg_init_table(pdma_rx->sgrx, 1);
	pdma_rx->sgrx[0].dma_address = p->dest_mem_p.phy_addr;
	pdma_rx->sgrx[0].length = p->dest_mem_p.size;
	dma_crx.reqsel = p->PDMA_UARTx_RX;
	dma_crx.timeout_counter = 1000;
	dma_crx.timeout_prescaler = 7;
	dma_crx.en_sc = 1;
	pdma_rx->rxdesc=pdma_rx->chan_rx->device->device_prep_slave_sg(pdma_rx->chan_rx,
	                pdma_rx->sgrx,
	                1,
	                DMA_FROM_DEVICE,
	                DMA_PREP_INTERRUPT | DMA_CTRL_ACK,
	                (void *)&dma_crx); //PDMA Request Source Select
	if (!pdma_rx->rxdesc) {
		printk("pdma->rxdesc=NULL\n");
		while(1);
	}
	//dma_slave_done.done = false;
	pdma_rx->rxdesc->callback = nuc980_Rx_dma_callback;
	p->dma_Rx_done.callback_param = p;
	p->dma_Rx_done.base_addr = 0;
	p->dma_Rx_done.timeout = 0;
	pdma_rx->rxdesc->callback_param = &(p->dma_Rx_done);
	cookie = pdma_rx->rxdesc->tx_submit(pdma_rx->rxdesc);
	if (dma_submit_error(cookie)) {
		printk("rx dma_submit_error  \n");
		while(1);
	}

}

static void nuc980_prepare_TX_dma(struct uart_nuc980_port *p)
{
	struct nuc980_dma_config dma_ctx;
	struct nuc980_ip_tx_dma *pdma_tx = &(p->dma_tx);
	dma_cookie_t cookie;
	struct circ_buf *xmit = &p->port.state->xmit;

	if(p->src_mem_p.size == 0) {
		p->src_mem_p.size = UART_XMIT_SIZE;
		p->src_mem_p.vir_addr = (unsigned int)dma_alloc_writecombine(NULL,
		                        PAGE_ALIGN(p->src_mem_p.size),
		                        &(p->src_mem_p.phy_addr),
		                        GFP_KERNEL);
	}

	p->tx_dma_len = uart_circ_chars_pending(xmit);
	if (xmit->tail < xmit->head) {
		memcpy((unsigned char *)p->src_mem_p.vir_addr, &xmit->buf[xmit->tail], p->tx_dma_len);
	} else {
		size_t first = UART_XMIT_SIZE - xmit->tail;
		size_t second = xmit->head;
		memcpy((unsigned char *)p->src_mem_p.vir_addr, &xmit->buf[xmit->tail], first);
		if (second)
			memcpy((unsigned char *)p->src_mem_p.vir_addr+first, &xmit->buf[0], second);
	}
	xmit->tail = (xmit->tail +  p->tx_dma_len) & (UART_XMIT_SIZE - 1);

	serial_out(p, UART_REG_IER, (serial_in(p, UART_REG_IER) &~ TXPDMAEN));
	pdma_tx->slave_config.dst_addr = (unsigned int)(p->port.membase - 0x40000000);
	pdma_tx->slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	pdma_tx->slave_config.dst_maxburst = 1;
	pdma_tx->slave_config.direction = DMA_MEM_TO_DEV;
	dmaengine_slave_config(pdma_tx->chan_tx,&(pdma_tx->slave_config));
	sg_init_table(pdma_tx->sgtx, 1);
	pdma_tx->sgtx[0].dma_address =p->src_mem_p.phy_addr;
	pdma_tx->sgtx[0].length = p->tx_dma_len;
	dma_ctx.reqsel = p->PDMA_UARTx_TX;
	// disable time-out
	dma_ctx.timeout_counter = 0;
	dma_ctx.timeout_prescaler = 0;
	dma_ctx.en_sc = 0;
	pdma_tx->txdesc = pdma_tx->chan_tx->device->device_prep_slave_sg(pdma_tx->chan_tx,
	                  pdma_tx->sgtx,
	                  1,
	                  DMA_TO_DEVICE,
	                  DMA_PREP_INTERRUPT | DMA_CTRL_ACK,
	                  (void *)&dma_ctx);
	if (!pdma_tx->txdesc) {
		printk("pdma->txdes==NULL\n");
		while(1);
	}

	pdma_tx->txdesc->callback = nuc980_Tx_dma_callback;
	p->dma_Tx_done.callback_param = p;
	pdma_tx->txdesc->callback_param = &(p->dma_Tx_done);

	cookie = pdma_tx->txdesc->tx_submit(pdma_tx->txdesc);
	if (dma_submit_error(cookie)) {
		printk("dma_submit_error\n");
		while(1);
	}
}

#endif


static void rs485_start_rx(struct uart_nuc980_port *port)
{
#if 0  // user can enable to control RTS pin level
	// when enable this define, user need disable auto-flow control
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;

	if(port->rs485.flags & SER_RS485_RTS_AFTER_SEND) {
		// Set logical level for RTS pin equal to high
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) & ~0x200) );
	} else {
		// Set logical level for RTS pin equal to low
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) | 0x200) );
	}
#endif
}

static void rs485_stop_rx(struct uart_nuc980_port *port)
{
#if 0  // user can enable to control RTS pin level
	// when enable this define, user need disable auto-flow control
	if(port->rs485.flags & SER_RS485_RTS_ON_SEND) {
		// Set logical level for RTS pin equal to high
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) & ~0x200) );
	} else {
		// Set logical level for RTS pin equal to low
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) | 0x200) );
	}
#endif

}

static inline void __stop_tx(struct uart_nuc980_port *p)
{
	unsigned int ier;
	struct tty_struct *tty = p->port.state->port.tty;

	if ((ier = serial_in(p, UART_REG_IER)) & THRE_IEN) {
		serial_out(p, UART_REG_IER, ier & ~THRE_IEN);
	}
	if (p->rs485.flags & SER_RS485_ENABLED)
		rs485_start_rx(p);

	if (tty->termios.c_line == N_IRDA) {
		while(!(serial_in(p, UART_REG_FSR) & TX_EMPTY));
		while(!(serial_in(p, UART_REG_FSR) & TE_FLAG));

		serial_out(p, UART_REG_IRCR, (serial_in(p, UART_REG_IRCR) & ~0x2) ); // Tx disable (select Rx)
	}

}

static void nuc980serial_stop_tx(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;

	__stop_tx(up);
}

static void transmit_chars(struct uart_nuc980_port *up);

static void nuc980serial_start_tx(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	unsigned int ier;
	struct tty_struct *tty = up->port.state->port.tty;
#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	struct circ_buf *xmit = &up->port.state->xmit;
#endif

	if (tty->termios.c_line == N_IRDA) {
		serial_out(up, UART_REG_IRCR, (serial_in(up, UART_REG_IRCR) | 0x2) ); // Tx enable
	}

	if (up->rs485.flags & SER_RS485_ENABLED)
		rs485_stop_rx(up);

#if 0   // No use FIFO
	if (!((ier = serial_in(up, UART_REG_IER)) & THRE_IEN)) {
		ier |= THRE_IEN;
		serial_out(up, UART_REG_IER, ier);
	}
#else // use FIFO
#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	if(up->uart_pdma_enable_flag == 1) {
		if(up->Tx_pdma_busy_flag == 1) {
			return;
		}

		if (uart_circ_empty(xmit)) {
			__stop_tx(up);
			return;
		}

		up->Tx_pdma_busy_flag = 1;
		nuc980_prepare_TX_dma(up);
		serial_out(up, UART_REG_IER, (serial_in(up, UART_REG_IER)|TXPDMAEN));
	} else
#endif
	{
		struct circ_buf *xmit = &up->port.state->xmit;
		ier = serial_in(up, UART_REG_IER);
		serial_out(up, UART_REG_IER, ier & ~THRE_IEN);
		if( uart_circ_chars_pending(xmit)<(16-((serial_in(up, UART_REG_FSR)>>16)&0x3F)) )
			transmit_chars(up);
		serial_out(up, UART_REG_IER, ier | THRE_IEN);
	}
#endif

}

static void nuc980serial_stop_rx(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;

	serial_out(up, UART_REG_IER, serial_in(up, UART_REG_IER) & ~RDA_IEN);
}

static void nuc980serial_enable_ms(struct uart_port *port)
{

}

static int max_count = 0;

static void
receive_chars(struct uart_nuc980_port *up)
{
	unsigned char ch;
	unsigned int fsr;
	unsigned int isr;
	unsigned int dcnt;

	char flag;
	isr = serial_in(up, UART_REG_ISR);
	fsr = serial_in(up, UART_REG_FSR);

	while(!(fsr & RX_EMPTY)) {
		//fsr = serial_in(up, UART_REG_FSR);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(fsr & (BIF | FEF | PEF | RX_OVER_IF))) {
			if (fsr & BIF) {
				serial_out(up, UART_REG_FSR, BIF);
				up->port.icount.brk++;
				if (uart_handle_break(&up->port))
					continue;
			}

			if (fsr & FEF) {
				serial_out(up, UART_REG_FSR, FEF);
				up->port.icount.frame++;
			}

			if (fsr & PEF) {
				serial_out(up, UART_REG_FSR, PEF);
				up->port.icount.parity++;
			}

			if (fsr & RX_OVER_IF) {
				serial_out(up, UART_REG_FSR, RX_OVER_IF);
				up->port.icount.overrun++;
			}
			// FIXME: check port->read_status_mask to determin report flags
			if (fsr & BIF)
				flag = TTY_BREAK;
			if (fsr & PEF)
				flag = TTY_PARITY;
			if (fsr & FEF)
				flag = TTY_FRAME;
		}

		ch = (unsigned char)serial_in(up, UART_REG_RBR);

		if (uart_handle_sysrq_char(&up->port, ch))
			continue;

		uart_insert_char(&up->port, fsr, RX_OVER_IF, ch, flag);
		max_count++;
		dcnt=(serial_in(up, UART_REG_FSR) >> 8) & 0x3f;
		if(max_count > 1023)
		{
			spin_lock(&up->port.lock);
			tty_flip_buffer_push(&up->port.state->port);
			spin_unlock(&up->port.lock);
			max_count=0;
			if((isr & TOUT_IF) && (dcnt == 0))
				goto tout_end;
		}

		if(isr & RDA_IF) {
			if(dcnt == 1)
				return; // have remaining data, don't reset max_count
		}
		fsr = serial_in(up, UART_REG_FSR);
	}

	spin_lock(&up->port.lock);
	tty_flip_buffer_push(&up->port.state->port);
	spin_unlock(&up->port.lock);
tout_end:
	max_count=0;
	return;
}

static void transmit_chars(struct uart_nuc980_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	//int count = 12;
	int count = 16 -((serial_in(up, UART_REG_FSR)>>16)&0x3F);

	if (up->port.x_char) {
		while(serial_in(up, UART_REG_FSR) & TX_FULL);
		serial_out(up, UART_REG_THR, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}

	if (uart_tx_stopped(&up->port)) {
		nuc980serial_stop_tx(&up->port);
		return;
	}

	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

	do {
		//while(serial_in(up, UART_REG_FSR) & TX_FULL);
		serial_out(up, UART_REG_THR, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		__stop_tx(up);


}

static unsigned int check_modem_status(struct uart_nuc980_port *up)
{
	unsigned int status = 0;

	if (0) {
		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}

	return status;
}

static irqreturn_t nuc980serial_interrupt(int irq, void *dev_id)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)dev_id;
	unsigned int isr, fsr;

	isr = serial_in(up, UART_REG_ISR);
	fsr = serial_in(up, UART_REG_FSR);

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	if(up->uart_pdma_enable_flag == 1) {
		if(fsr & (BIF | FEF | PEF | RX_OVER_IF | HWBUFE_IF)) {
			serial_out(up, UART_REG_FSR, (BIF | FEF | PEF | RX_OVER_IF | TX_OVER_IF));
		}
	} else
#endif
	{
		//isr = serial_in(up, UART_REG_ISR);

		if (isr & (RDA_IF | TOUT_IF))
			receive_chars(up);

		check_modem_status(up);

		if (isr & THRE_INT)
			transmit_chars(up);

		if(fsr & (BIF | FEF | PEF | RX_OVER_IF)) {
			serial_out(up, UART_REG_FSR, (BIF | FEF | PEF | RX_OVER_IF | TX_OVER_IF));
		}
	}

	return IRQ_HANDLED;
}

static unsigned int nuc980serial_tx_empty(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	//unsigned long flags;
	unsigned int fsr;

	//spin_lock_irqsave(&up->port.lock, flags);
	fsr = serial_in(up, UART_REG_FSR);
	//spin_unlock_irqrestore(&up->port.lock, flags);

	return (fsr & (TE_FLAG | TX_EMPTY)) == (TE_FLAG | TX_EMPTY) ? TIOCSER_TEMT : 0;
}

static unsigned int nuc980serial_get_mctrl(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	unsigned int status;
	unsigned int ret = 0;

	status = serial_in(up, UART_REG_MSR);;

	if(!(status & 0x10))
		ret |= TIOCM_CTS;

	return ret;
}

static void nuc980serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	unsigned int mcr = 0;
	unsigned int ier = 0;

	if (mctrl & TIOCM_RTS) {
		// set RTS high level trigger
		mcr = serial_in(up, UART_REG_MCR);
		mcr |= 0x200;
		mcr &= ~(0x2);
	}

	if (up->mcr & UART_MCR_AFE) {
		// set RTS high level trigger
		mcr = serial_in(up, UART_REG_MCR);
		mcr |= 0x200;
		mcr &= ~(0x2);

		// enable CTS/RTS auto-flow control
		serial_out(up, UART_REG_IER, (serial_in(up, UART_REG_IER) | (0x3000)));

		// Set hardware flow control
		up->port.flags |= UPF_HARD_FLOW;
	} else {
		// disable CTS/RTS auto-flow control
		ier = serial_in(up, UART_REG_IER);
		ier &= ~(0x3000);
		serial_out(up, UART_REG_IER, ier);

		//un-set hardware flow control
		up->port.flags &= ~UPF_HARD_FLOW;
	}

	// set CTS high level trigger
	serial_out(up, UART_REG_MSR, (serial_in(up, UART_REG_MSR) | (0x100)));
	serial_out(up, UART_REG_MCR, mcr);
}

static void nuc980serial_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	unsigned long flags;
	unsigned int lcr;

	spin_lock_irqsave(&up->port.lock, flags);
	lcr = serial_in(up, UART_REG_LCR);
	if (break_state != 0)
		lcr |= BCB; // set break
	else
		lcr &= ~BCB;    // clr break
	serial_out(up, UART_REG_LCR, lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int nuc980serial_startup(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	struct tty_struct *tty = port->state->port.tty;
	int retval;
#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	struct nuc980_ip_rx_dma *pdma_rx = &(up->dma_rx);
	struct nuc980_ip_tx_dma *pdma_tx = &(up->dma_tx);

	dma_cap_mask_t mask;
#endif

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);

	if(up->uart_pdma_enable_flag == 1) {
		pdma_rx->chan_rx = dma_request_channel(mask, NULL, NULL);
		if (!pdma_rx->chan_rx) {
			printk("RX DMA channel request error\n");
			return -1;
		}
		pdma_rx->chan_rx->private=(void *)1;

		pdma_tx->chan_tx = dma_request_channel(mask, NULL, NULL);
		if (!pdma_tx->chan_tx) {
			printk("TX DMA channel request error\n");
			return -1;
		}
		pdma_tx->chan_tx->private=(void *)1;
	}
#endif

	/* Reset FIFO */
	serial_out(up, UART_REG_FCR, TFR | RFR /* | RX_DIS */);

	/* Clear pending interrupts (not every bit are write 1 clear though...) */
	serial_out(up, UART_REG_ISR, 0xFFFFFFFF);

	retval = request_irq(port->irq, nuc980serial_interrupt, IRQF_NO_SUSPEND, tty ? tty->name : "nuc980_serial", port);

	if (retval) {
		printk("request irq failed...\n");
		return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_REG_FCR, serial_in(up, UART_REG_FCR) | 0x10); // Trigger level 4 byte
	serial_out(up, UART_REG_LCR, 0x7); // 8 bit
	serial_out(up, UART_REG_TOR, 0x40);

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	if(up->uart_pdma_enable_flag == 1)
		serial_out(up, UART_REG_IER, RLS_IEN | BUFERR_IEN);
	else
#endif
		serial_out(up, UART_REG_IER, RTO_IEN | RDA_IEN | TIME_OUT_EN | BUFERR_IEN);
	//serial_out(up, UART_REG_IER, RTO_IEN | RDA_IEN | TIME_OUT_EN);

	/* 12MHz reference clock input, 115200 */
	serial_out(up, UART_REG_BAUD, 0x30000066);

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	if(up->uart_pdma_enable_flag == 1) {
		nuc980_prepare_RX_dma(up);

		nuc980_prepare_TX_dma(up);

		// trigger pdma
		serial_out(up, UART_REG_IER, (serial_in(up, UART_REG_IER)|RXPDMAEN));
	}
#endif

	return 0;
}

static void nuc980serial_shutdown(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	struct nuc980_ip_rx_dma *pdma_rx = &(up->dma_rx);
	struct nuc980_ip_tx_dma *pdma_tx = &(up->dma_tx);
#endif

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
	if(up->uart_pdma_enable_flag == 1) {
		dma_release_channel(pdma_rx->chan_rx);
		dma_release_channel(pdma_tx->chan_tx);

#ifdef USING_SRAM
		sram_free((void *)up->dest_mem_p.vir_addr, up->dest_mem_p.size);
#else
		if(up->dest_mem_p.size != 0)
		{
			dma_free_writecombine(NULL, up->dest_mem_p.size, (void *)up->dest_mem_p.vir_addr, up->dest_mem_p.phy_addr);
		}
#endif

		if(up->src_mem_p.size != 0)
		{
			dma_free_writecombine(NULL, up->src_mem_p.size, (void *)up->src_mem_p.vir_addr, up->src_mem_p.phy_addr);
		}

		up->Tx_pdma_busy_flag = 0;
		up->dest_mem_p.size = 0;
		up->src_mem_p.size = 0;
	}
#endif

	free_irq(port->irq, port);

	/*
	 * Disable interrupts from this port
	 */
	serial_out(up, UART_REG_IER, 0);
}

static unsigned int nuc980serial_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	quot = (port->uartclk / baud) - 2;

	return quot;
}

static void
nuc980serial_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	unsigned int lcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = 0;
		break;
	case CS6:
		lcr |= 1;
		break;
	case CS7:
		lcr |= 2;
		break;
	default:
	case CS8:
		lcr |= 3;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= NSB;
	if (termios->c_cflag & PARENB)
		lcr |= PBE;
	if (!(termios->c_cflag & PARODD))
		lcr |= EPE;
	if (termios->c_cflag & CMSPAR)
		lcr |= SPE;

	baud = uart_get_baud_rate(port, termios, old, port->uartclk / 0xffff, port->uartclk / 11);

	quot = nuc980serial_get_divisor(port, baud);

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	up->port.read_status_mask = RX_OVER_IF /*| UART_LSR_THRE | UART_LSR_DR*/;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= FEF | PEF;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= BIF;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= FEF | PEF;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= BIF;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= RX_OVER_IF;
	}

	if (termios->c_cflag & CRTSCTS)
		up->mcr |= UART_MCR_AFE;
	else
		up->mcr &= ~UART_MCR_AFE;

	nuc980serial_set_mctrl(&up->port, up->port.mctrl);

	serial_out(up, UART_REG_BAUD, quot | 0x30000000);

	serial_out(up, UART_REG_LCR, lcr);

	spin_unlock_irqrestore(&up->port.lock, flags);

}

static void
nuc980serial_set_ldisc(struct uart_port *port, struct ktermios *termios)
{
	struct uart_nuc980_port *uart = (struct uart_nuc980_port *)port;
	unsigned int baud;

	switch (termios->c_line) {
	case N_IRDA:

		baud = serial_in(uart, UART_REG_BAUD);
		baud = baud & (0x0000ffff);
		baud = baud + 2;
		baud = baud / 16;
		baud = baud - 2;

		serial_out(uart, UART_REG_BAUD, baud);
		serial_out(uart, UART_REG_IRCR, (serial_in(uart, UART_REG_IRCR) & ~0x40) );  // Rx inverse

		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) & ~FUN_SEL_Msk) );
		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) | FUN_SEL_IrDA) );

		break;
	default:
		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) & ~FUN_SEL_Msk) );
	}

}

static void
nuc980serial_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
	struct uart_nuc980_port *p = (struct uart_nuc980_port *)port;

	if (p->pm)
		p->pm(port, state, oldstate);
}

static void nuc980serial_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

	release_mem_region(port->mapbase, size);

	iounmap(port->membase);
	port->membase = NULL;
}

static int nuc980serial_request_port(struct uart_port *port)
{
	return 0;
}

static void nuc980serial_config_port(struct uart_port *port, int flags)
{
	int ret;

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = nuc980serial_request_port(port);
	if (ret < 0)
		return;
	port->type = PORT_NUC980;
}

static int nuc980serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_NUC980)
		return -EINVAL;
	return 0;
}

static const char *nuc980serial_type(struct uart_port *port)
{
	return (port->type == PORT_NUC980) ? "NUC980" : NULL;
}

/* Enable or disable the rs485 support */
static int nuc980serial_config_rs485(struct uart_port *port, struct serial_rs485 *rs485conf)
{
	struct uart_nuc980_port *p = to_nuc980_uart_port(port);

	p->rs485 = *rs485conf;

	if (p->rs485.delay_rts_before_send >= 1000)
		p->rs485.delay_rts_before_send = 1000;

	serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) & ~FUN_SEL_Msk) );

	if(rs485conf->flags & SER_RS485_ENABLED) {
		serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) | FUN_SEL_RS485) );

		//rs485_start_rx(p);    // stay in Rx mode

		if(rs485conf->flags & SER_RS485_RTS_ON_SEND) {
			serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) & ~0x200) );
		} else {
			serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) | 0x200) );
		}

		// set auto direction mode
		serial_out(p,UART_REG_ALT_CSR,(serial_in(p, UART_REG_ALT_CSR) | (1 << 10)) );
	}

	return 0;
}

static int nuc980serial_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {

	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static struct uart_ops nuc980serial_ops = {
	.tx_empty    = nuc980serial_tx_empty,
	.set_mctrl   = nuc980serial_set_mctrl,
	.get_mctrl   = nuc980serial_get_mctrl,
	.stop_tx     = nuc980serial_stop_tx,
	.start_tx    = nuc980serial_start_tx,
	.stop_rx     = nuc980serial_stop_rx,
	.enable_ms   = nuc980serial_enable_ms,
	.break_ctl   = nuc980serial_break_ctl,
	.startup     = nuc980serial_startup,
	.shutdown    = nuc980serial_shutdown,
	.set_termios = nuc980serial_set_termios,
	.set_ldisc   = nuc980serial_set_ldisc,
	.pm          = nuc980serial_pm,
	.type        = nuc980serial_type,
	.release_port= nuc980serial_release_port,
	.request_port= nuc980serial_request_port,
	.config_port = nuc980serial_config_port,
	.verify_port = nuc980serial_verify_port,
	.ioctl       = nuc980serial_ioctl,
};

static void __init nuc980serial_init_ports(void)
{
	static int first = 1;
	int i;

	// enable clock
	clk = clk_get(NULL, "uart0");
	clk_prepare(clk);
	clk_enable(clk);

	// UART0 multi-function  PF11,PF12
	//__raw_writel((__raw_readl(NUC980_VA_GCR+0x9C)&0xfff00fff) | 0x11000,(NUC980_VA_GCR+0x9C));
	if (!first)
		return;
	first = 0;

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc980_port *up = &nuc980serial_ports[i];

		up->port.line = i;
		spin_lock_init(&up->port.lock);

		up->port.ops = &nuc980serial_ops;
		up->port.iobase = (long)(NUC980_VA_UART0 + (i*0x1000));
		up->port.membase = NUC980_VA_UART0 + (i*0x1000);
		up->port.uartclk = 12000000;

	}
}

#ifdef CONFIG_SERIAL_NUC980_CONSOLE
static void nuc980serial_console_putchar(struct uart_port *port, int ch)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;

	while(!(serial_in(up, UART_REG_FSR) & TX_EMPTY));
	serial_out(up, UART_REG_THR, ch);
}

/*
 *  Print a string to the serial port trying not to disturb
 *  any possible real use of the port...
 *
 *  The console_lock must be held when we get here.
 */
static void nuc980serial_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_nuc980_port *up = &nuc980serial_ports[co->index];
	unsigned long flags;
	unsigned int ier;

	local_irq_save(flags);

	/*
	 *  First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_REG_IER);
	serial_out(up, UART_REG_IER, 0);

	uart_console_write(&up->port, s, count, nuc980serial_console_putchar);

	/*
	 *  Finally, wait for transmitter to become empty
	 *  and restore the IER
	 */
	while(!(serial_in(up, UART_REG_FSR) & TX_EMPTY));
	serial_out(up, UART_REG_IER, ier);

	local_irq_restore(flags);
}

static int __init nuc980serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	port = &nuc980serial_ports[co->index].port;

	if (!port->iobase && !port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}


static struct console nuc980serial_console = {
	.name    = "ttyS",
	.write   = nuc980serial_console_write,
	.device  = uart_console_device,
	.setup   = nuc980serial_console_setup,
	.flags   = CON_PRINTBUFFER,
	.index   = -1,
	.data    = &nuc980serial_reg,
};

static int __init nuc980serial_console_init(void)
{
	nuc980serial_init_ports();
	register_console(&nuc980serial_console);

	return 0;
}
console_initcall(nuc980serial_console_init);

#define NUC980SERIAL_CONSOLE    &nuc980serial_console
#else
#define NUC980SERIAL_CONSOLE    NULL
#endif

static struct uart_driver nuc980serial_reg = {
	.owner        = THIS_MODULE,
	.driver_name  = "serial",
	.dev_name     = "ttyS",
	.major        = TTY_MAJOR,
	.minor        = 64,
	.cons         = NUC980SERIAL_CONSOLE,
	.nr           = UART_NR,
};


/**
 *
 *  Suspend one serial port.
 */
void nuc980serial_suspend_port(int line)
{
	uart_suspend_port(&nuc980serial_reg, &nuc980serial_ports[line].port);
}

/**
 *
 *  Resume one serial port.
 */
void nuc980serial_resume_port(int line)
{
	struct uart_nuc980_port *up = &nuc980serial_ports[line];

	uart_resume_port(&nuc980serial_reg, &up->port);
}

#ifndef CONFIG_USE_OF
static int nuc980serial_pinctrl(struct platform_device *pdev)
{
	struct pinctrl *p = NULL;
	int retval = 0;

	if(pdev->id == 1) {
#if defined (CONFIG_NUC980_UART1_PA)
		p = devm_pinctrl_get_select(&pdev->dev, "uart1-PA");
#elif defined (CONFIG_NUC980_UART1_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "uart1-PC");
#elif defined (CONFIG_NUC980_UART1_PF)
		p = devm_pinctrl_get_select(&pdev->dev, "uart1-PF");
#elif defined (CONFIG_NUC980_UART1_FC_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "uart1-fc-PC");
#elif defined (CONFIG_NUC980_UART1_FC_PF)
		p = devm_pinctrl_get_select(&pdev->dev, "uart1-fc-PF");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	} else if(pdev->id == 2) {
#if defined (CONFIG_NUC980_UART2_PA)
		p = devm_pinctrl_get_select(&pdev->dev, "uart2-PA");
#elif defined (CONFIG_NUC980_UART2_PG)
		p = devm_pinctrl_get_select(&pdev->dev, "uart2-PG");
#elif defined (CONFIG_NUC980_UART2_PD)
		p = devm_pinctrl_get_select(&pdev->dev, "uart2-PD");
#elif defined (CONFIG_NUC980_UART2_FC_PA)
		p = devm_pinctrl_get_select(&pdev->dev, "uart2-fc-PA");
#elif defined (CONFIG_NUC980_UART2_FC_PG)
		p = devm_pinctrl_get_select(&pdev->dev, "uart2-fc-PG");
#elif defined (CONFIG_NUC980_UART2_FC_PA_PB)
		p = devm_pinctrl_get_select(&pdev->dev, "uart2-fc-PA_PB");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	} else if(pdev->id == 3) {
#if defined (CONFIG_NUC980_UART3_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "uart3-PC");
#elif defined (CONFIG_NUC980_UART3_PB)
		p = devm_pinctrl_get_select(&pdev->dev, "uart3-PB");
#elif defined (CONFIG_NUC980_UART3_PD)
		p = devm_pinctrl_get_select(&pdev->dev, "uart3-PD");
#elif defined (CONFIG_NUC980_UART3_PB_PF)
		p = devm_pinctrl_get_select(&pdev->dev, "uart3-PB_PF");
#elif defined (CONFIG_NUC980_UART3_PF)
		p = devm_pinctrl_get_select(&pdev->dev, "uart3-PF");
#elif defined (CONFIG_NUC980_UART3_FC_PB)
		p = devm_pinctrl_get_select(&pdev->dev, "uart3-fc-PB");
#elif defined (CONFIG_NUC980_UART3_FC_PD)
		p = devm_pinctrl_get_select(&pdev->dev, "uart3-fc-PD");
#elif defined (CONFIG_NUC980_UART3_FC_PF)
		p = devm_pinctrl_get_select(&pdev->dev, "uart3-fc-PF");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	} else if(pdev->id == 4) {
#if defined (CONFIG_NUC980_UART4_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "uart4-PC");
#elif defined (CONFIG_NUC980_UART4_PD)
		p = devm_pinctrl_get_select(&pdev->dev, "uart4-PD");
#elif defined (CONFIG_NUC980_UART4_PE)
		p = devm_pinctrl_get_select(&pdev->dev, "uart4-PE");
#elif defined (CONFIG_NUC980_UART4_FC_PD)
		p = devm_pinctrl_get_select(&pdev->dev, "uart4-fc-PD");
#elif defined (CONFIG_NUC980_UART4_FC_PE)
		p = devm_pinctrl_get_select(&pdev->dev, "uart4-fc-PE");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	} else if(pdev->id == 5) {
#if defined (CONFIG_NUC980_UART5_PG_0)
		p = devm_pinctrl_get_select(&pdev->dev, "uart5-PG_0");
#elif defined (CONFIG_NUC980_UART5_PD)
		p = devm_pinctrl_get_select(&pdev->dev, "uart5-PD");
#elif defined (CONFIG_NUC980_UART5_PG_1)
		p = devm_pinctrl_get_select(&pdev->dev, "uart5-PG_1");
#elif defined (CONFIG_NUC980_UART5_FC_PG_0)
		p = devm_pinctrl_get_select(&pdev->dev, "uart5-fc-PG_0");
#elif defined (CONFIG_NUC980_UART5_FC_PG_1)
		p = devm_pinctrl_get_select(&pdev->dev, "uart5-fc-PG_1");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	} else if(pdev->id == 6) {
#if defined (CONFIG_NUC980_UART6_PA)
		p = devm_pinctrl_get_select(&pdev->dev, "uart6-PA");
#elif defined (CONFIG_NUC980_UART6_PD)
		p = devm_pinctrl_get_select(&pdev->dev, "uart6-PD");
#elif defined (CONFIG_NUC980_UART6_PE)
		p = devm_pinctrl_get_select(&pdev->dev, "uart6-PE");
#elif defined (CONFIG_NUC980_UART6_FC_PA)
		p = devm_pinctrl_get_select(&pdev->dev, "uart6-fc-PA");
#elif defined (CONFIG_NUC980_UART6_FC_PD)
		p = devm_pinctrl_get_select(&pdev->dev, "uart6-fc-PD");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	} else if(pdev->id == 7) {
#if defined (CONFIG_NUC980_UART7_PA)
		p = devm_pinctrl_get_select(&pdev->dev, "uart7-PA");
#elif defined (CONFIG_NUC980_UART7_PB)
		p = devm_pinctrl_get_select(&pdev->dev, "uart7-PB");
#elif defined (CONFIG_NUC980_UART7_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "uart7-PC");
#elif defined (CONFIG_NUC980_UART7_PF)
		p = devm_pinctrl_get_select(&pdev->dev, "uart7-PF");
#elif defined (CONFIG_NUC980_UART7_FC_PB)
		p = devm_pinctrl_get_select(&pdev->dev, "uart7-fc-PB");
#elif defined (CONFIG_NUC980_UART7_FC_PF)
		p = devm_pinctrl_get_select(&pdev->dev, "uart7-fc-PF");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	} else if(pdev->id == 8) {
#if defined (CONFIG_NUC980_UART8_PA)
		p = devm_pinctrl_get_select(&pdev->dev, "uart8-PA");
#elif defined (CONFIG_NUC980_UART8_PB)
		p = devm_pinctrl_get_select(&pdev->dev, "uart8-PB");
#elif defined (CONFIG_NUC980_UART8_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "uart8-PC");
#elif defined (CONFIG_NUC980_UART8_FC_PA_PG)
		p = devm_pinctrl_get_select(&pdev->dev, "uart8-fc-PA_PG");
#elif defined (CONFIG_NUC980_UART8_FC_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "uart8-fc-PC");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	} else if(pdev->id == 9) {
#if defined (CONFIG_NUC980_UART9_PB)
		p = devm_pinctrl_get_select(&pdev->dev, "uart9-PB");
#elif defined (CONFIG_NUC980_UART9_PE_0)
		p = devm_pinctrl_get_select(&pdev->dev, "uart9-PE_0");
#elif defined (CONFIG_NUC980_UART9_PE_1)
		p = devm_pinctrl_get_select(&pdev->dev, "uart9-PE_1");
#elif defined (CONFIG_NUC980_UART9_FC_PE)
		p = devm_pinctrl_get_select(&pdev->dev, "uart9-fc-PE");
#endif

		if (IS_ERR(p)) {
			dev_err(&pdev->dev, "unable to reserve pin\n");
			retval = PTR_ERR(p);
		}
	}

	return retval;
}
#endif

void nuc980serial_set_clock(struct uart_nuc980_port *up)
{
	struct clk *clkmux;
	struct clk *upll_clk;

	if(up->port.line == 0) {
		clk = clk_get(NULL, "uart0");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart0_eclk");
		clk_prepare(clk);
		clk_enable(clk);
	}

#ifdef CONFIG_NUC980_UART1
	if(up->port.line == 1) {
		clk = clk_get(NULL, "uart1");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart1_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart1_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart1_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

#ifdef CONFIG_NUC980_UART2
	if(up->port.line == 2) {
		clk = clk_get(NULL, "uart2");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart2_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart2_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart2_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

#ifdef CONFIG_NUC980_UART3
	if(up->port.line == 3) {
		clk = clk_get(NULL, "uart3");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart3_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart3_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart3_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

#ifdef CONFIG_NUC980_UART4
	if(up->port.line == 4) {
		clk = clk_get(NULL, "uart4");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart4_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart4_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart4_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

#ifdef CONFIG_NUC980_UART5
	if(up->port.line == 5) {
		clk = clk_get(NULL, "uart5");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart5_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart5_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart5_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

#ifdef CONFIG_NUC980_UART6
	if(up->port.line == 6) {
		clk = clk_get(NULL, "uart6");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart6_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart6_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart6_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

#ifdef CONFIG_NUC980_UART7
	if(up->port.line == 7) {
		clk = clk_get(NULL, "uart7");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart7_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart7_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart7_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

#ifdef CONFIG_NUC980_UART8
	if(up->port.line == 8) {
		clk = clk_get(NULL, "uart8");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart8_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart8_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart8_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

#ifdef CONFIG_NUC980_UART9
	if(up->port.line == 9) {
		clk = clk_get(NULL, "uart9");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "uart9_eclk");
		clk_prepare(clk);
		clk_enable(clk);

		clkmux = clk_get(NULL, "uart9_eclk_mux");
		upll_clk = clk_get(NULL, "upll");
		clk_set_parent(clkmux, upll_clk);

		clk = clk_get(NULL, "uart9_eclk_div");

		//clk_set_rate(clk, 100000000);
		clk_set_rate(clk, 150000000);
		up->port.uartclk = clk_get_rate(clk);
	}
#endif

}

#if defined(CONFIG_USE_OF)
static int  get_uart_port_number(struct platform_device *pdev)
{
	u32   val32[2];

	if (of_property_read_u32_array(pdev->dev.of_node, "port-number", val32, 1) != 0) {
		printk("%s - can not get port-number!\n", __func__);
		return -EINVAL;
	}

	return val32[0];
}
#endif

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int nuc980serial_probe(struct platform_device *pdev)
{
	struct uart_nuc980_port *up;

	int ret, i;

#if defined(CONFIG_USE_OF)
	struct pinctrl *pinctrl;
	u32   val32[2];
#else
	int retval;
	struct plat_nuc980serial_port *p = pdev->dev.platform_data;
#endif

#if defined(CONFIG_USE_OF)
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		return PTR_ERR(pinctrl);
	}
#else
	retval = nuc980serial_pinctrl(pdev);
	if(retval != 0)
		return retval;
#endif

#if defined(CONFIG_USE_OF)
	i = get_uart_port_number(pdev);
	if (i < 0)
		return -EINVAL;
#else
	i = pdev->id;
#endif

	up = &nuc980serial_ports[i];

	up->port.line = i;

	nuc980serial_set_clock(up);

#if defined(CONFIG_ENABLE_UART_PDMA) || defined(CONFIG_USE_OF)
#if defined(CONFIG_USE_OF)
	if (of_property_read_u32_array(pdev->dev.of_node, "pdma-enable", val32, 1) != 0) {
		printk("%s - can not get map-addr!\n", __func__);
		return -EINVAL;
	}

	if(val32[0] == 1) set_pdma_flag(up, i);

#else
	set_pdma_flag(up, i);
#endif
#endif

#if defined(CONFIG_USE_OF)
	/*--------------------------------------------------------------*/
	/*  get UART register map address from DTB                      */
	/*--------------------------------------------------------------*/
	if (of_property_read_u32_array(pdev->dev.of_node, "map-addr", val32, 1) != 0) {
		printk("%s - can not get map-addr!\n", __func__);
		return -EINVAL;
	}

	up->port.membase = (unsigned char __iomem *)val32[0];

	up->port.iobase         = (unsigned long)up->port.membase;
	up->port.irq            = platform_get_irq(pdev, 0);
	up->port.dev            = &pdev->dev;
	up->port.flags          = ASYNC_BOOT_AUTOCONF;

#else
	up->port.membase        = p->membase;

	up->port.iobase         = p->iobase;
	up->port.irq            = p->irq;
	//up->port.uartclk        = p->uartclk;
	up->port.mapbase        = p->mapbase;
	up->port.private_data   = p->private_data;
	up->port.dev            = &pdev->dev;
	up->port.flags          = ASYNC_BOOT_AUTOCONF;

	/* Possibly override default I/O functions.  */
	if (p->serial_in)
		up->port.serial_in = p->serial_in;
	if (p->serial_out)
		up->port.serial_out = p->serial_out;

#endif

	up->port.rs485_config = nuc980serial_config_rs485;

	ret = uart_add_one_port(&nuc980serial_reg, &up->port);
	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int nuc980serial_remove(struct platform_device *dev)
{
	int i;
	struct uart_port *port = platform_get_drvdata(dev);

	free_irq(port->irq, port);

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc980_port *up = &nuc980serial_ports[i];

		if (up->port.dev == &dev->dev)
			uart_remove_one_port(&nuc980serial_reg, &up->port);
	}
	return 0;
}

static int nuc980serial_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;
	int wakeup_flag = 0;

	struct uart_nuc980_port *up;

#if defined(CONFIG_USE_OF)
	i = get_uart_port_number(dev);
	if (i < 0)
		return i;
#else
	i = dev->id;
#endif

	up = &nuc980serial_ports[i];

#ifdef CONFIG_ENABLE_UART1_CTS_WAKEUP
	if(i == 1) {
		__raw_writel((1<<17) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
		wakeup_flag = 1;
	}
#endif

#ifdef CONFIG_ENABLE_UART2_CTS_WAKEUP
	if(i == 2) {
		__raw_writel((1<<18) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
		wakeup_flag = 1;
	}
#endif

#ifdef CONFIG_ENABLE_UART3_CTS_WAKEUP
	if(i == 3) {
		__raw_writel((1<<19) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
		wakeup_flag = 1;
	}
#endif

#ifdef CONFIG_ENABLE_UART4_CTS_WAKEUP
	if(i == 4) {
		__raw_writel((1<<20) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
		wakeup_flag = 1;
	}
#endif

#ifdef CONFIG_ENABLE_UART5_CTS_WAKEUP
	if(i == 5) {
		__raw_writel((1<<21) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
		wakeup_flag = 1;
	}
#endif

#ifdef CONFIG_ENABLE_UART6_CTS_WAKEUP
	if(i == 6) {
		__raw_writel((1<<22) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
		wakeup_flag = 1;
	}
#endif

#ifdef CONFIG_ENABLE_UART7_CTS_WAKEUP
	if(i == 7) {
		__raw_writel((1<<23) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
		wakeup_flag = 1;
	}
#endif

#ifdef CONFIG_ENABLE_UART8_CTS_WAKEUP
	if(i == 8)
		__raw_writel((1<<24) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
	wakeup_flag = 1;
#endif

#ifdef CONFIG_ENABLE_UART9_CTS_WAKEUP
	if(i == 9) {
		__raw_writel((1<<25) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
		wakeup_flag = 1;
	}
#endif

	if(wakeup_flag == 1) {
		serial_out(up, UART_REG_IER, serial_in(up, UART_REG_IER) | (0x1 << 6));
		serial_out(up, UART_REG_WKSTS, 0x1); // Clear CTS Wakeup status
		serial_out(up, UART_REG_WKCTL, 0x1); // Enable CTS Wakeup

		enable_irq_wake(up->port.irq);
	}

	return 0;
}

static int nuc980serial_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc980_port *up = &nuc980serial_ports[i];

		serial_out(up, UART_REG_WKSTS, 0x1); // Clear CTS Wakeup status
	}

	return 0;
}

static const struct of_device_id nuc980_serial_of_match[] = {
	{ .compatible = "nuvoton,nuc980-uart" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_serial_of_match);

static struct platform_driver nuc980serial_driver = {
	.probe      = nuc980serial_probe,
	.remove     = nuc980serial_remove,
	.suspend    = nuc980serial_suspend,
	.resume     = nuc980serial_resume,
	.driver     =
	{
		.name   = "nuc980-uart",
		.owner  = THIS_MODULE,
#if defined(CONFIG_USE_OF)
		.of_match_table = of_match_ptr(nuc980_serial_of_match),
#endif
	},
};

static int __init nuc980serial_init(void)
{
	int ret;

	ret = uart_register_driver(&nuc980serial_reg);
	if (ret)
		return ret;
#ifndef CONFIG_SERIAL_NUC980_CONSOLE
	nuc980serial_init_ports();
#endif
	ret = platform_driver_register(&nuc980serial_driver);
	if (ret)
		uart_unregister_driver(&nuc980serial_reg);

	return ret;
}

static void __exit nuc980serial_exit(void)
{
	platform_driver_unregister(&nuc980serial_driver);
	uart_unregister_driver(&nuc980serial_reg);
}

module_init(nuc980serial_init);
module_exit(nuc980serial_exit);

EXPORT_SYMBOL(nuc980serial_suspend_port);
EXPORT_SYMBOL(nuc980serial_resume_port);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NUC980 serial driver");

MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
