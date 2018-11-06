/*
 *  linux/drivers/tty/serial/nuc980_scuart.c
 *
 *  NUC980 Smartcard UART mode driver
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


#include <linux/export.h>
#include <linux/kernel.h>
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
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/serial.h>

#include <mach/map.h>
#include <mach/regs-sc.h>
#include <mach/regs-gcr.h>
#include <mach/mfp.h>


#define SCUART_NR 2
static struct uart_driver nuc980serial_reg;
struct plat_nuc980serial_port {
	unsigned long	iobase;		/* io base address */
	void __iomem	*membase;	/* ioremap cookie or NULL */
	resource_size_t	mapbase;	/* resource base */
	unsigned int	irq;		/* interrupt number */
	unsigned int	uartclk;	/* UART clock rate */
	void            *private_data;
	unsigned int	(*serial_in)(struct uart_port *, int);
	void		(*serial_out)(struct uart_port *, int, int);
};


struct uart_nuc980_port {
	struct uart_port	port;
	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

static struct uart_nuc980_port nuc980serial_ports[SCUART_NR];

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


static inline void __stop_tx(struct uart_nuc980_port *p)
{
	unsigned int ier;

	if ((ier = serial_in(p, REG_SC_INTEN)) & SC_INTEN_TBEIEN) {
		serial_out(p, REG_SC_INTEN, ier & ~SC_INTEN_TBEIEN);
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


	if (!((ier = serial_in(up, REG_SC_INTEN)) & SC_INTEN_TBEIEN)) {
		ier |= SC_INTEN_TBEIEN;
		serial_out(up, REG_SC_INTEN, ier);
	}

}

static void nuc980serial_stop_rx(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;

	serial_out(up, REG_SC_INTEN, serial_in(up, REG_SC_INTEN) & ~SC_INTEN_RDAIEN);
}

static void nuc980serial_enable_ms(struct uart_port *port)
{

}

static void receive_chars(struct uart_nuc980_port *up)
{
	unsigned char ch;
	unsigned int status;
	int max_count = 256;
	char flag;

	do {
		ch = (unsigned char)serial_in(up, REG_SC_DAT);
		status = serial_in(up, REG_SC_STATUS);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(status & (SC_STATUS_BEF | SC_STATUS_FEF | SC_STATUS_PEF | SC_STATUS_RXOV))) {
			if (status & SC_STATUS_BEF) {
				serial_out(up, REG_SC_STATUS, SC_STATUS_BEF);
				up->port.icount.brk++;
				if (uart_handle_break(&up->port))
					continue;
			}

			if (status & SC_STATUS_FEF) {
				serial_out(up, REG_SC_STATUS, SC_STATUS_FEF);
				up->port.icount.parity++;
			}

			if (status & SC_STATUS_PEF) {
				serial_out(up, REG_SC_STATUS, SC_STATUS_PEF);
				up->port.icount.frame++;
			}

			if (status & SC_STATUS_RXOV) {
				serial_out(up, REG_SC_STATUS, SC_STATUS_RXOV);
				up->port.icount.overrun++;
			}
			// FIXME: check port->read_status_mask to determin report flags
			if (status & SC_STATUS_BEF)
				flag = TTY_BREAK;
			if (status & SC_STATUS_PEF)
				flag = TTY_PARITY;
			if (status & SC_STATUS_FEF)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			continue;

		uart_insert_char(&up->port, status, SC_STATUS_RXOV, ch, flag);

	} while (!(status & SC_STATUS_RXEMPTY) && (max_count-- > 0));

	tty_flip_buffer_push(&up->port.state->port);
}

static void transmit_chars(struct uart_nuc980_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count = 4;	// SCUART FIFO depath is 4 bytes

	if (up->port.x_char) {
		while(serial_in(up, REG_SC_STATUS) & SC_STATUS_TXFULL);
		serial_out(up, REG_SC_DAT, up->port.x_char);
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
		serial_out(up, REG_SC_DAT, xmit->buf[xmit->tail]);
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


static irqreturn_t nuc980serial_interrupt(int irq, void *dev_id)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)dev_id;
	unsigned int isr, ier;

	isr = serial_in(up, REG_SC_INTSTS);
	ier = serial_in(up, REG_SC_INTEN);

	if (isr & (SC_INTSTS_RXTOIF | SC_INTSTS_RDAIF))
		receive_chars(up);

	if ((isr & SC_INTSTS_TBEIF) && (ier & SC_INTEN_TBEIEN))
		transmit_chars(up);

	return IRQ_HANDLED;
}

static unsigned int nuc980serial_tx_empty(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	unsigned long flags;
	unsigned int status;

	spin_lock_irqsave(&up->port.lock, flags);
	status = serial_in(up, REG_SC_STATUS);
	spin_unlock_irqrestore(&up->port.lock, flags);

	return (status & SC_STATUS_TXEMPTY) ? TIOCSER_TEMT : 0;
}

static unsigned int nuc980serial_get_mctrl(struct uart_port *port)
{

	return 0;
}

static void nuc980serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{

}

static void nuc980serial_break_ctl(struct uart_port *port, int break_state)
{

}

static int nuc980serial_startup(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	struct tty_struct *tty = port->state->port.tty;
	int retval;

	/* Reset FIFO */
	serial_out(up, REG_SC_ALTCTL, SC_ALTCTL_RXRST| SC_ALTCTL_TXRST);

	/* Clear pending interrupts (not every bit are write 1 clear though...) */
	serial_out(up, REG_SC_INTSTS, 0xFFFFFFFF);

	retval = request_irq(port->irq, nuc980serial_interrupt, 0,
			tty ? tty->name : "nuc980_scuart", port);

	if (retval) {
		printk("request irq failed...\n");
		return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, REG_SC_CTL, 0x41);	// enable SC engine, trigger level 2 bytes
	serial_out(up, REG_SC_UARTCTL, serial_in(up, REG_SC_UARTCTL) | 1);	// enable UART mode
	serial_out(up, REG_SC_RXTOUT, 0x40);
	serial_out(up, REG_SC_INTEN, SC_INTEN_RXTOIEN | SC_INTEN_RDAIEN);

	return 0;
}

static void nuc980serial_shutdown(struct uart_port *port)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	//unsigned long flags;

	/*
	 * Disable interrupts from this port
	 */
	serial_out(up, REG_SC_INTEN, 0);
	serial_out(up, REG_SC_ALTCTL, SC_ALTCTL_RXRST| SC_ALTCTL_TXRST);
	free_irq(port->irq, port);

}

static unsigned int nuc980serial_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	quot = ((port->uartclk + baud / 2) / baud) - 1;

	return quot;
}

static void nuc980serial_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_nuc980_port *up = (struct uart_nuc980_port *)port;
	unsigned int uartctl = 1, ctl;	// 1 is for enable UART mode
	unsigned long flags;
	unsigned int baud, quot;


	switch (termios->c_cflag & CSIZE) {
	case CS5:
		uartctl |= 0x30;
		break;
	case CS6:
		uartctl |= 0x20;
		break;
	case CS7:
		uartctl |= 0x10;
		break;
	default:
	case CS8:
		uartctl |= 0;
		break;
	}

	if (termios->c_cflag & PARENB)
		uartctl &= ~0x40;
	else
		uartctl |= 0x40;
	if (termios->c_cflag & PARODD)
		uartctl |= 0x80;


	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 0xFFF,
				  port->uartclk / 5);	// 4 < bauddate divider <= 0xFFF

	quot = nuc980serial_get_divisor(port, baud);

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	up->port.read_status_mask = SC_STATUS_RXOV;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= SC_STATUS_FEF | SC_STATUS_PEF;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= SC_STATUS_BEF;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= SC_STATUS_FEF | SC_STATUS_PEF;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= SC_STATUS_BEF;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= SC_STATUS_RXOV;
	}

	serial_out(up, REG_SC_ETUCTL, quot);
	serial_out(up, REG_SC_UARTCTL, uartctl);

	ctl = serial_in(up, REG_SC_CTL);

	if (termios->c_cflag & CSTOPB)
		ctl &= ~SC_CTL_NSB;
	else
		ctl |= SC_CTL_NSB;
	serial_out(up, REG_SC_CTL, ctl);


	spin_unlock_irqrestore(&up->port.lock, flags);

}

static void nuc980serial_set_ldisc(struct uart_port *port, struct ktermios *termios)
{
	return;

}

static void nuc980serial_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
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

static int
nuc980serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_NUC980)
		return -EINVAL;
	return 0;
}

static const char *
nuc980serial_type(struct uart_port *port)
{

	return (port->type == PORT_NUC980) ? "NUC980" : NULL;
}

static struct uart_ops nuc980serial_ops = {
	.tx_empty	= nuc980serial_tx_empty,
	.set_mctrl	= nuc980serial_set_mctrl,
	.get_mctrl	= nuc980serial_get_mctrl,
	.stop_tx	= nuc980serial_stop_tx,
	.start_tx	= nuc980serial_start_tx,
	.stop_rx	= nuc980serial_stop_rx,
	.enable_ms	= nuc980serial_enable_ms,
	.break_ctl	= nuc980serial_break_ctl,
	.startup	= nuc980serial_startup,
	.shutdown	= nuc980serial_shutdown,
	.set_termios	= nuc980serial_set_termios,
	.set_ldisc	= nuc980serial_set_ldisc,
	.pm		= nuc980serial_pm,
	.type		= nuc980serial_type,
	.release_port	= nuc980serial_release_port,
	.request_port	= nuc980serial_request_port,
	.config_port	= nuc980serial_config_port,
	.verify_port	= nuc980serial_verify_port,

};

static void __init nuc980serial_init_ports(void)
{
	int i;

	for (i = 0; i < SCUART_NR; i++) {
		struct uart_nuc980_port *up = &nuc980serial_ports[i];
		up->port.line = i;
		spin_lock_init(&up->port.lock);

		up->port.ops = &nuc980serial_ops;
		up->port.iobase = (long)(NUC980_VA_SC0 + (i * 0x1000));
		up->port.membase = NUC980_VA_SC0 + (i * 0x1000);
		up->port.uartclk = 12000000;

	}
}

static struct uart_driver nuc980serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "sc_serial",
	.dev_name		= "ttySCU",
	.major			= TTY_MAJOR,
	.minor			= 80,//64,  reserve at least 11 for real UART
	.nr			= SCUART_NR,
};


/**
 *
 *	Suspend one serial port.
 */
static void nuc980scuart_suspend_port(int line)
{
	uart_suspend_port(&nuc980serial_reg, &nuc980serial_ports[line].port);
}

/**
 *
 *	Resume one serial port.
 */
static void nuc980scuart_resume_port(int line)
{
	struct uart_nuc980_port *up = &nuc980serial_ports[line];

	uart_resume_port(&nuc980serial_reg, &up->port);
}

static int nuc980serial_pinctrl(struct platform_device *pdev)
{
	struct pinctrl *p = NULL;
	int retval = 0;

#ifdef CONFIG_USE_OF
	p = devm_pinctrl_get_select_default(&pdev->dev);
#else
	if(pdev->id == 0) {
#	if defined (CONFIG_NUC980_SCUART0_PA)
		p = devm_pinctrl_get_select(&pdev->dev, "scuart0-PA");
#	elif defined (CONFIG_NUC980_SCUART0_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "scuart0-PC");
#	endif
	} else { // if(pdev->id == 1)
#	if defined (CONFIG_NUC980_SCUART1_PC)
		p = devm_pinctrl_get_select(&pdev->dev, "scuart1-PC");
#	elif defined (CONFIG_NUC980_SCUART1_PF)
		p = devm_pinctrl_get_select(&pdev->dev, "scuart1-PF");
#	endif
	}
#endif
	if (IS_ERR(p)) {
		dev_err(&pdev->dev, "Unable to reserve SC%d pin", pdev->id);
		retval = PTR_ERR(p);
	}

	return retval;
}

static void nuc980serial_set_clock(int id)
{
	struct clk *clk;

	if(id == 0) {
		clk = clk_get(NULL, "smc0");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "smc0_eclk");
		clk_prepare(clk);
		clk_enable(clk);
	} else {
		clk = clk_get(NULL, "smc1");
		clk_prepare(clk);
		clk_enable(clk);

		clk = clk_get(NULL, "smc1_eclk");
		clk_prepare(clk);
		clk_enable(clk);
	}

}

static const struct of_device_id nuc980_sc_of_match[] = {
	{ .compatible = "nuvoton,nuc980-scuart" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_sc_of_match);

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int nuc980serial_probe(struct platform_device *pdev)
{
#ifndef CONFIG_USE_OF
	struct plat_nuc980serial_port *p = pdev->dev.platform_data;
#endif
	struct uart_nuc980_port *up;
	int retval;

#ifdef CONFIG_USE_OF
	int i;

	if(!of_match_device(nuc980_sc_of_match, &pdev->dev)) {
		dev_err(&pdev->dev, "Failed to find matching device\n");
		return -EINVAL;
	}
	of_property_read_u32_array(pdev->dev.of_node, "port-number", &i, 1);
	pdev->id = i;
#endif
	retval = nuc980serial_pinctrl(pdev);
	if(retval != 0)
		return retval;

	nuc980serial_set_clock(pdev->id);

	up = &nuc980serial_ports[pdev->id];
	up->port.line 		= pdev->id;
#ifdef CONFIG_USE_OF
	of_property_read_u32_array(pdev->dev.of_node, "reg", &i, 1);
	up->port.iobase 	= (long)i;
	up->port.membase      	= (void *)i;
	up->port.irq = platform_get_irq(pdev, 0);
	up->port.uartclk 	= 12000000;
	of_property_read_u32_array(pdev->dev.of_node, "map-addr", &up->port.mapbase, 1);
#else
	up->port.iobase       	= (long)p->membase;
	up->port.membase      	= p->membase;
	up->port.irq          	= p->irq;
	up->port.uartclk      	= p->uartclk;
	up->port.mapbase     	= p->mapbase;
	//up->port.private_data 	= p->private_data;
#endif
	up->port.dev 		= &pdev->dev;

	up->port.flags 		= ASYNC_BOOT_AUTOCONF;
	up->port.ops = &nuc980serial_ops;

	spin_lock_init(&up->port.lock);


	return uart_add_one_port(&nuc980serial_reg, &up->port);
}

/*
 * Remove serial ports registered against a platform device.
 */
static int nuc980serial_remove(struct platform_device *pdev)
{
	struct uart_nuc980_port *up = &nuc980serial_ports[pdev->id];

	uart_remove_one_port(&nuc980serial_reg, &up->port);
	return 0;
}

static int nuc980serial_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct uart_nuc980_port *up = &nuc980serial_ports[pdev->id];

	uart_suspend_port(&nuc980serial_reg, &up->port);
	return 0;
}

static int nuc980serial_resume(struct platform_device *pdev)
{
	struct uart_nuc980_port *up = &nuc980serial_ports[pdev->id];

	uart_resume_port(&nuc980serial_reg, &up->port);
	return 0;
}


static struct platform_driver nuc980serial_driver = {
	.probe		= nuc980serial_probe,
	.remove		= nuc980serial_remove,
	.suspend	= nuc980serial_suspend,
	.resume		= nuc980serial_resume,
	.driver		= {
#ifdef CONFIG_USE_OF
		.name	= "nuc980-scuart",
#else
		.name	= "nuc980-sc",		// share same dev structure with smartcard
#endif
		.of_match_table = of_match_ptr(nuc980_sc_of_match),
		.owner	= THIS_MODULE,
	},
};

static int __init nuc980serial_init(void)
{
	int ret;

	ret = uart_register_driver(&nuc980serial_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&nuc980serial_driver);
	if (ret)
		uart_unregister_driver(&nuc980serial_reg);

	nuc980serial_init_ports();

	return ret;
}

static void __exit nuc980serial_exit(void)
{
	platform_driver_unregister(&nuc980serial_driver);
	uart_unregister_driver(&nuc980serial_reg);
}

module_init(nuc980serial_init);
module_exit(nuc980serial_exit);

EXPORT_SYMBOL(nuc980scuart_suspend_port);
EXPORT_SYMBOL(nuc980scuart_resume_port);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NUC980 scuart driver");

MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
