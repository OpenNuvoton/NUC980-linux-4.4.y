/*
 * linux/drivers/i2c/busses/i2c-nuc980-p0.c
 *
 * Copyright (c) 2014 Nuvoton technology corporation.
 *
 * This driver based on S3C2410 I2C driver of Ben Dooks <ben-Y5A6D6n0/KfQXOPxS62xeg@public.gmane.org>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/io.h>
//#include <linux/of_i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/pm_runtime.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>

#include <mach/mfp.h>
#include <linux/platform_data/i2c-nuc980.h>

/* nuc980 i2c registers offset */

#define CTL0        0x00
#define ADDR0       0x04
#define DAT         0x08
#define STATUS0     0x0C
#define CLKDIV      0x10
#define TOCTL       0x14
#define ADDR1       0x18
#define ADDR2       0x1C
#define ADDR3       0x20
#define ADDRMSK0    0x24
#define ADDRMSK1    0x28
#define ADDRMSK2    0x2C
#define ADDRMSK3    0x30
#define WKCTL       0x3C
#define WKSTS       0x40
#define CTL1        0x44
#define STATUS1     0x48
#define TMCTL       0x4C
#define BUSCTL      0x50
#define BUSTCTL     0x54
#define BUSSTS      0x58
#define PKTSIZE     0x5C
#define PKTCRC      0x60
#define BUSTOUT     0x64
#define CLKTOUT     0x68

/* nuc980 i2c Status */
// Master
#define  M_START                 0x08  //Start
#define  M_REPEAT_START          0x10  //Master Repeat Start
#define  M_TRAN_ADDR_ACK         0x18  //Master Transmit Address ACK
#define  M_TRAN_ADDR_NACK        0x20  //Master Transmit Address NACK
#define  M_TRAN_DATA_ACK         0x28  //Master Transmit Data ACK
#define  M_TRAN_DATA_NACK        0x30  //Master Transmit Data NACK
#define  M_ARB_LOST              0x38  //Master Arbitration Los
#define  M_RECE_ADDR_ACK         0x40  //Master Receive Address ACK
#define  M_RECE_ADDR_NACK        0x48  //Master Receive Address NACK
#define  M_RECE_DATA_ACK         0x50  //Master Receive Data ACK
#define  M_RECE_DATA_NACK        0x58  //Master Receive Data NACK
#define  BUS_ERROR               0x00  //Bus error

// Slave
#define  S_REPEAT_START_STOP     0xA0  //Slave Transmit Repeat Start or Stop
#define  S_TRAN_ADDR_ACK         0xA8  //Slave Transmit Address ACK
#define  S_TRAN_DATA_ACK         0xB8  //Slave Transmit Data ACK
#define  S_TRAN_DATA_NACK        0xC0  //Slave Transmit Data NACK
#define  S_TRAN_LAST_DATA_ACK    0xC8  //Slave Transmit Last Data ACK
#define  S_RECE_ADDR_ACK         0x60  //Slave Receive Address ACK
#define  S_RECE_ARB_LOST         0x68  //Slave Receive Arbitration Lost
#define  S_RECE_DATA_ACK         0x80  //Slave Receive Data ACK
#define  S_RECE_DATA_NACK        0x88  //Slave Receive Data NACK

//GC Mode
#define  GC_ADDR_ACK             0x70  //GC mode Address ACK
#define  GC_ARB_LOST             0x78  //GC mode Arbitration Lost
#define  GC_DATA_ACK             0x90  //GC mode Data ACK
#define  GC_DATA_NACK            0x98  //GC mode Data NACK

//Other
#define  ADDR_TRAN_ARB_LOST      0xB0  //Address Transmit Arbitration Lost
#define  BUS_RELEASED            0xF8  //Bus Released


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C_CTL constant definitions.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define I2C_CTL_STA_SI            0x28UL /* I2C_CTL setting for I2C control bits. It would set STA and SI bits       */
#define I2C_CTL_STA_SI_AA         0x2CUL /* I2C_CTL setting for I2C control bits. It would set STA, SI and AA bits   */
#define I2C_CTL_STO_SI            0x18UL /* I2C_CTL setting for I2C control bits. It would set STO and SI bits       */
#define I2C_CTL_STO_SI_AA         0x1CUL /* I2C_CTL setting for I2C control bits. It would set STO, SI and AA bits   */
#define I2C_CTL_SI                0x08UL /* I2C_CTL setting for I2C control bits. It would set SI bit                */
#define I2C_CTL_SI_AA             0x0CUL /* I2C_CTL setting for I2C control bits. It would set SI and AA bits        */
#define I2C_CTL_STA               0x20UL /* I2C_CTL setting for I2C control bits. It would set STA bit               */
#define I2C_CTL_STO               0x10UL /* I2C_CTL setting for I2C control bits. It would set STO bit               */
#define I2C_CTL_AA                0x04UL /* I2C_CTL setting for I2C control bits. It would set AA bit                */

#define I2C_GCMODE_ENABLE   1    /*!< Enable I2C GC Mode  \hideinitializer */
#define I2C_GCMODE_DISABLE  0    /*!< Disable I2C GC Mode  \hideinitializer */

/* i2c controller private data */

struct nuc980_i2c {
	spinlock_t      lock;
	wait_queue_head_t   wait;

	struct i2c_msg      *msg;
	unsigned int        msg_num;
	unsigned int        msg_idx;
	unsigned int        msg_ptr;
	unsigned int        irq;
	unsigned int        arblost;

	void __iomem        *regs;
	struct clk      *clk;
	struct device       *dev;
	struct resource     *ioarea;
	struct i2c_adapter  adap;

	struct i2c_client *slave;
};

/* nuc980_i2c0_master_complete
 *
 * complete the message and wake up the caller, using the given return code,
 * or zero to mean ok.
*/

static inline void nuc980_i2c0_master_complete(struct nuc980_i2c *i2c, int ret)
{
	dev_dbg(i2c->dev, "master_complete %d\n", ret);

	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;

	wake_up(&i2c->wait);
}

/* irq enable/disable functions */

static inline void nuc980_i2c0_disable_irq(struct nuc980_i2c *i2c)
{
	unsigned long tmp;

	tmp = readl(i2c->regs + CTL0);
	writel(tmp & ~(0x1 << 7), i2c->regs + CTL0);
}

static inline void nuc980_i2c0_enable_irq(struct nuc980_i2c *i2c)
{
	unsigned long tmp;

	tmp = readl(i2c->regs + CTL0);
	writel(tmp | (0x1 << 7), i2c->regs + CTL0);
}


/* nuc980_i2c0_message_start
 *
 * put the start of a message onto the bus
*/

static void nuc980_i2c0_message_start(struct nuc980_i2c *i2c)
{
	writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_STA), i2c->regs + CTL0);
}

static inline void nuc980_i2c0_stop(struct nuc980_i2c *i2c, int ret)
{
	dev_dbg(i2c->dev, "STOP\n");

	writel(((readl(i2c->regs+CTL0) &~ (0x3C))|(I2C_CTL_STO | I2C_CTL_SI | I2C_CTL_AA)), (i2c->regs+CTL0));
	while(readl(i2c->regs+CTL0) & I2C_CTL_STO);

	nuc980_i2c0_master_complete(i2c, ret);
}

/* helper functions to determine the current state in the set of
 * messages we are sending
*/

/* is_lastmsg()
 *
 * returns TRUE if the current message is the last in the set
*/

static inline int is_lastmsg(struct nuc980_i2c *i2c)
{
	return i2c->msg_idx >= (i2c->msg_num - 1);
}

/* is_msglast
 *
 * returns TRUE if we this is the last byte in the current message
*/

static inline int is_msglast(struct nuc980_i2c *i2c)
{
	return i2c->msg_ptr == i2c->msg->len-1;
}

/* is_msgend
 *
 * returns TRUE if we reached the end of the current message
*/

static inline int is_msgend(struct nuc980_i2c *i2c)
{
	return i2c->msg_ptr >= i2c->msg->len;
}



#if defined(CONFIG_ENABLE_I2C_P0_SLAVE_MODE)
static void I2C_SlaveTRx(struct nuc980_i2c *i2c, unsigned long iicstat)
{
	unsigned char byte;

	if (iicstat == S_RECE_ADDR_ACK) {  /* Own SLA+W has been receive; ACK has been return */

		writel(((readl(i2c->regs+CTL0) &~ (0x3C)) | (I2C_CTL_SI | I2C_CTL_AA)), (i2c->regs+CTL0));
	} else if (iicstat == S_RECE_DATA_ACK)  /* Previously address with own SLA address
	                                           Data has been received; ACK has been returned*/
	{
		byte = readb(i2c->regs + DAT);

		i2c_slave_event(i2c->slave, I2C_SLAVE_WRITE_RECEIVED, &byte);

		writel(((readl(i2c->regs+CTL0) &~ (0x3C))|(I2C_CTL_SI | I2C_CTL_AA)), (i2c->regs+CTL0));
	} else if(iicstat == S_TRAN_ADDR_ACK) { /* Own SLA+R has been receive; ACK has been return */

		i2c_slave_event(i2c->slave, I2C_SLAVE_READ_PROCESSED, &byte);    // I2C_SLAVE_READ_PROCESSED:
		//i2c_slave_event(i2c->slave, I2C_SLAVE_REQ_READ_START, &byte);

		writel(byte, i2c->regs+DAT);

		writel(((readl(i2c->regs+CTL0) &~ (0x3C))|(I2C_CTL_SI | I2C_CTL_AA)), (i2c->regs+CTL0));
	} else if (iicstat == S_TRAN_DATA_NACK)     /* Data byte or last data in I2CDAT has been transmitted
	                                               Not ACK has been received */
	{
		writel(((readl(i2c->regs+CTL0) &~ (0x3C))|(I2C_CTL_SI | I2C_CTL_AA)), (i2c->regs+CTL0));
	} else if (iicstat == S_RECE_DATA_NACK)     /* Previously addressed with own SLA address; NOT ACK has
	                                               been returned */
	{
		writel(((readl(i2c->regs+CTL0) &~ (0x3C))|(I2C_CTL_SI | I2C_CTL_AA)), (i2c->regs+CTL0));
	} else if (iicstat == S_REPEAT_START_STOP)  /* A STOP or repeated START has been received while still
	                                               addressed as Slave/Receiver*/
	{
		i2c_slave_event(i2c->slave, I2C_SLAVE_STOP, &byte);

		writel(((readl(i2c->regs+CTL0) &~ (0x3C))|(I2C_CTL_SI | I2C_CTL_AA)), (i2c->regs+CTL0));
	} else {
		/* TO DO */
		//printk("Status 0x%x is NOT processed\n", iicstat);
	}
}
#else
static void i2c_nuc980_irq_master_TRx(struct nuc980_i2c *i2c, unsigned long iicstat)
{
	unsigned char byte;

	if (iicstat == M_START)
	{ /* START has been transmitted and prepare SLA+W */

		if (i2c->msg->flags & I2C_M_RD)
			writel( (((i2c->msg->addr & 0x7f) << 1)|0x1), (i2c->regs+DAT)); /* Write SLA+R to Register I2CDAT */
		else
			writel( ((i2c->msg->addr & 0x7f) << 1), (i2c->regs+DAT)); /* Write SLA+W to Register I2CDAT */

		writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_SI), (i2c->regs+CTL0));
	}
	else if ((iicstat == M_TRAN_ADDR_ACK) || (iicstat == M_TRAN_DATA_ACK))
	{ /* SLA+W has been transmitted and ACK has been received */
		//I2C_SET_DATA(I2C0, g_au8I2C_MasterTxData[g_u8I2C_MasterTxDataCnt++]);
		//I2C_SET_CONTROL_REG(I2C0, I2C_SI);

		if(iicstat == M_TRAN_ADDR_ACK)
		{
			if (is_lastmsg(i2c) && i2c->msg->len == 0)
			{
				nuc980_i2c0_stop(i2c, 0);
				return;
			}
		}

		if (!is_msgend(i2c))
		{
			byte = i2c->msg->buf[i2c->msg_ptr++];
			writel(byte, i2c->regs+DAT);
			writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_SI), (i2c->regs+CTL0));
		}
		else if (!is_lastmsg(i2c))
		{ /* we need to go to the next i2c message */
			dev_dbg(i2c->dev, "WRITE: Next Message\n");

			i2c->msg_ptr = 0;
			i2c->msg_idx++;
			i2c->msg++;

			/* send the new start */
			writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_STA|I2C_CTL_SI), (i2c->regs+CTL0));
		}
		else
		{ /* send stop */
			nuc980_i2c0_stop(i2c, 0);
		}
	}
	else if ((iicstat == M_TRAN_ADDR_NACK) || (iicstat == M_RECE_ADDR_NACK))
	{	/* Master Transmit Address NACK */
		/* 0x20: SLA+W has been transmitted and NACK has been received */
		/* 0x48: SLA+R has been transmitted and NACK has been received */
		//I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA | I2C_CTL_STO | I2C_CTL_SI);

		if (!(i2c->msg->flags & I2C_M_IGNORE_NAK))
		{
			printk("\n i2c_0: ack was not received\n");
			//writel((I2C_CTL_STA | I2C_CTL_STO | I2C_CTL_SI), (i2c->regs+CTL0));
			nuc980_i2c0_stop(i2c, -ENXIO);
		}
	}
	else if (iicstat == M_REPEAT_START)
	{  /* Repeat START has been transmitted and prepare SLA+R */

		if (i2c->msg->flags & I2C_M_RD)
			writel( (((i2c->msg->addr & 0x7f) << 1)|0x1), (i2c->regs+DAT)); /* Write SLA+R to Register I2CDAT */
		else
			writel( ((i2c->msg->addr & 0x7f) << 1), (i2c->regs+DAT)); /* Write SLA+W to Register I2CDAT */

		writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_SI), (i2c->regs+CTL0));
	}
	else if (iicstat == M_RECE_ADDR_ACK)
	{  /* SLA+R has been transmitted and ACK has been received */
		//I2C_SET_CONTROL_REG(I2C0, I2C_AA | I2C_SI);

		if (is_lastmsg(i2c) && i2c->msg->len == 0)
		{
			nuc980_i2c0_stop(i2c, 0);
			//return;
		}
		else
			writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_SI), (i2c->regs+CTL0));
	}
	else if ((iicstat == M_RECE_DATA_ACK) || (iicstat == M_RECE_DATA_NACK))
	{ /* DATA has been transmitted and ACK has been received */
		byte = readb(i2c->regs + DAT);
		i2c->msg->buf[i2c->msg_ptr++] = byte;

		if (is_msglast(i2c))
		{ /* last byte of buffer */

			writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_SI), (i2c->regs+CTL0));

			//if (is_lastmsg(i2c))
			//	writel((readl(i2c->regs+CTL0)|I2C_CTL_SI), (i2c->regs+CTL0));
		}
		else if (is_msgend(i2c))
		{	/* ok, we've read the entire buffer, see if there
			 * is anything else we need to do
			 */

			if (is_lastmsg(i2c))
			{ /* last message, send stop and complete */
				dev_dbg(i2c->dev, "READ: Send Stop\n");

				nuc980_i2c0_stop(i2c, 0);
			}
			else
			{ /* go to the next transfer */
				dev_dbg(i2c->dev, "READ: Next Transfer\n");

				i2c->msg_ptr = 0;
				i2c->msg_idx++;
				i2c->msg++;

				/* send the new start */
				writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_STA|I2C_CTL_SI), (i2c->regs+CTL0));
			}
		}
		else
		{
			writel(((readl(i2c->regs+CTL0) &~ (0x3C))|I2C_CTL_SI_AA), (i2c->regs+CTL0));
		}

	}
	else
	{
		/* TO DO */
		//printf("Status 0x%x is NOT processed\n", u32Status);
	}


}
#endif

/* nuc980_i2c_irq
 *
 * top level IRQ servicing routine
*/

static irqreturn_t nuc980_i2c_irq(int irqno, void *dev_id)
{
	struct nuc980_i2c *i2c = dev_id;
	unsigned long status;

	status = readl(i2c->regs + STATUS0);

	if (status == M_ARB_LOST) {
		/* deal with arbitration loss */
		dev_err(i2c->dev, "deal with arbitration loss\n");
		i2c->arblost = 1;
		goto out;
	}

	if (status == BUS_ERROR) {
		dev_dbg(i2c->dev, "IRQ: error i2c->state == IDLE\n");
		goto out;
	}

	/* pretty much this leaves us with the fact that we've
	 * transmitted or received whatever byte we last sent
	*/

	#if defined(CONFIG_ENABLE_I2C_P0_SLAVE_MODE)
	I2C_SlaveTRx(i2c, status);
	#else
	i2c_nuc980_irq_master_TRx(i2c, status);
	#endif

out:
	return IRQ_HANDLED;
}

#if 0
/* nuc980_i2c0_hangup
 *
 * send out some dummy clocks to let SDA free
*/
static void nuc980_i2c0_hangup(struct nuc980_i2c *i2c)
{
	int i;

	for(i=0;i<2;i++) {
		writel(0x6, i2c->regs + SWR);       //CLK Low
		ndelay(2);
		writel(0x7, i2c->regs + SWR);       //CLK High
		ndelay(2);
	}
}
#endif

/* nuc980_i2c0_doxfer
 *
 * this starts an i2c transfer
*/

static int nuc980_i2c0_doxfer(struct nuc980_i2c *i2c,
				  struct i2c_msg *msgs, int num)
{
	unsigned long iicstat, timeout;
	int spins = 20;
	int ret;

	spin_lock_irq(&i2c->lock);

	i2c->msg     = msgs;
	i2c->msg_num = num;
	i2c->msg_ptr = 0;
	i2c->msg_idx = 0;

	nuc980_i2c0_message_start(i2c);
	spin_unlock_irq(&i2c->lock);

	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 5);
	ret = i2c->msg_idx;

	/* having these next two as dev_err() makes life very
	 * noisy when doing an i2cdetect
	*/

	if (timeout == 0)
		dev_dbg(i2c->dev, "timeout\n");
	else if (ret != num)
		dev_dbg(i2c->dev, "incomplete xfer (%d)\n", ret);

	/* ensure the stop has been through the bus */
	dev_dbg(i2c->dev, "waiting for bus idle\n");

	/* first, try busy waiting briefly */
	do
	{
		// chekc stop bit auto clear
		iicstat = readl(i2c->regs + CTL0);
	} while ((iicstat & (0x1<<4)) && --spins);

	/* if that timed out sleep */
	if (!spins) {
		msleep(1);
		iicstat = readl(i2c->regs + CTL0);
	}

	if (iicstat & (0x1<<4))
		dev_warn(i2c->dev, "timeout waiting for bus idle\n");

	if(i2c->arblost) {
		dev_dbg(i2c->dev, "arb lost, stop\n");
		i2c->arblost = 0;
		nuc980_i2c0_stop(i2c, 0);
		msleep(1);
		nuc980_i2c0_disable_irq(i2c);
		//nuc980_i2c0_hangup(i2c);
		ret = -EAGAIN;
	}

// out:
	return ret;
}

/* nuc980_i2c0_xfer
 *
 * first port of call from the i2c bus code when an message needs
 * transferring across the i2c bus.
*/

static int nuc980_i2c0_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct nuc980_i2c *i2c = (struct nuc980_i2c *)adap->algo_data;
	int retry;
	int ret;

	nuc980_i2c0_enable_irq(i2c);

	for (retry = 0; retry < adap->retries; retry++) {

		ret = nuc980_i2c0_doxfer(i2c, msgs, num);

		if (ret != -EAGAIN)
			return ret;

		dev_dbg(i2c->dev, "Retrying transmission (%d)\n", retry);

		udelay(100);
	}

	return -EREMOTEIO;
}

#if defined(CONFIG_ENABLE_I2C_P0_SLAVE_MODE)
static int nuc980_reg_slave(struct i2c_client *slave)
{
	struct nuc980_i2c *priv = i2c_get_adapdata(slave->adapter);

	if (priv->slave)
		return -EBUSY;

	if (slave->flags & I2C_CLIENT_TEN)
		return -EAFNOSUPPORT;

	nuc980_i2c0_enable_irq(priv);

	pm_runtime_forbid(priv->dev);

	priv->slave = slave;

	// Enable I2C
	writel(readl(priv->regs + CTL0) | (0x1 << 6), (priv->regs + CTL0)); // CTL0

	// Set Slave Address
	writel(slave->addr, (priv->regs + ADDR0));

	// I2C enter SLV mode
	writel((readl(priv->regs+CTL0)|I2C_CTL_AA|I2C_CTL_SI), (priv->regs+CTL0));

	return 0;
}

static int nuc980_unreg_slave(struct i2c_client *slave)
{
	struct nuc980_i2c *priv = i2c_get_adapdata(slave->adapter);

	// Disable I2C
	writel(readl(priv->regs + CTL0) &~ (0x1 << 6), (priv->regs + CTL0)); // CTL0
	// Disable i2c interrupt
	nuc980_i2c0_disable_irq(priv);

	priv->slave = NULL;

	pm_runtime_allow(priv->dev);

	return 0;
}
#endif

/* declare our i2c functionality */
static u32 nuc980_i2c0_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_PROTOCOL_MANGLING | I2C_FUNC_SMBUS_EMUL ;
}

/* i2c bus registration info */

static const struct i2c_algorithm nuc980_i2c0_algorithm = {
	.master_xfer        = nuc980_i2c0_xfer,
	.functionality      = nuc980_i2c0_func,
#if defined(CONFIG_ENABLE_I2C_P0_SLAVE_MODE)
	.reg_slave	= nuc980_reg_slave,
	.unreg_slave	= nuc980_unreg_slave,
#endif
};

/* nuc980_i2c0_probe
 *
 * called by the bus driver when a suitable device is found
*/

static int nuc980_i2c0_probe(struct platform_device *pdev)
{
	struct nuc980_i2c *i2c;
	struct nuc980_platform_i2c *pdata=NULL;
	struct resource *res;
	struct i2c_adapter *adap;
	int ret;
	int busnum = 0, busfreq = 0;
	struct device *dev = &pdev->dev;

	struct pinctrl *pinctrl;

	if (!pdev->dev.of_node) {
		pdata = pdev->dev.platform_data;
		if (!pdata) {
			dev_err(&pdev->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	i2c = kzalloc(sizeof(struct nuc980_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	strlcpy(i2c->adap.name, "nuc980-i2c0", sizeof(i2c->adap.name));
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &nuc980_i2c0_algorithm;
	i2c->adap.retries = 2;
	i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;

	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	/* find the clock and enable it */

	i2c->dev = &pdev->dev;
	i2c->clk = clk_get(NULL, "i2c0");
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = -ENOENT;
		goto err_noclk;
	}

	dev_dbg(&pdev->dev, "clock source %p\n", i2c->clk);

	clk_prepare(i2c->clk);
	clk_enable(i2c->clk);

	/* map the registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err_clk;
	}

#if defined(CONFIG_USE_OF)
	i2c->regs = devm_ioremap_resource(&pdev->dev, res);
#else
	i2c->ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
	if (i2c->ioarea == NULL) {
		dev_err(&pdev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto err_clk;
	}

	i2c->regs = ioremap(res->start, resource_size(res));
	if (i2c->regs == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}

	dev_dbg(&pdev->dev, "registers %p (%p, %p)\n", i2c->regs, i2c->ioarea, res);
#endif

	/* setup info block for the i2c core */

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	if (pdata) {
		busfreq = pdata->bus_freq;
		busnum = pdata->bus_num;
	} else {
		of_property_read_u32(pdev->dev.of_node, "bus_freq", &busfreq);
		of_property_read_u32(pdev->dev.of_node, "bus_num", &busnum);
	}

	// Set Clock divider
	ret = clk_get_rate(i2c->clk)/(busfreq * 4) - 1;
	writel(ret & 0xffff, i2c->regs + CLKDIV);

	writel((readl(i2c->regs+CTL0)|(0x1 << 6)), i2c->regs + CTL0);

	//printk("\n i2c busfreq = %d, CLKDIV = 0x%x, i2c_clk = %d \n", busfreq, ret, clk_get_rate(i2c->clk));

	/* find the IRQ for this unit (note, this relies on the init call to
	 * ensure no current IRQs pending
	 */

	i2c->irq = ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	ret = request_irq(i2c->irq, nuc980_i2c_irq, IRQF_SHARED, dev_name(&pdev->dev), i2c);

	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto err_iomap;
	}

	/* Note, previous versions of the driver used i2c_add_adapter()
	 * to add the bus at any number. We now pass the bus number via
	 * the platform data, so if unset it will now default to always
	 * being bus 0.
	 */

	adap = &i2c->adap;

	i2c->adap.nr = busnum;
	i2c->adap.dev.of_node = pdev->dev.of_node;

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_irq;
	}
	//of_i2c_register_devices(&i2c->adap);

	i2c_set_adapdata(adap, i2c);
	strlcpy(adap->name, pdev->name, sizeof(adap->name));

	pm_runtime_enable(dev);

	platform_set_drvdata(pdev, i2c);

	dev_info(&pdev->dev, "%s: nuc980 I2C adapter\n", dev_name(&i2c->adap.dev));

#if defined(CONFIG_USE_OF)
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
#else
 #ifdef CONFIG_NUC980_I2C0_PA
	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2c0-PA");
 #elif defined(CONFIG_NUC980_I2C0_PA_PG)
	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2c0-PA_PG");
 #elif defined(CONFIG_NUC980_I2C0_PE)
	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2c0-PE");
 #endif
#endif

	if (IS_ERR(pinctrl)) {
		return PTR_ERR(pinctrl);
	}

	return 0;

err_irq:
	free_irq(i2c->irq, i2c);

err_iomap:
	iounmap(i2c->regs);

#ifndef CONFIG_USE_OF
err_ioarea:
	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);
#endif

err_clk:
	clk_disable(i2c->clk);
	clk_put(i2c->clk);

err_noclk:
	kfree(i2c);
	return ret;
}

/* nuc980_i2c0_remove
 *
 * called when device is removed from the bus
*/

static int nuc980_i2c0_remove(struct platform_device *pdev)
{
	struct nuc980_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);
	free_irq(i2c->irq, i2c);

	clk_disable(i2c->clk);
	clk_put(i2c->clk);

	iounmap(i2c->regs);

	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);
	kfree(i2c);

	return 0;
}

#ifdef CONFIG_PM
static int nuc980_i2c0_suspend(struct device *dev)
{
#ifdef CONFIG_SLAVE_WAKEUP
	struct nuc980_i2c *i2c = dev_get_drvdata(dev);

	__raw_writel((1<<0) | __raw_readl(REG_WKUPSER1),REG_WKUPSER1);

	nuc980_i2c0_enable_irq(i2c);

	writel(I2C_CTL_SI_AA | readl(i2c->regs + CTL0), i2c->regs + CTL0);
	writel(0x1, i2c->regs + WKCTL);
	writel(readl(i2c->regs + WKSTS), i2c->regs + WKSTS);

	enable_irq_wake(i2c->irq);
#endif

	return 0;
}

static int nuc980_i2c0_resume(struct device *dev)
{
#ifdef CONFIG_SLAVE_WAKEUP
	struct nuc980_i2c *i2c = dev_get_drvdata(dev);

	writel(0x0, i2c->regs + WKCTL);
	writel(readl(i2c->regs + WKSTS), i2c->regs + WKSTS);
#endif
	return 0;
}

static const struct dev_pm_ops nuc980_i2c0_pmops = {
	.suspend    = nuc980_i2c0_suspend,
	.resume     = nuc980_i2c0_resume,
};

#define NUC980_I2C0_PMOPS (&nuc980_i2c0_pmops)

#else
#define NUC980_I2C0_PMOPS NULL
#endif

#if defined(CONFIG_USE_OF)
static const struct of_device_id nuc980_i2c0_of_match[] = {
	{   .compatible = "nuvoton,nuc980-i2c0" },
	{	},
};
MODULE_DEVICE_TABLE(of, nuc980_i2c0_of_match);
#endif

static struct platform_driver nuc980_i2c0_driver = {
	.probe      = nuc980_i2c0_probe,
	.remove     = nuc980_i2c0_remove,
	.driver     = {
		.name   = "nuc980-i2c0",
		.owner  = THIS_MODULE,
#if defined(CONFIG_USE_OF)
		.of_match_table = of_match_ptr(nuc980_i2c0_of_match),
#endif
		.pm = NUC980_I2C0_PMOPS,
	},
};
module_platform_driver(nuc980_i2c0_driver);

MODULE_DESCRIPTION("nuc980 I2C Bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc980-i2c0");
