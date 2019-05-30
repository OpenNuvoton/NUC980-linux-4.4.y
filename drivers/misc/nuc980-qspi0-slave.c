/*
 * Copyright (c) 2018 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <mach/mfp.h>
#include <linux/platform_data/spi-nuc980.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/regs-gcr.h>

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <mach/regs-pdma.h>

#include <linux/platform_data/dma-nuc980.h>

/* spi registers offset */
#define REG_CTL		0x00
#define REG_CLKDIV	0x04
#define REG_SSCTL	0x08
#define REG_PDMACTL	0x0C
#define REG_FIFOCTL	0x10
#define REG_STATUS	0x14
#define REG_TX		0x20
#define REG_RX		0x30

/* spi register bit */
#define UNITIEN		(0x01 << 17)
#define TXNEG		(0x01 << 2)
#define RXNEG		(0x01 << 1)
#define SLAVE		(0x01 << 18)
#define LSB		(0x01 << 13)
#define SELECTLEV	(0x01 << 2)
#define SELECTPOL	(0x01 << 3)
#define SSACTIEN	(0x01 << 12)
#define SELECTSLAVE0	0x01
#define SELECTSLAVE1	0x02
#define SPIEN		0x01
#define TXTHIEN		(0x01 << 3)
#define RXTHIEN		(0x01 << 2)
#define SSINAIEN	(0x01 << 13)
#define SSACTIF		(0x01 << 2)
#define SSINAIF		(0x01 << 3)
#define RXTHIF		(0x01 << 10)
#define TXTHIF		(0x01 << 18)
#define TXUFIF		(0x01 << 19)

#ifdef CONFIG_SPI_NUC980_QSPI0
# error Do not enable CONFIG_SPI_NUC980_QSPI0 and CONFIG_NUC980_QSPI0_SLAVE at the same time!
#endif

static volatile int slave_done_state=0;
static DECLARE_WAIT_QUEUE_HEAD(slave_done);

static int QSPI0_SlaveDataLen = 256;
static int QSPI0_SlaveData[256];
static int TransmittedCnt = 0;
static int InTransmitted = 0;

struct nuc980_spi {
	struct spi_bitbang	bitbang;
	struct completion	done;
	void __iomem		*regs;
	int			irq;
	unsigned int 		len;
	unsigned int		count;
	const void		*tx;
	void			*rx;
	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct nuc980_spi_info *pdata;
	spinlock_t		lock;
	struct resource		*res;
};


static inline struct nuc980_qspi0 *to_hw(struct spi_device *sdev) {
	return spi_master_get_devdata(sdev->master);
}

static inline void nuc980_slave_select(struct spi_device *spi, unsigned int ssr)
{
	struct nuc980_spi *hw = (struct nuc980_spi *)to_hw(spi);
	unsigned int val;
	unsigned int cs = spi->mode & SPI_CS_HIGH ? 1 : 0;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_SSCTL);

	if (!cs)
		val &= ~SELECTLEV;
	else
		val |= SELECTLEV;

	if(spi->chip_select == 0) {
		if (!ssr)
			val &= ~SELECTSLAVE0;
		else
			val |= SELECTSLAVE0;
	} else {
		if (!ssr)
			val &= ~SELECTSLAVE1;
		else
			val |= SELECTSLAVE1;
	}

	__raw_writel(val, hw->regs + REG_SSCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_qspi0_chipsel(struct spi_device *spi, int value)
{
	switch (value) {
	case BITBANG_CS_INACTIVE:
		nuc980_slave_select(spi, 0);
		break;

	case BITBANG_CS_ACTIVE:
		nuc980_slave_select(spi, 1);
		break;
	}
}

static inline void nuc980_qspi0_setup_txbitlen(struct nuc980_spi *hw,
                unsigned int txbitlen)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);
	val &= ~0x1f00;
	if(txbitlen != 32)
		val |= (txbitlen << 8);

	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_set_clock_polarity(struct nuc980_spi *hw, unsigned int polarity)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);

	if (polarity)
		val |= SELECTPOL;
	else
		val &= ~SELECTPOL;
	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_tx_edge(struct nuc980_spi *hw, unsigned int edge)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);

	if (edge)
		val |= TXNEG;
	else
		val &= ~TXNEG;
	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_rx_edge(struct nuc980_spi *hw, unsigned int edge)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);

	if (edge)
		val |= RXNEG;
	else
		val &= ~RXNEG;
	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_send_first(struct nuc980_spi *hw, unsigned int lsb)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);

	if (lsb)
		val |= LSB;
	else
		val &= ~LSB;
	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_enable_slave(struct nuc980_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);

	val |= SLAVE;

	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_set_sleep(struct nuc980_spi *hw, unsigned int sleep)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);

	val &= ~(0x0f << 4);

	if (sleep)
		val |= (sleep << 4);

	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}


static inline void nuc980_set_divider(struct nuc980_spi *hw)
{
	__raw_writel(hw->pdata->divider, hw->regs + REG_CLKDIV);
}

static int nuc980_qspi0_update_state(struct spi_device *spi,
                                     struct spi_transfer *t)
{
	struct nuc980_spi *hw = (struct nuc980_spi *)to_hw(spi);
	unsigned int clk;
	unsigned int div;
	unsigned int bpw;
	unsigned int hz;
	unsigned char spimode;

	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if(hw->pdata->txbitlen != bpw)
		hw->pdata->txbitlen = bpw;

	if(hw->pdata->hz != hz) {
		clk = clk_get_rate(hw->clk);
		div = DIV_ROUND_UP(clk, hz) - 1;
		hw->pdata->hz = hz;
		hw->pdata->divider = div;
	}

	//Mode 0: CPOL=0, CPHA=0; active high
	//Mode 1: CPOL=0, CPHA=1 ;active low
	//Mode 2: CPOL=1, CPHA=0 ;active low
	//Mode 3: POL=1, CPHA=1;active high
	if (spi->mode & SPI_CPOL)
		hw->pdata->clkpol = 1;
	else
		hw->pdata->clkpol = 0;

	spimode = spi->mode & 0xff; //remove dual/quad bit

	if ((spimode == SPI_MODE_0) || (spimode == SPI_MODE_2)) {
		hw->pdata->txneg = 1;
		hw->pdata->rxneg = 0;
	} else {
		hw->pdata->txneg = 0;
		hw->pdata->rxneg = 1;
	}

	if (spi->mode & SPI_LSB_FIRST)
		hw->pdata->lsb = 1;
	else
		hw->pdata->lsb = 0;

	return 0;
}

static int nuc980_qspi0_setupxfer(struct spi_device *spi,
                                  struct spi_transfer *t)
{
	struct nuc980_spi *hw = (struct nuc980_spi *)to_hw(spi);
	int ret;

	ret = nuc980_qspi0_update_state(spi, t);
	if (ret)
		return ret;

	nuc980_qspi0_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuc980_tx_edge(hw, hw->pdata->txneg);
	nuc980_rx_edge(hw, hw->pdata->rxneg);
	nuc980_set_clock_polarity(hw, hw->pdata->clkpol);
	nuc980_send_first(hw, hw->pdata->lsb);
	nuc980_set_divider(hw);

	return 0;
}

static int nuc980_qspi0_setup(struct spi_device *spi)
{
	struct nuc980_spi *hw = (struct nuc980_spi *)to_hw(spi);
	int ret;

	ret = nuc980_qspi0_update_state(spi, NULL);
	if (ret)
		return ret;

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		nuc980_set_divider(hw);
		nuc980_slave_select(spi, 0);
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

static inline void nuc980_enable_rxth_int(struct nuc980_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_FIFOCTL);

	val = (val & ~0x7000000) | 0x0000000; /* set RXTH = 0 */
	val |= RXTHIEN; /* enable RXTHIEN */

	__raw_writel(val, hw->regs + REG_FIFOCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_disable_rxth_int(struct nuc980_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_FIFOCTL);

	val &= ~RXTHIEN; /* disable RXTHIEN */

	__raw_writel(val, hw->regs + REG_FIFOCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_enable_txth_int(struct nuc980_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_FIFOCTL);

	val = (val & ~0x70000000) | 0x30000000; /* set TXTH = 3 */
	val |= TXTHIEN; /* enable TXTHIEN */

	__raw_writel(val, hw->regs + REG_FIFOCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_disable_txth_int(struct nuc980_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_FIFOCTL);

	val &= ~TXTHIEN; /* disable TXTHIEN */

	__raw_writel(val, hw->regs + REG_FIFOCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void nuc980_enable_ssinact_int(struct nuc980_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_SSCTL);

	val |= SSINAIEN; /* enable SSINAIEN */

	__raw_writel(val, hw->regs + REG_SSCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static irqreturn_t nuc980_qspi0_irq(int irq, void *dev)
{
	struct nuc980_spi *hw = dev;
	unsigned int val, status;

	status = __raw_readl(hw->regs + REG_STATUS);
	__raw_writel(status, hw->regs + REG_STATUS);

	if (status & RXTHIF) {
		if (InTransmitted == 0) {
			nuc980_disable_rxth_int(hw);
			slave_done_state = 1;
			wake_up_interruptible(&slave_done);
			InTransmitted = 1;
		}
	}
	if (status & TXTHIF) {
		while(!(__raw_readl(hw->regs + REG_STATUS) & 0x20000)) {//TXFULL
			__raw_writel(QSPI0_SlaveData[TransmittedCnt++], hw->regs + REG_TX);
			if (TransmittedCnt >= QSPI0_SlaveDataLen) {
				nuc980_disable_txth_int(hw);
				InTransmitted = 0;
				TransmittedCnt = 0;
				break;
			}
		}
	}
	if (status & SSINAIF) {
		/* Check if transmition complete */
		if (InTransmitted == 1) {
			int i;
			printk("Master pull SS high, but slave TX doesn't complete!\n");
			__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | 0x3, hw->regs + REG_FIFOCTL); //TXRX reset
			while (__raw_readl(hw->regs + REG_STATUS) & (1<<23)); //TXRXRST
			nuc980_enable_rxth_int(hw);
			InTransmitted = 0;
			TransmittedCnt = 0;
			for (i = 0; i < QSPI0_SlaveDataLen; i++)
				QSPI0_SlaveData[i] = 0;
		}
	}

	return IRQ_HANDLED;
}


static void nuc980_init_spi(struct nuc980_spi *hw)
{
	clk_prepare(hw->clk);
	clk_enable(hw->clk);

	spin_lock_init(&hw->lock);

	nuc980_tx_edge(hw, hw->pdata->txneg);
	nuc980_rx_edge(hw, hw->pdata->rxneg);
	nuc980_send_first(hw, hw->pdata->lsb);
	nuc980_set_sleep(hw, hw->pdata->sleep);
	nuc980_qspi0_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuc980_set_clock_polarity(hw, hw->pdata->clkpol);
	nuc980_set_divider(hw);
	nuc980_enable_slave(hw);
	nuc980_enable_rxth_int(hw);
	nuc980_enable_ssinact_int(hw);
}

#ifdef CONFIG_OF
static struct nuc980_spi_info *nuc980_qspi0_parse_dt(struct device *dev) {
	struct nuc980_spi_info *sci;
	u32 temp;

	sci = devm_kzalloc(dev, sizeof(*sci), GFP_KERNEL);
	if (!sci) {
		dev_err(dev, "memory allocation for spi_info failed\n");
		return ERR_PTR(-ENOMEM);
	}

	if (of_property_read_u32(dev->of_node, "num_cs", &temp)) {
		dev_warn(dev, "can't get num_cs from dt\n");
		sci->num_cs = 2;
	} else {
		sci->num_cs = temp;
	}

	if (of_property_read_u32(dev->of_node, "lsb", &temp)) {
		dev_warn(dev, "can't get lsb from dt\n");
		sci->lsb = 0;
	} else {
		sci->lsb = temp;
	}

	if (of_property_read_u32(dev->of_node, "txneg", &temp)) {
		dev_warn(dev, "can't get txneg from dt\n");
		sci->txneg = 1;
	} else {
		sci->txneg = temp;
	}

	if (of_property_read_u32(dev->of_node, "clkpol", &temp)) {
		dev_warn(dev, "can't get clkpol from dt\n");
		sci->clkpol = 0;
	} else {
		sci->clkpol = temp;
	}

	if (of_property_read_u32(dev->of_node, "rxneg", &temp)) {
		dev_warn(dev, "can't get rxneg from dt\n");
		sci->rxneg = 0;
	} else {
		sci->rxneg = temp;
	}

	if (of_property_read_u32(dev->of_node, "divider", &temp)) {
		dev_warn(dev, "can't get divider from dt\n");
		sci->divider = 4;
	} else {
		sci->divider = temp;
	}

	if (of_property_read_u32(dev->of_node, "sleep", &temp)) {
		dev_warn(dev, "can't get sleep from dt\n");
		sci->sleep = 0;
	} else {
		sci->sleep = temp;
	}

	if (of_property_read_u32(dev->of_node, "txbitlen", &temp)) {
		dev_warn(dev, "can't get txbitlen from dt\n");
		sci->txbitlen = 8;
	} else {
		sci->txbitlen = temp;
	}

	if (of_property_read_u32(dev->of_node, "bus_num", &temp)) {
		dev_warn(dev, "can't get bus_num from dt\n");
		sci->bus_num = 0;
	} else {
		sci->bus_num = temp;
	}

	return sci;
}
#else
static struct nuc980_spi_info *nuc980_qspi0_parse_dt(struct device *dev) {
	return dev->platform_data;
}
#endif

/* In Thread, only prepare data and enable TXTHIEN.
   The data will be transmitted in IRQ
 */
static int QSPI0_Slave_Thread_TXRX(struct nuc980_spi *hw)
{
	unsigned char rx;
	unsigned long flags;
	int i;

	while(1) {

		wait_event_interruptible(slave_done, (slave_done_state != 0));
		rx = __raw_readl(hw->regs + REG_RX);
		//printk("Receive [0x%x] \n", rx);

		switch (rx) {
		case 0x9f:
			for (i = 0; i < QSPI0_SlaveDataLen; i++)
				QSPI0_SlaveData[i] = i;

			nuc980_enable_txth_int(hw);
			break;
		default:
			break;
		}

		InTransmitted = 0;
		slave_done_state = 0;
		nuc980_enable_rxth_int(hw);
	}

	return 0;
}

static int nuc980_qspi0_slave_probe(struct platform_device *pdev)
{
	struct nuc980_spi *hw;
	struct spi_master *master;
	int err = 0;
	struct pinctrl *p;

	master = spi_alloc_master(&pdev->dev, sizeof(struct nuc980_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	hw->master = spi_master_get(master);
	hw->pdata = nuc980_qspi0_parse_dt(&pdev->dev);
	hw->dev = &pdev->dev;

	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_pdata;
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);
#if defined(SPI_NUC980_QSPI0_SLAVE_NORMAL)
	master->mode_bits          = (SPI_MODE_0 | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);
#elif defined(SPI_NUC980_QSPI0_SLAVE_QUAD)
	master->mode_bits          = (SPI_MODE_0 | SPI_TX_DUAL | SPI_RX_DUAL | SPI_TX_QUAD | SPI_RX_QUAD | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);
#endif
	master->dev.of_node        = pdev->dev.of_node;
	master->num_chipselect     = hw->pdata->num_cs;
	master->bus_num            = hw->pdata->bus_num;
	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = nuc980_qspi0_setupxfer;
	hw->bitbang.chipselect     = nuc980_qspi0_chipsel;
	hw->bitbang.master->setup  = nuc980_qspi0_setup;

	hw->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (hw->res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_pdata;
	}

#if defined(CONFIG_OF)
	hw->regs = devm_ioremap_resource(&pdev->dev, hw->res);
#else
	hw->ioarea = request_mem_region(hw->res->start,
	                                resource_size(hw->res), pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_pdata;
	}

	hw->regs = ioremap(hw->res->start, resource_size(hw->res));
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_iomap;
	}
#endif

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_irq;
	}

	err = request_irq(hw->irq, nuc980_qspi0_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_irq;
	}

	hw->clk = clk_get(NULL, "qspi0_eclk");
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_clk;
	}
	clk_prepare(hw->clk);
	clk_enable(hw->clk);

	hw->clk = clk_get(NULL, "qspi0");
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_clk;
	}

	clk_prepare(hw->clk);
	clk_enable(hw->clk);

#if defined(CONFIG_OF)
	p = devm_pinctrl_get_select_default(&pdev->dev);
#else
	/*
#if defined(CONFIG_SPI_NUC980_P0_NORMAL) && !defined(CONFIG_SPI_NUC980_P0_SS1)
		p = devm_pinctrl_get_select(&pdev->dev, "qspi0");
#elif defined(CONFIG_SPI_NUC980_P0_NORMAL) && defined(CONFIG_SPI_NUC980_P0_SS1_PB0)
		p = devm_pinctrl_get_select(&pdev->dev, "qspi0-ss1-PB");
#elif defined(CONFIG_SPI_NUC980_P0_NORMAL) && defined(CONFIG_SPI_NUC980_P0_SS1_PH12)
		p = devm_pinctrl_get_select(&pdev->dev, "qspi0-ss1-PH");
#elif defined(CONFIG_SPI_NUC980_P0_QUAD) && !defined(CONFIG_SPI_NUC980_P0_SS1)
		p = devm_pinctrl_get_select(&pdev->dev, "qspi0-quad");
#elif defined(CONFIG_SPI_NUC980_P0_QUAD) && defined(CONFIG_SPI_NUC980_P0_SS1_PB0)
		p = devm_pinctrl_get_select(&pdev->dev, "qspi0-quad-ss1-PB");
#elif defined(CONFIG_SPI_NUC980_P0_QUAD) && defined(CONFIG_SPI_NUC980_P0_SS1_PH12)
		p = devm_pinctrl_get_select(&pdev->dev, "qspi0-quad-ss1-PB");
#endif
	*/
	/*TODO : use pinctrl*/
	__raw_writel(0x11111100, REG_MFP_GPD_L);
#endif
	if(IS_ERR(p)) {
		dev_err(&pdev->dev, "unable to reserve spi pin by mode\n");
		err = PTR_ERR(p);
		goto err_register;
	}

	nuc980_init_spi(hw);

	__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | 0x3, hw->regs + REG_FIFOCTL); //CWWeng : RXRST & TXRST
	while (__raw_readl(hw->regs + REG_STATUS) & (1<<23)); //TXRXRST

	__raw_writel(__raw_readl(hw->regs + REG_CTL) | SPIEN, hw->regs + REG_CTL); /* enable SPI */
	while ((__raw_readl(hw->regs + REG_STATUS) & (1<<15)) == 0); //SPIENSTS

	kthread_run(QSPI0_Slave_Thread_TXRX, hw, "QSPI0_SLAVE_THread_TXRX");

	return 0;

err_register:
	clk_disable(hw->clk);
	clk_put(hw->clk);
err_clk:
	free_irq(hw->irq, hw);
err_irq:
	iounmap(hw->regs);
#ifndef CONFIG_OF
err_iomap:
	release_mem_region(hw->res->start, resource_size(hw->res));
	kfree(hw->ioarea);
#endif
err_pdata:
	spi_master_put(hw->master);

err_nomem:
	return err;
}

static int nuc980_qspi0_slave_remove(struct platform_device *dev)
{
	struct nuc980_spi *hw = platform_get_drvdata(dev);

	free_irq(hw->irq, hw);
	platform_set_drvdata(dev, NULL);
	spi_bitbang_stop(&hw->bitbang);

	clk_disable(hw->clk);
	clk_put(hw->clk);

	iounmap(hw->regs);

#ifndef CONFIG_OF
	release_mem_region(hw->res->start, resource_size(hw->res));
	kfree(hw->ioarea);
#else
	kfree(hw->pdata);
#endif

	spi_master_put(hw->master);
	return 0;
}

#ifdef CONFIG_PM
static int nuc980_qspi0_suspend(struct device *dev)
{
	struct nuc980_spi *hw = dev_get_drvdata(dev);

	while(__raw_readl(hw->regs + REG_CTL) & 0x1)
		msleep(1);

	// disable interrupt
	__raw_writel((__raw_readl(hw->regs + REG_CTL) & ~0x20000), hw->regs + REG_CTL);

	return 0;
}

static int nuc980_qspi0_resume(struct device *dev)
{
	struct nuc980_spi *hw = dev_get_drvdata(dev);

	// enable interrupt
	__raw_writel((__raw_readl(hw->regs + REG_CTL) | 0x20000), hw->regs + REG_CTL);

	return 0;
}

static const struct dev_pm_ops nuc980_qspi0_pmops = {
	.suspend    = nuc980_qspi0_suspend,
	.resume     = nuc980_qspi0_resume,
};

#define NUC980_QSPI0_PMOPS (&nuc980_qspi0_pmops)

#else
#define NUC980_QSPI0_PMOPS NULL
#endif

#if defined(CONFIG_OF)
static const struct of_device_id nuc980_qspi0_slave_of_match[] = {
	{   .compatible = "nuvoton,nuc980-qspi0-slave" },
	{	},
};
MODULE_DEVICE_TABLE(of, nuc980_qspi0_of_match);
#endif

static struct platform_driver nuc980_qspi0_slave_driver = {
	.probe      = nuc980_qspi0_slave_probe,
	.remove     = nuc980_qspi0_slave_remove,
	.driver     = {
		.name   = "nuc980-qspi0-slave",
		.owner  = THIS_MODULE,
		.pm	= NUC980_QSPI0_PMOPS,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(nuc980_qspi0_slave_of_match),
#endif
	},
};
module_platform_driver(nuc980_qspi0_slave_driver);

MODULE_DESCRIPTION("nuc980 qspi0 slave driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc980-qspi0-slave");
