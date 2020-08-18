/*
 *  linux/drivers/drivers/gpio/nuc980-gpio.c - Nuvoton NUC980 GPIO Driver
 *
 *  Copyright (c) 2018 Nuvoton Technology Corp.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not,     write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.  /gpio-tc3589x.c/
 */



#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>

#include <mach/irqs.h>

//#define GPIO_DEBUG_ENABLE_ENTER_LEAVE
#ifdef GPIO_DEBUG_ENABLE_ENTER_LEAVE
#define ENTRY()                 printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()                 printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif

#define GPIO_PMD_INPUT              0x0UL                  /*!< Input Mode */
#define GPIO_PMD_OUTPUT             0x1UL                  /*!< Output Mode */
#define GPIO_PMD_OPEN_DRAIN         0x2UL                  /*!< Open-Drain Mode */
#define GPIO_PMD_QUASI              0x3UL                  /*!< Quasi-bidirectional Mode */
#define GPIO_PMD_MODE(pin, mode)    ((mode) << ((pin)<<1)) /*!< Generate the PMD mode setting for each pin  */


static DEFINE_SPINLOCK(gpio_lock);

static unsigned short gpio_ba;

struct gpio_port {
	volatile unsigned int * dir;
	volatile unsigned int * out;
	volatile unsigned int * in;
};

static const struct gpio_port port_class[] = {
	{
		(volatile unsigned int *)REG_GPIOA_MODE, (volatile unsigned int *)REG_GPIOA_DOUT,
		(volatile unsigned int *)REG_GPIOA_PIN
	},
	{
		(volatile unsigned int *)REG_GPIOB_MODE, (volatile unsigned int *)REG_GPIOB_DOUT,
		(volatile unsigned int *)REG_GPIOB_PIN
	},
	{
		(volatile unsigned int *)REG_GPIOC_MODE, (volatile unsigned int *)REG_GPIOC_DOUT,
		(volatile unsigned int *)REG_GPIOC_PIN
	},
	{
		(volatile unsigned int *)REG_GPIOD_MODE, (volatile unsigned int *)REG_GPIOD_DOUT,
		(volatile unsigned int *)REG_GPIOD_PIN
	},
	{
		(volatile unsigned int *)REG_GPIOE_MODE, (volatile unsigned int *)REG_GPIOE_DOUT,
		(volatile unsigned int *)REG_GPIOE_PIN
	},
	{
		(volatile unsigned int *)REG_GPIOF_MODE, (volatile unsigned int *)REG_GPIOF_DOUT,
		(volatile unsigned int *)REG_GPIOF_PIN
	},
	{
		(volatile unsigned int *)REG_GPIOG_MODE, (volatile unsigned int *)REG_GPIOG_DOUT,
		(volatile unsigned int *)REG_GPIOG_PIN
	},
	{},
};

static const struct gpio_port *nuc980_gpio_cla_port(unsigned gpio_num,
                unsigned *group,unsigned *num) {
	*group = gpio_num / GPIO_OFFSET;
	*num = gpio_num % GPIO_OFFSET;
	return &port_class[*group];
}

static int nuc980_gpio_core_direction_in(struct gpio_chip *gc,
                unsigned gpio_num)
{
	int port_num,group_num;
	unsigned long value;
	unsigned long flags;
	const struct gpio_port *port =
	        nuc980_gpio_cla_port(gpio_num, &group_num, &port_num);
	ENTRY();
	spin_lock_irqsave(&gpio_lock, flags);
	value = __raw_readl(port->dir);
	value &= ~GPIO_PMD_MODE(port_num,GPIO_PMD_QUASI);
	value |= GPIO_PMD_MODE(port_num,GPIO_PMD_INPUT);
	__raw_writel(value, port->dir);
	spin_unlock_irqrestore(&gpio_lock, flags);
	LEAVE();
	return 0;
}

static int nuc980_gpio_core_get(struct gpio_chip *gc, unsigned gpio_num)
{
	int port_num,group_num;
	const struct gpio_port *port;
	ENTRY();
	port = nuc980_gpio_cla_port(gpio_num, &group_num, &port_num);
	LEAVE();
	return GPIO_PIN_DATA(group_num,port_num);
}

static void nuc980_gpio_core_set(struct gpio_chip *gc, unsigned gpio_num,
                                 int val)
{
	int port_num,group_num;
	const struct gpio_port *port;
	ENTRY();
	port = nuc980_gpio_cla_port(gpio_num, &group_num, &port_num);
	GPIO_PIN_DATA(group_num,port_num)=val;
	LEAVE();
}

static int nuc980_gpio_core_direction_out(struct gpio_chip *gc,
                unsigned gpio_num, int val)
{
	int port_num,group_num;
	unsigned long value;
	unsigned long flags;
	const struct gpio_port *port =
	        nuc980_gpio_cla_port(gpio_num, &group_num, &port_num);
	ENTRY();
	spin_lock_irqsave(&gpio_lock, flags);
	value = __raw_readl(port->dir);
	value &= ~GPIO_PMD_MODE(port_num,GPIO_PMD_QUASI);
	value |= GPIO_PMD_MODE(port_num,GPIO_PMD_OUTPUT);
	__raw_writel(value, port->dir);
	spin_unlock_irqrestore(&gpio_lock, flags);
	nuc980_gpio_core_set(gc, gpio_num, val);
	LEAVE();
	return 0;
}

static int nuc980_gpio_core_to_request(struct gpio_chip *chip, unsigned offset)
{
	unsigned int group,num1,num,reg,value;
	ENTRY();
	group = offset / GPIO_OFFSET;
	num1  = num = offset % GPIO_OFFSET;
	reg   = (unsigned int)REG_MFP_GPA_L+(group* 0x08);
	if(num>7) {
		num -= 8;
		reg = reg + 0x04 ;
	}

	value = ( __raw_readl((volatile unsigned int *)reg) & (0xf<<(num*4)))>>(num*4);
	if(value>0 && value<0xf) {
		printk(KERN_ERR "Please Check GPIO%c%02d's multi-function = 0x%x \n",(char)(65+group),num1,value);
		LEAVE();
		return -EINVAL;
	}
	LEAVE();
	return 0;
}

static void nuc980_gpio_core_to_free(struct gpio_chip *chip, unsigned offset)
{
	ENTRY();
}

static int nuc980_gpio_core_to_irq(struct gpio_chip *chip, unsigned offset)
{
	unsigned int irqno= IRQ_GPIO_START+offset;
	ENTRY();
	switch(offset) {
	case NUC980_PA0:
		if((__raw_readl(REG_MFP_GPA_L) & (0xf<<0))==(0x5<<0))
			irqno = IRQ_EXT0_A0;
		break;

	case NUC980_PA1:
		if((__raw_readl(REG_MFP_GPA_L) & (0xf<<4))==(0x5<<4))
			irqno = IRQ_EXT1_A1;
		break;
	case NUC980_PD0:
		if((__raw_readl(REG_MFP_GPD_L) & (0xf<<0))==(0x4<<0))
			irqno = IRQ_EXT2_D0;
		break;
	case NUC980_PD1:
		if((__raw_readl(REG_MFP_GPD_L) & (0xf<<4))==(0x4<<4))
			irqno = IRQ_EXT3_D1;
		break;
	case NUC980_PA13:
		if((__raw_readl(REG_MFP_GPA_H) & (0xf<<20))==(0x8<<20))
			irqno = IRQ_EXT0_A13;
		break;
	case NUC980_PA14:
		if((__raw_readl(REG_MFP_GPA_H) & (0xf<<24))==(0x8<<24))
			irqno = IRQ_EXT1_A14;
		break;
	case NUC980_PE10:
		if((__raw_readl(REG_MFP_GPE_H) & (0xf<<8))==(0x5<<8))
			irqno = IRQ_EXT2_E10;
		break;
	case NUC980_PE12:
		if((__raw_readl(REG_MFP_GPE_H) & (0xf<<16))==(0x5<<16))
			irqno = IRQ_EXT3_E12;
		break;
	case NUC980_PB3:
		if((__raw_readl(REG_MFP_GPB_L) & (0xf<<12))==(0x3<<12))
			irqno = IRQ_EXT2_B3;
		break;
	case NUC980_PB13:
		if((__raw_readl(REG_MFP_GPB_H) & (0xf<<20))==(0x2<<20))
			irqno = IRQ_EXT2_B13;
		break;
	case NUC980_PG15:
		if((__raw_readl(REG_MFP_GPG_H) & (0xf<<24))==(0x4<<24))
			irqno = IRQ_EXT3_G15;
		break;
	default:
		irqno = IRQ_GPIO_START+offset;
		break;
	}
	LEAVE();
	return irqno;
}

static struct gpio_chip nuc980_gpio_port = {
	.label = "nuc980_gpio_port",
	.owner = THIS_MODULE,
	.direction_input = nuc980_gpio_core_direction_in,
	.get = nuc980_gpio_core_get,
	.direction_output = nuc980_gpio_core_direction_out,
	.set = nuc980_gpio_core_set,
	.request = nuc980_gpio_core_to_request,
	.free = nuc980_gpio_core_to_free,
	.to_irq = nuc980_gpio_core_to_irq,
	.base = 0,
	.ngpio = NUMGPIO,
};


#ifndef CONFIG_USE_OF
/*
 * @brief       External Interrupt 0 Handler
 * @details     This function will be used by EINT0,
 *              when enable IRQ_EXT0_A0 or IRQ_EXT0_A13 in eint0
 */
/*
static irqreturn_t nuc980_eint0_interrupt(int irq, void *dev_id){
    printk("@0\n");
    return IRQ_HANDLED;
}
*/

/* If enable IRQ_EXT0_A0 or IRQ_EXT0_A13 , linux will enable EINT0
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc980_eint_pins eint0[]= {
//{IRQ_EXT0_A0, nuc980_eint0_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint0"},
//{IRQ_EXT0_A13,nuc980_eint0_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint0"},
	{0,0,0,0}
};

/*
 * @brief       External Interrupt 0 Handler
 * @details     This function will be used by EINT1,
 *              when enable IRQ_EXT1_A1 or IRQ_EXT1_A14 in eint1
 */
/*
static irqreturn_t nuc980_eint1_interrupt(int irq, void *dev_id){
    printk("@1\n");
    return IRQ_HANDLED;
}
*/

/* If enable IRQ_EXT1_A1 or IRQ_EXT1_A14 , linux will enable EINT1
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc980_eint_pins eint1[]= {
//{IRQ_EXT1_A1, nuc980_eint1_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint1"},
//{IRQ_EXT1_A14,nuc980_eint1_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint1"},
	{0,0,0,0}
};

/*
 * @brief       External Interrupt 2 Handler
 * @details     This function will be used by EINT2,
 *              when enable IRQ_EXT2_D0 , IRQ_EXT2_E10 , IRQ_EXT2_B3 or IRQ_EXT2_B13 in eint2
 */
/*
static irqreturn_t nuc980_eint2_interrupt(int irq, void *dev_id){
    printk("@2\n");
    return IRQ_HANDLED;
}
*/


/* If enable IRQ_EXT2_D0 , IRQ_EXT2_E10 , IRQ_EXT2_B3 , IRQ_EXT2_B13 , linux will enable EINT2
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc980_eint_pins eint2[]= {
//{IRQ_EXT2_D0, nuc980_eint2_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint2"},
//{IRQ_EXT2_E10,nuc980_eint2_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint2"},
//{IRQ_EXT2_B3,nuc980_eint2_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint2"},
//{IRQ_EXT2_B13,nuc980_eint2_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint2"},
	{0,0,0,0}
};

/*
 * @brief       External Interrupt 3 Handler
 * @details     This function will be used by EINT3,
 *              when enable IRQ_EXT3_D1 , IRQ_EXT3_E12 or IRQ_EXT3_G15 in eint3
 */
/*
static irqreturn_t nuc980_eint3_interrupt(int irq, void *dev_id){
    printk("@3\n");
    return IRQ_HANDLED;
}
*/


/* If enable IRQ_EXT3_D1 , IRQ_EXT3_E12 or IRQ_EXT3_G15 , linux will enable EINT31
 * User can modify trigger tiypes as below :
 * IRQF_TRIGGER_FALLING / IRQF_TRIGGER_RISING / IRQF_TRIGGER_HIGH / IRQF_TRIGGER_LOW
 */
struct nuc980_eint_pins eint3[]= {
//{IRQ_EXT3_D1, nuc980_eint3_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint3"},
//{IRQ_EXT3_E12,nuc980_eint3_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint3"},
//{IRQ_EXT3_G15,nuc980_eint3_interrupt,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,"eint3"},
	{0,0,0,0}
};

static int nuc980_enable_eint(uint32_t flag,struct platform_device *pdev)
{
	int err;
	struct nuc980_eint_pins *peint;
	struct pinctrl *p = NULL;
	switch(pdev->id) {
	case 1:
		peint=eint0;
		while(peint->pin!=(u32)0) {
			if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
				printk("register %s irq failed %d\n",peint->name ,err);
			}
			if(flag==1) {
				__raw_writel((1<<4) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
				enable_irq_wake(peint->pin);
			}
			switch(peint->pin) {
			case IRQ_EXT0_A0:
				p = devm_pinctrl_get_select(&pdev->dev, "eint0-PA0");
				break;
			case IRQ_EXT0_A13:
				p = devm_pinctrl_get_select(&pdev->dev, "eint0-PA13");
				break;
			}
			if (IS_ERR(p)) {
				dev_err(&pdev->dev, "unable to reserve pin\n");
				return PTR_ERR(p);
			}
			peint++;
		}
		break;
	case 2:
		peint=eint1;
		while(peint->pin!=(u32)0) {
			if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
				printk("register %s irq failed %d\n",peint->name ,err);
			}
			if(flag==1) {
				__raw_writel((1<<5) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
				enable_irq_wake(peint->pin);
			}
			switch(peint->pin) {
			case IRQ_EXT1_A1:
				p = devm_pinctrl_get_select(&pdev->dev, "eint1-PA1");
				break;
			case IRQ_EXT1_A14:
				p = devm_pinctrl_get_select(&pdev->dev, "eint1-PA14");
				break;
			}
			if (IS_ERR(p)) {
				dev_err(&pdev->dev, "unable to reserve pin\n");
				return PTR_ERR(p);
			}
			peint++;
		}
		break;
	case 3:
		peint=eint2;
		while(peint->pin!=(u32)0) {
			if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
				printk("register %s irq failed %d\n",peint->name ,err);
			}
			if(flag==1) {
				__raw_writel((1<<6) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
				enable_irq_wake(peint->pin);
			}
			switch(peint->pin) {
			case IRQ_EXT2_D0:
				p = devm_pinctrl_get_select(&pdev->dev, "eint2-PD0");
				break;
			case IRQ_EXT2_E10:
				p = devm_pinctrl_get_select(&pdev->dev, "eint2-PE10");
				break;
			case IRQ_EXT2_B3:
				p = devm_pinctrl_get_select(&pdev->dev, "eint2-PB3");
				break;
			case IRQ_EXT2_B13:
				p = devm_pinctrl_get_select(&pdev->dev, "eint2-PB13");
				break;
			}
			if (IS_ERR(p)) {
				dev_err(&pdev->dev, "unable to reserve pin\n");
				return PTR_ERR(p);
			}
			peint++;
		}
		break;
	case 4:
		peint=eint3;
		while(peint->pin!=(u32)0) {
			if ((err = request_irq(peint->pin,peint->handler, peint->trigger|IRQF_NO_SUSPEND, peint->name, 0)) != 0) {
				printk("register %s irq failed %d\n",peint->name ,err);
			}
			if(flag==1) {
				__raw_writel((1<<7) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
				enable_irq_wake(peint->pin);
			}
			switch(peint->pin) {
			case IRQ_EXT3_D1:
				p = devm_pinctrl_get_select(&pdev->dev, "eint3-PD1");
				break;
			case IRQ_EXT3_E12:
				p = devm_pinctrl_get_select(&pdev->dev, "eint3-PE12");
				break;
			case IRQ_EXT3_G15:
				p = devm_pinctrl_get_select(&pdev->dev, "eint3-PG15");
				break;
			}
			if (IS_ERR(p)) {
				dev_err(&pdev->dev, "unable to reserve pin\n");
				return PTR_ERR(p);
			}
			peint++;
		}
		break;
	}
	return 0;
}
#else

static irqreturn_t nuc980_eint0_interrupt(int irq, void *dev_id)
{
	printk("@0\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc980_eint1_interrupt(int irq, void *dev_id)
{
	printk("@1\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc980_eint2_interrupt(int irq, void *dev_id)
{
	printk("@2\n");
	return IRQ_HANDLED;
}
__attribute__ ((unused)) static irqreturn_t nuc980_eint3_interrupt(int irq, void *dev_id)
{
	printk("@3\n");
	return IRQ_HANDLED;
}

u32 trigger_type[5]= {   (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
                         IRQF_TRIGGER_RISING,
                         IRQF_TRIGGER_FALLING,
                         IRQF_TRIGGER_HIGH,
                         IRQF_TRIGGER_LOW
                     };

static int nuc980_enable_eint(uint32_t flag,struct platform_device *pdev)
{
	int err;
	u32 val32[3];
	u32 irqnum,irqflag;

	//eint 0
	if (of_property_read_u32_array(pdev->dev.of_node, "eint0-config", val32, 3) != 0) {
		printk("%s - eint0 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1) {
		irqnum=(val32[1]==0)?(IRQ_EXT0_A0):(IRQ_EXT0_A13);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1) {
			__raw_writel((1<<4) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
			enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc980_eint0_interrupt,irqflag, "eint0", 0)) != 0) {
			printk("%s - eint0 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 1
	if (of_property_read_u32_array(pdev->dev.of_node, "eint1-config", val32, 3) != 0) {
		printk("%s - eint1 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1) {
		irqnum=(val32[1]==0)?(IRQ_EXT1_A1):(IRQ_EXT1_A14);
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1) {
			__raw_writel((1<<5) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
			enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc980_eint1_interrupt,irqflag, "eint1", 0)) != 0) {
			printk("%s - eint1 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 2
	if (of_property_read_u32_array(pdev->dev.of_node, "eint2-config", val32, 3) != 0) {
		printk("%s - eint2 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1) {
		if(val32[1]==0)
			irqnum = IRQ_EXT2_D0;
		else if(val32[1]==1)
			irqnum = IRQ_EXT2_E10;
		else if(val32[1]==2) {
			printk("======================>IRQ_EXT2_B3\n");
			irqnum = IRQ_EXT2_B3;
		} else
			irqnum = IRQ_EXT2_B13;
		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1) {
			__raw_writel((1<<6) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
			enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc980_eint2_interrupt,irqflag, "eint2", 0)) != 0) {
			printk("%s - eint2 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}

	//eint 3
	if (of_property_read_u32_array(pdev->dev.of_node, "eint3-config", val32, 3) != 0) {
		printk("%s - eint3 can not get port-number!\n", __func__);
		return -EINVAL;
	}
	if(val32[0]==1) {
		if(val32[1]==0)
			irqnum = IRQ_EXT3_D1;
		else if(val32[1]==1)
			irqnum = IRQ_EXT3_E12;
		else
			irqnum = IRQ_EXT3_G15;

		irqflag=trigger_type[val32[2]]|IRQF_NO_SUSPEND;
		if(flag==1) {
			__raw_writel((1<<7) | __raw_readl(REG_WKUPSER0),REG_WKUPSER0);
			enable_irq_wake(irqnum);
		}
		if ((err = request_irq(irqnum,nuc980_eint3_interrupt,irqflag, "eint3", 0)) != 0) {
			printk("%s - eint3 can not get irq!\n", __func__);
			return -EINVAL;
		}
	}
	return 0;
}
#endif

static int nuc980_gpio_probe(struct platform_device *pdev)
{
	int err;
	struct clk *clk;

#ifndef CONFIG_USE_OF
	if(pdev->id == 0)
#else
	struct device_node *np = pdev->dev.of_node;
#endif
	{
		printk("%s - pdev = %s\n", __func__, pdev->name);
		/* Enable GPIO clock */
		clk = clk_get(NULL, "gpio_hclk");
		if (IS_ERR(clk)) {
			printk(KERN_ERR "nuc980-gpio:failed to get gpio clock source\n");
			err = PTR_ERR(clk);
			return err;
		}
		clk_prepare(clk);
		clk_enable(clk);
#ifdef CONFIG_USE_OF
		irq_domain_add_legacy(np, IRQ_GPIO_END-IRQ_GPIO_START, IRQ_GPIO_START, 0,&irq_domain_simple_ops, NULL);
#endif
		nuc980_gpio_port.dev = &pdev->dev;
		err = gpiochip_add(&nuc980_gpio_port);
		if (err < 0) {
			goto err_nuc980_gpio_port;
		}

	}

#ifdef CONFIG_USE_OF
	{
		struct pinctrl *pinctrl;
		pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
		if (IS_ERR(pinctrl)) {
			return PTR_ERR(pinctrl);
		}
	}
#endif
#ifdef CONFIG_GPIO_NUC980_EINT_WKUP
	nuc980_enable_eint(1,pdev);
#else
	nuc980_enable_eint(0,pdev);
#endif

	return 0;

err_nuc980_gpio_port:
	gpio_ba = 0;
	return err;
}

static int nuc980_gpio_remove(struct platform_device *pdev)
{
	struct clk *clk;

	/* Disable GPIO clock */
	clk = clk_get(NULL, "gpio");
	if (IS_ERR(clk)) {
		int err;

		printk(KERN_ERR "nuc980-gpio:failed to get gpio clock source\n");
		err = PTR_ERR(clk);
		return err;
	}

	clk_disable(clk);

	if (gpio_ba) {
		gpiochip_remove(&nuc980_gpio_port);
	}

	return 0;
}

static int nuc980_gpio_resume(struct platform_device *pdev)
{
	ENTRY();
	LEAVE();
	return 0;
}

static int nuc980_gpio_suspend(struct platform_device *pdev,pm_message_t state)
{
	ENTRY();
	LEAVE();
	return 0;
}

static const struct of_device_id nuc980_gpio_of_match[] = {
	{ .compatible = "nuvoton,nuc980-gpio" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_serial_of_match);

static struct platform_driver nuc980_gpio_driver = {
	.probe      = nuc980_gpio_probe,
	.remove     = nuc980_gpio_remove,
	.resume     = nuc980_gpio_resume,
	.suspend    = nuc980_gpio_suspend,
	.driver     = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_gpio_of_match),
	},
};
module_platform_driver(nuc980_gpio_driver);

MODULE_DESCRIPTION("GPIO interface for Nuvoton NUC980 GPIO Drive");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc980_gpio");
