/*
 * linux/arch/arm/mach-nuc980/dev.c
 *
 * Copyright (C) 2017 Nuvoton technology corporation
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/pwm_backlight.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/pwm.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-serial.h>
#include <mach/irqs.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>

#include <mach/map.h>
#include <mach/gpio.h>

#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/i2c-hid.h>

#include <linux/platform_data/i2c-nuc980.h>
#include <linux/platform_data/spi-nuc980.h>
#include <linux/platform_data/dma-nuc980.h>

#include "cpu.h"

/* USB EHCI Host Controller */
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
static struct resource nuc980_ehci_resource[] = {
	[0] = {
		.start = NUC980_PA_EHCI,
		.end = NUC980_PA_EHCI + NUC980_SZ_EHCI - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_EHCI,
		.end = IRQ_EHCI,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc980_device_usb_ehci_dmamask = 0xffffffffUL;

static struct platform_device nuc980_device_ehci = {
	.name = "nuc980-ehci",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_ehci_resource),
	.resource = nuc980_ehci_resource,
	.dev = {
		.dma_mask = &nuc980_device_usb_ehci_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif
/* USB OHCI Host Controller */
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct resource nuc980_ohci_resource[] = {
	[0] = {
		.start = NUC980_PA_OHCI,
		.end = NUC980_PA_OHCI + NUC980_SZ_OHCI - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_OHCI,
		.end = IRQ_OHCI,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc980_device_usb_ohci_dmamask = 0xffffffffUL;
static struct platform_device nuc980_device_ohci = {
	.name = "nuc980-ohci",
	.id = 0,
	.num_resources = ARRAY_SIZE(nuc980_ohci_resource),
	.resource = nuc980_ohci_resource,
	.dev = {
		.dma_mask = &nuc980_device_usb_ohci_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
static struct platform_device nuc980_device_usb_lite0 = {
	.name = "nuc980-ohci",
	.id = 1,
	.resource = nuc980_ohci_resource,
};
static struct platform_device nuc980_device_usb_lite1 = {
	.name = "nuc980-ohci",
	.id = 2,
	.resource = nuc980_ohci_resource,
};
static struct platform_device nuc980_device_usb_lite2 = {
	.name = "nuc980-ohci",
	.id = 3,
	.resource = nuc980_ohci_resource,
};
static struct platform_device nuc980_device_usb_lite3 = {
	.name = "nuc980-ohci",
	.id = 4,
	.resource = nuc980_ohci_resource,
};
static struct platform_device nuc980_device_usb_lite4 = {
	.name = "nuc980-ohci",
	.id = 5,
	.resource = nuc980_ohci_resource,
};
static struct platform_device nuc980_device_usb_lite5 = {
	.name = "nuc980-ohci",
	.id = 6,
	.resource = nuc980_ohci_resource,
};
#endif

/* Cryptographic Accelerator */
#if defined(CONFIG_CRYPTO_DEV_NUC980) || defined(CONFIG_CRYPTO_DEV_NUC980_MODULE)
static struct resource nuc980_crypto_resource[] = {
	[0] = {
		.start = NUC980_PA_CRYPTO,
		.end = NUC980_PA_CRYPTO + NUC980_SZ_CRYPTO - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CRYPTO,
		.end = IRQ_CRYPTO,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc980_device_crypto_dmamask = 0xffffffffUL;
static struct platform_device nuc980_device_crypto = {
	.name = "nuc980-crypto",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_crypto_resource),
	.resource = nuc980_crypto_resource,
	.dev = {
		.dma_mask = &nuc980_device_crypto_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

static struct platform_device nuc980_device_crypto_raw = {
	.name = "nuc980-crypto-raw",
	.id = -1,
	.resource = nuc980_crypto_resource,
};

static struct platform_device nuc980_device_prng = {
	.name = "nuvoton-rng",
	.id = -1,
	.resource = nuc980_crypto_resource,
};
#endif

/* USB Device (Gadget)*/
#if defined(CONFIG_USB_NUC980) || defined(CONFIG_USB_NUC980_MODULE)
static struct resource nuc980_usbgadget_resource[] = {
	[0] = {
		.start = NUC980_PA_USBDEV,
		.end = NUC980_PA_USBDEV + NUC980_SZ_USBDEV - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UDC,
		.end = IRQ_UDC,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc980_device_udc_dmamask = 0xffffffffUL;
static struct platform_device nuc980_device_usbgadget = {
	.name = "nuc980-usbdev",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_usbgadget_resource),
	.resource = nuc980_usbgadget_resource,
	.dev = {
		.dma_mask = &nuc980_device_udc_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

/* Initial serial platform data */
static struct plat_nuc980serial_port nuc980_uart_data[] = {
	[0] = NUC980SERIAL_PORT(UART0),
	[1] = NUC980SERIAL_PORT(UART1),
	[2] = NUC980SERIAL_PORT(UART2),
	[3] = NUC980SERIAL_PORT(UART3),
	[4] = NUC980SERIAL_PORT(UART4),
	[5] = NUC980SERIAL_PORT(UART5),
	[6] = NUC980SERIAL_PORT(UART6),
	[7] = NUC980SERIAL_PORT(UART7),
	[8] = NUC980SERIAL_PORT(UART8),
	[9] = NUC980SERIAL_PORT(UART9),
	{},
};

static struct platform_device nuc980_serial_device0 = {
	.name = "nuc980-uart",
	.id = 0,
	.dev = {
		.platform_data = &nuc980_uart_data[0],
	},
};

#if defined(CONFIG_NUC980_UART1) || defined(CONFIG_NUC980_UART1_MODULE)
static struct platform_device nuc980_serial_device1 = {
	.name = "nuc980-uart",
	.id = 1,
	.dev = {
		.platform_data = &nuc980_uart_data[1],
	},
};
#endif

#if defined(CONFIG_NUC980_UART2) || defined(CONFIG_NUC980_UART2_MODULE)
static struct platform_device nuc980_serial_device2 = {
	.name = "nuc980-uart",
	.id = 2,
	.dev = {
		.platform_data = &nuc980_uart_data[2],
	},
};
#endif

#if defined(CONFIG_NUC980_UART3) || defined(CONFIG_NUC980_UART3_MODULE)
static struct platform_device nuc980_serial_device3 = {
	.name = "nuc980-uart",
	.id = 3,
	.dev = {
		.platform_data = &nuc980_uart_data[3],
	},
};
#endif

#if defined(CONFIG_NUC980_UART4) || defined(CONFIG_NUC980_UART4_MODULE)
static struct platform_device nuc980_serial_device4 = {
	.name = "nuc980-uart",
	.id = 4,
	.dev = {
		.platform_data = &nuc980_uart_data[4],
	},
};
#endif

#if defined(CONFIG_NUC980_UART5) || defined(CONFIG_NUC980_UART5_MODULE)
static struct platform_device nuc980_serial_device5 = {
	.name = "nuc980-uart",
	.id = 5,
	.dev = {
		.platform_data = &nuc980_uart_data[5],
	},
};
#endif

#if defined(CONFIG_NUC980_UART6) || defined(CONFIG_NUC980_UART6_MODULE)
static struct platform_device nuc980_serial_device6 = {
	.name = "nuc980-uart",
	.id = 6,
	.dev = {
		.platform_data = &nuc980_uart_data[6],
	},
};
#endif

#if defined(CONFIG_NUC980_UART7) || defined(CONFIG_NUC980_UART7_MODULE)
static struct platform_device nuc980_serial_device7 = {
	.name = "nuc980-uart",
	.id = 7,
	.dev = {
		.platform_data = &nuc980_uart_data[7],
	},
};
#endif

#if defined(CONFIG_NUC980_UART8) || defined(CONFIG_NUC980_UART8_MODULE)
static struct platform_device nuc980_serial_device8 = {
	.name = "nuc980-uart",
	.id = 8,
	.dev = {
		.platform_data = &nuc980_uart_data[8],
	},
};
#endif

#if defined(CONFIG_NUC980_UART9) || defined(CONFIG_NUC980_UART9_MODULE)
static struct platform_device nuc980_serial_device9 = {
	.name = "nuc980-uart",
	.id = 9,
	.dev = {
		.platform_data = &nuc980_uart_data[9],
	},
};
#endif

/* SDIO Controller */
#if defined(CONFIG_MMC_NUC980_SD) || defined(CONFIG_MMC_NUC980_SD_MODULE)
static struct resource nuc980_sdh_resource[] = {
	[0] = {
		.start = NUC980_PA_SDH,
		.end = NUC980_PA_SDH + NUC980_SZ_SDH - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDH,
		.end = IRQ_SDH,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc980_device_sdh_dmamask = 0xffffffffUL;
struct platform_device nuc980_device_sdh = {
	.name = "nuc980-sdh",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_sdh_resource),
	.resource = nuc980_sdh_resource,
	.dev = {
		.dma_mask = &nuc980_device_sdh_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

/* NAND, SD0 Controller */
#if defined(CONFIG_MTD_NAND_NUC980) || defined(CONFIG_MTD_NAND_NUC980_SD_MODULE) || defined(CONFIG_MMC_NUC980_FMI) || defined(CONFIG_MMC_NUC980_FMI_MODULE)
static struct resource nuc980_fmi_resource[] = {
	[0] = {
		.start = NUC980_PA_FMI,
		.end = NUC980_PA_FMI + NUC980_SZ_FMI - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_FMI,
		.end = IRQ_FMI,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc980_device_fmi_dmamask = 0xffffffffUL;
struct platform_device nuc980_device_fmi = {
	.name = "nuc980-fmi",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_fmi_resource),
	.resource = nuc980_fmi_resource,
	.dev = {
		.dma_mask = &nuc980_device_fmi_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

/* Ethernet MAC0 Controller */
#if defined(CONFIG_NUC980_ETH0) || defined(CONFIG_NUC980_ETH0_MODULE)
static struct resource nuc980_emac0_resource[] = {
	[0] = {
		.start = NUC980_PA_EMAC0,
		.end = NUC980_PA_EMAC0 + NUC980_SZ_EMAC0 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_EMC0TX,
		.end = IRQ_EMC0TX,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_EMC0RX,
		.end = IRQ_EMC0RX,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc980_device_emac0_dmamask = 0xffffffffUL;
struct platform_device nuc980_device_emac0 = {
	.name = "nuc980-emac0",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_emac0_resource),
	.resource = nuc980_emac0_resource,
	.dev = {
		.dma_mask = &nuc980_device_emac0_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

#if defined(CONFIG_NUC980_ETH1) || defined(CONFIG_NUC980_ETH1_MODULE)
/* Ethernet MAC1 Controller */
static struct resource nuc980_emac1_resource[] = {
	[0] = {
		.start = NUC980_PA_EMAC1,
		.end = NUC980_PA_EMAC1 + NUC980_SZ_EMAC1 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_EMC1TX,
		.end = IRQ_EMC1TX,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_EMC1RX,
		.end = IRQ_EMC1RX,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc980_device_emac1_dmamask = 0xffffffffUL;
struct platform_device nuc980_device_emac1 = {
	.name = "nuc980-emac1",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_emac1_resource),
	.resource = nuc980_emac1_resource,
	.dev = {
		.dma_mask = &nuc980_device_emac1_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

/* VIDEOIN0 */
#if defined(CONFIG_VIDEO0_NUC980) || defined(CONFIG_VIDEO0_NUC980_MODULE)
static struct resource nuc980_cap0_resource[] = {
	[0] = {
		.start = NUC980_PA_CAP0,
		.end = NUC980_PA_CAP0 + NUC980_SZ_CAP0 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CAP0,
		.end = IRQ_CAP0,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_cap0 = {
	.name = "nuc980-videoin0",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_cap0_resource),
	.resource = nuc980_cap0_resource,
};

#endif


/* VIDEOIN1 */
#if defined(CONFIG_VIDEO1_NUC980) || defined(CONFIG_VIDEO1_NUC980_MODULE)
static struct resource nuc980_cap1_resource[] = {
	[0] = {
		.start = NUC980_PA_CAP1,
		.end = NUC980_PA_CAP1 + NUC980_SZ_CAP1 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CAP1,
		.end = IRQ_CAP1,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_cap1 = {
	.name = "nuc980-videoin1",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_cap1_resource),
	.resource = nuc980_cap1_resource,
};

#endif


/* Normal ADC */
#if defined(CONFIG_IIO_NUC980ADC) || defined(CONFIG_IIO_NUC980ADC_MODULE)
static struct resource nuc980_nadc_resource[] = {
	[0] = {
		.start = NUC980_PA_ADC,
		.end = NUC980_PA_ADC + NUC980_SZ_ADC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_ADC,
		.end = IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_nadc = {
	.name = "nuc980-nadc",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_nadc_resource),
	.resource = nuc980_nadc_resource,
};
#endif


#if defined(CONFIG_NUC980_DMA) || defined(CONFIG_NUC980_DMA_MODULE)
#define DMA_CHANNEL(_name, _base, _irq) \
        { .name = (_name), .base = (_base), .irq = (_irq) }
static struct resource nuc980_dma_resource[] = {
	[0] = {
		.start = NUC980_PA_PDMA0,
		.end   = NUC980_PA_PDMA0 + NUC980_SZ_PDMA0 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = NUC980_PA_PDMA1,
		.end   = NUC980_PA_PDMA1 + NUC980_SZ_PDMA1 - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = IRQ_PDMA0,
		.end   = IRQ_PDMA0,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_PDMA1,
		.end   = IRQ_PDMA1,
		.flags = IORESOURCE_IRQ,
	}
};

struct nuc980_dma_chan_data nuc980_dma_channels[] = {
	DMA_CHANNEL("ch0",  NUC980_VA_PDMA0 + 0x000, IRQ_PDMA0),
	DMA_CHANNEL("ch1",  NUC980_VA_PDMA1 + 0x000, IRQ_PDMA1),
	DMA_CHANNEL("ch2",  NUC980_VA_PDMA0 + 0x010, IRQ_PDMA0),
	DMA_CHANNEL("ch3",  NUC980_VA_PDMA1 + 0x010, IRQ_PDMA1),
	DMA_CHANNEL("ch4",  NUC980_VA_PDMA0 + 0x020, IRQ_PDMA0),
	DMA_CHANNEL("ch5",  NUC980_VA_PDMA1 + 0x020, IRQ_PDMA1),
	DMA_CHANNEL("ch6",  NUC980_VA_PDMA0 + 0x030, IRQ_PDMA0),
	DMA_CHANNEL("ch7",  NUC980_VA_PDMA1 + 0x030, IRQ_PDMA1),
	DMA_CHANNEL("ch8",  NUC980_VA_PDMA0 + 0x040, IRQ_PDMA0),
	DMA_CHANNEL("ch9",  NUC980_VA_PDMA1 + 0x040, IRQ_PDMA1),
	DMA_CHANNEL("ch10", NUC980_VA_PDMA0 + 0x050, IRQ_PDMA0),
	DMA_CHANNEL("ch11", NUC980_VA_PDMA1 + 0x050, IRQ_PDMA1),
	DMA_CHANNEL("ch12", NUC980_VA_PDMA0 + 0x060, IRQ_PDMA0),
	DMA_CHANNEL("ch13", NUC980_VA_PDMA1 + 0x060, IRQ_PDMA1),
	DMA_CHANNEL("ch14", NUC980_VA_PDMA0 + 0x070, IRQ_PDMA0),
	DMA_CHANNEL("ch15", NUC980_VA_PDMA1 + 0x070, IRQ_PDMA1),
	DMA_CHANNEL("ch16", NUC980_VA_PDMA0 + 0x080, IRQ_PDMA0),
	DMA_CHANNEL("ch17", NUC980_VA_PDMA1 + 0x080, IRQ_PDMA1),
	DMA_CHANNEL("ch18", NUC980_VA_PDMA0 + 0x090, IRQ_PDMA0),
	DMA_CHANNEL("ch19", NUC980_VA_PDMA1 + 0x090, IRQ_PDMA1),
};
struct nuc980_dma_platform_data nuc980_dma_data = {
	.channels               = nuc980_dma_channels,
	.num_channels           = ARRAY_SIZE(nuc980_dma_channels),
};


static struct platform_device nuc980_device_dma = {
	.name                   = "nuc980-dma",
	.id                     = -1,
	.num_resources      = ARRAY_SIZE(nuc980_dma_resource),
	.resource   = nuc980_dma_resource,
	.dev                    = {
		.platform_data  = &nuc980_dma_data,
	},
};

#endif


/* AUDIO controller*/
#if defined(CONFIG_SND_SOC_NUC980) || defined(CONFIG_SND_SOC_NUC980_MODULE)
static u64 nuc980_device_audio_dmamask = -1;
static struct resource nuc980_i2s_resource[] = {
	[0] = {
		.start = NUC980_PA_I2S,
		.end = NUC980_PA_I2S + NUC980_SZ_I2S - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2S,
		.end = IRQ_I2S,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device nuc980_device_audio_i2s = {
	.name = "nuc980-audio-i2s",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_i2s_resource),
	.resource = nuc980_i2s_resource,
	.dev = {
		.dma_mask = &nuc980_device_audio_dmamask,
		.coherent_dma_mask = -1,
	}
};

struct platform_device nuc980_device_audio = {
	.name = "nuc980-audio",
	.id = -1,
};

struct platform_device nuc980_device_audio_pcm = {
	.name = "nuc980-audio-pcm",
	.id = 0,
};
#endif

/* I2C */
#if defined(CONFIG_I2C_BUS_NUC980_P0) || defined(CONFIG_I2C_BUS_NUC980_P0_MODULE)
// port 0
/* I2C clients */
static struct i2c_board_info __initdata nuc980_i2c_clients0[] = {
#ifdef CONFIG_SND_SOC_NAU8822
	{I2C_BOARD_INFO("nau8822", 0x1a),},
#endif
};


static struct resource nuc980_i2c0_resource[] = {
	[0] = {
		.start = NUC980_PA_I2C0,
		.end = NUC980_PA_I2C0 + NUC980_SZ_I2C0 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C0,
		.end = IRQ_I2C0,
		.flags = IORESOURCE_IRQ,
	}
};

static struct nuc980_platform_i2c nuc980_i2c0_data = {
	.bus_num = 0,
	.bus_freq = 100000,
};

struct platform_device nuc980_device_i2c0 = {
	.name = "nuc980-i2c0",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_i2c0_resource),
	.resource = nuc980_i2c0_resource,
	.dev = {
		.platform_data = &nuc980_i2c0_data,
	}
};
#endif

#if defined(CONFIG_I2C_BUS_NUC980_P1) || defined(CONFIG_I2C_BUS_NUC980_P1_MODULE)

/* I2C clients */
static struct i2c_board_info __initdata nuc980_i2c_clients1[] = {
#if defined(CONFIG_SENSOR1_OV7725)
	{I2C_BOARD_INFO("cap1_ov7725", 0x21),},
#endif
#if defined(CONFIG_SENSOR1_OV5640)
	{I2C_BOARD_INFO("cap1_ov5640", 0x3c),},
#endif
#if defined(CONFIG_SENSOR1_NT99141)
	{I2C_BOARD_INFO("cap1_nt99141", 0x2a),},
#endif
#if defined(CONFIG_SENSOR1_NT99050)
	{I2C_BOARD_INFO("cap1_nt99050", 0x21),},
#endif
#if defined(CONFIG_SENSOR1_GC0308)
	{I2C_BOARD_INFO("cap1_gc0308", 0x21),},
#endif
};

//port 1
static struct nuc980_platform_i2c nuc980_i2c1_data = {
	.bus_num = 1,
	.bus_freq = 100000,
};

static struct resource nuc980_i2c_p1_resource[] = {
	[0] = {
		.start = NUC980_PA_I2C1,
		.end = NUC980_PA_I2C1 + NUC980_SZ_I2C1 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C1,
		.end = IRQ_I2C1,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device nuc980_device_i2c1 = {
	.name = "nuc980-i2c1",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_i2c_p1_resource),
	.resource = nuc980_i2c_p1_resource,
	.dev = {
		.platform_data = &nuc980_i2c1_data,
	}
};
#endif
#if defined(CONFIG_I2C_BUS_NUC980_P2) || defined(CONFIG_I2C_BUS_NUC980_P2_MODULE)

static struct i2c_board_info __initdata nuc980_i2c_clients2[] = {
#if defined(CONFIG_SENSOR0_OV7725)
	{I2C_BOARD_INFO("cap0_ov7725", 0x21),},
#endif
#if defined(CONFIG_SENSOR0_OV5640)
	{I2C_BOARD_INFO("cap0_ov5640", 0x3c),},
#endif
#if defined(CONFIG_SENSOR0_NT99141)
	{I2C_BOARD_INFO("cap0_nt99141", 0x2a),},
#endif
#if defined(CONFIG_SENSOR0_NT99050)
	{I2C_BOARD_INFO("cap0_nt99050", 0x21),},
#endif
#if defined(CONFIG_SENSOR0_GC0308)
	{I2C_BOARD_INFO("cap0_gc0308", 0x21),},
#endif
};

//port 2
static struct nuc980_platform_i2c nuc980_i2c2_data = {
	.bus_num = 2,
	.bus_freq = 100000,
};

static struct resource nuc980_i2c_p2_resource[] = {
	[0] = {
		.start = NUC980_PA_I2C2,
		.end = NUC980_PA_I2C2 + NUC980_SZ_I2C2 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C2,
		.end = IRQ_I2C2,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device nuc980_device_i2c2 = {
	.name = "nuc980-i2c2",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_i2c_p2_resource),
	.resource = nuc980_i2c_p2_resource,
	.dev = {
		.platform_data = &nuc980_i2c2_data,
	}
};
#endif
#if defined(CONFIG_I2C_BUS_NUC980_P3) || defined(CONFIG_I2C_BUS_NUC980_P3_MODULE)
//port 3
static struct nuc980_platform_i2c nuc980_i2c3_data = {
	.bus_num = 3,
	.bus_freq = 100000,
};

static struct resource nuc980_i2c_p3_resource[] = {
	[0] = {
		.start = NUC980_PA_I2C3,
		.end = NUC980_PA_I2C3 + NUC980_SZ_I2C3 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C3,
		.end = IRQ_I2C3,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device nuc980_device_i2c3 = {
	.name = "nuc980-i2c3",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_i2c_p3_resource),
	.resource = nuc980_i2c_p3_resource,
	.dev = {
		.platform_data = &nuc980_i2c3_data,
	}
};
#endif

/* SPI */
#if defined(CONFIG_SPI_NUC980_QSPI0) || defined(CONFIG_SPI_NUC980_QSPI0_MODULE)
/* spi device, spi flash info */
#ifdef CONFIG_MTD_M25P80
static struct mtd_partition nuc980_qspi0_flash_partitions[] = {
#ifdef CONFIG_BOARD_ETH2UART
	{
		.name = "lighttpd",
		.size = 0x0200000,
		.offset = 0x0C00000,
	},
#else
	{
		.name = "kernel",
		.size = 0x0400000,
		.offset = 0,
	},
	{
		.name = "rootfs",
		.size = 0x0400000,
		.offset = 0x0400000,
	},
#endif
};

static struct flash_platform_data nuc980_qspi0_flash_data = {
#ifdef CONFIG_BOARD_IOT
	.name = "mt29f",
#else
	.name = "m25p80",
#endif
	.parts = nuc980_qspi0_flash_partitions,
	.nr_parts = ARRAY_SIZE(nuc980_qspi0_flash_partitions),
	.type = "mx66l51235l",

};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
static struct spi_board_info nuc980_qspi0_board_info[] __initdata = {
#ifdef CONFIG_MTD_M25P80
	{
#ifdef CONFIG_BOARD_IOT
		.modalias = "mt29f",
#else
		.modalias = "m25p80",
#endif
		.max_speed_hz = 30000000,
		.bus_num = 0,
		.chip_select = 0,	//use SS0
		.platform_data = &nuc980_qspi0_flash_data,
#if defined(CONFIG_SPI_NUC980_QSPI0_NORMAL)
		.mode = (SPI_MODE_0),
#elif defined(CONFIG_SPI_NUC980_QSPI0_QUAD)
		.mode = (SPI_MODE_0 | SPI_TX_QUAD | SPI_RX_QUAD),
#endif
	},
#endif

#ifdef CONFIG_SPI_SPIDEV
	{
		.modalias = "spidev",
		.max_speed_hz = 75000000,
		.bus_num = 0,
		.chip_select = 1,	//use SS1
		.mode = SPI_MODE_0,
	},
#endif
};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
static struct nuc980_spi_info nuc980_qspi0_platform_data = {
	.num_cs = 2,
	.lsb = 0,
	.txneg = 1,
	.rxneg = 0,
	.divider = 4,
	.sleep = 0,
	.txbitlen = 8,
	.bus_num = 0,
};
#endif

static struct resource nuc980_qspi0_resource[] = {
	[0] = {
		.start = NUC980_PA_SPI0,
		.end = NUC980_PA_SPI0 + NUC980_SZ_SPI0 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI0,
		.end = IRQ_SPI0,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_qspi0 = {
	.name = "nuc980-qspi0",
	.id = 0,
	.num_resources = ARRAY_SIZE(nuc980_qspi0_resource),
	.resource = nuc980_qspi0_resource,
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
	.dev = {
		.platform_data = &nuc980_qspi0_platform_data,
	}
#endif
};
#endif

#if defined(CONFIG_SPI_NUC980_SPI0) || defined(CONFIG_SPI_NUC980_SPI0_MODULE)
/* spi device, spi flash info */

#ifdef CONFIG_MTD_M25P80
static struct mtd_partition nuc980_spi0_flash_partitions[] = {
	{
		.name = "SPI flash",
		.size = 0x0200000,
		.offset = 0,
	},
};

static struct flash_platform_data nuc980_spi0_flash_data = {
	.name = "m25p80",
	.parts = nuc980_spi0_flash_partitions,
	.nr_parts = ARRAY_SIZE(nuc980_spi0_flash_partitions),
	.type = "en25qh16",
};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
static struct spi_board_info nuc980_spi0_board_info[] __initdata = {
#ifdef CONFIG_MTD_M25P80
	{
		.modalias = "m25p80",
		.max_speed_hz = 30000000,
		.bus_num = 1,
		#if defined(CONFIG_BOARD_IOT)
		.chip_select = 1,	//use SS1
		#else
		.chip_select = 0,       //use SS0
		#endif
		.platform_data = &nuc980_spi0_flash_data,
		.mode = SPI_MODE_0,
	},
#endif
#ifdef CONFIG_SPI_SPIDEV
	{
		.modalias = "spidev",
		.max_speed_hz = 75000000,
		.bus_num = 1,
		#if defined(CONFIG_BOARD_IOT)
		.chip_select = 0,	//use SS0
		#else
		.chip_select = 1,	//use SS1
		#endif
		.mode = SPI_MODE_0,
	},
#endif
};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
static struct nuc980_spi_info nuc980_spi0_platform_data = {
	.num_cs = 2,
	.lsb = 0,
	.txneg = 1,
	.rxneg = 0,
	.divider = 4,
	.sleep = 0,
	.txbitlen = 8,
	.bus_num = 1,
};
#endif

static struct resource nuc980_spi0_resource[] = {
	[0] = {
		.start = NUC980_PA_SPI1,
		.end = NUC980_PA_SPI1 + NUC980_SZ_SPI1 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI1,
		.end = IRQ_SPI1,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_spi0 = {
	.name = "nuc980-spi0",
	.id = 0,
	.num_resources = ARRAY_SIZE(nuc980_spi0_resource),
	.resource = nuc980_spi0_resource,
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
	.dev = {
		.platform_data = &nuc980_spi0_platform_data,
	}
#endif
};
#endif


#if defined(CONFIG_SPI_NUC980_SPI1) || defined(CONFIG_SPI_NUC980_SPI1_MODULE)
/* spi device, spi flash info */

#ifdef CONFIG_MTD_M25P80
static struct mtd_partition nuc980_spi1_flash_partitions[] = {
	{
		.name = "SPI flash",
		.size = 0x0200000,
		.offset = 0,
	},
};

static struct flash_platform_data nuc980_spi1_flash_data = {
	.name = "m25p80",
	.parts = nuc980_spi1_flash_partitions,
	.nr_parts = ARRAY_SIZE(nuc980_spi1_flash_partitions),
	.type = "en25qh16",
};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
static struct spi_board_info nuc980_spi1_board_info[] __initdata = {
#ifdef CONFIG_MTD_M25P80
	{
		.modalias = "m25p80",
		.max_speed_hz = 30000000,
		.bus_num = 2,
		.chip_select = 0,	//use SS0
		.platform_data = &nuc980_spi1_flash_data,
		.mode = SPI_MODE_0,
	},
#endif
#ifdef CONFIG_SPI_SPIDEV
	{
		.modalias = "spidev",
		.max_speed_hz = 75000000,
		.bus_num = 2,
		.chip_select = 1,	//use SS1
		.mode = SPI_MODE_0,
	},
#endif
};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
static struct nuc980_spi_info nuc980_spi1_platform_data = {
	.num_cs = 2,
	.lsb = 0,
	.txneg = 1,
	.rxneg = 0,
	.divider = 4,
	.sleep = 0,
	.txbitlen = 8,
	.bus_num = 2,
};
#endif

static struct resource nuc980_spi1_resource[] = {
	[0] = {
		.start = NUC980_PA_SPI2,
		.end = NUC980_PA_SPI2 + NUC980_SZ_SPI2 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI2,
		.end = IRQ_SPI2,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_spi1 = {
	.name = "nuc980-spi1",
	.id = 0,
	.num_resources = ARRAY_SIZE(nuc980_spi1_resource),
	.resource = nuc980_spi1_resource,
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
	.dev = {
		.platform_data = &nuc980_spi1_platform_data,
	}
#endif
};
#endif

#if defined(CONFIG_NUC980_QSPI0_SLAVE) || defined(CONFIG_NUC980_QSPI0_SLAVE_MODULE)
static struct spi_board_info nuc980_qspi0_slave_board_info[] __initdata = {
	{
		.modalias = "nuc980-qspi0-slave",
		.max_speed_hz = 30000000,
		.bus_num = 0,
		.chip_select = 0,       //use SS0
		.mode = SPI_MODE_0,
	},
};

static struct nuc980_spi_info nuc980_qspi0_slave_platform_data = {
	.num_cs = 2,
	.lsb = 0,
	.txneg = 1,
	.rxneg = 0,
	.divider = 4,
	.sleep = 0,
	.txbitlen = 8,
	.bus_num = 0,
};

static struct resource nuc980_qspi0_slave_resource[] = {
	[0] = {
		.start = NUC980_PA_SPI0,
		.end = NUC980_PA_SPI0 + NUC980_SZ_SPI0 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI0,
		.end = IRQ_SPI0,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_qspi0_slave = {
	.name = "nuc980-qspi0-slave",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_qspi0_slave_resource),
	.resource = nuc980_qspi0_slave_resource,
	.dev = {
		.platform_data = &nuc980_qspi0_slave_platform_data,
	}
};
#endif

#if defined(CONFIG_NUC980_SPI0_SLAVE) || defined(CONFIG_NUC980_SPI0_SLAVE_MODULE)
static struct spi_board_info nuc980_spi0_slave_board_info[] __initdata = {
	{
		.modalias = "nuc980-spi0-slave",
		.max_speed_hz = 30000000,
		.bus_num = 1,
		.chip_select = 0,       //use SS0
		.mode = SPI_MODE_0,
	},
};

static struct nuc980_spi_info nuc980_spi0_slave_platform_data = {
	.num_cs = 2,
	.lsb = 0,
	.txneg = 1,
	.rxneg = 0,
	.divider = 4,
	.sleep = 0,
	.txbitlen = 8,
	.bus_num = 1,
};

static struct resource nuc980_spi0_slave_resource[] = {
	[0] = {
		.start = NUC980_PA_SPI1,
		.end = NUC980_PA_SPI1 + NUC980_SZ_SPI1 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI1,
		.end = IRQ_SPI1,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_spi0_slave = {
	.name = "nuc980-spi0-slave",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_spi0_slave_resource),
	.resource = nuc980_spi0_slave_resource,
	.dev = {
		.platform_data = &nuc980_spi0_slave_platform_data,
	}
};
#endif

#if defined(CONFIG_NUC980_SPI1_SLAVE) || defined(CONFIG_NUC980_SPI1_SLAVE_MODULE)
static struct spi_board_info nuc980_spi1_slave_board_info[] __initdata = {
	{
		.modalias = "nuc980-spi1-slave",
		.max_speed_hz = 30000000,
		.bus_num = 2,
		.chip_select = 0,       //use SS0
		.mode = SPI_MODE_0,
	},
};

static struct nuc980_spi_info nuc980_spi1_slave_platform_data = {
	.num_cs = 2,
	.lsb = 0,
	.txneg = 1,
	.rxneg = 0,
	.divider = 4,
	.sleep = 0,
	.txbitlen = 8,
	.bus_num = 2,
};

static struct resource nuc980_spi1_slave_resource[] = {
	[0] = {
		.start = NUC980_PA_SPI2,
		.end = NUC980_PA_SPI2 + NUC980_SZ_SPI2 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI2,
		.end = IRQ_SPI2,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_spi1_slave = {
	.name = "nuc980-spi1-slave",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_spi1_slave_resource),
	.resource = nuc980_spi1_slave_resource,
	.dev = {
		.platform_data = &nuc980_spi1_slave_platform_data,
	}
};
#endif

#if defined(CONFIG_RTC_DRV_NUC980) || defined(CONFIG_RTC_DRV_NUC980_MODULE)
static struct resource nuc980_rtc_resource[] = {
	[0] = {
		.start = NUC980_PA_RTC,
		.end = NUC980_PA_RTC + NUC980_SZ_RTC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_RTC,
		.end = IRQ_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device nuc980_device_rtc = {
	.name = "nuc980-rtc",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_rtc_resource),
	.resource = nuc980_rtc_resource,
};
#endif

#if defined(CONFIG_NUC980_CAN0) || defined(CONFIG_NUC980_CAN0_MODULE)
static struct resource nuc980_can0_resource[] = {
	[0] = {
		.start = NUC980_PA_CAN0,
		.end = NUC980_PA_CAN0 + NUC980_SZ_CAN0 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CAN0,
		.end = IRQ_CAN0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device nuc980_device_can0 = {
	.name = "nuc980-can0",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_can0_resource),
	.resource = nuc980_can0_resource,
};
#endif

#if defined(CONFIG_NUC980_CAN1) || defined(CONFIG_NUC980_CAN1_MODULE)
static struct resource nuc980_can1_resource[] = {
	[0] = {
		.start = NUC980_PA_CAN1,
		.end = NUC980_PA_CAN1 + NUC980_SZ_CAN1 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CAN1,
		.end = IRQ_CAN1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device nuc980_device_can1 = {
	.name = "nuc980-can1",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_can1_resource),
	.resource = nuc980_can1_resource,
};
#endif

#if defined(CONFIG_NUC980_CAN2) || defined(CONFIG_NUC980_CAN2_MODULE)
static struct resource nuc980_can2_resource[] = {
	[0] = {
		.start = NUC980_PA_CAN2,
		.end = NUC980_PA_CAN2 + NUC980_SZ_CAN2 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CAN2,
		.end = IRQ_CAN2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device nuc980_device_can2 = {
	.name = "nuc980-can2",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_can2_resource),
	.resource = nuc980_can2_resource,
};
#endif

#if defined(CONFIG_NUC980_CAN3) || defined(CONFIG_NUC980_CAN3_MODULE)
static struct resource nuc980_can3_resource[] = {
	[0] = {
		.start = NUC980_PA_CAN3,
		.end = NUC980_PA_CAN3 + NUC980_SZ_CAN3 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CAN3,
		.end = IRQ_CAN3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device nuc980_device_can3 = {
	.name = "nuc980-can3",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_can3_resource),
	.resource = nuc980_can3_resource,
};
#endif

#if defined(CONFIG_NUC980_EBI) || defined(CONFIG_NUC980_EBI_MODULE)
static struct resource nuc980_ebi_resource[] = {
	[0] = {
		.start = NUC980_PA_EBI,
		.end = NUC980_PA_EBI + NUC980_SZ_EBI - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device nuc980_device_ebi = {
	.name = "nuc980-ebi",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_ebi_resource),
	.resource = nuc980_ebi_resource,
};
#endif

#if defined(CONFIG_PWM_NUC980_PWM0) || defined(CONFIG_PWM_NUC980_PWM0_MODULE)
static struct pwm_lookup board_pwm0_lookup[] = {
	PWM_LOOKUP("nuc980-pwm0", 0, "pwm-backlight", NULL, 10000, PWM_POLARITY_NORMAL),
};

#if 0
static struct resource nuc980_pwm_resource[] = {
	[0] = {
		.start = NUC980_PA_PWM,
		.end = NUC980_PA_PWM + NUC980_SZ_PWM - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_PWM,
		.end = IRQ_PWM,
		.flags = IORESOURCE_IRQ,
	}
};
#endif

struct platform_device nuc980_device_pwm0_ch0 = {
	.name = "nuc980-pwm0",
	.id = 0,
};

struct platform_device nuc980_device_pwm0_ch1 = {
	.name = "nuc980-pwm0",
	.id = 1,
};

struct platform_device nuc980_device_pwm0_ch2 = {
	.name = "nuc980-pwm0",
	.id = 2,
};

struct platform_device nuc980_device_pwm0_ch3 = {
	.name = "nuc980-pwm0",
	.id = 3,
};
#endif

#if defined(CONFIG_PWM_NUC980_PWM1) || defined(CONFIG_PWM_NUC980_PWM1_MODULE)
static struct pwm_lookup board_pwm1_lookup[] = {
	PWM_LOOKUP("nuc980-pwm1", 0, "pwm-backlight", NULL, 10000, PWM_POLARITY_NORMAL),
};

struct platform_device nuc980_device_pwm1_ch0 = {
	.name = "nuc980-pwm1",
	.id = 4,
};

struct platform_device nuc980_device_pwm1_ch1 = {
	.name = "nuc980-pwm1",
	.id = 5,
};

struct platform_device nuc980_device_pwm1_ch2 = {
	.name = "nuc980-pwm1",
	.id = 6,
};

struct platform_device nuc980_device_pwm1_ch3 = {
	.name = "nuc980-pwm1",
	.id = 7,
};
#endif

#if defined(CONFIG_NUC980_WDT) || defined(CONFIG_NUC980_WDT_MODULE)
static struct resource nuc980_wdt_resource[] = {
	[0] = {
		.start = NUC980_PA_WDT,
		.end = NUC980_PA_WDT + NUC980_SZ_WDT - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_WDT,
		.end = IRQ_WDT,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_wdt = {
	.name = "nuc980-wdt",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_wdt_resource),
	.resource = nuc980_wdt_resource,
};
#endif

#if defined(CONFIG_NUC980_WWDT) || defined(CONFIG_NUC980_WWDT_MODULE)
static struct resource nuc980_wwdt_resource[] = {
	[0] = {
		.start = NUC980_PA_WWDT,
		.end = NUC980_PA_WWDT + NUC980_SZ_WWDT - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_WWDT,
		.end = IRQ_WWDT,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc980_device_wwdt = {
	.name = "nuc980-wwdt",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc980_wwdt_resource),
	.resource = nuc980_wwdt_resource,
};
#endif

#if defined(CONFIG_SCUART_NUC980) || defined(CONFIG_SCUART_NUC980_MODULE)
/* Initial serial platform data */
static struct plat_nuc980serial_port nuc980_scuart_data[] = {
	[0] = {
		.membase = NUC980_VA_SC0,
		.mapbase = NUC980_PA_SC0,
		.irq = IRQ_SMC0,
		.uartclk = 12000000,

	},
	[1] = {
		.membase = NUC980_VA_SC1,
		.mapbase = NUC980_PA_SC1,
		.irq = IRQ_SMC1,
		.uartclk = 12000000,

	},
	{},
};
#endif

#if defined(CONFIG_NUC980_SC) || defined(CONFIG_NUC980_SC_MODULE) || defined(CONFIG_SCUART_NUC980) || defined(CONFIG_SCUART_NUC980_MODULE)

#if defined(CONFIG_NUC980_SC) || defined(CONFIG_NUC980_SC_MODULE)
static struct resource nuc980_sc0_resource[] = {
	[0] = {
		.start = NUC980_PA_SC0,
		.end = NUC980_PA_SC0 + NUC980_SZ_SC0- 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SMC0,
		.end = IRQ_SMC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource nuc980_sc1_resource[] = {
	[0] = {
		.start = NUC980_PA_SC1,
		.end = NUC980_PA_SC1 + NUC980_SZ_SC1 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SMC1,
		.end = IRQ_SMC1,
		.flags = IORESOURCE_IRQ,
	},
};
#endif

struct platform_device nuc980_device_sc0 = {
	.name = "nuc980-sc",
	.id = 0,
#if defined(CONFIG_NUC980_SC) || defined(CONFIG_NUC980_SC_MODULE)
	.num_resources = ARRAY_SIZE(nuc980_sc0_resource),
	.resource = nuc980_sc0_resource,
#endif
#if defined(CONFIG_SCUART_NUC980) || defined(CONFIG_SCUART_NUC980_MODULE)
	.dev = {
		.platform_data = &nuc980_scuart_data[0],
	},
#endif
};

struct platform_device nuc980_device_sc1 = {
	.name = "nuc980-sc",
	.id = 1,
#if defined(CONFIG_NUC980_SC) || defined(CONFIG_NUC980_SC_MODULE)
	.num_resources = ARRAY_SIZE(nuc980_sc1_resource),
	.resource = nuc980_sc1_resource,
#endif
#if defined(CONFIG_SCUART_NUC980) || defined(CONFIG_SCUART_NUC980_MODULE)
	.dev = {
		.platform_data = &nuc980_scuart_data[1],
	},
#endif
};

#endif

#if defined(CONFIG_NUC980_TIMER) || defined(CONFIG_NUC980_TIMER_MODULE)
static struct resource nuc980_timer_resource[] = {
	[0] = {
		.start = IRQ_TIMER0,
		.end = IRQ_TIMER0,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start = IRQ_TIMER1,
		.end = IRQ_TIMER1,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_TIMER2,
		.end = IRQ_TIMER2,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_TIMER3,
		.end = IRQ_TIMER3,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_TIMER4,
		.end = IRQ_TIMER4,
		.flags = IORESOURCE_IRQ,
	},
	[5] = {
		.start = IRQ_TIMER5,
		.end = IRQ_TIMER5,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device nuc980_device_timer0 = {
	.name = "nuc980-timer",
	.id = 0,
	.num_resources = ARRAY_SIZE(nuc980_timer_resource),
	.resource = nuc980_timer_resource,
};

struct platform_device nuc980_device_timer1 = {
	.name = "nuc980-timer",
	.id = 1,
	.num_resources = ARRAY_SIZE(nuc980_timer_resource),
	.resource = nuc980_timer_resource,
};

struct platform_device nuc980_device_timer2 = {
	.name = "nuc980-timer",
	.id = 2,
	.num_resources = ARRAY_SIZE(nuc980_timer_resource),
	.resource = nuc980_timer_resource,
};

struct platform_device nuc980_device_timer3 = {
	.name = "nuc980-timer",
	.id = 3,
	.num_resources = ARRAY_SIZE(nuc980_timer_resource),
	.resource = nuc980_timer_resource,
};

struct platform_device nuc980_device_timer4 = {
	.name = "nuc980-timer",
	.id = 4,
	.num_resources = ARRAY_SIZE(nuc980_timer_resource),
	.resource = nuc980_timer_resource,
};

struct platform_device nuc980_device_timer5 = {
	.name = "nuc980-timer",
	.id = 5,
	.num_resources = ARRAY_SIZE(nuc980_timer_resource),
	.resource = nuc980_timer_resource,
};
#endif

#if defined(CONFIG_PINCTRL) || defined(CONFIG_PINCTRL_MODULE)
struct platform_device nuc980_device_pinctrl = {
	.name = "pinctrl-nuc980",
	.id = -1,
};
#endif

#if defined(CONFIG_GPIO_NUC980) || defined(CONFIG_GPIO_NUC980_MODULE)
#if defined(CONFIG_I2C_ALGOBIT) || defined(CONFIG_I2C_ALGOBIT_MODULE)
static struct i2c_board_info __initdata nuc980_i2c_clients4[] = {
#if defined(CONFIG_SENSOR0_OV7725) || defined(CONFIG_SENSOR1_OV7725)
	{I2C_BOARD_INFO("ov7725", 0x21),},
#endif
#if defined(CONFIG_SENSOR0_OV5640) || defined(CONFIG_SENSOR1_OV5640)
	{I2C_BOARD_INFO("ov5640", 0x3c),},
#endif
#if defined(CONFIG_SENSOR0_NT99141) || defined(CONFIG_SENSOR1_NT99141)
	{I2C_BOARD_INFO("nt99141", 0x2a),},
#endif
#if defined(CONFIG_SENSOR0_NT99050) || defined(CONFIG_SENSOR1_NT99050)
	{I2C_BOARD_INFO("nt99050", 0x21),},
#endif

};

static struct i2c_gpio_platform_data i2c_gpio_adapter_data = {
	.sda_pin = NUC980_PA0,
	.scl_pin = NUC980_PA1,
	.udelay = 1,
	.timeout = 100,
	.sda_is_open_drain = 0,	//not support open drain mode
	.scl_is_open_drain = 0,	//not support open drain mode
};

static struct platform_device i2c_gpio = {
	.name = "i2c-gpio",
	.id = 4,
	.dev = {
		.platform_data = &i2c_gpio_adapter_data,
	},
};
#endif
static struct resource nuc980_gpio_resource[] = {
	[0] = {
		.start = NUC980_PA_GPIO,
		.end = NUC980_PA_GPIO + NUC980_SZ_GPIO - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device nuc980_device_gpio = {
	.name = "nuc980-gpio",
	.id = 0,
	.num_resources = ARRAY_SIZE(nuc980_gpio_resource),
	.resource = nuc980_gpio_resource,
};

#ifndef CONFIG_OF
struct platform_device nuc980_device_eint0 = {
	.name = "nuc980-gpio",
	.id = 1,
};

struct platform_device nuc980_device_eint1 = {
	.name = "nuc980-gpio",
	.id = 2,
};

struct platform_device nuc980_device_eint2 = {
	.name = "nuc980-gpio",
	.id = 3,
};

struct platform_device nuc980_device_eint3 = {
	.name = "nuc980-gpio",
	.id = 4,
};
#endif
#endif

#if defined(CONFIG_IIO_GPIO_TRIGGER) || defined(CONFIG_IIO_GPIO_TRIGGER_MODULE)
static struct resource iio_gpio_trigger_resources[] = {
	[0] = {
		.start = IRQ_GPIO_START + NUC980_PE2,
		.end = IRQ_GPIO_START + NUC980_PE2,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device iio_gpio_trigger = {
	.name = "iio_gpio_trigger",
	.num_resources = ARRAY_SIZE(iio_gpio_trigger_resources),
	.resource = iio_gpio_trigger_resources,
};
#endif
#if defined(CONFIG_BACKLIGHT_PWM)
static struct platform_pwm_backlight_data nuc980_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 1,
	.dft_brightness = 1,
	.pwm_period_ns = 78000,
};

struct platform_device nuc980_pwm_bl = {
	.name = "pwm-backlight",
	.dev = {
		.platform_data = &nuc980_backlight_data,
	},
};
#endif
static struct platform_device *nuc980_public_dev[] __initdata = {
	&nuc980_serial_device0,

#if defined(CONFIG_NUC980_UART1) || defined(CONFIG_NUC980_UART1_MODULE)
	&nuc980_serial_device1,
#endif

#if defined(CONFIG_NUC980_UART2) || defined(CONFIG_NUC980_UART2_MODULE)
	&nuc980_serial_device2,
#endif

#if defined(CONFIG_NUC980_UART3) || defined(CONFIG_NUC980_UART3_MODULE)
	&nuc980_serial_device3,
#endif

#if defined(CONFIG_NUC980_UART4) || defined(CONFIG_NUC980_UART4_MODULE)
	&nuc980_serial_device4,
#endif

#if defined(CONFIG_NUC980_UART5) || defined(CONFIG_NUC980_UART5_MODULE)
	&nuc980_serial_device5,
#endif

#if defined(CONFIG_NUC980_UART6) || defined(CONFIG_NUC980_UART6_MODULE)
	&nuc980_serial_device6,
#endif

#if defined(CONFIG_NUC980_UART7) || defined(CONFIG_NUC980_UART7_MODULE)
	&nuc980_serial_device7,
#endif

#if defined(CONFIG_NUC980_UART8) || defined(CONFIG_NUC980_UART8_MODULE)
	&nuc980_serial_device8,
#endif

#if defined(CONFIG_NUC980_UART9) || defined(CONFIG_NUC980_UART9_MODULE)
	&nuc980_serial_device9,
#endif

#if defined(CONFIG_NUC980_CAN0) || defined(CONFIG_NUC980_CAN0_MODULE)
	&nuc980_device_can0,
#endif

#if defined(CONFIG_NUC980_CAN1) || defined(CONFIG_NUC980_CAN1_MODULE)
	&nuc980_device_can1,
#endif

#if defined(CONFIG_NUC980_CAN2) || defined(CONFIG_NUC980_CAN2_MODULE)
	&nuc980_device_can2,
#endif

#if defined(CONFIG_NUC980_CAN3) || defined(CONFIG_NUC980_CAN3_MODULE)
	&nuc980_device_can3,
#endif

#if defined(CONFIG_NUC980_EBI) || defined(CONFIG_NUC980_EBI_MODULE)
	&nuc980_device_ebi,
#endif

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	&nuc980_device_ohci,
	&nuc980_device_usb_lite0,
	&nuc980_device_usb_lite1,
	&nuc980_device_usb_lite2,
	&nuc980_device_usb_lite3,
	&nuc980_device_usb_lite4,
	&nuc980_device_usb_lite5,
#endif
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	&nuc980_device_ehci,
#endif
#if defined(CONFIG_CRYPTO_DEV_NUC980) || defined(CONFIG_CRYPTO_DEV_NUC980_MODULE)
	&nuc980_device_crypto,
	&nuc980_device_crypto_raw,
	&nuc980_device_prng,
#endif

#if defined(CONFIG_I2C_BUS_NUC980_P0) || defined(CONFIG_I2C_BUS_NUC980_P0_MODULE)
	&nuc980_device_i2c0,
#endif
#if defined(CONFIG_I2C_BUS_NUC980_P1) || defined(CONFIG_I2C_BUS_NUC980_P1_MODULE)
	&nuc980_device_i2c1,
#endif
#if defined(CONFIG_I2C_BUS_NUC980_P2) || defined(CONFIG_I2C_BUS_NUC980_P2_MODULE)
	&nuc980_device_i2c2,
#endif
#if defined(CONFIG_I2C_BUS_NUC980_P3) || defined(CONFIG_I2C_BUS_NUC980_P3_MODULE)
	&nuc980_device_i2c3,
#endif

#if defined(CONFIG_NUC980_DMA) || defined(CONFIG_NUC980_DMA_MODULE)
	&nuc980_device_dma,
#endif


#if defined(CONFIG_MMC_NUC980_SD) || defined(CONFIG_MMC_NUC980_SD_MODULE)
	&nuc980_device_sdh,
#endif
#if defined(CONFIG_MTD_NAND_NUC980) || defined(CONFIG_MTD_NAND_NUC980_MODULE) || defined(CONFIG_MMC_NUC980_FMI) || defined(CONFIG_MMC_NUC980_FMI_MODULE)
	&nuc980_device_fmi,
#endif
#if defined(CONFIG_NUC980_ETH0) || defined(CONFIG_NUC980_ETH0_MODULE)
	&nuc980_device_emac0,
#endif
#if defined(CONFIG_NUC980_ETH1) || defined(CONFIG_NUC980_ETH1_MODULE)
	&nuc980_device_emac1,
#endif
#if defined(CONFIG_PWM_NUC980_PWM0) || defined(CONFIG_PWM_NUC980_PWM0_MODULE)
	&nuc980_device_pwm0_ch0,
	&nuc980_device_pwm0_ch1,
	&nuc980_device_pwm0_ch2,
	&nuc980_device_pwm0_ch3,
#endif
#if defined(CONFIG_PWM_NUC980_PWM1) || defined(CONFIG_PWM_NUC980_PWM1_MODULE)
	&nuc980_device_pwm1_ch0,
	&nuc980_device_pwm1_ch1,
	&nuc980_device_pwm1_ch2,
	&nuc980_device_pwm1_ch3,
#endif
#if defined(CONFIG_NUC980_WDT) || defined(CONFIG_NUC980_WDT_MODULE)
	&nuc980_device_wdt,
#endif
#if defined(CONFIG_NUC980_WWDT) || defined(CONFIG_NUC980_WWDT_MODULE)
	&nuc980_device_wwdt,
#endif
#if defined(CONFIG_VIDEO0_NUC980) || defined(CONFIG_VIDEO0_NUC980_MODULE)
	&nuc980_device_cap0,
#endif
#if defined(CONFIG_VIDEO1_NUC980) || defined(CONFIG_VIDEO1_NUC980_MODULE)
	&nuc980_device_cap1,
#endif
#if defined(CONFIG_SND_SOC_NUC980) || defined(CONFIG_SND_SOC_NUC980_MODULE)
	&nuc980_device_audio_pcm,
	&nuc980_device_audio,
	&nuc980_device_audio_i2s,
#endif
#if defined(CONFIG_USB_NUC980) || defined(CONFIG_USB_NUC980_MODULE)
	&nuc980_device_usbgadget,
#endif
#if defined(CONFIG_SPI_NUC980_QSPI0) || defined(CONFIG_SPI_NUC980_QSPI0_MODULE)
	&nuc980_device_qspi0,
#endif
#if defined(CONFIG_SPI_NUC980_SPI0) || defined(CONFIG_SPI_NUC980_SPI0_MODULE)
	&nuc980_device_spi0,
#endif
#if defined(CONFIG_SPI_NUC980_SPI1) || defined(CONFIG_SPI_NUC980_SPI1_MODULE)
	&nuc980_device_spi1,
#endif

#if defined(CONFIG_NUC980_QSPI0_SLAVE) || defined(CONFIG_NUC980_QSPI0_SLAVE_MODULE)
	&nuc980_device_qspi0_slave,
#endif

#if defined(CONFIG_NUC980_SPI0_SLAVE) || defined(CONFIG_NUC980_SPI0_SLAVE_MODULE)
	&nuc980_device_spi0_slave,
#endif

#if defined(CONFIG_NUC980_SPI1_SLAVE) || defined(CONFIG_NUC980_SPI1_SLAVE_MODULE)
	&nuc980_device_spi1_slave,
#endif

#if defined(CONFIG_IIO_NUC980ADC) || defined(CONFIG_IIO_NUC980ADC_MODULE)
	&nuc980_device_nadc,
#endif

#if defined(CONFIG_NUC980_TIMER) || defined(CONFIG_NUC980_TIMER_MODULE)
	&nuc980_device_timer0,
	&nuc980_device_timer1,
	&nuc980_device_timer2,
	&nuc980_device_timer3,
	// These 2 timers are reserved for kernel clock soruce and clock event
	// So we're not populate them here unless kernle use other timers.
	//&nuc980_device_timer4,
	//&nuc980_device_timer5,
#endif
#if defined(CONFIG_PINCTRL) || defined(CONFIG_PINCTRL_MODULE)
	&nuc980_device_pinctrl,
#endif
#if defined(CONFIG_RTC_DRV_NUC980) || defined(CONFIG_RTC_DRV_NUC980_MODULE)
	&nuc980_device_rtc,
#endif
#if defined(CONFIG_GPIO_NUC980) || defined(CONFIG_GPIO_NUC980_MODULE)
	&nuc980_device_gpio,
#ifndef CONFIG_OF
	&nuc980_device_eint0,
	&nuc980_device_eint1,
	&nuc980_device_eint2,
	&nuc980_device_eint3,
#endif
#endif
#if defined(CONFIG_I2C_ALGOBIT) || defined(CONFIG_I2C_ALGOBIT_MODULE)
	&i2c_gpio,
#endif
#if defined(CONFIG_NUC980_SC) || defined(CONFIG_NUC980_SC_MODULE) || defined(CONFIG_SCUART_NUC980) || defined(CONFIG_SCUART_NUC980_MODULE)
	&nuc980_device_sc0,
	&nuc980_device_sc1,
#endif

#if defined(CONFIG_IIO_GPIO_TRIGGER) || defined(CONFIG_IIO_GPIO_TRIGGER_MODULE)
	&iio_gpio_trigger,
#endif
#if defined(CONFIG_BACKLIGHT_PWM)
	&nuc980_pwm_bl,
#endif
};

void __init nuc980_platform_init(struct platform_device **device, int size)
{
	platform_add_devices(device, size);
	platform_add_devices(nuc980_public_dev, ARRAY_SIZE(nuc980_public_dev));

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	/* register spi devices */
#if defined(CONFIG_SPI_NUC980_QSPI0) || defined(CONFIG_SPI_NUC980_QSPI0_MODULE)
	spi_register_board_info(nuc980_qspi0_board_info,
	                        ARRAY_SIZE(nuc980_qspi0_board_info));
#endif
#if defined(CONFIG_SPI_NUC980_SPI0) || defined(CONFIG_SPI_NUC980_SPI0_MODULE)
	spi_register_board_info(nuc980_spi0_board_info,
	                        ARRAY_SIZE(nuc980_spi0_board_info));
#endif
#if defined(CONFIG_SPI_NUC980_SPI1) || defined(CONFIG_SPI_NUC980_SPI1_MODULE)
	spi_register_board_info(nuc980_spi1_board_info,
	                        ARRAY_SIZE(nuc980_spi1_board_info));
#endif
#endif

#if defined(CONFIG_NUC980_QSPI0_SLAVE) || defined(CONFIG_NUC980_QSPI0_SLAVE_MODULE)
	spi_register_board_info(nuc980_qspi0_slave_board_info,
	                        ARRAY_SIZE(nuc980_qspi0_slave_board_info));
#endif

#if defined(CONFIG_NUC980_SPI0_SLAVE) || defined(CONFIG_NUC980_SPI0_SLAVE_MODULE)
	spi_register_board_info(nuc980_spi0_slave_board_info,
	                        ARRAY_SIZE(nuc980_spi0_slave_board_info));
#endif

#if defined(CONFIG_NUC980_SPI1_SLAVE) || defined(CONFIG_NUC980_SPI1_SLAVE_MODULE)
	spi_register_board_info(nuc980_spi1_slave_board_info,
	                        ARRAY_SIZE(nuc980_spi1_slave_board_info));
#endif

#if defined(CONFIG_I2C_BUS_NUC980_P0) || defined(CONFIG_I2C_BUS_NUC980_P0_MODULE)
	i2c_register_board_info(0, nuc980_i2c_clients0,
	                        sizeof(nuc980_i2c_clients0) /
	                        sizeof(struct i2c_board_info));
#endif

#if defined(CONFIG_I2C_BUS_NUC980_P1) || defined(CONFIG_I2C_BUS_NUC980_P1_MODULE)
	i2c_register_board_info(1, nuc980_i2c_clients1,
	                        sizeof(nuc980_i2c_clients1) /
	                        sizeof(struct i2c_board_info));
#endif

#if defined(CONFIG_I2C_BUS_NUC980_P2) || defined(CONFIG_I2C_BUS_NUC980_P2_MODULE)
	i2c_register_board_info(2, nuc980_i2c_clients2,
	                        sizeof(nuc980_i2c_clients2) /
	                        sizeof(struct i2c_board_info));
#endif

#if defined(CONFIG_GPIO_NUC980) || defined(CONFIG_GPIO_NUC980_MODULE)
#if defined(CONFIG_I2C_ALGOBIT) || defined(CONFIG_I2C_ALGOBIT_MODULE)
	i2c_register_board_info(4, nuc980_i2c_clients4,
	                        sizeof(nuc980_i2c_clients4) /
	                        sizeof(struct i2c_board_info));
#endif
#endif

#if defined(CONFIG_PWM_NUC980_PWM0) || defined(CONFIG_PWM_NUC980_PWM0_MODULE)
	pwm_add_table(board_pwm0_lookup, ARRAY_SIZE(board_pwm0_lookup));
#endif

#if defined(CONFIG_PWM_NUC980_PWM1) || defined(CONFIG_PWM_NUC980_PWM1_MODULE)
	pwm_add_table(board_pwm1_lookup, ARRAY_SIZE(board_pwm1_lookup));
#endif
}
