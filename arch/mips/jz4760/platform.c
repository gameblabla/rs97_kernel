/*
 * Platform device support for Jz4760 SoC.
 *
 * Copyright 2007, <yliu@ingenic.cn>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/jzsoc.h>
#include <linux/usb/musb.h>
#include <linux/spi/spi.h>

extern void __init board_msc_init(void);

/* OHCI (USB full speed host controller) */
static struct resource jz_usb_ohci_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UHC_BASE), // phys addr for ioremap
		.end		= CPHYSADDR(UHC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_UHC,
		.end		= IRQ_UHC,
		.flags		= IORESOURCE_IRQ,
	},
};

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static struct platform_device jz_usb_ohci_device = {
	.name		= "jz-ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_ohci_resources),
	.resource	= jz_usb_ohci_resources,
};

/*** LCD controller ***/
static struct resource jz_lcd_resources[] = {
	[0] = {
		.start          = CPHYSADDR(LCD_BASE),
		.end            = CPHYSADDR(LCD_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_LCD,
		.end            = IRQ_LCD,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_lcd_dmamask = ~(u32)0;

static struct platform_device jz_lcd_device = {
	.name           = "jz-lcd",
	.id             = 0,
	.dev = {
		.dma_mask               = &jz_lcd_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_lcd_resources),
	.resource       = jz_lcd_resources,
};

/* USB OTG Controller */
static struct platform_device jz_usb_otg_xceiv_device = {
	.name	= "nop_usb_xceiv",
	.id	= 0,
};

static struct musb_hdrc_config jz_usb_otg_config = {
	.multipoint	= 1,
	.dyn_fifo	= 0,
	.soft_con	= 1,
	.dma		= 1,
/* Max EPs scanned. Driver will decide which EP can be used automatically. */
	.num_eps	= 6,
};

static struct musb_hdrc_platform_data jz_usb_otg_platform_data = {
#if defined(CONFIG_USB_MUSB_OTG)
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.config		= &jz_usb_otg_config,
};

static struct resource jz_usb_otg_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UDC_BASE),
		.end		= CPHYSADDR(UDC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_OTG,
		.end		= IRQ_OTG,
		.flags		= IORESOURCE_IRQ,
	},
};

static u64  usb_otg_dmamask = ~(u32)0;

static struct platform_device jz_usb_otg_device = {
	.name	= "musb_hdrc",
	.id	= 0,
	.dev = {
		.dma_mask		= &usb_otg_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &jz_usb_otg_platform_data,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_otg_resources),
	.resource	= jz_usb_otg_resources,
};

/** MMC/SD/SDIO controllers**/
#define __BUILD_JZ_MSC_PLATFORM_DEV(msc_id)				\
	static struct resource jz_msc##msc_id##_resources[] = {		\
		{							\
			.start          = CPHYSADDR(MSC##msc_id##_BASE), \
			.end            = CPHYSADDR(MSC##msc_id##_BASE) + 0x1000 - 1, \
			.flags          = IORESOURCE_MEM,		\
		},							\
		{							\
			.start          = IRQ_MSC##msc_id,		\
			.end            = IRQ_MSC##msc_id,		\
			.flags          = IORESOURCE_IRQ,		\
		},							\
		{							\
			.start          = DMA_ID_MSC##msc_id,	\
			.end            = DMA_ID_MSC##msc_id,	\
			.flags          = IORESOURCE_DMA,		\
		},							\
	};								\
									\
	static u64 jz_msc##msc_id##_dmamask =  ~(u32)0;			\
									\
	static struct platform_device jz_msc##msc_id##_device = {	\
		.name = "jz-msc",					\
		.id = msc_id,						\
		.dev = {						\
			.dma_mask               = &jz_msc##msc_id##_dmamask, \
			.coherent_dma_mask      = 0xffffffff,		\
		},							\
		.num_resources  = ARRAY_SIZE(jz_msc##msc_id##_resources), \
		.resource       = jz_msc##msc_id##_resources,		\
	};

#ifdef CONFIG_JZ_MSC0
__BUILD_JZ_MSC_PLATFORM_DEV(0)
#endif
#ifdef CONFIG_JZ_MSC1
__BUILD_JZ_MSC_PLATFORM_DEV(1)
#endif
#ifdef CONFIG_JZ_MSC2
__BUILD_JZ_MSC_PLATFORM_DEV(2)
#endif

static struct platform_device *jz_msc_devices[] __initdata = {
#ifdef CONFIG_JZ_MSC0
	&jz_msc0_device,
#else
	NULL,
#endif
#ifdef CONFIG_JZ_MSC1
	&jz_msc1_device,
#else
	NULL,
#endif
#ifdef CONFIG_JZ_MSC2
	&jz_msc2_device,
#else
	NULL,
#endif
};

int __init jz_add_msc_devices(unsigned int id, struct jz_mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (JZ_MSC_ID_INVALID(id))
		return -EINVAL;

	pdev = jz_msc_devices[id];
	if (NULL == pdev) {
		return -EINVAL;
	}

	pdev->dev.platform_data = plat;

	return platform_device_register(pdev);
}

static struct platform_device vogue_snd_device = {
	.name = "mixer",
	.id = -1,
	.dev = {
		.platform_data = &vogue_snd_device,
	},
};

static struct resource jz_i2c0_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C0_BASE),
		.end            = CPHYSADDR(I2C0_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C0,
		.end            = IRQ_I2C0,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct resource jz_i2c1_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C1_BASE),
		.end            = CPHYSADDR(I2C1_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C1,
		.end            = IRQ_I2C1,
		.flags          = IORESOURCE_IRQ,
	},
};

static u64 jz_i2c_dmamask =  ~(u32)0;

static struct platform_device jz_i2c0_device = {
	.name = "jz_i2c0",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c0_resources),
	.resource       = jz_i2c0_resources,
};

static struct platform_device jz_i2c1_device = {
	.name = "jz_i2c1",
	.id = 1,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c1_resources),
	.resource       = jz_i2c1_resources,
};

static struct platform_device rtc_device = {
	.name		= "jz4760-rtc",
	.id		= -1,
};
///////////////////////////////////
/* SSI controller --- SPI (0) */
#ifndef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
#define __jz_spi0_board_info 	NULL
#define __jz_spi1_board_info 	NULL
#else
extern struct spi_board_info jz4760_spi0_board_info[];
extern struct spi_board_info jz4760_spi1_board_info[];
#define __jz_spi0_board_info 	&jz4760_spi0_board_info[0]
#define __jz_spi1_board_info 	&jz4760_spi1_board_info[0]
#endif

/** AX88796C controller **/
static struct resource ax88796c_resources[] = {
	[0] = {
		.start          = CPHYSADDR(0xb4000000),
		.end            = CPHYSADDR(0xb4000000) + 0x6800 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = GPIO_NET_INT + IRQ_GPIO_0,
		.end            = GPIO_NET_INT + IRQ_GPIO_0,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 ax88796c_dmamask =  ~(u32)0;

static struct platform_device ax88796c_dev = {
	.name = "ax88796c",
	.id = 0,
	.dev = {
		.dma_mask               = &ax88796c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(ax88796c_resources),
	.resource       = ax88796c_resources,
};

struct jz47xx_spi_info spi0_info_cfg = {
	.chnl = 0,
	.bus_num = 0,
	.is_pllclk = 1,
	.board_size = 2,				/* spi������?��y??*/
#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
	.board_info = __jz_spi0_board_info,
#else
	.board_info = NULL,
#endif
//	.set_cs = spi_gpio_cs,
	.set_cs = NULL,
	.pin_cs ={
		PIN_SSI_CE0,
//		32*2+31,				/*apus: GPC31 --- SW6 --- BOOT_SEL1 (dummy, example) */
		32*4+16,				/*lepus: TP56 */
	},
};
static struct resource jz_spi0_resource[] = {
	[0] = {
		.start          = CPHYSADDR(SSI0_BASE),
		.end            = CPHYSADDR(SSI0_BASE) + 0x2000 - 1,
		.flags 			= IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SSI0,
		.end   = IRQ_SSI0,
		.flags = IORESOURCE_IRQ,
	}
};
static u64 jz_spi0_dmamask =  ~(u32)0;

struct platform_device jz_spi0_device = {
	.name		  = "jz47xx-spi0",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(jz_spi0_resource),
	.resource	  = jz_spi0_resource,
        .dev              = {
                .dma_mask = &jz_spi0_dmamask,
                .coherent_dma_mask = 0xffffffffUL,
                .platform_data = & spi0_info_cfg,
        }
};

/* SSI controller --- SPI (1) */
struct jz47xx_spi_info spi1_info_cfg = {
	.chnl = 1,
	.bus_num = 1,
	.board_size = 1,
#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
	.board_info = __jz_spi1_board_info,
#else
	.board_info = NULL,
#endif
//	.set_cs = spi_gpio_cs,
	.set_cs = NULL,
	.pins_config = NULL,
	.pin_cs ={
		PIN_SSI_CE0,
	},
};
static struct resource jz_spi1_resource[] = {
	[0] = {
		.start          = CPHYSADDR(SSI1_BASE) + 0x2000,
		.end            = CPHYSADDR(SSI1_BASE) + 0x4000 - 1,
		.flags 			= IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SSI1,
		.end   = IRQ_SSI1,
		.flags = IORESOURCE_IRQ,
	}
};
static u64 jz_spi1_dmamask =  ~(u32)0;

struct platform_device jz_spi1_device = {
	.name		  = "jz47xx-spi1",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(jz_spi1_resource),
	.resource	  = jz_spi1_resource,
        .dev              = {
                .dma_mask = &jz_spi1_dmamask,
                .coherent_dma_mask = 0xffffffffUL,
                .platform_data = & spi1_info_cfg,
        }
};

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
	&jz_usb_ohci_device,
	&jz_usb_otg_xceiv_device,
	&jz_usb_otg_device,
	&jz_lcd_device,
	&vogue_snd_device,
	&jz_i2c0_device,
	&jz_i2c1_device,
	// &jz_msc0_device,
	// &jz_msc1_device,
	&rtc_device,
	&jz_spi0_device,
	&jz_spi1_device,
	&ax88796c_dev,
};

extern void __init board_i2c_init(void);
extern void __init board_spi_init(void);
static int __init jz_platform_init(void)
{
	int ret = 0;

	board_i2c_init();
#ifndef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
	board_spi_init();
#endif

	ret = platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));
#ifdef CONFIG_ANDROID_PMEM
	platform_pmem_device_setup();
#endif

	printk("jz_platform_init\n");
	board_msc_init();
	return ret;
}

arch_initcall(jz_platform_init);

