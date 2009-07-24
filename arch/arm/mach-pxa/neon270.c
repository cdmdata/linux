/*
 *  linux/arch/arm/mach-pxa/neon270.c
 *
 *  Support for the Boundary Devices Neon-270 board, a PXA270-based
 *  system with Silicon-Motion SM-501 graphics processor and SMSC
 *  LAN-91C1111 ethernet adapter.
 *
 *  Author:	Eric Nelson
 *  Created:	June 25, 2007
 *  Copyright:	Boundary Devices
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/fb.h>
#include <linux/ioport.h>
#include <linux/bootmem.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/sm501-int.h>

#include <sound/ac97_codec.h>

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/flash.h>

#include <mach/audio.h>
#include <mach/mmc.h>
#include <linux/mmc/host.h>
#include <mach/irda.h>
#include <mach/ohci.h>
#include <mach/udc.h>
#include <mach/pxa2xx-gpio.h>
#include <mach/pxa2xx-regs.h>
#include <plat/i2c.h>

#include "generic.h"
#include "devices.h"
#include "read_regs.h"

#define MMC_CARD_DETECT_GP 36
#define NEON_GPIO_USB_PULLUP 3
#define NEON_GPIO_USB_VBUS 2
#define SM501_INTERRUPT_GP 22

/* UCB1400 registers */
#define AC97_IO_DATA_REG          0x005A
#define AC97_IO_DIRECTION_REG     0x005C
#define BOUNDARY_AC97_MUTE        (0+(1<<4))
#define BOUNDARY_AC97_UNMUTE      ((1<<8)+(1<<4))
#define BOUNDARY_AC97_OUTPUTS 0x0101

extern struct snd_ac97 *pxa2xx_ac97_ac97;

static void __init neon_init_irq(void)
{
	int gpdr = GPDR(0);	//0-31
	pxa27x_init_irq();

	pxa_gpio_mode(89 | GPIO_OUT);	/* USBHPEN1 */ 
        GPCR(89) = GPIO_bit(89);

	pxa_gpio_mode(SM501_INTERRUPT_GP | GPIO_IN);
	set_irq_type(IRQ_GPIO(SM501_INTERRUPT_GP), IRQ_TYPE_EDGE_RISING); /* SM501 Interrupt, neon,neon-b board  */

#if defined(CONFIG_TOUCHSCREEN_I2C) || defined(CONFIG_TOUCHSCREEN_I2C_MODULE)
	pxa_gpio_mode(CONFIG_TOUCHSCREEN_I2C_IRQ | GPIO_IN);
	set_irq_type(IRQ_GPIO(CONFIG_TOUCHSCREEN_I2C_IRQ), IRQ_TYPE_EDGE_FALLING); /* I2C touch screen  */
#endif

	if ((gpdr & (1 << 4)) == 0)
		set_irq_type(IRQ_GPIO(4), IRQ_TYPE_EDGE_RISING);	/* UCB1400 Interrupt, neon board  */
	if ((gpdr & (1 << 23)) == 0)
		set_irq_type(IRQ_GPIO(23), IRQ_TYPE_EDGE_RISING); /* UCB1400 Interrupt, neon-b board  */
	if ((gpdr & (1 << 24)) == 0)
		set_irq_type(IRQ_GPIO(24), IRQ_TYPE_EDGE_RISING); /* 91c111 Interrupt, sm501 board  */
	set_irq_type(IRQ_GPIO(MMC_CARD_DETECT_GP), IRQ_TYPE_EDGE_FALLING); // MMC card detect
}

static void __init
fixup_neon(struct machine_desc *desc, struct tag *t,
	       char **cmdline, struct meminfo *mi)
{
	if (t->hdr.tag != ATAG_CORE) {
		unsigned long size = mi->bank[0].size;
		if ((size & 0xfffff) || (size == 0)) {
			size = 32 << 20;
			printk(KERN_ERR "!!!!! Invalid memory size\r\n");
		}
		SET_BANK(0, 0xa0000000, size);
		mi->nr_banks = 1;
	}
}

///////////////////////////////////////////////////////////////////////////////
#define PHYSICAL_CS1 0x04000000
#define PHYSICAL_CS3 0x0C000000
#define PHYSICAL_CS4 0x10000000

#define BOARD_SMC_BASE PHYSICAL_CS4

#define GP_SMC_INT 24
#define SMC_IRQ			GPIO_2_x_TO_IRQ(GP_SMC_INT)

static struct resource smc91x_resources[] = {
	[0] = {
		.name   = "smc91x-regs",
		.start	= (BOARD_SMC_BASE + 0x300),
		.end	= (BOARD_SMC_BASE + 0xfffff),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= SMC_IRQ,
		.end	= SMC_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#define fbStart     (SM501_FBSTART)
#define fbMax       0x00700000		//hi meg belongs to USB
#define usbMax      0x00100000
#define mmioStart   (fbStart+0x03E00000)
#define mmioLength  0x00200000
#define SM501_USBREG_OFFSET		0x00040000
#define fbLength    (1280*1024*2)

static struct resource sm501_resources[] = {
	[0] = {
		.start	= (fbStart),
		.end	= (fbStart+fbMax-1),
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device sm501_device = {
	.name		      = SM501FB_CLASS,
	.id		      = 0,
	.num_resources	= ARRAY_SIZE(sm501_resources),
	.resource	   = sm501_resources,
};

static struct platform_device sm501mem_device = {
	.name		= SM501MEM_CLASS,
	.num_resources	= 0,
	.resource	= 0,
};

static struct platform_device sm501alpha_device = {
	.name		= SM501ALPHA_CLASS,
	.num_resources	= 0,
	.resource	= 0,
};

static struct platform_device sm501yuv_device = {
	.name		= SM501YUV_CLASS,
	.num_resources	= 0,
	.resource	= 0,
};

static struct platform_device sm501int_device = {
	.name		= SM501INT_CLASS,
	.num_resources	= 0,
	.resource	= 0,
};

static struct platform_device sm501cmd_device = {
	.name		= SM501CMD_CLASS,
	.num_resources	= 0,
	.resource	= 0,
};

static int audio_startup(struct snd_pcm_substream *substream, void *priv)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		snd_ac97_write(pxa2xx_ac97_ac97, AC97_IO_DIRECTION_REG, BOUNDARY_AC97_OUTPUTS );
		snd_ac97_write(pxa2xx_ac97_ac97, AC97_IO_DATA_REG, BOUNDARY_AC97_UNMUTE );
	}
	return 0;
}

static void audio_shutdown(struct snd_pcm_substream *substream, void *priv)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		snd_ac97_write(pxa2xx_ac97_ac97, AC97_IO_DATA_REG, BOUNDARY_AC97_MUTE );
	}
}

static void audio_suspend(void *priv)
{
	snd_ac97_write(pxa2xx_ac97_ac97, AC97_IO_DATA_REG, BOUNDARY_AC97_MUTE );
}

static void audio_resume(void *priv)
{
	snd_ac97_write(pxa2xx_ac97_ac97, AC97_IO_DATA_REG, BOUNDARY_AC97_UNMUTE );
}

static pxa2xx_audio_ops_t audio_ops = {
	.startup	= audio_startup,
	.shutdown	= audio_shutdown,
	.suspend	= audio_suspend,
	.resume		= audio_resume,
};

static struct platform_device neon_audio_device = {
	.name		= "pxa2xx-ac97",
	.id		= 4,
	.dev		= { .platform_data = &audio_ops },
};

#ifdef CONFIG_USB_OHCI_SM501
///////////////////////////////
static struct resource sm501_ohci_resources[] = {
	[0] = {
		.start	= mmioStart,		//cfgBase
		.end	= mmioStart+0x1000-1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= mmioStart+SM501_USBREG_OFFSET,		//usbBase
		.end	= mmioStart+SM501_USBREG_OFFSET+0x1000-1,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= fbStart+fbMax,		//bufBase
		.end	= fbStart+fbMax+usbMax-1,
		.flags	= IORESOURCE_MEM,
	},
	[3] = {
		.start	= IRQ_GPIO(SM501_INTERRUPT_GP),
		.end	   = IRQ_GPIO(SM501_INTERRUPT_GP),
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device sm501_ohci_device = {
	.name		= "sm501-ohci",
	.id		= -1,
	.dev		= {
		.dma_mask = NULL,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(sm501_ohci_resources),
	.resource	= sm501_ohci_resources,
};
#endif

/////////////////////////////////////////////////////////
#define NEON_MMC_CARD_DETECT_GPIO 36
#define NEON_MMC_IRQ GPIO_2_x_TO_IRQ(NEON_MMC_CARD_DETECT_GPIO)

static int neon_mci_init(struct device *dev, irq_handler_t neonMMC_detect_int, void *data)
{
	int err;
	err = request_irq(NEON_MMC_IRQ, neonMMC_detect_int, IRQF_DISABLED, "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "neon_mci_init: MMC/SD: can't request MMC card detect IRQ\n");
		return -1;
	}
	return 0;
}

static void neon_mci_exit(struct device *dev, void *data)
{
	free_irq(NEON_MMC_IRQ, data);
}

static struct pxamci_platform_data neon_mci_platform_data = {
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.init 		= neon_mci_init,
//	.setpower 	= neon_mci_setpower,
	.exit		= neon_mci_exit,
};

#ifdef CONFIG_BACKLIGHT_LCD_SUPPORT
/*
 * Pulse Width Modulator
 */

#define PWM_CTRL0       __REG(0x40B00000)  /* PWM 0 Control Register */
#define PWM_PWDUTY0     __REG(0x40B00004)  /* PWM 0 Duty Cycle Register */
#define PWM_PERVAL0     __REG(0x40B00008)  /* PWM 0 Period Control Register */

#define PWM_CTRL1       __REG(0x40C00000)  /* PWM 1Control Register */
#define PWM_PWDUTY1     __REG(0x40C00004)  /* PWM 1 Duty Cycle Register */
#define PWM_PERVAL1     __REG(0x40C00008)  /* PWM 1 Period Control Register */

static void neon_backlight_power(int on)
{
	if (on) {
		pxa_gpio_mode(GPIO16_PWM0_MD);
		//pxa_set_cken(CKEN_PWM0, 1);
		PWM_CTRL0 = 0;
		PWM_PWDUTY0 = 0x3ff;
		PWM_PERVAL0 = 0x3ff;
	} else {
		PWM_CTRL0 = 0;
		PWM_PWDUTY0 = 0x0;
		PWM_PERVAL0 = 0x3FF;
		//pxa_set_cken(CKEN_PWM0, 0);
	}
}
#endif

static struct pxafb_mode_info display_mode __initdata = {
	.pixclock		= 110000,	//(7-1)	//was 4 then 7
	.xres			= 320,
	.yres			= 240,
	.bpp			= 16,
	.hsync_len		= 64,
	.left_margin		= 34,
	.right_margin		= 1,
	.vsync_len		= 20,
	.upper_margin		= 8,
	.lower_margin		= 3,
	.sync			= FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
};

static struct pxafb_mach_info neon_pxafb_info = {
	.num_modes      	= 1,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_PCP | LCCR3_Acb(255),
#ifdef CONFIG_BACKLIGHT_LCD_SUPPORT
	.pxafb_backlight_power	= neon_backlight_power,
#endif
};

static int neon_udc_is_connected(void)
{
	return ((GPLR(NEON_GPIO_USB_VBUS) & GPIO_bit(NEON_GPIO_USB_VBUS)) == 0);
}


static struct pxa2xx_udc_mach_info udc_info __initdata = {
	/* no connect GPIO; corgi can't tell connection status */
	.udc_is_connected	= neon_udc_is_connected,
	.gpio_pullup		= NEON_GPIO_USB_PULLUP,
	.gpio_vbus		= NEON_GPIO_USB_VBUS
};

static int neon270_ohci_init(struct device *dev)
{
	printk( KERN_ERR "%s\n", __FUNCTION__ );
	/* setup Port1 GPIO pin. */

	pxa_gpio_mode(88 | GPIO_ALT_FN_1_IN);	/* USBHPWR1 */
	pxa_gpio_mode(89 | GPIO_ALT_FN_2_OUT);	/* USBHPEN1 */

	return 0;
}

static struct pxaohci_platform_data neon270_ohci_platform_data = {
	.port_mode 	= PMM_PERPORT_MODE,
	.init 		= neon270_ohci_init,
	.flags          = ENABLE_PORT1 | ENABLE_PORT2 | ENABLE_PORT3 | POWER_CONTROL_LOW | POWER_SENSE_LOW,
};


static struct platform_device *devices[] __initdata = {
	&smc91x_device,
	&sm501mem_device,
	&sm501alpha_device,
	&sm501yuv_device,
	&sm501int_device,
	&sm501cmd_device,
	&sm501_device,
#ifdef CONFIG_USB_OHCI_SM501
	&sm501_ohci_device,
#endif
	&neon_audio_device
};

static void __init neon_init(void)
{
	pxa_set_udc_info(&udc_info);
	pxa_set_mci_info(&neon_mci_platform_data);

	(void) platform_add_devices(devices, ARRAY_SIZE(devices));

	pxa_set_ohci_info(&neon270_ohci_platform_data);
	
	neon_pxafb_info.modes = &display_mode;
	set_pxa_fb_info(&neon_pxafb_info);
	pxa_set_i2c_info(NULL);

	pxa_mode_from_registers(&pxa_device_fb);
}

#define DEBUG_SIZE (PAGE_SIZE*4)
static struct map_desc neon_io_desc[] __initdata = {
  	{	/* debugging variables */
		.virtual	=  0xf0000000,
		.pfn		= __phys_to_pfn(0),
		.length		= DEBUG_SIZE,
		.type		= MT_HIGH_VECTORS
	}
};

static void __init neon_map_io(void)
{
	void* init_maps;
	void* src=(void *)0xff000000;
	unsigned long msc0;
	pxa_map_io();

	init_maps = alloc_bootmem_low_pages(DEBUG_SIZE);
	//
	// Create a mapping for 1st pages of flash and DEBUG variables
	// This is copied to ram to allow debugging while flash is being written.
	// It also allows caching, and faster access on cache miss.
	// Also, user mode need read access because of errata in exiting SDS
	msc0 = MSC0;
	if ( (msc0 & 8) && ((msc0 & 0x80000)==0) ) {
		//is nCS0 is 16 bits wide, and nCS1 is 32 bits wide then use nCS1
		src=(void *)0xff100000;
	}
	memcpy(init_maps,src,DEBUG_SIZE);
	cpu_dcache_clean_area(init_maps,DEBUG_SIZE);

	neon_io_desc[0].pfn = __phys_to_pfn(virt_to_phys(init_maps));
	iotable_init(neon_io_desc,ARRAY_SIZE(neon_io_desc));
}

MACHINE_START(SCANPASS, "Boundary Devices Neon-270 board")
	/* Maintainer: Boundary Devices */
	.phys_io	= 0x40000000,
	.boot_params	= 0xa0000100,	/* BLOB boot parameter setting */
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= neon_map_io,
	.init_irq	= neon_init_irq,
	.fixup 		= fixup_neon,
	.timer		= &pxa_timer,
	.init_machine	= neon_init,
MACHINE_END
