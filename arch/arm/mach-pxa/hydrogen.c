/*
 *  linux/arch/arm/mach-pxa/hydrogen.c
 *
 *  Support for the Boundary Devices Hydrogen board, a
 *  PXA-270 based single board computer with on-board
 *  relays and support for an Okaya QVGA display
 *
 *  Author:	Eric Nelson
 *  Created:	June 27, 2008
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
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/bootmem.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/backlight.h>
#include <linux/dma-mapping.h>
#include <net/ax88796.h>
#include <sound/ac97_codec.h>
#include <linux/leds_pwm.h>

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

#include <mach/pxa2xx-regs.h>
//#include <mach/pxa2xx-gpio.h>
#include <mach/audio.h>
#include <mach/mmc.h>
#include <mach/gpio.h>
#include <linux/mmc/host.h>
#include <mach/irda.h>
#include <mach/ohci.h>
#include <plat/i2c.h>
#ifdef CONFIG_USB_GADGET_PXA27X
#include <mach/udc.h>
#endif
#include <mach/mfp-pxa27x.h>

#include "generic.h"
#include "devices.h"
#include "read_regs.h"

#define MMC_CARD_DETECT_GP 36
/* UCB1400 registers */
#define AC97_IO_DATA_REG          0x005A
#define AC97_IO_DIRECTION_REG     0x005C
#define BOUNDARY_AC97_MUTE        (0+(1<<4))
#define BOUNDARY_AC97_UNMUTE      ((1<<8)+(1<<4))
#define BOUNDARY_AC97_OUTPUTS 0x0101

#ifdef CONFIG_USB_GADGET_PXA27X
#define GPIO_UDC_PULLUP 3
#define GPIO_UDC_VBUS 1
#endif

#define TOUCH_SCREEN_INTERRUPT_GP 0
#define GPIO_ALT_FN_2_OUT	0x280
#define GPIO_ALT_FN_1_OUT	0x180

#define GPIO16_PWM0	16
#define GPIO17_PWM1	17
#define GPIO_OUT	0x080
#define GPIO16_PWM0_MD		(16 | GPIO_ALT_FN_2_OUT)

int pxa_gpio_mode(int gpio_mode)
{
        unsigned long flags;
        int gpio = gpio_mode & 0x7f;
        int fn = (gpio_mode & 0x300) >> 8;
        int gafr;

        local_irq_save(flags);
        if (gpio_mode & 0x80)
                GPDR(gpio) |= GPIO_bit(gpio);
        else
                GPDR(gpio) &= ~GPIO_bit(gpio);
        gafr = GAFR(gpio) & ~(0x3 << (((gpio) & 0xf)*2));
        GAFR(gpio) = gafr |  (fn  << (((gpio) & 0xf)*2));
        local_irq_restore(flags);
        return 0;
}

static struct gpio backlight_gpios[] = {
        { GPIO16_PWM0, GPIOF_OUT_INIT_HIGH, "backlight pwm" },
        { GPIO17_PWM1, GPIOF_OUT_INIT_HIGH, "backlight on/off" },
};

static void __init hydrogen_init_irq(void)
{
	int gpdr;
	gpio_request_array(ARRAY_AND_SIZE(backlight_gpios));
	gpdr = GPDR(0);	//0-31
	pxa27x_init_irq();
	set_irq_type(IRQ_GPIO(22), IRQ_TYPE_EDGE_FALLING);	//pcmcia irq
	if ((gpdr & (1 << 4)) == 0)
		set_irq_type(IRQ_GPIO(4), IRQ_TYPE_EDGE_RISING);	/* UCB1400 Interrupt, neon board  */
	if ((gpdr & (1 << 5)) == 0)
		set_irq_type(IRQ_GPIO(5), IRQ_TYPE_EDGE_RISING);	/* SM501 Interrupt, neon,neon-b board  */
	if ((gpdr & (1 << 24)) == 0)
		set_irq_type(IRQ_GPIO(24), IRQ_TYPE_EDGE_RISING);	/* 91c111 Interrupt, sm501 board  */
        gpdr &= ~(1<<12);	// set GP12 as input
#ifdef CONFIG_USB_GADGET_PXA27X
	gpdr &= ~(1<<GPIO_UDC_PULLUP); // GP3 as input (output high means present)
	gpdr &= ~(1<<GPIO_UDC_VBUS); // GP1 is an input (reads value of VBUS)
#endif
	gpdr &= ~(1<<TOUCH_SCREEN_INTERRUPT_GP);

	GPDR(0) = gpdr ;
        set_irq_type(IRQ_GPIO(12), IRQ_TYPE_EDGE_FALLING);	/* Asix */
        set_irq_type(IRQ_GPIO(3), IRQ_TYPE_EDGE_FALLING);	/* i2c touch */
}

static void __init
fixup_hydrogen(struct machine_desc *desc, struct tag *t,
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

#ifdef CONFIG_SND_AC97_CODEC
extern struct snd_ac97 *pxa2xx_ac97_ac97;

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
	.reset_gpio	= 113
};

#endif

/* Asix AX88796 10/100 ethernet controller */

static struct ax_plat_data asix_platform_data = {
	.flags		= AXFLG_HAS_EEPROM,
	.wordlength	= 2,
	.dcr_val	= 0x49,
	.rcr_val	= 0x1F,
};

#define ASIX_IRQ (IRQ_GPIO(12))

static struct resource asix_resources[] = {
	[0] = {
		.start = PXA_CS1_PHYS,
		.end   = PXA_CS1_PHYS + (0x20 * 0x20) -1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = ASIX_IRQ,
		.end   = ASIX_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct led_pwm hydrogen_pwms[] = {
	[0] = {
		.name		= "led_pwm0",
		.default_trigger = "what_default_trigger",
		.pwm_id		= 0,
		.active_low	= 0,
		.max_brightness	= 0x100,
		.pwm_period_ns	= 3822192,	/* middle "C" is 261.63 hz or */
	},
	[1] = {
		.name		= "led_pwm1",
		.default_trigger = "what_default_trigger1",
		.pwm_id		= 1,
		.active_low	= 0,
		.max_brightness	= 0x100,
		.pwm_period_ns	= 3822192,	/* middle "C" is 261.63 hz or */
	}
};

static struct led_pwm_platform_data hydrogen_led_data = {
	.num_leds = 2,
	.leds = hydrogen_pwms,
};
static struct platform_device hydrogen_leds_pwd = {
	.name	= "leds_pwm",
	.dev	= {
		.platform_data	= &hydrogen_led_data,
	},
};

static struct platform_device asix_device = {
	.name		= "ax88796",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(asix_resources),
	.resource	= asix_resources,
	.dev		= {
		.platform_data = &asix_platform_data,
	}
};

static u64 pxafb_yuv_dma_mask = DMA_BIT_MASK(32);
static struct platform_device pxafb_yuv_device = {
	.name		= "pxafb_yuv",
	.id		= 3,
	.num_resources	= 0,
	.resource	= 0,
	.dev		= {
		.dma_mask = &pxafb_yuv_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

#ifdef CONFIG_FB_PXA_HARDWARE_CURSOR
static struct platform_device pxafb_cursor = {
	.name		= "pxafb_cursor",
	.id		= 4,
	.num_resources	= 0,
	.resource	= 0,
	.dev		= {
		.dma_mask = &pxafb_yuv_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif

static struct i2c_board_info __initdata hydrogen_i2c_board_info[] = {
	{
		.type = "sgtl5000-i2c",
		.addr = 0x2a,
//		.platform_data = 
	},
};

static struct platform_device *platform_devices[] __initdata = {
        &asix_device,
        &pxafb_yuv_device,
#ifdef CONFIG_FB_PXA_HARDWARE_CURSOR
	&pxafb_cursor,
#endif
	&hydrogen_leds_pwd
};

static struct pxafb_mode_info fb_modes __initdata = {
	.pixclock = 7400000,	//(3-1)
	.xres = 480,
	.yres = 272,
	.bpp = 16,
	.hsync_len = 30,
	.left_margin = 20,
	.right_margin = 28,
	.vsync_len = 3,
	.upper_margin = 5,
	.lower_margin = 15,
	.sync = 0, // !FB_SYNC_HOR_HIGH_ACT !FB_SYNC_VERT_HIGH_ACT,
};

static struct pxafb_mach_info fb_hw = {
	.num_modes = 1,
	.lccr0 = LCCR0_Act,
	.lccr3 = LCCR3_PCP,
};

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
/*
 * Pulse Width Modulator
 */

#define PWM_CTRL0       __REG(0x40B00000)  /* PWM 0 Control Register */
#define PWM_PWDUTY0     __REG(0x40B00004)  /* PWM 0 Duty Cycle Register */
#define PWM_PERVAL0     __REG(0x40B00008)  /* PWM 0 Period Control Register */

#define PWM_CTRL1       __REG(0x40C00000)  /* PWM 1Control Register */
#define PWM_PWDUTY1     __REG(0x40C00004)  /* PWM 1 Duty Cycle Register */
#define PWM_PERVAL1     __REG(0x40C00008)  /* PWM 1 Period Control Register */

static int hydrogen_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	int duty;
	if ((bl->props.power != FB_BLANK_UNBLANK) ||
	    (bl->props.fb_blank != FB_BLANK_UNBLANK))
		brightness = 0;

	duty = brightness ^ 0x3ff;	/* on this panel the duty cycle need inverted */
	if (duty && (duty != bl->props.max_brightness)) {
		gpio_set_value(GPIO17_PWM1, 1);
		pxa_gpio_mode(GPIO16_PWM0_MD);
		pxa_gpio_mode(GPIO17_PWM1 | GPIO_OUT);
		//pxa_set_cken(CKEN_PWM0, 1);
		PWM_CTRL0 = 0;
		PWM_PWDUTY0 = duty;
		PWM_PERVAL0 = bl->props.max_brightness;
	} else {
//value is either high or low, PWM not needed
		PWM_CTRL0 = 0;
		PWM_PWDUTY0 = duty;
		PWM_PERVAL0 = bl->props.max_brightness;
		gpio_set_value(GPIO16_PWM0, (duty)? 1 : 0);
		gpio_set_value(GPIO17_PWM1, (brightness)? 1 : 0);
		pxa_gpio_mode(GPIO16_PWM0 | GPIO_OUT);
		pxa_gpio_mode(GPIO17_PWM1 | GPIO_OUT);
		//pxa_set_cken(CKEN_PWM0, 0);
	}
	return 0;
}

static int hydrogen_backlight_get_brightness(struct backlight_device *bl)
{
	return PWM_PWDUTY0 ^ 0x3ff;
}

static /*const*/ struct backlight_ops hydrogen_backlight_ops = {
	.update_status	= hydrogen_backlight_update_status,
	.get_brightness	= hydrogen_backlight_get_brightness,
};

static void __init hydrogen_backlight_register(void)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 1023;
	bl = backlight_device_register("hydrogen-bl", &pxa_device_fb.dev,
				       NULL, &hydrogen_backlight_ops, &props);
	if (IS_ERR(bl)) {
		printk(KERN_ERR "hydrogen: unable to register backlight: %ld\n",
		       PTR_ERR(bl));
		return;
	}
	bl->props.brightness = 1023;
	backlight_update_status(bl);
}
#else
#define hydrogen_backlight_register()	do { } while (0)
#endif

#ifdef CONFIG_USB_GADGET_PXA27X
static int udc_is_connected(void)
{
	int connected = ((GPLR(GPIO_UDC_VBUS) & GPIO_bit(GPIO_UDC_VBUS)) == 0);
	printk( KERN_ERR "%s: %d\n", __func__, connected );
	return connected ;
}

static void udc_command(int cmd)
{
	printk( KERN_ERR "%s: command %d\n", __func__, cmd );
	if( PXA2XX_UDC_CMD_CONNECT == cmd ) /* let host see us */{
		int mask = (1<<GPIO_UDC_PULLUP);
		GPDR(0) |= mask ;
		GPSR(0) |= mask ;
	} else if( PXA2XX_UDC_CMD_DISCONNECT == cmd ){ /* so host won't see us */
		int mask = (1<<GPIO_UDC_PULLUP);
		GPDR(0) &= ~mask ;
	} else
		printk( KERN_ERR "%s: unknown command %d\n", __func__, cmd );
}

static struct pxa2xx_udc_mach_info udc_info __initdata = {
	.udc_is_connected	= udc_is_connected,
        .udc_command		= udc_command,
        .gpio_pullup		= GPIO_UDC_PULLUP
};
#endif

static int hydrogen_mci_init(struct device *dev, irq_handler_t intHandler,
			     void *data)
{
	int err = request_irq(IRQ_GPIO(MMC_CARD_DETECT_GP), intHandler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "MMC card detect", data);
	if (err) {
		printk(KERN_ERR
		       "hydrogen_mci_init: MMC/SD: can't request MMC card detect IRQ\n");
		return -1;
	}
	printk(KERN_ERR "%s\n", __func__);
	return 0;
}

static void hydrogen_mci_setpower(struct device *dev, unsigned int vdd)
{
}

static void hydrogen_mci_exit(struct device *dev, void *data)
{
	free_irq(IRQ_GPIO(MMC_CARD_DETECT_GP), data);
}

static struct pxamci_platform_data hydrogen_mci_platform_data = {
	.ocr_mask		= MMC_VDD_32_33|MMC_VDD_33_34,
	.init			= hydrogen_mci_init,
	.setpower		= hydrogen_mci_setpower,
	.exit			= hydrogen_mci_exit,
	.gpio_card_detect	= -1,
	.gpio_card_ro		= -1,
	.gpio_power		= -1,
};

static int hydrogen_ohci_init(struct device *dev)
{
	return 0;
}

static struct pxaohci_platform_data hydrogen_ohci_platform_data = {
	.port_mode 	= PMM_PERPORT_MODE,
	.init 		= hydrogen_ohci_init,
	.flags          = ENABLE_PORT1 | ENABLE_PORT2 | ENABLE_PORT3 | POWER_CONTROL_LOW | POWER_SENSE_LOW,
};

#define ARB_CNTRL      __REG(0x48000048)  /* Arbiter Control Register */
#define ARB_CORE_PARK          (1<<24)    /* Be parked with core when idle */

static unsigned long hydrogen_pin_config[] = {
	/* LCD - 16bpp Active TFT */
//	GPIOxx_LCD_TFT_16BPP,
//	GPIO16_PWM0_OUT,	/* Backlight */

	/* MMC */
	GPIO32_MMC_CLK,
	GPIO112_MMC_CMD,
	GPIO92_MMC_DAT_0,
	GPIO109_MMC_DAT_1,
	GPIO110_MMC_DAT_2,
	GPIO111_MMC_DAT_3,

	/* USB Host Port 1 */
//	GPIO88_USBH1_PWR,
//	GPIO89_USBH1_PEN,

#ifdef CONFIG_SND_AC97_CODEC
	/* AC97 */
	GPIO28_AC97_BITCLK,
	GPIO29_AC97_SDATA_IN_0,
	GPIO30_AC97_SDATA_OUT,
	GPIO31_AC97_SYNC,
	GPIO45_AC97_SYSCLK,
#endif
#ifdef CONFIG_SND_PXA2XX_SOC_I2S
	/* I2S */
	GPIO28_I2S_BITCLK_OUT,
	GPIO29_I2S_SDATA_IN,
	GPIO30_I2S_SDATA_OUT,
	GPIO31_I2S_SYNC,
	GPIO113_I2S_SYSCLK,
#endif
	/* I2C */
	GPIO117_I2C_SCL,
	GPIO118_I2C_SDA,
};

static void __init hydrogen_init(void)
{
	unsigned gpdr ;
	pxa2xx_mfp_config(ARRAY_AND_SIZE(hydrogen_pin_config));

	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);

	/* system bus arbiter setting
	 * - Core_Park
	 * - LCD_wt:DMA_wt:CORE_Wt = 2:3:4
	 */
	ARB_CNTRL = ARB_CORE_PARK | 0x234;

#ifdef CONFIG_USB_GADGET_PXA27X
	pxa_set_udc_info(&udc_info);
#endif

	/* system bus arbiter setting
	 * - Core_Park
	 * - LCD_wt:DMA_wt:CORE_Wt = 2:3:4
	 */
	gpdr = GPDR(88);
	if (gpdr & (1 << (88 & 0x1f))) {
		/* Output, Power sense not used, make active high */
		hydrogen_ohci_platform_data.flags &= ~(POWER_CONTROL_LOW | POWER_SENSE_LOW);
	}

	printk( KERN_ERR "%s: %u devices\n", __func__, ARRAY_SIZE(platform_devices));
	fb_hw.modes = &fb_modes;
	set_pxa_fb_info(&fb_hw);
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
	hydrogen_backlight_register();

	pxa_set_mci_info(&hydrogen_mci_platform_data);
	pxa_set_ohci_info(&hydrogen_ohci_platform_data);
	clk_add_alias(NULL, "0-002a", NULL, &pxa_device_i2s.dev);
	i2c_register_board_info(0, hydrogen_i2c_board_info,
	                        ARRAY_SIZE(hydrogen_i2c_board_info));
	pxa_set_i2c_info(NULL);
#ifdef CONFIG_SND_AC97_CODEC
	pxa_set_ac97_info(&audio_ops);
#endif
	pxa_mode_from_registers(&pxa_device_fb);
}

#define DEBUG_SIZE (PAGE_SIZE*4)
static struct map_desc hydrogen_io_desc[] __initdata = {
 /* virtual      	      pfn    	    length      domain       r  w  c  b */
 { 0xfff00000, __phys_to_pfn(0x00000000), DEBUG_SIZE, MT_HIGH_VECTORS },	//DOMAIN_USER,   1, 0, 1, 1for debugging variables, DOMAIN_USER because of errata on exiting SDS
};
static void __init hydrogen_map_io(void)
{
	void* init_maps;
	void* src=(void *)0xff000000;
	pxa_map_io();

	/* initialize sleep mode regs (wake-up sources, etc) */
	PGSR0 = 0x00008800;
	PGSR1 = 0x00000002;
	PGSR2 = 0x0001FC00;
	PGSR3 = 0x00001F81;
	PWER = 0xC0000002;
	PRER = 0x00000002;
	PFER = 0x00000002;
	/*
	 * Create a mapping for 1st pages of flash and DEBUG variables
	 * This is copied to ram to allow debugging while flash is being written.
	 * It also allows caching, and faster access on cache miss.
	 * Also, user mode need read access because of errata in exiting SDS
	 */
	init_maps = alloc_bootmem_low_pages(DEBUG_SIZE);
	memcpy(init_maps,src,DEBUG_SIZE);
	cpu_dcache_clean_area(init_maps,DEBUG_SIZE);

	hydrogen_io_desc[0].pfn = __phys_to_pfn(virt_to_phys(init_maps));
	iotable_init(hydrogen_io_desc,ARRAY_SIZE(hydrogen_io_desc));
}

MACHINE_START(SCANPASS, "Boundary Devices Hydrogen Board")
	/* Maintainer: Boundary Devices */
	.phys_io = 0x40000000,
	.io_pg_offst = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params = 0xa0000100,	/* BLOB boot parameter setting */
	.map_io = hydrogen_map_io,
	.init_irq = hydrogen_init_irq,
	.fixup = fixup_hydrogen,
	.timer = &pxa_timer,
	.init_machine = hydrogen_init, 
MACHINE_END
