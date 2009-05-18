/*
 *  linux/arch/arm/mach-pxa/argon.c
 *
 *  Support for the Boundary Devices Argon board, a
 *  PXA-270 based single board computer with on-board
 *  relays and support for an Okaya QVGA display
 *
 *  Author:	Eric Nelson
 *  Created:	February 4, 2007
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
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/backlight.h>
#include <linux/dma-mapping.h>
#include <net/ax88796.h>
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

#include <mach/pxa2xx-regs.h>
#include <mach/pxa2xx-gpio.h>
#include <mach/audio.h>
#include <mach/mmc.h>
#include <mach/gpio.h>
#include <linux/mmc/host.h>
#include <mach/irda.h>
#include <mach/ohci.h>
#include <mach/i2c.h>

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

static void __init argon_init_irq(void)
{
	int gpdr = GPDR(0);	//0-31
	pxa27x_init_irq();
	set_irq_type(IRQ_GPIO(22), IRQ_TYPE_EDGE_FALLING);	//pcmcia irq
	if ((gpdr & (1 << 4)) == 0)
		set_irq_type(IRQ_GPIO(4), IRQ_TYPE_EDGE_RISING);	/* UCB1400 Interrupt, neon board  */
	if ((gpdr & (1 << 5)) == 0)
		set_irq_type(IRQ_GPIO(5), IRQ_TYPE_EDGE_RISING);	/* SM501 Interrupt, neon,neon-b board  */
	if ((gpdr & (1 << 23)) == 0)
		set_irq_type(IRQ_GPIO(23), IRQ_TYPE_EDGE_RISING);	/* UCB1400 Interrupt, neon-b board  */
	if ((gpdr & (1 << 24)) == 0)
		set_irq_type(IRQ_GPIO(24), IRQ_TYPE_EDGE_RISING);	/* 91c111 Interrupt, sm501 board  */
	set_irq_type(IRQ_GPIO(MMC_CARD_DETECT_GP), IRQ_TYPE_EDGE_FALLING);	//MMC card detect
}

static void __init
fixup_argon(struct machine_desc *desc, struct tag *t,
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
};

static struct platform_device audio_device = {
	.name		= "pxa2xx-ac97",
	.id		= -1,
	.dev		= { .platform_data = &audio_ops },
};
#endif

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

static struct platform_device *platform_devices[] __initdata = {
        &audio_device,
#ifdef CONFIG_FB_PXA_HARDWARE_CURSOR
        &pxafb_cursor,
#endif
        &pxafb_yuv_device
};

static struct pxafb_mode_info fb_modes __initdata = {
	.pixclock = 7400000,	//(3-1)
	.xres = 320,
	.yres = 240,
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

static int argon_backlight_update_status(struct backlight_device *bl)
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

static int argon_backlight_get_brightness(struct backlight_device *bl)
{
	return PWM_PWDUTY0 ^ 0x3ff;
}

static /*const*/ struct backlight_ops backlight_ops = {
	.update_status	= argon_backlight_update_status,
	.get_brightness	= argon_backlight_get_brightness,
};

static void __init backlight_register(void)
{
	struct backlight_device *bl;

	bl = backlight_device_register("hydrogen-bl", &pxa_device_fb.dev,
				       NULL, &backlight_ops);
	if (IS_ERR(bl)) {
		printk(KERN_ERR "hydrogen: unable to register backlight: %ld\n",
		       PTR_ERR(bl));
		return;
	}
	bl->props.max_brightness = 1023;
	bl->props.brightness = 1023;
	backlight_update_status(bl);
}
#else
#define backlight_register()	do { } while (0)
#endif

static int argon_mci_init(struct device *dev, irq_handler_t intHandler,
			     void *data)
{
	int err;

	/*
	 * setup GPIO for PXA27x MMC controller
	 */
	pxa_gpio_mode(GPIO32_MMCCLK_MD);
	pxa_gpio_mode(GPIO112_MMCCMD_MD);
	pxa_gpio_mode(GPIO92_MMCDAT0_MD);
	pxa_gpio_mode(GPIO109_MMCDAT1_MD);
	pxa_gpio_mode(GPIO110_MMCDAT2_MD);
	pxa_gpio_mode(GPIO111_MMCDAT3_MD);

	err =
	    request_irq(IRQ_GPIO(MMC_CARD_DETECT_GP), intHandler, IRQF_DISABLED,
			"MMC card detect", data);
	if (err) {
		printk(KERN_ERR
		       "argon_mci_init: MMC/SD: can't request MMC card detect IRQ\n");
		return -1;
	}

	return 0;
}

static void argon_mci_setpower(struct device *dev, unsigned int vdd)
{
}

static void argon_mci_exit(struct device *dev, void *data)
{
	free_irq(IRQ_GPIO(MMC_CARD_DETECT_GP), data);
}

static struct pxamci_platform_data argon_mci_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.init = argon_mci_init,
	.setpower = argon_mci_setpower,
	.exit = argon_mci_exit,
};

static int argon_ohci_init(struct device *dev)
{
	/* setup Port1 GPIO pin. */
	pxa_gpio_mode(88 | GPIO_ALT_FN_1_IN);	/* USBHPWR1 */
	pxa_gpio_mode(89 | GPIO_ALT_FN_2_OUT);	/* USBHPEN1 */

	return 0;
}

static struct pxaohci_platform_data argon_ohci_platform_data = {
	.port_mode 	= PMM_PERPORT_MODE,
	.init 		= argon_ohci_init,
	.flags          = ENABLE_PORT1 | ENABLE_PORT2 | ENABLE_PORT3 | POWER_CONTROL_LOW | POWER_SENSE_LOW,
};

static void __init argon_init(void)
{
	/* system bus arbiter setting
	 * - Core_Park
	 * - LCD_wt:DMA_wt:CORE_Wt = 2:3:4
	 */
	ARB_CNTRL = ARB_CORE_PARK | 0x234;

	pxa_gpio_mode(GPIO45_SYSCLK_AC97_MD);

	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));

	fb_hw.modes = &fb_modes;
	set_pxa_fb_info(&fb_hw);
	backlight_register();


	pxa_set_mci_info(&argon_mci_platform_data);
	pxa_set_ohci_info(&argon_ohci_platform_data);

	pxa_mode_from_registers(&pxa_device_fb);
}

static void __init argon_map_io(void)
{
	pxa_map_io();

	/* initialize sleep mode regs (wake-up sources, etc) */
	PGSR0 = 0x00008800;
	PGSR1 = 0x00000002;
	PGSR2 = 0x0001FC00;
	PGSR3 = 0x00001F81;
	PWER = 0xC0000002;
	PRER = 0x00000002;
	PFER = 0x00000002;
}

MACHINE_START(SCANPASS, "Boundary Devices Argon Board")
	/* Maintainer: Boundary Devices */
	.phys_io = 0x40000000,
	.io_pg_offst = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params = 0xa0000100,	/* BLOB boot parameter setting */
	.map_io = argon_map_io,
	.init_irq = argon_init_irq,
	.fixup = fixup_argon,
	.timer = &pxa_timer,
	.init_machine = argon_init, 
MACHINE_END
