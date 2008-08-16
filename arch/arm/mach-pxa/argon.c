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

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/flash.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/audio.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/mmc.h>
#include <linux/mmc/host.h>
#include <asm/arch/irda.h>
#include <asm/arch/ohci.h>

#include "generic.h"

#define MMC_CARD_DETECT_GP 36

static void __init argon_init_irq(void)
{
	int gpdr = GPDR(0);	//0-31
	pxa27x_init_irq();
	set_irq_type(IRQ_GPIO(22), IRQT_FALLING);	//pcmcia irq
	if ((gpdr & (1 << 4)) == 0)
		set_irq_type(IRQ_GPIO(4), IRQT_RISING);	/* UCB1400 Interrupt, neon board  */
	if ((gpdr & (1 << 5)) == 0)
		set_irq_type(IRQ_GPIO(5), IRQT_RISING);	/* SM501 Interrupt, neon,neon-b board  */
	if ((gpdr & (1 << 23)) == 0)
		set_irq_type(IRQ_GPIO(23), IRQT_RISING);	/* UCB1400 Interrupt, neon-b board  */
	if ((gpdr & (1 << 24)) == 0)
		set_irq_type(IRQ_GPIO(24), IRQT_RISING);	/* 91c111 Interrupt, sm501 board  */
	set_irq_type(IRQ_GPIO(MMC_CARD_DETECT_GP), IRQT_FALLING);	//MMC card detect
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

static struct platform_device argon_audio_device = {
	.name		= "pxa2xx-ac97",
	.id		= -1,
};

static struct platform_device *platform_devices[] __initdata = {
        &argon_audio_device,
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

	/* Set the Power Control Polarity Low and Power Sense
	   Polarity Low to active low. */
	UHCHR = (UHCHR | UHCHR_PCPL | UHCHR_PSPL) &
	    ~(UHCHR_SSEP1 | UHCHR_SSEP2 | UHCHR_SSEP3 | UHCHR_SSE);

	return 0;
}

static struct pxaohci_platform_data argon_ohci_platform_data = {
	.port_mode = PMM_PERPORT_MODE,
	.init = argon_ohci_init,
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

	pxa_set_mci_info(&argon_mci_platform_data);
	pxa_set_ohci_info(&argon_ohci_platform_data);
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
