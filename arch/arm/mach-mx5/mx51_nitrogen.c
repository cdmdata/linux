/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/mtd/mtd.h>
#if defined(CONFIG_MTD)
#include <linux/mtd/map.h>
#endif
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/powerkey.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/mxc_edid.h>
#include <mach/iomux-mx51.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <linux/android_pmem.h>
#include <linux/usb/android_composite.h>
#ifdef CONFIG_KEYBOARD_GPIO
#include <linux/gpio_keys.h>
#endif
#ifdef CONFIG_MAGSTRIPE_MODULE
#include <linux/magstripe.h>
#endif

#include "devices.h"
#include "crm_regs.h"
#include "usb.h"

#define MAKE_GP(port, bit) ((port - 1) * 32 + bit)

#ifdef CONFIG_FEC
	/* Nitrogen-E, audio codec is on 1st i2c bus */
	#define SGTL5000_I2C0
	#ifdef CONFIG_TOUCHSCREEN_I2C
		#define TOUCHSCREEN_I2C_HS
	#endif
#else
	/* Nitrogen-P, audio codec is on 2nd i2c bus */
	#define SGTL5000_I2C1
	#ifdef CONFIG_TOUCHSCREEN_I2C
		#define TOUCHSCREEN_I2C0
	#endif
#endif

/*!
 * @file mach-mx51/mx51_nitrogen.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */


/*
 * differences between Nitrogen variants:
 *
 * Pad		GP		Nitrogen-P (handheld)		Nitrogen-E		Nitrogen-VM (vertical mount)
 * ----------	--------	-----------------------		--------------------	----------------------------
 * CSPI1_RDY	GP4_26		GPIO				I2C(Pic) interrupt	N/C
 * EIM_D17	GP2_1		USB/Audio clock enable		N/C			I2C(Pic) interrupt
 */
#ifdef CONFIG_NITROGEN_P
	#warning Nitrogen Portable selected
#elif defined (CONFIG_NITROGEN_E)
	#warning Nitrogen Ethernet selected
#elif defined (CONFIG_NITROGEN_VM)
	#warning Nitrogen vertical mount selected
#else
	#error Nitrogen variant not selected
#endif

#define NITROGEN_GP_1_3			(0*32 + 3)	/* GPIO_1_3 */
#define NITROGEN_GP_1_4			(0*32 + 4)	/* GPIO_1_4 */
#define NITROGEN_GP_1_5			(0*32 + 5)	/* GPIO_1_5 */
#define NITROGEN_GP_1_6			(0*32 + 6)	/* GPIO_1_6 */
#define NITROGEN_GP_1_22		(0*32 + 22)	/* GPIO_1_22 */

#define BABBAGE_SD1_CD			(0*32 + 0)	/* GPIO_1_0 */
#define BABBAGE_SD1_WP		(0*32 + 1)	/* GPIO_1_1 */
#define BABBAGE_SD2_CD_2_0		(0*32 + 4)	/* GPIO_1_4 */
#define BABBAGE_SD2_WP		(0*32 + 5)	/* GPIO_1_5 */
#define BABBAGE_SD2_CD_2_5		(0*32 + 6)	/* GPIO_1_6 */
#define BABBAGE_USBH1_HUB_RST		(0*32 + 7)	/* GPIO_1_7 */
#define BABBAGE_PMIC_INT		(0*32 + 8)	/* GPIO_1_8 */

#define GPIO_1_22			(0*32 + 22)	/* GPIO_1_22 */

#define BABBAGE_USB_CLK_EN_B		(1*32 + 1)	/* GPIO_2_1 */
#define GPIO_2_1			(1*32 + 1)	/* GPIO_2_1 */
#define BABBAGE_OSC_EN_B		(1*32 + 2)	/* GPIO_2_2 */
#define BABBAGE_PHY_RESET		(1*32 + 5)	/* GPIO_2_5 */
#define BABBAGE_CAM_RESET		(1*32 + 7)	/* GPIO_2_7 */
#define BABBAGE_FM_PWR		(1*32 + 12)	/* GPIO_2_12 */
#define BABBAGE_VGA_RESET		(1*32 + 13)	/* GPIO_2_13 */
#define BABBAGE_FEC_PHY_RESET		(1*32 + 14)	/* GPIO_2_14 */
#define BABBAGE_FM_RESET		(1*32 + 15)	/* GPIO_2_15 */
#define BABBAGE_AUDAMP_STBY		(1*32 + 17)	/* GPIO_2_17 */
#define BABBAGE_POWER_KEY		(1*32 + 21)	/* GPIO_2_21 */

#define BABBAGE_26M_OSC_EN		(2*32 + 1)	/* GPIO_3_1 */
#define BABBAGE_LVDS_POWER_DOWN	(2*32 + 3)	/* GPIO_3_3 */
#define BABBAGE_DISP_BRIGHTNESS_CTL	(2*32 + 4)	/* GPIO_3_4 */
#define BABBAGE_DVI_RESET		(2*32 + 5)	/* GPIO_3_5 */
#define BABBAGE_DVI_POWER		(2*32 + 6)	/* GPIO_3_6 */
#define BABBAGE_HEADPHONE_DET	(2*32 + 26)	/* GPIO_3_26 */
#define BABBAGE_DVI_DET		(2*32 + 28)	/* GPIO_3_28 */

#define BABBAGE_LCD_3V3_ON		(3*32 + 9)	/* GPIO_4_9 */
#define BABBAGE_LCD_5V_ON		(3*32 + 10)	/* GPIO_4_10 */
#define BABBAGE_CAM_LOW_POWER	(3*32 + 10)	/* GPIO_4_12 */
#define BABBAGE_DVI_I2C_EN		(3*32 + 14)	/* GPIO_4_14 */
#define GPIO_4_16 			(3*32 + 16)	/* GPIO_4_16 */
#define GPIO_4_17 			(3*32 + 17)	/* GPIO_4_17 */
#define BABBAGE_CSP1_SS0_GPIO		(3*32 + 24)	/* GPIO_4_24 */
#define BABBAGE_AUDIO_CLK_EN		(3*32 + 26)	/* GPIO_4_26 */
#define GPIO_4_26 			(3*32 + 26)	/* GPIO_4_26 */
#define NITROGEN_GP_4_30		(3*32 + 30)	/* GPIO_4_30 */
#define NITROGEN_GP_4_31		(3*32 + 31)	/* GPIO_4_31 */

extern int __init mx51_nitrogen_init_mc13892(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 3;

static struct pad_desc mx51nitrogen_pads[] = {
	/* UART1 */
	MX51_PAD_UART1_RXD__UART1_RXD,
	MX51_PAD_UART1_TXD__UART1_TXD,
	MX51_PAD_UART1_RTS__GPIO_4_30,
	MX51_PAD_UART1_CTS__GPIO_4_31,
	MX51_PAD_UART2_RXD__UART2_RXD,
	MX51_PAD_UART2_TXD__UART2_TXD,

	/* USB HOST1 */
	MX51_PAD_USBH1_STP__USBH1_STP,
	MX51_PAD_USBH1_CLK__USBH1_CLK,
	MX51_PAD_USBH1_DIR__USBH1_DIR,
	MX51_PAD_USBH1_NXT__USBH1_NXT,
	MX51_PAD_USBH1_DATA0__USBH1_DATA0,
	MX51_PAD_USBH1_DATA1__USBH1_DATA1,
	MX51_PAD_USBH1_DATA2__USBH1_DATA2,
	MX51_PAD_USBH1_DATA3__USBH1_DATA3,
	MX51_PAD_USBH1_DATA4__USBH1_DATA4,
	MX51_PAD_USBH1_DATA5__USBH1_DATA5,
	MX51_PAD_USBH1_DATA6__USBH1_DATA6,
	MX51_PAD_USBH1_DATA7__USBH1_DATA7,

	MX51_PAD_GPIO_1_0__GPIO_1_0,
	MX51_PAD_GPIO_1_1__GPIO_1_1,
        MX51_PAD_GPIO_1_3__GPIO_1_3,
	MX51_PAD_GPIO_1_4__GPIO_1_4,
	MX51_PAD_GPIO_1_5__GPIO_1_5,
	MX51_PAD_GPIO_1_6__GPIO_1_6,
	MX51_PAD_GPIO_1_7__GPIO_1_7,
	MX51_PAD_GPIO_1_8__GPIO_1_8,
#if (defined(CONFIG_TOUCHSCREEN_I2C) || defined(CONFIG_MMA7660)) && defined(CONFIG_NITROGEN_E)
	MX51_PAD_UART3_RXD__GPIO_1_22,
#else
	MX51_PAD_UART3_RXD__UART3_RXD,
#endif
        MX51_PAD_UART3_TXD__UART3_TXD,

	MX51_PAD_EIM_D17__GPIO_2_1,
	MX51_PAD_EIM_D18__GPIO_2_2,
	MX51_PAD_EIM_D21__GPIO_2_5,
	MX51_PAD_EIM_D23__GPIO_2_7,
	MX51_PAD_EIM_D24__I2C2_SDA,
	MX51_PAD_EIM_D27__I2C2_SCL,
	MX51_PAD_EIM_A16__GPIO_2_10,
	MX51_PAD_EIM_A17__GPIO_2_11,
	MX51_PAD_EIM_A18__GPIO_2_12,
	MX51_PAD_EIM_A19__GPIO_2_13,
	MX51_PAD_EIM_A20__GPIO_2_14,
	MX51_PAD_EIM_A21__GPIO_2_15,
	MX51_PAD_EIM_A22__GPIO_2_16,
	MX51_PAD_EIM_A23__GPIO_2_17,
	MX51_PAD_EIM_A27__GPIO_2_21,
	MX51_PAD_EIM_DTACK__GPIO_2_31,

	MX51_PAD_EIM_LBA__GPIO_3_1,
	MX51_PAD_DI1_D0_CS__GPIO_3_3,
	MX51_PAD_DISPB2_SER_DIN__GPIO_3_5,
	MX51_PAD_DISPB2_SER_DIO__GPIO_3_6,
	MX51_PAD_NANDF_CS0__GPIO_3_16,
	MX51_PAD_NANDF_CS1__GPIO_3_17,
	MX51_PAD_NANDF_D14__GPIO_3_26,
	MX51_PAD_NANDF_D12__GPIO_3_28,

	MX51_PAD_CSI2_D12__GPIO_4_9,
	MX51_PAD_CSI2_D13__GPIO_4_10,
	MX51_PAD_CSI2_D19__GPIO_4_12,
	MX51_PAD_CSI2_HSYNC__GPIO_4_14,
	MX51_PAD_CSPI1_RDY__GPIO_4_26,

	MX51_PAD_EIM_EB2__FEC_MDIO,
	MX51_PAD_EIM_EB3__FEC_RDAT1,
	MX51_PAD_EIM_CS2__FEC_RDAT2,
	MX51_PAD_EIM_CS3__FEC_RDAT3,
	MX51_PAD_EIM_CS4__FEC_RX_ER,
	MX51_PAD_EIM_CS5__FEC_CRS,
	MX51_PAD_NANDF_RB2__FEC_COL,
	MX51_PAD_NANDF_RB3__FEC_RXCLK,
	MX51_PAD_NANDF_RB6__FEC_RDAT0,
	MX51_PAD_NANDF_RB7__FEC_TDAT0,
	MX51_PAD_NANDF_CS2__FEC_TX_ER,
	MX51_PAD_NANDF_CS3__FEC_MDC,
	MX51_PAD_NANDF_CS4__FEC_TDAT1,
	MX51_PAD_NANDF_CS5__FEC_TDAT2,
	MX51_PAD_NANDF_CS6__FEC_TDAT3,
	MX51_PAD_NANDF_CS7__FEC_TX_EN,
	MX51_PAD_NANDF_RDY_INT__FEC_TX_CLK,

	MX51_PAD_GPIO_NAND__PATA_INTRQ,

	MX51_PAD_DI_GP4__DI2_PIN15,
	MX51_PAD_I2C1_CLK__HSI2C_CLK,
	MX51_PAD_I2C1_DAT__HSI2C_DAT,
	MX51_PAD_EIM_D16__I2C1_SDA,
	MX51_PAD_EIM_D19__I2C1_SCL,

	MX51_PAD_GPIO_1_2__PWM_PWMO,

	MX51_PAD_SD1_CMD__SD1_CMD,
	MX51_PAD_SD1_CLK__SD1_CLK,
	MX51_PAD_SD1_DATA0__SD1_DATA0,
	MX51_PAD_SD1_DATA1__SD1_DATA1,
	MX51_PAD_SD1_DATA2__SD1_DATA2,
	MX51_PAD_SD1_DATA3__SD1_DATA3,

	MX51_PAD_SD2_CMD__SD2_CMD,
	MX51_PAD_SD2_CLK__SD2_CLK,
	MX51_PAD_SD2_DATA0__SD2_DATA0,
	MX51_PAD_SD2_DATA1__SD2_DATA1,
	MX51_PAD_SD2_DATA2__SD2_DATA2,
	MX51_PAD_SD2_DATA3__SD2_DATA3,

	MX51_PAD_AUD3_BB_TXD__AUD3_BB_TXD,
	MX51_PAD_AUD3_BB_RXD__AUD3_BB_RXD,
	MX51_PAD_AUD3_BB_CK__AUD3_BB_CK,
	MX51_PAD_AUD3_BB_FS__AUD3_BB_FS,

	MX51_PAD_CSPI1_SS1__CSPI1_SS1,

	MX51_PAD_DI_GP3__CSI1_DATA_EN,
	MX51_PAD_CSI1_D10__CSI1_D10,
	MX51_PAD_CSI1_D11__CSI1_D11,
	MX51_PAD_CSI1_D12__CSI1_D12,
	MX51_PAD_CSI1_D13__CSI1_D13,
	MX51_PAD_CSI1_D14__CSI1_D14,
	MX51_PAD_CSI1_D15__CSI1_D15,
	MX51_PAD_CSI1_D16__CSI1_D16,
	MX51_PAD_CSI1_D17__CSI1_D17,
	MX51_PAD_CSI1_D18__CSI1_D18,
	MX51_PAD_CSI1_D19__CSI1_D19,
	MX51_PAD_CSI1_VSYNC__CSI1_VSYNC,
	MX51_PAD_CSI1_HSYNC__CSI1_HSYNC,

	MX51_PAD_OWIRE_LINE__SPDIF_OUT1,
};

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1175000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 166250000,
	 .pdf = 4,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,},
};

static struct fb_videomode video_modes[] = {
	{
	 /* NTSC TV output */
	 "TV-NTSC", 60, 720, 480, 74074,
	 122, 15,
	 18, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 0,},
	{
	 /* PAL TV output */
	 "TV-PAL", 50, 720, 576, 74074,
	 132, 11,
	 22, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 720p60 TV output */
	 "720P60", 60, 1280, 720, 13468,
	 260, 109,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /*MITSUBISHI LVDS panel */
	 "XGA", 60, 1024, 768, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	 "CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

struct cpu_wp *mx51_nitrogen_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_nitrogen_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

static u16 keymapping[16] = {
	KEY_UP, KEY_DOWN, KEY_MENU, KEY_BACK,
	KEY_RIGHT, KEY_LEFT, KEY_SELECT, KEY_ENTER,
	KEY_F1, KEY_F3, KEY_1, KEY_3,
	KEY_F2, KEY_F4, KEY_2, KEY_4,
};

static struct keypad_data keypad_plat_data = {
	.rowmax = 4,
	.colmax = 4,
	.learning = 0,
	.delay = 2,
	.matrix = keymapping,
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 78770,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 2,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.reset = mx5_vpu_reset,
};

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void mx51_nitrogen_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			{
			struct pad_desc cspi1_ss0 = MX51_PAD_CSPI1_SS0__CSPI1_SS0;

			mxc_iomux_v3_setup_pad(&cspi1_ss0);
			break;
			}
		case 0x2:
			{
			struct pad_desc cspi1_ss0_gpio = MX51_PAD_CSPI1_SS0__GPIO_4_24;

			mxc_iomux_v3_setup_pad(&cspi1_ss0_gpio);
			gpio_request(BABBAGE_CSP1_SS0_GPIO, "cspi1-gpio");
			gpio_direction_output(BABBAGE_CSP1_SS0_GPIO, 0);
			gpio_set_value(BABBAGE_CSP1_SS0_GPIO, 1 & (~status));
			break;
			}
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static void mx51_nitrogen_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			gpio_free(BABBAGE_CSP1_SS0_GPIO);
			break;

		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx51_nitrogen_gpio_spi_chipselect_active,
	.chipselect_inactive = mx51_nitrogen_gpio_spi_chipselect_inactive,
};

static struct imxi2c_platform_data mxci2c_data[] = {
	{
	.bitrate = 100000,
	}
,	{
	.bitrate = 100000,
	}
};

static struct resource mxci2c1_resources[] = {
	{
		.start = I2C1_BASE_ADDR,
		.end = I2C1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_I2C1,
		.end = MXC_INT_I2C1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource mxci2c2_resources[] = {
	{
		.start = I2C2_BASE_ADDR,
		.end = I2C2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_I2C2,
		.end = MXC_INT_I2C2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device i2c_devices[] = {
	{
		.name = "imx-i2c",
		.id = 0,
		.num_resources = ARRAY_SIZE(mxci2c1_resources),
		.resource = mxci2c1_resources,
	},
	{
		.name = "imx-i2c",
		.id = 1,
		.num_resources = ARRAY_SIZE(mxci2c2_resources),
		.resource = mxci2c2_resources,
	},
};

static struct pad_desc hs_i2c_clk_pad_gp = MX51_PAD_I2C1_CLK__GPIO_4_16 ;
static struct pad_desc hs_i2c_dat_pad_gp = MX51_PAD_I2C1_DAT__GPIO_4_17 ;
static struct pad_desc hs_i2c_clk_pad_clk = MX51_PAD_I2C1_CLK__HSI2C_CLK ;
static struct pad_desc hs_i2c_dat_pad_dat = MX51_PAD_I2C1_DAT__HSI2C_DAT ;

#define PRINT_SDA
/* Generate a pulse on the i2c clock pin. */
static void hs_i2c_clock_toggle(void)
{
	unsigned i;
	unsigned gp_clk = GPIO_4_16;
	unsigned gp_dat = GPIO_4_17;
	printk(KERN_INFO "%s\n", __FUNCTION__);
	gpio_request(gp_clk, "hs_i2c_clk");
	gpio_direction_input(gp_clk);
	mxc_iomux_v3_setup_pad(&hs_i2c_clk_pad_gp);

#ifdef PRINT_SDA
	gpio_request(gp_dat, "hs_i2c_data");
	gpio_direction_input(gp_dat);
	mxc_iomux_v3_setup_pad(&hs_i2c_dat_pad_gp);
	printk(KERN_INFO "%s dat = %i\n", __FUNCTION__, gpio_get_value(gp_dat));
#endif
	/* Send high and low on the SCL line */
	for (i = 0; i < 9; i++) {
		gpio_direction_output(gp_clk,0);
		udelay(20);
		gpio_direction_input(gp_clk);
#ifdef PRINT_SDA
		printk(KERN_INFO "%s dat = %i\n", __FUNCTION__, gpio_get_value(gp_dat));
#endif
		udelay(20);
	}

	gpio_free(gp_clk);
        mxc_iomux_v3_setup_pad(&hs_i2c_clk_pad_clk);
#ifdef PRINT_SDA
	gpio_free(gp_dat);
	mxc_iomux_v3_setup_pad(&hs_i2c_dat_pad_dat);
#endif
}

static struct imxi2c_platform_data mxci2c_hs_data = {
	.bitrate = 100000,
	.i2c_clock_toggle = hs_i2c_clock_toggle,
};

static struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "SW1",
	.lp_reg_id = "SW2",
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
	.num_wp = 3,
};

static struct mxc_dvfsper_data dvfs_per_data = {
	.reg_id = "SW2",
	.clk_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.gpc_adu = 0x0,
	.vai_mask = MXC_DVFSPMCR0_FSVAI_MASK,
	.vai_offset = MXC_DVFSPMCR0_FSVAI_OFFSET,
	.dvfs_enable_bit = MXC_DVFSPMCR0_DVFEN,
	.irq_mask = MXC_DVFSPMCR0_FSVAIM,
	.div3_offset = 0,
	.div3_mask = 0x7,
	.div3_div = 2,
	.lp_high = 1250000,
	.lp_low = 1250000,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* spdif_ext_clk source for 44.1KHz */
	.spdif_clk_48000 = 7,	/* audio osc source */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "1024x768M-16@60",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB565,
	 .mode_str = "CLAA-WVGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

static void mxc_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;
	/* Enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;

	/* Disable fuse blown */
	if (!ccm_base)
		return;

	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_MX51_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX51_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	/* DI0-LVDS */
	gpio_set_value(BABBAGE_LVDS_POWER_DOWN, 0);
	msleep(1);
	gpio_set_value(BABBAGE_LVDS_POWER_DOWN, 1);
	gpio_set_value(BABBAGE_LCD_3V3_ON, 1);
	gpio_set_value(BABBAGE_LCD_5V_ON, 1);

	/* DVI Detect */
	gpio_request(BABBAGE_DVI_DET, "dvi-detect");
	gpio_direction_input(BABBAGE_DVI_DET);

	/* WVGA Reset */
	gpio_set_value(BABBAGE_DISP_BRIGHTNESS_CTL, 1);

	if (primary_di) {
		printk(KERN_INFO "DI1 is primary\n");

		/* DI1 -> DP-BG channel: */
		mxc_fb_devices[1].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[1].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);

		/* DI0 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
	} else {
		printk(KERN_INFO "DI0 is primary\n");

		/* DI0 -> DP-BG channel: */
		mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[0].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);

		/* DI1 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
	}

	/*
	 * DI0/1 DP-FG channel:
	 */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(mxc_init_fb);

#if 0
static int handle_edid(int *pixclk)
{
	int err = 0;
	int dvi = 0;
	int fb0 = 0;
	int fb1 = 1;
	struct fb_var_screeninfo screeninfo;
	struct i2c_adapter *adp;

	memset(&screeninfo, 0, sizeof(screeninfo));

	adp = i2c_get_adapter(1);

	if (cpu_is_mx51_rev(CHIP_REV_3_0) > 0) {
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), 1);
		msleep(1);
	}
	err = read_edid(adp, &screeninfo, &dvi);
	if (cpu_is_mx51_rev(CHIP_REV_3_0) > 0)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), 0);

	if (!err) {
		printk(KERN_INFO " EDID read\n");
		if (!dvi) {
			enable_vga = 1;
			fb0 = 1; /* fb0 will be VGA */
			fb1 = 0; /* fb1 will be DVI or TV */
		}

		/* Handle TV modes */
		/* This logic is fairly complex yet still doesn't handle all
		   possibilities.  Once a customer knows the platform
		   configuration, this should be simplified to what is desired.
		 */
		if (screeninfo.xres == 1920 && screeninfo.yres != 1200) {
			/* MX51 can't handle clock speeds for anything larger.*/
			if (!enable_tv)
				enable_tv = 1;
			if (enable_vga || enable_wvga || enable_tv == 2)
				enable_tv = 2;
			fb_data[0].mode = &(video_modes[0]);
			if (!enable_wvga)
				fb_data[1].mode_str = "800x600M-16@60";
		} else if (screeninfo.xres > 1280 && screeninfo.yres > 1024) {
			if (!enable_wvga) {
				fb_data[fb0].mode_str = "1280x1024M-16@60";
				fb_data[fb1].mode_str = NULL;
			} else {
				/* WVGA is preset so the DVI can't be > this. */
				fb_data[0].mode_str = "1024x768M-16@60";
			}
		} else if (screeninfo.xres > 0 && screeninfo.yres > 0) {
			if (!enable_wvga) {
				fb_data[fb0].mode =
					kzalloc(sizeof(struct fb_videomode),
							GFP_KERNEL);
				fb_var_to_videomode(fb_data[fb0].mode,
						    &screeninfo);
				fb_data[fb0].mode_str = NULL;
				if (screeninfo.xres >= 1280 &&
						screeninfo.yres > 720)
					fb_data[fb1].mode_str = NULL;
				else if (screeninfo.xres > 1024 &&
						screeninfo.yres > 768)
					fb_data[fb1].mode_str =
						"800x600M-16@60";
				else if (screeninfo.xres > 800 &&
						screeninfo.yres > 600)
					fb_data[fb1].mode_str =
						"1024x768M-16@60";
			} else {
				/* A WVGA panel was specified and an EDID was
				   read thus there is a DVI monitor attached. */
				if (screeninfo.xres >= 1024)
					fb_data[0].mode_str = "1024x768M-16@60";
				else if (screeninfo.xres >= 800)
					fb_data[0].mode_str = "800x600M-16@60";
				else
					fb_data[0].mode_str = "640x480M-16@60";
			}
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_KEYBOARD_GPIO
static struct gpio_keys_button nitrogen_gpio_keys[] = {
#ifdef CONFIG_NITROGEN_P
	{
		.type	= EV_KEY,
		.code	= KEY_HOME,
		.gpio	= NITROGEN_GP_4_31,
		.desc	= "Home Button",
		.wakeup	= 1,
		.active_low = 1,
		.debounce_interval = 30,
	},
	{
		.type	= EV_KEY,
		.code	= KEY_BACK,
		.gpio	= NITROGEN_GP_1_4,
		.desc	= "Back Button",
		.wakeup	= 1,
		.active_low = 1,
		.debounce_interval = 30,
	},
	{
		.type	= EV_KEY,
		.code	= KEY_MENU,
		.gpio	= NITROGEN_GP_4_30,
		.desc	= "Menu Button",
		.wakeup	= 1,
		.active_low = 1,
		.debounce_interval = 30,
	},
	{
		.type	= EV_KEY,
		.code	= KEY_SEARCH,
		.gpio	= NITROGEN_GP_1_3,
		.desc	= "Search Button",
		.wakeup	= 1,
		.active_low = 1,
		.debounce_interval = 30,
	},
#else
	{
		.type	= EV_KEY,
		.code	= KEY_HOME,
		.gpio	= NITROGEN_GP_1_3,
		.desc	= "Home Button",
		.wakeup	= 1,
		.active_low = 1,
		.debounce_interval = 30,
	},
	{
		.type	= EV_KEY,
		.code	= KEY_BACK,
		.gpio	= NITROGEN_GP_1_4,
		.desc	= "Back Button",
		.wakeup	= 1,
		.active_low = 1,
		.debounce_interval = 30,
	},
	{
		.type	= EV_KEY,
		.code	= KEY_MENU,
		.gpio	= NITROGEN_GP_4_30,
		.desc	= "Menu Button",
		.wakeup	= 1,
		.active_low = 1,
		.debounce_interval = 30,
	},
	{
		.type	= EV_KEY,
		.code	= KEY_SEARCH,
		.gpio	= NITROGEN_GP_4_31,
		.desc	= "Search Button",
		.wakeup	= 1,
		.active_low = 1,
		.debounce_interval = 30,
	},
#endif
};

static struct gpio_keys_platform_data nitrogen_gpio_keys_platform_data = {
       .buttons        = nitrogen_gpio_keys,
       .nbuttons       = ARRAY_SIZE(nitrogen_gpio_keys),
};

static struct platform_device nitrogen_gpio_keys_device = {
       .name   = "gpio-keys",
       .id     = -1,
       .dev    = {
               .platform_data  = &nitrogen_gpio_keys_platform_data,
       },
};
#endif

#ifdef CONFIG_MAGSTRIPE_MODULE

static struct mag_platform_data nitrogen_mag_platform_data = {
	.front_pin = CONFIG_MAG_FRONT,
	.rear_pin = CONFIG_MAG_REAR,
	.clock_pin = CONFIG_MAG_CLOCK,
	.data_pin = CONFIG_MAG_DATA,
	.edge = CONFIG_MAG_RISING_EDGE,
	.timeout = CONFIG_MAG_TIMEOUT,
};

static struct platform_device magstripe_device = {
	.name	= "magstripe",
	.dev	= {
		.platform_data = &nitrogen_mag_platform_data
	},
};
#endif


struct plat_i2c_generic_data {
	unsigned irq;
	unsigned gp;
};

static struct plat_i2c_generic_data i2c_generic_data = {
#ifdef CONFIG_NITROGEN_VM
	.irq = IOMUX_TO_IRQ_V3(GPIO_2_1), .gp = GPIO_2_1 /* EIM_D17 Nitrogen-VM */
#elif defined(CONFIG_NITROGEN_E)
#if 1
	.irq = IOMUX_TO_IRQ_V3(GPIO_1_22), .gp = GPIO_1_22 /* temporary */
#else
	.irq = IOMUX_TO_IRQ_V3(GPIO_4_26), .gp = GPIO_4_26 /* CSPI1_RDY Nitrogen-E */
#endif
#endif
};

struct plat_i2c_tfp410_data {
	unsigned gp;
};

static struct plat_i2c_tfp410_data i2c_tfp410_data = {
	.gp = MAKE_GP(3, 5)
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
#if defined(CONFIG_MXC_CAMERA_OV3640)
	{
	.type = "ov3640",
	.addr = 0x3C,
	.platform_data = (void *)&camera_data,
	},
#endif
#ifdef SGTL5000_I2C0
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
#endif
#ifdef TOUCHSCREEN_I2C0
	{
	 .type = "Pic16F616-ts",
	 .addr = 0x22,
	 .platform_data  = &i2c_generic_data,
	},
#endif
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
#ifdef SGTL5000_I2C1
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
#endif
};

static struct i2c_board_info mxc_i2c_hs_board_info[] __initdata = {
#ifdef TOUCHSCREEN_I2C_HS
	{
	 .type = "Pic16F616-ts",
	 .addr = 0x22,
	 .platform_data  = &i2c_generic_data,
	},
#endif
	{
	 .type = "mma7660",
	 .addr = 0x4c,
	 .platform_data  = &i2c_generic_data,
	},
	{
	 .type = "tfp410",
	 .addr = 0x38,
	 .platform_data  = &i2c_tfp410_data,
	}
};

static struct mtd_partition mxc_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00040000,},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,},

};

static struct mtd_partition mxc_dataflash_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 1024 * 528,},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,},
};

static struct flash_platform_data mxc_spi_flash_data[] = {
	{
	 .name = "mxc_spi_nor",
	 .parts = mxc_spi_nor_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_spi_nor_partitions),
	 .type = "sst25vf016b",},
	{
	 .name = "mxc_dataflash",
	 .parts = mxc_dataflash_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_dataflash_partitions),
	 .type = "at45db321d",}
};

static struct spi_board_info mxc_spi_nor_device[] __initdata = {
	{
	 .modalias = "mxc_spi_nor",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[0],
	},
};

static struct spi_board_info mxc_dataflash_device[] __initdata = {
	{
	 .modalias = "mxc_dataflash",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[1],},
};

static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = gpio_get_value(BABBAGE_SD1_WP);
	else
		rc = gpio_get_value(BABBAGE_SD2_WP);

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;

	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(BABBAGE_SD1_CD);
		return ret;
	} else {		/* config the det pin for SDHC2 */
		/* BB2.5 */
		ret = gpio_get_value(BABBAGE_SD2_CD_2_5);
		return ret;
	}
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static int mxc_sgtl5000_amp_enable(int enable)
{
	gpio_set_value(BABBAGE_AUDAMP_STBY, enable ? 1 : 0);
	return 0;
}

static int headphone_det_status(void)
{
	return 1 ;
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_irq = IOMUX_TO_IRQ_V3(BABBAGE_HEADPHONE_DET),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.sysclk = 26000000,	//12288000,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

static int __initdata enable_w1 = { 0 };
static int __init w1_setup(char *__unused)
{
	enable_w1 = 1;
	return cpu_is_mx51();
}

__setup("w1", w1_setup);

static struct android_pmem_platform_data android_pmem_data = {
	.name = "pmem_adsp",
	.size = SZ_32M,
};

static struct android_pmem_platform_data android_pmem_gpu_data = {
	.name = "pmem_gpu",
	.size = SZ_32M,
	.cached = 1,
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
	"rndis",
	"usb_mass_storage",
	"adb"
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0c01,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c02,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0c03,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0c04,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_data = {
	.nluns		= 3,
	.vendor		= "Freescale",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct usb_ether_platform_data rndis_data = {
	.vendorID	= 0x0bb4,
	.vendorDescr	= "Freescale",
};

static struct android_usb_platform_data android_usb_data = {
	.vendor_id      = 0x0bb4,
	.product_id     = 0x0c01,
	.version        = 0x0100,
	.product_name   = "Android Phone",
	.manufacturer_name = "Freescale",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

#define MAX_CAMERA_FRAME_SIZE (((2592*1944*2+PAGE_SIZE-1)/PAGE_SIZE)*PAGE_SIZE)
#define MAX_CAMERA_FRAME_COUNT 4
#define MAX_CAMERA_MEM SZ_64M
//#define MAX_CAMERA_MEM ((MAX_CAMERA_FRAME_SIZE)*(MAX_CAMERA_FRAME_COUNT))

#if defined(CONFIG_VIDEO_BOUNDARY_CAMERA) || defined(CONFIG_VIDEO_BOUNDARY_CAMERA_MODULE)

static unsigned long camera_buf_phys = 0UL ;
unsigned long get_camera_phys(unsigned maxsize) {
	if (maxsize <= MAX_CAMERA_MEM)
		return camera_buf_phys ;
	else
		return 0UL ;
}
EXPORT_SYMBOL(get_camera_phys);

#endif

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_512M;
	int left_mem = 0, temp_mem = 0;
	int gpu_mem = SZ_16M;
	int fb_mem = SZ_32M;
#ifdef CONFIG_ANDROID_PMEM
	int pmem_gpu_size = android_pmem_gpu_data.size;
	int pmem_adsp_size = android_pmem_data.size;
	fb_mem = 0;
#endif

	mxc_set_cpu_type(MXC_CPU_MX51);

	get_cpu_wp = mx51_nitrogen_get_cpu_wp;
	set_num_cpu_wp = mx51_nitrogen_set_num_cpu_wp;


	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				temp_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}
			break;
		}
	}

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
#ifdef CONFIG_ANDROID_PMEM
			left_mem = total_mem - gpu_mem - pmem_gpu_size - pmem_adsp_size;
#else
			left_mem = total_mem - gpu_mem - fb_mem;
#endif
			break;
		}
	}

	if (temp_mem > 0 && temp_mem < left_mem)
		left_mem = temp_mem;

	if (mem_tag) {
#ifndef CONFIG_ANDROID_PMEM
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
#else
		android_pmem_data.start = mem_tag->u.mem.start
				+ left_mem + gpu_mem + pmem_gpu_size;
		android_pmem_gpu_data.start = mem_tag->u.mem.start
				+ left_mem + gpu_mem;
#endif
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
		gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
#endif
	}
#ifdef CONFIG_DEBUG_LL
	mx5_map_uart();
#endif
}

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)
#define USEROFFSPI (1<<3)

static void mxc_power_off(void)
{
	/* We can do power down one of two ways:
	   Set the power gating
	   Set USEROFFSPI */

	/* Set the power gate bits to power down */
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
}

/*!
 * Power Key interrupt handler.
 */
static irqreturn_t power_key_int(int irq, void *dev_id)
{
	pwrkey_callback cb = (pwrkey_callback)dev_id;

	cb((void *)1);

	if (gpio_get_value(BABBAGE_POWER_KEY))
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
	else
		set_irq_type(irq, IRQF_TRIGGER_RISING);

	return 0;
}

static void mxc_register_powerkey(pwrkey_callback pk_cb)
{
	/* Set power key as wakeup resource */
	int irq, ret;
	irq = IOMUX_TO_IRQ_V3(BABBAGE_POWER_KEY);

	if (gpio_get_value(BABBAGE_POWER_KEY))
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
	else
		set_irq_type(irq, IRQF_TRIGGER_RISING);

	ret = request_irq(irq, power_key_int, 0, "power_key", pk_cb);
	if (ret)
		pr_info("register on-off key interrupt failed\n");
	else
		enable_irq_wake(irq);
}

static int mxc_pwrkey_getstatus(int id)
{
	return gpio_get_value(BABBAGE_POWER_KEY);
}

static struct power_key_platform_data pwrkey_data = {
	.key_value = KEY_F4,
	.register_pwrkey = mxc_register_powerkey,
	.get_key_status = mxc_pwrkey_getstatus,
};

static void __init mx51_nitrogen_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx51nitrogen_pads,
					ARRAY_SIZE(mx51nitrogen_pads));

	gpio_request(BABBAGE_PMIC_INT, "pmic-int");
	gpio_request(BABBAGE_SD1_CD, "sdhc1-detect");
	gpio_request(BABBAGE_SD1_WP, "sdhc1-wp");

	gpio_direction_input(BABBAGE_PMIC_INT);
	gpio_direction_input(BABBAGE_SD1_CD);
	gpio_direction_input(BABBAGE_SD1_WP);

	/* SD2 CD for BB2.5 */
	gpio_request(BABBAGE_SD2_CD_2_5, "sdhc2-detect");
	gpio_direction_input(BABBAGE_SD2_CD_2_5);

	gpio_request(BABBAGE_SD2_WP, "sdhc2-wp");
	gpio_direction_input(BABBAGE_SD2_WP);

	gpio_request(NITROGEN_GP_1_5, "gp_1_5");
	gpio_direction_output(NITROGEN_GP_1_5,1);
	gpio_request(NITROGEN_GP_1_6, "gp_1_6");
	gpio_direction_output(NITROGEN_GP_1_6,1);

	/* reset usbh1 hub */
	gpio_request(BABBAGE_USBH1_HUB_RST, "hub-rst");
	gpio_direction_output(BABBAGE_USBH1_HUB_RST, 0);
	gpio_set_value(BABBAGE_USBH1_HUB_RST, 0);
	msleep(1);
	gpio_set_value(BABBAGE_USBH1_HUB_RST, 1);

	/* reset FEC PHY */
	gpio_request(BABBAGE_FEC_PHY_RESET, "fec-phy-reset");
	gpio_direction_output(BABBAGE_FEC_PHY_RESET, 0);
	msleep(10);
	gpio_set_value(BABBAGE_FEC_PHY_RESET, 1);

	/* reset FM */
	gpio_request(BABBAGE_FM_RESET, "fm-reset");
	gpio_direction_output(BABBAGE_FM_RESET, 0);
	msleep(10);
	gpio_set_value(BABBAGE_FM_RESET, 1);

	/* Drive 26M_OSC_EN line high */
	gpio_request(BABBAGE_26M_OSC_EN, "26m-osc-en");
	gpio_direction_output(BABBAGE_26M_OSC_EN, 1);

	gpio_request(BABBAGE_USB_CLK_EN_B, "usb-clk_en_b");
#ifdef CONFIG_NITROGEN_P
	/* Drive USB_CLK_EN_B line low */
	gpio_direction_output(BABBAGE_USB_CLK_EN_B, 1);
#else
	gpio_direction_input(BABBAGE_USB_CLK_EN_B);
	gpio_free(BABBAGE_USB_CLK_EN_B);
#endif

	/* De-assert USB PHY RESETB */
	gpio_request(BABBAGE_PHY_RESET, "usb-phy-reset");
	gpio_direction_output(BABBAGE_PHY_RESET, 1);

	/* hphone_det_b */
	gpio_request(BABBAGE_HEADPHONE_DET, "hphone-det");
	gpio_direction_input(BABBAGE_HEADPHONE_DET);

	/* audio_clk_en_b */
	gpio_request(BABBAGE_AUDIO_CLK_EN, "audio-clk-en");
	gpio_direction_output(BABBAGE_AUDIO_CLK_EN, 0);

	/* power key */
	gpio_request(BABBAGE_POWER_KEY, "power-key");
	gpio_direction_input(BABBAGE_POWER_KEY);

	if (cpu_is_mx51_rev(CHIP_REV_3_0) > 0) {
		/* DVI_I2C_ENB = 0 tristates the DVI I2C level shifter */
		gpio_request(BABBAGE_DVI_I2C_EN, "dvi-i2c-en");
		gpio_direction_output(BABBAGE_DVI_I2C_EN, 0);
	}

#ifdef CONFIG_FB_MXC_CH7026
	/* Deassert VGA reset to free i2c bus */
	gpio_request(BABBAGE_VGA_RESET, "vga-reset");
	gpio_direction_output(BABBAGE_VGA_RESET, 1);
#endif

	/* LCD related gpio */
	gpio_request(BABBAGE_DISP_BRIGHTNESS_CTL, "disp-brightness-ctl");
	gpio_request(BABBAGE_LVDS_POWER_DOWN, "lvds-power-down");
	gpio_request(BABBAGE_LCD_3V3_ON, "lcd-3v3-on");
	gpio_request(BABBAGE_LCD_5V_ON, "lcd-5v-on");
	gpio_direction_output(BABBAGE_DISP_BRIGHTNESS_CTL, 0);
	gpio_direction_output(BABBAGE_LVDS_POWER_DOWN, 0);
	gpio_direction_output(BABBAGE_LCD_3V3_ON, 0);
	gpio_direction_output(BABBAGE_LCD_5V_ON, 0);

	/* Camera reset */
	gpio_request(BABBAGE_CAM_RESET, "cam-reset");
	gpio_direction_output(BABBAGE_CAM_RESET, 1);

	/* Camera low power */
	gpio_request(BABBAGE_CAM_LOW_POWER, "cam-low-power");
	gpio_direction_output(BABBAGE_CAM_LOW_POWER, 0);

	/* OSC_EN */
	gpio_request(BABBAGE_OSC_EN_B, "osc-en");
	gpio_direction_output(BABBAGE_OSC_EN_B, 1);

	if (enable_w1) {
		/* OneWire */
		struct pad_desc onewire = MX51_PAD_OWIRE_LINE__OWIRE_LINE;
		mxc_iomux_v3_setup_pad(&onewire);
	}
}

static void I2C_gpio_init(void)
{
	gpio_request(i2c_generic_data.gp, "I2C connector int");
	gpio_direction_input(i2c_generic_data.gp);
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	I2C_gpio_init();
	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "csi_mclk1");
	mxc_ipu_data.csi_clk[1] = clk_get(NULL, "csi_mclk2");

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);
	/* SD card detect irqs */
	mxcsdhc2_device.resource[2].start = IOMUX_TO_IRQ_V3(BABBAGE_SD2_CD_2_5);
	mxcsdhc2_device.resource[2].end = IOMUX_TO_IRQ_V3(BABBAGE_SD2_CD_2_5);
	mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ_V3(BABBAGE_SD1_CD);
	mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ_V3(BABBAGE_SD1_CD);

	mxc_cpu_common_init();
	mx51_nitrogen_io_init();

	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mx51_nitrogen_init_mc13892();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&i2c_devices[0], &mxci2c_data[0]);
	mxc_register_device(&i2c_devices[1], &mxci2c_data[1]);
	mxc_register_device(&mxci2c_hs_device, &mxci2c_hs_data);
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, NULL);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&mx51_lpmode_device, NULL);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&sdram_autogating_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	mxc_register_device(&mxc_iim_device, &iim_data);
	mxc_register_device(&mxc_pwm1_device, NULL);
	mxc_register_device(&mxc_pwm1_backlight_device,&mxc_pwm_backlight_data);
	mxc_register_device(&mxc_keypad_device, &keypad_plat_data);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_ssi3_device, NULL);
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	mxc_register_device(&mxc_fec_device, NULL);
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
	mxc_register_device(&mxc_powerkey_device, &pwrkey_data);

	mxc_register_device(&mxc_android_pmem_device, &android_pmem_data);
	mxc_register_device(&mxc_android_pmem_gpu_device, &android_pmem_gpu_data);
	mxc_register_device(&usb_mass_storage_device, &mass_storage_data);
	mxc_register_device(&usb_rndis_device, &rndis_data);
	mxc_register_device(&android_usb_device, &android_usb_data);

	if (board_is_rev(BOARD_REV_2))
		/* BB2.5 */
		spi_register_board_info(mxc_dataflash_device,
					ARRAY_SIZE(mxc_dataflash_device));
	else
		/* BB2.0 */
		spi_register_board_info(mxc_spi_nor_device,
					ARRAY_SIZE(mxc_spi_nor_device));

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	i2c_register_board_info(3, mxc_i2c_hs_board_info,
				ARRAY_SIZE(mxc_i2c_hs_board_info));

	pm_power_off = mxc_power_off;

	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 2) {
		sgtl5000_data.sysclk = 26000000;
	}
	gpio_request(BABBAGE_AUDAMP_STBY, "audioamp-stdby");
	gpio_direction_output(BABBAGE_AUDAMP_STBY, 0);
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);

	mx5_usb_dr_init();
	mx5_usbh1_init();


#ifdef CONFIG_KEYBOARD_GPIO
	gpio_request(NITROGEN_GP_4_30, "gp_4_30");
	gpio_request(NITROGEN_GP_4_31, "gp_4_31");
	gpio_direction_input(NITROGEN_GP_4_30);
	gpio_direction_input(NITROGEN_GP_4_31);
	gpio_free(NITROGEN_GP_4_30);
	gpio_free(NITROGEN_GP_4_31);
	platform_device_register(&nitrogen_gpio_keys_device);
#endif

#ifdef CONFIG_MAGSTRIPE_MODULE
	platform_device_register(&magstripe_device);
#endif

	dont_sleep_yet = 0 ;
}

static void __init mx51_nitrogen_timer_init(void)
{
	struct clk *uart_clk;
printk (KERN_ERR "%s\n", __func__ );
	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

printk (KERN_ERR "%s: clocks init\n", __func__ );
	mx51_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
printk (KERN_ERR "%s: setup console 0x%08x\n", __func__,UART1_BASE_ADDR );
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx51_nitrogen_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX51_BABBAGE data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(NITROGEN_IMX51, "Boundary Devices Nitrogen MX51 Board")
	/* Maintainer: Boundary Devices */
	.phys_io	= AIPS1_BASE_ADDR,
	.io_pg_offst	= ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END