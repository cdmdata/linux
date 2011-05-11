/*
 * IR Touch serial touch screen driver
 *
 * Copyright (c) 2010 Boundary Devices
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <mach/mxc_uart.h>

#define DRIVER_DESC	"IR Touch serial touch screen driver"
#define DRIVER_NAME "irtouch"

MODULE_AUTHOR("Boundary Devices <info@boundarydevices.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
#define MXC_ISR_PASS_LIMIT      256
#define CKSUM_START ((signed char)0xAA)
#define XRESOLUTION	4096
#define YRESOLUTION	4096

struct irtouch_data_t {
	struct input_dev *dev;
        uart_mxc_port port ;
	void *base ;
	struct clk *clk ;
	int irq ;
	int idx;
	unsigned char data[10];
	char phys[32];
	int cksum ;
};

static char const constants[] = {
	" UTfxxyy\xff"
};

static void mxcuart_rx_chars(struct irtouch_data_t *data)
{
	unsigned int sr2;
	unsigned count = 0;

	while ((0 != (sr2 = readl(data->base + MXC_UARTUSR2)&MXC_UARTUSR2_RDR)) && (count++ < 256)) {
		unsigned ch = readl(data->base + MXC_UARTURXD);
		if (0 == (ch & 0x8000)) {
			printk (KERN_ERR "spurious\n" );
			continue;
		}
		data->data[data->idx] = ch;
		if (9 > data->idx) {
			data->cksum += (signed char)(ch&0xff);
		}

		switch (data->idx++) {
			case 0:
			case 1:
			case 7:
			case 8:
			{
				if ((ch & 0xFF) != constants[data->idx]) {
					printk( KERN_ERR "data[%u]: 0x%02x->0x%08x != 0x%02x\n", data->idx-1, data->data[data->idx-1],ch,constants[data->idx]);
					data->idx = 0;
					data->cksum = CKSUM_START ;
				}
				break;
			}
			case 9: {
				if ((u8)data->cksum == (u8)data->data[9]) {
                                        struct input_dev *dev = data->dev ;
					unsigned x = (data->data[4] << 8) | data->data[3];
					unsigned y = (data->data[6] << 8) | data->data[5];
					int touch = (0 != (data->data[2] & 3));
					input_report_abs(dev, ABS_X, x);
					input_report_abs(dev, ABS_Y, y);
					input_report_abs(dev, ABS_PRESSURE, touch);
					input_report_key(dev, BTN_TOUCH, touch);
					input_sync(dev);
				} else {
					char hex[41];
					int i ;
					for (i = 0 ; i < sizeof(data->data); i++) {
						sprintf(hex+(i*2),"%02x",data->data[i]);
					}
					hex[40] = '\0' ;
					printk (KERN_ERR "cksum 0x%02x, hex: %s\n", data->cksum, hex);
				}
				data->idx = 0 ;
				data->cksum = CKSUM_START ;
			}
		}
	}
}

/*!
 * Interrupt service routine registered to handle the muxed ANDed interrupts.
 * This routine is registered only in the case where the UART interrupts are
 * muxed.
 *
 * @param   irq    the interrupt number
 * @param   dev_id driver private data
 *
 * @return  The function returns \b IRQ_RETVAL(1) if interrupt was handled,
 *          returns \b IRQ_RETVAL(0) if the interrupt was not handled.
 *          \b IRQ_RETVAL is defined in \b include/linux/interrupt.h.
 */
static irqreturn_t mxcuart_int(int irq, void *dev_id)
{
        struct irtouch_data_t *data = dev_id ;
	unsigned int sr2 ;

	while (0 != (sr2 = readl(data->base + MXC_UARTUSR2))) {
		/* Clear the bits that triggered the interrupt */
		writel(sr2, data->base + MXC_UARTUSR2);
		writel(readl(data->base + MXC_UARTUSR1), data->base + MXC_UARTUSR1);
		/*
		 * Read if there is data available
		 */
		if (sr2 & MXC_UARTUSR2_RDR) {
			mxcuart_rx_chars(data);
		}
		else
			break;
	}

	return IRQ_HANDLED ;
}

/*!
 * This function is called by the core driver to stop receiving characters; the
 * port is in the process of being closed.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 */
static void stop_rx(struct irtouch_data_t *data)
{
	volatile unsigned int cr1;

	cr1 = readl(data->base + MXC_UARTUCR1);
	cr1 &= ~MXC_UARTUCR1_RRDYEN;
	writel(cr1, data->base + MXC_UARTUCR1);
}

static int uart_startup(struct irtouch_data_t *data)
{
	volatile unsigned int cr, cr1 = 0, cr2 = 0, ufcr = 0;
	volatile unsigned int cr4 = 0;
	u_int num, denom, baud;
	u_int cr2_mask;		/* Used to add the changes to CR2 */
	unsigned long per_clk;
	int div;
	unsigned int d ;

	/*
	 * Clear Status Registers 1 and 2
	 */
	writel(0xFFFF, data->base + MXC_UARTUSR1);
	writel(0xFFFF, data->base + MXC_UARTUSR2);

	cr2 = MXC_UARTUCR2_TXEN|MXC_UARTUCR2_RXEN;

	writel(cr2, data->base + MXC_UARTUCR2);
	/* Wait till we are out of software reset */
	do {
		cr = readl(data->base + MXC_UARTUCR2);
	} while (!(cr & MXC_UARTUCR2_SRST));

	ufcr |= ((0x10 << MXC_UARTUFCR_TXTL_OFFSET) |
		 MXC_UARTUFCR_RFDIV | 1);
	writel(ufcr, data->base + MXC_UARTUFCR);

	writel(0x404, data->base + MXC_UARTUCR3);

	cr2_mask = ~(MXC_UARTUCR2_IRTS | MXC_UARTUCR2_CTSC | MXC_UARTUCR2_PREN |
		     MXC_UARTUCR2_PROE | MXC_UARTUCR2_STPB | MXC_UARTUCR2_WS);

	per_clk = clk_get_rate(data->clk);

	/*
	 * Ask the core to get the baudrate, if requested baudrate is not
	 * between max and min, then either use the baudrate in old termios
	 * setting. If it's still invalid, we try 9600 baud.
	 */
	baud = 9600 ;
	d = 1;

	/*
	 * Choose the smallest possible prescaler to maximize
	 * the chance of using integer scaling.  Ensure that
	 * the calculation won't overflow.  Limit the denom
	 * to 15 bits since a 16-bit denom doesn't work.
	 */
	if (baud < (1 << (31 - (4 + 15))))
		d = per_clk / (baud << (4 + 15)) + 1;

	per_clk /= d ;
	/*
	 * Set the ONEMS register that is used by IR special case bit and
	 * the Escape character detect logic
	 */
	writel(per_clk / 1000, data->base + MXC_UARTONEMS);
	div = d;

	cr2 = MXC_UARTUCR2_WS;

	ufcr = readl(data->base + MXC_UARTUFCR);
	ufcr = (ufcr & (~MXC_UARTUFCR_RFDIV_MASK)) |
	    ((6 - div) << MXC_UARTUFCR_RFDIV_OFFSET);
	writel(ufcr, data->base + MXC_UARTUFCR);

	cr4 = readl(data->base + MXC_UARTUCR4);
	cr2 |= MXC_UARTUCR2_IRTS;

	/* Add Parity, character length and stop bits information */
	cr2 |= (readl(data->base + MXC_UARTUCR2) & cr2_mask);
	writel(0x4027, data->base + MXC_UARTUCR2);
	writel(cr4, data->base + MXC_UARTUCR4);

	/*
	 * Set baud rate
	 */

	/* Use integer scaling, if possible. Limit the denom to 15 bits. */
	num = 0;
	denom = (per_clk + 8 * baud) / (16 * baud) - 1;

	/* Use fractional scaling if needed to limit the max error to 0.5% */
	if (denom < 100) {
		u64 n64 = (u64) 16 * 0x8000 * baud + (per_clk / 2);
		do_div(n64, per_clk);
		num = (u_int) n64 - 1;
		denom = 0x7fff;
	}
	writel(num, data->base + MXC_UARTUBIR);
	writel(denom, data->base + MXC_UARTUBMR);

	cr1 |= (MXC_UARTUCR1_RRDYEN | MXC_UARTUCR1_UARTEN);
	writel(cr1, data->base + MXC_UARTUCR1);

	return 0;
}

static int irtouch_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;
	int retval ;
        struct irtouch_data_t *data ;
	int irq = -1 ;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk (KERN_ERR "%s: unspecified MEM\n", __func__ );
		retval = -ENODEV;
		goto out1 ;
	}

	base = ioremap(res->start, res->end - res->start + 1);
	if (!base) {
		printk (KERN_ERR "%s: ioremap error\n", __func__ );
		retval = -ENOMEM;
		goto out1 ;
	}
	irq = platform_get_irq(pdev, 0);
	if (0 > irq) {
		printk (KERN_ERR "%s: invalid irq\n", __func__ );
		retval = -ENODEV ;
		goto out2 ;
	}

	data = kzalloc(sizeof(struct irtouch_data_t), GFP_KERNEL);
	if (0 == data) {
		printk (KERN_ERR "%s: alloc error %d\n", __func__, sizeof(*data));
		retval = -ENOMEM ;
		goto out2 ;
	}
	retval = request_irq(irq, mxcuart_int, 0,
			     DRIVER_NAME, data);
	if (0 != retval) {
		printk (KERN_ERR "%s: error grabbing irq %d\n", __func__, irq );
		goto out3 ;
	}
	data->irq = irq ;
	data->base = base ;
	data->clk = clk_get(&pdev->dev, "uart_clk");
	if (IS_ERR(data->clk)) {
		printk (KERN_ERR "%s: error %dgetting clock\n", __func__, (int)data->clk );
		retval = -EINVAL ;
		goto out4 ;
	} else {
		printk (KERN_ERR "%s: got clock %p\n", __func__, data->clk );
	}
        clk_enable(data->clk);
	uart_startup(data);

	data->cksum = CKSUM_START ;
	data->dev = input_allocate_device();
	if (0 == data->dev) {
		printk (KERN_ERR "%s: error allocating input device\n", __func__ );
		retval = -ENOMEM ;
		goto out4 ;
	}
	data->dev->name = "IR Touch Serial TouchScreen";
	data->dev->id.bustype = BUS_RS232;
	data->dev->id.vendor = 0;
	data->dev->id.product = 0;
	data->dev->id.version = 0x0100;
	data->dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	data->dev->absbit[0] = BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) | BIT_MASK(ABS_PRESSURE);
	data->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(data->dev, ABS_X,
			     0, XRESOLUTION-1, 0, 0);
	input_set_abs_params(data->dev, ABS_Y,
			     0, YRESOLUTION-1, 0, 0);

	input_set_drvdata(data->dev, pdev);
	retval = input_register_device(data->dev);
	if (retval) {
		printk (KERN_ERR "%s: error registering input device\n", __func__ );
		retval = -ENODEV ;
		goto out5 ;
	}

	platform_set_drvdata(pdev, data);
	printk (KERN_ERR "%s: IR Touch screen @%x..%x, irq %d\n", __func__,
		res->start,res->end,irq);
	printk (KERN_ERR "%s: returning %d\n", __func__, retval);
	return retval ;
out5:
	input_free_device(data->dev);
out4:
	free_irq(data->irq, data);
out3:
	kfree(data);
out2:
	iounmap(base);
out1:
	return retval ;
}

static int irtouch_remove(struct platform_device *pdev)
{
        struct irtouch_data_t *data = platform_get_drvdata(pdev);
	if (data) {
                input_unregister_device(data->dev);
		stop_rx(data);
		if (0 <= data->irq) {
			free_irq(data->irq, data);
		}
		if (data->base) {
			iounmap(data->base);
		}
	}
	return 0;
}

static struct platform_driver irtouch_driver = {
	.driver = {
		.name = DRIVER_NAME,
		},
	.probe = irtouch_probe,
	.remove = irtouch_remove,
	.suspend = NULL,
	.resume = NULL,
};

/*
 * The functions for inserting/removing us as a module.
 */

static int __init irtouch_init(void)
{
	return platform_driver_register(&irtouch_driver);
}

static void __exit irtouch_exit(void)
{
	platform_driver_unregister(&irtouch_driver);
}

module_init(irtouch_init);
module_exit(irtouch_exit);
