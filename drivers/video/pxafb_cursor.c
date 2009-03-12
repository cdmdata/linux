/*
 *  linux/drivers/video/pxafb_cursor.c
 *
 *  Copyright (C) 2009 Valli <valli@BoundaryDevices.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/poll.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/div64.h>
#include <mach/pxa-regs.h>
#include <mach/mfp-pxa27x.h>
#include <mach/bitfield.h>
#include <mach/pxafb.h>
#include <video/pxa27xfb.h>
#include "pxafb.h"
int pxafb_get_mmio(void);

#define MAX_CURSOR_SIZE		2048 /*128*128 pixels*/

static atomic_t g_cursor;

static struct cursorfb_mode modes[] = {
	{32, 32, 2, 2, 2},   /* 2 color and transparency */
	{32, 32, 2, 3, 3},   /* 3 color and transparency */
	{32, 32, 2, 4, -1},   /* 4 color */
	{64, 64, 2, 2, 2},   /* 2 color and transparency */
	{64, 64, 2, 3, 3},   /* 3 color and transparency */
	{64, 64, 2, 4, -1},   /* 4 color */
	{128, 128, 1, 2, -1}, /* 2 color */
	{128, 128, 1, 1, 1}  /* 1 color and transparency */
};

struct fb_info_cursor {
	struct device* dev;
	struct class* pxafb_cursor_class;
	u_int major;
	dma_addr_t map_dma;
	u_char* map_virtual;
	u_int map_size;
	u_int cursor_size;
	u_int pal_size;
	struct pxafb_dma_descriptor *dmadesc_pal;
	struct pxafb_dma_descriptor *dmadesc_cursor;
	u_char* map_cursor;
	u_int* map_palette;
	struct cursorfb_info cinfo;
	bool dont_delete_atexit;
};

struct fb_info_cursor *get_cursor(void)
{
	struct fb_info_cursor * cursor = (struct fb_info_cursor *)atomic_read(&g_cursor);
	if (!cursor) {
		struct fb_info_cursor *p;
		cursor = (struct fb_info_cursor *)kzalloc( sizeof(struct fb_info_cursor), GFP_KERNEL);
		if (!cursor)
			return NULL;
		p = (struct fb_info_cursor *)atomic_cmpxchg(&g_cursor, (int)NULL, (int)cursor); 
		if (p) {
			kfree(cursor);
			cursor = p;
		}
	}
	return cursor;
}

static inline unsigned int lcd_readl(int base, unsigned int reg)
{
	return __raw_readl(base + reg);
}

static inline void lcd_writel(int base, unsigned int reg, unsigned int val)
{
	__raw_writel(val, base + reg);
}

void cursorfb_disable(struct fb_info_cursor *cursor)
{
	int mmio_base = pxafb_get_mmio();
	if (mmio_base) {
                u_int ccr = lcd_readl(mmio_base, CCR);
		u_int lcsr1 = lcd_readl(mmio_base, LCSR1);
		u_int timeout = 1000;

		/*
		 * cursor does not get disabled properly when
		 * there is an address in FBR5 that has not yet
		 * been completely transferred. to avoid this
		 * we check the branch status for DMA channel
		 * 5 and then reset it. we use a time out because
		 * the branch status is not set when the cursor
		 * image was loaded only onto the FDADR5 register.
		 */
		while(!(lcsr1 & 0x100000) && timeout) {
			udelay(10);
			timeout -= 10;
		}
		lcd_writel(mmio_base, CCR, ccr & ~CCR_CEN);
		lcd_writel(mmio_base, FBR5, cursor->dmadesc_cursor->fdadr | FBR_BRA);
	}
}

int cursorfb_alloc_buffer(struct fb_info_cursor *cursor)
{
	if(!cursor->map_virtual) {
		/*
		 * allocate space large enough to hold the cursor image,
		 * palette data and the dma descriptors that point to
		 * the two.
		 *
		 * max cursor data would be required by modes 7 and 8
		 * which would be
		 * 	(128 * 128)/8 = 2048 (2KB)
		 * so dma desc for cursor image will start at map_dma and
		 * extend to a size of 2K.
		 *
		 * we will place the palette at the 2K boundary and the
		 * descriptors will be placed at the end of the allocated
		 * memory
		 */
		cursor->map_virtual = dma_alloc_writecombine(cursor->dev, PAGE_SIZE,
			&cursor->map_dma, GFP_KERNEL);
		if(!cursor->map_virtual)
			return -ENOMEM;

		cursor->dmadesc_cursor = (struct pxafb_dma_descriptor *)
					(cursor->map_virtual + (PAGE_SIZE -
					sizeof(struct pxafb_dma_descriptor)));
		cursor->dmadesc_pal = (struct pxafb_dma_descriptor *)
					(cursor->map_virtual + (PAGE_SIZE -
					(sizeof(struct pxafb_dma_descriptor)
					* 2)));

		cursor->dmadesc_cursor->fdadr = cursor->map_dma + (PAGE_SIZE -
                                        (sizeof(struct pxafb_dma_descriptor)
                                        * 2));
		cursor->dmadesc_pal->fdadr = cursor->map_dma + (PAGE_SIZE -
                                        sizeof(struct pxafb_dma_descriptor));


		cursor->dmadesc_cursor->fsadr = cursor->map_dma;
		cursor->dmadesc_pal->fsadr = cursor->map_dma + MAX_CURSOR_SIZE;

		cursor->dmadesc_cursor->fidr = 0;
		cursor->dmadesc_pal->fidr = 0;

		cursor->map_cursor = cursor->map_virtual;
		cursor->map_palette = (u_int *)(cursor->map_virtual + MAX_CURSOR_SIZE);
	}
        return 0;
}

void cursorfb_set_color(struct fb_info_cursor *cursor, struct color24_info ci)
{
	if(!cursor->map_virtual) {
		int ret;
		ret = cursorfb_alloc_buffer(cursor);
		if(ret)
			return;
	}

	if(ci.color_idx < modes[cursor->cinfo.mode].color_count)
		cursor->map_palette[ci.color_idx] = ci.color;
}

int cursorfb_enable(struct fb_info_cursor *cursor)
{
	u_int height, width, size;
	int mmio_base = pxafb_get_mmio();
	u_int lccr5;
	bool is_disabled = true;

	if (!mmio_base)
		return -1;

	if (!cursor->map_virtual) {
		int ret;
		ret = cursorfb_alloc_buffer(cursor);
		if(ret)
			return ret;
	}

	height = modes[cursor->cinfo.mode].xres;
	width = modes[cursor->cinfo.mode].yres;
	size = height * width;
	cursor->cursor_size = ((size * modes[cursor->cinfo.mode].bpp) / 8);

	if(cursor->cursor_size > MAX_CURSOR_SIZE)
		return -1;

	if(lcd_readl(mmio_base, CCR) & CCR_CEN) {
		is_disabled = false;
	}

	cursor->pal_size = modes[cursor->cinfo.mode].color_count * sizeof(unsigned int);

	cursor->dmadesc_cursor->ldcmd = cursor->cursor_size;
	cursor->dmadesc_pal->ldcmd = cursor->pal_size | LDCMD_PAL;

	lccr5 = lcd_readl(mmio_base, LCCR5);
	lccr5 |= (IUM5 | BSM5 | EOFM5 | SOFM5);
	lcd_writel(mmio_base, LCCR5, lccr5);

	/* set the cursor */
	if(is_disabled) {
		lcd_writel(mmio_base, FDADR5, cursor->dmadesc_cursor->fdadr);
		lcd_writel(mmio_base, CCR, (CCR_CEN | ((0x3FF & cursor->cinfo.y_loc) << CYPOS_BIT_POS) |
						((0x3FF & cursor->cinfo.x_loc) << CXPOS_BIT_POS) |
						cursor->cinfo.mode ));
	}
	else {
		lcd_writel(mmio_base, CCR, (CCR_CEN | ((0x3FF & cursor->cinfo.y_loc) << CYPOS_BIT_POS) |
						((0x3FF & cursor->cinfo.x_loc) << CXPOS_BIT_POS) |
						cursor->cinfo.mode ));
		lcd_writel(mmio_base, FBR5, cursor->dmadesc_cursor->fdadr | FBR_BRA);
	}

	return 0;
}

static int pxafb_cursor_open(struct inode *inode, struct file *filp)
{
	filp->private_data = get_cursor();
	if (!filp->private_data)
		return -ENOMEM;

	return 0 ;
}

static int pxafb_cursor_release(struct inode *inode, struct file *filp)
{
	struct fb_info_cursor *cursor = (struct fb_info_cursor *)filp->private_data;

	if (!cursor)
		return -EINVAL;

	if(!cursor->dont_delete_atexit) {
		cursorfb_disable(cursor);
	}

	cursor->dont_delete_atexit = false;

	return 0;
}

static ssize_t pxafb_cursor_write(struct file *filp, const char *buffer,
		size_t count, loff_t *ppos)
{
	int rval;
	struct fb_info_cursor *cursor = (struct fb_info_cursor *)filp->private_data;

	if (!cursor)
		return -EINVAL;

	if (!cursor->map_virtual) {
		int ret;
		ret = cursorfb_alloc_buffer(cursor);
		if(ret)
			return 0;
	}

	if (count > MAX_CURSOR_SIZE)
		count = MAX_CURSOR_SIZE;

	*ppos += count;
	rval = copy_from_user(cursor->map_cursor, buffer, count);

	if (!rval) {
		return count ;
	} else {
		printk( KERN_ERR "%s: short write %u of %u\n", __func__, count-rval, count );
		return rval ;
	}
}

static unsigned int pxafb_cursor_poll(struct file *filp, poll_table *wait)
{
	return POLLIN;
}

static ssize_t  pxafb_cursor_read (struct file *filp, char *buffer,
		size_t count, loff_t *ppos)
{
	return count ;
}

static int pxafb_cursor_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct fb_info_cursor *cursor = (struct fb_info_cursor *)filp->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	if (!cursor)
		return -EINVAL;

	if (off < cursor->map_size) {
		return dma_mmap_writecombine(cursor->dev, vma, cursor->map_virtual,
				cursor->map_dma, cursor->map_size);
	}
	return -EINVAL;
}

static int pxafb_cursor_ioctl( struct inode *inode, struct file  *filp,
		unsigned int  cmd, unsigned long arg)
{
	struct fb_info_cursor * cursor = (struct fb_info_cursor *) filp->private_data;
	int return_val = 0;

	if (!cursor)
		return -EINVAL;

	switch (cmd) {
		case PXA27X_CURSOR_SETLOC:
		{
			struct cursorfb_info info;
			if (0 == copy_from_user(&info, (void *)arg,
					sizeof(info))) {
				cursor->cinfo = info;
				if (cursorfb_enable(cursor))
					return_val = -EINVAL;
			} else
				return_val = -EFAULT ;
			break;
		}
		case PXA27X_CURSOR_GETLOC:
		{
			return_val = copy_to_user((void *)arg,
					&cursor->cinfo, sizeof(cursor->cinfo) );
			break;
		}
		case PXA27X_CURSOR_ACTIVATE:
		{
			cursorfb_enable(cursor);
			break;
		}
		case PXA27X_CURSOR_DEACTIVATE:
		{
			cursorfb_disable(cursor);
			break;
		}
		case PXA27X_16_BIT_COLOR:
		{
			struct color16_info ci16;
			struct color24_info ci24;

			if(0 == copy_from_user(&ci16, (void *)arg,
						sizeof(ci16))) {
				unsigned red = (ci16.color & 0xF100) << 8;
				unsigned green = (ci16.color & 0x7E0) << 5;
				unsigned blue = (ci16.color & 0x1F) << 3;

				if(red & 0x00800000)
					red |= 0x00070000;
				if(green & 0x00008000)
					green |= 0x00000300;
				if(blue & 0x00000080)
					blue |= 0x00000007;

				ci24.color_idx = ci16.color_idx;
				ci24.color = red | green | blue;
				cursorfb_set_color(cursor, ci24);
			}
			else 
				return_val = -EINVAL;
			break;
		}
		case PXA27X_24_BIT_COLOR:
		{
			struct color24_info ci24;

			if(0 == copy_from_user(&ci24, (void *)arg,
						sizeof(ci24))) {
				ci24.color &= 0xFFFFFF;
				cursorfb_set_color(cursor, ci24);
			}
			else 
				return_val = -EINVAL;
			break;
		}
		case PXA27X_DONT_REMOVE_CURSOR_ATEXIT:
		{
			cursor->dont_delete_atexit = true;
			break;
		}
		default:
			return_val= -EINVAL;
			printk( KERN_ERR "Invalid ioctl: 0x%x\n", cmd);
	}
	return return_val;
}

static struct file_operations cursor_fops = {
	owner:	THIS_MODULE,
	mmap:	pxafb_cursor_mmap,
	open:	pxafb_cursor_open,
	read:	pxafb_cursor_read,
	poll:	pxafb_cursor_poll,
	write:	pxafb_cursor_write,
	ioctl:	pxafb_cursor_ioctl,
	release: pxafb_cursor_release
};

int __init pxafb_cursor_probe(struct platform_device *dev)
{
	int result ;
	struct device *c;
	struct fb_info_cursor *cursor;

	cursor = get_cursor();
	if (!cursor)
		return -ENOMEM;

	cursor->pxafb_cursor_class = class_create(THIS_MODULE, "pxafb_cursor");
	if (!cursor->pxafb_cursor_class) {
		printk(KERN_ERR "%s: error creating class\n", __func__ );
		return -EINVAL;
	}
	result = register_chrdev(cursor->major,"pxafb_cursor",&cursor_fops);
	if( result < 0 )
		return result;

	if (cursor->major==0)
		cursor->major = result; //dynamic assignment

	c = device_create(cursor->pxafb_cursor_class, NULL,
			MKDEV(cursor->major, 0), NULL, PXA27X_CURSOR);
	if (IS_ERR(c)) {
		printk(KERN_ERR "Unable to create class_device for "
				PXA27X_CURSOR "\n" );
	}
	cursor->dev = &dev->dev;
	platform_set_drvdata(dev, cursor);
	return 0 ;
}

static int __devexit pxafb_cursor_remove(struct platform_device *dev)
{
	struct fb_info_cursor * cursor = (struct fb_info_cursor *)atomic_read(&g_cursor);
	if (!cursor)
		return 0;
	atomic_set(&g_cursor, 0);

	if (cursor->pxafb_cursor_class) {
		device_destroy(cursor->pxafb_cursor_class,MKDEV(cursor->major, 0));
		class_destroy(cursor->pxafb_cursor_class);
		cursor->pxafb_cursor_class = NULL;
	}

	if (cursor->major) {
		unregister_chrdev(cursor->major,"pxafb_cursor" );
		cursor->major = 0;
	}
	cursorfb_disable(cursor);
	if (cursor->map_virtual) {
		dma_free_writecombine(cursor->dev, cursor->map_size,
				cursor->map_virtual, cursor->map_dma);
		cursor->map_virtual = NULL;
	}
	kfree(cursor);
	return 0;
}

static struct platform_driver pxafb_cursor_driver = {
	.driver		= {
		.name	= "pxafb_cursor",
		.owner  = THIS_MODULE,
	},
	.probe		= pxafb_cursor_probe,
	.remove		= __devexit_p(pxafb_cursor_remove),
};

static int __init pxafb_cursor_init(void)
{
	int rval ;
	rval = platform_driver_register(&pxafb_cursor_driver);
	printk(KERN_ERR "%s %d\n", __func__, rval ); 
	return rval ;
}

static void __exit pxafb_cursor_exit(void)
{
	platform_driver_unregister(&pxafb_cursor_driver);
}
module_init(pxafb_cursor_init);
module_exit(pxafb_cursor_exit);

MODULE_AUTHOR("(c) 2009 Valli <valli@boundarydevices.com>");
MODULE_DESCRIPTION("loadable hardware cursor driver for PXA27x");
MODULE_LICENSE("GPL");
