/*
 *  linux/drivers/video/pxafb_yuv.c
 *
 *  Copyright (C) 2008 Troy Kisky <troy.kisky@BoundaryDevices.com>
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
#include <mach/gpio.h>
#include <mach/bitfield.h>
#include <mach/pxafb.h>
#include <video/pxa27xfb.h>
#include "pxafb.h"
#include "pxafb_yuv.h"
int pxafb_get_mmio(void);

#define ROUND_UP2(num) (((num) + 1) & ~1)
#define ROUND_UP4(num) (((num) + 3) & ~3)
#define ROUND_UP8(num) (((num) + 7) & ~7)

static atomic_t g_yuv;

struct fb_info_yuv *get_yuv(void)
{
	struct fb_info_yuv * yuv = (struct fb_info_yuv *)atomic_read(&g_yuv);
	if (!yuv) {
		struct fb_info_yuv *p;
		yuv = (struct fb_info_yuv *)kzalloc( sizeof(struct fb_info_yuv), GFP_KERNEL);
		if (!yuv)
			return NULL;
		yuv->fb_planes[0].parent = yuv;
		yuv->fb_planes[1].parent = yuv;
		yuv->fb_planes[2].parent = yuv;
		p = (struct fb_info_yuv *)atomic_cmpxchg(&g_yuv, (int)NULL, (int)yuv); 
		if (p) {
			kfree(yuv);
			yuv = p;
		}
	}
	return yuv;
}

static inline unsigned int lcd_readl(int base, unsigned int reg)
{
	return __raw_readl(base + reg);
}

static inline void lcd_writel(int base, unsigned int reg, unsigned int val)
{
	printk(KERN_ERR "lcd_writel base=0x%x, reg=0x%x, val=0x%x\n", base , reg, val);
	__raw_writel(val, base + reg);
}

/* Cache large dma space at probe time
 * to avoid fragmentation problems.
 */
static void fb_init_dma_cache(struct fb_info_yuv *yuv)
{
	u_int width;
	u_int height;
	u_int size;
	unsigned int lccr1;
	unsigned int lccr2;
	int i;
	int mmio_base = pxafb_get_mmio();
	if (!mmio_base)
		return;
	
	lccr1 = lcd_readl(mmio_base, LCCR1);
	lccr2 = lcd_readl(mmio_base, LCCR2);
	width = (lccr1 & 0x3ff) + 1;
	height = (lccr2 & 0x3ff) + 1;
	if (width < 320)
		width = 320;
	if (height < 240)
		height = 240;
	size = ROUND_UP4(width) * height;
	size = PAGE_ALIGN(size);
	for (i=0; i<3; i++) {
		struct fb_dma *cache = &yuv->fb_cache[i];
		cache->map_virtual = dma_alloc_writecombine(yuv->dev, size,
			&cache->map_dma, GFP_KERNEL);
		if (cache->map_virtual) {
			cache->map_size = size;
			printk(KERN_ERR "%s: map_dma 0x%x\n", __func__, cache->map_dma);
		} else
			break;
	}
}

static void fb_free_dma_cache(struct fb_info_yuv *yuv)
{
	int i;
	for (i=0; i<3; i++) {
		struct fb_dma *cache = &yuv->fb_cache[i];
		if (cache->map_virtual) {
			dma_free_writecombine(yuv->dev,
					cache->map_size,
					cache->map_virtual,
					cache->map_dma);
			cache->map_virtual = NULL;
			cache->map_size = 0;
		}
	}
}

int alloc_plane_dma(struct fb_info_yuv *yuv, struct fb_plane *pi,
		int plane_index, int size)
{
	struct fb_dma *cache = &yuv->fb_cache[plane_index];
	if (cache->map_virtual) {
		if (cache->map_size >= size) {
			pi->map_virtual = cache->map_virtual;
			pi->map_dma = cache->map_dma;
			pi->map_size = cache->map_size;
			return 0;
		}
	}
	printk(KERN_ERR "calling dma_alloc_writecombine, size %i, plane %i\n",
			size, plane_index);
	pi->map_virtual = dma_alloc_writecombine(yuv->dev, size,
		&pi->map_dma, GFP_KERNEL);
	if (!pi->map_virtual)
		return -ENOMEM;

	pi->map_size = size;
	if (cache->map_virtual) {
		if (cache->map_size < size) {
			dma_free_writecombine(yuv->dev,
					cache->map_size,
					cache->map_virtual,
					cache->map_dma);
			cache->map_virtual = NULL;
		}
	}
	if (!cache->map_virtual) {
		cache->map_virtual = pi->map_virtual;
		cache->map_dma = pi->map_dma;
		cache->map_size = size;
	}
	printk(KERN_ERR "%s: map_dma 0x%x\n", __func__, pi->map_dma);
	return 0;
}

static void free_plane_dma(struct fb_info_yuv *yuv, int plane_index)
{
	struct fb_plane *pi = &yuv->fb_planes[plane_index];
	struct fb_dma *cache = &yuv->fb_cache[plane_index];
	if (pi->map_virtual) {
		if (pi->map_virtual != cache->map_virtual) {
			dma_free_writecombine(yuv->dev,
					pi->map_size,
					pi->map_virtual,
					pi->map_dma);
		}
		pi->map_virtual = NULL;
	}
}



void destroy_planes(struct fb_info_yuv *yuv)
{
	int mmio_base = pxafb_get_mmio();
	if (mmio_base) {
		/* disable overlay */
#if 1
                unsigned int ovl2c1 = lcd_readl(mmio_base, OVL2C1);
		lcd_writel(mmio_base, OVL2C1, ovl2c1 & 0x00ffffff);
		lcd_writel(mmio_base, FBR3, (yuv->desc_dma +
				(PLANE_U * sizeof(struct pxafb_dma_descriptor))) | 1);
		lcd_writel(mmio_base, FBR4, (yuv->desc_dma +
				(PLANE_V * sizeof(struct pxafb_dma_descriptor))) | 1);
		lcd_writel(mmio_base, FBR2, (yuv->desc_dma +
				(PLANE_Y * sizeof(struct pxafb_dma_descriptor))) | 1);
#else
		int cnt=5;
		unsigned int tmp,tmp2;
                unsigned int lccr0 = lcd_readl(mmio_base, LCCR0);
                unsigned int ovl2c1 = lcd_readl(mmio_base, OVL2C1);
                // Initiate power down sequence
		lcd_writel(mmio_base, LCCR0, lccr0 | LCCR0_DIS);

                // Wait for LDD bit to get set once the last DMA transfer has completed
		do {
			tmp = lcd_readl(mmio_base, LCSR);
			if ((tmp & LCSR_LDD))
				break;
			tmp2 = lcd_readl(mmio_base, LCCR0);
			if (!(tmp2 & LCCR0_ENB))
				break;
			msleep(1);
		} while (cnt--);

                // Clear the sticky LDD bit
		lcd_writel(mmio_base, LCSR, tmp | LCSR_LDD);
		lcd_writel(mmio_base, OVL2C1, ovl2c1 & 0x00ffffff);
		/* Don't know why FDADR0 needs rewritten,
		 * but it does.
		 */
		tmp = lcd_readl(mmio_base, FDADR0);
		lcd_writel(mmio_base, FDADR0, tmp);
		lcd_writel(mmio_base, LCCR0, lccr0);
#endif
	}
	free_plane_dma(yuv, PLANE_Y);
	free_plane_dma(yuv, PLANE_U);
	free_plane_dma(yuv, PLANE_V);
}

static int alloc_plane_buffer(struct fb_plane *pi, int index, int plane_size,int val)
{
	struct pxafb_dma_descriptor *dma_desc;
	/*  buffer aligned to page boundary 
	 *  to make mmap safer 
	 */
	int size = PAGE_ALIGN(plane_size);
	int ret = alloc_plane_dma(pi->parent, pi, index, size);
	if (ret)
		return ret;
	pi->plane_size = plane_size;
	memset(pi->map_virtual, val, plane_size);
	dma_desc = &pi->parent->desc_virtual[index];
	dma_desc->fdadr = pi->parent->desc_dma +
		(index * sizeof(struct pxafb_dma_descriptor));
	dma_desc->fsadr = pi->map_dma;
	dma_desc->fidr  = 0;
	dma_desc->ldcmd = plane_size;
	return 0;
}

int create_yuv_surface(struct fb_info_yuv *yuv, struct pxa27x_overlay_t *pdata)
{
	int ret;
	int size_y, size_uv, width_y, height_y, width_uv, height_uv;
	int mmio_base = pxafb_get_mmio();
	if (!mmio_base)
		return -1;
	width_y = pdata->width;
	height_y = pdata->height;
	{
		unsigned int lccr1 = lcd_readl(mmio_base, LCCR1);
		unsigned int lccr2 = lcd_readl(mmio_base, LCCR2);
		int max_width = (lccr1 & 0x3ff) + 1;
		int max_height = (lccr2 & 0x3ff) + 1;
		if (max_width > pdata->offset_x)
			max_width -= pdata->offset_x;
		else
			pdata->offset_x = 0;
		if (max_height > pdata->offset_y)
			max_height -= pdata->offset_y;
		else
			pdata->offset_y = 0;
		if (width_y > max_width) {
			pdata->width = width_y = max_width;
		}
		if (height_y > max_height) {
			pdata->height = height_y = max_height;
		}
	}
	switch (pdata->for_type) {
	case FOR_RGB:
		return -2;	/* not implemented */
	case FOR_PACKED_YUV444:
		width_y = ROUND_UP4(width_y * 3);
		size_y = width_y * height_y;
		size_uv = 0;
		break;
	case FOR_PLANAR_YUV444:
		width_uv = width_y = ROUND_UP4(width_y);
		size_uv = size_y = width_y * height_y;
		break;
	case FOR_PLANAR_YUV422:
		width_y = ROUND_UP4(width_y);
		width_uv = ROUND_UP8(width_y) >> 1;
		size_y = width_y * height_y;
		size_uv = width_uv * height_y;
		break;
	case FOR_PLANAR_YUV420:
		width_y = ROUND_UP4(width_y);
		width_uv = ROUND_UP8(width_y) >> 1;
		height_uv = ROUND_UP2(height_y) >> 1;
		size_y = width_y * height_y;
		size_uv = width_uv * height_uv;
		break;
	default:
		return -3;
	}
	destroy_planes(yuv);
	if (!yuv->desc_virtual) {
		printk(KERN_ERR "calling dma_alloc_writecombine desc\n");
		yuv->desc_virtual = dma_alloc_writecombine(yuv->dev, 64,
			&yuv->desc_dma, GFP_KERNEL);
		if (!yuv->desc_virtual)
			return -ENOMEM;
		yuv->desc_size = 64;
	}

	ret = alloc_plane_buffer(&yuv->fb_planes[PLANE_Y], PLANE_Y,
			size_y, 0x10);
	if (ret)
		return ret;
	if (size_uv) {
		ret = alloc_plane_buffer(&yuv->fb_planes[PLANE_U], PLANE_U,
				size_uv, 0x80);
		if (ret)
			return ret;
		ret = alloc_plane_buffer(&yuv->fb_planes[PLANE_V], PLANE_V,
				size_uv, 0x80);
		if (ret)
			return ret;
	}
	lcd_writel(mmio_base, FBR2, 0);
	lcd_writel(mmio_base, FBR3, 0);
	lcd_writel(mmio_base, FBR4, 0);
	lcd_writel(mmio_base, LCCR5, 0x3f3f3f3f);
	lcd_writel(mmio_base, OVL2C2, (pdata->for_type<<20)|
			(pdata->offset_y<<10)|pdata->offset_x);
	lcd_writel(mmio_base, OVL2C1, (1<<31)|(4<<20)|((height_y-1)<<10)|(width_y-1));
	if (size_uv) {
		lcd_writel(mmio_base, FDADR3, yuv->desc_dma +
			(PLANE_U * sizeof(struct pxafb_dma_descriptor)));
		lcd_writel(mmio_base, FDADR4, yuv->desc_dma +
			(PLANE_V * sizeof(struct pxafb_dma_descriptor)));
	}
	lcd_writel(mmio_base, FDADR2, yuv->desc_dma +
			(PLANE_Y * sizeof(struct pxafb_dma_descriptor)));
	return 0;
}

#define GetPlane(filp) ((struct fb_plane *)((filp)->private_data))

static int pxafb_yuv_open(struct inode *inode, struct file *filp)
{
	struct fb_info_yuv *yuv;
	struct fb_plane *pi;
	unsigned int minor = iminor(inode);
	yuv = get_yuv();
	if (!yuv)
		return -ENOMEM;
	filp->private_data = pi = &yuv->fb_planes[minor];

	atomic_inc(&yuv->usage);
	printk(KERN_ERR "%s: %d uses\n", __func__, atomic_read(&yuv->usage));
	return 0 ;
}

static int pxafb_yuv_release(struct inode *inode, struct file *filp)
{
	int uses;
	struct fb_info_yuv *yuv;
	struct fb_plane *pi = GetPlane(filp);
	if (!pi)
		return -EINVAL;
	yuv = pi->parent;
	uses = atomic_dec_return(&yuv->usage);
	printk(KERN_ERR "%s: %d uses\n", __func__, uses);
	if (!uses)
		destroy_planes(yuv);
	return 0;
}

static ssize_t pxafb_yuv_write (struct file *filp, const char *buffer,
		size_t count, loff_t *ppos)
{
	int rval;
	unsigned max;
	unsigned char * dest;
	struct fb_plane *pi = GetPlane(filp);
	if (!pi)
		return -EINVAL;
	if (!pi->map_virtual) {
		int ret;
		struct pxa27x_overlay_t data;
		unsigned int lccr1,lccr2;
		int mmio_base = pxafb_get_mmio();
		if (!mmio_base)
			return -EINVAL;
		
		lccr1 = lcd_readl(mmio_base, LCCR1);
		lccr2 = lcd_readl(mmio_base, LCCR2);
		data.for_type = FOR_PLANAR_YUV422;
		data.offset_x = 0;
		data.offset_y = 0;
		data.width = (lccr1 & 0x3ff) + 1;
		data.height = (lccr2 & 0x3ff) + 1;

		printk(KERN_ERR "calling create_yuv_surface\n");
		ret = create_yuv_surface(pi->parent, &data);
		printk(KERN_ERR "create default yuv plane %ux%u 0x%x bytes ret=%i\n",
				data.width, data.height, pi->plane_size, ret);
	}

	if (!pi->map_virtual)
		return -EIO;
	if (*ppos >= pi->plane_size)
		return -EFBIG;

	dest = pi->map_virtual + *ppos;
	max = pi->plane_size - *ppos;
	if (count > max )
		count = max ;
	*ppos += count;
	rval = copy_from_user(dest, buffer, count);
	if (!rval) {
		return count ;
	} else {
		printk( KERN_ERR "%s: short write %u of %u\n", __func__, count-rval, count );
		return rval ;
	}
}

static unsigned int pxafb_yuv_poll(struct file *filp, poll_table *wait)
{
	return POLLIN;
}

static ssize_t  pxafb_yuv_read (struct file *filp, char *buffer,
		size_t count, loff_t *ppos)
{
	return count ;
}

static int pxafb_yuv_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct fb_plane *pi = GetPlane(filp);
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	if (!pi)
		return -EINVAL;

	if (off < pi->plane_size) {
		return dma_mmap_writecombine(pi->parent->dev, vma, pi->map_virtual,
				pi->map_dma, pi->plane_size);
	}
	return -EINVAL;
}

static int pxafb_yuv_ioctl( struct inode *inode, struct file  *filp,
		unsigned int  cmd, unsigned long arg)
{
	struct fb_info_yuv * yuv;
	int return_val = 0;
	struct fb_plane *pi = GetPlane(filp);
	if (!pi)
		return -EINVAL;
	yuv = pi->parent;
	switch (cmd) {
	case PXA27X_YUV_SET_DIMENSIONS:
	{
		struct pxa27x_overlay_t overlay_data;
		if (0 == copy_from_user(&overlay_data, (void *)arg,
				sizeof(overlay_data))) {
			if (create_yuv_surface(yuv, &overlay_data))
				return_val = -EINVAL;
			else {
				return_val = copy_to_user((void *)arg,
						&overlay_data, sizeof(overlay_data) );
			}
		} else
			return_val = -EFAULT ;
		break;
	}
	default:
		return_val= -EINVAL;
		printk( KERN_ERR "Invalid ioctl: 0x%x, allowed 0x%x\n",
				cmd, PXA27X_YUV_SET_DIMENSIONS);
	}
	return return_val;
}

static struct file_operations yuv_fops = {
	owner:	THIS_MODULE,
	mmap:	pxafb_yuv_mmap,
	open:	pxafb_yuv_open,
	read:	pxafb_yuv_read,
	poll:	pxafb_yuv_poll,
	write:	pxafb_yuv_write,
	ioctl:	pxafb_yuv_ioctl,
	release: pxafb_yuv_release
};


int __init pxafb_yuv_probe(struct platform_device *dev)
{
	int result ;
	struct device *c;
	struct fb_info_yuv *yuv;

	printk(KERN_ERR "%s:\n", __func__);
	yuv = get_yuv();
	if (!yuv)
		return -ENOMEM;

	yuv->pxafb_plane_class = class_create(THIS_MODULE, "pxafb_plane");
	if (!yuv->pxafb_plane_class) {
		printk(KERN_ERR "%s: error creating class\n", __func__ );
		return -EINVAL;
	}
	result = register_chrdev(yuv->plane_major,"pxafb_plane",&yuv_fops);
	if( result < 0 )
		return result;

	if (yuv->plane_major==0)
		yuv->plane_major = result; //dynamic assignment

	c = device_create(yuv->pxafb_plane_class, NULL,
			MKDEV(yuv->plane_major, 0), NULL, PXA27X_Y_CLASS);
	if (IS_ERR(c)) {
		printk(KERN_ERR "Unable to create class_device for "
				PXA27X_Y_CLASS "\n" );
	}
	c = device_create(yuv->pxafb_plane_class, NULL,
			MKDEV(yuv->plane_major, 1), NULL, PXA27X_U_CLASS);
	if (IS_ERR(c)) {
		printk(KERN_ERR "Unable to create class_device for "
				PXA27X_U_CLASS "\n" );
	}
	c = device_create(yuv->pxafb_plane_class, NULL,
			MKDEV(yuv->plane_major, 2), NULL, PXA27X_V_CLASS);
	if (IS_ERR(c)) {
		printk(KERN_ERR "Unable to create class_device for "
				PXA27X_V_CLASS "\n" );
	}
	yuv->dev = &dev->dev;
	platform_set_drvdata(dev, yuv);
	fb_init_dma_cache(yuv);
	return 0 ;
}

static int __devexit pxafb_yuv_remove(struct platform_device *dev)
{
	struct fb_info_yuv * yuv = (struct fb_info_yuv *)atomic_read(&g_yuv);
	if (!yuv)
		return 0;
	atomic_set(&g_yuv, 0);

	printk(KERN_ERR "%s\n", __func__);
	if (yuv->pxafb_plane_class) {
		device_destroy(yuv->pxafb_plane_class,MKDEV(yuv->plane_major, 0));
		device_destroy(yuv->pxafb_plane_class,MKDEV(yuv->plane_major, 1));
		device_destroy(yuv->pxafb_plane_class,MKDEV(yuv->plane_major, 2));
		class_destroy(yuv->pxafb_plane_class);
		yuv->pxafb_plane_class = NULL;
	}

	if (yuv->plane_major) {
		unregister_chrdev(yuv->plane_major,"pxafb_plane" );
		yuv->plane_major = 0;
	}
	destroy_planes(yuv);
	fb_free_dma_cache(yuv);
	if (yuv->desc_virtual) {
		dma_free_writecombine(yuv->dev, yuv->desc_size,
				yuv->desc_virtual, yuv->desc_dma);
		yuv->desc_virtual = NULL;
	}
	kfree(yuv);
	return 0;
}

static struct platform_driver pxafb_yuv_driver = {
	.driver		= {
		.name	= "pxafb_yuv",
		.owner  = THIS_MODULE,
	},
	.probe		= pxafb_yuv_probe,
	.remove		= __devexit_p(pxafb_yuv_remove),
//	.shutdown	= pxafb_yuv_shutdown,
//	.suspend	= pxafb_yuv_suspend,
//	.resume		= pxafb_yuv_resume,
};

static int __init pxafb_yuv_init(void)
{
	int rval ;
	rval = platform_driver_register(&pxafb_yuv_driver);
	printk(KERN_ERR "%s %d\n", __func__, rval ); 
	return rval ;
}

static void __exit pxafb_yuv_exit(void)
{
	platform_driver_unregister(&pxafb_yuv_driver);
}
module_init(pxafb_yuv_init);
module_exit(pxafb_yuv_exit);

MODULE_AUTHOR("(c) 2008 Troy Kisky <troy.kisky@boundarydevices.com>");
MODULE_DESCRIPTION("loadable yuv layer driver for PXA27x");
MODULE_LICENSE("GPL");
