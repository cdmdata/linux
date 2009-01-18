#include "read_regs.h"

/*
 * This method is used  to read data from the iomem location
 */
inline unsigned long
lcd_readl(void __iomem *mmio_base, unsigned int off)
{
	return *((unsigned long *)(mmio_base + off));
}

void pxa_mode_from_registers(struct platform_device *dev)
{
	struct resource *r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	struct pxafb_mach_info *mach = dev->dev.platform_data;
	void __iomem *p_regs;
	unsigned long lccr0;

	if (r == NULL) {
		printk(KERN_ERR "%s no I/O memory resource defined\n",
								__func__);
		return;
	}

	p_regs = ioremap(r->start, r->end - r->start);
	lccr0 = lcd_readl(p_regs, LCCR0);

	if (0 != (lccr0 & LCCR0_ENB)) {
		unsigned long const lccr1 = lcd_readl(p_regs, LCCR1);
		unsigned long const lccr2 = lcd_readl(p_regs, LCCR2);
		unsigned long const lccr3 = lcd_readl(p_regs, LCCR3);
		unsigned int const cccr = CCCR ;
		unsigned L = cccr & 0x1F ;
		unsigned K ;
		unsigned lclk ;
		unsigned pcd = lccr3 & 0xFF ;

		if (L < 2)
			L = 2 ;
		K = (8 > L)? 1 : (16 >= L) ? 2 : 4 ;

		lclk = (13000000*L)/K ;
		mach->modes->pixclock = lclk/(2*(pcd+1));
		mach->modes->xres = (lccr1 & 0x3FF)+1 ;
		mach->modes->yres = (lccr2 & 0x3FF)+1 ;
		mach->modes->bpp	= 16 ;
		mach->modes->hsync_len = ((lccr1>>10)&0x3F)+1 ;
		mach->modes->left_margin	= (lccr1>>24)+1 ;
		mach->modes->right_margin = ((lccr1>>16)&0xFF)+1 ;
		mach->modes->vsync_len = ((lccr2>>10)&0x3f)+1 ;
		mach->modes->upper_margin = (lccr2>>24);
		mach->modes->lower_margin = ((lccr2>>16)&0xFF);
		mach->modes->sync = 0 ;
		if (0 == (lccr3 & LCCR3_HSP))
			mach->modes->sync = FB_SYNC_HOR_HIGH_ACT ;
		else
			mach->modes->sync = 0 ;
		if (0 == (lccr3 & LCCR3_VSP))
			mach->modes->sync |= FB_SYNC_VERT_HIGH_ACT ;
		if (lccr3&LCCR3_PCP)
			mach->lccr3 |= LCCR3_PCP ;
		else
			mach->lccr3 &= ~LCCR3_PCP ;
		printk(KERN_INFO "Display %ix%ix%i pixclock=%lu\n",
					mach->modes->xres,
					mach->modes->yres,
					mach->modes->bpp,
					mach->modes->pixclock);
	}
}

