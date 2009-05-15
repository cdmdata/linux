/* 
 * linux/drivers/video/sm501yuv.c
 *
 * This module provides read/write access to the
 * Video overlay channel of a Silicon Motion SM-501
 * graphics controller.
 *
 * $Id: sm501yuv.c,v 1.15 2006/08/30 02:10:14 ericn Exp $
 *
 * Revision History:
 *
 * $Log: sm501yuv.c,v $
 * Revision 1.15  2006/08/30 02:10:14  ericn
 * -allow asynchronous DMA completion if FASYNC
 *
 * Revision 1.13  2006/08/28 00:25:49  ericn
 * -updates
 *
 * Revision 1.12  2006/08/27 23:39:28  tkisky
 * -handle large page entries
 *
 * Revision 1.11  2006/08/27 22:24:53  ericn
 * -make it compile
 *
 * Revision 1.10  2006/08/27 22:05:45  tkisky
 * -page offset fix
 *
 * Revision 1.9  2006/08/27 21:58:23  tkisky
 * -..
 *
 * Revision 1.8  2006/08/27 13:32:54  ericn
 * -fix typos
 *
 * Revision 1.7  2006/08/27 01:57:02  tkisky
 * -pxa dma for YUV
 *
 * Revision 1.6  2006/08/25 04:27:24  ericn
 * -return yuv plane offset
 *
 * Revision 1.5  2006/08/16 02:50:51  ericn
 * -use current display size, enable after write
 *
 * Revision 1.4  2005/11/17 13:43:47  ericn
 * -fix line ends
 *
 * Revision 1.3  2005/11/17 13:43:04  ericn
 * -register class, device, device node
 *
 * Revision 1.2  2005/11/06 15:41:48  ericn
 * -use sm501_get_var/get_fix directly
 *
 * Revision 1.1  2005/10/21 06:16:58  ericn
 * -Initial import
 *
 * Revision 1.5  2005/05/01 17:15:10  ericn
 * -init yuv plane to gray
 *
 * Revision 1.4  2005/04/23 18:54:03  ericn
 * -updates
 *
 * Revision 1.3  2005/01/23 22:59:58  ericn
 * -move address handling to sm501, add ioctl for plane details
 *
 * Revision 1.2  2004/10/30 19:41:09  ericn
 * -functional w/command line params
 *
 * Revision 1.1  2004/10/03 13:15:34  ericn
 * -Initial SM-501 YUV support
 *
 *
 *
 * Copyright 2004, Boundary Devices
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/cpufreq.h>

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/kmod.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/ioport.h>

#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/mach/map.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <mach/pxa2xx-regs.h>

// #define SM501_USE_PXA_DMA

#ifdef SM501_USE_PXA_DMA
#include <asm/dma.h>
#include <asm/arch/pxa-regs.h>
#endif

#include <linux/platform_device.h>

#include <linux/sm501-int.h>
#include <linux/sm501yuv.h>
#include "sm501.h"

#define CLASS_NAME SM501YUV_CLASS

#define OSCR_HZ 3686480

typedef int (*sm501_get_fix_t)(struct fb_fix_screeninfo *fix, int con, struct fb_info *info);
typedef int (*sm501_get_var_t)(struct fb_var_screeninfo *var, int con, struct fb_info *info);
static struct class *yuvClass_ = 0 ;

#define YUV_MAJOR 0

static int yuv_major = YUV_MAJOR;
static int initialized = 0 ;
module_param(yuv_major, int,0);
MODULE_PARM_DESC(yuv_major, "Choose major device number");

static int yuv = 1 ;
module_param( yuv, int, 0 );
MODULE_PARM_DESC( yuv, "set non-zero for YUV support, zero for RGB");

static unsigned long fbStart = 0 ;
static unsigned long fbMax = 0 ;
static unsigned long mmioStart = 0 ;
static unsigned long mmioLength = 0 ;
static unsigned panelWidth = 0 ;
static unsigned panelHeight = 0 ;
static atomic_t usage = ATOMIC_INIT(0);

static char *fbYUV = 0 ;

static unsigned long const sysRegAddr = SM501_MMIOSTART ;
static unsigned long const videoRegAddr = SM501_MMIOSTART + 0x80040 ;
static unsigned long const videoRegValues[] = {
   0x00010000, 0x0703E360, 0x00200400, 0x00A81330,
   0x0385009C, 0x02200240, 0x00000000, 0x00000000,
   0x00EDEDED, 0x089C4040, 0x0031E3B0
};
static unsigned long const numVideoRegValues = sizeof(videoRegValues)/sizeof(videoRegValues[0]);

#define VIDEO_DISPLAY_CTRL                              0x080040
#define VIDEO_FB_0_ADDRESS                              0x080044
#define VIDEO_FB_WIDTH                                  0x080048
#define VIDEO_FB_0_LAST_ADDRESS                         0x08004C
#define VIDEO_PLANE_TL                                  0x080050
#define VIDEO_PLANE_BR                                  0x080054
#define VIDEO_SCALE                                     0x080058
#define VIDEO_INITIAL_SCALE                             0x08005C
#define VIDEO_YUV_CONSTANTS                             0x080060

#define DISPLAYREG( reg ) ((unsigned long volatile *)(mmioVirtual+reg))
static unsigned long yuvStartOffset = 0 ;

#define PLANEINFO( __filp ) ((struct sm501yuvPlane_t *)( (__filp)->private_data ))

// #define SM501_DEBUG

#ifdef SM501_DEBUG
#define DEBUGMSG( __fmt, ... ) printk( __fmt, ## __VA_ARGS__ )
#else
#define DEBUGMSG( __fmt, ... ) 
#endif

static void createPlane( struct sm501yuvPlane_t *plane )
{
   unsigned long inStride ;
   unsigned short hscale ;
   unsigned short vscale ;
   unsigned long  outScale ;

   inStride = (((plane->inWidth_+15)/16)*16)   // 128-bit (16 byte) aligned
            * 2 ; // 2 bytes/pixel

   *DISPLAYREG(VIDEO_FB_0_ADDRESS)        = yuvStartOffset ; // base + 640x240
   *DISPLAYREG(VIDEO_FB_0_LAST_ADDRESS)   = yuvStartOffset+(inStride*plane->inHeight_);
   *DISPLAYREG(VIDEO_PLANE_TL)            = (plane->yTop_ << 16)
                                          |  plane->xLeft_ ;
   *DISPLAYREG(VIDEO_PLANE_BR)            = ((plane->yTop_+plane->outHeight_-1) << 16 )
                                          | (plane->xLeft_+plane->outWidth_-1);
   if( plane->inWidth_ >= plane->outWidth_ )
   {
      hscale = ((plane->outWidth_<<12)/plane->inWidth_)
             | 0x8000 ;
   } // shrink
   else
   {
      hscale = (plane->inWidth_<<12)/plane->outWidth_ ;
   } // grow


   if( plane->inHeight_ >= plane->outHeight_ )
   {
      vscale = ((plane->outHeight_<<12)/plane->inHeight_)
             | 0x8000 ;
   } // shrink
   else
   {
      vscale = (plane->inHeight_<<12)/plane->outHeight_ ;
   } // grow
   
   DEBUGMSG( "%u->%u, hscale 0x%04x\n", plane->inWidth_, plane->outWidth_, hscale );
   DEBUGMSG( "%u->%u, vscale 0x%04x\n", plane->inHeight_, plane->outHeight_, vscale );

   *DISPLAYREG(VIDEO_FB_WIDTH) = (inStride<<16) | inStride ;  // bytes/line
   DEBUGMSG( "inWidth: %u, stride %lu\n", plane->inWidth_, inStride );

   outScale = (vscale<<16) | hscale ;
   DEBUGMSG( "outScale: 0x%08lx\n", outScale );

   *DISPLAYREG(VIDEO_SCALE) = outScale ;
   plane->planeOffset_ = yuvStartOffset ;

// *DISPLAYREG(VIDEO_INITIAL_SCALE)       = ??? ;
// *DISPLAYREG(VIDEO_YUV_CONSTANTS)       = ??? ;
}

static void destroyPlane( void )
{
   *DISPLAYREG(VIDEO_DISPLAY_CTRL) &= ~4UL ;  // disable
   *DISPLAYREG(VIDEO_PLANE_TL)     = 0 ;
   *DISPLAYREG(VIDEO_PLANE_BR)     = 0 ;
}

unsigned char dmaChannel = -1;


#ifndef SM501_USE_PXA_DMA
#define CP_FROM_USER(dest, src, cnt) copy_from_user( dest, src, cnt)
#else
#define CP_FROM_USER(dest, src, cnt) PxaDmaToSm501( dest, src, cnt, dmaChannel,((unsigned int)dest - (unsigned int)fbVirtual + 0x0c000000),filp->f_flags & FASYNC )

unsigned int* TransferDesc(unsigned int * pT,unsigned int physDest,unsigned int physSrc,int l,unsigned int* ppTphysLink,int cnt)
{
	unsigned int pTphysLink = *ppTphysLink;
	while (l) {
		int tmpLen = l;
		BUG_ON(l<0);
		if (tmpLen >= (8<<10)) {
			tmpLen = (8<<10) - 8;
		} else if (cnt==0) pTphysLink = 1;	//stop marker

//		printk( KERN_ERR "ddadr:%p,dsadr:%p,dtadr:%p\n",(void *)pTphysLink,(void *)physSrc,(void *)physDest);
		*pT++ = pTphysLink;
		*pT++ = physSrc;
		*pT++ = physDest;
		*pT++ = (DCMD_INCSRCADDR |DCMD_INCTRGADDR | DCMD_BURST32 |
		     DCMD_WIDTH4 | tmpLen);
		pTphysLink += 16;
		physSrc += tmpLen;
		physDest+= tmpLen;
		l -= tmpLen;
	}
	*ppTphysLink = pTphysLink;
	return pT;
}

static int tableSpaceLen=0;
static unsigned int * tableSpacePtr=NULL;
static unsigned int * tableSpaceAlignedPtr=NULL;
static char * tableSpaceEnd=NULL ;

static void printDescr( unsigned int *desc )
{
	while( 1 ){
		printk( KERN_ERR "%08x %08x %08x %08x\n", desc[0], desc[1], desc[2], desc[3] );
		if( desc[0] & 1 )
			break ;
		else
			desc += 4 ;
	}
}

static void waitDMA( int dma )
{
	unsigned long const start = OSCR ;
DEBUGMSG( "a: %08lx/%08lx/%08lx\n", DDADR(dma), DCSR(dma), DCMD(dma) );
	do {
		unsigned long now ;
		unsigned long cmd;
		unsigned long csr;
		cmd = DCMD(dma);
		if( 0 == (cmd & 0x1fff) )
			break ;
		csr = DCSR(dma);
		if(0 != (csr & DCSR_BUSERR)){
			printk( KERN_ERR "sm501: bus error\n" );
			break ;
		}
		now = OSCR ;
		if( OSCR_HZ > (now-start) )
			cpu_relax();
		else {
			printk( KERN_ERR "YUV dma stall: %08x:%08x:%08x\n",
				DDADR(dma), DCSR(dma), DCMD(dma) );
			printDescr( tableSpaceAlignedPtr );
			break ;
		}
	} while( 1 );
DEBUGMSG( "b:%08lx:%08lx:%08lx\n", DDADR(dma), DCSR(dma), DCMD(dma) );
}

static int validateDesc( unsigned int *desc, unsigned size )
{
	unsigned int total = 0 ;
	int rval ;
	while( 1 ){
		total += desc[3]&0x1fff ;
		if( desc[0] & 1 )
			break ;
		desc += 4 ;
	}
	rval = (size == total);
	if( 0 == rval ){
		printk( KERN_ERR "Invalid descriptor length: (%u not %u)\n", total, size );
	}
	return rval ;
}

//int dma_map_sg(struct device *dev, struct scatterlist *sg, int nents, enum dma_data_direction direction);
//void dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nhwentries, enum dma_data_direction direction)           
static int PxaDmaToSm501(char *dest, const char *src, unsigned int cnt, int dma, u_long physDest, int flags)
{
	unsigned const totalLen = cnt ;
	unsigned int * ttbr;
	unsigned int* pT;
	unsigned int pTphysLink,pTphysStart;

	DEBUGMSG( "%s: %p (%u) -> %p (%08lx) (dma %d), f%d\n", __FUNCTION__,
		  src, cnt, dest, physDest, dma, flags );

	if ( ((u_long)src|physDest) & 0x7) {
		/* 8 bytes alignment is required for memory to memory DMA */
		dma = (unsigned char)-1;
		printk( KERN_ERR "YUV buffer not aligned, src:%p, dest:0x%lx\n",src,physDest );
	}

	if (dma) {
		int entryLen = (((((cnt+0xfff)|0xfff)+1)>>12)+1) <<4 ;	//this is maximum # of entries needed for any alignment
		if (entryLen>tableSpaceLen) {
			if (tableSpacePtr) { 
DEBUGMSG( "p%s\n", __FUNCTION__ );
                           waitDMA(dma); 
                           kfree(tableSpacePtr); tableSpacePtr=NULL; tableSpaceLen = 0;
                        }
			tableSpacePtr = (unsigned int*)kmalloc(entryLen,GFP_KERNEL);
			if (tableSpacePtr) {
				tableSpaceLen = entryLen;
				tableSpaceAlignedPtr = (void *)((((unsigned int)tableSpacePtr)|0xf)&~0xf);
				tableSpaceEnd = ((char *)tableSpacePtr)+entryLen ;
			} else dma = (unsigned char)-1;
		}
	}
	/* fallback if no DMA available */
	if (dma == (unsigned char)-1) {
		return copy_from_user( dest, src, cnt);
	}

	consistent_sync((char *)src,cnt,DMA_TO_DEVICE);	//this should be done earlier, when frame is queued to output list.

	pT = tableSpaceAlignedPtr;
	pTphysStart = virt_to_phys(pT);
	pTphysLink = pTphysStart+16;

	__asm__ __volatile__ ("mrc p15, 0, %0, c2, c0, 0" : "=r" (ttbr));
	ttbr = (unsigned int *)( ((unsigned int)ttbr) + 0x20000000); //phys to virtual
	ttbr += ((unsigned int)src)>>20;	//go to the 1st level descriptor for this meg

	waitDMA(dma);

	do {
		unsigned int l1 = ( ((unsigned int)src) | 0xfffff) + 1 - ((unsigned int)src); //just go to the next meg boundary
		unsigned int val = *ttbr++;
		if (l1>cnt) l1 = cnt;
		if ((val&3)==2) {
			//this is a section descriptor
			val &= ~0xfffff;
			val |= ( ((unsigned int)src) & 0xfffff);
			cnt -= l1;
			src += l1;
			pT = TransferDesc(pT,physDest,val,l1,&pTphysLink,cnt);
			physDest += l1;
		} else if ((val&3)==1) {
			//this is normal case, coarse page table
			unsigned int * secondLevel;
			val &= ~0x03ff;	//2nd level page tables are 1k aligned
			val += 0x20000000;	//phys to virt
			val |= (( ((unsigned int)src) & 0x000ff000)>>(12-2));
			secondLevel = (unsigned int*)val;
			do {
				unsigned int l2;
				unsigned int contig;
				unsigned int start,next;
				val = *secondLevel;
				if ((val&3)==1) {
					//this is a large 64k entry, 16 entries are identical
					l2 = ( ((unsigned int)src) | 0xffff) + 1 - ((unsigned int)src); //just go to the next 64k block
					start = val & ~0x0ffff;
					next = start+(1<<16);
					start |= (((unsigned int)src) & 0xffff);
					//round up to next 64k block
					secondLevel = (unsigned int*) ((((unsigned int)secondLevel)|(0xf<<2))+4);
				} else if ( ((val&3)==2) 
                                            ||
                                            ((val&3)==3) ){
					//normal case 
					l2 = ( ((unsigned int)src) | 0xfff) + 1 - ((unsigned int)src); //just go to the next page
					start = val & ~0x0fff;
					next = start+(1<<12);
					start |= (((unsigned int)src) & 0xfff);
					secondLevel++;
				} else {
					printk( KERN_ERR "unexpected 2nd level mapping: 0x%x\n", val );
					return -1;
				}
				contig = l2;
				while (contig<l1) {
					unsigned int size,entries;
					val = *secondLevel;
					if ((val&3)==1) {
						size = 1<<16;
						entries = 16;
						val &= ~0xffff;
					} else if (((val&3)==2) 
                                                   ||
                                                   ((val&3)==3) ){
						size = 1<<12;
						entries = 1 ;
						val &= ~0x0fff;
					} else {
      						printk( KERN_ERR "unexpected 2nd level mapping2: 0x%x\n", val );
						return -1;
					}
					if (val != next ) break;
					secondLevel+= entries;
					contig += size;
					next += size;
				}
				if (contig > l1) contig = l1;
				cnt -= contig;
				l1 -= contig;
				src += contig;
				pT = TransferDesc(pT,physDest,start,contig,&pTphysLink,cnt);
				physDest += contig;
			} while (l1);
		} else {
			printk( KERN_ERR "unexpected mapping\n");
			return -1;
		}
	} while (cnt);
	
/*
	memset(pT,0,4*sizeof(long));
	pT[0] = 1 ;
	pT[3] = DCMD_ENDIRQEN ;
*/

        if (pTphysLink != pTphysStart) {
                dmac_clean_range(tableSpaceAlignedPtr,pT);
   		if(0 == (flags & FASYNC)){
			DDADR(dma) = pTphysStart;
			DCSR(dma) = DCSR_RUN;
DEBUGMSG( "wait %s\n", __FUNCTION__ );
			waitDMA(dma);
		}
                else {
//			pT[-1] |= DCMD_ENDIRQEN ;
			if( !validateDesc(tableSpaceAlignedPtr,totalLen) ){
				printDescr(tableSpaceAlignedPtr);
			}
			BUG_ON( (char *)pT > tableSpaceEnd );
DEBUGMSG( "-> %08lx/%08lx/%08lx\n",pTphysStart, DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR, DCSR_RUN|DCSR_STOPIRQEN );
			DDADR(dma) = pTphysStart;
                        DCSR(dma) = DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR ;
			DCSR(dma) = DCSR_RUN ;
DEBUGMSG( "| %08lx/%08lx/%08lx\n",DDADR(dma), DCSR(dma), DCMD(dma) );

// here's the benefit of FASYNC..
//			while (!(DCSR(dma) & DCSR_STOPSTATE)) cpu_relax();	//wait for dma completion interrupt
                }
	}
	else {
		DEBUGMSG( "no data: %08x/%08x\n", pTphysStart, pTphysLink );
	}
DEBUGMSG( "~%s:%08lx:%08lx:%08lx\n", __FUNCTION__, DDADR(dma), DCSR(dma), DCMD(dma) );
	return 0 ;
}

//dma completion interrupt handler
static void sm501_pxa_dma_irq(int dma, void *dummy)
{
   	DCSR(dma) = DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR;
}

#endif

static int yuv_open(struct inode *inode, struct file *filp)
{
   atomic_inc(&usage);
   printk( KERN_DEBUG "%s: %d uses\n", __FUNCTION__, atomic_read(&usage) );
   filp->private_data = 0 ;
#ifdef SM501_USE_PXA_DMA
	if (dmaChannel==(unsigned char)-1) {
		int dma = pxa_request_dma(CLASS_NAME, DMA_PRIO_LOW, sm501_pxa_dma_irq, NULL);
		if (dma >= 0){ 
                        DCSR(dma) = DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR ; // ack interrupts
			DDADR(dma) = 0 ;
			DCMD(dma) = 0 ;
			dmaChannel = dma;
		}
	}
        printk( KERN_DEBUG "SM501 using dma channel %d\n", dmaChannel );
#endif
   
   return 0 ;
}

static int yuv_release(struct inode *inode, struct file *filp)
{
   int uses = atomic_dec_return(&usage);
   printk( KERN_DEBUG "%s: %d uses\n", __FUNCTION__, uses );
   if( 0 == uses ){
      destroyPlane();
#ifdef SM501_USE_PXA_DMA
	if (dmaChannel != (unsigned char)-1){
		waitDMA(dmaChannel);
		pxa_free_dma(dmaChannel);
		dmaChannel = (unsigned char)-1;
        }
#endif
   }

   return 0;
}

static ssize_t yuv_write (struct file *filp, const char *buffer,size_t count, loff_t *ppos)
{
   struct sm501yuvPlane_t *pi = PLANEINFO(filp);
   if( 0 == pi ){
      pi = (struct sm501yuvPlane_t *)kzalloc( sizeof(struct sm501yuvPlane_t), GFP_KERNEL );
      pi->xLeft_ = 0 ;
      pi->yTop_  = 0 ;
      pi->inWidth_ = panelWidth/2 ;
      pi->inHeight_ = panelHeight/2 ;
      pi->outWidth_ = panelWidth ;
      pi->outHeight_ = panelHeight ;
      pi->planeOffset_ = 0 ; // output: offset into SM-501 RAM 

      createPlane( pi );      
      DEBUGMSG( "create default yuv plane %ux%u -> %ux%u\n", 
		pi->inWidth_, pi->inHeight_, panelWidth, panelHeight );
      filp->private_data = pi ;
   }

   if( pi )
   {
      int rval ;

      unsigned max = pi->inWidth_*pi->inHeight_*2 ;

      if( count > max )
         count = max ;

      *ppos += count ; // can it be NULL?
      rval = CP_FROM_USER(fbYUV, buffer, count);
      if( 0 == rval ){
         if( 0 == (READ_SM501_REG(VIDEO_DISPLAY_CTRL)&4) ){
            if( yuv )
               *DISPLAYREG(VIDEO_DISPLAY_CTRL)        = 0x00010307 ;   // enable, YUV, Interpolation
            else
               *DISPLAYREG(VIDEO_DISPLAY_CTRL)        = 0x00010305 ;   // enable, RGB565, Interpolation
         } // not yet enabled
   
         return count ;
      }
      else {
         printk( KERN_ERR "%s: short write %u of %u\n", __FUNCTION__, count-rval, count );
         return rval ;
      }
   }
   else
      return -EIO ;
}

static unsigned int yuv_poll(struct file *filp, poll_table *wait)
{
   unsigned returnval = POLLIN ;

   return returnval ;
}

static ssize_t yuv_read (struct file *filp, char *buffer,size_t count, loff_t *ppos)
{
   return count ;
}

// This routine allows the driver to implement device-
// specific ioctl's.  If the ioctl number passed in cmd is
// not recognized by the driver, it should return ENOIOCTLCMD.

static int yuv_ioctl( struct inode *inode, 
                        struct file  *filp,
		        unsigned int  cmd, 
                        unsigned long arg)
{
   int return_val= -ENOTTY;
   
   struct sm501yuvPlane_t *pi = PLANEINFO(filp);
   switch( cmd )
   {
      case SM501YUV_SETPLANE:
         {
            destroyPlane();
            if( 0 == pi )
            {
               pi = (struct sm501yuvPlane_t *)kmalloc( sizeof(struct sm501yuvPlane_t), GFP_KERNEL );
               filp->private_data = pi ;
            }
            
            if( 0 == copy_from_user( pi, (void *)arg, sizeof(*pi) ) )
            {
               createPlane( pi );
               return_val = copy_to_user( (void *)arg, pi, sizeof(*pi) );
            }
            else
               return_val = -EFAULT ;
            break;
         }
      case SM501YUV_SLICE: 
         {
            struct yuv_slice slice ;
            if( 0 == pi ){
               return_val = -EFAULT ;
               break ;
            }
            if( 0 == copy_from_user( &slice, (void *)arg, sizeof(slice)) ){
               /*
                * See if each of y, u, and v are in contiguous physical RAM
                */
               #define Y_SINGLE 1
               #define U_SINGLE 2
               #define V_SINGLE 4
               #define ALL_SINGLE (Y_SINGLE|U_SINGLE|V_SINGLE)
               int mask = 0 ;
               uint8_t *next ;
               uint8_t *end ;
               unsigned long page ;
               unsigned count ;
               
               page = virt_to_phys(slice.ybuf);
               next = slice.ybuf ;
               count = slice.ystride*slice.h ;
               end=slice.ybuf+count;
               mask = Y_SINGLE ;
               BUG_ON(0==next);
//            	consistent_sync(next,count,DMA_TO_DEVICE);
               while( next < end ){
                  unsigned long phys = virt_to_phys(next);
                  if( phys != page ){
                     mask &= ~Y_SINGLE ;
                     break ;
                  }
                  next += PAGE_SIZE ;
                  page += PAGE_SIZE ;
               }

               if( Y_SINGLE == mask ){
                  page = virt_to_phys(slice.ubuf);
                  next = slice.ubuf ;
                  count = slice.ustride*slice.h/2 ;
                  BUG_ON(0==next);
//                  consistent_sync(next,count,DMA_TO_DEVICE);
                  end=slice.ubuf+count;
                  mask |= U_SINGLE ;
                  while( next < end ){
                     unsigned long phys = virt_to_phys(next);
                     if( phys != page ){
                        mask &= ~U_SINGLE ;
                        break ;
                     }
                     next += PAGE_SIZE ;
                     page += PAGE_SIZE ;
                  }
               } // Y is physically contiguous
               
               if( (Y_SINGLE|U_SINGLE) == mask ){
                  page = virt_to_phys(slice.vbuf);
                  next = slice.vbuf ;
                  count = slice.vstride*slice.h/2 ;
                  BUG_ON(0==next);
//                  consistent_sync(next,next+count,DMA_TO_DEVICE);
                  end=slice.vbuf+count;
                  mask |= V_SINGLE ;
                  while( next < end ){
                     unsigned long phys = virt_to_phys(next);
                     if( phys != page ){
                        mask &= ~V_SINGLE ;
                        break ;
                     }
                     next += PAGE_SIZE ;
                     page += PAGE_SIZE ;
                  }
               }
               
               if( ALL_SINGLE == mask ){
                  unsigned fbw = 1280 ; // (READ_SM501_REG(SMIDISPCTRL_FBWIDTH) & 0x0FFF0000) >> 16 ;
                  unsigned fbh = 800 ; // (READ_SM501_REG(SMIDISPCTRL_FBHEIGHT) & 0x0FFF0000) >> 16 ;
printk( KERN_ERR "%s: %ux%u\n", __FUNCTION__, fbw, fbh );
                  return_val = 0 ;
                  *DISPLAYREG(SM501_CSC_YSOURCE)      = 0x08000000 | (virt_to_phys(slice.ybuf)&0x3FFFFFF);
                  *DISPLAYREG(SM501_CSC_YSOURCEX)     = 0 ;
                  *DISPLAYREG(SM501_CSC_YSOURCEY)     = 0 ;
                  *DISPLAYREG(SM501_CSC_USOURCE)      = 0x08000000 | (virt_to_phys(slice.ubuf)&0x3FFFFFF);
                  *DISPLAYREG(SM501_CSC_VSOURCE)      = 0x08000000 | (virt_to_phys(slice.vbuf)&0x3FFFFFF);
                  *DISPLAYREG(SM501_CSC_SRCDIM)       = (slice.w << 16)  | slice.h ;
                  *DISPLAYREG(SM501_CSC_SRCPITCH)     = ((slice.ystride/16) << 16) | (slice.ustride/16);
                  *DISPLAYREG(SM501_CSC_DESTINATION)  = (slice.x << 16) | slice.y ;
//                  *DISPLAYREG(SM501_CSC_DESTDIM)      = (slice.w << 16) | slice.h ; 
                  *DISPLAYREG(SM501_CSC_DESTPITCH)    = (((fbw*2)/16)<<16)|fbh ;
                  *DISPLAYREG(SM501_CSC_SCALE)        = 0x20002000 ;
//                  *DISPLAYREG(SM501_CSC_DESTBASE)     = READ_SM501_REG(0x8000c); // LCD frame buffer
                  *DISPLAYREG(SM501_CSC_CONTROL)      = READ_SM501_REG(SM501_CSC_CONTROL)
                                                      | 0x80000000 // start
//                                                      | 0x20000000 // Source is YUV420 ;
                                                      // no filtering
                                                      // dest is RGB565
                                                      ;
               } else {
                  printk( KERN_ERR "yuv non-contiguous: %x\n", mask );
               }
            }
            break;
         }
      default:
         printk( KERN_ERR "Invalid sm501yuv ioctl: %u, %u, %u\n", cmd, SM501YUV_SETPLANE, SM501YUV_SLICE );
   }

   return return_val;
}

// This structure is the file operations structure, which specifies what
// callbacks functions the kernel should call when a user mode process
// attempts to perform these operations on the device.

static struct file_operations yuv_fops = {
    owner:		THIS_MODULE,
    open:		yuv_open,
    read:		yuv_read,
    poll:		yuv_poll,
    write:		yuv_write,
    ioctl:		yuv_ioctl,
    release:	yuv_release
};

int sm501yuv_probe(struct device *dev)
{
   unsigned long dispCtrl, crtCtrl ;
   int result ;
   unsigned panelMemSize ;

   get_mmioVirtual();
   get_fbVirtual();

   dispCtrl = READ_SM501_REG( SMIGRAPH_DISPCTRL ); 
   crtCtrl = READ_SM501_REG( SMIGRAPH_CRTCTRL );
   if( ( dispCtrl & DISPCTRL_ENABLE )
       ||
       ( crtCtrl & CRTCTRL_ENABLE ) ){
DEBUGMSG( "%s:\n", __FUNCTION__ );
      yuvClass_ = class_create( THIS_MODULE, "yuv" );
      if( 0 != yuvClass_ )
      {
         struct device *c;
         result = register_chrdev(yuv_major,"yuv",&yuv_fops);
         if( result < 0 )
            return result;
      
         if (yuv_major==0) 
            yuv_major = result; //dynamic assignment
         
      	c = device_create( yuvClass_, NULL, MKDEV(yuv_major, 0), NULL, CLASS_NAME);
      	if (!IS_ERR(c)) {         
            if( dispCtrl & DISPCTRL_ENABLE ){
               panelWidth = ( READ_SM501_REG( SMIGRAPH_WIDTHREG ) >> 16 );
               panelHeight = ( READ_SM501_REG( SMIGRAPH_HEIGHTREG ) >> 16 );
            } else {
               panelWidth =  ( READ_SM501_REG( SMIGRAPH_CRTWIDTH ) >> 16 );
               panelHeight = ( READ_SM501_REG( SMIGRAPH_CRTVTOT ) & 0x7ff ) + 1 ;
            }
            if( 0 == (panelWidth*panelHeight) ){
               printk( KERN_ERR "%s: No YUV space\n", __FUNCTION__ );
               return -ENOMEM ;
            }
            panelMemSize = 2*panelWidth*panelHeight ;
            fbYUV = (char *)sm501_alloc(panelMemSize);
            if( 0 == fbYUV ){
               printk( KERN_ERR "Error allocating YUV ram\n" );
               return -ENOMEM ;
            }
            else {
               DEBUGMSG( KERN_ERR "yuv alloc: %u/%p\n", 2*panelWidth*panelHeight, fbYUV );
            }
   
            yuvStartOffset = (char *)fbYUV - (char *)fbVirtual ;
         
            printk( KERN_INFO "allocated YUV buffer at 0x%p/0x%lx\n",
                    fbYUV, yuvStartOffset );
         
            memcpy( mmioVirtual+videoRegAddr-sysRegAddr, videoRegValues, sizeof(videoRegValues));
            memset( fbYUV, 0x7f, panelWidth*panelHeight*2 );
         
            printk (KERN_INFO "yuv::init_module from Boundary Devices, 2004\n"
                             "major device %d\n"
                             "w:%u, h:%u\n"
                             "fb:0x%lx, len 0x%lx\n"
                             "mm:0x%lx, len 0x%lx\n", 
                    yuv_major, 
                    panelWidth, panelHeight,
                    fbStart, fbMax,
                    mmioStart, mmioLength );
         
            initialized = 1 ;
         } else {
      		printk( KERN_ERR "Unable to create class_device for yuv\n" );
      	}
      }
      else
         printk( KERN_ERR "%s: error creating class\n", __FUNCTION__ );
   }

   return 0 ;
}

static struct device_driver sm501yuv_driver = {
	.name = CLASS_NAME,
	.probe = sm501yuv_probe,
	.bus = &platform_bus_type
};

static int __init yuvInit(void)
{
   int rval ;
   rval = driver_register(&sm501yuv_driver);
   printk( KERN_DEBUG "yuv::yuvInit: %d\n", rval ); 
   return rval ;
}

static void yuvCleanup(void)
{
   printk( KERN_DEBUG "yuv::cleanup_module\n" ); 
   if( yuvClass_ ) {
      device_destroy(yuvClass_,MKDEV(yuv_major, 0));
      class_destroy(yuvClass_);
      yuvClass_ = 0 ;
   }
   
   if( initialized )
   {
      unregister_chrdev(yuv_major,"yuv" );
   }
}

MODULE_DESCRIPTION( "loadable yuv driver for SM-501");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boundary Devices");

module_init(yuvInit);
module_exit(yuvCleanup);
