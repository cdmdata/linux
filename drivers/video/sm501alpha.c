/* 
 * linux/drivers/video/sm501alpha.c
 *
 * This module provides read/write access to the alpha layer
 * of the Silicon Motion SM-501 graphics controller.
 *
 * Refer to figure 1.9 of the SM-501 reference manual for 
 * details of how this relates to the Graphics, Video, and
 * Video Alpha layers.
 *
 * $Id: sm501alpha.c,v 1.1 2006/08/16 02:53:59 ericn Exp $
 *
 * Revision History:
 *
 * $Log: sm501alpha.c,v $
 * Revision 1.1  2006/08/16 02:53:59  ericn
 * -Initial import
 *
 *
 *
 * Copyright 2006, Boundary Devices
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
#include <linux/mm.h>
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
#include <linux/sm501-int.h>
#include <linux/sm501alpha.h>
#include <linux/platform_device.h>
#include <mach/pxa2xx-regs.h>

#include "sm501.h"

#define ALPHA_MAJOR 0

static int alpha_major = ALPHA_MAJOR ;
static int initialized = 0 ;

module_param(alpha_major, int, ALPHA_MAJOR);
MODULE_PARM_DESC(alpha_major, "Choose major device number");

#define CLASSNAME SM501ALPHA_CLASS

static struct class *alphaClass_ = 0 ;
static char *alphaAlloc_ = 0 ;
static char *fbAlpha_ = 0 ;
static unsigned alphaLen_ = 0 ;

#define SM501_DEBUG

#ifdef SM501_DEBUG
#define DEBUGMSG( __fmt, ... ) printk( __fmt, ## __VA_ARGS__ )
#else
#define DEBUGMSG( __fmt, ... ) 
#endif

static int createPlane( struct sm501_alphaPlane_t *plane )
{
   if( 0 == fbAlpha_ ){
      unsigned panelWidth = ( READ_SM501_REG( SMIGRAPH_WIDTHREG ) >> 16 );
      unsigned panelHeight = ( READ_SM501_REG( SMIGRAPH_HEIGHTREG ) >> 16 );
      unsigned const bytesPerPix = (SM501_ALPHA_RGBA44==plane->mode_) 
                                   ? 1
                                   : 2 ;
      unsigned const bytesPerLine = panelWidth*bytesPerPix ;
      unsigned const alphaLen = PAGE_ALIGN(panelHeight*bytesPerLine+PAGE_SIZE);
      unsigned i ;
      unsigned colorTblBase ;

      alphaAlloc_ = (char *)sm501_alloc(alphaLen);
      fbAlpha_ = (char *)PAGE_ALIGN((unsigned long)alphaAlloc_);
      if( 0 == fbAlpha_ ){
         printk( KERN_ERR "Error allocating %u bytes of alpha plane\n", alphaLen );
         return -ENOMEM ;
      }

      memset( alphaAlloc_, 0, alphaLen ); // transparent by default
      alphaLen_ = alphaLen-(fbAlpha_-alphaAlloc_);

      plane->planeOffset_ = VIDEORAMPTR(fbAlpha_);
      STUFF_SM501_REG( SMIALPHA_FBADDR, plane->planeOffset_ );
      STUFF_SM501_REG( SMIALPHA_FBOFFS, (bytesPerLine<<16)|bytesPerLine );
      STUFF_SM501_REG( SMIALPHA_TL, 0 );
      STUFF_SM501_REG( SMIALPHA_BR, (panelHeight<<16) | panelWidth );
      STUFF_SM501_REG( SMIALPHA_CHROMA, 0 );

      colorTblBase = SMIALPHA_COLORTBL ;
      for( i = 0 ; i < sizeof(plane->palette_)/sizeof(plane->palette_[0]) ; i++ ){
         STUFF_SM501_REG( colorTblBase + (i*4), plane->palette_[i] );
      }
      //
      // Turn on alpha
      //       per pixel alpha
      //       3 entries to fill FIFO
      //       No interpolation
      //       No chroma
      //       
      //
      STUFF_SM501_REG( SMIALPHA_CONTROL,
                     (1<<16)      // FIFO
                     |(1<<2)      // enable
                     | plane->mode_ );

      return 0 ;
   }

   return -EBUSY ;
}

static void destroyPlane( void )
{
   if( fbAlpha_ ){
      sm501_free( alphaAlloc_ );
      fbAlpha_ = alphaAlloc_ = 0 ;
      STUFF_SM501_REG( SMIALPHA_CONTROL, 0 ); // disable alpha layer until told differently
   }
}

static int alpha_open(struct inode *inode, struct file *filp)
{
   filp->private_data = 0 ;
   DEBUGMSG( "%s:\n", __FUNCTION__ );

   return 0 ;
}

static int alpha_release(struct inode *inode, struct file *filp)
{
   DEBUGMSG( "%s:\n", __FUNCTION__ );
   destroyPlane();

   return 0;
}

static ssize_t alpha_write (struct file *filp, const char *buffer,size_t count, loff_t *ppos)
{
   return -EIO ;
}

static unsigned int alpha_poll(struct file *filp, poll_table *wait)
{
   unsigned returnval = POLLIN ;

   return returnval ;
}

static ssize_t alpha_read (struct file *filp, char *buffer,size_t count, loff_t *ppos)
{
   return count ;
}


// This routine allows the driver to implement device-
// specific ioctl's.  If the ioctl number passed in cmd is
// not recognized by the driver, it should return ENOIOCTLCMD.

static int alpha_ioctl( struct inode *inode, 
                        struct file  *filp,
                        unsigned int  cmd, 
                        unsigned long arg)
{
   int return_val= -ENOTTY;
   
   DEBUGMSG( "%s:%d/%ld\n", __FUNCTION__, cmd, arg );

   switch( cmd )
   {
      case SM501ALPHA_SETPLANE:
         {
            struct sm501_alphaPlane_t plane ;
            if( 0 == ( return_val = copy_from_user( &plane, (void *)arg, sizeof(plane) ) ) )
            {
               destroyPlane();
               if( 0 == ( return_val = createPlane( &plane ) ) )
                  return_val = copy_to_user( (void *)arg, &plane, sizeof(plane) );
            }
            else
               return_val = -EFAULT ;
            break;
         }
      default:
         printk( KERN_ERR "Invalid sm501alpha ioctl: %u\n", cmd );
   }

   return return_val;
}

static int alpha_mmap(struct file * file, struct vm_area_struct * vma)
{
   unsigned long offset = fbAlpha_-get_fbVirtual();
   unsigned long mapSize = vma->vm_end-vma->vm_start ;
   if( mapSize > alphaLen_ )
      return -EINVAL ;
   if( 0 == fbAlpha_ )
      return -EINVAL ;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_IO|VM_RESERVED|VM_PFNMAP;
   vma->vm_pgoff = (SM501_FBSTART+offset) >> PAGE_SHIFT;
   return io_remap_pfn_range( vma, vma->vm_start, vma->vm_pgoff,
                              mapSize, vma->vm_page_prot);
}


// This structure is the file operations structure, which specifies what
// callbacks functions the kernel should call when a user mode process
// attempts to perform these operations on the device.

static struct file_operations alpha_fops = {
    owner:	THIS_MODULE,
    open:	alpha_open,
    read:	alpha_read,
    poll:	alpha_poll,
    write:	alpha_write,
    ioctl:	alpha_ioctl,
    release:	alpha_release,
    mmap: alpha_mmap
};

int sm501alpha_probe(struct device *dev)
{
   int result ;

   DEBUGMSG( "%s:\n", __FUNCTION__ );

   alphaClass_ = class_create( THIS_MODULE, CLASSNAME );
   if( 0 != alphaClass_ )
   {
      struct device *c;
      result = register_chrdev(alpha_major,CLASSNAME,&alpha_fops);
      if( result < 0 )
         return result;
      if (alpha_major==0) 
         alpha_major = result; //dynamic assignment
      c = device_create( alphaClass_, NULL, MKDEV(alpha_major, 0), NULL, CLASSNAME);
      if (!IS_ERR(c)) {
         printk(KERN_ERR "%s: class device created\n", CLASSNAME );
      } else {
         /* Not fatal */
         printk(KERN_WARNING "%s: Unable to create device\n", CLASSNAME );
      }
   }
   else
      printk( KERN_ERR "%s: error creating class\n", CLASSNAME );
   
   get_mmioVirtual();
   get_fbVirtual();

   printk (KERN_ERR "alpha::init_module from Boundary Devices, 2007\n"
                    "major device %d\n",
           alpha_major );

   initialized = 1 ;
   return 0 ;
}

static struct device_driver sm501alpha_driver = {
	.name	= SM501ALPHA_CLASS,
	.probe	= sm501alpha_probe,
	.bus = &platform_bus_type
};

static int __init alphaInit(void)
{
   int rval ;
   rval = driver_register(&sm501alpha_driver);
   return rval ;
}

static void alphaCleanup(void)
{
   if( initialized )
      unregister_chrdev(alpha_major,"alpha" );
}

MODULE_DESCRIPTION( "loadable alpha layer driver for SM-501");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boundary Devices");

module_init(alphaInit);
module_exit(alphaCleanup);
