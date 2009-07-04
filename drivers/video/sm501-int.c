/*
 * sm501-int.c
 *
 * This driver provides Silicon Motion SM-501 interrupt handling for
 * both the video and USB drivers.
 *
 * Copyright (c) Boundary Devices, 2006
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/poll.h>

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/spinlock.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/mach/map.h>
#include <mach/pxa2xx-regs.h>
#include <linux/sm501-int.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include "../video/sm501.h"

#define VERSION "$Revision: 1.2 $"     /* Driver revision number */

// #define SM501_DEBUG

#ifdef SM501_DEBUG
#define DEBUGMSG( __fmt, ... ) printk( __fmt, ## __VA_ARGS__ )
#else
#define DEBUGMSG( __fmt, ... ) 
#endif

#ifdef CONFIG_MACH_NEON270
#define SM501_IRQ IRQ_GPIO(22)
#else
#define SM501_IRQ IRQ_GPIO(5)
#endif

#define CLASSNAME SM501INT_CLASS

static int int_major = 0 ;

struct SM501_IntData* intD = NULL;
static int haveIrq = 0 ;
char *mmioVirtual = 0 ;
char *fbVirtual = 0 ;

static struct class *intClass_ = 0 ;

static struct fasync_struct *vsync_async = 0 ;

char *get_mmioVirtual( void )
{
   if( 0 == mmioVirtual ){
      mmioVirtual = ioremap_nocache(SM501_MMIOSTART, SM501_MMIOLENGTH);
DEBUGMSG("SM-501 mmio: %p\n", mmioVirtual );
   }
   return mmioVirtual ;
}

char *get_fbVirtual( void )
{
   if( 0 == fbVirtual ){
      fbVirtual = ioremap(SM501_FBSTART, SM501_FBMAX);
DEBUGMSG("SM-501 fb: %p\n", fbVirtual );
   }
   return fbVirtual ;
}

int SM501_grab_int_slot(port_t cfgBase, unsigned int slotnum,handler501_t h,void* data)
{
   int rval ;
   if( !haveIrq )
   {
      rval = request_irq( SM501_IRQ, sm501_interrupt, IRQF_TRIGGER_RISING, "SM-501", 0 );
      if( 0 == rval ){
         haveIrq = 1 ;
      }
      else
         DEBUGMSG("Error grabbing SM-501 IRQ %p\n", sm501_interrupt );
   }
   else
      rval = 0 ;

   if( 0 == rval ){
      if (slotnum<32) {
         if (!intD) {
            intD = (struct SM501_IntData*)kmalloc(sizeof(*intD), GFP_KERNEL);
DEBUGMSG("allocated intD %p\n", intD );
            memset(intD, 0, sizeof(*intD));
            intD->cfgBase = cfgBase;
         }
	 else {
DEBUGMSG("??? already allocated intD %p\n", intD );
	 }
         if (intD) {
            struct handlerData *hd = intD->hd[slotnum];
            int i ;
            for( i = 0 ; i < SM501_MAX_HANDLERS_PER_INT ; i++ )
            {
               if( 0 == hd[i].handler )
               {
                  hd[i].hdata=data;
DEBUGMSG("mask int %d/%p/%p/%p\n", i, cfgBase, hd, data );
                  hd[i].handler=h;
                  SMI_REG(cfgBase,SMIR_INT_MASK_REG) |= (1<<slotnum);
                  break ;
               }
            }
         }
      }
      else {
         DEBUGMSG("Invalid SM-501 int slot #%d\n", slotnum );
      }
   }
   return rval ;
}

irqreturn_t sm501_interrupt (int irq, void *param)
{
   if (intD) {
      port_t cfgBase = intD->cfgBase;
      unsigned long mask = SMI_REG(cfgBase,SMIR_INT_MASK_REG);
      while (1) {
         unsigned long pending = SMI_REG(cfgBase,SMIR_INT_STATUS_REG) & mask;
         if (pending==0) break;
         SMI_REG(cfgBase,SMIR_INT_MASK_REG) = 0;   //this also zeros the status reg, so status must be read first
         while (pending){
            int i = __ffs(pending);
            unsigned long const notbitmask = ~(1<<i);
            struct handlerData *hd = intD->hd[i];
            
            pending &= notbitmask;
            if(hd->handler){
DEBUGMSG("h%p/%08lx\n", hd->handler, pending );
               while(hd->handler){
                  handler501_t h = hd->handler;
DEBUGMSG("h2:%p\n", h );
                  if (h) {
                     h(i,hd->hdata);
                  }
                  hd++ ;
               } // walk all registered handlers for this interrupt
            } else {
               mask &= notbitmask;
DEBUGMSG("~h%08lx\n", mask );
            }
         }
         SMI_REG(cfgBase,SMIR_INT_MASK_REG) = mask; //restore to get status bits back
      }
   }
   return IRQ_HANDLED;
}

static void vsyncDev_vsync(int slotnum, void * hdata)
{
   kill_fasync((struct fasync_struct **)hdata, SIGIO, POLL_IN );
   STUFF_SM501_REG( SMIR_INT_CLEAR_REG, (1<<SMI_RAW_INTERRUPT_PVS)|(1<<SMI_RAW_INTERRUPT_CVS));
}

static int vsync_release(struct inode *inode, struct file *filp)
{
   fasync_helper(-1, filp, 0, &vsync_async );
   return 0;
}

static int vsync_fasync(int fd, struct file *filp, int on)
{
	int retval;

	retval = fasync_helper(fd, filp, on, &vsync_async );
	if (retval < 0)
		return retval;
	return 0;
}

static int vsync_open(struct inode *inode, struct file *filp)
{
   filp->private_data = 0 ;
   DEBUGMSG( "%s:\n", __FUNCTION__ );

   return 0 ;
}

static struct file_operations vsync_fops = {
      owner:     THIS_MODULE
    , open:      vsync_open
    , release:   vsync_release
    , fasync:    vsync_fasync,
};

int sm501int_probe(struct device *dev)
{
   int result ;
   
   get_mmioVirtual();
   get_fbVirtual();

   intClass_ = class_create( THIS_MODULE, CLASSNAME );
   if( 0 != intClass_ )
   {
      struct device *c;
      result = register_chrdev(int_major,CLASSNAME,&vsync_fops);
      if( result < 0 )
         return result;
      if (int_major==0) 
         int_major = result; //dynamic assignment
      c = device_create( intClass_, NULL, MKDEV(int_major, 0), NULL, CLASSNAME);
      if (!IS_ERR(c)) {
         printk(KERN_ERR "%s: class device created\n", CLASSNAME );
      } else {
         /* Not fatal */
         printk(KERN_WARNING "%s: Unable to create device\n", CLASSNAME );
      }
   }
   else
      printk( KERN_ERR "%s: error creating class\n", CLASSNAME );

   printk( KERN_ERR "SM-501 interrupt driver version " VERSION " from Boundary Devices, 2007\n" 
           "major device nodes %d(vsync)\n", int_major );
   
   SM501_grab_int_slot(mmioVirtual,SMI_INTERRUPT_PVS,vsyncDev_vsync,&vsync_async);

   return 0 ;
}

static struct device_driver sm501int_driver = {
	.name = CLASSNAME,
	.probe = sm501int_probe,
	.bus = &platform_bus_type
};

//
// This function is called to initialise the driver, either from misc.c at
// bootup if the driver is compiled into the kernel, or from init_module
// below at module insert time. It attempts to register the device node.
//
static int __init sm501_int_init(void)
{
      int rval ;
DEBUGMSG( "%s:\n", __FUNCTION__ );
      rval = driver_register(&sm501int_driver);
      return rval ;
}

static void __exit sm501_int_exit (void)
{
    printk (KERN_ERR "SM-501 interrupt driver version " VERSION " from Boundary Devices, 2006\n" );
}

MODULE_AUTHOR("Boundary Devices");
MODULE_LICENSE("GPL");

module_init(sm501_int_init);
module_exit(sm501_int_exit);

EXPORT_SYMBOL(mmioVirtual);
EXPORT_SYMBOL(fbVirtual);
EXPORT_SYMBOL(SM501_grab_int_slot);
EXPORT_SYMBOL(get_mmioVirtual);
EXPORT_SYMBOL(get_fbVirtual);
