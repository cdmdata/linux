/* 
 * linux/drivers/video/sm501mem.c
 *
 * This module provides memory allocation support for 
 * a Silicon Motion SM-501 graphics controller.
 *
 * $Id: sm501mem.c,v 1.2 2006/11/23 19:03:12 ericn Exp $
 *
 * Revision History:
 *
 * $Log: sm501mem.c,v $
 * Revision 1.2  2006/11/23 19:03:12  ericn
 * -additional check on memory dealloc
 *
 * Revision 1.1  2006/08/16 02:53:59  ericn
 * -Initial import
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
#include <linux/mm.h>
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
#include <linux/list.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/ioport.h>

#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/mach/map.h>
#include <linux/sm501mem.h>
#include <linux/sm501-int.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <mach/pxa2xx-regs.h>

#include "sm501.h"
#include "ffit.h"

// #define SM501_DEBUG

#ifdef SM501_DEBUG
#define DEBUGMSG( __fmt, ... ) printk( __fmt, ## __VA_ARGS__ )
#else
#define DEBUGMSG( __fmt, ... ) 
#endif

static int mem_major = 0 ;
static int initialized = 0 ;
extern unsigned long fb0_offs ;

#define CLASSNAME SM501MEM_CLASS
static struct class *memClass_ = 0 ;

typedef struct {
   struct list_head node_ ;
   unsigned long    size_ ;
   unsigned long    pad_ ; // to 16-bytes
} allocHeader_t ;

static int mem_open(struct inode *inode, struct file *filp)
{
   struct list_head *allocs = (struct list_head *)kmalloc( sizeof(struct list_head), GFP_KERNEL );
   if( allocs )
   {
      INIT_LIST_HEAD(allocs);
      filp->private_data = allocs ;
DEBUGMSG( "%s:\n", __FUNCTION__ );
      return 0 ;
   }

   return -ENOMEM ;
}

static int mem_release(struct inode *inode, struct file *filp)
{
   struct list_head *allocs = (struct list_head *)filp->private_data ;
DEBUGMSG( "%s:\n", __FUNCTION__ );
   if( allocs ){

      while( !list_empty(allocs) )
      {
         allocHeader_t *node = (allocHeader_t *)allocs->next ;
         if( ( 0 == node->node_.next )
             ||
             ( 0 == node->node_.prev )
             ||
             ( &node->node_ == node->node_.next )
             ||
             ( &node->node_ == node->node_.prev )
             ||
             ( 0 != ((unsigned long)node->node_.next & 15 ) )
             ||
             ( 0 != ((unsigned long)node->node_.prev & 15 ) ) ){
            printk( KERN_ERR "Invalid memory node %p/%p/%p/%lu/%lu\n", 
                    node, 
                    node ? node->node_.next : 0,
                    node ? node->node_.prev : 0,
                    node ? node->size_ : 0, 
                    node ? node->pad_ : 0 
            );
            break ;
         }
         else {
            list_del(&node->node_);
            sm501_free(node);
         }
      }

      kfree( allocs );
   }

   filp->private_data = 0 ;

   return 0;
}

static ssize_t mem_write (struct file *filp, const char *buffer,size_t count, loff_t *ppos)
{
   DEBUGMSG( CLASSNAME "::write\n" );
   return -EIO ;
}

static unsigned int mem_poll(struct file *filp, poll_table *wait)
{
   DEBUGMSG( CLASSNAME "::poll\n" );
   return -EIO ;
}

static ssize_t mem_read (struct file *filp, char *buffer,size_t count, loff_t *ppos)
{
   DEBUGMSG( CLASSNAME "::read\n" );
   return -EIO ;
}

static int mem_ioctl( 
   struct inode *inode, 
   struct file  *filp,
	unsigned int  cmd, 
   unsigned long arg )
{
   struct list_head *allocs = (struct list_head *)filp->private_data ;
   if( 0 == allocs )
      return -EFAULT ;

DEBUGMSG( "%s:\n", __FUNCTION__ );
   switch (cmd) {
      case SM501_ALLOC: {
         unsigned size ;
         allocHeader_t *mem ;
         unsigned long  offs ;
         
         if( copy_from_user( &size, (void __user *)arg, sizeof(size) ) )
            return -EFAULT;

         if( 0 == size ){
            printk( KERN_ERR "alloc zero size\n" );
            return -ENOMEM ;
         }

         mem = (allocHeader_t *)sm501_alloc(size+sizeof( allocHeader_t ));

         if( 0 == mem ){
            printk( KERN_ERR "SM501_ALLOC error: %u bytes\n", size );
            return -ENOMEM ;
         }

         mem->size_ = size ;
         mem->pad_  = 0 ;
         list_add( &mem->node_, allocs );

         offs = (char *)(mem+1) - fbVirtual - fb0_offs ;

         if( copy_to_user( (void __user *)arg, &offs, sizeof(offs) ) )
            return -EFAULT ;
         else
            return 0 ;
      }
      case SM501_BASEADDR: {
         if( copy_to_user( (void __user *)arg, &fb0_offs, sizeof(fb0_offs) ) )
            return -EFAULT ;
         else
            return 0 ;
      }
      case SM501_FREE: {
         allocHeader_t *mem ;
         unsigned long  offs ;

         if( copy_from_user( &offs, (void __user *)arg, sizeof(offs) ) )
            return -EFAULT;

         if( (0 == offs) || (SM501_FBMAX-fb0_offs <= offs) ){
            printk( KERN_ERR "Invalid SM501 free: %lx (max %lx)\n", offs, SM501_FBMAX-fb0_offs );
            return -EFAULT ;
         }
         offs += fb0_offs ; // mmap offset
         mem = (allocHeader_t *)(fbVirtual + offs - sizeof(allocHeader_t) );

         if( ( 0 == mem->node_.next )
             ||
             ( 0 == mem->node_.prev )
             ||
             ( &mem->node_ == mem->node_.next )
             ||
             ( &mem->node_ == mem->node_.prev )
             ||
             ( 0 != ((unsigned long)mem->node_.next & 15 ) )
             ||
             ( 0 != ((unsigned long)mem->node_.prev & 15 ) ) )
         {
            printk( KERN_ERR "free invalid ptr: %p/%p/%p\n", 
                    &mem->node_,
                    mem->node_.next,
                    mem->node_.prev );
            return -EFAULT ;
         }

         list_del(&mem->node_);
         memset( mem, 0xaa, mem->size_ + sizeof(allocHeader_t) );
         sm501_free(mem);
         return 0 ;
      }
	}
	return -EINVAL;
}

static int mem_mmap(struct file * file, struct vm_area_struct * vma)
{
   vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_flags |= VM_IO|VM_RESERVED|VM_PFNMAP;
   vma->vm_pgoff = SM501_FBSTART>>PAGE_SHIFT ;
   return io_remap_pfn_range( vma, vma->vm_start, vma->vm_pgoff,
                              SM501_FBMAX, vma->vm_page_prot);
}

static struct file_operations mem_fops = {
      owner:	THIS_MODULE
    , open:    mem_open
    , read:		mem_read
    , poll:		mem_poll
    , write:	mem_write
    , ioctl:	mem_ioctl
    , release:	mem_release
    , mmap:    mem_mmap
};


static int sm501mem_read_proc( char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
   if( memPool_ ){
      list_t *last_phys = 0 ;
      list_t *b = memPool_->head ;
      printk( KERN_INFO "\nPRINTING FREE LIST\n\n");
      while (b) {
         printk( KERN_INFO "<0x%x> size: %d used: %d, last: %d\n", (int)b,  
                 (int) GET_BLOCK_SIZE (b), IS_USED_BLOCK(b), IS_LAST_BLOCK(b));
         printk( KERN_INFO "\tprev_free: 0x%x\n", (int) b -> mem.free_ptr.prev);
         printk( KERN_INFO "\tnext_free: 0x%x\n", (int) b -> mem.free_ptr.next);
         printk( KERN_INFO "\tprev_phys: 0x%x\n", (int)b -> prev_phys);
         if( !IS_LAST_BLOCK (b) ){
            last_phys = (list_t *)( (int) b + GET_BLOCK_SIZE (b) + LISTHEADERSIZE );
            printk( KERN_INFO "\tnext_phys: %p\n", last_phys );
         } else {
            printk( KERN_INFO "\tnext_phys: 0x0\n");
            break;
         }
         b = b -> mem.free_ptr.next;
      }
      if( last_phys ){
         printk( KERN_INFO "--- last phys:\n" );
         printk( KERN_INFO "<0x%x> size: %d used: %d, last %d\n", (int)last_phys,  
                 (int) GET_BLOCK_SIZE (last_phys), IS_USED_BLOCK(last_phys), IS_LAST_BLOCK(last_phys));
         printk( KERN_INFO "\tprev_free: 0x%x\n", (int) last_phys -> mem.free_ptr.prev);
         printk( KERN_INFO "\tnext_free: 0x%x\n", (int) last_phys -> mem.free_ptr.next);
         printk( KERN_INFO "\tprev_phys: 0x%x\n", (int)last_phys -> prev_phys);
         return snprintf( page, 512, "x" );
      }
      return 0 ;
   }
   else
      return snprintf( page, 512, "Memory pool not allocated\n" );
}

static int sm501mem_write_proc( struct file *file, const char __user *buffer,
                                unsigned long count, void *data )
{
   if( memPool_ ){
/*
      list_t *b = memPool_->first_block;
      while (1) {
         printk( KERN_INFO "<0x%x> size: %d used: %d\n", (int)b, GET_BLOCK_SIZE(b), IS_USED_BLOCK(b));
         printk( KERN_INFO "\tprev_phys: 0x%x\n", (int)b -> prev_phys);
         if (!IS_LAST_BLOCK (b))
            printk( KERN_INFO "\tnext_phys: 0x%x\n",  (int) b + GET_BLOCK_SIZE (b) 
                     + HEADER_SIZE);
         else {
            printk( KERN_INFO "\tnext_phys: 0x0\n");
            break;
         }

         b = (list_t *) (((char *) b) + (unsigned long) LISTHEADERSIZE + (unsigned long) GET_BLOCK_SIZE(b));
      }
*/      
   }
   return count ;
}

int sm501mem_probe(struct device *dev)
{
   memClass_ = class_create( THIS_MODULE, CLASSNAME );
   if( 0 != memClass_ )
   {
      struct device *c;
      int result ;
      struct proc_dir_entry *pde ;

DEBUGMSG( "%s:\n", __FUNCTION__ );
      result = register_chrdev(mem_major,CLASSNAME,&mem_fops);
      if( result < 0 )
         return result;

      if (mem_major==0) 
         mem_major = result; //dynamic assignment

      c = device_create( memClass_, NULL, MKDEV(mem_major, 0), NULL, CLASSNAME);
      if (IS_ERR(c)) {
         /* Not fatal */
         printk(KERN_WARNING "%s: Unable to create device\n", __FUNCTION__ );
      }

      get_mmioVirtual();
      get_fbVirtual();

      pde = create_proc_entry("driver/sm501mem", 0, 0);
      if( pde ) {
         pde->read_proc  = sm501mem_read_proc ;
         pde->write_proc = sm501mem_write_proc ;
      }

      printk (KERN_ERR CLASSNAME "::init_module from Boundary Devices, 2006\n"
                       "major device %d\n", mem_major );

      initialized = 1 ;
   }
   else
      printk( KERN_ERR "%s: error creating class\n", __FUNCTION__ );

   return 0 ;
}

static struct device_driver sm501mem_driver = {
	.name = CLASSNAME,
	.probe = sm501mem_probe,
	.bus = &platform_bus_type
};

static int __init memInit(void)
{
   int rval ;
DEBUGMSG( "%s:\n", __FUNCTION__ );
   rval = driver_register(&sm501mem_driver);
   return rval ;
}

static void memCleanup(void)
{
   if( initialized ) {
DEBUGMSG( "%s:\n", __FUNCTION__ );
      unregister_chrdev(mem_major,"mem" );
   }
}

MODULE_DESCRIPTION( "loadable memory driver for SM-501");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boundary Devices");

module_init(memInit);
module_exit(memCleanup);
