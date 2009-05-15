/*
 * sm501-cmd.c
 *
 * This driver provides Silicon Motion SM-501 command-list handling
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

#define CLASSNAME SM501CMD_CLASS

static int cmdlist_major = 0 ;
static struct class *cmdClass_ = 0 ;
struct fasync_struct *cmdlist_async = 0 ;

static int cmdlist_release(struct inode *inode, struct file *filp)
{
   clearCmdList();
   fasync_helper(-1, filp, 0, &cmdlist_async );
   return 0;
}

static int cmdlist_fasync(int fd, struct file *filp, int on)
{
	int retval;

	retval = fasync_helper(fd, filp, on, &cmdlist_async );
	if (retval < 0)
		return retval;
	return 0;
}

static void cmdlist_callback(ulong param)
{
   kill_fasync((struct fasync_struct **)param, SIGIO, POLL_IN );
}

static ssize_t cmdlist_write( struct file *filp, const char *buffer,size_t count, loff_t *ppos)
{
   if( 0 == (count & 3) ){
      ssize_t numWritten = 0 ;
      void (*callback)(ulong param) = 0 ;
      int rval = 0 ;

      while( numWritten < count )
      {
         ulong addr ;
         if( 0 != (rval = copy_from_user(&addr,buffer,sizeof(addr)) ) )
            break ;
         
         numWritten += sizeof(addr);
         buffer += sizeof(addr);

         if( count == numWritten )
            callback = cmdlist_callback ;
         else
            callback = 0 ;

         if( 0 != ( rval = executeCommand( addr, callback, (ulong)&cmdlist_async ) ) )
            break ;

      } while( numWritten < count );

      *ppos += numWritten ;

      return (rval == 0) ? numWritten : rval ;

   }
   else
      return -EFAULT ;
}

static int cmdlist_open(struct inode *inode, struct file *filp)
{
   filp->private_data = 0 ;
   DEBUGMSG( "%s:\n", __FUNCTION__ );

   return 0 ;
}

static struct file_operations cmdlist_fops = {
      owner:     THIS_MODULE
    , open:      cmdlist_open
    , release:   cmdlist_release
    , write:     cmdlist_write
    , fasync:    cmdlist_fasync
};

int sm501cmd_probe(struct device *dev)
{
   cmdClass_ = class_create( THIS_MODULE, CLASSNAME );
   if( 0 != cmdClass_ )
   {
      struct device *c;
      int result = register_chrdev(cmdlist_major,CLASSNAME,&cmdlist_fops);
      if( result < 0 )
         return result;
      if (cmdlist_major==0) 
         cmdlist_major = result; //dynamic assignment
      printk( KERN_ERR "%s: class created\n", CLASSNAME );
      c = device_create( cmdClass_, NULL, MKDEV(cmdlist_major, 0), NULL, CLASSNAME);
      if (IS_ERR(c)) {
         /* Not fatal */
         printk(KERN_WARNING "%s: Unable to create device\n", CLASSNAME );
      }
   }
   else
      printk( KERN_ERR "%s: error creating class\n", CLASSNAME );

   printk( KERN_ERR "SM-501 commandlist driver version " VERSION " from Boundary Devices, 2007\n" 
           "major device node %d(cmdlist)\n", cmdlist_major );

   return 0 ;
}

static struct device_driver sm501cmd_driver = {
	.name = CLASSNAME,
	.probe = sm501cmd_probe,
	.bus = &platform_bus_type
};

static int __init sm501_cmd_init(void)
{
      int rval ;
DEBUGMSG( "%s:\n", __FUNCTION__ );
      rval = driver_register(&sm501cmd_driver);
      printk( KERN_ERR "cmd::cmdInit: %d\n", rval ); 
      return rval ;
}

static void __exit sm501_cmd_exit (void)
{
}

MODULE_AUTHOR("Boundary Devices");
MODULE_LICENSE("GPL");

module_init(sm501_cmd_init);
module_exit(sm501_cmd_exit);

