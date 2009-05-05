/*
 * linux/drivers/video/sm501.c
 *
 * This is a minimal driver for the Silicon Motion SM-501
 * graphics adapter for use with a Hitachi TX16D11VM2CBA
 * LCD display. Not very generic, but pretty small and simple.
 *
 * Copyright (C) Boundary Devices, 2004. No rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copied in part from the anakinfb.c module, because it appears
 * to be the smallest fully-functional FB device around.
 *
 * Changelog:
 *   $Log: sm501.c,v $
 *   Revision 1.7  2006/10/19 00:27:40  ericn
 *   -Use symbolic constant
 *
 *   Revision 1.6  2006/08/21 19:11:14  ericn
 *   -remove page-flipping code, blt support
 *
 *   Revision 1.5  2006/08/16 02:50:07  ericn
 *   -add memory allocation interface
 *
 *   Revision 1.4  2005/11/19 20:06:27  tkisky
 *   -don't reserve mmio, usb has it too for now
 *
 *   Revision 1.3  2005/11/19 20:00:16  tkisky
 *   -don't map usb meg of buffer
 *
 *   Revision 1.2  2005/11/06 15:41:23  ericn
 *   -export get_var, get_fix
 *
 *   Revision 1.1  2005/10/21 06:16:58  ericn
 *   -Initial import
 *
 *   Revision 1.8  2005/05/01 17:09:34  ericn
 *   -initialize CRT regs
 *
 *   Revision 1.7  2005/04/23 18:54:34  ericn
 *   -allow pre-configured SM501 (from U-boot)
 *
 *   Revision 1.6  2005/03/30 14:56:29  ericn
 *   -don't re-initialize display controller if already enabled
 *
 *   Revision 1.5  2005/01/27 01:26:46  tkisky
 *   -add screen size select to SM501
 *
 *   Revision 1.4  2005/01/23 22:59:17  ericn
 *   -get ready for new displays
 *
 *   Revision 1.3  2004/10/30 15:05:04  ericn
 *   -added support for 320x240
 *
 *   Revision 1.2  2004/10/03 03:15:15  ericn
 *   -disable hardware cursor
 *
 *   Revision 1.1  2004/10/03 03:09:27  ericn
 *   -Initial import
 *
 */
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/kmod.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include "sm501.h"
#include "linux/sm501-int.h"
#include "pxafb.h"
#include "ffit.h"
#include "console/fbcon.h"
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <mach/pxa2xx-regs.h>

static int nocursor = 0;
module_param(nocursor, int, 0644);
MODULE_PARM_DESC(nocursor, "cursor enable/disable");

unsigned long fb0_offs = 0 ;
static unsigned long const fbStart = SM501_FBSTART ;
static unsigned long const fbMax = SM501_FBMAX ;    //
static unsigned long const mmioStart = SM501_MMIOSTART ;
static unsigned long const mmioLength = 0x00200000 ;
#define BIGGEST_DISPLAYMEM (1280*1024*2)
static unsigned char      *cursorMem ;
#define CURSORMEM_SIZE ((64*64*2)/8)

list_header_t *memPool_ = 0 ;
static DECLARE_MUTEX(memSem_);

// #define SM501_DEBUG

#ifdef SM501_DEBUG
#define DEBUGMSG( __fmt, ... ) printk( __fmt, ## __VA_ARGS__ )
#else
#define DEBUGMSG( __fmt, ... ) 
#endif

void *sm501_alloc( unsigned size )
{
   void *rval = 0 ;
   
	if( down_interruptible(&memSem_) ){
      printk( KERN_ERR "SM501_ALLOC: interrupted\n" );
		return 0 ;
   }

   if( 0 == memPool_ ){
      DEBUGMSG( KERN_ERR "SM501MemInit: %u bytes\n", SM501_FBMAX );
      memPool_ = init_memory_pool( SM501_FBMAX, fbVirtual );
      if( 0 == memPool_ ){
         printk( KERN_ERR "SM501_ALLOC: bad alloc: %p/%p\n", fbVirtual, memPool_ );
         return 0 ;
      }
      DEBUGMSG( KERN_ERR "SM501_ALLOC init: %p/%p\n", fbVirtual, memPool_ );
   }

   rval = rtl_malloc( memPool_, size );

DEBUGMSG( KERN_ERR "alloc: %u/%p\n", size, rval );

   up(&memSem_);

   return rval ;
}

void sm501_free( void *ptr )
{
	if( down_interruptible(&memSem_) )
		return ;

   rtl_free(memPool_,ptr);

   up(&memSem_);
}

static unsigned const dispctrlReg    = 0x00080000 ;
static unsigned const fbAddrReg      = 0x0008000C ;
static unsigned const windowWidthReg = 0x00080010 ;  //  02800280
static unsigned const fbWidthPixReg  = 0x00080014 ;  //  01400000
static unsigned const fbHeightReg    = 0x00080018 ;  //  00f00000
static unsigned const bottomRightReg = 0x00080020 ;  //  00ef013f
static unsigned const hTotalReg      = 0x00080024 ;  //  015F0140
static unsigned const hSyncReg       = 0x00080028 ;  //  0008014f
static unsigned const vTotalReg      = 0x0008002c ;  //  010700F0
static unsigned const vSyncReg       = 0x00080030 ;  //  000200FE
static unsigned const crtctrlReg     = 0x00080200 ;
static unsigned const crtAddrReg     = 0x00080204 ;
static unsigned const crtWindowWidthReg = 0x00080208 ;  //  02800280
static unsigned const crtFbVTotReg   = 0x00080214 ;

#define STUFFREG( addr, value ) *( (unsigned long volatile *)((addr)+mmioVirtual) ) = (value)
#define READREG( addr ) ( *( (unsigned long volatile *)((addr)+mmioVirtual) ) )


unsigned long vsyncCount_ = 0 ;

DECLARE_WAIT_QUEUE_HEAD(sync_wait);
DECLARE_WAIT_QUEUE_HEAD(draw_wait);

static void fb_vsync(int slotnum, void * hdata)
{
   STUFF_SM501_REG( SMIR_INT_CLEAR_REG, (1<<SMI_RAW_INTERRUPT_PVS)|(1<<SMI_RAW_INTERRUPT_CVS));

   vsyncCount_++ ;
   wake_up_interruptible(&sync_wait);
}

static void drawing_engine_int(int slotnum, void * hdata)
{
   STUFF_SM501_REG( SMIR_2D_STATUS, 0 ); //   SMIR_2D_STATUS_2D_COMPLETE|SMIR_2D_STATUS_CSC_COMPLETE );
//   STUFF_SM501_REG( SMIR_INT_CLEAR_REG, SMI_INTERRUPT_2D);
   wake_up_interruptible(&draw_wait);
}

static DEFINE_SPINLOCK(cmd_lock);
DECLARE_WAIT_QUEUE_HEAD(cmd_wait);
typedef struct {
   unsigned long cmdAddr_ ;
   void (*callback_)( unsigned long param );
   unsigned long cbParam_ ;
} cmdListEntry_t ;

#define NUMCMDLISTENTRIES 128
#define CMDLISTMASK    (NUMCMDLISTENTRIES-1)

static cmdListEntry_t cmdListEntries_[NUMCMDLISTENTRIES] = { {0} };
static int   volatile cmdListAdd_ = 0 ;
static int   volatile cmdListTake_ = 0 ;
static DECLARE_MUTEX(cmdListWait_);

static void command_list_int(int slotnum, void * hdata)
{
   int entryIdx ;
   cmdListEntry_t const *entry ;

   if( cmdListAdd_ != cmdListTake_ ){
      entryIdx = cmdListTake_ & CMDLISTMASK ;
      cmdListTake_ = ( cmdListTake_ + 1 ) & CMDLISTMASK ;
   
      entry = cmdListEntries_+entryIdx ;
      if( entry->callback_ ){
         DEBUGMSG( KERN_ERR "cmdlist callback %p/%lx\n", entry->callback_, entry->cbParam_ );

         entry->callback_(entry->cbParam_);
      }
      
      DEBUGMSG( KERN_ERR "cmdlist wake\n" );
      wake_up_interruptible(&cmd_wait); // wake processes waiting for space

      if( cmdListAdd_ != cmdListTake_ )
      {
         unsigned long const addr = cmdListEntries_[cmdListTake_].cmdAddr_ ;
         STUFF_SM501_REG( SMICMD_ADDRESS, addr | SMICMD_START );
      } // start next one
   }
   else
      printk( KERN_ERR "spurious command_list_int\n" );

   STUFF_SM501_REG( SMIR_INT_CLEAR_REG, 1<<SMI_RAW_INTERRUPT_CMD);
}

static int 
sm501_open(struct fb_info *info, int user)
{
   return 0 ;
}

static int 
sm501_release(struct fb_info *info, int user)
{
   unsigned long addr = READREG(SMICMD_ADDRESS);
   STUFF_SM501_REG( SMICMD_ADDRESS, addr & ~SMICMD_START );
   if( addr & SMICMD_START ){
      printk( KERN_ERR "%s: aborted command-list 0x%08lx\n", __func__, addr );
   }
   cmdListTake_ = cmdListAdd_ ;

   return 0 ;
}

void clearCmdList( void )
{
   unsigned long addr = READREG(SMICMD_ADDRESS);
   STUFF_SM501_REG( SMICMD_ADDRESS, addr & ~SMICMD_START );
   if( addr & SMICMD_START ){
      printk( KERN_ERR "%s: aborted command-list 0x%08lx\n", __func__, addr );
   }
   cmdListTake_ = cmdListAdd_ ;
}

static void sync_cmd_callback( unsigned long cbparam )
{
   up(&cmdListWait_);
}

//
// asynchronous version
//
int executeCommand( 
   ulong  addr, 
   void (*callback)(ulong param),
   ulong  cbParam
)
{
   if( 0 == (addr & 15) )
   {
      if( addr < SM501_FBMAX )
      {
         unsigned long flags ;
         int count ;
         int rval ;

         spin_lock_irqsave(&cmd_lock, flags);

         do {
            count = (cmdListAdd_ - cmdListTake_) & CMDLISTMASK ;
            if( CMDLISTMASK == count )
            {
               // wait for space
               spin_unlock_irqrestore(&cmd_lock, flags);

               rval = wait_event_interruptible(cmd_wait, CMDLISTMASK != ((cmdListAdd_ - cmdListTake_) & CMDLISTMASK) );

               spin_lock_irqsave(&cmd_lock, flags);
            }
            else {
               cmdListEntry_t *const entry= &cmdListEntries_[cmdListAdd_];
               addr += fb0_offs ;
               entry->cmdAddr_ = addr ;
               entry->callback_ = callback  ;
               entry->cbParam_ = cbParam ;
               DEBUGMSG( KERN_ERR "callback added: %u\n", cmdListAdd_ );
   
               cmdListAdd_ = ( cmdListAdd_ + 1 ) & CMDLISTMASK ;

               rval = 0 ;
               break ;
            }
         } while( !signal_pending(current) );

         if( ( 0 == rval ) && ( 0 == count ) ){
            // kick start
            DEBUGMSG( KERN_ERR "kick start\n" );
            STUFF_SM501_REG( SMICMD_ADDRESS, addr | SMICMD_START );
         }
         
         spin_unlock_irqrestore(&cmd_lock, flags);
         
         return rval ;
      }
   }

   return -EFAULT ;
}

//
// synchronous version
//
static int executeCmdList( unsigned long addr )
{
   if( 0 == (addr & 15) )
   {
      if( addr < 0x700000 )
      {
         int rval = executeCommand(addr,sync_cmd_callback, 0 );
         if( 0 == rval ){
            down(&cmdListWait_);
            return rval ;
         }
      }
   }

   return -EFAULT ;
}

static int
sm501_ioctl( struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
           case SM501_READREG:
           {
               ulong reg ;
               if( copy_from_user( &reg, (void __user *)arg, sizeof(reg) ) )
                  return -EFAULT;
               if( reg < mmioLength-sizeof(reg) )
               {
                  ulong value = READREG(reg);
                  if( copy_to_user( (void __user *)arg, &value, sizeof(value) ) )
                     return -EFAULT ;
                  else
                     return 0 ;
               }
               
               break ;
           }
           case SM501_WRITEREG:
           {
               struct reg_and_value rv ;
               if( copy_from_user( &rv, (void __user *)arg, sizeof(rv) ) )
                  return -EFAULT;
               if( rv.reg_ < mmioLength-sizeof(rv.value_) )
               {
                  STUFFREG( rv.reg_, rv.value_ );
                  return 0 ;
               }
               
               break ;
           }
           case SM501_WAITSYNC:
           {
              interruptible_sleep_on(&sync_wait);

              // ... intentional fall-through
           }
           case SM501_GET_SYNCCOUNT:
           {
              return copy_to_user( (void __user *)arg, &vsyncCount_, sizeof(vsyncCount_) );
           }

           case SM501_EXECCMDLIST :
           {
              return executeCmdList( arg );
           }
	}
	return -EINVAL;
}


static u32 colreg[17];

static int
sm501_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	/* only pseudo-palette (16 bpp) allowed */
	if(regno >= 16)	 /* maximum number of palette entries */
		return 1;

	if(info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Truecolor has hardware-independent 16-entry pseudo-palette */
	if(info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if(regno >= 16)
			return 1;

		red >>= (16 - info->var.red.length);
		green >>= (16 - info->var.green.length);
		blue >>= (16 - info->var.blue.length);

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) | (blue << info->var.
							 blue.offset);

		switch(info->var.bits_per_pixel) {
			case 16:
				colreg[regno] = v;
				break;
			default:
				return 1;
		}
		return 0;
	}
	return 0;
}

static int sm501_set_par(struct fb_info *info)
{
   DEBUGMSG( KERN_ERR "%s\n", __FUNCTION__ );
	return 0;
}

static int sm501_blank(int blank, struct fb_info *info)
{
   DEBUGMSG( KERN_ERR "%s\n", __FUNCTION__ );
	return 0;
}

static int sm501_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
   DEBUGMSG( KERN_ERR "%s\n", __FUNCTION__ );
	if (nocursor)
		return 0;
	else
		return -EINVAL;	/* just to force soft_cursor() call */
}

static struct fb_ops sm501_ops = {
   owner:         THIS_MODULE,
   fb_open:       sm501_open,
   fb_release:    sm501_release,
   fb_fillrect:   cfb_fillrect,
   fb_copyarea:   cfb_copyarea,
   fb_imageblit:  cfb_imageblit,
   fb_cursor:     sm501_cursor,
   fb_ioctl:      sm501_ioctl,
   fb_set_par:    sm501_set_par, 
   fb_setcolreg:  sm501_setcolreg,
   fb_blank:      sm501_blank,
};

static int sm501_read_proc( char *page, char **start, off_t off,
                            int count, int *eof, void *data )
{
   return snprintf( page, 512, 
                    "add %u\n"
                    "take %u\n",
                    cmdListAdd_,
                    cmdListTake_ );
}

static struct fb_info *register_fb( struct device *dev, unsigned w, unsigned h, unsigned intNum )
{
   int rval ;
   struct fb_info *fbi = (struct fb_info *)kzalloc(sizeof(struct fb_info),GFP_KERNEL);
   struct fb_var_screeninfo *var ;
   struct fb_fix_screeninfo *fix ;
   unsigned displayBytes = PAGE_ALIGN(w*h*2);
   unsigned allocBytes = (displayBytes+PAGE_SIZE); // allocate an extra page so we can align it

DEBUGMSG( KERN_ERR "new fb(%d): %ux%u\n", intNum, w, h );
   fbi->node = -1;
   fbi->flags = FBINFO_FLAG_DEFAULT;
   fbi->fbops = &sm501_ops;

   fbi->screen_size = displayBytes ;
   fbi->screen_base = sm501_alloc(allocBytes);
   fbi->screen_base = (char __iomem *)PAGE_ALIGN((unsigned long)fbi->screen_base);
   memset(fbi->screen_base,0,fbi->screen_size);

   DEBUGMSG( KERN_ERR "alloc screen %p/%lu\n", fbi->screen_base, fbi->screen_size );

   fix = &fbi->fix ;
   snprintf(fix->id, sizeof(fix->id)-1, "SM501-%d", intNum );
   fix->smem_start = fbStart+fbi->screen_base-fbVirtual ;
   if( 1 == intNum ){
      fb0_offs = (char *)fbi->screen_base-(char *)fbVirtual ;
      DEBUGMSG( KERN_ERR "fb0 offset 0x%lx\n", fb0_offs );
   }
DEBUGMSG( KERN_ERR "smem_start: %lx\n", fix->smem_start );
   fix->smem_len = SM501_FBMAX ;
   fix->type = FB_TYPE_PACKED_PIXELS;
   fix->type_aux = 0;
   fix->visual = FB_VISUAL_TRUECOLOR;
   fix->xpanstep = 0;
   fix->ypanstep = 0;
   fix->ywrapstep = 0;
   fix->line_length = w * 2 ;
   fix->mmio_start = mmioStart ;
   fix->mmio_len = mmioLength ;
   fix->accel = FB_ACCEL_NONE ;

   var = &fbi->var ;
   var->xres = w ;
   var->yres = h ;
   var->xres_virtual = w ;
   var->yres_virtual = h ;
   var->xoffset = 0;
   var->yoffset = 0;
   var->bits_per_pixel = 16;
   var->grayscale = 0;
   var->red.offset = 11;
   var->red.length = 5;
   var->green.offset = 5;
   var->green.length = 6;
   var->blue.offset = 0;
   var->blue.length = 5;
   var->transp.offset = 0;
   var->transp.length = 0;
   var->nonstd = 0;
   var->activate = FB_ACTIVATE_NOW;
   var->height = h ;
   var->width = w ;
   var->pixclock = 0;
   var->left_margin = 0;
   var->right_margin = 0;
   var->upper_margin = 0;
   var->lower_margin = 0;
   var->hsync_len = 0;
   var->vsync_len = 0;
   var->sync = 0;
   var->vmode = FB_VMODE_NONINTERLACED;

   fbi->pseudo_palette = colreg;
   fbi->device = dev ;   

   DEBUGMSG( KERN_ERR "register fbDev: %ux%u\n", w, h );
   rval = register_framebuffer(fbi);
   if( 0 <= rval ){
      DEBUGMSG( KERN_ERR "grab int slot %x\n", intNum );
      if( 0 != SM501_grab_int_slot(mmioVirtual,intNum,fb_vsync,fbi) )
         printk( KERN_ERR "%s: Error grabbing int slot %d\n", __FUNCTION__, intNum );
   }
   DEBUGMSG( KERN_ERR "return fb %p\n", fbi );
   return fbi ;
}

static void enableCRT(struct device *dev,unsigned long crtCtrl)
{
   struct fb_info *fbi ;
   unsigned long bytesPerLine ;
   unsigned w ;
   unsigned h ;
   
   w = ( READREG( crtWindowWidthReg ) >> 16 );
   h = ( READREG( crtFbVTotReg ) & 0x7ff ) + 1 ;
   
   bytesPerLine = w*2 ;
   
   crtCtrl &= ~CRTCTRL_ENABLE ;
   STUFFREG( crtctrlReg, crtCtrl );
   
   crtCtrl &= ~CRTCTRL_MODEMASK ;
   crtCtrl |= CRTCTRL_USECRTDATA;
   crtCtrl |= CRTCTRL_MODE16BIT ;
   
   STUFFREG( crtctrlReg, crtCtrl );
   
   STUFFREG( crtWindowWidthReg, ( bytesPerLine << 16 ) | bytesPerLine );
   
   crtCtrl |= CRTCTRL_ENABLE ;
   STUFFREG( crtctrlReg, crtCtrl );
   
   fbi = register_fb( dev, w, h, SMI_RAW_INTERRUPT_CVS );
   if( fbi ){
      STUFFREG(crtAddrReg, fbi->screen_base-fbVirtual);
      DEBUGMSG( KERN_ERR "CRT at 0x%x\n", fbi->screen_base-fbVirtual );
   } else {
      printk( KERN_ERR "Error initializing CRT\n" );
   }
}

int __init
sm501_probe(struct device *dev)
{
   unsigned long dispCtrl, crtCtrl ;
   unsigned long lcdFB, crtFB ;
   unsigned w, h ;
   struct proc_dir_entry *pde ;
   
   DEBUGMSG( KERN_ERR "%s:\n", __FUNCTION__ );
   get_mmioVirtual();
   get_fbVirtual();

   if( 0 == mmioVirtual )
      return -EINVAL;

   dispCtrl = READREG( dispctrlReg ); 
   crtCtrl = READREG( crtctrlReg );

   /*
    * four cases to handle here:
    *    panel only or panel and crt using same fb
    *       set to 16-bits, one device
    *    crt only
    *       set to 16-bits, one device
    *    panel + crt
    *       set each to 16-bits, one device
    *    no display enabled
    *       do nothing
    *
    * What's more, if they use separate frame buffers, we want to 
    * enable the CRT and LCD panel in the same order that they were
    * defined by the boot loader (the first will have a lower 
    * frame-buffer address)
    */
   crtFB = READREG(crtAddrReg);
   lcdFB = READREG(fbAddrReg);
   if( ( 0 != ( CRTCTRL_ENABLE & crtCtrl ) )
       &&
       ( 0 != ( CRTCTRL_USECRTDATA & crtCtrl ) ) 
       &&
       ( crtFB < lcdFB ) )
   {
      enableCRT(dev,crtCtrl);
   }

   if( 0 != ( DISPCTRL_ENABLE & dispCtrl ) )
   {
      struct fb_info *fbi ;
      unsigned mode = ( DISPCTRL_MODEMASK & dispCtrl );
      unsigned long bytesPerLine ;

      w = ( READREG( fbWidthPixReg ) >> 16 );
      h = ( READREG( fbHeightReg ) >> 16 );
      bytesPerLine = w*2 ;

      if( DISPCTRL_MODE16BIT != mode )
      {
         dispCtrl &= ~DISPCTRL_ENABLE ;
         STUFFREG( dispctrlReg, dispCtrl );

         dispCtrl &= ~DISPCTRL_MODEMASK ;
         dispCtrl |= DISPCTRL_MODE16BIT ;

         STUFFREG( dispctrlReg, dispCtrl );
         dispCtrl |= DISPCTRL_ENABLE ;
         
         STUFFREG( windowWidthReg, ( bytesPerLine << 16 ) | bytesPerLine );
         STUFFREG( dispctrlReg, dispCtrl );
      }

      fbi = register_fb( dev, w, h, SMI_RAW_INTERRUPT_PVS );
      if( fbi ){
         DEBUGMSG( KERN_ERR "LCD at 0x%x\n", fbi->screen_base-fbVirtual );
         STUFFREG(fbAddrReg, fbi->screen_base-fbVirtual);
      } else {
         printk( KERN_ERR "Error initializing LCD\n" );
      }
   }

   if( 0 != ( CRTCTRL_ENABLE & crtCtrl ) )
   {
      if( ( 0 != ( CRTCTRL_USECRTDATA & crtCtrl ) ) 
          &&
          ( crtFB >= lcdFB ) ){
         enableCRT(dev,crtCtrl);
         if( 0 == ( DISPCTRL_ENABLE & dispCtrl ) ){
            printk( KERN_ERR "No YUV support available until CRT uses FB description\n" );
         }
      }
      else {
         printk( KERN_ERR "LCD and CRT sharing data\n" );
      }
   }

   if( 0 == nocursor ){
      unsigned cursorOffs ;
      cursorMem  = (unsigned char *)sm501_alloc(CURSORMEM_SIZE);
      memset(cursorMem,0,CURSORMEM_SIZE); // transparent for now
      cursorOffs = cursorMem-(unsigned char *)fbVirtual ;
      STUFF_SM501_REG(SMIPCURSOR_ADDR,cursorOffs);
      STUFF_SM501_REG(SMIPCURSOR_LOC,0);
      STUFF_SM501_REG(SMIPCURSOR_COLOR12,0xFFFF0000);	// 0 == black, 1 == white
      STUFF_SM501_REG(SMIPCURSOR_COLOR3,0);
      STUFF_SM501_REG(SMICCURSOR_ADDR,cursorOffs);
      STUFF_SM501_REG(SMICCURSOR_LOC,0);
      STUFF_SM501_REG(SMICCURSOR_COLOR12,0xFFFF0000);
      STUFF_SM501_REG(SMICCURSOR_COLOR3,0);
      printk(KERN_ERR "SM501 cursor at 0x%x\n", cursorOffs );
   }

   if( 0 == SM501_grab_int_slot(mmioVirtual,SMI_INTERRUPT_2D,drawing_engine_int,0) )
      DEBUGMSG( KERN_ERR "have drawing engine int\n" );
   else
      printk( KERN_ERR "Error grabbing 2d engine int\n" );

   if( 0 == SM501_grab_int_slot(mmioVirtual,SMI_INTERRUPT_CMD,command_list_int,0) )
      DEBUGMSG( KERN_ERR "have cmdlist int\n" );
   else
      printk( KERN_ERR "Error grabbing cmdlist int\n" );

   pde = create_proc_entry("driver/sm501", 0, 0);
   if( pde ) {
      pde->read_proc  = sm501_read_proc ;
   }
   else
      printk( KERN_ERR "Error creating SM501 proc entry\n" );
   
   printk( KERN_ERR "Initialized sm501 module\n" );

   return 0;
}

static struct device_driver sm501_driver = {
	.name = SM501FB_CLASS,
	.probe = sm501_probe,
	.bus = &platform_bus_type
};


int __devinit sm501_init(void)
{
	printk( KERN_ERR "SM-501 init.\n");
	return driver_register(&sm501_driver);
}

module_init(sm501_init);

MODULE_AUTHOR("Boundary Devices");
MODULE_DESCRIPTION("Silicon Motion SM-501");
MODULE_SUPPORTED_DEVICE("fb");

