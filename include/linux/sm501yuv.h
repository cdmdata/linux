#ifndef _VIDEO_SM501YUV_H
#define _VIDEO_SM501YUV_H
/*
 * sm501yuv.h
 *
 * Declares ioctl constants and data structures
 * for use in communicating video parameters 
 * between userland and the sm501yuv driver
 *
 */
 
struct sm501yuvPlane_t {
   unsigned      xLeft_ ;
   unsigned      yTop_ ;
   unsigned      inWidth_ ;
   unsigned      inHeight_ ;
   unsigned      outWidth_ ;
   unsigned      outHeight_ ;
   unsigned      planeOffset_ ; // output: offset into SM-501 RAM 
};
 
#endif 
