#ifndef _VIDEO_SM501ALPHA_H
#define _VIDEO_SM501ALPHA_H
/*
 * sm501alpha.h
 *
 * Declares ioctl constants and data structures
 * for use in communicating alpha parameters 
 * between userland and the sm501alpha driver.
 *
 * Usage involves opening a channel to the alpha
 * driver (via /dev/sm501alpha) and using the 
 * SM501ALPHA_SETPLANE ioctl to create the plane
 *
 * The SM501ALPHA_SETPLANE ioctl returns
 *
 */
 

/*
 * This structure allows an application to define
 * a 'subset' alpha layer to conserve RAM (and bandwidth)
 * through the first four parameters.
 *
 * The fifth parameter is used by the driver to return
 * the offset in SM501 RAM of the plane.
 *
 * The palette parameter is used to set the color palette
 * for RGBA44 mode.
 *
 * Note that only RGBA44 and RGBA4444 are currently supported.
 */
#define SM501_ALPHA_RGB565    1
#define SM501_ALPHA_RGBA44    2
#define SM501_ALPHA_RGBA4444  3

struct sm501_alphaPlane_t {
   unsigned       xLeft_ ;         
   unsigned       yTop_ ;
   unsigned       width_ ;
   unsigned       height_ ;
   unsigned       mode_ ;
   unsigned       planeOffset_ ;
   unsigned long  palette_[8];
};
 
#endif 
