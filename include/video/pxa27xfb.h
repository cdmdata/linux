#ifndef __PXA27XFB_H__
#define __PXA27XFB_H__
/*
 * include/video/pxa27xfb.h -- pxa27x Framebuffer overlay Driver
 *
 * Copyright (c) 2008  Troy Kisky <troy.kisky@BoundaryDevices.com>
 *
 *
 * Declares ioctl constants and data structures
 * for use in communicating video parameters 
 * between userland and the pxa27xyuv driver
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License.  See the file COPYING in the main directory of this
 * archive for more details.
 */
#define PXA27X_Y_CLASS "fb_y"
#define PXA27X_U_CLASS "fb_u"
#define PXA27X_V_CLASS "fb_v"

/*
 * hardware cursor
 */
#define PXA27X_CURSOR "fb_cursor"

struct cursorfb_info {
	unsigned int x_loc;
	unsigned int y_loc;
	unsigned int mode;
};

#define DEFAULT_CURSOR_MODE	0

struct cursorfb_mode {
	unsigned int xres;
	unsigned int yres;
	unsigned int bpp;
	/* Total number of colors supportes in this mode */
	unsigned int color_count;
	/* Holds the integer value for transparency bits */
	int transparency;
};

struct color16_info {
	unsigned short color;
	unsigned char color_idx;
};

struct color24_info {
	unsigned int color;
	unsigned char color_idx;
};

#define FOR_RGB 0
#define FOR_PACKED_YUV444 1
#define FOR_PLANAR_YUV444 2
#define FOR_PLANAR_YUV422 3
#define FOR_PLANAR_YUV420 4

struct pxa27x_overlay_t {
	unsigned for_type;
	unsigned offset_x;	/* relative to the base plane */
	unsigned offset_y;
	unsigned width;
	unsigned height;
};

/*
 * pxa27x_yuv ioctls
 */

#define BASE_MAGIC 0xBD

#define PXA27X_YUV_SET_DIMENSIONS  _IOWR(BASE_MAGIC, 0x03, struct pxa27x_overlay_t)
#define PXA27X_CURSOR_SETLOC  _IOW(BASE_MAGIC, 0x04, struct cursorfb_info)
#define PXA27X_CURSOR_GETLOC  _IOR(BASE_MAGIC, 0x05, struct cursorfb_info)
#define PXA27X_CURSOR_ACTIVATE  _IO(BASE_MAGIC, 0x06)
#define PXA27X_CURSOR_DEACTIVATE  _IO(BASE_MAGIC, 0x07)
#define PXA27X_16_BIT_COLOR  _IOW(BASE_MAGIC, 0x08, struct color16_info)
#define PXA27X_24_BIT_COLOR  _IOW(BASE_MAGIC, 0x09, struct color24_info)
#define PXA27X_DONT_REMOVE_CURSOR_ATEXIT  _IO(BASE_MAGIC, 0x0A)
#endif
