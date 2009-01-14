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
