#ifndef __PXAFB_YUV_H__
#define __PXAFB_YUV_H__
/*
 * linux/drivers/video/pxafb_yuv.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */
struct fb_info_yuv;
struct fb_plane {
	dma_addr_t	map_dma;
	u_char *	map_virtual;
	u_int		map_size;
	u_int		plane_size;
	struct fb_info_yuv* parent;
};

#define PLANE_Y		0
#define PLANE_U		1
#define PLANE_V		2

struct fb_info_yuv {
	struct device *dev;
	struct class *pxafb_plane_class;
	int plane_major;
	atomic_t usage;
	dma_addr_t	desc_dma;
	struct pxafb_dma_descriptor *desc_virtual;
	u_int		desc_size;
	struct fb_plane	fb_planes[3];
};
#endif
