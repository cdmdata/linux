#ifndef _VIDEO_SM501YUV_H
#define _VIDEO_SM501YUV_H
/*
 * sm501mem.h
 *
 * Declares ioctl constants and data structures
 * for use in allocating memory from the SM-501
 * graphics controller. 
 * 
 * Note that this module (and corresponding device "/dev/sm501mem") 
 * are separated from the SM501 frame buffer driver to simplify 
 * freeing of data upon release (close).
 *
 */

#ifndef BASE_MAGIC
#define BASE_MAGIC 0xBD
#endif

/* 
 * Input: bytes to allocate
 * Output: offset into frame-buffer memory (or NULL)
 */
#define SM501_ALLOC  _IOWR(BASE_MAGIC, 0x01, unsigned long)

/* 
 * Input: offset into frame-buffer memory
 */
#define SM501_FREE   _IOR( BASE_MAGIC, 0x02, unsigned long)

/* 
 * Input: nothing
 * Output: offset of fb0 in SM-501 memory
 */
#define SM501_BASEADDR  _IOWR(BASE_MAGIC, 0x03, unsigned long)


#endif 
