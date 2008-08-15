#ifndef __SM501_H_INCLUDED__
#define __SM501_H_INCLUDED__

#include <linux/types.h>
#include <linux/fb.h>

#include "ffit.h"

int
sm501_get_fix(struct fb_fix_screeninfo *fix, int con, struct fb_info *info);

int
sm501_get_var(struct fb_var_screeninfo *var, int con, struct fb_info *info);

extern void *sm501_alloc( unsigned size );
extern void sm501_free( void *ptr );

extern list_header_t *memPool_ ;

extern int executeCommand( 
   ulong       addr, 
   void       (*callback)(ulong param),
   ulong       cbParam
);

#endif
