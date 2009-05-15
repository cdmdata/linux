#ifndef __PXA_READ_REGS_H__
#define __PXA_READ_REGS_H__

/*
 * arch/arm/mach-pxa/read_regs.h
 *
 * Author: Valliappan (Valli) Annamalai, Boundary Devices
 *
 * This header file defines the function to read values
 * stored in PXA registers. This function would be called
 * from the board specific init method.
 *
 */

#include <linux/platform_device.h>
#include <mach/pxa2xx-regs.h>
#include <mach/pxafb.h>

void pxa_mode_from_registers(struct platform_device *dev);

#endif
