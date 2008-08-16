/*
 *
 *  Author:     Troy Kisky
 *  Created:    Apr 28, 2004
 *  Copyright:  Boundary Devices
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <asm/BigMacro.h>

	.global gpioGetPxaFiq
//void * GetPxa_FIQ_WordFifoDMA(int* length,int* lockedLength)
gpioGetPxaFiq:
	mov	r2,#GpioFiq_rtn_end-GpioFiq_rtn
	mov	r3,#GpioFiq_rtn_lockend-GpioFiq_rtn
	str	r2,[r0]
	str	r3,[r1]
	adr	r0,GpioFiq_rtn
	mov	pc,lr
	nop


#define IC_BASE  0xf8d00000	//virtual addresses
#define __ICIP	0x00
#define __ICMR	0x04
#define __ICLR	0x08

#define save_size 32
#define callback   (0+save_size)
#define private    (4+save_size)
#define activeMask (8+save_size)	//3 fields, gp0, gp1, gp2-80 (bits 2-7 same value)
#define happenedMask (12+save_size)
//ip,r0-r3 are never saved on "c" procedure call
//(sl==r10),(fp==r11),(ip==r12),(sp==r13),(lr==r14),(pc==r15)
//constant
#define rGEDR0 r4
#define rGEDR1 r5
#define rGEDR2 r6
#define rGpioShft2  r8

#define rDevArray r9
#define rDCSR r10
#define rGPIO_BASE fp

#define DCSR_NODESC (1<<30)
#define DCSR_IDLE_INT DCSR_NODESC|(1<<29)

#define GEDR0 0x48
#define GEDR1 0x4c
#define GEDR2 0x50
GpioFiq_rtn:
GpioFiq_rtn_lockend:
	sub	lr,lr,#4	//debugger has this instruction cached for 1st instruction of fiq rtn
				//and CANNOT be changed without corresponding debugger change
	stmdb	sp!,{r0-r6,lr}
	ldrsb	r3, [sp, #activeMask]
	ldr	rGEDR0, [rGPIO_BASE, #GEDR0]
	tst	r3,#1<<2
	and	rGEDR0,rGEDR0,r3
	ldrne	rGEDR1, [rGPIO_BASE, #GEDR1]
	moveq	rGEDR1,#0
	ldrne	rGEDR2, [rGPIO_BASE, #GEDR2]
	moveq	rGEDR2,#0
	str	rGEDR0, [rGPIO_BASE, #GEDR0]
	strne	rGEDR1, [rGPIO_BASE, #GEDR1]
	strne	rGEDR2, [rGPIO_BASE, #GEDR2]

	ldr	r0,[sp,#happenedMask]
	ldrne	r1,[sp,#happenedMask+4]
	ldrne	r2,[sp,#happenedMask+8]
	orr	r0,r0,rGEDR0
	orrne	r1,r1,rGEDR1
	orrne	r2,r2,rGEDR2
	str	r0,[sp,#happenedMask]
	strne	r1,[sp,#happenedMask+4]
	strne	r2,[sp,#happenedMask+8]

	mov     rGpioShft2,#0
	b	2f
1:
	mov	r1,#1
	bic	rGEDR0,rGEDR0,r1,lsl r0
	add	r0,rGpioShft2,r0,lsl #2
	ldr	r1,[rDevArray,r0]
	ldr	r3,[sp,#callback]
	movs	r1,r1
	BigMov	r0,-1
	sub	r2,sp,#(16*4)-save_size		//align the pc in the structure
	blxne	r3			//call if device valid
2:	clz	r0,rGEDR0
	rsbs	r0,r0,#31
	bpl	1b
	orrs	r0,rGEDR1,rGEDR2
	mov	rGEDR0,rGEDR1
	mov	rGEDR1,rGEDR2
	mov	rGEDR2,#0
	addne	rGpioShft2,rGpioShft2,#32<<2
	bne	2b

	mov	r0,#DCSR_IDLE_INT
	str	r0,[rDCSR,#0]		//generate a dma irq
	ldmia	sp!,{r0-r6,pc}^
GpioFiq_rtn_end:
