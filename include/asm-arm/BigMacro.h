/*
 *  linux/include/asm-arm/BigMacro.h
 *
 *  Author:     Troy Kisky
 *  Created:    Jun 30, 2002
 *  Copyright:  Boundary Devices
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
	.nolist

//find set bit pair >= curbit
//out: NBit
.macro NextSetBitUp	val,curBit
	.set	NBit,(\curBit)
	.set	nV1,(\val)
	.if ((nV1) & (0x03<<(NBit)))
	.else
		.if ((NBit)-30)
			NextSetBitUp nV1,((NBit)+2)
		.endif
	.endif
.endm
//find set bit pair <= curbit
//out: NBit
.macro NextSetBitDown	val,curBit
	.set	NBit,(\curBit)
	.set	nV1,(\val)
	.if ((nV1)&(0xc0<<(NBit)))
	.else
		.if (NBit)
			NextSetBitDown nV1,((NBit)-2)
		.endif
	.endif
.endm

//OUT: NMask
.macro NextSetMask	val
	NextSetBitDown	\val,24
	.if (NBit>=20)
		NextSetBitUp	\val,NBit
		.set NMask,(0xff<<((NBit)-16))
		.set NMask,(((NMask)>>16)+(((NMask)<<16)&0xffff0000))
	.else
		.set NMask,(0xff<<(NBit))
	.endif

.endm

.macro Big2CC inst,dest,val
	.set nVal,(\val)
	.if (nVal)
		NextSetMask nVal
		\inst	\dest,\dest,#(nVal)&(NMask)
		Big2CC \inst,\dest,(nVal)&~(NMask)
	.endif
.endm

.macro BigAnd2CC cc,dest,val
	.set nVal,(\val)
	.if (~nVal)
		NextSetMask nVal
		.if (((nVal)&~(NMask))=0)
			and\cc	\dest,\dest,#(nVal)&(NMask)
		.else
			Big2CC bic\cc,\dest,~nVal
		.endif
	.endif
.endm

///////////////////////////////////////////////////////
.macro	BigMovCC  cc,dest, val
	.set nVal,(\val)
	NextSetMask ~nVal
	.if (((~(nVal))&~(NMask)) > 0x255)
		NextSetMask nVal
		mov\cc	\dest,#(nVal)&(NMask)
		Big2CC orr\cc,\dest,(nVal)&~(NMask)
	.else
		mvn\cc	\dest,#(~(nVal))&(NMask)	//complement of complement is original
		Big2CC bic\cc,\dest,(~(nVal))&~(NMask)
	.endif
.endm

.macro	BigAddCC cc,dest,src,val
	.set nVal,(\val)
	.if (nVal)
		NextSetMask -nVal
		.if (((-(nVal))&~(NMask)) > 0x255)
			NextSetMask nVal
			add\cc	\dest,\src,#(nVal)&(NMask)
			Big2CC add\cc,\dest,(nVal)&~(NMask)
		.else
			sub\cc	\dest,\src,#(-(nVal))&(NMask)
			Big2CC sub\cc,\dest,(-(nVal))&~(NMask)
		.endif
	.else
		mov\cc	\dest,\src
	.endif
.endm

.macro	BigSubCC cc,dest,src,val
	.set nVal,(\val)
	BigAddCC \cc,\dest,\src,-nVal
.endm

.macro BigCC inst,cc,dest,src,val
	.set nVal,(\val)
	.if (nVal)
		NextSetMask nVal
		\inst\cc	\dest,\src,#(nVal)&(NMask)
		Big2CC \inst\cc,\dest,(nVal)&~(NMask)
	.else
		mov\cc	\dest,\src
	.endif
.endm


.macro BigAndCC cc,dest,src,val
	.set nVal,(\val)
	.if (~nVal)
		NextSetMask nVal
		.if (((nVal)&~(NMask))=0)
			and\cc	\dest,\src,#(nVal)&(NMask)
		.else
			BigCC bic,\cc,\dest,\src,~nVal
		.endif
	.else
		mov\cc	\dest,\src
	.endif
.endm

/////////////////////////////////////
//dest, value
#define BigAdd2   Big2CC add,
#define BigAdd2Eq Big2CC addeq,
#define BigAdd2Ne Big2CC addne,

#define BigSub2   Big2CC sub,
#define BigSub2Eq Big2CC subeq,
#define BigSub2Ne Big2CC subne,

#define BigOrr2   Big2CC orr,
#define BigOrr2Eq Big2CC orreq,
#define BigOrr2Ne Big2CC orrne,

#define BigEor2   Big2CC eor,
#define BigEor2Eq Big2CC eoreq,
#define BigEor2Ne Big2CC eorne,
#define BigEor2Cs Big2CC eorcs,
#define BigEor2Cc Big2CC eorcc,

#define BigBic2   Big2CC bic,
#define BigBic2Eq Big2CC biceq,
#define BigBic2Ne Big2CC bicne,

#define BigAnd2   BigAnd2CC al,
#define BigAnd2Eq BigAnd2CC eq,
#define BigAnd2Ne BigAnd2CC ne,
/////////////////////////////////////

#define BigMov    BigMovCC  al,
#define BigMovEq  BigMovCC  eq,
#define BigMovNe  BigMovCC  ne,
// dest,src,value
#define BigAdd    BigAddCC  al,
#define BigAddEq  BigAddCC  eq,
#define BigAddNe  BigAddCC  ne,

#define BigSub    BigSubCC  al,
#define BigSubEq  BigSubCC  eq,
#define BigSubNe  BigSubCC  ne,

#define BigOrr    BigCC  orr,al,
#define BigOrrEq  BigCC  orr,eq,
#define BigOrrNe  BigCC  orr,ne,

#define BigEor    BigCC  eor,al,
#define BigEorEq  BigCC  eor,eq,
#define BigEorNe  BigCC  eor,ne,
#define BigEorCs  BigCC  eor,cs,
#define BigEorCc  BigCC  eor,cc,

#define BigBic    BigCC  bic,al,
#define BigBicEq  BigCC  bic,eq,
#define BigBicNe  BigCC  bic,ne,

#define BigAnd    BigAndCC  al,
#define BigAndEq  BigAndCC  eq,
#define BigAndNe  BigAndCC  ne,

// *******************************************************************************************
	.list
