#ifndef __SM501_INT_H_INCLUDED__
#define __SM501_INT_H_INCLUDED__

typedef unsigned char* port_t;

typedef void (*handler501_t) (int slotnum, void * hdata);

struct handlerData
{
   handler501_t handler;
   void * hdata;
};

#define SM501_MAX_HANDLERS_PER_INT 2

struct SM501_IntData
{
   port_t cfgBase;
   struct handlerData hd[32][SM501_MAX_HANDLERS_PER_INT+1];
   unsigned intCount[32];
   unsigned hard_ints ;
};

extern struct SM501_IntData* intD ;
extern char *get_mmioVirtual(void); // call this to initialize
extern char *get_fbVirtual(void);   // call this to map
extern char *mmioVirtual ;
extern char *fbVirtual ;
extern unsigned long vsyncCount_ ;

#ifdef __KERNEL__
#include <linux/interrupt.h>
/* returns 0 if successful */
extern int SM501_grab_int_slot(port_t cfgBase, unsigned int slotnum,handler501_t h,void* data);
extern irqreturn_t sm501_interrupt (int irq, void * hdata);
#endif

extern void clearCmdList(void);

#define SMIGRAPH_WIDTHREG     0x00080014   //  01400000
#define SMIGRAPH_HEIGHTREG    0x00080018   //  00f00000
#define SMIGRAPH_CRT_FBADDR   0x00080204
#define SMIGRAPH_CRTWIDTH     0x00080208   //  02800280
#define SMIGRAPH_CRTHTOTAL    0x0008020C
#define SMIGRAPH_CRTHSYNC     0x00080210
#define SMIGRAPH_CRTVTOT      0x00080214
#define SMIGRAPH_CRTVSYNC      0x00080218

#define SMIGRAPH_DISPCTRL     0x00080000
#define SMIGRAPH_CRTCTRL      0x00080200

#define SMIDISPCTRL_FBADDR    0x0008000C
#define SMIDISPCTRL_WWIDTH    0x00080010
#define SMIDISPCTRL_FBWIDTH   0x00080014
#define SMIDISPCTRL_FBHEIGHT  0x00080018
#define SMIDISPCTRL_BRIGHT    0x00080020
#define SMIDISPCTRL_HTOTAL    0x00080024
#define SMIDISPCTRL_HSYNC     0x00080028
#define SMIDISPCTRL_VTOTAL    0x0008002c
#define SMIDISPCTRL_VSYNC     0x00080030

#define DISPCTRL_ENABLE (1<<2)
#define DISPCTRL_MODEMASK  (3)
#define DISPCTRL_MODE8BIT  (0)
#define DISPCTRL_MODE16BIT (1)

#define CRTCTRL_ENABLE (1<<2)
#define CRTCTRL_MODEMASK  (3)
#define CRTCTRL_MODE8BIT  (0)
#define CRTCTRL_MODE16BIT (1)
#define CRTCTRL_USECRTDATA (1<<9)
#define CRTCTRL_USECRTTIMING (1<<8)
#define CRTCTRL_USELCDDATA (0<<9)

#define SMIVIDEO_CTRL 0x80040
#define SMIVIDEO_CTRL_ENABLE_YUV 0x00010307
#define SMIVIDEO_CTRL_ENABLE_RGB 0x00010305

#define SMIR_GPIO_0_31_CONTROL      0x00008  //use SMI_REG
#define SMIR_GPIO_32_63_CONTROL     0x0000c
#define SMIR_INT_CLEAR_REG    0x00028
#define SMIR_INT_STATUS_REG         0x0002c
#define SMIR_INT_MASK_REG        0x00030
#define SMIR_CURRENT_GATE        0x00038     //read only
#define SMIR_PWRM0_GATE          0x00040
#define SMIR_PWRM1_GATE          0x00048
#define SMIR_POWER_MODE_CONTROL     0x00054
#define SMIR_MISCELLANEOUS_TIMING   0x00068
#define SMIR_2D_STATUS        0x100050

#define SMIR_2D_STATUS_2D_COMPLETE  1
#define SMIR_2D_STATUS_CSC_COMPLETE 2


#define SMIR_USB_INT_STATUS_REG     0x000c
#define SMIR_USB_INT_MASK_REG    0x0010   //use SMI_USBREG
#define HcControl             0x04
#define HcCommandStatus          0x08
#define HcRhDescriptorA          0x48
#define HcRhDescriptorB          0x4c
#define HcRhStatus               0x50
#define HcRhPort1             0x54

/*
 * Command list registers
 */
#define SMICMD_ADDRESS        0x00000018
#define SMICMD_CONDITION      0x0000001C
#define SMICMD_RETURNADDR     0x00000020

#define SMICMD_START          0x80000000

/*
 * Drawing engine registers
 */
#define SMIDRAW_2D_Source              0x100000 
#define SMIDRAW_2D_Destination         0x100004 
#define SMIDRAW_2D_Dimension           0x100008 
#define SMIDRAW_2D_Control             0x10000C 
#define SMIDRAW_2D_Pitch               0x100010 
#define SMIDRAW_2D_Foreground          0x100014 
#define SMIDRAW_2D_Background          0x100018 
#define SMIDRAW_2D_Stretch_Format      0x10001C 
#define SMIDRAW_2D_Color_Compare       0x100020 
#define SMIDRAW_2D_Color_Compare_Mask  0x100024 
#define SMIDRAW_2D_Mask                0x100028 
#define SMIDRAW_2D_Clip_TL             0x10002C 
#define SMIDRAW_2D_Clip_BR             0x100030 
#define SMIDRAW_2D_Mono_Pattern_Low    0x100034 
#define SMIDRAW_2D_Mono_Pattern_High   0x100038 
#define SMIDRAW_2D_Window_Width        0x10003C 
#define SMIDRAW_2D_Source_Base         0x100040 
#define SMIDRAW_2D_Destination_Base    0x100044 
#define SMIDRAW_2D_Alpha               0x100048 
#define SMIDRAW_2D_Wrap                0x10004C 
#define SMIDRAW_2D_Status              0x100050 

#define SMI_REG(cfgBase,reg)  (*(volatile unsigned long *)(cfgBase+reg))
#define SMI_USBREG(usbBase,reg)  (*(volatile unsigned long *)(usbBase+reg))
#define SMI_DUMMY(bufBase,reg)   (*(volatile unsigned long *)(bufBase+reg))

#define MHcCommandStatusHCRmask     0x00000001;

/////////////////////////////////////////////////////////////
#define SMI_RAW_INTERRUPT_CMD     0       // command interpreter
#define SMI_RAW_INTERRUPT_PVS     1       // panel vertical sync
#define SMI_RAW_INTERRUPT_US      2       // USB slave
#define SMI_RAW_INTERRUPT_CVS     3       // CRT vertical sync
#define SMI_RAW_INTERRUPT_ZV      4       // ZV port
#define SMI_RAW_INTERRUPT_UP      5       // USB slave plug-in

#define SMI_INTERRUPT_CMD     0       // command interpreter
#define SMI_INTERRUPT_PVS     1  // panel vertical sync
#define SMI_INTERRUPT_ZV      2
#define SMI_INTERRUPT_2D      3       // 2D drawing engine
#define SMI_INTERRUPT_USB     6
#define SMI_INTERRUPT_CVS    11 // CRT vertical sync
#define SMI_INTERRUPT_UART1  12
#define SMI_INTERRUPT_PWM    22
#define SMI_INTERRUPT_I2C    23

/*
 * Panel cursor registers
 */
#define SMIPCURSOR_ADDR               0x800F0
#define SMIPCURSOR_LOC                0x800F4
#define SMIPCURSOR_COLOR12            0x800F8
#define SMIPCURSOR_COLOR3             0x800FC

/*
 * CRT cursor registers
 */
#define SMICCURSOR_ADDR               0x80230
#define SMICCURSOR_LOC                0x80234
#define SMICCURSOR_COLOR12            0x80238
#define SMICCURSOR_COLOR3             0x8023C

/*
 * Alpha layer registers
 */
#define SMIALPHA_CONTROL              0x80100
#define SMIALPHA_FBADDR               0x80104
#define SMIALPHA_FBOFFS               0x80108
#define SMIALPHA_TL                   0x8010C
#define SMIALPHA_BR                   0x80110
#define SMIALPHA_CHROMA               0x80114
#define SMIALPHA_COLORTBL             0x80118

struct reg_and_value {
   unsigned long reg_ ;
   unsigned long value_ ;
};

struct yuv_slice {
   unsigned x ;
   unsigned y ;
   unsigned w ;
   unsigned h ;
   unsigned char *ybuf ;
   unsigned ystride ;
   unsigned char *ubuf ;
   unsigned ustride ;
   unsigned char *vbuf ;
   unsigned vstride ;
};

/*
 * SM-501 ioctls
 */

#define BASE_MAGIC 0xBD

#define SM501_GET_SYNCCOUNT   _IOR(BASE_MAGIC, 0x01, unsigned long) // returns sync count
#define SM501_WAITSYNC        _IOR(BASE_MAGIC, 0x02, unsigned long) // returns sync count after wait
#define SM501_READREG         _IOWR(BASE_MAGIC, 0x04, unsigned long)
#define SM501_WRITEREG        _IOWR(BASE_MAGIC, 0x05, struct reg_and_value)
#define SM501_EXECCMDLIST     _IOWR(BASE_MAGIC, 0x07, unsigned long) // execute cmd list (SM-501 RAM address of cmdlist)

#define SM501YUV_SETPLANE  _IOWR(BASE_MAGIC, 0x03, unsigned long)
#define SM501YUV_SLICE     _IOWR(BASE_MAGIC, 0x04, struct yuv_slice)

#define SM501ALPHA_SETPLANE _IOW(BASE_MAGIC, 0x01, unsigned long)

#ifdef __KERNEL__
#if defined(CONFIG_MACH_NEON270) && !defined(CONFIG_NEON270ENC)
#define SM501_FBSTART      PXA_CS1_PHYS
#else
#define SM501_FBSTART      PXA_CS3_PHYS
#endif
#endif
#define SM501_FBMAX        0x00700000
#define SM501_MMIOSTART    (SM501_FBSTART+0x3E00000)
#define SM501_MMIOLENGTH   0x00200000

/*
 * Color-space conversion registers
 */
#define SM501_CSC_YSOURCE     0x1000C8 
#define SM501_CSC_CONSTANTS   0x1000CC
#define SM501_CSC_YSOURCEX    0x1000D0
#define SM501_CSC_YSOURCEY    0x1000D4
#define SM501_CSC_USOURCE     0x1000D8
#define SM501_CSC_VSOURCE     0x1000DC
#define SM501_CSC_SRCDIM      0x1000E0
#define SM501_CSC_SRCPITCH    0x1000E4
#define SM501_CSC_DESTINATION 0x1000E8
#define SM501_CSC_DESTDIM     0x1000EC
#define SM501_CSC_DESTPITCH   0x1000F0
#define SM501_CSC_SCALE       0x1000F4
#define SM501_CSC_DESTBASE    0x1000F8
#define SM501_CSC_CONTROL     0x1000FC


#define STUFF_SM501_REG( addr, value ) *( (unsigned long volatile *)((addr)+mmioVirtual) ) = (value)
#define READ_SM501_REG( addr ) ( *( (unsigned long volatile *)((addr)+mmioVirtual) ) )
#define VIDEORAMPTR( ptr ) ((char *)ptr-(char*)fbVirtual)

#define SM501FB_CLASS  "sm501fb"
#define SM501YUV_CLASS "sm501yuv"
#define SM501MEM_CLASS "sm501mem"
#define SM501CMD_CLASS "sm501cmd"
#define SM501INT_CLASS "sm501int"
#define SM501ALPHA_CLASS "sm501alpha"

#endif
