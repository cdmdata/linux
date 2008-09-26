#
# Makefile for the linux kernel.
#
#

# Common objects
obj-y 			:= time.o irq.o clock.o serial.o io.o id.o psc.o \
			   gpio.o mux.o dma.o devices.o common.o usb.o iram.o

# Board specific
obj-$(CONFIG_MACH_DAVINCI_EVM)  	+= board-evm.o
obj-$(CONFIG_MACH_DAVINCI_XENON)	+= board-xenon.o env-emac.o
obj-$(CONFIG_DAVINCI_MCBSP)		+= mcbsp.o
