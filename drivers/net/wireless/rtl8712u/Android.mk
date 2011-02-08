#Android makefile to build kernel as a part of Android Build

ifneq ($(TARGET_SIMULATOR),true)

LOCAL_PATH := $(call my-dir)
DRIVER_SRC := $(realpath $(LOCAL_PATH))
DRIVER=8712u

LINUXPATH := kernel_imx
MODULE_FILE := $(TARGET_OUT)/lib/modules/$(DRIVER).ko
KERNELVER := $(shell cd $(LINUXPATH) && $(MAKE) ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- kernelrelease)

$(MODULE_FILE):
	@echo "Kernel version is $(KERNELVER)"
	$(MAKE) ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- \
		TOPDIR=$(DRIVER_SRC) \
		KVER=$(KERNELVER) \
		-C $(LINUXPATH) \
		M=$(DRIVER_SRC) \
		modules
	mkdir -p $(TARGET_OUT)/lib/modules/
	$(ACP) $(DRIVER_SRC)/$(DRIVER).ko $(TARGET_OUT)/lib/modules/
ALL_PREBUILT+=$(MODULE_FILE)

endif
