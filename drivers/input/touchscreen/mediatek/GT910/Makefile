include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder

ifeq ($(CONFIG_LCM_WIDTH),"800")
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT910_WXGA /
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT910 /
endif
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT910/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  goodix_tool.o
obj-y	+=  gt9xx_driver.o
obj-y	+=  gt9xx_update.o

