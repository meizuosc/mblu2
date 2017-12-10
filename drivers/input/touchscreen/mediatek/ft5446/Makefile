include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ft5446/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ft5446/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

#ge_lib_dir := $(srctree)/drivers/input/touchscreen/mediatek/ft5446 
ldflags-y += -L$(srctree)/drivers/input/touchscreen/mediatek/ft5446/ --whole-archive $(srctree)/drivers/input/touchscreen/mediatek/ft5446/ft_gesture_64bit_lib.a --no-whole-archive 





obj-y	+=  focaltech_driver.o
obj-y	+=  focaltech_ex_fun.o
obj-y	+=  focaltech_ctl.o

