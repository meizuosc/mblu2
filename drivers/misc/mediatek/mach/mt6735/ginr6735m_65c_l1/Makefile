
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

#obj-y   +=  lcm/
obj-y   +=  camera/camera/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
obj-y   +=  keypad/
#obj-y   +=  sound/
obj-y   +=  accdet/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  alsps/cm36652-new/
endif
#dixiaobing@wind-mobi.com 20150710 start
ifeq ($(CONFIG_MTK_STK3X1X_NEW),y)
obj-y	+=  alsps/stk3x1x-new/
endif

ifeq ($(CONFIG_MTK_EPL2182_NEW),y)
obj-y	+=  alsps/epl2182-new/
endif

ifeq ($(CONFIG_MTK_STK3X1X),y)
obj-y	+=  alsps/stk3x1x/
endif
#dixiaobing@wind-mobi.com 20150710 end

ifeq ($(CONFIG_MTK_MC3410_NEW),y)
obj-y   +=  accelerometer/mc3410-new/
endif
#dixiaobing@wind-mobi.com 20150710 start
ifeq ($(CONFIG_MTK_LIS3DH_NEW),y)
obj-y   +=  accelerometer/lis3dh-new/
endif
#dixiaobing@wind-mobi.com 20150710 end
#wenggaojian@wind-mobi.com 2015.6.11 begin
ifeq ($(CONFIG_MTK_KXTJ2_1009_NEW),y)
obj-y   +=  accelerometer/KXTJ2_1009-new/
endif
#wenggaojian@wind-mobi.com 2015.6.11 end 

ifeq ($(CONFIG_MTK_ITG1010_NEW),y)
obj-y   +=  gyroscope/ITG1010-new/
endif

ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y	+=  magnetometer/akm09911-new/
endif
#dixiaobing@wind-mobi.com 20150710 start
ifeq ($(CONFIG_MTK_MMC3416X),y)
obj-y	+=  magnetometer/mmc3416x/
endif
ifeq ($(CONFIG_MTK_MMC3524X),y)
obj-y	+=  magnetometer/mmc3524x/
endif
ifeq ($(CONFIG_MTK_ist8303),y)
obj-y	+=  magnetometer/ist8303/
endif
#dixiaobing@wind-mobi.com 20150710 end
#wenggaojian@wind-mobi.com 2015.6.11 begin
ifeq ($(CONFIG_MTK_QMCX983),y)
obj-y	+=  magnetometer/qmcX983/
endif
#wenggaojian@wind-mobi 2015.6.11 end
endif
