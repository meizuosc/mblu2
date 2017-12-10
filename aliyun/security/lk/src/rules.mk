LOCAL_DIR := $(GET_LOCAL_DIR)

INCLUDES += \
         -I$(LOCAL_DIR)/../include \
         -Ilib/openssl/include

MODULES  += \
         lib/openssl

ifeq ($(BUILD_LK_SEC),yes)
OBJS     += \
         $(LOCAL_DIR)/image_verify.o
endif

ifeq ($(BUILD_PIGGY_SEC),yes)
OBJS     += \
         $(LOCAL_DIR)/image_decrypt.o \
         $(LOCAL_DIR)/image_key.o
else ifeq ($(BUILD_RAMDISK_SEC),yes)
OBJS     += \
         $(LOCAL_DIR)/image_decrypt.o \
         $(LOCAL_DIR)/image_key.o
endif
