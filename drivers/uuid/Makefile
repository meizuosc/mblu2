obj-y	:= uuid.o
ifeq ($(TARGET_BUILD_VARIANT),user)
	uuid-objs	:= prebuild/uuid_user.o
else ifeq	($(TARGET_BUILD_VARIANT),userdebug)
	uuid-objs	:= prebuild/uuid_userdebug.o
else
	uuid-objs	:= prebuild/uuid_eng.o
endif