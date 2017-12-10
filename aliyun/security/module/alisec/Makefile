PWD:=$(shell pwd)
SRC_PWD:=$(KBUILD_SRC)/aliyun/security/module/alisec

ifeq ("$(TARGET_BUILD_VARIANT)", "eng")
alisec_bin:=alisec_mod.preo.eng
endif

ifeq ("$(TARGET_BUILD_VARIANT)", "user")
alisec_bin:= alisec_mod.preo.user
endif

ifeq ("$(TARGET_BUILD_VARIANT)", "userdebug")
alisec_bin:= alisec_mod.preo.userdebug
endif

ALISEC_OUT:=$(shell \
	if [ ! -x "$(PWD)/$(obj)" ]; then \
		mkdir -p $(PWD)/$(obj); \
	fi; \
	cp $(SRC_PWD)/$(alisec_bin) $(PWD)/$(obj)/$(alisec_bin))

obj-$(CONFIG_ALIYUNOS_SECURITY_MODE) += alisec_mod.o

alisec_mod-objs += $(alisec_bin)
alisec_mod-objs += kerndeps.o
alisec_mod-objs += ase/qconfig.o

install: build
	echo "Copy alisec_mod from $(PWD)"

build: $(EXTRACT_KSYM) $(GEN_KSYM) $(CONVERT_S)
	@echo "make alisec mod"
	$(MAKE) -C $(TOP) M=$(PWD)
	$(CROSS_COMPILE)strip --strip-debug alisec_mod.ko

clean:
	@rm -rf tmp_ksym a.out *.o *.ko .*.cmd *.mod.c *.order *.symvers .tmp_versions *~ *.S .*.S ..*.S


