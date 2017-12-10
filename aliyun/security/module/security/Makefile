PWD:=$(shell pwd)
TOP:=../../../../../../../out/target/product/${MTK_PROJECT}/obj/KERNEL_OBJ/

obj-m += alisecurity_mod.o
alisecurity_mod-objs := alisecurity_mod.preo

install: build
	echo "Copy alisecurity_mod from $(PWD)"

build:
	@echo "make alisecurity mod"
	$(MAKE) -C $(TOP) M=$(PWD)
	$(CROSS_COMPILE)strip --strip-debug alisecurity_mod.ko

clean:
	@rm -rf tmp_ksym a.out *.o *.ko .*.cmd *.mod.c *.order *.symvers .tmp_versions *~ *.S .*.S ..*.S


