PWD:=$(shell pwd)
TOP:=../../../../../../../out/target/product/${MTK_PROJECT}/obj/KERNEL_OBJ/

obj-m += aliaudit_mod.o
aliaudit_mod-objs := aliaudit_mod.preo

install: build
	echo "Copy aliaudit_mod from $(PWD)"

build:
	@echo "make aliaudit mod"
	$(MAKE) -C $(TOP) M=$(PWD)
	$(CROSS_COMPILE)strip --strip-debug aliaudit_mod.ko

clean:
	@rm -rf tmp_ksym a.out *.o *.ko .*.cmd *.mod.c *.order *.symvers .tmp_versions *~ *.S .*.S ..*.S


