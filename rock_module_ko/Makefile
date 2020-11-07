ifeq ($(KERNELRELEASE),)

KDIR ?= /home/hpsp/rock_space/iMX6Q_files/iMX6Q_kernel/kernel_imx

PWD := $(shell pwd)

modules:
	make -C $(KDIR) M=$(PWD)   modules


.PHONEY:clean  
clean :
	rm -rf .*.ko.cmd *.ko *.mod.c *.mod.o .*.mod.o.cmd *.o .*.o.cmd *.order *.symvers .tmp_versions  *.o.cmd


else

kernel_module_name := rock_module

obj-m := $(kernel_module_name).o
$(kernel_module_name)-objs := module1.o module2.o

endif
