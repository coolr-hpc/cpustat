
ifneq ($(KERNELRELEASE),)

obj-m  := cpustat.o 

else

KERNELDIR := /lib/modules/`uname -r`/build 
#CC := 
#ARCH :=

all::
	$(MAKE) -C $(KERNELDIR) M=$(PWD) # CC=$(CC)  ARCH=$(ARCH) 

install::
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean::
	rm -f *.o  *.ko *.mod.c  modules.order  Module.symvers
	rm -f .built-in.o.cmd
	rm -f .turbofreq.*
	rm -rf .tmp_versions

endif
