MODNAME:=gpio_leds
obj-m:=$(MODNAME).o
PWD:=$(shell pwd)

KERNELDIR=/home/qpicpicxxz/peta_prj/linuxPsBase/hyperion/build/tmp/work/zynqmp_generic-xilinx-linux/linux-xlnx/5.4+git999-r0/linux-xlnx-5.4+git999
CROSS_COMPILE=/opt/petalinux/2020.2/sysroots/x86_64-petalinux-linux/usr/bin/aarch64-xilinx-linux/aarch64-xilinx-linux-
ARCH=arm64

all:
	@$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNELDIR) M=$(PWD) modules > ./log.txt
	@echo "$(MODNAME).c  -> $(MODNAME).ko"
	@$(CROSS_COMPILE)objdump -d $(MODNAME).ko > $(MODNAME)_objdump.txt
	@echo "$(MODNAME).ko -> $(MODNAME)_objdump.txt"
	@echo "make done"

clean:
	@rm -f *.o *~ core .depend .*.cmd *.ko *mod* *symvers *txt

.PHONY: all clean
