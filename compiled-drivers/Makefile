KERNEL_VERSION=6.1.64-Unraid

# KDIR:=/lib/modules/$(shell  uname -r)/build/ 
KDIR:=/lib/modules/$(KERNEL_VERSION)/build/
# KDIR:=/linux-kernel

DEBIAN_VERSION_FILE:=/etc/debian_version
DEBIAN_DISTRO:=$(wildcard $(DEBIAN_VERSION_FILE))
CURRENT=6.1.64-Unraid
MAJORVERSION=6
MINORVERSION=1
SUBLEVEL=64

ifeq ($(MAJORVERSION),4)
	MDIR=drivers/tty/serial
else
ifeq ($(MAJORVERSION),3)
	MDIR=drivers/tty/serial
else
ifeq ($(MAJORVERSION),2)
	ifneq (,$(filter $(SUBLEVEL),38 39))
		MDIR=drivers/tty/serial
	else
		MDIR=drivers/serial
	endif
else
	MDIR=drivers/serial
endif
endif
endif

obj-m +=99xx.o

default:
	$(RM) *.mod.c *.o *.ko .*.cmd *.symvers
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD) modules
	gcc -pthread select_BR.c -o select_BR
	gcc -pthread advanced_BR.c -o advanced_BR
	gcc -pthread gpio_99xx.c -o gpio_99xx

install:
	cp 99xx.ko  /lib/modules/$(shell uname -r)/kernel/$(MDIR)
	depmod -A
	chmod +x mcs99xx
	cp mcs99xx /etc/init.d/
ifeq ($(DEBIAN_DISTRO), $(DEBIAN_VERSION_FILE))
	ln -s /etc/init.d/mcs99xx /etc/rcS.d/Smcs99xx || true
else
	ln -s /etc/init.d/mcs99xx /etc/rc.d/rc3.d/Smcs99xx || true  	
	ln -s /etc/init.d/mcs99xx /etc/rc.d/rc5.d/Smcs99xx || true
endif
	modprobe 99xx

uninstall:
	modprobe -r 99xx
	rm -f /lib/modules/$(shell uname -r)/kernel/$(MDIR)/99xx.*
	depmod -A
	rm -f /etc/init.d/mcs99xx
ifeq ($(DEBIAN_DISTRO), $(DEBIAN_VERSION_FILE))
	rm -f /etc/init.d/mcs99xx /etc/rcS.d/Smcs99xx || true
else
	rm -f /etc/rc.d/rc3.d/Smcs99xx
	rm -f /etc/rc.d/rc5.d/Smcs99xx
endif

clean:
	$(RM) *.mod.c *.o *.ko .*.cmd *.symvers
	rm -rf .tmp_version* *~
	rm -rf Module.markers modules.*
	rm -f select_BR advanced_BR gpio_99xx
