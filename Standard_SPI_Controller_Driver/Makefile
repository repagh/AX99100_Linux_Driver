KDIR:=/lib/modules/$(shell  uname -r)/build/ 

DEBIAN_VERSION_FILE:=/etc/debian_version
DEBIAN_DISTRO:=$(wildcard $(DEBIAN_VERSION_FILE))
CURRENT=$(shell uname -r)
MAJORVERSION=$(shell uname -r | cut -d '.' -f 1)
MINORVERSION=$(shell uname -r | cut -d '.' -f 2)
SUBLEVEL=$(shell uname -r | cut -d '.' -f 3)

UNAME := $(shell uname -m)

obj-m +=spi-ax99100.o

default:
	$(RM) *.mod.c *.o *.ko .*.cmd *.symvers
	make -C $(KDIR) M=$(PWD) modules
	$(RM) *.mod.c *.o .*.cmd *.symvers *.mod
	rm -rf .tmp_version* *~
	rm -rf Module.markers modules.*		

clean:
	$(RM) *.mod.c *.o *.o.* *.ko .*.cmd *.symvers *.o.ur-safe *.mod *.d *o.d *.spi-ax99100.o.d
	rm -rf .tmp_version* *~
	rm -rf Module.markers modules.* .cache.*
