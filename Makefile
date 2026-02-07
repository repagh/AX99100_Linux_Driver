# Get machine architecture
ARCH := $(shell uname -m)

# Get kernel version
KVER = $(shell uname -r)
KMAJ = $(shell echo $(KVER) | \
sed -e 's/^\([0-9][0-9]*\)\.[0-9][0-9]*\.[0-9][0-9]*.*/\1/')
KMIN = $(shell echo $(KVER) | \
sed -e 's/^[0-9][0-9]*\.\([0-9][0-9]*\)\.[0-9][0-9]*.*/\1/')
KREV = $(shell echo $(KVER) | \
sed -e 's/^[0-9][0-9]*\.[0-9][0-9]*\.\([0-9][0-9]*\).*/\1/')

kver_ge = $(shell \
echo test | awk '{if($(KMAJ) < $(1)) {print 0} else { \
if($(KMAJ) > $(1)) {print 1} else { \
if($(KMIN) < $(2)) {print 0} else { \
if($(KMIN) > $(2)) {print 1} else { \
if($(KREV) < $(3)) {print 0} else { print 1 } \
}}}}}' \
)

# Get kernel directory
KDIR:=/lib/modules/$(KVER)
DEBIAN_VERSION_FILE:=/etc/debian_version
DEBIAN_DISTRO:=$(wildcard $(DEBIAN_VERSION_FILE))

# Get current directory
PWD:=$(shell pwd)

# Get driver module directory
ifeq ($(call kver_ge,2,6,38),1)
# For kernel version >= 2.6.38
	MDIR=$(KDIR)/kernel/drivers/tty/serial
else
	MDIR=$(KDIR)/kernel/drivers/serial
endif
MDIR_PP=$(KDIR)/kernel/drivers/parport



# Get PCI BAR0, BAR1, and IRQ
PARPORT_IO    := $(shell lspci -vv | grep -A10 "Parallel controller: Asix Electronics Corporation AX99100" | grep "Region 0:" | awk '{print $$6}' || true)
PARPORT_IO_HI := $(shell lspci -vv | grep -A10 "Parallel controller: Asix Electronics Corporation AX99100" | grep "Region 1:" | awk '{print $$6}' || true)
PARPORT_IRQ   := $(shell lspci -vv | grep -A10 "Parallel controller: Asix Electronics Corporation AX99100" | grep "Interrupt:" | awk '{print $$7}' || true)

ifneq ($(findstring $(ARCH),arm aarch64),)
# For ARM/ARM64 platform - use ax99100x_pp
	obj-m += ax99100x.o
	obj-m += ax99100x_pp.o
	obj-m += ax99100x_i2c.o
	ax99100x-objs := ax99100x_spi.o ax99100x_sp.o
else
# For x86/x64 or other platforms - use both parport_pc and ax99100x_pp
	obj-m += ax99100x.o
	obj-m += parport_pc.o
	obj-m += ax99100x_pp.o
	obj-m += ax99100x_i2c.o
	ax99100x-objs := ax99100x_spi.o ax99100x_sp.o
endif

$(info current kernel version=$(KMAJ).$(KMIN).$(KREV))
$(info current serial port modules directory=$(MDIR))
$(info current parallel port modules directory=$(MDIR_PP))

default:
	$(RM) *.mod.c *.o *.ko .*.cmd *.symvers
	$(MAKE) -C $(KDIR)/build/ M=$(PWD) modules
ifneq ($(findstring $(ARCH),arm aarch64),)
	@echo "Compiling for ARM platform with ax99100x_pp"
else
	@echo "Compiling parport_pc for non-ARM platform"
	@if [ ! -z "$(PARPORT_IO)" ] && [ ! -z "$(PARPORT_IO_HI)" ] && [ ! -z "$(PARPORT_IRQ)" ]; then \
		echo "Found parport_pc configuration:"; \
		echo "  IO: 0x$(PARPORT_IO)"; \
		echo "  IO_HI: 0x$(PARPORT_IO_HI)"; \
		echo "  IRQ: $(PARPORT_IRQ)"; \
	fi
endif
	gcc -pthread select_BR.c -o select_BR
	gcc -pthread advanced_BR.c -o advanced_BR
	gcc -pthread gpio_99100.c -o gpio_99100
	gcc -pthread -Wall 9bit_test.c -o 9bit_test
	gcc -I /usr/include spi_test.c -o spi_test
	$(RM) *.mod.c *.o .*.cmd *.symvers *.mod
	rm -rf .tmp_version* *~
	rm -rf Module.markers modules.*
	find . -name "*.o.d" -type f -delete

install:
	@echo "Checking and unbinding driver..."
	@for dev in $$(lspci -n | grep "125b:9100" | cut -d' ' -f1); do \
		if [ -e "/sys/bus/pci/devices/0000:$$dev/driver" ]; then \
			driver_path=$$(readlink -f /sys/bus/pci/devices/0000:$$dev/driver); \
			driver=$$(basename $$driver_path); \
			echo "Found device $$dev using driver: $$driver"; \
			echo "0000:$$dev" > /sys/bus/pci/devices/0000:$$dev/driver/unbind; \
			echo "Unbound $$dev from $$driver"; \
		else \
			echo "No active driver found for device $$dev"; \
		fi \
	done
	cp ax99100x.ko $(MDIR)
ifeq ($(call kver_ge,6,5,0),0)
# For kernel version < 6.5.0
	cp parport_pc.ko $(MDIR_PP)
endif	
	depmod -a
ifeq ($(call kver_ge,5,0,0),0)
# For kernel version < 5.0.0
	cp ax99100x.ko /etc/init.d/
	chmod +x /etc/init.d/ax99100x.ko
ifeq ($(DEBIAN_DISTRO), $(DEBIAN_VERSION_FILE))
	ln -s /etc/init.d/ax99100x.ko /etc/rcS.d/Sax99100x || true
else
	ln -s /etc/init.d/ax99100x.ko /etc/rc.d/rc3.d/Sax99100x || true  	
	ln -s /etc/init.d/ax99100x.ko /etc/rc.d/rc5.d/Sax99100x || true
endif
endif
	-modprobe ax99100x
ifeq ($(call kver_ge,6,5,0),0)
# For kernel version < 6.5.0
	-modprobe -r lp
	-modprobe -r parport_pc
	-modprobe parport_pc
	-modprobe lp
endif

uninstall:
	-modprobe -r ax99100x
	rm -f $(MDIR)/ax99100x.*
ifeq ($(call kver_ge,6,5,0),0)	
# For kernel version < 6.5.0
	-modprobe -r lp
	-modprobe -r parport_pc
	rm -f $(MDIR_PP)/parport_pc.*
endif
	depmod -a
ifeq ($(call kver_ge,5,0,0),0)
# For kernel version < 5.0.0
	rm -f /etc/init.d/ax99100x.ko
ifeq ($(DEBIAN_DISTRO), $(DEBIAN_VERSION_FILE))
	rm -f /etc/rcS.d/Sax99100x || true
else
	rm -f /etc/rc.d/rc3.d/Sax99100x
	rm -f /etc/rc.d/rc5.d/Sax99100x
endif
endif

clean:
	$(RM) *.mod.c *.o *.o.* *.ko .*.cmd *.symvers *.o.ur-safe *.mod *.o.d *.bak
	rm -rf .tmp_version* *~
	rm -rf Module.markers modules.* .cache.*
	rm -f .ax99100x_spi.o.d
	rm -f select_BR advanced_BR gpio_99100 spi_test 9bit_test
	find . -name "*.o.d" -type f -delete
	rm -f *.deb

deb:
	@echo "Building deb package..."
	@chmod +x build_deb.sh
	@./build_deb.sh
