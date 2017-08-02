KDIR:=/home/esp/SDK/sysroots/i586-poky-linux/usr/src/kernel
#PWD:= $(shell pwd)

CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-
SROOT=/home/esp/SDK/sysroots/i586-poky-linux/

APP = test

obj-m:= imu.o

all:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -Wall -C $(KDIR) M=$(PWD) modules
	i586-poky-linux-gcc -Wall -lm -pthread -o $(APP) main_2.c --sysroot=$(SROOT)

clean:
	rm -f *.ko
	rm -f *.o
	rm -f Module.symvers
	rm -f modules.order
	rm -f *.mod.c
	rm -rf .tmp_versions
	rm -f *.mod.c
	rm -f *.mod.o
	rm -f \.*.cmd
	rm -f Module.markers
	rm -f $(APP) 

