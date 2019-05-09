obj-m := c.o
#module-objs := cts_i2c_driver.o cts_core.o cts_platform.o cts_firmware.o icnt8xxx_flash.o cts_tool.o cts_sysfs.o cts_test.o
#objs := cts_i2c_driver.o cts_core.o cts_platform.o cts_firmware.o icnt8xxx_flash.o cts_tool.o cts_sysfs.o cts_test.o
#cts_i2c_driver-objs := cts_i2c_driver.o cts_core.o cts_platform.o cts_firmware.o icnt8xxx_flash.o cts_tool.o cts_sysfs.o cts_test.o
#cts_i2c_driver-objs := cts_core.o cts_platform.o cts_firmware.o icnt8xxx_flash.o cts_tool.o cts_sysfs.o cts_test.o cts_i2c_driver.o 
c-objs := cts_i2c_driver.o cts_core.o cts_platform.o cts_firmware.o icnt8xxx_flash.o cts_tool.o cts_sysfs.o cts_test.o
#objs := cts_i2c_driver.o cts_core.o cts_platform.o cts_firmware.o icnt8xxx_flash.o cts_tool.o cts_sysfs.o cts_test.o
HEAD := $(shell uname -r)
KERNEL := /usr/src/linux-headers-4.14.79-v7+/
all:
	make -C $(KERNEL) M=$(shell pwd) modules
clean:
	make -C $(KERNEL) M=$(shell pwd) clean
install:
	ismod c.ko
uninstall:
	rmmod c.ko
