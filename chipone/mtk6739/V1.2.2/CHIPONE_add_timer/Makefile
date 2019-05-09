# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_MTK_FTS_DIRECTORY)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

#obj-$(CONFIG_TOUCHSCREEN_CHIPONE) += chipone-tddi.o

obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_tpd_driver.o 
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_i2c_driver.o 
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_core.o 
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_platform.o
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_sfctrlv2.o 
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_spi_flash.o 
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_firmware.o
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_tool.o 
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_sysfs.o 
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE) += cts_test.o

