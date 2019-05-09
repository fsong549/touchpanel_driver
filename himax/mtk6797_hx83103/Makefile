#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

ccflags-y += -I$(srctree)/drivers/input/touchscreen/hxchipset_hx83103/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/wdt/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/wdt/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/i2c/$(MTK_PLATFORM)/

obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET)	+= himax_common.o himax_ic.o himax_platform.o himax_debug.o
