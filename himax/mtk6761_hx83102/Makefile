# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/hxchipset/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/i2c/$(MTK_PLATFORM)/

# Makefile for the Himax touchscreen drivers.



obj-y  			+=	himax_ic_incell_core.o
obj-y  	+=	himax_ic_HX83102.o
obj-y   			+=	himax_common.o himax_platform.o
obj-y   			+=	himax_debug.o
obj-y   		+=	himax_inspection.o