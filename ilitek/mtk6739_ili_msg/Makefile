ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/msgdrv_v7_bird/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc/

obj-y += ilitek_drv_mtk.o
obj-y += ilitek_drv_iliproc.o
obj-y += ilitek_drv_main.o
obj-y += ilitek_drv_mp_test.o
obj-y += ilitek_drv_update.o

CTP_DEFINES := $(shell echo $(CONFIG_BIRD_MSGDRV_V7_CTP_TSFW) | tr a-z A-Z)
CTP_CONFIG_DEFINE := $(foreach n,$(CTP_DEFINES),-D$(n))

$(info $(CTP_DEFINES) )
$(info $(CTP_CONFIG_DEFINE) )

ccflags-y += $(CTP_CONFIG_DEFINE)
