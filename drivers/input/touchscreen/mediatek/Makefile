include $(srctree)/drivers/misc/mediatek/Makefile.custom
#subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
# In case the platform does NOT support this type of sensors

ifneq ($(CONFIG_TOUCHSCREEN_MTK),)
obj-y   +=  mtk_tpd.o tpd_button.o tpd_calibrate.o tpd_debug.o \
		tpd_default.o tpd_init.o tpd_misc.o tpd_setting.o
endif

obj-$(CONFIG_TOUCHSCREEN_MTK)   += tpd_common_probe.o

obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151TB)	+=  GT1151TB/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT910)	+=  GT910/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336)	+=  ft6336/
obj-$(CONFIG_TOUCHSCREEN_MTK_EKTF2K_V3702)	+=  ektf2k_v3702/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XXTB_HOTKNOT)	+=  GT9XXTB_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT)	+=  GT9XX_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_PHONE)	+=  GT9XX_hotknot_phone/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_I2C_RMI4)	+=  synaptics_i2c_rmi4/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT928)	+=  GT928/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_SCP)	+=  GT9XX_hotknot_scp/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5X0X)	+=  ft5x0x/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_3320_50) +=  synaptics_3320_50/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9271TB_HOTKNOT) +=  GT9271TB_hotknot/
obj-$(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER_4)	+=  unified_driver_4/
obj-$(CONFIG_MTK_TGESTURE)	+=  tgesture/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336_V3610)	+=  ft6336_v3610/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336_V2600)	+=  ft6336_v2600/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336_V3800)	+=  ft6336_v3800/
obj-$(CONFIG_TOUCHSCREEN_MTK_EKTF2K_V3800)	+=  ektf2k_v3800/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336_V2610)	+=  ft6336_v2610/
obj-$(CONFIG_TOUCHSCREEN_MTK_EKTF2K_V2610)	+=  ektf2k_v2610/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5346_V3730)	+=  ft5346_v3730/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5346_V3740)	+=  ft5346_v3740/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5346I)    +=  ft5346i/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT5668)    +=  gt5668/