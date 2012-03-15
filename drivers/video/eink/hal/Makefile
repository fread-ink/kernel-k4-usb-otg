obj-$(CONFIG_FB_EINK_HAL)            += eink_fb_hal.o
eink_fb_hal-objs                     := einkfb_hal_main.o                   \
                                        einkfb_hal_events.o                 \
                                        einkfb_hal_util.o                   \
                                        einkfb_hal_proc.o                   \
                                        einkfb_hal_mem.o                    \
                                        einkfb_hal_io.o                     \
                                        einkfb_hal_pm.o

obj-$(CONFIG_FB_EINK_HAL_EMULATOR)   += eink_fb_hal_emu.o 
eink_fb_hal_emu-objs                 := ../emulator/emulator_hal.o

obj-$(CONFIG_FB_EINK_HAL_FSLEPDC)    += eink_fb_hal_fslepdc.o
eink_fb_hal_fslepdc-objs             := ../fslepdc/fslepdc_hal.o            \
                                        ../fslepdc/fslepdc_pmic.o
