#
# Makefile for the linux kernel.
#

# Object file lists.


obj-y   := system.o iomux.o cpu.o mm.o devices.o serial.o lpmodes.o pm.o \
sdram_autogating.o bus_freq.o usb_dr.o usb_h1.o usb_h2.o dummy_gpio.o  early_setup.o \
boardid.o

obj-$(CONFIG_MXC_SDMA_API) += dma.o
obj-$(CONFIG_ARCH_MX51) += clock.o
obj-$(CONFIG_ARCH_MX53) += clock.o

obj-$(CONFIG_MACH_MX51_3DS) += mx51_3stack.o mx51_3stack_gpio.o mx51_3stack_pmic_mc13892.o
obj-$(CONFIG_MACH_MX51_BABBAGE) += mx51_babbage.o mx51_babbage_gpio.o mx51_babbage_pmic_mc13892.o
obj-$(CONFIG_MACH_MX53_EVK) += mx53_evk.o mx53_evk_gpio.o mx53_evk_pmic_mc13892.o
obj-$(CONFIG_MACH_MX50_ARM2) += mx50_arm2.o mx50_arm2_gpio.o mx50_arm2_pmic_mc13892.o clock_mx50.o

obj-$(CONFIG_MACH_MX50_TEQUILA) += mx50_yoshi.o dmaengine.o dma-apbh.o mx50_yoshi_gpio.o mx50_yoshi_pmic_mc13892.o clock_mx50_yoshi.o boot_globals.o bootdata.o mx50_suspend.o mx50_freq.o mx50_ddr_freq.o mx50_wfi.o

obj-$(CONFIG_MX50_YOSHI_ACCESSORY) += mx50_yoshi_accessory.o

