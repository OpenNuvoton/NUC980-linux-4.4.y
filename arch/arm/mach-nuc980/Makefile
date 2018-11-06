#
# Makefile for the linux kernel.
#

# Object file lists.
#obj-y				:= irq.o time.o clk-ccf.o clk-apll.o clk-upll.o dev.o cpu.o mfp.o mach-nuc980.o pm.o suspend.o sram.o sram_test.o
obj-y                           := irq.o time.o dev.o cpu.o mfp.o mach-nuc980.o pm.o suspend.o sram.o
obj-$(CONFIG_NUC980_FIQ)	+= fiq.o
obj-$(CONFIG_USE_OF)  		+= dt-nuc980-dev-v1.0.o
