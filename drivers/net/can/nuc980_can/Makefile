#
#  Makefile for the NUC980 CAN controller drivers.
#


obj-$(CONFIG_NUC980_CAN) += nuc980_can.o
obj-$(CONFIG_NUC980_CAN0) += nuc980_can0_platform.o
obj-$(CONFIG_NUC980_CAN1) += nuc980_can1_platform.o
obj-$(CONFIG_NUC980_CAN2) += nuc980_can2_platform.o
obj-$(CONFIG_NUC980_CAN3) += nuc980_can3_platform.o

ccflags-$(CONFIG_CAN_DEBUG_DEVICES) := -DDEBUG
