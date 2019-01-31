
obj-$(CONFIG_VIDEO0_NUC980) += nuc980_cap0.o
obj-$(CONFIG_VIDEO1_NUC980) += nuc980_cap1.o
ifeq ($(CONFIG_USE_OF),y)
obj-y += sensor0_ov7725.o
obj-y += sensor0_ov5640.o
obj-y += sensor0_nt99141.o
obj-y += sensor0_nt99050.o
obj-y += sensor0_tw9912.o
obj-y += sensor0_gc0308.o
obj-y += sensor1_ov7725.o
obj-y += sensor1_ov5640.o
obj-y += sensor1_nt99141.o
obj-y += sensor1_nt99050.o
obj-y += sensor1_tw9912.o
obj-y += sensor1_gc0308.o
else
obj-$(CONFIG_SENSOR0_OV7725) += sensor0_ov7725.o
obj-$(CONFIG_SENSOR0_OV5640) += sensor0_ov5640.o
obj-$(CONFIG_SENSOR0_NT99141) += sensor0_nt99141.o
obj-$(CONFIG_SENSOR0_NT99050) += sensor0_nt99050.o
obj-$(CONFIG_SENSOR0_TW9912) += sensor0_tw9912.o
obj-$(CONFIG_SENSOR0_GC0308) += sensor0_gc0308.o

obj-$(CONFIG_SENSOR1_OV7725) += sensor1_ov7725.o
obj-$(CONFIG_SENSOR1_OV5640) += sensor1_ov5640.o
obj-$(CONFIG_SENSOR1_NT99141) += sensor1_nt99141.o
obj-$(CONFIG_SENSOR1_NT99050) += sensor1_nt99050.o
obj-$(CONFIG_SENSOR1_TW9912) += sensor1_tw9912.o
obj-$(CONFIG_SENSOR1_GC0308) += sensor1_gc0308.o
endif
