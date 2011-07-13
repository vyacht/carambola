#
# OTG infrastructure and transceiver drivers
#
obj-$(CONFIG_USB_DWC_OTG)	+= dwc_otg.o

dwc_otg-objs := dwc_otg_cil.o dwc_otg_cil_intr.o dwc_otg_param.o

ifeq ($(CONFIG_4xx_SOC),y)
dwc_otg-objs += dwc_otg_apmppc.o
else
dwc_otg-objs += dwc_otg_driver.o
endif

ifneq ($(CONFIG_DWC_DEVICE_ONLY),y)
dwc_otg-objs += dwc_otg_hcd.o dwc_otg_hcd_intr.o \
		dwc_otg_hcd_queue.o
endif

ifneq ($(CONFIG_DWC_HOST_ONLY),y)
dwc_otg-objs += dwc_otg_pcd.o dwc_otg_pcd_intr.o
endif
