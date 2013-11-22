#
# Copyright (C) 2013 vyacht.net
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=vyacht-platform
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/vyacht-platform
  SUBMENU:=vYacht
  TITLE:=Custom vYacht Platform module
  FILES:=$(PKG_BUILD_DIR)/vyacht-platform.ko
  KCONFIG:=   
endef

define KernelPackage/vyacht-platform/description
 Kernel module to register all vyacht's platform devices
endef

EXTRA_KCONFIG:= \
	CONFIG_VYACHT_PLATFORM=m

EXTRA_CFLAGS:= \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=m,%,$(filter %=m,$(EXTRA_KCONFIG)))) \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=y,%,$(filter %=y,$(EXTRA_KCONFIG))))

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)" \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(EXTRA_KCONFIG)

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,vyacht-platform))

