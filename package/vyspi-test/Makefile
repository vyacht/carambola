#
# Copyright (C) 2009 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=vyspi-test
PKG_RELEASE:=$(LINUX_VERSION)

include $(INCLUDE_DIR)/package.mk

define Package/vyspi-test
  SECTION:=utils
  CATEGORY:=Utilities
  DEPENDS:=+kmod-vyspi
  TITLE:=SPI testing utility
  VERSION:=$(LINUX_VERSION)-$(PKG_RELEASE)
endef

define Package/vyspi-test/description
  VYSPI testing utility.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) -o $(PKG_BUILD_DIR)/vyspi-test \
		$(PKG_BUILD_DIR)/vyspi-test.c
endef

define Package/vyspi-test/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/vyspi-test $(1)/bin/
endef

$(eval $(call BuildPackage,vyspi-test))
