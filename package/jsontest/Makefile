#
# Copyright (C) 2009 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=jsontest
PKG_RELEASE:=$(LINUX_VERSION)

include $(INCLUDE_DIR)/package.mk

define Package/jsontest
  CATEGORY:=VYACHT
  DEPENDS:=+jansson
  TITLE:=json test
  VERSION:=$(LINUX_VERSION)-$(PKG_RELEASE)
endef

define Package/jsontest/description
  VYSPI testing utility.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) -ljansson -o $(PKG_BUILD_DIR)/jsontest \
		$(PKG_BUILD_DIR)/main.c
endef

define Package/jsontest/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/jsontest $(1)/bin/
endef

$(eval $(call BuildPackage,jsontest))
