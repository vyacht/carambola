#
# Copyright (C) 2013 Bernd Ocklin <bernd@vyacht.net>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=vyacht-web
PKG_VERSION:=0.3
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Package/vyacht-web
	CATEGORY:=VYACHT
        DEPENDS:=+iomode +luci-lib-core +luci-lib-nixio +uhttpd +uhttpd-mod-lua +jquery-ui
	TITLE:=allows easy web based configuartion of an vYachtWiFi Router
	MAINTAINER:=Bernd Ocklin <bernd@vyacht.net>
endef

define Package/vyacht-web/description
  A tiny package to allow web based router configuration.
endef

define Build/Prepare
endef

define Build/Configure
endef

define Build/Compile
endef

define Package/vyacht-web/install
	$(INSTALL_DIR) $(1)/www
	$(INSTALL_DIR) $(1)/www/img
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_DIR) $(1)/usr/lib/lua/vyacht/
	$(INSTALL_DATA) ./files/index.html $(1)/www/index.html
	$(INSTALL_DATA) ./files/test.html $(1)/www/test.html
	$(INSTALL_DATA) ./files/debug5.html $(1)/www/debug5.html
	$(INSTALL_DATA) ./files/debug6.html $(1)/www/debug6.html
	$(INSTALL_BIN)  ./files/vyacht.lua $(1)/www/vyacht.lua
	$(INSTALL_BIN)  ./files/test-vyacht.lua $(1)/www/
	$(INSTALL_BIN)  ./files/cmd-vyacht.lua $(1)/www/
	$(INSTALL_BIN)  ./files/upgrade.lua $(1)/www/
	$(INSTALL_BIN)  ./files/lib/get-opt-alt.lua $(1)/usr/lib/lua/vyacht/get-opt-alt.lua
	$(INSTALL_BIN)  ./files/lib/ip.lua $(1)/usr/lib/lua/vyacht/ip.lua
	$(INSTALL_BIN)  ./files/lib/mime.lua $(1)/usr/lib/lua/vyacht/mime.lua
	$(INSTALL_BIN)  ./files/lib/json.lua $(1)/usr/lib/lua/vyacht/json.lua
	$(INSTALL_BIN)  ./files/lib/sys.lua $(1)/usr/lib/lua/vyacht/sys.lua
	$(INSTALL_BIN)  ./files/lib/version.lua $(1)/usr/lib/lua/vyacht/version.lua
	$(INSTALL_BIN)  ./files/lib/protocol.lua $(1)/usr/lib/lua/vyacht/protocol.lua
	$(INSTALL_BIN)  ./files/lib/config.lua $(1)/usr/lib/lua/vyacht/config.lua
	$(INSTALL_BIN)  ./files/fix_hosts $(1)/etc/init.d/fix_hosts
	$(INSTALL_DATA) ./files/global.css $(1)/www/global.css
	$(INSTALL_DATA) ./files/jquery-latest.js $(1)/www/jquery-latest.js
	$(INSTALL_DATA) ./files/spin.js $(1)/www/spin.js
	$(INSTALL_DATA) ./files/img/glyphicons_085_repeat.png $(1)/www/img/glyphicons_085_repeat.png
	$(INSTALL_DATA) ./files/img/glyphicons_206_ok_2.png $(1)/www/img/glyphicons_206_ok_2.png
endef

$(eval $(call BuildPackage,vyacht-web))
