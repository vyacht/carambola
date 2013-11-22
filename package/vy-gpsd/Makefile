#
# Copyright (C) 2006-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=gpsd
PKG_VERSION:=3.9
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz
#PKG_MD5SUM:=52d9785eaf1a51298bb8900dbde88f98

PKG_BUILD_DEPENDS:=libncurses

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/scons.mk

define Package/vy-gpsd/Default
  DEPENDS:=+librt
  URL:=http://catb.org/gpsd/
endef

define Package/vy-gpsd/Default/description
  gpsd is a userland daemon acting as a translator between GPS and AIS receivers
  and their clients. gpsd listens on port 2947 for clients requesting
  position/time/velocity information.  The receivers are expected to generate
  position information in a well-known format -- as NMEA-0183 sentences, SiRF
  binary, Rockwell binary, Garmin binary format, or other vendor binary
  protocols.  gpsd takes this information from the GPS and translates it into
  something uniform and easier to understand for clients.
endef

define Package/vy-gpsd
  $(call Package/vy-gpsd/Default)
  SECTION:=net
  CATEGORY:=Network
  DEPENDS+= +libcap +libncurses
  TITLE:=An interface daemon for GPS receivers
endef

define Package/vy-gpsd/conffiles
/etc/config/gpsd
endef

define Package/vy-gpsd/description
  $(call Package/vy-gpsd/Default/description)
  This package contains the GPS daemon.
endef

SCONS_OPTIONS += \
	dbus_export=no \
	tsip=no \
	fv18=no \
	tripmate=no \
	earthmate=no \
	itrax=no \
	navcom=no \
	ubx=no \
	evermore=no \
	ntrip=no \
	libgpsmm=no \
	libQgpsmm=no \
	bluez=no \
	strip=no \
	python=no \
	implicit_link=no \
	chrpath=no

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_BUILD_DIR)/gps.h $(1)/usr/include/
	$(CP) $(PKG_BUILD_DIR)/libgpsmm.h $(1)/usr/include/
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_BUILD_DIR)/libgps*.so* $(1)/usr/lib/
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(CP) $(PKG_BUILD_DIR)/libgps*.pc $(1)/usr/lib/pkgconfig/
endef

define Package/vy-gpsd/install
#gpsd install
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/gpsd.config $(1)/etc/config/gpsd
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/gpsd.init $(1)/etc/init.d/gpsd
	$(INSTALL_DIR) $(1)/etc/hotplug.d/usb
	$(INSTALL_DATA) ./files/gpsd.hotplug $(1)/etc/hotplug.d/usb/20-gpsd
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/gpsd $(1)/usr/sbin/
#lib install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_BUILD_DIR)/libgpsd.so.* $(1)/usr/lib/

#clients
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) \
		$(PKG_BUILD_DIR)/cgps \
		$(PKG_BUILD_DIR)/gps{ctl,decode,mon,pipe} \
		$(PKG_BUILD_DIR)/gpxlogger \
		$(PKG_BUILD_DIR)/lcdgps \
		$(1)/usr/bin/

#lib
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_BUILD_DIR)/libgps.so.* $(1)/usr/lib/
endef

define Package/vy-gpsd/postinst
#!/bin/sh
# check if we are on real system
if [ -z "$${IPKG_INSTROOT}" ]; then
        echo "Enabling rc.d symlink for gpsd"
        /etc/init.d/gpsd enable
fi
exit 0
endef

define Package/vy-gpsd/prerm
#!/bin/sh
# check if we are on real system
if [ -z "$${IPKG_INSTROOT}" ]; then
        echo "Removing rc.d symlink for gpsd"
        /etc/init.d/gpsd disable
fi
exit 0
endef


$(eval $(call BuildPackage,vy-gpsd))

