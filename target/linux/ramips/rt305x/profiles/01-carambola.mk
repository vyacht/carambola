#
# Copyright (C) 2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/CARAMBOLA
	NAME:=Carambola profile
	PACKAGES:=kmod-rt2800-pci
endef

define Profile/CARAMBOLA/Description
	Profile of carambola board
endef
$(eval $(call Profile,CARAMBOLA))
