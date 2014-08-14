#
# Copyright (C) 2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=luci-app-gateworks
PKG_RELEASE:=1
PKG_VERSION:=1.0

include $(INCLUDE_DIR)/package.mk

define Package/luci-app-gateworks
  SECTION:=base
  CATEGORY:=Gateworks
  DEPENDS:=+luci
  TITLE:=Luci support for Gateworks boards
  MAINTAINER:=Gateworks Corporation <support@gateworks.com>
endef

define Package/luci-app-gateworks/description
	Luci web-admin support for Gateworks Boards
endef

define Build/Compile
endef

define Package/luci-app-gateworks/install
	$(CP) ./files/* $(1)
endef

$(eval $(call BuildPackage,luci-app-gateworks))
