#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=gwsoc
PKG_SOURCE_VERSION:=f2215bc5be1bb6c7208ac2e04d0e50c1d4535f59
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/Gateworks/gwsoc.git
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE:=gwsoc-$(PKG_SOURCE_VERSION).tar.gz

PKG_MAINTAINER:=Tim Harvey <tharvey@gateworks.com>

include $(INCLUDE_DIR)/package.mk

define Package/gwsoc
  SECTION:=utils
  CATEGORY:=Gateworks
  TITLE:=Gateworks Programable System On Chip
  MAINTAINER:=Gateworks Corporation <support@gateworks.com>
  DEPENDS:=+hidapi +libusb-1.0 +librt
endef

define Package/gwsoc/description
	Utility for communicating with Gateworks GWSoC family of products
	such as the GW16113.
endef

MAKE_PATH=gwsoc

define Package/gwsoc/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/gwsoc/gwsoc $(1)/usr/sbin/gwsoc
endef

$(eval $(call BuildPackage,gwsoc))
