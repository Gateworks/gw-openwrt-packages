#
# Copyright (C) 2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=gsc-update
PKG_VERSION:=1.6
PKG_RELEASE:=2

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=git://github.com/Gateworks/gsc_update
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=87acabd1101c673c5f8c97eec4d35358bc3df3d7
PKG_SOURCE:=$(PKG_NAME)-$(PKG_SOURCE_VERSION).tar.gz

include $(INCLUDE_DIR)/package.mk

define Package/gsc-update
  SECTION:=base
  CATEGORY:=Gateworks
  TITLE:=Gateworks System Controller Firmware Update Application 
  MAINTAINER:=Gateworks Corporation <support@gateworks.com>
endef

define Package/gsc-update/description
	Gateworks System Controller Firmware Update Application
endef

TARGET_CFLAGS += -D${CONFIG_TARGET_BOARD}

define Package/gsc-update/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/gsc_update $(1)/usr/bin/
endef

$(eval $(call BuildPackage,gsc-update))
