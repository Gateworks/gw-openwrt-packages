#
# Copyright (C) 2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=gsc-update
PKG_RELEASE:=1
PKG_VERSION:=1.4

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

define Build/Prepare
	$(INSTALL_DIR) $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
  $(TARGET_CC) $(TARGET_CFLAGS) -Wall \
    -o $(PKG_BUILD_DIR)/gsc_update $(PKG_BUILD_DIR)/gsc_update.c
endef

define Package/gsc-update/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/gsc_update $(1)/usr/bin/
endef

$(eval $(call BuildPackage,gsc-update))
