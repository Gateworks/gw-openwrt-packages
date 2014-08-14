#
# Copyright (C) 2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=gsc-daemon
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/gsc-daemon
  SECTION:=net
  CATEGORY:=Gateworks
  TITLE:=Gateworks System Controller Hotplug Daemon
  MAINTAINER:=Gateworks Corporation <support@gateworks.com>
  DEPENDS:=
endef

define Package/gsc-daemon/description
	Daemon for issuing hotplug events for GSC interrupts
endef

define Build/Prepare
	$(INSTALL_DIR) $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) \
		-o $(PKG_BUILD_DIR)/gscd $(PKG_BUILD_DIR)/gscd.c
endef

define Package/gsc-daemon/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/gscd $(1)/usr/sbin/gscd

	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/gsc-daemon.init $(1)/etc/init.d/gscd

	$(INSTALL_DIR) $(1)/etc/hotplug.d/button
	$(INSTALL_BIN) ./files/hotplug_gscd $(1)/etc/hotplug.d/button/10-gsc

	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_BIN) ./files/uci-defaults_gscd $(1)/etc/uci-defaults/10-gsc
endef

$(eval $(call BuildPackage,gsc-daemon))
