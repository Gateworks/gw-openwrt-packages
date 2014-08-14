#
# Copyright (C) 2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=test-audio
PKG_RELEASE:=1
PKG_VERSION:=1.0

include $(INCLUDE_DIR)/package.mk

define Package/test-audio
  SECTION:=base
  CATEGORY:=Gateworks
  DEPENDS:=+alsa-lib
  TITLE:=Gateworks Supplied Audio Test Tools
  MAINTAINER:=Gateworks Corporation <support@gateworks.com>
endef

define Package/test-audio/description
	Tools used by Gateworks for Testing and Development of Audio
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

TARGET_LDFLAGS += -lm

define Package/test-audio/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/test_audio $(1)/usr/bin/
	$(CP) ./files/* $(1)
endef

$(eval $(call BuildPackage,test-audio))
