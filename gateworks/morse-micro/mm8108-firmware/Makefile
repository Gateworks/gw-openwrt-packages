#
# Copyright 2026 Gateworks
#
# This is free software, licensed under the 3-Clause BSD License.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

PKG_NAME:=mm8108-firmware
PKG_RELEASE=1

PKG_VERSION:=1.16.4

PKG_SOURCE_VERSION:=68fcf18643b0fe12d1ff73ad03089b8c0b7d2979
PKG_SOURCE_URL:=https://github.com/MorseMicro/morse-firmware.git
PKG_HASH:=dbc6b5428fc6d4defa89a29c00c99a73ca07f704501d3618eddacb826007f486
PKG_SOURCE_PROTO:=git

PKG_MAINTAINER:=Gateworks Corporation <support@gateworks.com>
PKG_BUILD_PARALLEL:=1

RSTRIP:=:

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define Package/mm8108-firmware
  SECTION:=firmware
  CATEGORY:=Firmware
  URL:=$(PKG_SOURCE_URL)
  TITLE:=Morse Micro mm8108 WIFI HaLow firmware
endef

define Build/Compile
endef

define Package/mm8108-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/morse
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/firmware/mm8108b2-rl.bin $(1)/lib/firmware/morse
	$(CP) $(PKG_BUILD_DIR)/bcf/morsemicro/* $(1)/lib/firmware/morse
endef

$(eval $(call BuildPackage,mm8108-firmware))
