#
# Copyright 2025 Gateworks
#
# This is free software, licensed under the 3-Clause BSD License.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

PKG_NAME:=morse-fw
PKG_RELEASE=1

PKG_VERSION:=1.16.4
PKG_SOURCE:=gw16159-firmware-$(PKG_VERSION)-gateworks.tar.gz
PKG_SOURCE_URL:=https://dev.gateworks.com/sources
PKG_HASH:=d16b9131651ec17fd179b685b512a82aa286a6c948557a5890a4fc880f0e0458

PKG_BUILD_PARALLEL:=1

RSTRIP:=:

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

TAR_CMD=$(HOST_TAR) -C $(1) $(TAR_OPTIONS)

define Package/morse-fw
  SECTION:=firmware
  CATEGORY:=Firmware
  URL:=$(PKG_SOURCE_URL)
  TITLE:=Morse Micro WIFI HaLow firmware
  PROVIDES:=morse-fw
endef

define Build/Compile
endef

define Package/morse-fw/install
	$(INSTALL_DIR) $(1)/lib/firmware/morse
	# gw16159/mm6108/sdio
	$(INSTALL_DATA) "$(PKG_BUILD_DIR)/gw16159/silex_1_16_4_5V.bin" $(1)/lib/firmware/morse/bcf_default.bin
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/gw16159/mm6108.bin $(1)/lib/firmware/morse/
	# gw16167/mm8108/usb
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/gw16159/bcf_boardtype_0804.bin $(1)/lib/firmware/morse/
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/gw16159/mm8108b2-rl.bin $(1)/lib/firmware/morse/
endef

$(eval $(call BuildPackage,morse-fw))
