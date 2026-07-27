#
# Copyright 2025 Gateworks
#
# This is free software, licensed under the 3-Clause BSD License.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

PKG_NAME:=mm6108-firmware
PKG_RELEASE=1

PKG_VERSION:=2.0.1
PKG_SOURCE:=mm6108-firmware-$(PKG_VERSION)-gateworks.tar.gz
PKG_SOURCE_URL:=https://dev.gateworks.com/sources
PKG_HASH:=1cebf77c40d919460f84414c9d5032e2ddaba7af680289993df4e27ba64a68c0

PKG_MIRROR:=0
PKG_BUILD_PARALLEL:=1

RSTRIP:=:

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

TAR_CMD=$(HOST_TAR) -C $(1) $(TAR_OPTIONS)

define Package/mm6108-firmware
  SECTION:=firmware
  CATEGORY:=Firmware
  URL:=$(PKG_SOURCE_URL)
  TITLE:=Morse Micro mm6108 WIFI HaLow firmware
endef

define Build/Compile
endef

define Package/mm6108-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/morse
	$(INSTALL_DATA) "$(PKG_BUILD_DIR)/mm6108-$(PKG_VERSION)/silex_1_16_4_5V.bin" $(1)/lib/firmware/morse/
	$(LN) silex_1_16_4_5V.bin $(1)/lib/firmware/morse/bcf_default.bin
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/mm6108-$(PKG_VERSION)/mm6108.bin $(1)/lib/firmware/morse/
endef

$(eval $(call BuildPackage,mm6108-firmware))
