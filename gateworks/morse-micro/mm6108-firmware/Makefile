#
# Copyright 2025 Gateworks
#
# This is free software, licensed under the 3-Clause BSD License.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

PKG_NAME:=mm6108-firmware
PKG_RELEASE=1

PKG_VERSION:=1.16.4
PKG_SOURCE:=mm6108-firmware-$(PKG_VERSION)-gateworks.tar.gz
PKG_SOURCE_URL:=https://dev.gateworks.com/sources
PKG_HASH:=809cd9c3036dbcf6762c30435b8e198c1f0b68d73ee05dca29ff0dd96a836b4b

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
	$(INSTALL_DATA) "$(PKG_BUILD_DIR)/mm6108/silex_1_16_4_5V.bin" $(1)/lib/firmware/morse/
	$(LN) silex_1_16_4_5V.bin $(1)/lib/firmware/morse/bcf_default.bin
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/mm6108/mm6108.bin $(1)/lib/firmware/morse/
endef

$(eval $(call BuildPackage,mm6108-firmware))
