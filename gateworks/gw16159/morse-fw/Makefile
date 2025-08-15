#
# Copyright 2025 Gateworks
#
# This is free software, licensed under the 3-Clause BSD License.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

PKG_NAME:=morse-fw
PKG_RELEASE=1

PKG_VERSION:=v1.12.4.gateworks
PKG_SOURCE:=gw16159-firmware-1.12.4-gateworks.tar.gz
PKG_SOURCE_URL:=https://dev.gateworks.com/sources
PKG_HASH:=a912e0300d8b2df09f14543976a7867585a427db8d75be2bb0b2a3ccb984f990

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
	$(INSTALL_DATA) "$(PKG_BUILD_DIR)/gw16159/HY103760XB_SX-SDMAH-R2(US)_20240807_5V0.bin" $(1)/lib/firmware/morse/bcf_sxsdmah_fem5p0.bin
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/gw16159/mm6108.bin $(1)/lib/firmware/morse/
endef

$(eval $(call BuildPackage,morse-fw))
