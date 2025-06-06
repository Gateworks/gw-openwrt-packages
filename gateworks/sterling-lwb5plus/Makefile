#
# Copyright (C) 2024 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=sterling-lwb5plus-firmware
PKG_VERSION:=12.103.0.5
PKG_RELEASE:=1

PKG_SOURCE:=summit-lwb5plus-sdio-sa-firmware-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=https://github.com/Ezurio/SonaIF-Release-Packages/releases/download/LRD-REL-$(PKG_VERSION)
PKG_HASH:=ffd17719091cf4d9b5fc3c03ae144dee87c2d7ff76b68a0d0662ff13f8438bb8

PKG_MAINTAINER:=Tim Harvey <tharvey@gateworks.com>

include $(INCLUDE_DIR)/package.mk

define Package/sterling-lwb5plus-firmware-default
  SECTION:=firmware
  CATEGORY:=Firmware
  URL:=https://www.ezurio.com/wireless-modules/wifi-modules-bluetooth/sterling-lwb5-plus-wifi-5-bluetooth-5-module
endef

define Build/Prepare
	$(TAR) -C $(PKG_BUILD_DIR) -xjf $(DL_DIR)/$(PKG_SOURCE)
endef

define Build/Compile
	true
endef

# SDIO Firmware
define Package/sterling-lwb5plus-firmware-sdio
  $(Package/sterling-lwb5plus-firmware-default)
  TITLE:=CYW4373E FullMac SDIO firmware
endef

define Package/sterling-lwb5plus-firmware-sdio/install
	$(INSTALL_DIR) $(1)/lib/firmware/brcm
	$(INSTALL_DATA) \
		$(PKG_BUILD_DIR)/lib/firmware/brcm/brcmfmac4373-sdio.bin \
		$(1)/lib/firmware/brcm/brcmfmac4373-sdio.bin
	$(INSTALL_DATA) \
		$(PKG_BUILD_DIR)/lib/firmware/brcm/brcmfmac4373-sdio.clm_blob \
		$(1)/lib/firmware/brcm/brcmfmac4373-sdio.clm_blob
endef
$(eval $(call BuildPackage,sterling-lwb5plus-firmware-sdio))

# NVRAM
define Package/sterling-lwb5plus-nvram
  $(Package/sterling-lwb5plus-firmware-default)
  TITLE:=Sterling LWB NVRAM
endef

define Package/sterling-lwb5plus-nvram/install
	$(INSTALL_DIR) $(1)/lib/firmware/brcm
	$(INSTALL_DATA) \
		$(PKG_BUILD_DIR)/lib/firmware/brcm/brcmfmac4373-sdio.txt \
		$(1)/lib/firmware/brcm/brcmfmac4373-sdio.txt
endef
$(eval $(call BuildPackage,sterling-lwb5plus-nvram))

# HCI patchram Firmware
define Package/sterling-lwb5plus-firmware-hcd
  $(Package/sterling-lwb5plus-firmware-default)
  TITLE:=CYW4373E Bluetooth HCI patchram firmware
endef

define Package/sterling-lwb5plus-firmware-hcd/install
	$(INSTALL_DIR) $(1)/lib/firmware/brcm
	$(INSTALL_DATA) \
		$(PKG_BUILD_DIR)/lib/firmware/brcm/BCM4373A0.hcd \
		$(1)/lib/firmware/brcm/BCM4373A0.hcd
endef
$(eval $(call BuildPackage,sterling-lwb5plus-firmware-hcd))

