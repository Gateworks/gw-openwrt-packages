#
# Copyright (C) 2024 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=sterling-lwb5plus-firmware
PKG_VERSION:=10.4.0.10
PKG_RELEASE:=1

PKG_SOURCE:=laird-lwb5plus-sdio-sa-firmware-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=https://github.com/LairdCP/Sterling-LWB-and-LWB5-Release-Packages/releases/download/LRD-REL-$(PKG_VERSION)
PKG_HASH:=ef505e90641a28454d24b3a7b8c778db95ac658fd90c803dcda06c95f6f50625

PKG_MAINTAINER:=Tim Harvey <tharvey@gateworks.com>

include $(INCLUDE_DIR)/package.mk

define Package/sterling-lwb5plus-firmware-default
  SECTION:=firmware
  CATEGORY:=Firmware
  URL:=https://www.lairdconnect.com/wireless-modules/wifi-modules-bluetooth/sterling-lwb5-plus-wifi-5-bluetooth-5-module
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

