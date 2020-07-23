#
# Copyright (C) 2020 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=sterling-lwb-firmware
PKG_VERSION:=7.0.0.326
PKG_RELEASE:=1
PKG_REGDOMAIN:=fcc

PKG_SOURCE:=laird-lwb-$(PKG_REGDOMAIN)-firmware-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=https://connectivity-staging.s3.us-east-2.amazonaws.com/2020-04
PKG_HASH:=083eebd74d9a47941ca3a8555fcd73fa4b0b9e8fc8d6719e5c8846e82f1d0ed1

PKG_MAINTAINER:=Tim Harvey <tharvey@gateworks.com>

include $(INCLUDE_DIR)/package.mk

define Package/sterling-lwb-firmware-default
  SECTION:=firmware
  CATEGORY:=Firmware
  URL:=https://www.lairdconnect.com/wireless-modules/wi-fi-bt-modules/sterling-lwb
endef

define Build/Prepare
	$(TAR) -C $(PKG_BUILD_DIR) -xjf $(DL_DIR)/$(PKG_SOURCE)
endef

define Build/Compile
	true
endef

# SDIO Firmware
define Package/sterling-lwb-firmware-sdio
  $(Package/sterling-lwb-firmware-default)
  TITLE:=CYW4343W FullMac SDIO firmware
endef

define Package/sterling-lwb-firmware-sdio/install
	$(INSTALL_DIR) $(1)/lib/firmware/brcm
	$(INSTALL_DATA) \
		$(PKG_BUILD_DIR)/lib/firmware/brcm/brcmfmac43430-sdio-prod.bin \
		$(1)/lib/firmware/brcm/brcmfmac43430-sdio.bin
	$(INSTALL_DATA) \
		$(PKG_BUILD_DIR)/lib/firmware/brcm/brcmfmac43430-sdio.clm_blob \
		$(1)/lib/firmware/brcm/brcmfmac43430-sdio.clm_blob
endef

$(eval $(call BuildPackage,sterling-lwb-firmware-sdio))

# NVRAM
define Package/sterling-lwb-nvram
  $(Package/sterling-lwb-firmware-default)
  TITLE:=Sterling LWB NVRAM
endef

define Package/sterling-lwb-nvram/install
	$(INSTALL_DIR) $(1)/lib/firmware/brcm
	$(INSTALL_DATA) \
		$(PKG_BUILD_DIR)/lib/firmware/brcm/brcmfmac43430-sdio-$(PKG_REGDOMAIN).txt \
		$(1)/lib/firmware/brcm/brcmfmac43430-sdio.txt
endef

$(eval $(call BuildPackage,sterling-lwb-nvram))

# HCI patchram Firmware
define Package/sterling-lwb-firmware-hcd
  $(Package/sterling-lwb-firmware-default)
  TITLE:=CYW4343W Bluetooth HCI patchram firmware
endef

define Package/sterling-lwb-firmware-hcd/install
	$(INSTALL_DIR) $(1)/lib/firmware/brcm
	$(INSTALL_DATA) \
		$(PKG_BUILD_DIR)/lib/firmware/brcm/BCM43430A1.hcd \
		$(1)/lib/firmware/brcm/BCM43430A1.hcd
endef

$(eval $(call BuildPackage,sterling-lwb-firmware-hcd))

