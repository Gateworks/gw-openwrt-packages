#
# Copyright 2025 Gateworks
#
# This is free software, licensed under the GPL 2 license.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=mm6108-driver
PKG_RELEASE:=1

PKG_VERSION:=2.0.1
PKG_LICENSE:=GPLv2
PKG_LICENSE_FILES:=LICENSE
PKG_SOURCE_URL:=https://github.com/Gateworks/morse_driver.git
PKG_SOURCE_VERSION:=e42c40463921bb6c9d03e0029a3cb2888c2e518f
PKG_SOURCE:=$(PKG_NAME)-$(PKG_SOURCE_VERSION).tar.zst
PKG_MIRROR_HASH:=114c27af87bd58ee29042e35b12b1e855547be50512e9a3a1d9c5904c69b6f16
PKG_SOURCE_PROTO:=git

PKG_MAINTAINER:=Gateworks Corporation <support@gateworks.com>
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

# Extract mac80211 version
MAC80211_VERSION:=v$(shell grep '^PKG_VERSION:=' $(TOPDIR)/package/kernel/mac80211/Makefile | cut -d'=' -f2 | head -1)
MORSE_VERSION:=mm6108-$(PKG_VERSION)-gateworks-g$(call version_abbrev,$(PKG_SOURCE_VERSION))

define KernelPackage/mm6108_sdio
  SUBMENU:=Wireless Drivers
  TITLE:=Morse Micro WIFI HaLow mm6108 SDIO driver
  DEPENDS:= +kmod-dot11ah +kmod-mmc +kmod-mac80211 +kmod-lib-crc7
  FILES:= $(PKG_BUILD_DIR)/mm6108_sdio.ko
  AUTOLOAD:=$(call AutoProbe,mm6108_sdio)
  MODPARAMS.mm6108_sdio:=country=US enable_ext_xtal_init=1
  PROVIDES:=kmod-mm6108
endef

define KernelPackage/mm6108_sdio/description
	This package contains the proprietary wireless driver for the MorseMicro mm6108 chip SDIO interface.
endef

MORSE_MAKEDEFS += \
  MORSE_VERSION="$(MORSE_VERSION)" \
  KERNEL_SRC=$(LINUX_DIR) \
  CONFIG_BACKPORT_VERSION=$(MAC80211_VERSION) \
  CONFIG_MORSE_SDIO=y \
  CONFIG_MORSE_SDIO_ALIGNMENT=4 \
  CONFIG_MORSE_USER_ACCESS=y \
  CONFIG_MORSE_POWERSAVE_MODE=0 \
  CONFIG_MORSE_VENDOR_COMMAND=y \
  CONFIG_MORSE_RC=y \
  CONFIG_MORSE_MONITOR=y \
  CONFIG_MORSE_DEBUGFS=y \
  CONFIG_MORSE_COUNTRY=US \
  CONFIG_WLAN_VENDOR_MORSE=m

NOSTDINC_FLAGS = \
	-I$(PKG_BUILD_DIR) \
	-I$(STAGING_DIR)/usr/include/mac80211-backport/uapi \
	-I$(STAGING_DIR)/usr/include/mac80211-backport \
	-I$(STAGING_DIR)/usr/include/mac80211/uapi \
	-I$(STAGING_DIR)/usr/include/mac80211 \
	-include backport/autoconf.h \
	-include backport/backport.h

include $(INCLUDE_DIR)/kernel-defaults.mk

define Build/Compile
	$(MAKE) $(MORSE_MAKEDEFS) $(PKG_JOBS) -C "$(LINUX_DIR)" \
		$(KERNEL_MAKE_FLAGS) \
		M="$(PKG_BUILD_DIR)" \
		NOSTDINC_FLAGS="$(NOSTDINC_FLAGS)" \
		modules
endef

$(eval $(call KernelPackage,mm6108_sdio))
