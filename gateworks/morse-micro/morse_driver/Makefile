#
# Copyright 2025 Gateworks
#
# This is free software, licensed under the GPL 2 license.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=morse_driver
PKG_RELEASE:=2

PKG_VERSION:=1.16.4.1
PKG_LICENSE:=GPLv2
PKG_LICENSE_FILES:=LICENSE
PKG_SOURCE_URL:=https://github.com/Gateworks/morse_driver.git
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=dec5bc215b881c369ff530c440c212ec6dd13183
PKG_MIRROR_HASH:=d9038e8e2b3204df0c6bd8583aaa4ac60b585f8aef710b8f890aab0f6a3c0288

PKG_MAINTAINER:=Gateworks
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

# Extract mac80211 version
MAC80211_VERSION:=v$(shell grep '^PKG_VERSION:=' $(TOPDIR)/package/kernel/mac80211/Makefile | cut -d'=' -f2 | head -1)
MORSE_VERSION:=$(PKG_VERSION)-g$(call version_abbrev,$(PKG_SOURCE_VERSION))

define KernelPackage/morse
  SUBMENU:=Wireless Drivers
  TITLE:=Morse Micro WIFI HaLow driver
  DEPENDS:= +kmod-mmc +kmod-mac80211 +kmod-lib-crc7 +kmod-usb-core
  FILES:=\
    $(PKG_BUILD_DIR)/morse.ko \
    $(PKG_BUILD_DIR)/dot11ah/dot11ah.ko
  AUTOLOAD:=$(call AutoProbe,morse)
  MODPARAMS.morse:=country=US enable_ext_xtal_init=1
  PROVIDES:=kmod-morse
endef

define KernelPackage/morse/description
	This package contains the proprietary wireless driver for the MorseMicro mm6108 chip.
endef

MORSE_MAKEDEFS += \
  MORSE_VERSION="$(MORSE_VERSION)" \
  KERNEL_SRC=$(LINUX_DIR) \
  CONFIG_BACKPORT_VERSION=$(MAC80211_VERSION) \
  CONFIG_MORSE_USB=y \
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

$(eval $(call KernelPackage,morse))
