#
# Copyright 2023 Morse Micro
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=morse_cli
PKG_RELEASE=6

PKG_VERSION:=1.16.4

PKG_LICENSE:=GPLv2
PKG_LICENSE_FILES:=

PKG_SOURCE_VERSION:=$(PKG_VERSION)
PKG_SOURCE_URL:=https://github.com/MorseMicro/morse_cli.git
PKG_MIRROR_HASH:=ec2204dfc705d47c5c1962c1469819172c25b3c4b95369c0af060e8dc1c93e51
PKG_SOURCE_PROTO:=git

PKG_MAINTAINER:=Morse Micro <info@morsemicro.com>
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/package.mk

define Package/morse_cli
  SECTION:=utils
  CATEGORY:=Network
  TITLE:=Morse Micro WIFI HaLow driver command line tool
  DEPENDS:= +kmod-morse +libnl +libusb-1.0
  FILES:=\
	$(PKG_BUILD_DIR)/morse_cli
  PROVIDES:=morsectrl
endef

MAKE_FLAGS += \
    MORSECTRL_VERSION_STRING=$(PKG_VERSION) \
    CONFIG_MORSE_TRANS_NL80211=1 \

TARGET_CFLAGS += -I$(STAGING_DIR)/usr/include/libnl3

define Package/morse_cli/install
	$(INSTALL_DIR) $(1)/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/morse_cli $(1)/sbin/
endef

$(eval $(call BuildPackage,morse_cli))
