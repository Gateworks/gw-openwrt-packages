#
# Copyright (C) 2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=userled-heartbeat
PKG_RELEASE:=1
PKG_VERSION:=1.0

include $(INCLUDE_DIR)/package.mk

define Package/userled-heartbeat
  SECTION:=base
  CATEGORY:=Gateworks
  DEPENDS:=
  TITLE:=Install userled heartbeat trigger
  MAINTAINER:=Gateworks Corporation <support@gateworks.com>
endef

define Package/userled-heartbeat/description
	Will auto-install userled heartbeat trigger enabling blinking of user led
  on board powerup
endef

define Build/Compile
endef

define Package/userled-heartbeat/install
	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_BIN) ./files/uci-defaults $(1)/etc/uci-defaults/0-userled
endef

$(eval $(call BuildPackage,userled-heartbeat))
