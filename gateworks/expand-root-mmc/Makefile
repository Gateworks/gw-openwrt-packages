include $(TOPDIR)/rules.mk

PKG_NAME:=expand-root-mmc
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Package/expand-root-mmc
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Expand overlay partition and filesystem on first boot
  DEPENDS:=+parted +losetup +resize2fs
endef

define Package/expand-root-mmc/description
  Automatically resizes the overlay partition and filesystem to fill
  the underlying block device on first boot.
endef

define Build/Compile
endef

define Package/expand-root-mmc/install
	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_BIN) ./files/expand-root-mmc.defaults \
		$(1)/etc/uci-defaults/99-expand-root-mmc
endef

$(eval $(call BuildPackage,expand-root-mmc))
