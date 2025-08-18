include $(TOPDIR)/rules.mk

PKG_NAME:=tpm2-tools
PKG_VERSION:=5.7
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_URL:=https://github.com/tpm2-software/$(PKG_NAME)/releases/download/$(PKG_VERSION)/
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz
PKG_HASH:=3810d36b5079256f4f2f7ce552e22213d43b1031c131538df8a2dbc3c570983a
PKG_FIXUP:=configure
PKG_LICENSE:=BSD-3-Clause
PKG_LICENSE_FILES:=docs/LICENSE

PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
  SECTION:=utils
  CATEGORY:=Utilities
  SUBMENU:=TPM2
  TITLE:=TPM2-TOOLS
  URL:=https://github.com/tpm2-software/tpm2-tools
  DEPENDS+= +tpm2-tss +libcurl
endef

TARGET_CFLAGS:= -U_FORTIFY_SOURCE $(TARGET_OPTIMIZATION)$(if $(CONFIG_DEBUG), -g3) $(EXTRA_OPTIMIZATIONS)

define Package/$(PKG_NAME)/description
  TPM (Trusted Platform Module) 2 tools
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/tpm2 $(1)/usr/bin
	for file in $(PKG_INSTALL_DIR)/usr/bin/tpm2_*; do \
		$(LN) tpm2 $(1)/usr/bin/$$$$(basename $$$$file); \
	done
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
