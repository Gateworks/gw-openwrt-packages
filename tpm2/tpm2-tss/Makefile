include $(TOPDIR)/rules.mk

PKG_NAME:=tpm2-tss
PKG_VERSION:=4.1.3
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_URL:=https://github.com/tpm2-software/$(PKG_NAME)/releases/download/$(PKG_VERSION)/
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz
PKG_HASH:=37f1580200ab78305d1fc872d89241aaee0c93cbe85bc559bf332737a60d3be8
PKG_FIXUP:=patch-libtool
PKG_LICENSE:=BSD-2-Clause
PKG_LICENSE_FILES:=LICENSE

PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
  SECTION:=utils
  CATEGORY:=Utilities
  SUBMENU:=TPM2
  TITLE:=TPM2-TSS
  URL:=https://github.com/tpm2-software/tpm2-tss
  DEPENDS:= +libusb-1.0 +libusb-compat +libftdi1 +libopenssl
endef

TARGET_CFLAGS:= -U_FORTIFY_SOURCE $(TARGET_OPTIMIZATION)$(if $(CONFIG_DEBUG), -g3) $(EXTRA_OPTIMIZATION)

define Package/$(PKG_NAME)/description
	OSS implementation of the TCG TPM2 Software Stack (TSS2)
endef

CONFIGURE_ARGS += \
	--disable-fapi \
	--disable-policy \
	--disable-doxygen-doc \
	--disable-defaultflags

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/libtss2-{esys,mu,rc,sys,tcti*}.so* $(1)/usr/lib
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_INSTALL_DIR)/usr/include/tss2 $(1)/usr/include/
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libtss2-{esys,mu,rc,sys,tcti*}.{a,so*} $(1)/usr/lib
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig/
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/tss2-{esys,mu,rc,sys,tcti*}.pc $(1)/usr/lib/pkgconfig/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
