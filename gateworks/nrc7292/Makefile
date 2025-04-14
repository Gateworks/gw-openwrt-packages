include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=nrc7292
PKG_RELEASE:=1

PKG_VERSION:=v1.5.1-gateworks
PKG_SOURCE_URL:=https://github.com/Gateworks/nrc7292.git
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=95d6c81300c6f960b114cbc0253d358214542caf
PKG_MIRROR_HASH:=3b3de6130b64b69014294b2f0149f5852c61e10138848463c583bec26ae2e01e

PKG_FLAGS:=nonshared
PKG_EXTMOD_SUBDIRS:=package/src/nrc

include $(INCLUDE_DIR)/package.mk

# Extract mac80211 version
MAC80211_VERSION:=$(shell grep '^PKG_VERSION:=' $(TOPDIR)/package/kernel/mac80211/Makefile | cut -d'=' -f2)
MAC80211_VERSION_CODE=$(shell echo "$(MAC80211_VERSION)" | \
		      awk -F[v.-] '// {printf("%u", lshift($$2, 16) + lshift($$3, 8) + $$4)}')

define KernelPackage/nrc7292
  SUBMENU:=Wireless Drivers
  TITLE:=Driver for Newracom nrc7292 802.11ah chipset
  DEPENDS:=+kmod-mac80211 +kmod-ftdi-usb-spi
  FILES:=$(PKG_BUILD_DIR)/package/src/nrc/nrc.ko
  AUTOLOAD:=$(call AutoProbe,nrc)
  MODPARAMS.nrc:= \
		fw_name=nrc7292_cspi.bin \
		bd_name=nrc7292_bd.dat \
		spi_polling_interval=5
endef

define KernelPackage/nrc7292/description
	This package contains the proprietary wireless driver for the Newracom nrc7292 chip.
endef

NOSTDINC_FLAGS := \
	$(KERNEL_NOSTDINC_FLAGS) \
	-I$(PKG_BUILD_DIR) \
	-I$(STAGING_DIR)/usr/include/mac80211-backport/uapi \
	-I$(STAGING_DIR)/usr/include/mac80211-backport \
	-I$(STAGING_DIR)/usr/include/mac80211/uapi \
	-I$(STAGING_DIR)/usr/include/mac80211 \
	-include backport/backport.h \
	-DMAC80211_BACKPORT_VERSION_CODE=$(MAC80211_VERSION_CODE) \
	-DMAC80211_BACKPORT_VERSION=$(MAC80211_VERSION) \
	-Wno-unused-result

define Build/Compile
	+$(KERNEL_MAKE) $(PKG_JOBS) \
		M="$(PKG_BUILD_DIR)/package/src/nrc" \
		NOSTDINC_FLAGS="$(NOSTDINC_FLAGS)" \
		modules
	$(MAKE) -C $(PKG_BUILD_DIR)/package/src/cli_app \
		CC="$(TARGET_CC)" \
		LD="$(TARGET_CROSS)ld"
endef

$(eval $(call KernelPackage,nrc7292))

define Package/nrc7292_cli_app
	SECTION:=kernel
	CATEGORY:=Kernel modules
	TITLE:=Utility for NRC7292 Wifi driver
	DEPENDS:=@(PACKAGE_kmod-nrc7292)
endef

define Package/nrc7292_cli_app/description
	This package contains the proprietary utility (cli_app) for configuring the
	proprietary wireless driver for the Newracom nrc7292 chip.
endef

define Package/nrc7292_cli_app/install
	$(INSTALL_DIR) $(1)/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/package/src/cli_app/cli_app $(1)/sbin/
endef

$(eval $(call BuildPackage,nrc7292_cli_app))

define Package/nrc7292_firmware
  SECTION:=kernel
  CATEGORY:=Kernel modules
  TITLE:=Firmware for NRC7292 Wifi driver
  DEPENDS:=@(PACKAGE_kmod-nrc7292)
endef

define Package/nrc7292_firmware/description
	This package contains the firmware for the proprietary wireless driver for
	the Newracom nrc7292 chip.
endef

define Package/nrc7292_firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/package/evk/sw_pkg/nrc_pkg/sw/firmware/nrc7292_bd.dat $(1)/lib/firmware
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/package/evk/sw_pkg/nrc_pkg/sw/firmware/nrc7292_cspi.bin $(1)/lib/firmware
endef

$(eval $(call BuildPackage,nrc7292_firmware))
