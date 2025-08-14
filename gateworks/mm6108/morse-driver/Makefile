include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=mm6108
PKG_RELEASE:=1

#TODO: want gateworks in package name
#PKG_VERSION:=v1.12.4-gateworks
PKG_VERSION:=1.12.4
PKG_SOURCE_URL:=https://github.com/Gateworks/morse_driver.git
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=823c4fd97ad960c8ef9f3171e0b0843fac5ce456
PKG_MIRROR_HASH:=94a0413c84b6f4b3e38124662f495ff076909dca3446139cf77336459fb43efb

PKG_FLAGS:=nonshared

include $(INCLUDE_DIR)/package.mk

# Extract mac80211 version
MAC80211_VERSION:=$(shell grep '^PKG_VERSION:=' $(TOPDIR)/package/kernel/mac80211/Makefile | cut -d'=' -f2)
MAC80211_VERSION_CODE=$(shell echo "$(MAC80211_VERSION)" | \
		      awk -F[v.-] '// {printf("%u", lshift($$2, 16) + lshift($$3, 8) + $$4)}')

define KernelPackage/mm6108
  SUBMENU:=Wireless Drivers
  TITLE:=Driver for MorseMicro mm6108 802.11ah chipset
  DEPENDS:= +kmod-mmc +kmod-mac80211
  FILES:=\
	 $(PKG_BUILD_DIR)/morse.ko \
	 $(PKG_BUILD_DIR)/dot11ah/dot11ah.ko
  AUTOLOAD:=$(call AutoProbe,morse)
  MODPARAMS.morse := \
		country=US \
		enable_ext_xtal_init=1 \
		bcf=bc_sxsdma_fem5p0.bin	
endef

define KernelPackage/mm6108/description
	This package contains the proprietary wireless driver for the MorseMicro mm6108 chip.
endef

# TODO: sanitize this... need all of thse duplicated in MAKEDEFS and FLAGS?
# TODO: MORSE_VERSION not working... hsould show during modprobe
MORSE_MAKEDEFS += \
  MORSE_VERSION="$(PKG_VERSION)" \
  KERNEL_SRC=$(LINUX_DIR) \
  CONFIG_MORSE_SDIO=y \
  CONFIG_MORSE_SDIO_ALIGNMENT=4 \
  CONFIG_MORSE_USER_ACCESS=y \
  CONFIG_MORSE_POWERSAVE_MODE=0 \
  CONFIG_MORSE_VENDOR_COMMAND=y \
  CONFIG_MORSE_RC=y \
  CONFIG_MORSE_COUNTRY=US \
  CONFIG_WLAN_VENDOR_MORSE=m

NOSTDINC_FLAGS = \
	-I$(PKG_BUILD_DIR) \
	-I$(STAGING_DIR)/usr/include/mac80211-backport/uapi \
	-I$(STAGING_DIR)/usr/include/mac80211-backport \
	-I$(STAGING_DIR)/usr/include/mac80211/uapi \
	-I$(STAGING_DIR)/usr/include/mac80211 \
	-DMAC80211_BACKPORT_VERSION_CODE=$(MAC80211_VERSION_CODE) \
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

$(eval $(call KernelPackage,mm6108))
