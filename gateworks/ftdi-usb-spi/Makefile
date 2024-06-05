include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ftdi-usb-spi
PKG_RELEASE:=1

PKG_VERSION=master
PKG_SOURCE_URL:=https://github.com/gateworks/ftdi-usb-spi
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=3c1df20fa8c15340084ef60b480bcfe7ed60251a

include $(INCLUDE_DIR)/package.mk

define KernelPackage/ftdi-usb-spi
  SUBMENU:=Wireless Drivers
  TITLE:=Driver for FTDI SPI GW16146
  FILES:=$(PKG_BUILD_DIR)/spi-ft232h.ko
  AUTOLOAD:=$(call AutoProbe,spi-ft232h)
endef

define KernelPackage/ftdi-usb-spi/description
  This package contains the driver to register a SPI controller and device for GW16146
endef

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
endef

$(eval $(call KernelPackage,ftdi-usb-spi))
