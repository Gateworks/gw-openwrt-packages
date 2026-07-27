include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ftdi-usb-spi
PKG_RELEASE:=2

PKG_VERSION=1.0
PKG_SOURCE_URL:=https://github.com/gateworks/ftdi-usb-spi
PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=ce3f24bafffa31c41509a2d005058fa6e2ed579a
PKG_MIRROR_HASH:=50380d502927e8e4200fefe55c3191c1818ba0a4cadc215c2d7e1943cc71da4b

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
