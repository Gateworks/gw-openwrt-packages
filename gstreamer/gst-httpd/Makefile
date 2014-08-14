include $(TOPDIR)/rules.mk

PKG_NAME:=gst-httpd
PKG_VERSION:=1.1
PKG_RELEASE:=1
PKG_REV:=0e7a45a2f603774ae34b809f3fbcbab68aac8d37

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=git://github.com/tharvey/gst-httpd.git
PKG_SOURCE_SUBDIR:=gst-httpd-$(PKG_VERSION)
PKG_SOURCE_VERSION:=$(PKG_REV)

include $(INCLUDE_DIR)/package.mk

define Package/gst-httpd
  SECTION:=net
  CATEGORY:=Gateworks
  TITLE:=Gstreamer based HTTP server to stream MJPEG
  MAINTAINER:=Gateworks Corporation <support@gateworks.com>
  DEPENDS:=+gstreamer-utils +libgstapp +v4l-utils \
           +gst-ffmpeg +gst-mod-ffmpegcolorspace \
           +gst-mod-h264parse +gst-mod-jpeg +gst-mod-jpegformat \
           +gst-mod-multipart +gst-mod-rtp +gst-mod-rtpmanager \
           +gst-mod-udp +gst-mod-video4linux2 +gst-mod-videorate \
           +gst-mod-videotestsrc +gst-mod-app
  DEPENDS+=+kmod-video-core +kmod-video-gspca-core \
           +kmod-video-gspca-sn9c20x +kmod-video-uvc 
endef

# -liconv crept in from pkg-config
include $(INCLUDE_DIR)/nls.mk

define Package/gst-httpd/description
  A simple HTTP server capable of streaming MJPEG (multipart JPEG) from 
  a GStreamer pipeline
endef

define Package/gst-httpd/conffiles
/etc/config/gst-httpd
endef

define Package/gst-httpd/install
	$(INSTALL_DIR) $(1)/usr/sbin $(1)/etc/config $(1)/usr/share/gst-httpd/www $(1)/etc/init.d/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/gst-httpd $(1)/usr/sbin/
	$(INSTALL_DATA) ./files/gst-httpd.config $(1)/etc/config/gst-httpd
	$(INSTALL_BIN) ./files/gst-httpd.init $(1)/etc/init.d/gst-httpd
	cp -r $(PKG_BUILD_DIR)/www $(1)/usr/share/gst-httpd/
endef

$(eval $(call BuildPackage,gst-httpd))
