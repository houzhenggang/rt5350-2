#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=motor
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/motor
  SUBMENU:=Other modules
#DEPENDS:=@!LINUX_3_3
  TITLE:=Simple motor driver
  FILES:=$(PKG_BUILD_DIR)/motor.ko
#AUTOLOAD:=$(call AutoLoad,30,motor,1)
  KCONFIG:=
endef

define KernelPackage/motor/description
 This is a motor driver program! 
endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,motor))
