#Android makefile to build kernel as a part of Android Build

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_SRC := hardware/intel/linux-2.6
KERNEL_OUT := $(PWD)/$(PRODUCT_OUT)/kernel_build
KERNEL_CONFIG := $(KERNEL_OUT)/.config
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/x86/boot/bzImage
KERNEL_HEADERS_OUT := $(PWD)/$(PRODUCT_OUT)/kernel_headers
KERNEL_MODULES_INSTALL := $(PWD)/$(PRODUCT_OUT)/kernel_modules
KERNEL_MODULES_OUT := $(PWD)/$(PRODUCT_OUT)/root/lib/modules
KERNEL_DROIDBOOT_OUT := $(PWD)/$(PRODUCT_OUT)/droidboot_kernel_build
TARGET_DROIDBOOT_KERNEL := $(KERNEL_DROIDBOOT_OUT)/arch/x86/boot/bzImage
KERNEL_DROIDBOOT_CONFIG := $(KERNEL_DROIDBOOT_OUT)/.config
KERNEL_DROIDBOOT_HEADERS_OUT := $(PWD)/$(PRODUCT_OUT)/droidboot_kernel_headers
KERNEL_DROIDBOOT_MODULES_INSTALL := $(PWD)/$(PRODUCT_OUT)/droidboot_kernel_modules
KERNEL_DROIDBOOT_MODULES_OUT := $(PWD)/$(PRODUCT_OUT)/droidboot_modules
KERNEL_ARCH := i386
KERNEL_CROSS_COMPILE := $(PWD)/prebuilt/linux-x86/toolchain/i686-android-linux-4.4.3/bin/i686-android-linux-
KERNEL_ARCH_BUILD_FLAGS := "ANDROID_TOOLCHAIN_FLAGS=-mno-android"

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/piggy
else
TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)
endif

define mv-modules
mkdir -p $(1);\
ko=`find $(2) -type f -name \*.ko`;\
for i in $$ko; do mv $$i $(1)/; done;
endef

define clean-module-folder
rm -rf $(1)
endef

include $(KERNEL_SRC)/defconfig.mk

define do-kernel-config
	( cp $(3) $(2) && $(7) -C $(4) O=$(1) ARCH=$(5) CROSS_COMPILE=$(6) defoldconfig ) || ( rm -f $(2) && false )
endef

define do-kernel-build
	$(2) -C $(3) O=$(1) ARCH=$(4) CROSS_COMPILE=$(5) $(6)
	$(2) -C $(3) O=$(1) ARCH=$(4) CROSS_COMPILE=$(5) $(6) modules
	$(2) -C $(3) O=$(1) INSTALL_MOD_STRIP=--strip-unneeded INSTALL_MOD_PATH=$(7) ARCH=$(4) CROSS_COMPILE=$(5) $(6) modules_install
endef


$(PRODUCT_OUT)/bzImage: $(TARGET_PREBUILT_INT_KERNEL)
	$(hide)cp $< $@

$(KERNEL_OUT) $(KERNEL_DROIDBOOT_OUT):
	mkdir -p $@

$(KERNEL_CONFIG): $(KERNEL_OUT) $(TARGET_DEFCONFIG)
	$(call do-kernel-config,$(KERNEL_OUT),$@,$(TARGET_DEFCONFIG),$(KERNEL_SRC),$(KERNEL_ARCH),$(KERNEL_CROSS_COMPILE),$(MAKE))

$(KERNEL_DROIDBOOT_CONFIG): $(KERNEL_DROIDBOOT_OUT) $(TARGET_DROIDBOOT_DEFCONFIG)
	$(call do-kernel-config,$(KERNEL_DROIDBOOT_OUT),$@,$(TARGET_DROIDBOOT_DEFCONFIG),$(KERNEL_SRC),$(KERNEL_ARCH),$(KERNEL_CROSS_COMPILE),$(MAKE))

$(KERNEL_OUT)/piggy : $(TARGET_PREBUILT_INT_KERNEL)
	$(hide) gunzip -c $(KERNEL_OUT)/arch/arm/boot/compressed/piggy.gzip > $(KERNEL_OUT)/piggy

$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(call do-kernel-build,$(KERNEL_OUT),$(MAKE),$(KERNEL_SRC),$(KERNEL_ARCH),$(KERNEL_CROSS_COMPILE), $(KERNEL_ARCH_BUILD_FLAGS),$(KERNEL_MODULES_INSTALL))
	$(call mv-modules,$(KERNEL_MODULES_OUT),$(KERNEL_MODULES_INSTALL))
	$(call clean-module-folder,$(KERNEL_MODULES_INSTALL))

$(TARGET_DROIDBOOT_KERNEL): $(KERNEL_DROIDBOOT_OUT) $(KERNEL_DROIDBOOT_CONFIG)
	$(call do-kernel-build,$(KERNEL_DROIDBOOT_OUT),$(MAKE),$(KERNEL_SRC),$(KERNEL_ARCH),$(KERNEL_CROSS_COMPILE), $(KERNEL_ARCH_BUILD_FLAGS),$(KERNEL_DROIDBOOT_MODULES_INSTALL))
	$(call mv-modules,$(KERNEL_DROIDBOOT_MODULES_OUT),$(KERNEL_DROIDBOOT_MODULES_INSTALL))
	$(call clean-module-folder,$(KERNEL_DROIDBOOT_MODULES_INSTALL))

kernelheaders: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) INSTALL_HDR_PATH=$(KERNEL_HEADERS_INSTALL) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_ARCH_BUILD_FLAGS) headers_install

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_ARCH_BUILD_FLAGS) tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_ARCH_BUILD_FLAGS) menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_ARCH_BUILD_FLAGS) savedefconfig
	cp $(KERNEL_OUT)/defconfig $(DEFCONFIGSRC)/$(KERNEL_DEFCONFIG)

endif
