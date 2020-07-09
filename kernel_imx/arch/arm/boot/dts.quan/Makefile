ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_SOC_IMX6Q) += \
	imx6q-sabresd.dtb \
	e9-sabresd.dtb
endif

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs dtbs_install
targets += $(DTB_LIST)

always		:= $(DTB_LIST)
clean-files	:= *.dtb
