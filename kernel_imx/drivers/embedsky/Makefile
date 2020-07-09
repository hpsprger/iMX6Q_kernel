
ifneq ($(wildcard drivers/embedsky/tq_core.c),)
#obj-y	+= media/capture/ov5640.o
#obj-y	+= media/capture/core.o
#obj-y	+= media/capture/lib-sysfs.o
obj-y	+= tq_core.o
obj-y  	+= board_init.o
obj-$(CONFIG_PROCFS)  	+= procfs.o
obj-y  	+= fb_common.o 

obj-$(CONFIG_TQIMX6Q_3)  	+= board/imx6q_3/fb_imx6q_3.o 
obj-$(CONFIG_TQIMX6UL)  	+= board/imx6ul/fb_imx6ul.o 
obj-$(CONFIG_TQIMX6Q_4)  	+= board/imx6q_4/fb_imx6q_4.o 
obj-$(CONFIG_TQ335X)  	+= board/335x/fb_335x.o
obj-${CONFIG_TQ335X_RT} += board/335x_rt/fb_335x.o

obj-$(CONFIG_FT5X06)  	+= touch/edt-ft5x06.o
obj-$(CONFIG_GOODIX)  	+= touch/goodix.o
obj-$(CONFIG_TSC2007)  	+= touch/tsc2007.o

obj-$(CONFIG_WK2124)  	+= wk2124/wk2xxx_spi.o
obj-$(CONFIG_CAT9555)  	+= cat9555/cat9555.o

obj-$(CONFIG_RTL8723BT_LINUX)  	+= wifi/rtl8723bt/rtk_coex.o
obj-$(CONFIG_RTL8723BT_LINUX)  	+= wifi/rtl8723bt/rtk_bt.o
obj-$(CONFIG_RTL8723BT_ANDROID)  	+= wifi/rtl8723bt/rtk_btusb.o

obj-$(CONFIG_OV5640)  	+= camera/ov5640.o

obj-$(CONFIG_SGTL5000)  	+= sound/codecs/sgtl5000.o
obj-$(CONFIG_WM8960)  	+= sound/codecs/wm8960.o

obj-$(CONFIG_SII9022_HDMI) += lcd_to_hdmi/mxsfb_sii902x.o
obj-$(CONFIG_SII9022_AUDIO) += lcd_to_hdmi/imx-sii902x.o
obj-$(CONFIG_EMBEDSKY_KEYPAD) += keypad/matrix_keypad.o

else
obj-y	+= built-in.module
endif
