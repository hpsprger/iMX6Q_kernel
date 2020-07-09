
#ifndef _TQ_CORE_H
#define	_TQ_CORE_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/list.h>
//#include <linux/v4l2-controls.h>
#include <linux/sysfs.h>
#include <linux/fb.h>

#ifndef	DEV_INFO
#define	DEV_INFO	"[default]"
#endif

#ifdef	TQ_DEBUG
#define	dbg(format, args...)	printk(KERN_ERR DEV_INFO":%d:[DEBUG]:"format, __LINE__, ##args)
#else
#define	dbg(format, args...)
#endif
#define	dbg_err(format, args...)	printk(KERN_ERR DEV_INFO":%d:[ERROR]:"format, __LINE__, ##args)
#define	dbg_info(format, args...)	printk(KERN_ERR DEV_INFO":%d:[INFO]:"format, __LINE__, ##args)


/*carlos add*/
/*#pragma pack(1)*/
struct display {
	int xres;
	int yres;
	int pix_clk;
	int hbp;
	int hfp;
	int hsw;
	int vbp;
	int vfp;
	int vsw;
	int rgb;
	int bpp;
	char is_type[20];
	char mode_str[20];
	char is_pass[10];
	char diffdisplay;
};

struct USB{
		char touch_pid[7];
		char touch_vid[7];
		char serial_pid[7];
		char serial_vid[7];
};
struct linux_args {
	struct display fb[4];
	struct USB usb_id;
	int nand_page;
};
/*#pragma pack()*/

extern struct linux_args largs;

#define ATAG_EMBEDSKY	0x54410010
#define FIT_TQ_PROP		"embedsky"

#define BOARD_NAME      "TQIMX6Q_4"
#define PRODUCT_VERSION "1.9"

#if defined(CONFIG_TQIMX6Q_3)
#include <video/display_timing.h>
extern int mxcfb_args_run(int i,struct ipuv3_fb_platform_data *pdata);
extern void custom_args_write(int num,struct fb_videomode *modep);
extern int  ldb_psaa_set(int *g_ldb_mode);
extern void diffdisplay_arg(struct fsl_mxc_lcd_platform_data *lcdif,
		struct fsl_mxc_ldb_platform_data *ldb,struct fsl_mxc_hdmi_core_platform_data *hdmi,
		struct mipi_dsi_platform_data *mipi);
#elif defined(CONFIG_TQIMX6Q_4)
extern int fb_replace_modedbs(char* type, void* args);
extern int tq_fb_get_options(int id, char **options);
extern int fb_ipu_di(char* type, int* ipu, int* di);
#elif defined(CONFIG_TQIMX6UL)
extern int custom_display_args(struct display_timing *dt);
extern void tq_set_display(struct display_timings *disp);
#elif defined(CONFIG_TQ335X)
extern struct da8xx_panel* get_ini_fb(void);
#endif

#endif
