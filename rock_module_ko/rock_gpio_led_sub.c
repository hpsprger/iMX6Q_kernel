#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/sched.h>   //wake_up_process()
#include <linux/kthread.h> //kthread_create()
#include <linux/err.h> //IS_ERR()

static void hello_fun(void)
{
  printk("##### helloworld####\n");
}
EXPORT_SYMBOL(hello_fun);
