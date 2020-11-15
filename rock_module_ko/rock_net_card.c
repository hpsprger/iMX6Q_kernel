#include <linux/init.h>
#include <linux/module.h>
 

static int hello2_init(void)
{
    printk(KERN_EMERG "Hello2 world enter \n");
    return 0;
}

static void hello2_exit(void)
{
    printk(KERN_EMERG "Hello2 world exit \n");
}

