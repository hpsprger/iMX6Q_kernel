#include <linux/init.h>
#include <linux/module.h>
 
MODULE_LICENSE("Dual BSD/GPL");

static int hello_init(void)
{
    printk(KERN_EMERG "Hello world enter \n");
    return 0;
}

static void hello_exit(void)
{
    printk(KERN_EMERG "Hello world exit \n");
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_AUTHOR("Song Baohua");
MODULE_DESCRIPTION("A simple Hello world module");
MODULE_ALIAS("a simplest module");
