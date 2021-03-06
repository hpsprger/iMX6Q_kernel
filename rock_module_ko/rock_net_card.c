#include <linux/init.h>
#include <linux/module.h>
 
extern void hello_fun(void);

static int rock_net_card_init(void)
{
    hello_fun();
    printk(KERN_EMERG "rock net card init....\n");
    return 0;
}

static void rock_net_card_exit(void)
{
    printk(KERN_EMERG "rock net card exit....\n");
}


module_init(rock_net_card_init);
module_exit(rock_net_card_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("ROCKLEE");

