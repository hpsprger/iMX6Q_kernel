#include <linux/init.h>
#include <linux/module.h>
 
extern void hello_fun(void);

static int rocke_net_card_init(void)
{
    hello_fun();
    printk(KERN_EMERG "rock net card init....\n");
    return 0;
}

static void rock_net_card_exit(void)
{
    printk(KERN_EMERG "rock net card exit....\n");
}


module_init(rock_module_init);
module_exit(rock_module_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("ROCKLEE");

