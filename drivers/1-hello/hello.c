

#include <linux/kernel.h>
#include <linux/module.h>

MODULE_LICENSE("GPL");

static int __init hello_drv_init(void)
{
    printk("%s: hello world!\n", __FUNCTION__);
    return 0;
}
module_init(hello_drv_init);

static void __exit hello_drv_exit(void)
{
    printk("%s: goodbye hello!\n", __FUNCTION__);
}
module_exit(hello_drv_exit);

