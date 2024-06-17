#include <linux/init.h>
#include <linux/module.h>

MODULE_AUTHOR("OpenHD");
MODULE_DESCRIPTION("IMGONNARUNFORIT");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

static int __init my_module_init(void) {
    printk(KERN_INFO "DMR driver has been loaded\n");
    return 0;
}

static void __exit my_module_exit(void) {
    printk(KERN_INFO "DMR driver has been unloaded\n");
}

module_init(my_module_init);
module_exit(my_module_exit);
