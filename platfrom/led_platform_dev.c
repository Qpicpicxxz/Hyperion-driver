// 这个可以被设备树代替
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#define GPIO_DIRM_1 0xFF0A0244
#define GPIO_OEN_1  0xFF0A0248
#define GPIO_DATA_1 0xFF0A0044

static void led_release(struct device* dev) {
  printk("LED device released\n");
}

// struct resource {
// 	resource_size_t start;
// 	resource_size_t end;
// 	const char *name;
// 	unsigned long flags;
// 	unsigned long desc;
// 	struct resource *parent, *sibling, *child;
// };
static struct resource led_resource[3] = {
    {
        .start = GPIO_DIRM_1,
        .end   = GPIO_DIRM_1 + 3,
        .flags = IORESOURCE_MEM,  // include/linux/ioport.h
    },
    {
        .start = GPIO_OEN_1,
        .end   = GPIO_OEN_1 + 3,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = GPIO_DATA_1,
        .end   = GPIO_DATA_1 + 3,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device led_dev = {
    .name = "gpio_44_leds",
    .id   = 1,
    .dev  = {
         .release = led_release,
    },
    .num_resources = 3,
    .resource      = led_resource,
};

static int __init char_drv_init(void) {
  return platform_device_register(&led_dev);
}

static void __exit char_drv_exit(void) {
  platform_device_unregister(&led_dev);
}

module_init(char_drv_init);
module_exit(char_drv_exit);

MODULE_AUTHOR("Alinx");
MODULE_ALIAS("gpio_44_leds");
MODULE_DESCRIPTION("GPIO LED driver");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
