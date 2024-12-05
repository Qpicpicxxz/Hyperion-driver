// 这样就形成了驱动的分离，一边是 SOC 的硬件资源，另一边是用户设备，他们通过统一的接口来连接
// 驱动driver -> 总线bus -> 设备device
// struct bus_type {
//  int (*match)(struct device *dev, struct device_driver *drv);
// }

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#define CHAR_DEV_MAJOR 200
#define CHAR_DEV_NAME  "led_platform_drv"

struct led_char_dev {
  struct cdev cdev;
  // struct class* class;
  struct device* device;
  // struct device_node* nd;
  struct gpio_desc* gpio_d;
  struct semaphore lock;
};

// int dev_probe_times = 0;
dev_t devid = 0;
struct class* pclass;

// static struct led_char_dev leds_char = {
//     .cdev = {
//         .owner = THIS_MODULE,
//     },
// };

// static u32* GPIO_DIRM1;
// static u32* GPIO_OEN1;
// static u32* GPIO_DATA1;

// static int char_dev_open(struct inode* inode, struct file* file) {
//   printk(KERN_ERR "char_dev_open\n");
//   return 0;
// }

static int char_dev_open(struct inode* inode_p, struct file* file_p) {
  struct led_char_dev* leds_char;
  int ret = 0;
  /* 获取设备结构体指针 */
  // &leds_char
  // 通过 inode_p->i_cdev（指向 leds_char->cdev 的指针）
  // 计算出包含 cdev 的 struct led_char_dev 结构体的起始地址
  leds_char = container_of(inode_p->i_cdev, struct led_char_dev, cdev);
  /* 将设备结构体指针保存在file_p的private_data中 */
  file_p->private_data = leds_char;
  /* 获取设备锁 */
  ret = down_interruptible(&leds_char->lock);
  if (ret) {
    ret = -1;
  } else {
    printk("char dev open\n");
  }
  return ret;
}

static int char_dev_release(struct inode* inode_p, struct file* file_p) {
  struct led_char_dev* leds_char = file_p->private_data;
  up(&leds_char->lock);
  printk(KERN_ERR "char_dev_release\n");
  return 0;
}

static ssize_t char_dev_write(struct file* file, const char __user* buf, size_t len, loff_t* off) {
  struct led_char_dev* leds_char = file->private_data;
  int ret;
  char lc_buf;

  ret = copy_from_user(&lc_buf, buf, len);
  if (ret != 0) {
    printk("copy_from_user failed\n");
    return -EFAULT;
  }

  ret = copy_from_user(&lc_buf, buf, len);
  if (ret) {
    printk(KERN_ERR "copy_from_user failed\n");
    return -EFAULT;
  }

  if (len != 1) {
    printk("len error\n");
    return -EINVAL;
  }

  if (lc_buf == 1)
    gpiod_set_value(leds_char->gpio_d, 1);
  else if (lc_buf == 0)
    gpiod_set_value(leds_char->gpio_d, 0);
  else {
    printk("data error\n");
    return -EINVAL;
  }

  return len;
}

static struct file_operations char_dev_opt = {
    .owner   = THIS_MODULE,
    .open    = char_dev_open,
    .write   = char_dev_write,
    .release = char_dev_release,
};

static int gpio_leds_probe(struct platform_device* pdev) {
  printk("Hello platform leds probe!\n");
  int ret;
  struct led_char_dev* leds_char;
  // struct resource* led_res;
  // size_t res_size;

  // led_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  // if (!led_res) {
  //   printk(KERN_ERR "Failed to get resource\n");
  //   return -ENODEV;
  // }

  leds_char = devm_kzalloc(&pdev->dev, sizeof(struct led_char_dev), GFP_KERNEL);
  if (leds_char == NULL) {
    printk("devm_kzalloc_err\n");
    goto devm_kzalloc_err;
  }

  // 将设备数据（leds_char）与平台设备 pdev 关联
  platform_set_drvdata(pdev, leds_char);
  // 初始化字符设备结构体 cdev，并将其与 char_dev_opt（设备操作函数）关联。然后调用 cdev_add 注册字符设备
  cdev_init(&leds_char->cdev, &char_dev_opt);
  ret = cdev_add(&leds_char->cdev, devid, 1);
  if (ret < 0) {
    printk("cdev_add_err\n");
    goto cdev_add_err;
  }

  printk("probe devidcnt: %d\n", devid);
  leds_char->device = device_create(pclass, NULL, devid, NULL, CHAR_DEV_NAME);
  if (IS_ERR(leds_char->device)) {
    ret = PTR_ERR(leds_char->device);
    printk("device_create_err\n");
    goto device_create_err;
  }

  // dev_probe_times++;
  // devid++;

  // keys_char.device->of_node = of_find_node_by_path("/gpio79keys");
  leds_char->device->of_node = pdev->dev.of_node;
  leds_char->gpio_d          = gpiod_get_index(leds_char->device, "led44", 0, GPIOD_OUT_HIGH);
  if (IS_ERR(leds_char->gpio_d)) {
    ret = PTR_ERR(leds_char->gpio_d);
    printk("gpiod_get_index_err\n");
    goto gpiod_get_index_err;
  }
  gpiod_direction_output(leds_char->gpio_d, 1);
  sema_init(&leds_char->lock, 1);
  return 0;

// err:
//   return ret;
gpiod_get_index_err:
  gpiod_put(leds_char->gpio_d);
device_create_err:
  // device_destroy(pclass, devid);
  device_destroy(pclass, devid);
cdev_add_err:
  cdev_del(&leds_char->cdev);
devm_kzalloc_err:
  devm_kfree(&pdev->dev, leds_char);
  return ret;

  // res_size   = resource_size(led_res);
  // GPIO_DIRM1 = ioremap(led_res->start, res_size);
  // if (!GPIO_DIRM1) {
  //   printk(KERN_ERR "Failed to ioremap GPIO_DIRM1\n");
  //   return -ENOMEM;
  // }

  // GPIO_OEN1  = GPIO_DIRM1 + 1;
  // GPIO_DATA1 = GPIO_DIRM1 + 2;

  // *GPIO_OEN1 = 0xFFFFFFFF;
}

static int gpio_leds_remove(struct platform_device* pdev) {
  printk("Goodbye platform leds remove!\n");
  struct led_char_dev* leds_char = platform_get_drvdata(pdev);
  printk("success platform_get_drvdata!\n");
  gpiod_put(leds_char->gpio_d);
  printk("success gpiod_put!\n");
  cdev_del(&leds_char->cdev);
  printk("success cdev_del!\n");
  // printk("remove devidcnt: %d\n", devid);
  printk("remove devidcnt: %d\n", devid);
  // device_destroy(pclass, devid);
  device_destroy(pclass, devid);
  printk("success device_destroy!\n");
  devm_kfree(&pdev->dev, leds_char);
  printk("success devm_kfree!\n");
  // devid--;
  // dev_probe_times--;
  return 0;
}

// compatible 字段是设备树（Device Tree）结构体的一部分，用于描述硬件设备的兼容性。它是用来标识设备或设备驱动的类型
static const struct of_device_id led_of_dev[] = {
    {
        .compatible = "gpio_44_leds",
    },
    {},
};

static struct platform_driver led_driver = {
    .driver = {
        .name           = "gpio_44_leds",
        .of_match_table = led_of_dev,
    },
    .probe  = gpio_leds_probe,
    .remove = gpio_leds_remove,
};

static int __init char_drv_init(void) {
  printk("Hello platform led!\n");
  int ret = 0;
  // alloc_chrdev_region 分配一个范围的字符设备号
  ret = alloc_chrdev_region(&devid, 0, 1, CHAR_DEV_NAME);
  if (ret < 0)
    return -1;
  pclass = class_create(THIS_MODULE, "leds_char_dev");
  if (IS_ERR(pclass))
    return PTR_ERR(pclass);
  // platform_driver_register 注册平台驱动 led_driver，使得内核能够识别并管理与该驱动相关的设备
  return platform_driver_register(&led_driver);
}

static void __exit char_drv_exit(void) {
  printk("Goodbye platform led!\n");
  platform_driver_unregister(&led_driver);
  class_destroy(pclass);
  unregister_chrdev_region(devid, 1);
}

module_init(char_drv_init);
module_exit(char_drv_exit);

MODULE_AUTHOR("Alinx");
MODULE_ALIAS("gpio_44_leds");
MODULE_DESCRIPTION("GPIO LED driver");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");