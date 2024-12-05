#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/types.h>

#define KEYS_KEYS_CHAR_DEV_NAME "pl_key_dev"

struct char_dev {
  dev_t devid;
  struct cdev cdev;
  struct class* class;
  struct device* device;
  struct device_node* nd;
  struct gpio_desc* gpio_d;
  struct semaphore lock;
};

static struct char_dev keys_char = {
    .cdev = {
        .owner = THIS_MODULE,
    },
};

static int char_dev_open(struct inode* inode_p, struct file* file_p) {
  // printk(KERN_ERR"char dev open\n");
  int ret = 0;
  // 尝试获取信号量
  ret = down_interruptible(&keys_char.lock);
  if (ret) {
    printk("%s resource busy\n", KEYS_KEYS_CHAR_DEV_NAME);
    ret = -1;
  } else {
    printk("%s open\n", KEYS_KEYS_CHAR_DEV_NAME);
  }
  return ret;
}

static int char_dev_release(struct inode* inode_p, struct file* file_p) {
  up(&keys_char.lock);
  printk("char dev release\n");
  return 0;
}

static ssize_t char_dev_read(struct file* file_p, char __user* buf, size_t len, loff_t* loff_t_p) {
  int ret             = 0;
  unsigned char state = 0;

  if (0 == gpiod_get_value(keys_char.gpio_d)) {
    mdelay(50);
    // 读取电平
    while (0 == gpiod_get_value(keys_char.gpio_d))
      ;
    state = 1;
    ret   = copy_to_user(buf, &state, sizeof(state));
  } else {
    ret = copy_to_user(buf, &state, sizeof(state));
  }
  return ret;
}

static struct file_operations char_dev_opt = {
    .owner   = THIS_MODULE,
    .open    = char_dev_open,
    .read    = char_dev_read,
    .release = char_dev_release,
};

static int __init char_drv_init(void) {
  printk("Hello keys79\n");
  int ret = 0;
  // 在内核中注册一个字符设备(通常设备的注册是在设备树解析的时候完成的)，没改设备树就需要注册
  // ret = register_chrdev(CHAR_DEV_MAJOR, KEYS_KEYS_CHAR_DEV_NAME, &char_dev_opt);
  // 为设备动态分配一个设备号，存储在devid里
  ret = alloc_chrdev_region(&keys_char.devid, 0, 1, KEYS_KEYS_CHAR_DEV_NAME);
  if (ret < 0) {
    return ret;
  }
  // 初始化字符设备
  cdev_init(&keys_char.cdev, &char_dev_opt);
  // 将字符设备与设备号关联，并注册到内核中
  ret = cdev_add(&keys_char.cdev, keys_char.devid, 1);
  if (ret < 0) {
    return ret;
  }
  // 创建一个设备类
  keys_char.class = class_create(THIS_MODULE, "keys_char_dev");
  if (IS_ERR(keys_char.class)) {
    ret = PTR_ERR(keys_char.class);
    return ret;
  }
  // 在/dev目录下创建设备文件，与设备类和设备号关联(mknod)
  keys_char.device = device_create(keys_char.class, NULL, keys_char.devid, NULL, KEYS_KEYS_CHAR_DEV_NAME);
  if (IS_ERR(keys_char.device)) {
    ret = PTR_ERR(keys_char.device);
    return ret;
  }
  keys_char.device->of_node = of_find_node_by_path("/gpio79keys");
  if (IS_ERR(keys_char.device->of_node)) {
    ret = PTR_ERR(keys_char.device->of_node);
    printk(KERN_ERR "of_find_node_by_path\n");
    return ret;  // 退出初始化过程
  }
  keys_char.gpio_d = gpiod_get_index(keys_char.device, "key79", 0, GPIOD_OUT_HIGH);
  if (keys_char.gpio_d == NULL) {
    printk(KERN_ERR "gpiod_get_index error\n");
    return 0;
  }
  if (IS_ERR(keys_char.gpio_d)) {
    ret = PTR_ERR(keys_char.gpio_d);
    printk(KERN_ERR "gpiod_get_index\n");
    return ret;  // 退出初始化过程
  }
  gpiod_direction_input(keys_char.gpio_d);
  sema_init(&keys_char.lock, 1);

  // gpio_addr_vir = ioremap_wc(GPIO_BASE, GPIO_BASE_LEN);
  // if (NULL == gpio_addr_vir) {
  //   iounmap(gpio_addr_vir);
  //   ret = -1;
  //   return ret;
  // }
  // *GPIO_DIRM1  |= 0x00080000;  // 引脚方向为输出
  // *GPIO_OEN1   |= 0x00080000;  // 使能输出功能

  return ret;
}

static void __exit char_drv_exit(void) {
  printk("Goodbye keys\n");
  // iounmap(gpio_addr_vir);
  gpiod_put(keys_char.gpio_d);
  // unregister_chrdev(CHAR_DEV_MAJOR, KEYS_KEYS_CHAR_DEV_NAME);
  cdev_del(&keys_char.cdev);
  device_destroy(keys_char.class, keys_char.devid);
  class_destroy(keys_char.class);
  unregister_chrdev_region(keys_char.devid, 1);
}

module_init(char_drv_init);
module_exit(char_drv_exit);

MODULE_AUTHOR("DQY");
MODULE_ALIAS("gpio_led");
MODULE_DESCRIPTION("GPIO LED driver");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
