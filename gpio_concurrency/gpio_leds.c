#include <linux/module.h>  // 初始化加载的API
#include <linux/kernel.h>  // printk等
#include <linux/fs.h>      // 文件系统操作相关: file_operations, inode
#include <linux/init.h>    // module_init和module_exit
#include <linux/ide.h>     // Integrated Drive Electronics, 硬盘驱动器接口标准
#include <linux/types.h>   // u8, u64等
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>

// #define GPIO_BASE 0x00FF0A0000
// #define GPIO_BASE_LEN 0x300
// static unsigned int *gpio_addr_vir = 0;

// #define GPIO_OEN1  (unsigned int *)(0x248 + (unsigned long)gpio_addr_vir)  // GPIO的输出使能寄存器
// #define GPIO_DIRM1 (unsigned int *)(0x244 + (unsigned long)gpio_addr_vir)  // 用于控制GPIO引脚是输入还是输出
// #define GPIO_DATA1 (unsigned int *)(0x044 + (unsigned long)gpio_addr_vir)  // 用于读取或写入该GPIO引脚的状态

// 指定字符设备驱动程序的主设备号和名字
#define CHAR_DEV_MAJOR 200
#define CHAR_DEV_NAME "gpio_leds"

#define ATOMIC_T_ON
// #define SPINKLOCK_T_ON
// #define SEMAPHORE_ON

struct leds_char_dev {
  dev_t               devid;
  struct cdev         cdev;
  struct class       *class;
  struct device      *device;
  // struct device_node *nd;
  struct gpio_desc   *gpio_d;
#ifdef ATOMIC_T_ON
  atomic_t lock;
#endif

#ifdef SPINKLOCK_T_ON
  spinlock_t    lock;
  int           src_state;
  unsigned long irq_state;
#endif

#ifdef SEMAPHORE_ON
  struct semaphore lock;
#endif
};


static struct leds_char_dev leds_char  = {
  .cdev = {
    .owner = THIS_MODULE,
  },
};

// static u32 *GPIO_DIRM1;
// static u32 *GPIO_OEN1;
// static u32 *GPIO_DATA1;

static int char_dev_open(struct inode *inode_p, struct file *file_p) {
  // printk(KERN_ERR"char dev open\n");
  int ret = 0;
#ifdef ATOMIC_T_ON
  if (1 == atomic_add_return(1, &leds_char.lock)) {
    printk("char dev open\n");
  } else {
    atomic_set(&leds_char.lock, 1); // 在资源忙碌的情况下重置锁的状态
    printk("%s resource busy\n", CHAR_DEV_NAME);
      ret = -1;
  }
#endif

#ifdef SPINKLOCK_T_ON
  // 这个会禁用当前cpu的中断
  spin_lock_irqsave(&leds_char.lock, leds_char.irq_state);
  if (leds_char.src_state) {
    printk("%s resource busy\n", CHAR_DEV_NAME);
    ret = -1;
  } else {
    alinx_char.src_state = 1;
    printk("char dev open\n");
  }
  spin_unlock_irqrestore(&leds_char.lock, leds_char.irq_state);
#endif

#ifdef SEMAPHORE_ON
  // 尝试获取信号量
  ret = down_interruptible(&leds_char.lock);
  if (ret) {
    printk("%s resource busy\n", CHAR_DEV_NAME);
    ret = -1;
  } else {
    printk("char dev open\n");
  }
#endif

  return ret;
}

static int char_dev_release(struct inode *inode_p, struct file *file_p) {
  // printk(KERN_ERR"char dev release\n");
  // return 0;
#ifdef ATOMIC_T_ON
  atomic_set(&leds_char.lock, 0);
#endif

#ifdef SPINKLOCK_T_ON
  spin_lock_irqsave(&leds_char.lock, leds_char.irq_state);
#endif

#ifdef SEMAPHORE_ON
  up(&leds_char.lock);
#endif
  printk("char dev release\n");
}

static ssize_t char_dev_write(struct file *file_p, const char __user *buf, size_t len, loff_t *loff_t_p) {
  int ret = 0;
  char lc_buf = 0;
  ret = copy_from_user(&lc_buf, buf, len);
  if (ret != 0) {
    printk("gpio_leds char_dev_write copy_from_user failed\n");
    return -1;
  }
  if (len != 1) {
    printk("gpio_leds char_dev_write len err\n");
    return -2;
  }
  if (1 == lc_buf) {
    // printk("leds off!\n");
    // *GPIO_DATA1 |= 0x00080000;
    gpiod_set_value(leds_char.gpio_d, 1);
  } else if(0 == lc_buf) {
    // printk("leds on!\n");
    // *GPIO_DATA1 &= 0xfff7ffff;
    gpiod_set_value(leds_char.gpio_d, 0);
  } else {
    printk("gpio_leds char_dev_write data err\n");
  }
}

static struct file_operations char_dev_opt = {
  .owner   = THIS_MODULE,
  .open    = char_dev_open,
  .write   = char_dev_write,
  .release = char_dev_release,
};

static int __init char_drv_init(void) {
  printk("Hello leds\n");
  int ret = 0;
  // 在内核中注册一个字符设备(通常设备的注册是在设备树解析的时候完成的)，没改设备树就需要注册
  // ret = register_chrdev(CHAR_DEV_MAJOR, CHAR_DEV_NAME, &char_dev_opt);
  // 为设备动态分配一个设备号，存储在devid里
  ret = alloc_chrdev_region(&leds_char.devid, 0, 1, CHAR_DEV_NAME);
  if (ret < 0) {
    return ret;
  }
  // 初始化字符设备
  cdev_init(&leds_char.cdev, &char_dev_opt);
  // 将字符设备与设备号关联，并注册到内核中
  ret = cdev_add(&leds_char.cdev, leds_char.devid, 1);
  if (ret < 0) {
    return ret;
  }
  // 创建一个设备类
  leds_char.class = class_create(THIS_MODULE, "leds_char_dev");
  if (IS_ERR(leds_char.class)) {
    ret = PTR_ERR(leds_char.class);
    return ret;
  }
  // 在/dev目录下创建设备文件，与设备类和设备号关联(mknod)
  leds_char.device = device_create(leds_char.class, NULL, leds_char.devid, NULL, CHAR_DEV_NAME);
  if(IS_ERR(leds_char.device)) {
    ret = PTR_ERR(leds_char.device);
    return ret;
  }
  leds_char.device->of_node = of_find_node_by_path("/gpioleds");
  if (IS_ERR(leds_char.device->of_node)) {
    ret = PTR_ERR(leds_char.device->of_node);
    printk(KERN_ERR "of_find_node_by_path\n");
    return ret;  // 退出初始化过程
  }
  leds_char.gpio_d = gpiod_get_index(leds_char.device, "led", 0, GPIOD_OUT_HIGH);
  if (leds_char.gpio_d == NULL) {
    printk(KERN_ERR "gpiod_get_index error\n");
    return 0;
  }
  if (IS_ERR(leds_char.gpio_d)) {
    ret = PTR_ERR(leds_char.gpio_d);
    printk(KERN_ERR "gpiod_get_index\n");
    return ret;  // 退出初始化过程
  }
  gpiod_direction_output(leds_char.gpio_d, 1);

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
  printk("Goodbye leds\n");
  // iounmap(gpio_addr_vir);
  gpiod_put(leds_char.gpio_d);
  // unregister_chrdev(CHAR_DEV_MAJOR, CHAR_DEV_NAME);
  cdev_del(&leds_char.cdev);
  device_destroy(leds_char.class, leds_char.devid);
  class_destroy(leds_char.class);
  unregister_chrdev_region(leds_char.devid, 1);
}

module_init(char_drv_init);
module_exit(char_drv_exit);

MODULE_AUTHOR("DQY");
MODULE_ALIAS("gpio_led");
MODULE_DESCRIPTION("GPIO LED driver");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL") ;

