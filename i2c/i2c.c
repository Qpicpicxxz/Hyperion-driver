#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>  // module_init, module_exit
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/uaccess.h>  // 用户空间与内核空间之间数据访问的函数

// /* Standard module information, edit as appropriate */
// MODULE_LICENSE("GPL");
// MODULE_AUTHOR("Qpicpicxxz");
// MODULE_DESCRIPTION("i2c - first i2c test code");

// #define DRIVER_NAME "i2c"

/*
 * 1. 定义打开、释放、写入读取的函数并绑定到file_operations里
 * 2. 每个ret都要判断处理，一旦出问题就printk并打印调用函数与被调用函数的名字，并goto err
 */

// AXI EEPROM Device
// I2C设备（传感器、EEPROM等）可以被实现为字符设备
struct ax_eeprom_dev {
  dev_t devid;                // EEPROM 的设备 ID
  struct cdev cdev;           // 字符设备结构体
  struct class* class;        // 设备类别结构体
  struct device* device;      // 设备结构体
  struct i2c_client* client;  // I2C 客户端结构体，用于通信
};

// 打开字符设备
static int char_dev_open(struct inode* inode_p, struct file* file_p) {
  printk("i2c open\n");
  int ret = 0;
  struct ax_eeprom_dev* ax_eeprom;

  // 是个宏
  // container_of(ptr, type, member)
  // ptr - 指向结构体成员的指针
  // type - 结构体的类型
  // member - 结构体中的成员名
  // 直接通过type(cdev)获取结构体成员的类型

  ax_eeprom = container_of(inode_p->i_cdev, struct ax_eeprom_dev, cdev);
  // 将 ax_eeprom 设备的指针存储到文件结构的 private_data 成员中
  file_p->private_data = ax_eeprom;  // 将文件结构与设备关联起来

  return ret;
}

static int char_dev_release(struct inode* inode_p, struct file* file_p) {
  return 0;  // 释放操作不需要特定逻辑
}

// 读取设备数据(指向文件结构的指针， 用户区缓冲指针， 长度和文件偏移量指针)
// __user可以帮助开发者和编译器区分指向用户空间的指针与之相内核空间的指针
static ssize_t char_dev_read(struct file* file_p, char __user* buf, size_t len, loff_t* loff_t_p) {
  int ret                         = 0;
  struct ax_eeprom_dev* ax_eeprom = file_p->private_data;  // 获取设备信息
  struct i2c_msg msgs[2];                                  // I2C 消息数组
  unsigned char lo_buf[256] = {0};                         // 数据缓冲区，初始化为 0
  unsigned char data_addr_slave;                           // 从设备地址

  // 从用户空间复制一个字节到 data_addr_slave
  ret = copy_from_user(&data_addr_slave, (unsigned char*)buf, 1);
  if (ret < 0) {
    ret = -1;
    printk("i2c read copy_from_user err\n");
    goto err;
  }

  // 使用i2c总线与EEPROM设备进行通信
  // 许多i2c设备（尤其是存储设备）需要先指定读取的起始地址，然后再进行读取
  msgs[0].addr  = ax_eeprom->client->addr;
  msgs[0].flags = 0;                 // msgs[0].flags设置为0，表示这是一个写操作
  msgs[0].buf   = &data_addr_slave;  // 表示想要访问的从设备内的具体存储地址
  msgs[0].len   = 1;                 // 传输长度1byte
  msgs[1].addr  = ax_eeprom->client->addr;
  msgs[1].flags = I2C_M_RD;
  msgs[1].buf   = lo_buf;
  msgs[1].len   = len > (sizeof(lo_buf) - data_addr_slave) ? (sizeof(lo_buf) - data_addr_slave) : len;
  // 传入使用的i2c适配器，消息与消息的个数
  ret = i2c_transfer(ax_eeprom->client->adapter, msgs, 2);

  if (ret == 2) {
    // 将读取的内容从 `lo_buf` 拷贝到用户空间
    ret = copy_to_user(buf, (char*)lo_buf, len > (sizeof(lo_buf) - data_addr_slave) ? (sizeof(lo_buf) - data_addr_slave) : len);
    if (ret < 0) {
      ret = -1;
      printk("i2c read copy_to_user err\n");
      goto err;
    }
  } else {
    printk("i2c read i2c_transfer err\n");
    ret = -1;
  }

  mdelay(10);

err:
  return ret;
}

// file_p打开的设备文件，buf是用户空间传入的数据缓冲区指针，len是要写入的数据长度，loff_t_p是文件偏移量的指针
// long offset type pointer - 64位整型指针
static ssize_t char_dev_write(struct file* file_p, const char __user* buf, size_t len, loff_t* loff_t_p) {
  int ret                         = 0;
  struct ax_eeprom_dev* ax_eeprom = file_p->private_data;
  struct i2c_msg msgs[1];
  unsigned char lo_buf[17] = {0};

  ret = copy_from_user(lo_buf, (unsigned char*)buf, len > sizeof(lo_buf) ? sizeof(lo_buf) : len);
  if (ret < 0) {
    ret = -1;
    printk("i2c write copy_from_user err\n");
    goto err;
  }

  msgs[0].addr  = ax_eeprom->client->addr;
  msgs[0].flags = 0;  // 0就是写操作
  msgs[0].buf   = lo_buf;
  // 1. 确保len不大于缓冲区lo_buf的大小
  // 2. 确保写入长度不会超出EEPROM的边界，lo_buf[0]是起始地址
  msgs[0].len = (len > sizeof(lo_buf) ? sizeof(lo_buf) : len) > (256 - lo_buf[0] + 1)
                    ? (256 - lo_buf[0] + 1)
                    : (len > sizeof(lo_buf) ? sizeof(lo_buf) : len);
  ret         = i2c_transfer(ax_eeprom->client->adapter, msgs, 1);
  if (ret == 1) {
    ret = 0;
  } else {
    printk("i2c write i2c_transfer err\n");
    ret = -1;
  }

  mdelay(10);

err:
  return ret;
}

static struct file_operations char_dev_opt = {
    .owner   = THIS_MODULE,
    .open    = char_dev_open,
    .read    = char_dev_read,
    .write   = char_dev_write,
    .release = char_dev_release,
};

// 是 I2C 设备驱动的探测函数（probe function），在设备与驱动匹配时调用
// 输入i2c客户端指针（i2c设备地址）
// 此函数的主要目的是处理与设备相关的初始化工作，以确保驱动能够正确控制该设备
static int axi2c_probe(struct i2c_client* client, const struct i2c_device_id* id) {
  printk("Hello i2c!\n");
  int ret = 0;
  struct ax_eeprom_dev* ax_eeprom;
  // 分配一个与&client->dev设备关联的内存（此内存会在设备释放时自动释放）
  // GFP_KERNAL表示在内核态分配
  // Device-managed zero-initialized allocation 「为设备动态分配内存，并将其初始化为零」
  // void *devm_kzalloc(struct device *dev, size_t size, gfp_t flags);
  ax_eeprom = devm_kzalloc(&client->dev, sizeof(struct ax_eeprom_dev), GFP_KERNEL);
  if (ax_eeprom == NULL) {
    ret = -1;
    goto err;
  }
  printk("3Hello i2c!\n");
  ax_eeprom->client = client;
  i2c_set_clientdata(client, ax_eeprom);
  // 为字符设备分配一个设备号，保存在ax_eeprom->devid中，0 表示从第一个设备号开始分配，1 表示分配一个设备号
  alloc_chrdev_region(&ax_eeprom->devid, 0, 1, "ax_eeprom");
  printk("2Hello i2c!\n");
  cdev_init(&ax_eeprom->cdev, &char_dev_opt);
  cdev_add(&ax_eeprom->cdev, ax_eeprom->devid, 1);
  ax_eeprom->class = class_create(THIS_MODULE, "ax_eeprom");
  // 创建设备文件，注册设备，并使其出现在 /dev/ 目录下，设备的名字为 ax_eeprom
  printk("4Hello i2c!\n");
  ax_eeprom->device = device_create(ax_eeprom->class, NULL, ax_eeprom->devid, NULL, "ax_eeprom");
  if (IS_ERR(ax_eeprom->class)) {
    ret = PTR_ERR(ax_eeprom->class);
    goto err;
  }
err:
  return ret;
}

static int axi2c_remove(struct i2c_client* client) {
  printk("Goodbye i2c!\n");
  struct ax_eeprom_dev* ax_eeprom = i2c_get_clientdata(client);

  device_destroy(ax_eeprom->class, ax_eeprom->devid);
  class_destroy(ax_eeprom->class);
  cdev_del(&ax_eeprom->cdev);
  unregister_chrdev_region(ax_eeprom->devid, 1);

  return 0;
}

// 是用于设备树（Device Tree）中的设备匹配的结构体
static const struct of_device_id axi2c_of_match[] = {
    {.compatible = "ax_eeprom"},  // 用于指定设备的兼容字符串
    // 数组的最后一个元素是一个空结构 {}，这是一个惯例，用于标记数组的结束
    {},
};

static const struct i2c_device_id axi2c_id[] = {
    {"ax_eeprom"},  // 指定了 I2C 设备的名称 "ax_eeprom"
    {},
};

static struct i2c_driver axi2c_drv = {
    .driver = {
        .owner          = THIS_MODULE,
        .name           = "ax_eeprom",
        .of_match_table = axi2c_of_match,
    },
    .id_table = axi2c_id,
    .probe    = axi2c_probe,
    .remove   = axi2c_remove,
};

// // 该宏标记函数为初始化函数，该函数在模块加载时被调用
static int __init ax_i2c_init(void) {
  // 使用 i2c_add_driver 函数注册 axi2c_drv 驱动
  return i2c_add_driver(&axi2c_drv);
}

static void __exit ax_i2c_exit(void) {
  i2c_del_driver(&axi2c_drv);
}

module_init(ax_i2c_init);
module_exit(ax_i2c_exit);

// module_i2c_driver(axi2c_drv);

MODULE_LICENSE("GPL v2");
