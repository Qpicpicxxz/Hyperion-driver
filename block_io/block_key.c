/*
 * Linux的阻塞式访问中，应用程序调用 read（）函数从设备中读取数据时，如果设备或者数据没有准备好，
 * 就会进入休眠让出 CPU 资源，准备好时就会唤醒并返回数据给应用程序。内核提供了等待队列机制来实现这里的休眠唤醒工作。
 *
 * 等待队列：等待队列也就是进程组成的队列，Linux 在系统执行会根据不同的状态把进程分成不同的队列，等待队列就是其中之一。
 *
 * 为什么需要？不然CPU就是一直while(1)一直检测按键状态
 *
 * struct wait_queue_head {
 *  spinlock_t		lock;
 *  struct list_head	head;
 * };
 * typedef struct wait_queue_head wait_queue_head_t;
 */

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

#define CHAR_DEV_NAME "pl_key_irq_block_dev"

struct char_dev {
  dev_t devid;
  struct cdev cdev;
  struct class* class;
  struct device* device;
  struct device_node* nd;
  struct gpio_desc* gpio_d;
  struct semaphore lock;
  unsigned int irq;
  struct timer_list timer;
  char key_state;
  wait_queue_head_t wait_q_h;
};

static struct char_dev keys_char = {
    .cdev = {
        .owner = THIS_MODULE,
    },
};

static irqreturn_t key_handler(int irq, void* dev) {
  // mod_timer: 是内核函数，用于修改定时器的超时时间
  // 参数1: 定时器对象指针
  // 参数2: 定时器超时时间，当前时刻jiffies+10s
  // 通过这行代码，表示当中断触发时，会延迟10毫秒后触发一个定时器操作
  // 延迟 10 毫秒的作用通常是为了去抖动 (debounce)
  // 延迟执行逻辑操作（例如修改按键状态），确保只有稳定信号才会触发
  mod_timer(&keys_char.timer, jiffies + msecs_to_jiffies(10));
  return IRQ_RETVAL(IRQ_HANDLED);
}

// 定时器时间到了再将状态改为1
void timer_function(struct timer_list* timer) {
  keys_char.key_state = 1;
  wake_up_interruptible(&keys_char.wait_q_h);
}

static int char_dev_open(struct inode* inode_p, struct file* file_p) {
  int ret = 0;
  ret     = down_interruptible(&keys_char.lock);
  if (ret) {
    printk("%s wait resource break\n", CHAR_DEV_NAME);
  } else {
    printk("%s char dev open\n", CHAR_DEV_NAME);
  }
  return ret;
}

static int char_dev_release(struct inode* inode_p, struct file* file_p) {
  up(&keys_char.lock);
  printk("%s char dev release\n", CHAR_DEV_NAME);
  return 0;
}

/*
 *
 * #define __WAITQUEUE_INITIALIZER(name, tsk) {					\
 *  .private	= tsk,							\
 *  .func		= default_wake_function,				\
 *  .entry		= { NULL, NULL } }
 *
 * #define DECLARE_WAITQUEUE(name, tsk)						\
 * struct wait_queue_entry name = __WAITQUEUE_INITIALIZER(name, tsk)
 *
 * DECLARE_WAITQUEUE(queue_mem, current);
 *
 * 效果：
 *  struct wait_queue_entry queue_mem = {
 *    .private = current,
 *    .func    = default_wake_function,
 *    .entry   = { NULL, NULL }
 * }
 */

// 设备读取函数
static ssize_t char_dev_read(struct file* file_p, char __user* buf, size_t len, loff_t* loff_t_p) {
  int ret = 0;
  // 初始化了一个结构体，声明了一个等待队列元素 a
  // current 是 Linux 内核中表示当前线程的全局变量
  DECLARE_WAITQUEUE(queue_mem, current);
  // 将当前线程放到等待队列里面
  add_wait_queue(&keys_char.wait_q_h, &queue_mem);
  // 将当前线程的状态设置为 TASK_INTERRUPTIBLE，表示线程正在等待一个事件发生，同时允许信号中断
  set_current_state(TASK_INTERRUPTIBLE);
  // 这行代码触发调度器，当前线程主动让出 CPU，进入睡眠状态，等待被唤醒
  schedule();
  // 唤醒后，将线程状态改为可运行，表明它已经可以重新参与 CPU 调度
  set_current_state(TASK_RUNNING);
  remove_wait_queue(&keys_char.wait_q_h, &queue_mem);

  if (signal_pending(current)) {
    return -ERESTARTSYS;
  }
  if (keys_char.key_state) {
    // 将键值状态从内核空间拷贝到用户空间
    ret = copy_to_user(buf, &keys_char.key_state, sizeof(keys_char.key_state));
    // 重置键值状态
    keys_char.key_state = 0;
  } else {
    // 如果键值状态为0，也返回键值状态
    ret = copy_to_user(buf, &keys_char.key_state, sizeof(keys_char.key_state));
  }
  return ret;
}

// 文件操作结构体
static struct file_operations char_dev_opt = {
    .owner   = THIS_MODULE,
    .open    = char_dev_open,
    .read    = char_dev_read,
    .release = char_dev_release,
};

static int __init char_drv_init(void) {
  printk("Hello keys irq 79\n");
  int ret = 0;
  // 初始化字符设备并设置 GPIO 中断
  ret = alloc_chrdev_region(&keys_char.devid, 0, 1, CHAR_DEV_NAME);
  if (ret < 0) {
    goto err;
  }
  // 初始化字符设备并添加到内核
  cdev_init(&keys_char.cdev, &char_dev_opt);
  ret = cdev_add(&keys_char.cdev, keys_char.devid, 1);
  if (ret < 0) {
    goto err;
  }
  // 创建设备类
  keys_char.class = class_create(THIS_MODULE, CHAR_DEV_NAME);
  if (IS_ERR(keys_char.class)) {
    ret = PTR_ERR(keys_char.class);
    goto err;
  }
  // 创建设备节点
  keys_char.device = device_create(keys_char.class, NULL, keys_char.devid, NULL, CHAR_DEV_NAME);
  if (IS_ERR(keys_char.device)) {
    ret = PTR_ERR(keys_char.device);
    goto err;
  }
  // 绑定设备树节点
  keys_char.device->of_node = of_find_node_by_path("/gpio79keys");

  // 获取 GPIO 描述符并设置为输入模式
  keys_char.gpio_d = gpiod_get_index(keys_char.device, "key79", 0, GPIOD_IN);
  gpiod_direction_input(keys_char.gpio_d);

  // 获取 GPIO 中断号
  // gpiod_to_irq() 用于将一个 GPIO 描述符（struct gpio_desc）映射到与其关联的中断号
  keys_char.irq = gpiod_to_irq(keys_char.gpio_d);
  ret           = request_irq(keys_char.irq, key_handler, IRQF_TRIGGER_RISING, "keyirq", NULL);
  if (ret < 0) {
    printk("irq%d request failed\r\n", keys_char.irq);
    return -EFAULT;
  }

  // 初始化定时器: 内核会为指定的定时器初始化必要的结构和状态，以便该定时器能够正确地运行
  timer_setup(&keys_char.timer, timer_function, 0);
  add_timer(&keys_char.timer);

  // 初始化键值状态和信号量
  keys_char.key_state = 0;
  init_waitqueue_head(&keys_char.wait_q_h);
  sema_init(&keys_char.lock, 1);

err:
  return ret;
}

// 模块退出函数
static void __exit char_drv_exit(void) {
  printk("Goodbye keys irq 79\n");
  // 删除定时器
  del_timer(&keys_char.timer);
  // 释放中断资源
  free_irq(keys_char.irq, NULL);
  // 释放 GPIO 资源
  gpiod_put(keys_char.gpio_d);
  // 删除字符设备
  cdev_del(&keys_char.cdev);
  // 销毁设备节点
  device_destroy(keys_char.class, keys_char.devid);
  // 销毁设备类
  class_destroy(keys_char.class);
  // 注销字符设备号
  unregister_chrdev_region(keys_char.devid, 1);
}

// 模块信息
module_init(char_drv_init);  // 模块初始化函数
module_exit(char_drv_exit);  // 模块退出函数

MODULE_AUTHOR("DQY");                   // 模块作者
MODULE_ALIAS("gpio_key");               // 模块别名
MODULE_DESCRIPTION("GPIO KEY driver");  // 模块描述
MODULE_LICENSE("GPL");                  // 模块许可证
