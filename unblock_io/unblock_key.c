/*
 * 非阻塞IO的处理方式是轮询：select, poll, epoll
 *  int select(int maxfdp, fd_set *readfds, fd_set *writefds, fd_set *errorfds,struct timerval *timeout)
 *    maxfdp：是集合中所有文件描述符的范围，即等于所有文件描述符的最大值加 1
 *    readfds：struct fd_set 结构体可以理解为是文件描述符(file descriptor)的集合
 *    writefds：监视文件是否可写
 *    errorfds：用于监视文件异常
 *    struct timeval {
 *      long tv_sec; // 秒
 *      long tv_usec; // 微妙
 *    }
 *    // 如果传入 NULL，即不传入时间结构，就是一个阻塞函数，直到某个文件描述符发生变化才会返回
 *    // 如果把秒和微妙都设为 0，就是一个非阻塞函数，会立刻返，文件无变化返回 0，有变化返回正值
 *    // select 若在 timeout 时间内没有检测到文件描述符变化，则会直接返回 0，有变化则返回正值
 *    fd_set readfds;
 *    FD_ZERO(&readfds);
 *    FD_SET(fd, &readfds);
 *    ret = select(fd+1, &readfds, NULL, NULL, &timeout);
 *
 *   int poll(struct pollfd *fds, unsigned int nfds, int timeout);
 *   sturct pollfd {
 *     int fd;
 *     short events;   // 这个文件需要监视的事件类型
 *     short revents;  // 内核返回的事件类型
 *   }
 *   nfds：poll监视的文件句柄数量，也就是fds的数组长度
 *   timeout：超时时间
 *
 *   struct pollfd fds[1];
 *   fd = open(filename, O_RDWR | O_NONBLOCK);
 *   fds[0].fd = fd;
 *   fds[0].events = POLLIN;  // 有数据可读
 *   ret = poll(fds, sizeof(fd), 1500);
 *
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
#include <linux/poll.h>
#include <linux/types.h>

#define CHAR_DEV_NAME "pl_key_irq_unblock_dev"

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

// 设备读取函数
static ssize_t char_dev_read(struct file* file_p, char __user* buf, size_t len, loff_t* loff_t_p) {
  int ret = 0;

  if (file_p->f_flags & O_NONBLOCK) {
    ret                 = copy_to_user(buf, &keys_char.key_state, sizeof(keys_char.key_state));
    keys_char.key_state = 0;
  } else {
    DECLARE_WAITQUEUE(queue_mem, current);
    add_wait_queue(&keys_char.wait_q_h, &queue_mem);
    set_current_state(TASK_INTERRUPTIBLE);
    schedule();
    // set_current_state(TASK_RUNNING);
    remove_wait_queue(&keys_char.wait_q_h, &queue_mem);
    if (signal_pending(current)) {
      return -ERESTARTSYS;
    }
    if (keys_char.key_state) {
      ret                 = copy_to_user(buf, &keys_char.key_state, sizeof(keys_char.key_state));
      keys_char.key_state = 0;
    } else {
      ret = copy_to_user(buf, &keys_char.key_state, sizeof(keys_char.key_state));
    }
  }

  return ret;
}

// 在 Linux 内核中，poll 函数通常在用户态进程调用 poll() 或 select() 系统调用时触发
// 当用户态调用poll()或select()的时候，内核会检查用户传入的文件描述符列表，并根据每个描述符的操作类型调用相应的poll函数来判断当前设备或文件的组昂泰
// 如果描述符满足条件，poll()返回相应事件，如果超时，poll()返回0
unsigned int char_dev_poll(struct file* filp, struct poll_table_struct* wait) {
  // 将当前设备加入等待队列
  poll_wait(filp, &keys_char.wait_q_h, wait);
  // 检查设备状态并返回可用事件
  if (keys_char.key_state) {
    return POLLIN;  // 这个是用户态会监控的一些事件
  } else {
    return 0;
  }
}

// 文件操作结构体
static struct file_operations char_dev_opt = {
    .owner   = THIS_MODULE,
    .open    = char_dev_open,
    .read    = char_dev_read,
    .poll    = char_dev_poll,
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
  printk("irq is %d\n", keys_char.irq);
  ret = request_irq(keys_char.irq, key_handler, IRQF_TRIGGER_RISING, "keyirq", NULL);
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
