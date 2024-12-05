#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#define DEVICE_NAME  "dma_dev"
#define BUFFER_SIZE  (128 * 1024)
#define BUFFER_COUNT 32
#define MAX_SIZE     (512 * 64)
#define POLLDMA      1
#define DMA_MEM2HW   10
#define DMA_MEM2MEM  20

// #define dev_info(dev, fmt, ...) /* Do nothing */

struct dma_char_dev {
  dev_t devid;
  struct cdev cdev;
  struct device* dev;
  dma_cookie_t cookie;
  struct mutex mutex;
  spinlock_t lock;
  wait_queue_head_t queue;

  struct dma_chan* chan;
  // struct device* phy_dev;
  struct platform_device* pdev;
  int trans_index;
  struct list_head head;
};

struct dma_vm_list {
  struct dma_char_dev* dma_dev;
  void* addr_u;
  void* addr_k;
  dma_addr_t addr_p;
  size_t size;
  pid_t pid;
  struct list_head list;
  // enum addr_dir direction;
};

enum addr_attr { PHY,
                 VIR };

struct dma_proxy_token {
  void* src_base;
  size_t src_offset;
  void* dst_base;
  size_t dst_offset;
  size_t size;
};

dev_t dma_devid = 0;
static struct class* dma_class;

// static char* src;
// static char* dst;
// dma_addr_t dma_src;
// dma_addr_t dma_dst;

static void dma_callback(void* data) {
  struct dma_char_dev* dma_dev = data;
  dev_info(dma_dev->dev, "[ ] dma_callback()");
  //~ spin_lock_irqsave(&dma_dev->lock, flags);
  wake_up(&dma_dev->queue);
  //~ spin_unlock_irqrestore(&dma_dev->lock, flags);
  dev_info(dma_dev->dev, "[+] dma_callback()");
  // printk("dma_index = %d\n", dma_dev->trans_index);
  // int i;
  // for (i = 0; i < 10; i++) {
  //   printk("Byte at index %d is 0x%02x\n", i, dst[i]);
  // }
}

static int dma_transfer(struct dma_char_dev* dma_dev, dma_addr_t dma_phy_src, dma_addr_t dma_phy_dst, size_t len) {
  int res;
  struct dma_device* dma_phy_dev     = dma_dev->chan->device;
  enum dma_ctrl_flags flags          = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
  struct dma_async_tx_descriptor* tx = NULL;

  dev_info(dma_dev->dev, "[ ] DMA transfer");
  dev_info(dma_dev->dev, "[i] source address      = %p", (void*)dma_phy_src);
  dev_info(dma_dev->dev, "[i] destination address = %p", (void*)dma_phy_dst);
  dev_info(dma_dev->dev, "[i] transfer length     = %d bytes", (int)len);  // dma_phy_dev->copy_align: 0, 未启用
  dev_info(dma_dev->dev, "[i] aligned transfer?   = %d", is_dma_copy_aligned(dma_phy_dev, dma_phy_src, dma_phy_dst, len));

  dev_info(dma_phy_dev->dev, "[ ] device_prep_dma_memcpy()");
  tx = dma_phy_dev->device_prep_dma_memcpy(dma_dev->chan, dma_phy_dst, dma_phy_src, len, flags);
  if (!tx) {
    dev_err(dma_phy_dev->dev, "[x] device_prep_dma_memcpy()");
    res = -ENOMEM;
    goto err_tx;
  }
  dev_info(dma_phy_dev->dev, "[+] device_prep_dma_memcpy()");
  tx->callback       = dma_callback;
  tx->callback_param = dma_dev;

  // Submit DMA transfer
  dev_info(dma_phy_dev->dev, "[ ] dmaengine_submit()");
  dma_dev->cookie = dmaengine_submit(tx);
  res             = dma_submit_error(dma_dev->cookie);
  if (res) {
    dev_err(dma_phy_dev->dev, "[x] dmaengine_submit()");
    goto err_cookie;
  }
  dev_info(dma_phy_dev->dev, "[+] dmaengine_submit()");

  // Start pending transfers
  dma_async_issue_pending(dma_dev->chan);

err_cookie:
  dmaengine_desc_free(tx);

err_tx:
  dev_info(dma_phy_dev->dev, "[+] DMA transfer");
  return res;
}

static int dma_dev_init(struct platform_device* pdev) {
  int res;
  struct dma_chan* chan = NULL;
  const char* chan_name = "";  // e.g. "ps-dma"

  // Retrieve custom device info using platform device (private data)
  struct dma_char_dev* dma_dev = platform_get_drvdata(pdev);
  // Get device tree node pointer from platform device structure
  struct device_node* node = pdev->dev.of_node;

  dev_info(&pdev->dev, "[ ] of_property_read_string()");
  res = of_property_read_string(node, "dma-names", &chan_name);
  if (res) {
    dev_err(&pdev->dev, "[x] of_property_read_string()");
    return res;
  }
  dev_info(&pdev->dev, "[+] of_property_read_string() -> %s", chan_name);

  dev_info(&pdev->dev, "[ ] dma_request_slave_channel()");
  chan = dma_request_slave_channel(&pdev->dev, chan_name);
  if (IS_ERR(chan) || (!chan)) {
    dev_err(&pdev->dev, "[x] dma_request_slave_channel()");
    // return PTR_ERR(chan);
    return -EBUSY;
  }
  dev_info(&pdev->dev, "[+] dma_request_slave_channel() -> %s", chan_name);
  dev_info(chan->device->dev, "[i] dma_request_slave_channel()");

  // dma_dev->phy_dev = &pdev->dev;
  dma_dev->chan = chan;

  // dev_info(&pdev->dev, "[ ] dma_alloc_coherent()");
  // src = dma_alloc_coherent(&pdev->dev, MAX_SIZE, &dma_src, GFP_KERNEL);
  // if (NULL == src) {
  //   dev_info(&pdev->dev, "src [x] dma_alloc_coherent()");
  //   return -ENOMEM;
  // }
  // dst = dma_alloc_coherent(&pdev->dev, MAX_SIZE, &dma_dst, GFP_KERNEL);
  // if (NULL == dst) {
  //   dev_info(&pdev->dev, "dst [x] dma_alloc_coherent()");
  //   res = -ENOMEM;
  //   goto err_alloc_dst;
  // }
  // printk(KERN_INFO "%s [+] dma_alloc_coherent()\n", DEVICE_NAME);

  // 给源地址和目标地址一个初值
  // memset(src, 0x5A, MAX_SIZE);
  // memset(dst, 0xA5, MAX_SIZE);
  // int i;
  // for (i = 0; i < 10; i++) {
  //   printk("Byte at index %d is 0x%02x\n", i, dst[i]);
  // }

  dev_info(&pdev->dev, "[+] dma_dev_init()");
  return 0;

  // err_alloc_dst:
  // dma_free_coherent(NULL, MAX_SIZE, src, dma_src);
  return res;
}

static void dma_dev_exit(struct platform_device* pdev) {
  struct dma_char_dev* dma_dev = platform_get_drvdata(pdev);
  dev_info(&pdev->dev, "[ ] dma_dev_exit()");
  dma_release_channel(dma_dev->chan);
  dev_info(&pdev->dev, "[+] dma_dev_exit()");
}

static int dma_open(struct inode* inode_p, struct file* fp) {
  struct dma_char_dev* dma_dev = container_of(inode_p->i_cdev, struct dma_char_dev, cdev);
  fp->private_data             = dma_dev;
  dev_info(dma_dev->dev, "[ ] open()");
  dev_info(dma_dev->dev, "[+] open()");
  return 0;
}

static int dma_release(struct inode* inode_p, struct file* fp) {
  struct dma_char_dev* dma_dev = container_of(inode_p->i_cdev, struct dma_char_dev, cdev);
  fp->private_data             = NULL;
  dev_info(dma_dev->dev, "[ ] release()");
  dev_info(dma_dev->dev, "[+] release()");
  return 0;
}

static long dma_ioctl(struct file* fp, unsigned int cmd, unsigned long arg) {
  struct dma_char_dev* dma_dev = (struct dma_char_dev*)fp->private_data;
  struct dma_vm_list *vm_list, *backup;
  struct dma_proxy_token token;
  // struct platform_device* pdev = dma_dev->pdev;
  int res = 0;

  dev_info(dma_dev->dev, "[ ] ioctl()");

  switch (cmd) {
    case DMA_MEM2HW:
      dev_info(dma_dev->dev, "[x] DMA_MEM2HW() not support");
      break;
    case DMA_MEM2MEM:
      mutex_lock(&dma_dev->mutex);
      dev_info(dma_dev->dev, "[ ] copy_from_user()");
      res = copy_from_user(&token, (void*)arg, sizeof token);
      if (res) {
        dev_err(dma_dev->dev, "[X] copy_from_user()");
        return -ENOMEM;
      }
      dev_info(dma_dev->dev, "[+] copy_from_user() -> token");
      dev_info(dma_dev->dev, "[i] DMA from memory to memory");
      dev_info(dma_dev->dev, "[i] DMA -> src address   = %p", token.src_base);
      dev_info(dma_dev->dev, "[i] DMA -> src offset    = %p", (void*)token.src_offset);
      dev_info(dma_dev->dev, "[i] DMA -> dst address   = %p", token.dst_base);
      dev_info(dma_dev->dev, "[i] DMA -> dst offset    = %p", (void*)token.dst_offset);
      dev_info(dma_dev->dev, "[i] DMA -> transfer size = %d bytes", (int)token.size);

      // 遍历 Linux 内核中双向链表（list_head）
      // vm_list：当前遍历到的链表元素
      // backup：指向下一个链表元素
      // &dma_dev->head：链表的起始地址
      dma_addr_t dma_phy_src = 0;
      dma_addr_t dma_phy_dst = 0;
      list_for_each_entry_safe(vm_list, backup, &dma_dev->head, list) {
        // TODO: 看看为啥要判断pid
        // addr_u   = (void*)vma->vm_start;(用户空间传来的虚拟地址)
        // src_base = (用户空间的传输请求的虚拟地址)
        if ((vm_list->pid == current->pid) && (vm_list->addr_u == token.src_base)) {
          // Memory check
          dev_info(dma_dev->dev, "[+] DMA -> src %p match\n", token.src_base);
          if (vm_list->size < (token.src_offset + token.size)) {
            dev_err(dma_dev->dev, "[x] DMA -> src requested transfer out of memory region");
            res = -EINVAL;
            break;
          }
          dma_phy_src = vm_list->addr_p;
        }
        if ((vm_list->pid == current->pid) && (vm_list->addr_u == token.dst_base)) {
          dev_info(dma_dev->dev, "[+] DMA -> dst %p match\n", token.dst_base);
          if (vm_list->size < (token.dst_offset + token.size)) {
            dev_err(dma_dev->dev, "[x] DMA -> dst requested transfer out of memory region");
            res = -EINVAL;
            break;
          }
          dma_phy_dst = vm_list->addr_p;
        }
        if (dma_phy_src && dma_phy_dst) {
          break;
        }
      }
      if (!dma_phy_src) {
        dev_err(dma_dev->dev, "[x] DMA -> src address not found or invalid\n");
        res = -EINVAL;
      }
      if (!dma_phy_dst) {
        dev_err(dma_dev->dev, "[x] DMA -> dst address not found or invalid\n");
        res = -EINVAL;
      }
      dma_transfer(dma_dev, dma_phy_src + token.src_offset, dma_phy_dst + token.dst_offset, token.size);
      break;
    default:
      dev_err(dma_dev->dev, "[i] ioctl() -> command %x does not exist", cmd);
      res = -ENOTTY;
  }

  dev_info(dma_dev->dev, "[+] ioctl()");
  return res;

  // // long copy_from_user(void *to, const void __user *from, unsigned long len);
  // // 内核目标地址、用户空间源地址、要复制的字节数
  // dev_info(dma_dev->dev, "[ ] dma_ioctl()");
  // dev_info(dma_dev->dev, "[ ] copy_from_user()");
  // res = copy_from_user(&dma_dev->trans_index, (int*)arg, sizeof(dma_dev->trans_index));
  // if (res) {
  //   dev_info(dma_dev->dev, "[x] copy_from_user()");
  //   return -EINVAL;
  // }
  // dev_info(dma_dev->dev, "[+] copy_from_user()");
  // // 更改io调用，不同的情况trans不同的东西
  // switch (cmd) {
  //   case 10:
  //     dma_transfer(dma_dev);
  //     break;
  // }
  return 0;
}

static unsigned int dma_poll(struct file* fp, struct poll_table_struct* wait) {
  struct dma_char_dev* dma_dev = fp->private_data;
  unsigned long flags;
  unsigned int ret;
  enum dma_status status;

  dev_info(dma_dev->dev, "[ ] poll()");
  poll_wait(fp, &dma_dev->queue, wait);
  spin_lock_irqsave(&dma_dev->lock, flags);
  ret    = 0;
  status = dma_async_is_tx_complete(dma_dev->chan, dma_dev->cookie, NULL, NULL);
  if (status == DMA_COMPLETE) {
    dev_info(dma_dev->dev, "[i] poll() : ret |= POLLDMA");
    ret |= POLLDMA;
    // Release mutex (acquired in artico3_ioctl() before DMA transfer)
    mutex_unlock(&dma_dev->mutex);
  }
  spin_unlock_irqrestore(&dma_dev->lock, flags);

  dev_info(dma_dev->dev, "[+] poll()");
  return ret;
}

static void vir_mmap_dma_close(struct vm_area_struct* vma) {
  struct dma_vm_list* token      = vma->vm_private_data;
  struct dma_char_dev* dma_dev   = token->dma_dev;
  struct dma_device* dma_phy_dev = dma_dev->chan->device;

  dev_info(dma_dev->dev, "[ ] munmap()");
  dev_info(dma_dev->dev, "[i] vma->vm_start = %p", (void*)vma->vm_start);
  dev_info(dma_dev->dev, "[i] vma->vm_end   = %p", (void*)vma->vm_end);
  dev_info(dma_dev->dev, "[i] vma size      = %ld bytes", vma->vm_end - vma->vm_start);

  dma_free_coherent(dma_phy_dev->dev, token->size, token->addr_k, token->addr_p);

  // Critical section: remove region from dynamic list
  mutex_lock(&dma_dev->mutex);
  list_del(&token->list);
  mutex_unlock(&dma_dev->mutex);

  kfree(token);
  vma->vm_private_data = NULL;

  dev_info(dma_dev->dev, "[+] munmap()");
}

static struct vm_operations_struct vir_mmap_dma_ops = {
    .close = vir_mmap_dma_close,
};

static int vir_mmap_dma(struct file* fp, struct vm_area_struct* vma) {
  struct dma_char_dev* dma_dev   = (struct dma_char_dev*)fp->private_data;
  struct dma_device* dma_phy_dev = dma_dev->chan->device;

  int res        = 0;
  void* addr_vir = NULL;
  dma_addr_t addr_phy;
  struct dma_vm_list* token = NULL;

  dev_info(dma_dev->dev, "[ ] mmap_dma()");
  dev_info(dma_dev->dev, "[i] vma->vm_start = %p", (void*)vma->vm_start);
  dev_info(dma_dev->dev, "[i] vma->vm_end   = %p", (void*)vma->vm_end);
  dev_info(dma_dev->dev, "[i] vma size      = %ld bytes", vma->vm_end - vma->vm_start);

  dev_info(dma_phy_dev->dev, "[ ] dma_alloc_coherent()");
  addr_vir = dma_alloc_coherent(dma_phy_dev->dev, vma->vm_end - vma->vm_start, &addr_phy, GFP_KERNEL);
  if (IS_ERR(addr_vir)) {
    dev_err(dma_phy_dev->dev, "[x] dma_alloc_coherent()");
    return PTR_ERR(addr_vir);
  }
  dev_info(dma_phy_dev->dev, "[+] dma_alloc_coherent()");
  dev_info(dma_phy_dev->dev, "[i] dma_alloc_coherent() -> %p (virtual)", addr_vir);
  dev_info(dma_phy_dev->dev, "[i] dma_alloc_coherent() -> %p (physical)", (void*)addr_phy);
  dev_info(dma_phy_dev->dev, "[i] dma_alloc_coherent() -> %ld bytes", vma->vm_end - vma->vm_start);

  printk("dma malloc user virtual address   -> %p\n", vma->vm_start);
  printk("dma malloc kernel virtual address -> %p\n", addr_vir);
  printk("dma malloc physical address       -> %p\n", addr_phy);

  dev_info(dma_phy_dev->dev, "[ ] dma_mmap_coherent()");
  // 当设备和 CPU 之间共享内存时，它们的视图应该是一致的，即没有缓存不一致的问题
  res = dma_mmap_coherent(dma_phy_dev->dev, vma, addr_vir, addr_phy, vma->vm_end - vma->vm_start);
  if (res) {
    dev_err(dma_phy_dev->dev, "[x] dma_mmap_coherent() %d", res);
    goto err_dma_mmap;
  }
  dev_info(dma_phy_dev->dev, "[+] dma_mmap_coherent()");

  dev_info(dma_dev->dev, "[ ] kzalloc() -> token");
  token = kzalloc(sizeof *token, GFP_KERNEL);
  if (!token) {
    dev_err(dma_dev->dev, "[x] kzalloc() -> token");
    res = -ENOMEM;
    goto err_kmalloc_token;
  }
  dev_info(dma_dev->dev, "[+] kzalloc() -> token");

  token->dma_dev = dma_dev;
  token->addr_u  = (void*)vma->vm_start;
  token->addr_k  = addr_vir;
  token->addr_p  = addr_phy;
  token->size    = vma->vm_end - vma->vm_start;
  token->pid     = current->pid;
  INIT_LIST_HEAD(&token->list);

  // Critical section: add new region to dynamic list
  mutex_lock(&dma_dev->mutex);
  list_add(&token->list, &dma_dev->head);
  mutex_unlock(&dma_dev->mutex);

  // Set virtual memory structure operations
  vma->vm_ops = &vir_mmap_dma_ops;

  // Pass data to virtual memory structure (private data) to enable proper cleanup
  vma->vm_private_data = token;

  dev_info(dma_dev->dev, "[+] mmap_dma()");
  return 0;

err_kmalloc_token:

err_dma_mmap:
  dma_free_coherent(dma_phy_dev->dev, vma->vm_end - vma->vm_start, addr_vir, addr_phy);
  return res;
}

// mmap 函数用于将物理内存（设备的内存）映射到用户空间的虚拟地址空间，这样用户程序就可以通过指针直接访问设备内存
// void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
// addr不传递的话vma->start是系统自动分配的
// offset == 0 -> mem_idx = 0 | offset = sysconf(_SC_PAGESIZE) -> mem_idx = 1
static int dma_mmap(struct file* fp, struct vm_area_struct* vma) {
  struct dma_char_dev* dma_dev = (struct dma_char_dev*)fp->private_data;
  int res                      = 0;
  int mem_idx                  = (int)vma->vm_pgoff;
  dev_info(dma_dev->dev, "[ ] mmap()");
  dev_info(dma_dev->dev, "[i] memory index : %d", mem_idx);
  switch (mem_idx) {
    case 0:
      dev_info(dma_dev->dev, "[i] hardware map");
      // res = artico3_mmap_hw(fp, vma);
      break;
    case 1:
      dev_info(dma_dev->dev, "[i] vmem map");
      vma->vm_pgoff = 0;                      // Trick to avoid errors on dma_mmap_coherent()
      res           = vir_mmap_dma(fp, vma);  // sysconf(_SC_PAGESIZE)
      break;
    default:
      dev_info(dma_dev->dev, "[i] YOU SHOULD NOT BE HERE");
      res = -EINVAL;
      break;
  }
  dev_info(dma_dev->dev, "[+] mmap()");

  return res;
}

static struct file_operations dma_fops = {
    .owner          = THIS_MODULE,
    .open           = dma_open,
    .release        = dma_release,
    .unlocked_ioctl = dma_ioctl,
    .poll           = dma_poll,
    .mmap           = dma_mmap,
};

static int dma_cdev_create(struct platform_device* pdev) {
  int res;
  struct dma_char_dev* dma_dev = platform_get_drvdata(pdev);
  dev_info(&pdev->dev, "[ ] dma_cdev_create()");
  // res = alloc_chrdev_region(&dma_dev->devid, 0, 1, DEVICE_NAME);
  dma_dev->devid = MKDEV(MAJOR(dma_devid), 0);

  dev_info(&pdev->dev, "[ ] cdev_add()");
  cdev_init(&dma_dev->cdev, &dma_fops);
  res = cdev_add(&dma_dev->cdev, dma_dev->devid, 1);
  if (res) {
    dev_err(&pdev->dev, "[x] cdev_add() -> %d:%d", MAJOR(dma_dev->devid), MINOR(dma_dev->devid));
    return res;
  }
  dev_info(&pdev->dev, "[+] cdev_add() -> %d:%d", MAJOR(dma_dev->devid), MINOR(dma_dev->devid));

  dev_info(&pdev->dev, "[ ] device_create()");
  dma_dev->dev = device_create(dma_class, &pdev->dev, dma_dev->devid, dma_dev, "%s", DEVICE_NAME);
  if (IS_ERR(dma_dev->dev)) {
    dev_err(&pdev->dev, "[x] device_create() -> %d:%d", MAJOR(dma_dev->devid), MINOR(dma_dev->devid));
    res = PTR_ERR(dma_dev->dev);
    goto err_device;
  }
  dev_info(&pdev->dev, "[+] device_create() -> %d:%d", MAJOR(dma_dev->devid), MINOR(dma_dev->devid));
  dev_info(dma_dev->dev, "[i] device_create()");

  dev_info(&pdev->dev, "[+] dma_cdev_create()");
  return 0;

err_device:
  cdev_del(&dma_dev->cdev);

  return res;
}

static void dma_cdev_destroy(struct platform_device* pdev) {

  struct dma_char_dev* dma_dev = platform_get_drvdata(pdev);

  dev_info(&pdev->dev, "[ ] dma_cdev_destory()");

  dev_info(dma_dev->dev, "[i] dma_cdev_destory()");

  device_destroy(dma_class, dma_dev->devid);
  cdev_del(&dma_dev->cdev);

  dev_info(&pdev->dev, "[+] dma_cdev_destory()");
}

static int dma_cdev_init(void) {
  int res;

  printk(KERN_INFO "%s [ ] dma_cdev_init()\n", DEVICE_NAME);

  // Dynamically allocate major number for char device
  printk(KERN_INFO "%s [ ] alloc_chrdev_region()\n", DEVICE_NAME);
  res = alloc_chrdev_region(&dma_devid, 0, 1, DEVICE_NAME);
  if (res < 0) {
    printk(KERN_ERR "%s [x] alloc_chrdev_region()\n", DEVICE_NAME);
    return res;
  }
  printk(KERN_INFO "%s [+] alloc_chrdev_region() -> %d:%d-%d\n", DEVICE_NAME, MAJOR(dma_devid), MINOR(dma_devid), MINOR(dma_devid));

  // Create sysfs class
  printk(KERN_INFO "%s [ ] class_create()\n", DEVICE_NAME);
  dma_class = class_create(THIS_MODULE, DEVICE_NAME);
  if (IS_ERR(dma_class)) {
    printk(KERN_ERR "%s [x] class_create()\n", DEVICE_NAME);
    res = PTR_ERR(dma_class);
    goto err_class;
  }
  printk(KERN_INFO "%s [+] class_create() -> /sys/class/%s\n", DEVICE_NAME, DEVICE_NAME);

  printk(KERN_INFO "%s [+] dma_cdev_init()\n", DEVICE_NAME);
  return 0;

err_class:
  unregister_chrdev_region(dma_devid, 1);
  return res;
}

static void dma_cdev_exit(void) {
  printk(KERN_INFO "%s [ ] dma_cdev_exit()\n", DEVICE_NAME);
  class_destroy(dma_class);
  unregister_chrdev_region(dma_devid, 1);
  printk(KERN_INFO "%s [+] dma_cdev_exit()\n", DEVICE_NAME);
}

static int dma_dev_probe(struct platform_device* pdev) {
  int res;
  struct dma_char_dev* dma_dev = NULL;
  dev_info(&pdev->dev, "[ ] dma_dev_probe()");

  dev_info(&pdev->dev, "[ ] kzalloc() -> %s", DEVICE_NAME);
  dma_dev = (struct dma_char_dev*)kzalloc(sizeof(struct dma_char_dev), GFP_KERNEL);
  if (!dma_dev) {
    dev_err(&pdev->dev, "[x] kzalloc() -> %s", DEVICE_NAME);
    return -ENOMEM;
  }
  dev_info(&pdev->dev, "[+] kzalloc() -> %s", DEVICE_NAME);

  INIT_LIST_HEAD(&dma_dev->head);
  dma_dev->pdev = pdev;

  // dev_set_drvdata(dev, dma_dev)
  platform_set_drvdata(pdev, dma_dev);

  res = dma_dev_init(pdev);
  if (res) {
    dev_err(&pdev->dev, "[x] dma_dev_init()");
    goto err_dma_init;
  }

  res = dma_cdev_create(pdev);
  if (res) {
    dev_err(&pdev->dev, "[x] dma_cdev_create()");
    goto err_cdev_create;
  }

  mutex_init(&dma_dev->mutex);
  spin_lock_init(&dma_dev->lock);
  init_waitqueue_head(&dma_dev->queue);

  dev_info(&pdev->dev, "[+] dma_dev_probe()");
  return 0;

  // err_ioremap:
  //   dma_cdev_destroy(pdev);

err_cdev_create:
  dma_dev_exit(pdev);

err_dma_init:
  kfree(dma_dev);
  return res;
}

static int dma_dev_remove(struct platform_device* pdev) {

  // Retrieve custom device info using platform device (private data)
  struct dma_char_dev* dma_dev = platform_get_drvdata(pdev);

  dev_info(&pdev->dev, "[ ] dma_dev_remove()");
  mutex_destroy(&dma_dev->mutex);
  dma_cdev_destroy(pdev);
  dma_dev_exit(pdev);
  // dma_free_coherent(&pdev->dev, MAX_SIZE, dst, dma_dst);
  // dma_free_coherent(&pdev->dev, MAX_SIZE, src, dma_src);
  kfree(dma_dev);

  dev_info(&pdev->dev, "[+] dma_dev_remove()");
  return 0;
}

static const struct of_device_id dma_proxy_of_ids[] = {
    {
        .compatible = "xlnx,dma_proxy",
    },
    {}};

static struct platform_driver dma_driver = {
    .driver = {
        .name           = "dma_driver",
        .owner          = THIS_MODULE,
        .of_match_table = dma_proxy_of_ids,
    },
    .probe  = dma_dev_probe,
    .remove = dma_dev_remove,
};

static int __init dma_init(void) {
  int res;
  printk(KERN_INFO "%s [ ] dma_dev_init()\n", DEVICE_NAME);
  res = dma_cdev_init();
  if (res) {
    printk(KERN_ERR "%s [x] dma_cdev_init()\n", DEVICE_NAME);
    return res;
  }
  printk(KERN_INFO "%s [ ] platform_driver_register()\n", DEVICE_NAME);
  res = platform_driver_register(&dma_driver);
  if (res) {
    printk(KERN_ERR "%s [x] platform_driver_register()\n", DEVICE_NAME);
    goto err_driver;
  }
  printk(KERN_INFO "%s [+] platform_driver_register()\n", DEVICE_NAME);
  printk(KERN_INFO "%s [+] dma_dev_init()\n", DEVICE_NAME);

  // todo: 改到DMA的init里面去
  return 0;

err_driver:
  dma_cdev_exit();
  return res;
}

static void __exit dma_exit(void) {
  printk(KERN_INFO "%s [ ] dma_dev_exit()\n", DEVICE_NAME);
  platform_driver_unregister(&dma_driver);
  dma_cdev_exit();
  printk(KERN_INFO "%s [+] dma_dev_exit()\n", DEVICE_NAME);
}

module_init(dma_init);
module_exit(dma_exit);

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION("DMA Proxy Prototype");
MODULE_LICENSE("GPL v2");