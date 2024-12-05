

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <unistd.h>

#define POLLDMA         0x0001
#define DMA_MEM2HW      10
#define DMA_MEM2MEM     20
#define DMA_KERNEL_TEST 30

#define SRC_MAP_SIZE 1024000
#define DST_MAP_SIZE 1024000
struct dma_proxy_token {
  void* src_base;
  size_t src_offset;
  void* dst_base;
  size_t dst_offset;
  size_t size;
};
#define VIR_MAP sysconf(_SC_PAGESIZE)

int main(int argc, char* argv[]) {

  int fd_dma;
  struct pollfd pfd;
  fd_dma = open("/dev/dma_dev", O_RDWR);
  if (fd_dma < 0) {
    printf("file /dev/dma_dev open failed\r\n");
    return -1;
  }
  pfd.fd     = fd_dma;
  pfd.events = POLLDMA;
  struct dma_proxy_token token;

  // int token;
  // token = 80;

  // void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
  // addr = NULL的时候自动分配

  // char* src = mmap(NULL, SRC_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dma, VIR_MAP);
  // if (src == MAP_FAILED) {
  //   printf("src mmap() failed\n");
  //   return -ENOMEM;
  // }
  // char* dst = mmap(NULL, DST_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dma, VIR_MAP);
  // if (dst == MAP_FAILED) {
  //   printf("dst mmap() failed\n");
  //   return -ENOMEM;
  // }
  // char value = 0;
  // for (size_t i = 0; i < 256; ++i) {
  //   src[i] = value;
  //   value++;
  // }
  // value = 0x98;
  // for (size_t i = 0; i < 256; ++i) {
  //   dst[i] = value;
  // }
  struct timeval start, end;
  gettimeofday(&start, NULL);
  // token.src_base   = src;
  token.src_base   = 0;
  token.src_offset = 0;
  // token.dst_base   = dst;
  token.dst_base   = 0;
  token.dst_offset = 0;
  token.size       = 1024000;
  ioctl(fd_dma, DMA_KERNEL_TEST, &token);
  poll(&pfd, 1, -1);  // poll一个文件+等待无限时间
  // memcpy(dst, src, 1024000);
  gettimeofday(&end, NULL);

  // 打印执行时长
  long seconds  = end.tv_sec - start.tv_sec;
  long useconds = end.tv_usec - start.tv_usec;
  if (useconds < 0) {
    useconds += 1000000;
    seconds--;
  }
  printf("Function execution time: %ld seconds and %ld microseconds\n", seconds, useconds);
  // int res = memcmp(dst, src, 1024000);
  // printf("cmp result: %d\n", res);
  printf("Trans done! 1235\n");
  // for (size_t i = 0; i < 4; ++i) {
  //   printf("0x%x ", dst[i]);
  // }
  printf("\n");
  // res = munmap(src, SRC_MAP_SIZE);
  // if (res == -1) {
  //   printf("src munmap wrong!\n");
  // }
  // res = munmap(dst, DST_MAP_SIZE);
  // if (res == -1) {
  //   printf("dst munmap wrong!\n");
  // }
  close(fd_dma);
  // TODO: munmap
  return 0;
}