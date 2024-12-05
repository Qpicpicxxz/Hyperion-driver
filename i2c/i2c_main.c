#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char* argv[]) {
  int fd, ret = 0;
  char* filename;
  unsigned char w_buf[256] = {0};
  unsigned char r_buf[256] = {0};
  unsigned char s_buf[17]  = {0};

  // 初始化写入缓冲区
  for (int i = 0; i < 256; i++) {
    w_buf[i] = i;
  }

  // // 检查命令行参数
  // if (argc != 2) {
  //   printf("Error Usage: please specify a file name.\n");
  //   return -1;
  // }

  // filename = argv[1];
  fd = open("/dev/ax_eeprom", O_RDWR);
  if (fd < 0) {
    printf("file /dev/ax_eeprom open failed\r\n");
    return -1;
  }

  // 打开文件
  // fd = open(filename, O_RDWR);
  // if (fd < 0) {
  //   printf("File %s open failed\n", filename);
  //   return -1;
  // }

  // 写入数据到文件
  for (int i = 0; i < 16; i++) {
    s_buf[0] = i * 16;                        // 设置偏移地址
    memcpy(&s_buf[1], &w_buf[s_buf[0]], 16);  // 从写缓冲区复制16字节数据到s_buf
    ret = write(fd, s_buf, 17);               // 写入17字节（1字节地址 + 16字节数据）
    if (ret < 0) {
      printf("EEPROM write error at block %d\n", i);
      close(fd);
      return -1;
    }
  }

  // 读取数据并校验
  lseek(fd, 0, SEEK_SET);      // 将文件指针重置到起始位置
  ret = read(fd, r_buf, 256);  // 读取256字节数据
  if (ret < 0) {
    printf("EEPROM read error\n");
    close(fd);
    return -1;
  }

  // 校验数据
  if (!memcmp(r_buf, w_buf, 256)) {
    printf("EEPROM test OK\n");
  } else {
    printf("EEPROM test FAILED\n");
  }

  close(fd);
  return 0;
}
