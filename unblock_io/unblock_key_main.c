#include "poll.h"
#include "sys/select.h"
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char* argv[]) {
  int fd_led, fd_key;
  int ret;
  unsigned int key_value = 0;
  char led_value         = 0;
  fd_set readfds;
  struct timeval timeout;

  fd_key = open("/dev/pl_key_irq_unblock_dev", O_RDWR | O_NONBLOCK);
  if (fd_key < 0) {
    printf("file /dev/pl_key_irq_unblock_dev open failed\r\n");
    return -1;
  }
  fd_led = open("/dev/pl_leds_dev", O_RDWR);
  if (fd_key < 0) {
    printf("file /dev/pl_leds_dev open failed\r\n");
    return -1;
  }

  while (1) {
    FD_ZERO(&readfds);
    FD_SET(fd_key, &readfds);
    timeout.tv_sec  = 1;
    timeout.tv_usec = 500000;
    // 如果没有事件发生，select 会进入等待状态（不占用CPU）
    // 即使没有事件发生，这段代码也只会在每次超时后重新进入循环，不会频繁运行
    // 对于驱动那边来说，如果检测到这个file是O_NONBLOCK，就会马上返回
    // 然后有个poll函数会检测key_state是否为1（在key按键后会置1），如果为1就唤醒select
    ret = select(fd_key + 1, &readfds, NULL, NULL, &timeout);
    switch (ret) {
      case 0:
        break;  // timeout
      case -1:
        break;  // err
      default:
        if (FD_ISSET(fd_key, &readfds)) {
          ret = read(fd_key, &key_value, sizeof(key_value));
          if (ret < 0) {
            printf("read failed\r\n");
            break;
          }
          if (1 == key_value) {
            printf("key pushed\n");
            led_value = !led_value;
            ret       = write(fd_led, &led_value, sizeof(led_value));
            if (ret < 0) {
              printf("write failed\r\n");
              break;
            }
          }
        }
    }
  }

  close(fd_key);
  close(fd_led);
  return 0;
}