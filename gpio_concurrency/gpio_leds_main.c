#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char **argv) {
  int fd;
  char buf;
  int count;
  const char path[50] = "/dev/gpio_leds";
  fd = open(path, O_RDWR);
  if (fd < 0) {
    printf("Can't open file %s\r\n", path);
    return -1;
  }
  if (!strcmp("on",argv[1])) {
    printf("led on\n");
    buf = 0;
    write(fd, &buf, 1);
  } else if (!strcmp("off",argv[1])) {
    printf("led off\n");
    buf = 1;
    write(fd, &buf, 1);
  } else if (!strcmp("blink", argv[1])) {
    printf("led blink\n");
    for (int i = 0; i < 10; i++) {
      buf = 1;
      write(fd, &buf, 1);
      usleep(500000);
      buf = 0;
      write(fd, &buf, 1);
      usleep(500000);
    }
  } else {
    printf("wrong para\n");
    return -2;
  }

  count = 20;
  while(count--) {
    sleep(1);
  }
  close(fd);
  return 0;
}