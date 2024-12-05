#include <stdio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include "i2c/i2c.h"

/**
  * @brief  i2c 写
  * @param  fd：i2c设备句柄
  * @param  regAddr：寄存器地址
  * @param  val：需要写入的值
  * @retval 0：写入成功
  * @retval -1：写入失败
  */
int i2c_write(int fd, unsigned char regAddr, unsigned char val)
{
    int cnt; // 写入失败后，重复写入的次数
    unsigned char data[2]; // data[0]为寄存器地址，data[1]为需要写入的值
    data[0] = regAddr;
    data[1] = val;
    for(cnt=5; cnt>0; cnt--)
    {
        if(write(fd, data, 2) == 2)
            return 0; // 写入成功
    }
    return -1; // 写入失败
}
/**
  * @brief  i2c 读
  * @param  fd：i2c设备句柄
  * @param  regAddr：寄存器地址
  * @param  val：读取到数据保存的地方
  * @retval 0：读取成功
  * @retval -1：读取失败
  */
int i2c_read(int fd, unsigned char regAddr, unsigned char * val)
{
    int cnt; // 读取失败后，重新读取的次数
    for(cnt=5; cnt>0; cnt--)
    {
        if(write(fd, &regAddr, 1) == 1)
        {
            if(read(fd, val, 1) == 1)
                return 0;
        }
    }
    return -1;
}
/*

    reg_addr: 16 bits register address
    flags: 0 only write
           1 read register data
    val_buf:
        if flag==0
            val is send data buf
        if flag==1
            val is recv data buf
    cnt:the counter you want to read or write
*/

int i2c_transfer_16bits
(const char *i2c_dev,unsigned char i2c_dev_addr,unsigned long reg_addr,int flags,unsigned char *val_buf,size_t cnt)
{
    int i=0;
    int flag=flags;
    int file=-1;
    int nmsgs=0;
    int nmsgs_sent=0;
    struct i2c_rdwr_ioctl_data rdwr;
    //struct i2c_msg msgs[I2C_RDWR_IOCTL_MAX_MSGS];
    struct i2c_msg msgs[2];
    char *buf;

    for (i = 0; i < 2; i++)
        msgs[i].buf = NULL;

    file=open(i2c_dev,O_RDWR);
    if(file<0){
        printf("open %s failed\n",i2c_dev);
        return -1;
    }

    if(flags==0){
        msgs[nmsgs].addr = i2c_dev_addr;
        msgs[nmsgs].flags = 0;/*write flag*/
        msgs[nmsgs].len = 2+cnt;
        msgs[nmsgs].buf=malloc(msgs[nmsgs].len);/*buf alloc according to len*/
        msgs[nmsgs].buf[0]=(unsigned char)(reg_addr>>8);
        msgs[nmsgs].buf[1]=(unsigned char)(reg_addr&0x00ff);
        if(cnt==0||cnt>2){
            printf("cnt[%d] is error\n",cnt);
            goto err_out;
        }
        if(cnt==1)
            msgs[nmsgs].buf[2]=val_buf[0];
        if(cnt==2){
            msgs[nmsgs].buf[2]=val_buf[0];
            msgs[nmsgs].buf[3]=val_buf[1];
        }
        goto write_to_reg;
    }

    if(flags==1){
        msgs[nmsgs].addr = i2c_dev_addr;
        msgs[nmsgs].flags = 0;/*write flag*/
        msgs[nmsgs].len = 2;
        msgs[nmsgs].buf=malloc(msgs[nmsgs].len);/*buf alloc according to len*/
        msgs[nmsgs].buf[0]=(unsigned char)(reg_addr>>8);
        msgs[nmsgs].buf[1]=(unsigned char)(reg_addr&0x00ff);
    }

    nmsgs++;

/*perpare read buf*/
    msgs[nmsgs].addr = i2c_dev_addr;
    msgs[nmsgs].flags = 1; /*read flag*/
    msgs[nmsgs].len=cnt;
    buf=malloc(msgs[nmsgs].len);/*buf alloc according to len*/
    memset(buf,0,cnt);
    msgs[nmsgs].buf=buf;
/*perpare read buf*/

write_to_reg:
    rdwr.msgs = msgs;
    rdwr.nmsgs = nmsgs+1;/*if read rdwr.nmsgs==2,eles if write rdwr.nmsgs==1*/

#if 0
    int k=0;
    int f=0;

    printf("%s(%d) nmsgs %d\n",__func__,__LINE__,nmsgs);
    for(f=0;f<rdwr.nmsgs;f++){
        printf("%s(%d) addr %d\n",__func__,__LINE__,msgs[f].addr);
        printf("%s(%d) flags %d\n",__func__,__LINE__,msgs[f].flags);
        printf("%s(%d) len %d\n",__func__,__LINE__,msgs[f].len);
        for(k=0;k<msgs[f].len;k++)
            printf("%s(%d) buf 0x%02x \n",__func__,__LINE__,msgs[f].buf[k]);
        printf("\n");
    }
#endif

    nmsgs_sent = ioctl(file, I2C_RDWR, &rdwr);
    if(nmsgs_sent<0)
        goto err_out;
    else if(nmsgs_sent<nmsgs){
        printf("Warning: only %d/%d messages were sent\n", nmsgs_sent, nmsgs);
        goto err_out;
    }

//    printf("flag %d cnt %d\n",flag,cnt);

    if(flag==1){
        for(i=0;i<cnt;i++){
//            printf("msgs[%d].buf[%d] 0x%02x \n",nmsgs,i,msgs[nmsgs].buf[i]);
            val_buf[i]=msgs[nmsgs].buf[i];
            //printf("val_buf[%d] 0x%02x \n",i,val_buf[i]);
            //memcpy(val_buf+i,msgs[nmsgs].buf+i,1);
        }
    }

    close(file);
    for (i = 0; i <= nmsgs; i++)
        free(msgs[i].buf);

    return 0;


err_out:
    close(file);
    for (i = 0; i <= nmsgs; i++)
        free(msgs[i].buf);
    return -1;
}

void i2c1_8bit_send(unsigned char i2c_addr, unsigned long reg_addr, unsigned char send_data)
{
    const char i2c_path[50] = "/dev/i2c-1";
    int i2cFd;
    i2cFd = open(i2c_path, O_RDWR);
    if(i2cFd < 0)
    {
        printf("Can't open\n");
        exit(1);
    }
    if(ioctl(i2cFd, I2C_SLAVE, i2c_addr) < 0)
    {
        printf("fail to set i2c device slave address!");
        close(i2cFd);
        return -1;
    }
    i2c_write(i2cFd, reg_addr, send_data);
}
unsigned char i2c1_8bit_recv(unsigned char i2c_addr, unsigned long reg_addr)
{
    const char i2c_path[50] = "/dev/i2c-1";
    unsigned char i2c_read_val;
    int i2cFd;
    i2cFd = open(i2c_path, O_RDWR);
    if(i2cFd < 0)
    {
        printf("Can't open!\n");
        exit(1);
    }
    if(ioctl(i2cFd, I2C_SLAVE, i2c_addr) < 0)
    {
        printf("fail to set i2c device slave address!");
        close(i2cFd);
        return -1;
    }
    i2c_read_val = i2c_read(i2cFd, i2c_addr, reg_addr);
    return i2c_read_val;
}

void i2c0_16bit_send(unsigned char i2c_addr, unsigned long reg_addr, unsigned char send_data)
{
    const char i2c_path[50] = "/dev/i2c-0";
    int flags = 0;
    unsigned char val_buf[1];
    size_t cnt = 1;
    val_buf[0] = send_data;
    i2c_transfer_16bits(i2c_path, i2c_addr, reg_addr, flags, val_buf, cnt);
}

unsigned char i2c0_16bit_recv(unsigned char i2c_addr, unsigned long reg_addr)
{
    const char i2c_path[50] = "/dev/i2c-0";
    int flags = 1;
    unsigned char val_buf[1];
    unsigned char recv_data;
    size_t cnt = 1;
    i2c_transfer_16bits(i2c_path, i2c_addr, reg_addr, flags, val_buf, cnt);
    recv_data = val_buf[0];
    return recv_data;
}

void i2c1_16bit_send(unsigned char i2c_addr, unsigned long reg_addr, unsigned char send_data)
{
    const char i2c_path[50] = "/dev/i2c-1";
    int flags = 0;
    unsigned char val_buf[1];
    size_t cnt = 1;
    val_buf[0] = send_data;
    i2c_transfer_16bits(i2c_path, i2c_addr, reg_addr, flags, val_buf, cnt);
}

unsigned char i2c1_16bit_recv(unsigned char i2c_addr, unsigned long reg_addr)
{
    const char i2c_path[50] = "/dev/i2c-1";
    int flags = 1;
    unsigned char val_buf[1];
    unsigned char recv_data;
    size_t cnt = 1;
    i2c_transfer_16bits(i2c_path, i2c_addr, reg_addr, flags, val_buf, cnt);
    recv_data = val_buf[0];
    return recv_data;
}
