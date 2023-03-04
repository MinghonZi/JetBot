#include <fcntl.h>  // open O_RDWR
#include <unistd.h>  // close

#include <sys/ioctl.h>  // ioctl

#include <linux/i2c-dev.h>

/** 
Userspace I2C programming library

I2C devices are usually controlled by a kernel driver. Using this library
it is also possible to access all devices on an adapter from userspace and
without the knowledge of Linux kernel internals.

https://github.com/mozilla-b2g/i2c-tools/blob/master/lib/smbus.c
*/
extern "C" {
#include <i2c/smbus.h>
}

#include <iostream>
#include <bitset>

#include "i2c_dev_reg.hh"


auto main() -> int {
constexpr int encoder_i2c_slave_addr = 0x20;

int adapter_num = 0;
char fp[13];
snprintf(fp, sizeof(fp), "/dev/i2c-%d", adapter_num);
int fd = open(fp, O_RDWR);
ioctl(fd, I2C_SLAVE, encoder_i2c_slave_addr);

i2c_smbus_write_i2c_block_data(fd, REG_CVALB4 , I2C_SMBUS_BLOCK_MAX, reinterpret_cast<const unsigned char *>("\x00\x00\x00\x00"));
i2c_smbus_write_i2c_block_data(fd, REG_ISTEPB4, I2C_SMBUS_BLOCK_MAX, reinterpret_cast<const unsigned char *>("\x00\x00\x00\x01"));
i2c_smbus_write_i2c_block_data(fd, REG_CMAXB4 , I2C_SMBUS_BLOCK_MAX, reinterpret_cast<const unsigned char *>("\x7F\xFF\xFF\xFF"));
i2c_smbus_write_i2c_block_data(fd, REG_CMINB4 , I2C_SMBUS_BLOCK_MAX, reinterpret_cast<const unsigned char *>("\x80\x00\x00\x00"));

for (int32_t res;;) {
    i2c_smbus_read_i2c_block_data(fd, REG_CVALB4, I2C_SMBUS_BLOCK_MAX, reinterpret_cast<unsigned char *>(&res));
    res = __builtin_bswap32(res);  // swap byte order (I2C is big-endian)
    std::cout << res << std::endl;
    std::cout << std::bitset<32>(res) << std::endl;
    usleep(100000);
}

close(fd);
}
