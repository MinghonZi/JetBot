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

#include <bit>
#include <array>
#include <iostream>
#include <bitset>

#include "i2c_dev_reg.hh"

using namespace std;

auto main() -> int {
constexpr int addr = 0x20; // I2C slave addr

int adapter_num = 0;
char fp[13];
snprintf(fp, sizeof(fp), "/dev/i2c-%d", adapter_num);
int fd = open(fp, O_RDWR);
ioctl(fd, I2C_SLAVE, addr);

/* NOT WORKING */

// i2c_smbus_write_i2c_block_data(fd, bit_cast<__u8>(REG_CMAXB4 ), I2C_SMBUS_BLOCK_MAX, );
// i2c_smbus_write_i2c_block_data(fd, bit_cast<__u8>(REG_CMINB4 ), I2C_SMBUS_BLOCK_MAX, );
// i2c_smbus_write_i2c_block_data(fd, bit_cast<__u8>(REG_ISTEPB4), I2C_SMBUS_BLOCK_MAX, );
// i2c_smbus_write_i2c_block_data(fd, bit_cast<__u8>(REG_CVALB4 ), I2C_SMBUS_BLOCK_MAX, );

for (int32_t res;;) {
   // i2c_smbus_read_i2c_block_data(fd, bit_cast<__u8>(REG_CVALB4), I2C_SMBUS_BLOCK_MAX, bit_cast<__u8*>(&res));

   // I2C is big-endian, and assume the arch is little-endian
   res = __builtin_bswap32(res);

   cout << res << endl;
   cout << bitset<32>(res) << endl;
   usleep(100000);
}

close(fd);
}
