#include <fcntl.h> // open, O_RDWR
#include <unistd.h> // close, read, write

#include <sys/ioctl.h> // ioctl

#include <linux/i2c-dev.h>

#include <cstdio> // snprintf

#include <iostream>
#include <bitset>
#include <vector>

#include "i2c_dev_reg.hh"

auto main() -> int {
constexpr int addr = 0x20; // I2C slave addr

/* Construct I2C slave device file descriptor */
// TODO: Use std::string
int adapter_num = 0;
char fp[13]; // I2C device file path: /dev/i2c-0 to 255
snprintf(fp, sizeof(fp), "/dev/i2c-%d", adapter_num); // The last char is \0 (null character).
int fd = open(fp, O_RDWR);  // file descriptor
ioctl(fd, I2C_SLAVE, addr);

// https://stackoverflow.com/questions/30594966/modern-c-idiom-for-allocating-deallocating-an-i-o-buffer
std::vector<std::byte> buf;

/* Config */
buf = {
   REG_GCONF,
   WRAP_ENABLE | DIRE_LEFT | IPUP_DISABLE | RMOD_X4
};
write(fd, buf.data(), buf.size());

/* Set max count */
buf = {
   REG_CMAXB4,
   // Data is transferred MSB first
   std::byte{0x7F}, // MSB
   std::byte{0xFF},
   std::byte{0xFF},
   std::byte{0xFF}
};
write(fd, buf.data(), buf.size());

/* Set min count */
buf = {
   REG_CMINB4,
   std::byte{0x80},
   std::byte{0x00},
   std::byte{0x00},
   std::byte{0x00}
};
write(fd, buf.data(), buf.size());

/* Set count step */
buf = {
   REG_ISTEPB4,
   std::byte{0x00},
   std::byte{0x00},
   std::byte{0x00},
   std::byte{0x01}
};
write(fd, buf.data(), buf.size());

/* Clear counter */
buf = {
   REG_CVALB4,
   std::byte{0x00},
   std::byte{0x00},
   std::byte{0x00},
   std::byte{0x00}
};
write(fd, buf.data(), buf.size());

for (int32_t res;;) {
   write(fd, &REG_CVALB4, 1);
   read(fd, &res, 4);

   // I2C is big-endian, and assume the arch is little-endian
   res = __builtin_bswap32(res);

   std::cout << res << std::endl;
   std::cout << std::bitset<32>(res) << std::endl;
   usleep(500000);
}

close(fd);
}
