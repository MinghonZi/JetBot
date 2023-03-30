#include <fcntl.h> // open, O_RDWR
#include <unistd.h> // close

#include <sys/ioctl.h> // ioctl

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <cstdio> // snprintf
#include <cstring> // memcpy

#include <iostream>
#include <bitset>
#include <vector>
#include <span>
#include <bit> // bit_cast

#include "i2c_dev_reg.hh"

auto i2c_read(
   std::byte byte,
   std::byte const reg,
   int fd,
   int const addr
) -> int {

   i2c_msg msgs[2] {{
      .addr  = static_cast<__u16>(addr),
      .flags = 0, // write
      .len   = 1, // one byte
      .buf   = std::bit_cast<__u8*>(&reg)
   }, {
      .addr  = static_cast<__u16>(addr),
      .flags = I2C_M_RD, // read
      .len   = 1, // one byte
      .buf   = std::bit_cast<__u8*>(&byte)
   }};
   i2c_rdwr_ioctl_data data {
      .msgs = msgs,
      .nmsgs = 2
   };

   return ioctl(fd, I2C_RDWR, &data);;
}

auto i2c_read(
   std::span<std::byte> bytes,
   std::byte const reg,
   int fd,
   int const addr
) -> int {

   i2c_msg msgs[2] {{
      .addr  = static_cast<__u16>(addr),
      .flags = 0, // write
      .len   = 1, // one byte
      .buf   = std::bit_cast<__u8*>(&reg)
   }, {
      .addr  = static_cast<__u16>(addr),
      .flags = I2C_M_RD,
      .len   = static_cast<__u16>(bytes.size()),
      .buf   = std::bit_cast<__u8*>(bytes.data())
   }};
   i2c_rdwr_ioctl_data data {
      .msgs = msgs,
      .nmsgs = 2
   };

   return ioctl(fd, I2C_RDWR, &data);;
}

auto i2c_write(
   std::byte const byte,
   std::byte const reg,
   int fd,
   int const addr
) -> int {
   std::array<std::byte, sizeof(reg) + sizeof(byte)> buf;
   std::memcpy(buf.data(), &reg, sizeof(reg));
   std::memcpy(buf.data() + sizeof(reg), &byte, sizeof(byte));

   i2c_msg msgs[1] {{
      .addr  = static_cast<__u16>(addr),
      .flags = 0, // write
      .len   = static_cast<__u16>(buf.size()),
      .buf   = std::bit_cast<__u8*>(buf.data())
   }};
   i2c_rdwr_ioctl_data data {
      .msgs = msgs,
      .nmsgs = 1
   };

   return ioctl(fd, I2C_RDWR, &data);
}

auto i2c_write(
   std::span<const std::byte> const bytes,
   std::byte const reg,
   int fd,
   int const addr
) -> int {
   std::vector<std::byte> buf(sizeof(reg) + bytes.size());
   std::memcpy(buf.data(), &reg, sizeof(reg));
   std::memcpy(buf.data() + sizeof(reg), bytes.data(), bytes.size());

   i2c_msg msgs[1] {{
      .addr  = static_cast<__u16>(addr),
      .flags = 0, // write
      .len   = static_cast<__u16>(buf.size()),
      .buf   = std::bit_cast<__u8*>(buf.data())
   }};
   i2c_rdwr_ioctl_data data {
      .msgs = msgs,
      .nmsgs = 1
   };

   return ioctl(fd, I2C_RDWR, &data);
}



auto main() -> int {
constexpr int addr = 0x20; // I2C slave addr

uint8_t adapter_num = 0;
char fp[13];
snprintf(fp, sizeof(fp), "/dev/i2c-%d", adapter_num);
int fd = open(fp, O_RDWR);

i2c_write(
   WRAP_ENABLE | DIRE_LEFT | IPUP_DISABLE | RMOD_X4,
   REG_GCONF, fd, addr
);

i2c_write(
   std::array {
      std::byte{0x7F},
      std::byte{0xFF},
      std::byte{0xFF},
      std::byte{0xFF}
   },
   REG_CMAXB4, fd, addr
);

i2c_write(
   std::array {
      std::byte{0x80},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00}
   },
   REG_CMINB4, fd, addr
);

i2c_write(
   std::array {
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x01}
   },
   REG_ISTEPB4, fd, addr
);

i2c_write(
   std::array {
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00}
   },
   REG_CVALB4, fd, addr
);

for (std::array<std::byte, 4> bytes;;) {
   i2c_read(bytes, REG_CVALB4, fd, addr);

   int32_t res = bit_cast<int32_t>(bytes);
   // I2C is big-endian, and assume the arch is little-endian
   res = __builtin_bswap32(std::bit_cast<int32_t>(bytes));

   std::cout << res << std::endl;
   std::cout << std::bitset<32>(res) << std::endl;
   usleep(500000);
}

close(fd);
}
