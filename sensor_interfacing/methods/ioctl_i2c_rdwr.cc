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


template<std::size_t Extent>
auto i2c_write(
   std::span<std::byte, Extent> const bytes,
   std::byte const reg,
   int fd,
   int const addr
) -> int {
   std::vector<std::byte> buf(sizeof(reg) + bytes.size());
   std::memcpy(buf.data(), &reg, sizeof(reg));
   std::memcpy(buf.data() + sizeof(reg), bytes.data(), bytes.size());

   i2c_msg msgs[1] {{
      .addr  = static_cast<__u16>(addr),
      .flags = 0,  // write
      .len   = static_cast<__u16>(buf.size()),
      .buf   = std::bit_cast<__u8*>(buf.data())
   }};
   i2c_rdwr_ioctl_data data {
      .msgs = msgs,
      .nmsgs = 1
   };

   return ioctl(fd, I2C_RDWR, &data);
}

template<std::size_t Extent>
auto i2c_read(
   std::span<std::byte, Extent> buf,
   std::byte const reg,
   int fd,
   int const addr
) -> int {

   i2c_msg msgs[2] {{
      .addr  = static_cast<__u16>(addr),
      .flags = 0,  // write
      .len   = sizeof(reg),  // one byte
      .buf   = std::bit_cast<__u8*>(&reg)
   }, {
      .addr  = static_cast<__u16>(addr),
      .flags = I2C_M_RD,
      .len   = static_cast<__u16>(buf.size()),
      .buf   = std::bit_cast<__u8*>(buf.data())
   }};
   i2c_rdwr_ioctl_data data {
      .msgs = msgs,
      .nmsgs = 2
   };

   return ioctl(fd, I2C_RDWR, &data);;
}

// https://stackoverflow.com/questions/70983595/deducing-extent-when-passing-stdarray-to-function-expecting-stdspan#comment125484174_70983595
template<class T, class... Us>
decltype(auto) i2c_write(
   T&& value,
   Us&&... args
) {
   return i2c_write(
      std::span<std::byte> {
         std::forward<T>(value).data(), std::forward<T>(value).size()},
      std::forward<Us>(args)...
   );
}

template<class T, class... Us>
decltype(auto) i2c_read(
   T&& value,
   Us&&... args
) {
   return i2c_read(
      std::span{ std::forward<T>(value) },
      std::forward<Us>(args)...
   );
}



auto main() -> int {
constexpr int encoder_i2c_slave_addr = 0x20;

int adapter_num = 0;
char fp[13];
snprintf(fp, sizeof(fp), "/dev/i2c-%d", adapter_num);
int fd = open(fp, O_RDWR);

i2c_write(
   std::array {
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00}
   },
   REG_CVALB4, fd, encoder_i2c_slave_addr
);

i2c_write(
   std::array {
      std::byte{0x7F},
      std::byte{0xFF},
      std::byte{0xFF},
      std::byte{0xFF}
   },
   REG_CMAXB4, fd, encoder_i2c_slave_addr
);

i2c_write(
   std::array {
      std::byte{0x80},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00}
   },
   REG_CMINB4, fd, encoder_i2c_slave_addr
);

i2c_write(
   std::array {
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x00},
      std::byte{0x01}
   },
   REG_ISTEPB4, fd, encoder_i2c_slave_addr
);

for (int32_t res;;) {
   std::array<std::byte, 4> buf;
   i2c_read(buf, REG_CVALB4, fd, encoder_i2c_slave_addr);
   res = __builtin_bswap32(std::bit_cast<int32_t>(buf));

   std::cout << res << std::endl;
   std::cout << std::bitset<32>(res) << std::endl;
   usleep(500000);
}

close(fd);
}
