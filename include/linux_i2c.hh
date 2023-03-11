#pragma once

/* https://en.wikipedia.org/wiki/C_POSIX_library */
#include <fcntl.h> // open, O_RDWR
#include <unistd.h> // close

/* https://man7.org/linux/man-pages/man2/ioctl.2.html */
#include <sys/ioctl.h> // ioctl

/* Kernel headers (/usr/include/linux) for the express purpose of I2C */
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

/* C compatibility headers https://en.cppreference.com/w/cpp/header */
#include <cstddef> // byte
#include <cstdint> // uint8_t
#include <cstdio> // snprintf
#include <cstring> // memcpy

#include <string>
#include <span>
#include <array>
#include <vector>
#include <bit> // bit_cast
#include <stdexcept> // runtime_error

template <std::size_t S>
using bytearray = std::array<std::byte, S>;

/**
 * An adapter that transfers Linux's C-style I2C interfacing to C++-style
*/
class LinuxI2C {
public:
   LinuxI2C(uint8_t adapter_id, std::byte tgt_addr)
      : adapter_id_{adapter_id}, tgt_addr_{tgt_addr} {
      fd_ = open_i2c_bus(adapter_id_);
   }

   // TODO: Delete copy constructor
   // TODO: Add moving constructor
   // TODO: Add more constructors for convience

   ~LinuxI2C() {
      close(fd_);
   }

   void
   read(std::byte reg, std::byte &byte) {
      i2c_msg msgs[2] {{
         .addr  = static_cast<__u16>(tgt_addr_),
         .flags = 0, // write
         .len   = 1, // one byte
         .buf   = std::bit_cast<__u8*>(&reg)
      }, {
         .addr  = static_cast<__u16>(tgt_addr_),
         .flags = I2C_M_RD, // read
         .len   = 1, // one byte
         .buf   = std::bit_cast<__u8*>(&byte)
      }};
      i2c_rdwr_ioctl_data data {
         .msgs  = msgs,
         .nmsgs = 2
      };

      int code = ioctl(fd_, I2C_RDWR, &data);
      // If succeed, ioctl returns nmsgs (2 in this case) being transferred
      if (code < 0) [[unlikely]]
         throw std::runtime_error("I2C fault code: " + std::to_string(code));
   }

   void
   read(std::byte reg, std::span<std::byte> bytes) {
      i2c_msg msgs[2] {{
         .addr  = static_cast<__u16>(tgt_addr_),
         .flags = 0, // write
         .len   = 1, // one byte
         .buf   = std::bit_cast<__u8*>(&reg)
      }, {
         .addr  = static_cast<__u16>(tgt_addr_),
         .flags = I2C_M_RD,
         .len   = static_cast<__u16>(bytes.size()),
         .buf   = std::bit_cast<__u8*>(bytes.data())
      }};
      i2c_rdwr_ioctl_data data {
         .msgs  = msgs,
         .nmsgs = 2
      };

      int code = ioctl(fd_, I2C_RDWR, &data);
      if (code < 0) [[unlikely]]
         throw std::runtime_error("I2C fault code: " + std::to_string(code));
   }

   void
   write(std::byte reg, std::byte byte) {
      // Construct buffer as || reg | one byte ||
      bytearray<sizeof(reg) + sizeof(byte)> buf;
      std::memcpy(buf.data(), &reg, sizeof(reg));
      std::memcpy(buf.data() + sizeof(reg), &byte, sizeof(byte));

      i2c_msg msgs[1] {{
         .addr  = static_cast<__u16>(tgt_addr_),
         .flags = 0, // write
         .len   = static_cast<__u16>(buf.size()),
         .buf   = std::bit_cast<__u8*>(buf.data())
      }};
      i2c_rdwr_ioctl_data data {
         .msgs  = msgs,
         .nmsgs = 1
      };

      int code = ioctl(fd_, I2C_RDWR, &data);
      if (code < 0) [[unlikely]]
         throw std::runtime_error("I2C fault code: " + std::to_string(code));         
   }

   void
   write(std::byte reg, std::span<std::byte const> bytes) {
      // Construct buffer as || reg | bytes ||
      std::vector<std::byte> buf(sizeof(reg) + bytes.size());
      std::memcpy(buf.data(), &reg, sizeof(reg));
      std::memcpy(buf.data() + sizeof(reg), bytes.data(), bytes.size());

      i2c_msg msgs[1] {{
         .addr  = static_cast<__u16>(tgt_addr_),
         .flags = 0, // write
         .len   = static_cast<__u16>(buf.size()),
         .buf   = std::bit_cast<__u8*>(buf.data())
      }};
      i2c_rdwr_ioctl_data data {
         .msgs  = msgs,
         .nmsgs = 1
      };

      int code = ioctl(fd_, I2C_RDWR, &data);
      if (code < 0) [[unlikely]]
         throw std::runtime_error("I2C fault code: " + std::to_string(code));
   }

protected:
   uint8_t adapter_id_;
   std::byte tgt_addr_; // Target address
   int fd_;
   // TODO: Write getters?

   [[nodiscard]] static int
   open_i2c_bus(uint8_t adapter_id) {
      // TODO: Use <format>
      char fp[13];
      std::snprintf(fp, sizeof(fp), "/dev/i2c-%d", adapter_id);
      int fd = open(fp, O_RDWR);

      if (fd < 0) [[unlikely]]
         throw std::runtime_error("Could not open " + std::string(fp));
      else [[likely]]
         return fd;
   }
};
