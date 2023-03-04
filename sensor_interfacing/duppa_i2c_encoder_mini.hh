#pragma once

/* C compatibility headers https://en.cppreference.com/w/cpp/header */
#include <cstddef> // byte
#include <cstring> // memcpy
#include <cstdio> // snprintf

/* https://en.wikipedia.org/wiki/C_POSIX_library */
#include <fcntl.h> // open, O_RDWR
#include <unistd.h> // read, write, close

/* glibc: /usr/include/sys/ioctl.h */
#include <sys/ioctl.h> // ioctl

/* Kernel headers (/usr/include/linux) for the express purpose of I2C */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/* Encoder register definition */
constexpr std::byte REG_GCONF {0x00};
constexpr std::byte REG_INTCONF {0x01};
constexpr std::byte REG_ESTATUS {0x02};
constexpr std::byte REG_CVALB4 {0x03};
   constexpr std::byte REG_CVALB3 {0x04};
   constexpr std::byte REG_CVALB2 {0x05};
   constexpr std::byte REG_CVALB1 {0x06};
constexpr std::byte REG_CMAXB4 {0x07};
   constexpr std::byte REG_CMAXB3 {0x08};
   constexpr std::byte REG_CMAXB2 {0x09};
   constexpr std::byte REG_CMAXB1 {0x0A};
constexpr std::byte REG_CMINB4 {0x0B};
   constexpr std::byte REG_CMINB3 {0x0C};
   constexpr std::byte REG_CMINB2 {0x0D};
   constexpr std::byte REG_CMINB1 {0x0E};
constexpr std::byte REG_ISTEPB4 {0x0F};
   constexpr std::byte REG_ISTEPB3 {0x10};
   constexpr std::byte REG_ISTEPB2 {0x11};
   constexpr std::byte REG_ISTEPB1 {0x12};
constexpr std::byte REG_DPPERIOD {0x13};
constexpr std::byte REG_ADDRESS {0x14};
constexpr std::byte REG_IDCODE {0x70};
constexpr std::byte REG_VERSION {0x71};
constexpr std::byte REG_I2CADDRESS {0x72};
constexpr std::byte REG_EEPROMS {0x81};

/* Encoder configuration bit. Use with GCONF */
constexpr std::byte WRAP_ENABLE {0x01};
constexpr std::byte WRAP_DISABLE {0x00};
constexpr std::byte DIRE_LEFT {0x02};
constexpr std::byte DIRE_RIGHT {0x00};
constexpr std::byte IPUP_ENABLE {0x04};
constexpr std::byte IPUP_DISABLE {0x00};
constexpr std::byte RMOD_X4 {0x10};
constexpr std::byte RMOD_X2 {0x08};
constexpr std::byte RMOD_X1 {0x00};

constexpr std::byte RESET {0x80};

/* Encoder status bits and setting. Use with: INTCONF for set and with ESTATUS for read the bits */
constexpr std::byte PUSHR {0x01};
constexpr std::byte PUSHP {0x02};
constexpr std::byte PUSHD {0x04};
constexpr std::byte PUSHL {0x08};
constexpr std::byte RINC {0x10};
constexpr std::byte RDEC {0x20};
constexpr std::byte RMAX {0x40};
constexpr std::byte RMIN {0x80};
