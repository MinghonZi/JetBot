#pragma once

#include <cstddef>

/* DuPPa I2C Encoder Mini register definition */
inline constexpr std::byte REG_GCONF {0x00};
inline constexpr std::byte REG_INTCONF {0x01};
inline constexpr std::byte REG_ESTATUS {0x02};
inline constexpr std::byte REG_CVALB4 {0x03};  // 32-bit signed int
inline constexpr std::byte REG_CVALB3 {0x04};
inline constexpr std::byte REG_CVALB2 {0x05};
inline constexpr std::byte REG_CVALB1 {0x06};
inline constexpr std::byte REG_CMAXB4 {0x07};  // 32-bit signed int
inline constexpr std::byte REG_CMAXB3 {0x08};
inline constexpr std::byte REG_CMAXB2 {0x09};
inline constexpr std::byte REG_CMAXB1 {0x0A};
inline constexpr std::byte REG_CMINB4 {0x0B};  // 32-bit signed int
inline constexpr std::byte REG_CMINB3 {0x0C};
inline constexpr std::byte REG_CMINB2 {0x0D};
inline constexpr std::byte REG_CMINB1 {0x0E};
inline constexpr std::byte REG_ISTEPB4 {0x0F}; // 32-bit signed int
inline constexpr std::byte REG_ISTEPB3 {0x10};
inline constexpr std::byte REG_ISTEPB2 {0x11};
inline constexpr std::byte REG_ISTEPB1 {0x12};
inline constexpr std::byte REG_DPPERIOD {0x13};
inline constexpr std::byte REG_ADDRESS {0x14};
inline constexpr std::byte REG_IDCODE {0x70};
inline constexpr std::byte REG_VERSION {0x71};
inline constexpr std::byte REG_I2CADDRESS {0x72};
inline constexpr std::byte REG_EEPROMS {0x81};

/* Encoder configuration bit. Use with GCONF */
inline constexpr std::byte WRAP_ENABLE  {0x01};
inline constexpr std::byte WRAP_DISABLE {0x00};
inline constexpr std::byte DIRE_LEFT  {0x02};
inline constexpr std::byte DIRE_RIGHT {0x00};
inline constexpr std::byte IPUP_ENABLE  {0x04};
inline constexpr std::byte IPUP_DISABLE {0x00};
inline constexpr std::byte RMOD_X4 {0x10};
inline constexpr std::byte RMOD_X2 {0x08};
inline constexpr std::byte RMOD_X1 {0x00};
inline constexpr std::byte RESET {0x80};

/* Encoder status bits and setting. Use with: INTCONF for set and with ESTATUS for read the bits */
inline constexpr std::byte PUSHR {0x01};
inline constexpr std::byte PUSHP {0x02};
inline constexpr std::byte PUSHD {0x04};
inline constexpr std::byte PUSHL {0x08};
inline constexpr std::byte RINC {0x10};
inline constexpr std::byte RDEC {0x20};
inline constexpr std::byte RMAX {0x40};
inline constexpr std::byte RMIN {0x80};
