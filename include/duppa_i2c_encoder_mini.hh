#pragma once

#include <linux_i2c.hh>

/* Encoder register definitions */
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

/* Encoder configuration bits. Use with GCONF */
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


class DuPPaI2CEncoderMini : private LinuxI2C {
public:
   using LinuxI2C::LinuxI2C;
   // TODO: pybind11 doesn't have builtin conversion for std::byte
   DuPPaI2CEncoderMini(uint8_t adapter_id, uint8_t tgt_addr)
      : LinuxI2C{adapter_id, std::byte{tgt_addr}} {}

   void
   gconf_wr(std::byte byte) {
      LinuxI2C::write(REG_GCONF, byte);
   }
   // TODO: pybind11 doesn't have builtin conversion for std::byte
   void
   gconf_wr(uint8_t byte) {
      LinuxI2C::write(REG_GCONF, std::byte{byte});
   }

   [[nodiscard]] int32_t
   cval_rd() {
      bytearray<4> bytes;
      LinuxI2C::read(REG_CVALB4, bytes);
      return to_int32(bytes);
   }

   void
   cval_clr() {
      LinuxI2C::write(
         REG_CVALB4,
         std::array{
            std::byte{0x00},
            std::byte{0x00},
            std::byte{0x00},
            std::byte{0x00}});
   }

   void
   cmax_wr(int32_t val) {
      LinuxI2C::write(REG_CMAXB4, to_bytes(val));
   }

   void
   cmin_wr(int32_t val) {
      LinuxI2C::write(REG_CMINB4, to_bytes(val));
   }

   void
   istep_wr(int32_t val) {
      LinuxI2C::write(REG_ISTEPB4, to_bytes(val));
   }

   void
   dpperiod_wr(std::byte val) {
      LinuxI2C::write(REG_DPPERIOD, val);
   }
   // TODO: pybind11 doesn't have builtin conversion for std::byte
   void
   dpperiod_wr(uint8_t val) {
      LinuxI2C::write(REG_DPPERIOD, std::byte{val});
   }

   // FIXME: Non exhaustive

protected:
   // TODO: Replace __builtin_bswap32 with C++23 std::byteswap

   [[nodiscard]] static int32_t
   to_int32(bytearray<4> bytes) {
      // I2C is big-endian
      if constexpr (std::endian::native == std::endian::little)
         return __builtin_bswap32(std::bit_cast<int32_t>(bytes));
      else if constexpr (std::endian::native == std::endian::big)
         return std::bit_cast<int32_t>(bytes);
      else std::terminate();
   }

   [[nodiscard]] static bytearray<4>
   to_bytes(int32_t val) {
      // Send MSB first
      if constexpr (std::endian::native == std::endian::little)
         val = __builtin_bswap32(val);
      else if constexpr (std::endian::native == std::endian::big);
      else std::terminate();

      return std::bit_cast<bytearray<4>>(val);
   }
};
