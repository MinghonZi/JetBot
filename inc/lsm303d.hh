#pragma once

#include <algorithm>
#include <limits>

#include <linux_i2c.hh>

/* LSM303D register definitions */
inline constexpr std::byte TEMP_OUT_L {0x05};
inline constexpr std::byte TEMP_OUT_H {0x06};
inline constexpr std::byte STATUS_M {0x07};
inline constexpr std::byte OUT_X_L_M {0x08}; // expressed in 16-bit as 2’s complement
inline constexpr std::byte OUT_X_H_M {0x09};
inline constexpr std::byte OUT_Y_L_M {0x0A}; // expressed in 16-bit as 2’s complement
inline constexpr std::byte OUT_Y_H_M {0x0B};
inline constexpr std::byte OUT_Z_L_M {0x0C}; // expressed in 16-bit as 2’s complement
inline constexpr std::byte OUT_Z_H_M {0x0D};
inline constexpr std::byte WHO_AM_I {0x0F};
inline constexpr std::byte INT_CTRL_M {0x12};
inline constexpr std::byte INT_SRC_M {0x13};
inline constexpr std::byte INT_THS_L_M {0x14};
inline constexpr std::byte INT_THS_H_M {0x15};
inline constexpr std::byte OFFSET_X_L_M {0x16};
inline constexpr std::byte OFFSET_X_H_M {0x17};
inline constexpr std::byte OFFSET_Y_L_M {0x18};
inline constexpr std::byte OFFSET_Y_H_M {0x19};
inline constexpr std::byte OFFSET_Z_L_M {0x1A};
inline constexpr std::byte OFFSET_Z_H_M {0x1B};
inline constexpr std::byte REFERENCE_X {0x1C};
inline constexpr std::byte REFERENCE_Y {0x1D};
inline constexpr std::byte REFERENCE_Z {0x1E};
inline constexpr std::byte CTRL0 {0x1F};
inline constexpr std::byte CTRL1 {0x20};
inline constexpr std::byte CTRL2 {0x21};
inline constexpr std::byte CTRL3 {0x22};
inline constexpr std::byte CTRL4 {0x23};
inline constexpr std::byte CTRL5 {0x24};
inline constexpr std::byte CTRL6 {0x25};
inline constexpr std::byte CTRL7 {0x26};
inline constexpr std::byte STATUS_A {0x27};
inline constexpr std::byte OUT_X_L_A {0x28}; // expressed in 16-bit as 2’s complement
inline constexpr std::byte OUT_X_H_A {0x29};
inline constexpr std::byte OUT_Y_L_A {0x2A}; // expressed in 16-bit as 2’s complement
inline constexpr std::byte OUT_Y_H_A {0x2B};
inline constexpr std::byte OUT_Z_L_A {0x2C}; // expressed in 16-bit as 2’s complement
inline constexpr std::byte OUT_Z_H_A {0x2D};
inline constexpr std::byte FIFO_CTRL {0x2E};
inline constexpr std::byte FIFO_SRC {0x2F};
inline constexpr std::byte IG_CFG1 {0x30};
inline constexpr std::byte IG_SRC1 {0x31};
inline constexpr std::byte IG_THS1 {0x32};
inline constexpr std::byte IG_DUR1 {0x33};
inline constexpr std::byte IG_CFG2 {0x34};
inline constexpr std::byte IG_SRC2 {0x35};
inline constexpr std::byte IG_THS2 {0x36};
inline constexpr std::byte IG_DUR2 {0x37};
inline constexpr std::byte CLICK_CFG {0x38};
inline constexpr std::byte CLICK_SRC {0x39};
inline constexpr std::byte CLICK_THS {0x3A};
inline constexpr std::byte TIME_LIMIT {0x3B};
inline constexpr std::byte TIME_LATENCY {0x3C};
inline constexpr std::byte TIME_WINDOW {0x3D};
inline constexpr std::byte ACT_THS {0x3E};
inline constexpr std::byte ACT_DUR {0x3F};

/**
 * On the datasheet 6.1.1 it says if the MSb of the SUB field (register address)
 * is 1, the SUB is automatically incremented to allow multiple data read/write.
*/
inline constexpr std::byte MULTI_BYTES {0x80};


class LSM303D : private LinuxI2C {
public:
   LSM303D(uint8_t adapter_id, std::byte tgt_addr)
      : LinuxI2C{adapter_id, tgt_addr} {
      acceleration_fullscale_update();
      magnetic_fullscale_update();
   }

   /**
    * The temperature sensor can be used only to measure temperature
    * variations. It isn't suitable to return absolute temperatures measures.
    * The value represents difference respect to a reference not specified
    * value.
    * https://community.st.com/s/question/0D50X00009XkZbESAV/lsm303d-temperature-format
    * 
    * Unit: degree Celsius
   */
   [[nodiscard]] double
   temperature_rd() {
      bytearray<2> bytes;
      LinuxI2C::read(TEMP_OUT_L | MULTI_BYTES, bytes);
      // See the datasheet table 4
      return static_cast<double>(std::bit_cast<int16_t>(bytes)) / 8;
   }

   // Unit: standard acceleration due to gravity
   [[nodiscard]] std::array<double, 3>&
   accelerometer_rd() {
      acceleration_fullscale_update();

      bytearray<6> bytes;
      LinuxI2C::read(OUT_X_L_A | MULTI_BYTES, bytes);

      // Data is expressed in 16-bit as 2’s complement
      auto unscaled = std::bit_cast<std::array<int16_t, 3>>(bytes);

      /**
       * TODO: Use C++23 std::ranges::to with pipe syntax to eliminate
       * redundant call of begin iterator of the destination
      */
      std::ranges::transform(unscaled, acceleration.begin(), [&](int16_t e) {
         return static_cast<double>(e)
            / std::numeric_limits<int16_t>::max()
            * acceleration_fullscale;
      });

      return acceleration;
   }

   // Unit: gauss
   [[nodiscard]] std::array<double, 3>&
   magnetometer_rd() {
      magnetic_fullscale_update();

      bytearray<6> bytes;
      LinuxI2C::read(OUT_X_L_M | MULTI_BYTES, bytes);

      auto unscaled = std::bit_cast<std::array<int16_t, 3>>(bytes);

      std::ranges::transform(unscaled, magnetic.begin(), [&](int16_t e) {
         return static_cast<double>(e)
            / std::numeric_limits<int16_t>::max()
            * magnetic_fullscale;
      });

      return magnetic;
   }

   void
   control1_wr(std::byte byte) {
      LinuxI2C::write(CTRL1, byte);
   }

   void
   control2_wr(std::byte byte) {
      LinuxI2C::write(CTRL2, byte);
      acceleration_fullscale_update();
   }

   void
   control3_wr(std::byte byte) {
      LinuxI2C::write(CTRL3, byte);
   }

   void
   control4_wr(std::byte byte) {
      LinuxI2C::write(CTRL4, byte);
   }

   void
   control5_wr(std::byte byte) {
      LinuxI2C::write(CTRL5, byte);
   }

   void
   control6_wr(std::byte byte) {
      LinuxI2C::write(CTRL6, byte);
      magnetic_fullscale_update();
   }

   void
   control7_wr(std::byte byte) {
      LinuxI2C::write(CTRL7, byte);
   }

   // FIXME: Non exhaustive

   /**
    * Convenience Functions for Pybind11
    * TODO: pybind11 doesn't have built-in conversion for std::byte
    */

   LSM303D(uint8_t adapter_id, uint8_t tgt_addr)
      : LSM303D{adapter_id, std::byte{tgt_addr}} {}

   void
   control1_wr(uint8_t byte) {
      control1_wr(std::byte{byte});
   }

   void
   control2_wr(uint8_t byte) {
      control2_wr(std::byte{byte});
      acceleration_fullscale_update();
   }

   void
   control3_wr(uint8_t byte) {
      control3_wr(std::byte{byte});
   }

   void
   control4_wr(uint8_t byte) {
      control4_wr(std::byte{byte});
   }

   void
   control5_wr(uint8_t byte) {
      control5_wr(std::byte{byte});
   }

   void
   control6_wr(uint8_t byte) {
      control6_wr(std::byte{byte});
      magnetic_fullscale_update();
   }

   void
   control7_wr(uint8_t byte) {
      control7_wr(std::byte{byte});
   }

protected:
   int acceleration_fullscale;
   int magnetic_fullscale;

   // See the datasheet table 40
   void
   acceleration_fullscale_update() {
      switch (LinuxI2C::read(CTRL2) & std::byte{0b00'111'000}) {
      case std::byte{0b00'000'000}:
         acceleration_fullscale = 2;
         break;
      case std::byte{0b00'001'000}:
         acceleration_fullscale = 4;
         break;
      case std::byte{0b00'010'000}:
         acceleration_fullscale = 6;
         break;
      case std::byte{0b00'011'000}:
         acceleration_fullscale = 8;
         break;
      case std::byte{0b00'100'000}:
         acceleration_fullscale = 16;
         break;
      }
   }

   // See the datasheet table 50
   void
   magnetic_fullscale_update() {
      switch (LinuxI2C::read(CTRL6) & std::byte{0b0'11'00000}) {
      case std::byte{0b0'00'00000}:
         magnetic_fullscale = 2;
         break;
      case std::byte{0b0'01'00000}:
         magnetic_fullscale = 4;
         break;
      case std::byte{0b0'10'00000}:
         magnetic_fullscale = 8;
         break;
      case std::byte{0b0'11'00000}:
         magnetic_fullscale = 12;
         break;
      }
   }

   std::array<double, 3> acceleration;
   std::array<double, 3> magnetic;
};
