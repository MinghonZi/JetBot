#pragma once

#include <algorithm>
#include <limits>

#include <linux_i2c.hh>

inline constexpr std::byte XG_OFFS_TC_H      {0x04};
inline constexpr std::byte XG_OFFS_TC_L      {0x05};
inline constexpr std::byte YG_OFFS_TC_H      {0x07};
inline constexpr std::byte YG_OFFS_TC_L      {0x08};
inline constexpr std::byte ZG_OFFS_TC_H      {0x0a};
inline constexpr std::byte ZG_OFFS_TC_L      {0x0b};
inline constexpr std::byte SELF_TEST_X_ACCEL {0x0d};
inline constexpr std::byte SELF_TEST_Y_ACCEL {0x0e};
inline constexpr std::byte SELF_TEST_Z_ACCEL {0x0f};
inline constexpr std::byte XG_OFFS_USRH      {0x13};
inline constexpr std::byte XG_OFFS_USRL      {0x14};
inline constexpr std::byte YG_OFFS_USRH      {0x15};
inline constexpr std::byte YG_OFFS_USRL      {0x16};
inline constexpr std::byte ZG_OFFS_USRH      {0x17};
inline constexpr std::byte ZG_OFFS_USRL      {0x18};
inline constexpr std::byte SMPLRT_DIV        {0x19};
inline constexpr std::byte CONFIG            {0x1a};
inline constexpr std::byte GYRO_CONFIG       {0x1b};
inline constexpr std::byte ACCEL_CONFIG      {0x1c};
inline constexpr std::byte ACCEL_CONFIG2     {0x1d};
inline constexpr std::byte GYRO_LP_MODE_CFG  {0x1e};
inline constexpr std::byte ACCEL_WOM_X_THR   {0x20};
inline constexpr std::byte ACCEL_WOM_Y_THR   {0x21};
inline constexpr std::byte ACCEL_WOM_Z_THR   {0x22};
inline constexpr std::byte FIFO_EN           {0x23};
inline constexpr std::byte FSYNC_INT         {0x36};
inline constexpr std::byte INT_PIN_CFG       {0x37};
inline constexpr std::byte INT_ENABLE        {0x38};
inline constexpr std::byte FIFO_WM_INT_STATU {0x39};
inline constexpr std::byte INT_STATUS        {0x3a};
inline constexpr std::byte ACCEL_XOUT_H      {0x3b}; // Big-endian
inline constexpr std::byte ACCEL_XOUT_L      {0x3c};
inline constexpr std::byte ACCEL_YOUT_H      {0x3d};
inline constexpr std::byte ACCEL_YOUT_L      {0x3e};
inline constexpr std::byte ACCEL_ZOUT_H      {0x3f};
inline constexpr std::byte ACCEL_ZOUT_L      {0x40};
inline constexpr std::byte TEMP_OUT_H        {0x41};
inline constexpr std::byte TEMP_OUT_L        {0x42};
inline constexpr std::byte GYRO_XOUT_H       {0x43}; // Big-endian
inline constexpr std::byte GYRO_XOUT_L       {0x44};
inline constexpr std::byte GYRO_YOUT_H       {0x45};
inline constexpr std::byte GYRO_YOUT_L       {0x46};
inline constexpr std::byte GYRO_ZOUT_H       {0x47};
inline constexpr std::byte GYRO_ZOUT_L       {0x48};
inline constexpr std::byte SELF_TEST_X_GYRO  {0x50};
inline constexpr std::byte SELF_TEST_Y_GYRO  {0x51};
inline constexpr std::byte SELF_TEST_Z_GYRO  {0x52};
inline constexpr std::byte FIFO_WM_TH1       {0x60};
inline constexpr std::byte FIFO_WM_TH2       {0x61};
inline constexpr std::byte SIGNAL_PATH_RESET {0x68};
inline constexpr std::byte ACCEL_INTEL_CTRL  {0x69};
inline constexpr std::byte USER_CTRL         {0x6A};
inline constexpr std::byte PWR_MGMT_1        {0x6b};
inline constexpr std::byte PWR_MGMT_2        {0x6c};
inline constexpr std::byte I2C_IF            {0x70};
inline constexpr std::byte FIFO_COUNTH       {0x72};
inline constexpr std::byte FIFO_COUNTL       {0x73};
inline constexpr std::byte FIFO_R_W          {0x74};
inline constexpr std::byte WHO_AM_I          {0x75};
inline constexpr std::byte XA_OFFSET_H       {0x77};
inline constexpr std::byte XA_OFFSET_L       {0x78};
inline constexpr std::byte YA_OFFSET_H       {0x7a};
inline constexpr std::byte YA_OFFSET_L       {0x7b};
inline constexpr std::byte ZA_OFFSET_H       {0x7d};
inline constexpr std::byte ZA_OFFSET_L       {0x7e};


class ICM20600 : private LinuxI2C {
public:
  // TODO: Replace __builtin_bswap16 with C++23 std::byteswap

  enum power_mode {
    SLEEP = 0,
    STANDYBY,
    ACCEL_LOW_POWER,
    ACCEL_LOW_NOISE,
    GYRO_LOW_POWER,
    GYRO_LOW_NOISE,
    SIX_AXIS_LOW_POWER,
    SIX_AXIS_LOW_NOISE,
  };

  // Accelerometer low pass filter setting
  enum accel_dlpf_cfg {
    ACCEL_RATE_4K_BW_1046 = 0,
    ACCEL_RATE_1K_BW_420,
    ACCEL_RATE_1K_BW_218,
    ACCEL_RATE_1K_BW_99,
    ACCEL_RATE_1K_BW_44,
    ACCEL_RATE_1K_BW_21,
    ACCEL_RATE_1K_BW_10,
    ACCEL_RATE_1K_BW_5,
  };

  // Gyroscope low pass filter setting
  enum gyro_dlpf_cfg {
    GYRO_RATE_8K_BW_3281 = 0,
    GYRO_RATE_8K_BW_250,
    GYRO_RATE_1K_BW_176,
    GYRO_RATE_1K_BW_92,
    GYRO_RATE_1K_BW_41,
    GYRO_RATE_1K_BW_20,
    GYRO_RATE_1K_BW_10,
    GYRO_RATE_1K_BW_5,
  };

  // Accelerometer full-scale range
  enum accel_scale {
    FS_2G = 0,
    FS_4G,
    FS_8G,
    FS_16G,
  };

  // Gyroscope full-scale range
  enum gyro_scale {
    FS_250_DPS = 0,
    FS_500_DPS,
    FS_1K_DPS,
    FS_2K_DPS,
  };

  ICM20600(uint8_t adapter_id, bool AD0)
  : LinuxI2C{adapter_id, AD0 ? std::byte{0x69} : std::byte{0x68}} {

    LinuxI2C::write(CONFIG, std::byte{0b0'0'000'000});

    // Disable GYRO_FIFO and ACCEL_FIFO
    LinuxI2C::write(FIFO_EN, std::byte{0b000'0'0'000});
  }

  // Unit: standard acceleration due to gravity
  [[nodiscard]] std::array<double, 3>&
  accel_rd() {
    accel_scale_update();

    bytearray<2> xout, yout, zout;
    LinuxI2C::read(ACCEL_XOUT_H, xout); // Big-endian-encoded data stream
    LinuxI2C::read(ACCEL_YOUT_H, yout);
    LinuxI2C::read(ACCEL_ZOUT_H, zout);

    std::array<int16_t, 3> xyz;
    // Decode big-endian-encoded 16-bit int
    // TODO: Wrap the decode algo as a fn
    // https://commandcenter.blogspot.com/2012/04/byte-order-fallacy.html
    xyz[0] = __builtin_bswap16(std::bit_cast<int16_t>(xout));
    xyz[1] = __builtin_bswap16(std::bit_cast<int16_t>(yout));
    xyz[2] = __builtin_bswap16(std::bit_cast<int16_t>(zout));

    std::ranges::transform(xyz, linear_accel.begin(),
      [&](int16_t e) {
        return static_cast<double>(e)
          / std::numeric_limits<int16_t>::max()
          * accel_scale;
      }
    );

    return linear_accel;
  }

  // Unit: degree per second
  [[nodiscard]] std::array<double, 3>&
  gyro_rd() {
    gyro_scale_update();

    bytearray<2> xout, yout, zout;
    LinuxI2C::read(GYRO_XOUT_H, xout);
    LinuxI2C::read(GYRO_YOUT_H, yout);
    LinuxI2C::read(GYRO_ZOUT_H, zout);

    std::array<int16_t, 3> xyz;
    xyz[0] = __builtin_bswap16(std::bit_cast<int16_t>(xout));
    xyz[1] = __builtin_bswap16(std::bit_cast<int16_t>(yout));
    xyz[2] = __builtin_bswap16(std::bit_cast<int16_t>(zout));

    std::ranges::transform(xyz, angular_vel.begin(),
      [&](int16_t e) {
        return static_cast<double>(e)
          / std::numeric_limits<int16_t>::max()
          * gyro_scale;
      }
    );

    return angular_vel;
  }

  // See the datasheet 4.17
  void
  set_power_mode(power_mode mode) {
    std::byte pwr1_bits;
    std::byte pwr2_bits;
    std::byte gyro_lp_bits;

    pwr1_bits = LinuxI2C::read(PWR_MGMT_1);
    pwr1_bits &= std::byte{0b1'0'0'0'1'111}; // mask
    gyro_lp_bits = LinuxI2C::read(GYRO_LP_MODE_CFG);
    gyro_lp_bits = std::byte{0b0'111'1111}; // mask

    switch (mode) {
      case SLEEP:
        pwr1_bits |= std::byte{0b0'1'0'0'0'000};
        pwr2_bits = std::byte{0};
        break;

      case STANDYBY:
        pwr1_bits |= std::byte{0b0'0'0'1'0'000};
        pwr2_bits = std::byte{0b00'1'1'1'0'0'0};
        break;

      case ACCEL_LOW_POWER:
        pwr1_bits |= std::byte{0b0'0'1'0'0'000};
        pwr2_bits = std::byte{0b00'0'0'0'1'1'1};
        break;

      case ACCEL_LOW_NOISE:
        pwr1_bits |= std::byte{0};
        pwr2_bits = std::byte{0b00'0'0'0'1'1'1};
        break;

      case GYRO_LOW_POWER:
        pwr1_bits |= std::byte{0};
        pwr2_bits = std::byte{0b00'1'1'1'0'0'0};
        gyro_lp_bits = std::byte{0b1'000'0000};
        break;

      case GYRO_LOW_NOISE:
        pwr1_bits |= std::byte{0};
        pwr2_bits = std::byte{0b00'1'1'1'0'0'0};
        break;

      case SIX_AXIS_LOW_POWER:
        pwr1_bits |= std::byte{0};
        pwr2_bits = std::byte{0};
        gyro_lp_bits = std::byte{0b1'000'0000};
        break;

      case SIX_AXIS_LOW_NOISE:
        pwr1_bits |= std::byte{0};
        pwr2_bits = std::byte{0};
        break;

      default:
          break;
    }

    LinuxI2C::write(PWR_MGMT_1, pwr1_bits);
    LinuxI2C::write(PWR_MGMT_2, pwr2_bits);
    LinuxI2C::write(GYRO_LP_MODE_CFG, gyro_lp_bits);
  }

  // See the datasheet table 19
  void
  set_accel_dlpf(accel_dlpf_cfg cfg) {
    std::byte bits = LinuxI2C::read(ACCEL_CONFIG2);
    bits &= std::byte{0b11'11'0'000}; // mask
    switch (cfg) {
      case ACCEL_RATE_4K_BW_1046:
        bits |= std::byte{0x08};
        break;
      case ACCEL_RATE_1K_BW_420:
        bits |= std::byte{0x07};
        break;
      case ACCEL_RATE_1K_BW_218:
        bits |= std::byte{0x01};
        break;
      case ACCEL_RATE_1K_BW_99:
        bits |= std::byte{0x02};
        break;
      case ACCEL_RATE_1K_BW_44:
        bits |= std::byte{0x03};
        break;
      case ACCEL_RATE_1K_BW_21:
        bits |= std::byte{0x04};
        break;
      case ACCEL_RATE_1K_BW_10:
        bits |= std::byte{0x05};
        break;
      case ACCEL_RATE_1K_BW_5:
        bits |= std::byte{0x06};
        break;
    }
    LinuxI2C::write(ACCEL_CONFIG2, bits);
  }

  // See the datasheet table 18
  void
  set_gyro_dlpf(gyro_dlpf_cfg cfg) {
    std::byte bits = LinuxI2C::read(CONFIG);
    bits &= std::byte{0b1'1'111'000}; // mask
    // TODO: Add two cases for FCHOICE_B is 0bx1 or 0b10
    switch (cfg) {
      case GYRO_RATE_8K_BW_3281:
        bits |= std::byte{0x07};
        break;
      case GYRO_RATE_8K_BW_250:
        bits |= std::byte{0x00};
        break;
      case GYRO_RATE_1K_BW_176:
        bits |= std::byte{0x01};
        break;
      case GYRO_RATE_1K_BW_92:
        bits |= std::byte{0x02};
        break;
      case GYRO_RATE_1K_BW_41:
        bits |= std::byte{0x03};
        break;
      case GYRO_RATE_1K_BW_20:
        bits |= std::byte{0x04};
        break;
      case GYRO_RATE_1K_BW_10:
        bits |= std::byte{0x05};
        break;
      case GYRO_RATE_1K_BW_5:
        bits |= std::byte{0x06};
        break;
    }
    LinuxI2C::write(CONFIG, bits);
  }

  // TODO: Impl set_accel_avg_sample and set_gyro_avg_sample

  void
  set_accel_scale(accel_scale scale) {
    std::byte bits = LinuxI2C::read(ACCEL_CONFIG);
    bits &= std::byte{0b000'11'000}; // mask
    switch (scale) {
      case FS_2G:
        bits |= std::byte{0b000'00'000};
        break;
      case FS_4G:
        bits |= std::byte{0b000'01'000};
        break;
      case FS_8G:
        bits |= std::byte{0b000'10'000};
        break;
      case FS_16G:
        bits |= std::byte{0b000'11'000};
        break;
    }
    LinuxI2C::write(ACCEL_CONFIG, bits);
  }

  void
  set_gyro_scale(gyro_scale scale) {
    std::byte bits = LinuxI2C::read(GYRO_CONFIG);
    bits &= std::byte{0b000'11'000}; // mask
    switch (scale) {
      case FS_250_DPS:
        bits |= std::byte{0b000'00'000};
        break;
      case FS_500_DPS:
        bits |= std::byte{0b000'01'000};
        break;
      case FS_1K_DPS:
        bits |= std::byte{0b000'10'000};
        break;
      case FS_2K_DPS:
        bits |= std::byte{0b000'11'000};
        break;
    }
    LinuxI2C::write(GYRO_CONFIG, bits);
  }

protected:
  int accel_scale;
  int gyro_scale;

  void
  accel_scale_update() {
    switch (LinuxI2C::read(ACCEL_CONFIG) & std::byte{0b000'11'000}) {
      case std::byte{0b000'00'000}:
        accel_scale = 2;
        break;
      case std::byte{0b000'01'000}:
        accel_scale = 4;
        break;
      case std::byte{0b000'10'000}:
        accel_scale = 8;
        break;
      case std::byte{0b000'11'000}:
        accel_scale = 16;
        break;
    }
  }

  void
  gyro_scale_update() {
    switch (LinuxI2C::read(GYRO_CONFIG) & std::byte{0b000'11'000}) {
      case std::byte{0b000'00'000}:
        gyro_scale = 250;
        break;
      case std::byte{0b000'01'000}:
        gyro_scale = 500;
        break;
      case std::byte{0b000'10'000}:
        gyro_scale = 1000;
        break;
      case std::byte{0b000'11'000}:
        gyro_scale = 2000;
        break;
    }
  }

  std::array<double, 3> linear_accel;
  std::array<double, 3> angular_vel;
};
