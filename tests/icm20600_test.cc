#include <iostream>

#include <icm20600.hh>

using namespace std;

auto main() -> int {
  ICM20600 icm20600(0, true);

  icm20600.set_power_mode(ICM20600::SIX_AXIS_LOW_NOISE);
  icm20600.set_accel_dlpf(ICM20600::ACCEL_RATE_1K_BW_420);
  icm20600.set_gyro_dlpf(ICM20600::GYRO_RATE_1K_BW_176);

  icm20600.set_accel_scale(ICM20600::FS_4G);
  icm20600.set_gyro_scale(ICM20600::FS_500_DPS);

  array<double, 3> linear_accel, angular_vel;
  for (;;) {
    linear_accel = icm20600.accel_rd();
    angular_vel = icm20600.gyro_rd();

    // Display all the precision of double
    cout.precision(std::numeric_limits<double>::max_digits10);

    cout << "Linear acceleration (g): ";
    for (const auto &e : linear_accel) {
      std::cout << e << ' ';
    }
    cout << endl;

    cout << "Angular velocity (dps): ";
    for (const auto &e : angular_vel) {
      std::cout << e << ' ';
    }
    cout << endl;

    usleep(500000);
  }
}
