#include <iostream>

#include <lsm303d.hh>

using namespace std;

auto main() -> int {
   LSM303D lsm303d(0, byte{0x1d});

   lsm303d.control1_wr(byte{0b0101'0'1'1'1});
   lsm303d.control2_wr(byte{0b00'000'0'0'0});
   lsm303d.control3_wr(byte{0b0'0'0'0'0'0'0'0});
   lsm303d.control4_wr(byte{0b0'0'0'0'0'0'0'0});
   lsm303d.control5_wr(byte{0b1'11'100'0'0});
   lsm303d.control6_wr(byte{0b0'00'00000});
   lsm303d.control7_wr(byte{0b00'0'0'0'0'00});

   float temperature;
   array<float, 3> acceleration, magnetic;
   for (;;) {
      acceleration = lsm303d.accelerometer_rd();
      magnetic = lsm303d.magnetometer_rd();
      temperature = lsm303d.temperature_rd();

      cout << "Acceleration: ";
      for (const auto &e : acceleration) {
         std::cout << e << ' ';
      }
      cout << endl;

      cout << "Magnetic: ";
      for (const auto &e : magnetic) {
         std::cout << e << ' ';
      }
      cout << endl;

      cout << "Temperature: " << temperature << endl << endl;

      usleep(500000);
   }
}
