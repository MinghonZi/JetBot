#include <iostream>
#include <bitset>

#include "linux_i2c.hh"
#include "duppa_i2c_encoder_mini.hh"

using namespace std;

auto main() -> int {
   LinuxI2C encoder(0, byte{0x20});

   encoder.write(WRAP_ENABLE | DIRE_LEFT | IPUP_DISABLE | RMOD_X4, REG_GCONF);
   encoder.write(array{byte{0x7F},byte{0xFF},byte{0xFF},byte{0xFF}}, REG_CMAXB4);
   encoder.write(array{byte{0x80},byte{0x00},byte{0x00},byte{0x00}}, REG_CMINB4);
   encoder.write(array{byte{0x00},byte{0x00},byte{0x00},byte{0x01}}, REG_ISTEPB4);
   encoder.write(array{byte{0x00},byte{0x00},byte{0x00},byte{0x00}}, REG_CVALB4);

   for (array<byte, 4> bytes;;) {
      encoder.read(bytes, REG_CVALB4);

      int32_t res = bit_cast<int32_t>(bytes);
      // I2C is big-endian, and assume the arch is little-endian
      res = __builtin_bswap32(res);

      cout << res << endl;
      cout << bitset<32>(res) << endl;
      usleep(500000);
   }
}
