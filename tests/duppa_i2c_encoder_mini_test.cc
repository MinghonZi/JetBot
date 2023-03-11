#include <iostream>
#include <bitset>

#include <duppa_i2c_encoder_mini.hh>

using namespace std;

auto main() -> int {
   DuPPaI2CEncoderMini encoder(0, byte{0x20});

   encoder.gconf_wr(WRAP_ENABLE | DIRE_LEFT | IPUP_DISABLE | RMOD_X4);
   encoder.cmax_wr(50);
   encoder.cmin_wr(-50);
   encoder.istep_wr(5);
   encoder.cval_clr();

   for (int32_t cnt;;) {
      cnt = encoder.cval_rd();

      cout << cnt << endl;
      cout << bitset<32>(cnt) << endl;
      usleep(500000);
   }
}
