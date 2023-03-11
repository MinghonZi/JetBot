from time import sleep

from duppa_i2c_encoder_mini import DuPPaI2CEncoderMini

# Encoder configuration bit. Use with GCONF
WRAP_ENABLE = 0x01
WRAP_DISABLE = 0x00
DIRE_LEFT = 0x02
DIRE_RIGHT = 0x00
IPUP_ENABLE = 0x04
IPUP_DISABLE = 0x00
RMOD_X4 = 0x10
RMOD_X2 = 0x08
RMOD_X1 = 0x00

encoder = DuPPaI2CEncoderMini(0, 0x20)
encoder.gconf_wr(WRAP_DISABLE | DIRE_LEFT | IPUP_DISABLE | RMOD_X4)
encoder.cmax_wr(30)
encoder.cmin_wr(-30)
encoder.istep_wr(1)
encoder.cval_clr()

try:
    while True:
        print(encoder.cval_rd())
        sleep(0.1)
except KeyboardInterrupt:
    pass
