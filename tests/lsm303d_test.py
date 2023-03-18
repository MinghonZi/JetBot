from time import sleep

from lsm303d import LSM303D

ecompass = LSM303D(0, 0x1d)

ecompass.control1_wr(0b0101_0_1_1_1)
ecompass.control2_wr(0b00_000_0_0_0)
ecompass.control3_wr(0b0_0_0_0_0_0_0_0)
ecompass.control4_wr(0b0_0_0_0_0_0_0_0)
ecompass.control5_wr(0b1_11_100_0_0)
ecompass.control6_wr(0b0_00_00000)
ecompass.control7_wr(0b00_0_0_0_0_00)

try:
    while True:
        print("{:+06.2f}g₀ : {:+06.2f}g₀ : {:+06.2f}g₀"
              .format(*ecompass.accelerometer_rd()))
        print("{:+06.2f}G : {:+06.2f}G : {:+06.2f}G"
              .format(*ecompass.magnetometer_rd()))
        print(f"{ecompass.temperature_rd():+.2f}°C w.r.t. ref")
        print()
        sleep(0.1)
except KeyboardInterrupt:
    pass
