"""
https://docs.kernel.org/i2c/functionality.html
"""

import os
from array import array  # https://docs.python.org/3/library/array.html
from fcntl import ioctl

# Constants selectively scraped from <linux/i2c-dev.h> and <linux/i2c.h>
# TODO: Put into a dict and iterate it
I2C_FUNCS = 0x0705
I2C_RDWR  = 0x0707
I2C_SMBUS = 0x0720
I2C_FUNC_I2C = 0x00000001
I2C_FUNC_10BIT_ADDR = 0x00000002
I2C_FUNC_SMBUS_READ_BLOCK_DATA = 0x01000000

fd = os.open("/dev/i2c-0", os.O_RDWR)
buf = array('I', [0])

ioctl(fd, I2C_FUNCS, buf, True)
os.close(fd)

if buf[0] & I2C_FUNC_I2C == 0:
    print("Functionality (I2C_FUNC_I2C) is not available")
