from collections import namedtuple

__version__ = '0.0.7'


def _mask_width(value, bit_width=8):
    """Get the width of a bitwise mask

    ie: 0b000111 = 3
    """
    value >>= _trailing_zeros(value, bit_width)
    return value.bit_length()


def _leading_zeros(value, bit_width=8):
    """Count leading zeros on a binary number with a given bit_width

    ie: 0b0011 = 2

    Used for shifting around values after masking.
    """
    count = 0
    for _ in range(bit_width):
        if value & (1 << (bit_width - 1)):
            return count
        count += 1
        value <<= 1
    return count


def _trailing_zeros(value, bit_width=8):
    """Count trailing zeros on a binary number with a given bit_width

    ie: 0b11000 = 3

    Used for shifting around values after masking.
    """
    count = 0
    for _ in range(bit_width):
        if value & 1:
            return count
        count += 1
        value >>= 1
    return count


def _int_to_bytes(value, length, endianness='big'):
    try:
        return value.to_bytes(length, endianness)
    except AttributeError:
        output = bytearray()
        for x in range(length):
            offset = x * 8
            mask = 0xff << offset
            output.append((value & mask) >> offset)
        if endianness == 'big':
            output.reverse()
        return output


class MockSMBus:
    def __init__(self, i2c_bus, default_registers=None):
        self.regs = [0 for _ in range(255)]
        if default_registers is not None:
            for index in default_registers.keys():
                self.regs[index] = default_registers.get(index)

    def write_i2c_block_data(self, i2c_address, register, values):
        self.regs[register:register + len(values)] = values

    def read_i2c_block_data(self, i2c_address, register, length):
        return self.regs[register:register + length]


class _RegisterProxy(object):
    """Register Proxy

    This proxy catches lookups against non existent get_fieldname and set_fieldname methods
    and converts them into calls against the device's get_field and set_field methods with
    the appropriate options.

    This means device.register.set_field(value) and device.register.get_field(value) will work
    and also transparently update the underlying device without the register or field objects
    having to know anything about how data is written/read/stored.

    """
    def __init__(self, device, register):
        self.device = device
        self.register = register

    def __getattribute__(self, name):
        if name.startswith("get_"):
            name = name.replace("get_", "")
            return lambda: self.device.get_field(self.register.name, name)
        if name.startswith("set_"):
            name = name.replace("set_", "")
            return lambda value: self.device.set_field(self.register.name, name, value)
        return object.__getattribute__(self, name)

    def write(self):
        return self.device.write_register(self.register.name)

    def read(self):
        return self.device.read_register(self.register.name)

    def __enter__(self):
        self.device.read_register(self.register.name)
        self.device.lock_register(self.register.name)
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        self.device.unlock_register(self.register.name)


class Register():
    """Store information about an i2c register"""
    def __init__(self, name, address, fields=None, bit_width=8, read_only=False, volatile=True):
        self.name = name
        self.address = address
        self.bit_width = bit_width
        self.read_only = read_only
        self.volatile = volatile
        self.is_read = False
        self.fields = {}

        for field in fields:
            self.fields[field.name] = field

        self.namedtuple = namedtuple(self.name, sorted(self.fields))


class BitField():
    """Store information about a field or flag in an i2c register"""
    def __init__(self, name, mask, adapter=None, bit_width=8, read_only=False):
        self.name = name
        self.mask = mask
        self.adapter = adapter
        self.bit_width = bit_width
        self.read_only = read_only


class BitFlag(BitField):
    def __init__(self, name, bit, read_only=False):
        BitField.__init__(self, name, 1 << bit, adapter=None, bit_width=8, read_only=read_only)


class Device(object):
    def __init__(self, i2c_address, i2c_dev=None, bit_width=8, registers=None):
        self._bit_width = bit_width

        self.locked = {}
        self.registers = {}
        self.values = {}

        if type(i2c_address) is list:
            self._i2c_addresses = i2c_address
            self._i2c_address = i2c_address[0]
        else:
            self._i2c_addresses = [i2c_address]
            self._i2c_address = i2c_address

        self._i2c = i2c_dev

        if self._i2c is None:
            import smbus
            self._i2c = smbus.SMBus(1)

        for register in registers:
            self.locked[register.name] = False
            self.values[register.name] = 0
            self.registers[register.name] = register
            self.__dict__[register.name] = _RegisterProxy(self, register)

    def lock_register(self, name):
        self.locked[name] = True

    def unlock_register(self, name):
        self.locked[name] = False

    def read_register(self, name):
        register = self.registers[name]
        if register.volatile or not register.is_read:
            self.values[register.name] = self._i2c_read(register.address, register.bit_width)
            register.is_read = True
        return self.values[register.name]

    def write_register(self, name):
        register = self.registers[name]
        return self._i2c_write(register.address, self.values[register.name], register.bit_width)

    def get_addresses(self):
        return self._i2c_addresses

    def select_address(self, address):
        if address in self._i2c_addresses:
            self._i2c_address = address
            return True
        raise ValueError("Address {:02x} invalid!".format(address))

    def next_address(self):
        next_addr = self._i2c_addresses.index(self._i2c_address)
        next_addr += 1
        next_addr %= len(self._i2c_addresses)
        self._i2c_address = self._i2c_addresses[next_addr]
        return self._i2c_address

    def set(self, register, **kwargs):
        """Write one or more fields on a device register.

        Accepts multiple keyword arguments, one for each field to write.

        :param register: Name of register to write.

        """
        self.read_register(register)
        self.lock_register(register)
        for field in kwargs.keys():
            value = kwargs.get(field)
            self.set_field(register, field, value)
        self.write_register(register)
        self.unlock_register(register)

    def get(self, register):
        """Get a namedtuple containing register fields.

        :param register: Name of register to retrieve

        """
        result = {}
        self.read_register(register)
        self.lock_register(register)
        for field in self.registers[register].fields:
            result[field] = self.get_field(register, field)
        self.unlock_register(register)
        return self.registers[register].namedtuple(**result)

    def get_field(self, register, field):
        register = self.registers[register]
        field = register.fields[field]

        if not self.locked[register.name]:
            self.read_register(register.name)

        value = self.values[register.name]

        value = (value & field.mask) >> _trailing_zeros(field.mask, register.bit_width)

        if field.adapter is not None:
            try:
                value = field.adapter._decode(value)
            except ValueError as value_error:
                raise ValueError("{}: {}".format(field.name, str(value_error)))

        return value

    def set_field(self, register, field, value):
        register = self.registers[register]
        field = register.fields[field]
        shift = _trailing_zeros(field.mask, register.bit_width)

        if field.adapter is not None:
            value = field.adapter._encode(value)

        if not self.locked[register.name]:
            self.read_register(register.name)

        reg_value = self.values[register.name]

        reg_value &= ~field.mask
        reg_value |= (value << shift) & field.mask

        self.values[register.name] = reg_value

        if not self.locked[register.name]:
            self.write_register(register.name)

    def get_register(self, register):
        register = self.registers[register]
        return self._i2c_read(register.address, register.bit_width)

    def _i2c_write(self, register, value, bit_width):
        values = _int_to_bytes(value, bit_width // self._bit_width, 'big')
        values = list(values)
        self._i2c.write_i2c_block_data(self._i2c_address, register, values)

    def _i2c_read(self, register, bit_width):
        value = 0
        for x in self._i2c.read_i2c_block_data(self._i2c_address, register, bit_width // self._bit_width):
            value <<= 8
            value |= x
        return value

###########################################################################################################

class Adapter:
    """
    Must implement `_decode()` and `_encode()`.
    """
    def _decode(self, value):
        raise NotImplementedError

    def _encode(self, value):
        raise NotImplementedError


class LookupAdapter(Adapter):
    """Adaptor with a dictionary of values.
    :param lookup_table: A dictionary of one or more key/value pairs where the
           key is the human-readable value and the value is the bitwise
           register value
    """
    def __init__(self, lookup_table, snap=True):
        self.lookup_table = lookup_table
        self.snap = snap

    def _decode(self, value):
        for k, v in self.lookup_table.items():
            if v == value:
                return k
        raise ValueError("{} not in lookup table".format(value))

    def _encode(self, value):
        if self.snap and type(value) in [int, float]:
            value = min(list(self.lookup_table.keys()), key=lambda x: abs(x - value))
        return self.lookup_table[value]


class U16ByteSwapAdapter(Adapter):
    """Adaptor to swap the bytes in a 16bit integer."""
    def _byteswap(self, value):
        return (value >> 8) | ((value & 0xFF) << 8)

    def _decode(self, value):
        return self._byteswap(value)

    def _encode(self, value):
        return self._byteswap(value)

######################################################################################

import struct


__version__ = '0.0.5'


class TemperatureAdapter(Adapter):
    """
    Decode the two's compliment, right-justified, 12-bit temperature value.
    1LSb = 8degC
    """
    def _encode(self, value):
        return 0

    def _decode(self, value):
        # print(f"{value:16b}")
        output = ((value & 0xFF00) >> 8) | ((value & 0x000F) << 8)
        # print(f"{output:16b}")
        if output & (1 << 11):
            output = (output & 0xFFF) - (1 << 12)
        # print(f"{output:16b}")
        return output / 8.0


class S16ByteSwapAdapter(Adapter):
    def _encode(self, value):
        b = struct.pack("<h", value)
        return (b[0] << 8) | b[1]

    def _decode(self, value):
        b = _int_to_bytes(value, 2)
        return struct.unpack("<h", b)[0]


class LSM303D:
    def __init__(self, i2c_addr=0x1D, i2c_dev=None):
        self._i2c_addr = i2c_addr
        self._i2c_dev = i2c_dev
        self._lsm303d = Device([0x1D, 0x1E], i2c_dev=self._i2c_dev, bit_width=8, registers=(

            Register('TEMPERATURE', 0x05 | 0x80, fields=(
                BitField('temperature', 0xFFFF, adapter=TemperatureAdapter()),
            ), bit_width=16),

            # Magnetometer interrupt status
            Register('MAGNETOMETER_STATUS', 0x07, fields=(
                BitField('xdata', 0b00000001),
                BitField('ydata', 0b00000010),
                BitField('zdata', 0b00000100),
                BitField('data', 0b00001000),
                BitField('xoverrun', 0b00010000),
                BitField('yoverrun', 0b00100000),
                BitField('zoverrun', 0b01000000),
                BitField('overrun', 0b10000000),
            )),

            Register('MAGNETOMETER', 0x08 | 0x80, fields=(
                BitField('x', 0xFFFF00000000, adapter=S16ByteSwapAdapter()),
                BitField('y', 0x0000FFFF0000, adapter=S16ByteSwapAdapter()),
                BitField('z', 0x00000000FFFF, adapter=S16ByteSwapAdapter()),
            ), bit_width=8 * 6),

            Register('WHOAMI', 0x0F, fields=(
                BitField('id', 0xFF),
            )),

            Register('MAGNETOMETER_INTERRUPT', 0x12, fields=(
                BitField('enable', 0b00000001),
                BitField('enable_4d', 0b00000010),
                BitField('latch', 0b00000100),
                BitField('polarity', 0b00001000),    # 0 = active-low, 1 = active-high
                BitField('pin_config', 0b00010000),  # 0 = push-pull, 1 = open-drain
                BitField('z_enable', 0b00100000),
                BitField('y_enable', 0b01000000),
                BitField('x_enable', 0b10000000),
            )),

            Register('MAGNETOMETER_INTERRUPT_SOURCE', 0x13, fields=(
                BitField('event', 0b00000001),
                BitField('overflow', 0b00000010),
                BitField('z_negative', 0b00000100),
                BitField('y_negative', 0b00001000),
                BitField('x_negative', 0b00010000),
                BitField('z_positive', 0b00100000),
                BitField('y_positive', 0b01000000),
                BitField('x_positive', 0b10000000),
            )),

            Register('MAGNETOMETER_INTERRUPT_THRESHOLD', 0x14 | 0x80, fields=(
                BitField('threshold', 0xFFFF, adapter=U16ByteSwapAdapter()),
            ), bit_width=16),

            Register('MAGNETOMETER_OFFSET', 0x16 | 0x80, fields=(
                BitField('x', 0xFFFF00000000, adapter=S16ByteSwapAdapter()),
                BitField('y', 0x0000FFFF0000, adapter=S16ByteSwapAdapter()),
                BitField('z', 0x00000000FFFF, adapter=S16ByteSwapAdapter()),
            ), bit_width=8 * 6),

            Register('HP_ACCELEROMETER_REFERENCE', 0x1c | 0x80, fields=(
                BitField('x', 0xFF0000),
                BitField('y', 0x00FF00),
                BitField('z', 0x0000FF),
            ), bit_width=8 * 3),

            Register('CONTROL0', 0x1f, fields=(
                BitField('int2_high_pass', 0b00000001),
                BitField('int1_high_pass', 0b00000010),
                BitField('click_high_pass', 0b00000100),
                BitField('fifo_threshold', 0b00100000),
                BitField('fifo_enable', 0b01000000),
                BitField('reboot_memory', 0b10000000),
            )),

            Register('CONTROL1', 0x20, fields=(
                BitField('accel_x_enable', 0b00000001),
                BitField('accel_y_enable', 0b00000010),
                BitField('accel_z_enable', 0b00000100),
                BitField('block_data_update', 0b00001000),
                BitField('accel_data_rate_hz', 0b11110000, adapter=LookupAdapter({
                    0: 0,
                    3.125: 0b0001,
                    6.25: 0b0010,
                    12.5: 0b0011,
                    25: 0b0100,
                    50: 0b0101,
                    100: 0b0110,
                    200: 0b0111,
                    400: 0b1000,
                    800: 0b1001,
                    1600: 0b1010
                })),
            )),

            Register('CONTROL2', 0x21, fields=(
                BitField('serial_interface_mode', 0b00000001),
                BitField('accel_self_test', 0b00000010),
                BitField('accel_full_scale_g', 0b00111000, adapter=LookupAdapter({
                    2: 0b000,
                    4: 0b001,
                    6: 0b010,
                    8: 0b011,
                    16: 0b100
                })),
                BitField('accel_antialias_bw_hz', 0b11000000, adapter=LookupAdapter({
                    50: 0b11,
                    362: 0b10,
                    194: 0b01,
                    773: 0b00
                })),
            )),

            # Known in the datasheet as CTRL3
            Register('INTERRUPT1', 0x22, fields=(
                BitField('enable_fifo_empty', 0b00000001),
                BitField('enable_accel_dataready', 0b00000010),
                BitField('enable_accelerometer', 0b00000100),
                BitField('enable_magnetometer', 0b00001000),
                BitField('enable_ig2', 0b00010000),
                BitField('enable_ig1', 0b00100000),
                BitField('enable_click', 0b01000000),
                BitField('enable_boot', 0b10000000),
            )),

            # Known in the datasheet as CTRL4
            Register('INTERRUPT2', 0x23, fields=(
                BitField('enable_fifo', 0b00000001),
                BitField('enable_fifo_overrun', 0b00000010),
                BitField('enable_mag_dataready', 0b00000100),
                BitField('enable_accel_dataready', 0b00001000),
                BitField('enable_magnetometer', 0b00010000),
                BitField('enable_ig2', 0b00100000),
                BitField('enable_ig1', 0b01000000),
                BitField('enable_click', 0b10000000),
            )),

            Register('CONTROL5', 0x24, fields=(
                BitField('latch_int1', 0b00000001),
                BitField('latch_int2', 0b00000010),
                BitField('mag_data_rate_hz', 0b00011100, adapter=LookupAdapter({
                    3.125: 0b000,
                    6.25: 0b001,
                    12.5: 0b010,
                    25: 0b011,
                    50: 0b100,
                    100: 0b101,
                })),
                BitField('mag_resolution', 0b01100000),
                BitField('enable_temperature', 0b10000000),
            )),

            Register('CONTROL6', 0x25, fields=(
                BitField('mag_full_scale_gauss', 0b01100000, adapter=LookupAdapter({
                    2: 0b00,
                    4: 0b01,
                    8: 0b10,
                    12: 0b11
                })),
            )),

            Register('CONTROL7', 0x26, fields=(
                BitField('mag_mode', 0b00000011, adapter=LookupAdapter({'continuous': 0b00, 'single': 0b01, 'off': 0b10})),
                BitField('mag_lowpowermode', 0b00000100),
                BitField('temperature_only', 0b00010000),
                BitField('filter_accel', 0b00100000),
                BitField('high_pass_mode_accel', 0b11000000),  # See page 39 of lsm303d.pdf
            )),

            # Accelerometer interrupt status register
            Register('ACCELEROMETER_STATUS', 0x27, fields=(
                BitField('xdata', 0b00000001),
                BitField('ydata', 0b00000010),
                BitField('zdata', 0b00000100),
                BitField('data', 0b00001000),
                BitField('xoverrun', 0b00010000),
                BitField('yoverrun', 0b00100000),
                BitField('zoverrun', 0b01000000),
                BitField('overrun', 0b10000000)
            )),

            # X/Y/Z values from accelerometer
            Register('ACCELEROMETER', 0x28 | 0x80, fields=(
                BitField('x', 0xFFFF00000000, adapter=S16ByteSwapAdapter()),
                BitField('y', 0x0000FFFF0000, adapter=S16ByteSwapAdapter()),
                BitField('z', 0x00000000FFFF, adapter=S16ByteSwapAdapter()),
            ), bit_width=8 * 6),

            # FIFO control register
            Register('FIFO_CONTROL', 0x2e, fields=(
                BitField('mode', 0b11100000),
                BitField('threshold', 0b00011111),
            )),

            # FIFO status register
            Register('FIFO_STATUS', 0x2f, fields=(
                BitField('threshold_exceeded', 1 << 7),
                BitField('overrun', 1 << 6),
                BitField('empty', 1 << 5),
                BitField('unread_levels', 0b00011111),  # Current number of unread FIFO levels
            )),

            # 0x30: Internal interrupt generator 1: configuration register
            # 0x31: Internal interrupt generator 1: status register
            # 0x32: Internal interrupt generator 1: threshold register
            # 0x33: Internal interrupt generator 1: duration register
            Register('IG_CONFIG1', 0x30 | 0x80, fields=(
                # 0x30
                BitField('and_or_combination', 1 << 31),
                BitField('enable_6d', 1 << 30),
                BitField('z_high_enable', 1 << 29),
                BitField('z_low_enable', 1 << 28),
                BitField('y_high_enable', 1 << 27),
                BitField('y_low_enable', 1 << 26),
                BitField('x_high_enable', 1 << 25),
                BitField('x_low_enble', 1 << 24),
                # 0x31
                BitField('interrupt_status', 1 << 23),
                BitField('z_high', 1 << 22),
                BitField('z_low', 1 << 21),
                BitField('y_high', 1 << 20),
                BitField('y_low', 1 << 19),
                BitField('x_high', 1 << 18),
                BitField('x_low', 1 << 17),
                BitField('status', 1 << 16),
                # 0x32
                BitField('threshold', 0xff << 8),
                # 0x33
                BitField('duration', 0xff),
            ), bit_width=32),

            # 0x34: Internal interrupt generator 2: configuration register
            # 0x35: Internal interrupt generator 2: status register
            # 0x36: Internal interrupt generator 2: threshold register
            # 0x37: Internal interrupt generator 2: duration register
            Register('IG_CONFIG1', 0x30 | 0x80, fields=(
                # 0x34
                BitField('and_or_combination', 1 << 31),
                BitField('enable_6d', 1 << 30),
                BitField('z_high_enable', 1 << 29),
                BitField('z_low_enable', 1 << 28),
                BitField('y_high_enable', 1 << 27),
                BitField('y_low_enable', 1 << 26),
                BitField('x_high_enable', 1 << 25),
                BitField('x_low_enble', 1 << 24),
                # 0x35
                BitField('interrupt_status', 1 << 23),
                BitField('z_high', 1 << 22),
                BitField('z_low', 1 << 21),
                BitField('y_high', 1 << 20),
                BitField('y_low', 1 << 19),
                BitField('x_high', 1 << 18),
                BitField('x_low', 1 << 17),
                BitField('status', 1 << 16),
                # 0x36
                BitField('threshold', 0xff << 8),
                # 0x37
                BitField('duration', 0xff),
            ), bit_width=32),

            # 0x38: Click: configuration register
            # 0x39: Click: status register
            # 0x3A: Click: threshold register
            # 0x3B: Click: time limit register
            # 0x3C: Click: time latency register
            # 0x3D: Click: time window register
            Register('CLICK', 0x38 | 0x80, fields=(
                # 0x38
                # bits 1 << 47 and 1 << 46 are unimplemented
                BitField('z_doubleclick_enable', 1 << 45),
                BitField('z_click_enable', 1 << 44),
                BitField('y_doubleclick_enable', 1 << 43),
                BitField('y_click_enable', 1 << 42),
                BitField('x_doubleclick_enable', 1 << 41),
                BitField('x_click_enable', 1 << 40),
                # 0x39
                # bit 1 << 39 is unimplemented
                BitField('interrupt_enable', 1 << 38),
                BitField('doubleclick_enable', 1 << 37),
                BitField('click_enable', 1 << 36),
                BitField('sign', 1 << 35),  # 0 positive detection, 1 negative detection
                BitField('z', 1 << 34),
                BitField('y', 1 << 33),
                BitField('x', 1 << 32),
                # 0x3A
                BitField('threshod', 0xFF << 24),
                # 0x3B
                BitField('time_limit', 0xFF << 16),
                # 0x3C
                BitField('time_latency', 0xFF << 8),
                # 0x3D
                BitField('time_window', 0xFF),
            ), bit_width=8 * 6),

            # Controls the threshold and duration of returning to sleep mode
            Register('ACT', 0x3e | 0x80, fields=(
                BitField('threshold', 0xFF00),  # 1 LSb = 16mg
                BitField('duration', 0x00FF)    # (duration + 1) * 8/output_data_rate
            ), bit_width=16)

        ))

        self._is_setup = False

        self._accel_full_scale_g = 2
        self._mag_full_scale_guass = 2

    def set_accel_full_scale_g(self, scale):
        """Set the full scale range for the accelerometer in g

        :param scale: One of 2, 4, 6, 8 or 16 g

        """
        self._accel_full_scale_g = scale
        self._lsm303d.set('CONTROL2', accel_full_scale_g=self._accel_full_scale_g)

    def set_mag_full_scale_guass(self, scale):
        """Set the full scale range for the magnetometer in guass

        :param scale: One of 2, 4, 8 or 12 guass

        """
        self._mag_full_scale_guass = scale
        self._lsm303d.set('CONTROL6', mag_full_scale_gauss=scale)  # +-2

    def setup(self):
        if self._is_setup:
            return
        self._is_setup = True

        self._lsm303d.select_address(self._i2c_addr)

        try:
            chip = self._lsm303d.get('WHOAMI')
            if chip.id != 0x49:
                raise RuntimeError("Unable to find lsm303d on 0x{:02x}, WHOAMI returned {:02x}".format(self._i2c_addr, chip.id))
        except IOError:
            raise RuntimeError("Unable to find lsm303d on 0x{:02x}, IOError".format(self._i2c_addr))

        self._lsm303d.set('CONTROL1',
                          accel_x_enable=1,
                          accel_y_enable=1,
                          accel_z_enable=1,
                          accel_data_rate_hz=50)

        self.set_accel_full_scale_g(2)

        self._lsm303d.set('INTERRUPT1',
                          enable_fifo_empty=0,
                          enable_accel_dataready=0,
                          enable_accelerometer=0,
                          enable_magnetometer=0,
                          enable_ig2=0,
                          enable_ig1=0,
                          enable_click=0,
                          enable_boot=0)

        self._lsm303d.set('INTERRUPT2',
                          enable_fifo=0,
                          enable_fifo_overrun=0,
                          enable_mag_dataready=0,
                          enable_accel_dataready=0,
                          enable_magnetometer=0,
                          enable_ig2=0,
                          enable_ig1=0,
                          enable_click=0)

        self._lsm303d.set('CONTROL5',
                          mag_data_rate_hz=50,
                          enable_temperature=1)

        self.set_mag_full_scale_guass(2)

        self._lsm303d.set('CONTROL7', mag_mode='continuous')

    def magnetometer(self):
        """Return magnetometer x, y and z readings.

        These readings are given in guass and should be +/- the given mag_full_scale_guass value.

        """
        self.setup()
        mag = self._lsm303d.get('MAGNETOMETER')
        x, y, z = mag.x, mag.y, mag.z
        x, y, z = [(p / 32767.0) * self._mag_full_scale_guass for p in (x, y, z)]
        return x, y, z

    def accelerometer(self):
        """Return acelerometer x, y and z readings.

        These readings are given in g annd should be +/- the given accel_full_scale_g value.

        """
        self.setup()
        accel = self._lsm303d.get('ACCELEROMETER')
        x, y, z = accel.x, accel.y, accel.z
        x, y, z = [(p / 32767.0) * self._accel_full_scale_g for p in (x, y, z)]
        return x, y, z

    def temperature(self):
        """Return the temperature"""
        self.setup()
        return self._lsm303d.get('TEMPERATURE').temperature

###############################################################################

import smbus2
from time import sleep

lsm = LSM303D(0x1d, i2c_dev=smbus2.SMBus(0))  # Change to 0x1e if you have soldered the address jumper

while True:
    xyz = lsm.accelerometer()
    print(("{:+06.2f}g : {:+06.2f}g : {:+06.2f}g").format(*xyz))

    xyz = lsm.magnetometer()
    print(("{:+06.2f} : {:+06.2f} : {:+06.2f}").format(*xyz))

    print(f"Temperature: {lsm.temperature()}\n")

    sleep(0.5)
