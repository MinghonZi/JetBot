## TODO

In Linux kernel, what is
- file descriptor
- https://en.wikipedia.org/wiki/User_space_and_kernel_space
- /dev interface
- character device file
    - major and minor device number
- https://en.wikipedia.org/wiki/Sysfs

How to access *linux character device* within the container?

[pywrapcc](https://github.com/google/pywrapcc)



## Learning Resources

[Embedded Linux Wiki](https://elinux.org)

[嵌入式系统及物联网应用](https://theembeddedsystem.readthedocs.io/en/latest/index.html)

[Low Level Learning](https://www.youtube.com/@LowLevelLearning)

[Linux Serial Ports Using C/C++](https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/)

[Practical Guide to Bare Metal C++](https://arobenko.github.io/bare_metal_cpp/)



## Communication Protocols

[Linux I2C device interface](https://docs.kernel.org/i2c/dev-interface.html)

[Linux i2c Subsystem](https://i2c.wiki.kernel.org)
- i2c-tools > libi2c

[Interfacing with I2C Devices](https://elinux.org/Interfacing_with_I2C_Devices)

[Internet I2C Device Directory](https://i2cdevices.org/)

[I2C addresses](https://learn.adafruit.com/i2c-addresses/the-list)

[Python I2CEncoderMiniLib](https://github.com/Fattoresaimon/I2CEncoderMini/blob/1ce083be9defe9f0fb43c41e00b1812b8beac005/Python%20Raspberry%20Library/Source/i2cEncoderMiniLib.py)

[SMBus](https://www.kernel.org/doc/Documentation/i2c/smbus-protocol)

[Three major methods of communicating with i2c devices from userspace](https://stackoverflow.com/a/38382649/20015297)

[I2C/SMBUS fault codes](https://www.kernel.org/doc/html/next/i2c/fault-codes.html)

[spidev](https://elixir.bootlin.com/linux/latest/source/drivers/spi/spidev.c)

[libgpio](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/about/)

An [Introduction](https://www.beyondlogic.org/an-introduction-to-chardev-gpio-and-libgpiod-on-the-raspberry-pi/) to chardev GPIO and Libgpiod on the Raspberry PI



## C/C++

[Modern C++ Tutorial](https://github.com/changkun/modern-cpp-tutorial)

[Modern C++ Features](https://github.com/AnthonyCalandra/modern-cpp-features)

[Awesome C++](https://github.com/fffaraz/awesome-cpp)

[Trending C++ Repos](https://github.com/trending/c++)

Char is of 1 byte in C language.

[ES.31: Don’t use macros for constants or “functions”](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es31-dont-use-macros-for-constants-or-functions)

Strings are null terminating arrays of characters.

[make install C lib](https://github.com/swedishborgie/libmma8451/blob/master/Makefile)

Calling the intrinsics instead of rolling your own gives you the best performance and code density.

`__builtin_bswap32` is from gcc extensions. It is defined for unsigned int but works for signed int as well.

`cerrno` is actually /usr/include/asm/errno.h which is part of the linux headers.

`exit` provided by header file `cstdlib`.

Blender file extension [style guide](https://wiki.blender.org/wiki/Style_Guide/C_Cpp):
- C files should be named `.c` and `.h`.
- C++ files should be named `.cc` and `.hh`, although `.cpp`, `.hpp` and `.h` are sometimes used as well. As a rule of thumb, keep files in a single module consistent but use the preferred naming in new code.

`format` lib is avaliable since GCC 13.
```c++
#include <bitset>
std::bitset<32>(var)
--------------------
#include <format>
std::format("{:b}", var)
```

https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html

https://www.jetbrains.com/help/resharper/EditorConfig_CPP_CppOtherPageScheme.html#Indentation

https://code.visualstudio.com/docs/cpp/customize-default-settings-cpp#_visual-studio-code-settings

Balance `new` with `delete`, `new[]` with `delete[]`, and `malloc` with `free`. Well-written C++ will contain almost none of those; leave the responsibiltiy for dynamic memo­ry and lifetime management to suitable container or manager classes, most notably `std::vector` and `std::unique_ptr`.

[Modern C++ idiom for allocating / deallocating an I/O buffer](https://stackoverflow.com/a/35798248/20015297)

[A Buffers Library for C++20: Part 1](https://vector-of-bool.github.io/2020/08/29/buffers-1.html)

[Tips for Efficient C, Effective C, and lowering power consumption](https://embeddedgurus.com/stack-overflow/tag/i2c/)

[Why single-file headers?](https://github.com/nothings/stb#why-single-file-headers)



## Rust

[Rust Embedded](https://github.com/rust-embedded)

[Embedded Rust documentation](https://docs.rust-embedded.org/)

[Hardware Abstraction Layer (HAL) for embedded systems](https://github.com/rust-embedded/embedded-hal/)

[The Book](https://doc.rust-lang.org/book/)

[The Cargo Book](https://doc.rust-lang.org/cargo/)

[A Comprehensive list of all Rust learning material](https://www.reddit.com/r/rust/comments/1143yar/a_comprehensive_list_of_all_rust_learning_material/)



## Algos

### SLAM

https://github.com/cartographer-project/cartographer

https://github.com/introlab/rtabmap



## Misc

[Jetson Nano](https://elinux.org/Jetson_Nano)

[Connect to Eduroam](https://campus-rover.gitbook.io/lab-notebook/infrastructure/linux_terminal_eduroam_setup#connection-to-eduroam)

[Dockerfile "heredoc" notation examples](https://github.com/moby/moby/issues/34423)

[Dockerfile RUN over multiple lines without && \\](https://github.com/moby/moby/issues/16058#issuecomment-881901519)

[Why Docker does not set $USER env var](https://stackoverflow.com/a/54411816/20015297)
