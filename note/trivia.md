# Learning Resources

[嵌入式系统及物联网应用](https://theembeddedsystem.readthedocs.io/en/latest/index.html)

[Low Level Learning](https://www.youtube.com/@LowLevelLearning)



# ROS

[PointCloud2 explaination](https://youtu.be/lTami8Igc3c)

Publishing transform data
`ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link camera_link`
Receiving transform data
`ros2 run tf2_ros tf2_echo base_link camera_link`

ros2 pkg prefix cartographer_ros

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro /opt/ros/humble/share/cartographer_ros/urdf/backpack_3d.urdf )"

ros2 run tf2_ros static_transform_publisher 0 0 0 -1.57 0 -1.57 base_link vl53l5cx_link
ros2 run tf2_ros static_transform_publisher 0 0 0 1.57 0 -1.57 base_link mpu6050_link



# Communication Protocols

[libgpio](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/about/)

An [Introduction](https://www.beyondlogic.org/an-introduction-to-chardev-gpio-and-libgpiod-on-the-raspberry-pi/) to chardev GPIO and Libgpiod on the Raspberry PI

[Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio)

[spidev](https://elixir.bootlin.com/linux/latest/source/drivers/spi/spidev.c)

## I2C/SMBus

[Implementing I2C device drivers in userspace](https://docs.kernel.org/i2c/dev-interface.html)

[SMBus](https://www.kernel.org/doc/Documentation/i2c/smbus-protocol)

[Linux i2c Subsystem](https://i2c.wiki.kernel.org)
- i2c-tools > libi2c

[Interfacing with I2C Devices](https://elinux.org/Interfacing_with_I2C_Devices)

[Internet I2C Device Directory](https://i2cdevices.org/)

[I2C addresses](https://learn.adafruit.com/i2c-addresses/the-list)

[Three major methods of communicating with i2c devices from userspace](https://stackoverflow.com/a/38382649/20015297)

[I2C/SMBUS fault codes](https://www.kernel.org/doc/html/next/i2c/fault-codes.html)

>VL53L5CX requires a firmware upload on startup, and it's slow. Add a baudrate to the i2c line in /boot/config.txt to speed it up:
>```
>dtparam=i2c_arm=on,i2c_arm_baudrate=400000
>```
>Note: The default baudrate is 200000 (200KHz) and a typical maximum for most devices is 400000 (400KHz), but you can also use 1000000 (1MHz) if you're just driving VL53L5CX sensors.



# C/C++

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

[span: the best span](https://brevzin.github.io/c++/2018/12/03/span-best-span/)

[pybind11 built-in conversions](https://pybind11.readthedocs.io/en/stable/advanced/cast/overview.html#conversion-table)
[pybind11 built-in C++ to Python exception translation](https://pybind11.readthedocs.io/en/stable/advanced/exceptions.html)

[CMake single-configurations and multi-configurations](https://stackoverflow.com/questions/24460486/cmake-build-type-is-not-being-used-in-cmakelists-txt)

[`c++` is a standard name of a C++ compiler on a system](https://stackoverflow.com/a/11222582/20015297)



# IMU

> The lidar is going to give you so much more accuracy than imu or odometry you can likely buy any imu you want (even a 10 dollar one) and have reasonable success. I've found that the slam algorithms mostly need imu/odometry to simply say "the robot is moving" or "the robot is stationary" and all your positional accuracy comes from the lidar and slam algorithm

A [stack](https://github.com/CCNYRoboticsLab/imu_tools) contains IMU-related filters and visualizers.

VectorNav [VN-100](https://vectornav.com/products/detail/vn-100)

Cheap:
- ICM20948
- BNO085
- MPU9250

https://inertiallabs.com/
https://ceva-dsp.com/



# Camera

https://gitlab.com/boldhearts/ros2_v4l2_camera
https://reddit.com/r/robotics/comments/tx48u1/comment/i3jogcy/
https://e-consystems.com/See3CAM-USB-3-Camera.asp

[Cannot access CSI cam inside a container because the absent of `nvarguscamerasrc`](https://github.com/opendatacam/opendatacam/issues/178)

[Accelerated GStreamer](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/Multimedia/AcceleratedGstreamer.html)
https://github.com/george-hawkins/pololu-romi-jetbot/blob/master/jetson-nano-gstreamer.md
https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera#CameraGuide-CSIcamera.3



# Build
[meson](https://brennan.io/2020/05/08/meson/)

[meson-python](https://github.com/mesonbuild/meson-python)

[Scikit Build Proposal](https://iscinumpy.gitlab.io/post/scikit-build-proposal/)

[nanobind](https://nanobind.readthedocs.io/en/latest/index.html)



# [Connect to Eduroam](https://campus-rover.gitbook.io/lab-notebook/infrastructure/linux_terminal_eduroam_setup#connection-to-eduroam)

> Run the following command to get the names of your wireless devices.
> ```
> ip link show
> ```
> Running this command will list all of your networking devices. You will want to note the name of your wireless networking device, for this tutorial I will assume the wireless device's name will be wlan0 as it is named on the Raspberry Pi 3b+, however you will want to substitute this for the name of your wireless device if your's differs.
> Next you will run the following commands to connect your machine to eduroam.
> ```
> nmcli con add type wifi con-name "eduroam" ifname wlan0 ssid "eduroam" wifi-sec.key-mgmt wpa-eap 802-1x.identity "exampleemail@brandeis.edu" 802-1x.password "examplepassword123" 802-1x.system-ca-certs yes 802-1x.eap "peap" 802-1x.phase2-auth mschapv2
> ```
> ```
> nmcli connection up eduroam --ask
> ```
> You may then be prompted to enter in the wifi username and password, however the fields should already be filled in and you will just need to press enter.



# Build OpenCV

[A blog](https://www.simonwenkel.com/notes/software_libraries/opencv/compiling-opencv.html)

[Compilation flags](https://github.com/opencv/opencv/blob/725e440d278aca07d35a5e8963ef990572b07316/CMakeLists.txt)

[Some required libraries on debian](https://gist.github.com/changx03/b4aa9bb2827217c3a6a7e08365441417)
- QT: `apt install qt5-default`
- or GTK: `apt install libgtk-3-dev`

SIFT and SURF are [non-free algorithms](https://stackoverflow.com/a/64525431/20015297)

## opencv-python

Followed "[Manual builds](https://github.com/opencv/opencv-python#manual-builds)" instructions

[2023 April 27] https://github.com/opencv/opencv-python/pull/837

```diff
cmake_args = (
    (ci_cmake_generator if is_CI_build else [])
    + [
        # skbuild inserts PYTHON_* vars. That doesn't satisfy opencv build scripts in case of Py3
        "-DPYTHON3_EXECUTABLE=%s" % sys.executable,
        "-DPYTHON3_INCLUDE_DIR=%s" % python_include_dir,
        "-DPYTHON3_LIBRARY=%s" % python_lib_path,
        "-DBUILD_opencv_python3=ON",
        "-DBUILD_opencv_python2=OFF",
        # Disable the Java build by default as it is not needed
        "-DBUILD_opencv_java=%s" % build_java,
        # Relative dir to install the built module to in the build tree.
        # The default is generated from sysconfig, we'd rather have a constant for simplicity
        "-DOPENCV_PYTHON3_INSTALL_PATH=python",
        # Otherwise, opencv scripts would want to install `.pyd' right into site-packages,
        # and skbuild bails out on seeing that
        "-DINSTALL_CREATE_DISTRIB=ON",
        # See opencv/CMakeLists.txt for options and defaults
        "-DBUILD_opencv_apps=OFF",
        "-DBUILD_opencv_freetype=OFF",
        "-DBUILD_SHARED_LIBS=OFF",
        "-DBUILD_TESTS=OFF",
        "-DBUILD_PERF_TESTS=OFF",
        "-DBUILD_DOCS=OFF",
        "-DPYTHON3_LIMITED_API=ON",
        "-DBUILD_OPENEXR=ON",
+       "-DCMAKE_CXX_FLAGS=-I~/.local/lib/python3.7/site-packages/numpy/core/include/",
    ]
```



# Build Pytorch with CUDA support on Jetson

[Official guide](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
- Followed "Build from Source" of the "Instructions" section
    - https://github.com/pytorch/pytorch/tree/v1.10.2
- Applied the pytorch-1.10-jetpack-4.5.1.patch although the JetPack version is 4.6
- Added `python3.7 -m pip` prefix since the target Python version is 3.7
- ! Compile with Clang 8 (6 is too old, and 9 is too new; GCC can't compile)
- Installed the wheel file
- Built and installed the torchvision ([v0.11.3](https://github.com/pytorch/vision/tree/v0.11.3) is compatible with torch v1.10.2 as per the [matrix](https://github.com/pytorch/vision#installation))



# Misc

[NVIDIA Jetson](https://www.nvidia.com/en-gb/autonomous-machines/embedded-systems/)
- [Understand orders of magnitude in computer performance](https://kb.iu.edu/d/apeq#performance)
- [Jetson Nano](https://elinux.org/Jetson_Nano)
- [Jetson Nano Forum](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/jetson-nano/76)
- https://forums.developer.nvidia.com/t/jetson-nano-and-jetson-xavier-announcements/232520
- [Jetson roadmap](https://developer.nvidia.com/embedded/develop/roadmap)
- Jetson Nano Next in 2023
- [Jetson Software Roadmap for 2H-2021 and 2022](https://forums.developer.nvidia.com/t/jetson-software-roadmap-for-2h-2021-and-2022/177721)
- [Jetson Linux Archive](https://developer.nvidia.com/embedded/jetson-linux-archive)

[Allow non-root access to /ttyUSB](https://askubuntu.com/a/133244/1632699)

[Dockerfile "heredoc" notation examples](https://github.com/moby/moby/issues/34423)

[Dockerfile RUN over multiple lines without && \\](https://github.com/moby/moby/issues/16058#issuecomment-881901519)

[Why Docker does not set $USER env var](https://stackoverflow.com/a/54411816/20015297)

Keyboard libraries, e.g. pynput and keyboard, [require either X server or uinput](https://github.com/ollipal/sshkeyboard#comparison-to-other-keyboard-libraries)
- [[keyboard] Reading keypresses through SSH](https://github.com/boppreh/keyboard/issues/195)

[Creating a singleton in Python](https://stackoverflow.com/questions/6760685/creating-a-singleton-in-python)

Differential drive kinematics
- https://www.cs.columbia.edu/~allen/F19/NOTES/icckinematics.pdf
- https://learn.parallax.com/tutorials/robot/cyberbot/navigation-cyberbot/left-and-right-turns

> hatchling, hatchling, flit, and pdm-pep517 will look for `src/<package-name>` in addition to `<package-name>` without extra configuration. [Source](https://github.com/pypa/packaging-problems/issues/615#issuecomment-1257038564)

[ImportError: dynamic module does not define module export function (PyInit_python_example)](https://github.com/pybind/python_example/issues/99#issuecomment-1065104070)

[`apt-key` is deprecated. What do I do instead of `add-apt-repository`](https://unix.stackexchange.com/questions/717434/apt-key-is-deprecated-what-do-i-do-instead-of-add-apt-repository)
- `wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | sudo tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc`

[Sensors coordinate systems example](https://github.com/IntelRealSense/librealsense/issues/7568)

## Calc x,y,z coordinates for VL53L5CX data
https://forum.dronebotworkshop.com/sensors-modules/time-of-flight-tof-vl53l5cx-8x8-pixel-sensor/
https://community.st.com/s/question/0D53W000015XpcBSAS/vl53l5cx-multizone-sensor-get-xyz-of-points-relative-to-origin
https://community.st.com/s/question/0D53W00001NTlTzSAL/for-vl53l5cx-what-does-resultsdatadistancemmzonenum-exactly-mean
