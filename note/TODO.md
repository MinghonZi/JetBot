- SLAM & Auto-navi
    - [ ] Fix `range_data_collator.cc Dropped x earlier points`
        - https://github.com/cartographer-project/cartographer_ros/issues/1303
    - [ ] Map drifts
    - [ ] Nav2 doesn't work
    - Refs
        - https://youtu.be/ZaiA3hWaRzE
        - https://youtu.be/jkoGkAd0GYk
        - https://github.com/turtlebot/turtlebot4/tree/humble/turtlebot4_navigation/launch
        - https://answers.ros.org/question/381396/nav2-navigation-while-mapping-setup/
        - https://github.com/mlherd/navigation2/blob/master/doc/use_cases/navigation_with_slam.md
        - https://github.com/ros-planning/navigation2/pull/1057

- Improve workspace setup
    - https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html
    - https://github.com/athackst/vscode_ros2_workspace

- [ ] Precompiled header
- [ ] Buck2 can help with multi-language interdependent monorepos
- [ ] Implement a VL53L5CX C++ driver with https://github.com/Abstract-Horizon/vl53l5cx_python as a reference
- [ ] Take the [VL53L5CX ctypes Python library](https://github.com/Abstract-Horizon/vl53l5cx_python/issues/1) as a case study to understand ctypes wrapping 

- [x] ! Rewrite non-ROS compilation process descriptions in Meson to coexist with `colcon`
    - https://answers.ros.org/question/345068/using-colcon-is-there-a-way-to-skip-a-top-cmakeliststxt-and-detect-packages-in-sub-folders/
    - https://answers.ros.org/question/306624/ignore-package-in-colcon-but-not-catkin/?answer=306633#post-id-306633
- [ ] Teleop from foxglove
- [x] How to set up clangd
    - [Allow specifying more than one compile_commands.json file](https://github.com/clangd/clangd/issues/1092)
- [ ] How to set up ccls
- [ ] How to set up rust analyzer
- [ ] How to set up pyright
- [ ] How to set up clang-format
- [ ] How to use lldb
    - How to use CodeLLDB ext
- [x] [Using Ccache](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/advanced-usage-of-colcon/)
- [x] ~~BNO055~~ An IMU with gyroscope and RPLIDAR
    - https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview
    - https://automaticaddison.com/how-to-publish-imu-data-using-ros-and-the-bno055-imu-sensor/

In Linux kernel, what is
- file descriptor
- https://en.wikipedia.org/wiki/User_space_and_kernel_space
- /dev interface
- character device file
    - major and minor device number
- https://en.wikipedia.org/wiki/Sysfs

[pywrapcc](https://github.com/google/pywrapcc)
