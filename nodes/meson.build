# TODO: https://github.com/ros2-rust/ros2_rust/blob/main/docs/building.md?plain=1#L193-L199

rclcpp = dependency('rclcpp', method : 'cmake')
std_msgs = dependency('std_msgs', method : 'cmake')
sensor_msgs = dependency('sensor_msgs', method : 'cmake')

executable('example_publisher',
    sources : 'example.cc',
    # include_directories :
    dependencies : [rclcpp, std_msgs],
)

executable('lsm303d_publisher',
    sources : 'lsm303d.cc',
    include_directories : inc,
    dependencies : [rclcpp, sensor_msgs],
)

executable('icm20600_publisher',
    sources : 'icm20600.cc',
    include_directories : inc,
    dependencies : [rclcpp, sensor_msgs],
)
