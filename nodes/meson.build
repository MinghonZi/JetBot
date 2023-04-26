rclcpp = dependency('rclcpp', method : 'cmake')
std_msgs = dependency('std_msgs', method : 'cmake')
sensor_msgs = dependency('sensor_msgs', method : 'cmake')

executable('example',
    sources : 'example.cc',
    # include_directories :
    dependencies : [rclcpp, std_msgs],
)

executable('lsm303d',
    sources : 'lsm303d.cc',
    include_directories : inc,
    dependencies : [rclcpp, sensor_msgs],
)
