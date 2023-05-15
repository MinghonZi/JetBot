rclcpp = dependency('rclcpp', method : 'cmake')
std_msgs = dependency('std_msgs', method : 'cmake')
sensor_msgs = dependency('sensor_msgs', method : 'cmake')
cv_bridge = dependency('cv_bridge', method : 'cmake')

executable('duck_detector',
    sources : 'duck_detector.cc',
    include_directories : inc,
    dependencies : [tensorrt, cv, cv_bridge, rclcpp, std_msgs, sensor_msgs],
)
