rclcpp = dependency('rclcpp', method : 'cmake')
std_msgs = dependency('std_msgs', method : 'cmake')

executable('example',
    sources : 'example.cc',
    # include_directories :
    dependencies : [rclcpp, std_msgs],
)
