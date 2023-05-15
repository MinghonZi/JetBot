rclcpp = dependency('rclcpp', method : 'cmake')
std_msgs = dependency('std_msgs', method : 'cmake')
sensor_msgs = dependency('sensor_msgs', method : 'cmake')
geometry_msgs = dependency('geometry_msgs', method : 'cmake')
cv_bridge = dependency('cv_bridge', method : 'cmake')

executable('controller',
    sources : 'controller.cc',
    dependencies : [
        rclcpp,
        sensor_msgs,
        geometry_msgs,
        # https://mesonbuild.com/Python-module.html#dependency
        pybind11,
        py37.dependency(embed:true)],
)

executable('duck_detector',
    sources : 'duck_detector.cc',
    include_directories : inc,
    dependencies : [tensorrt, cv, cv_bridge, rclcpp, std_msgs, sensor_msgs],
)
