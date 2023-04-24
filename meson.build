project('non-ROS programs', 'cpp',
    default_options : [
        'cpp_std=c++20',
        'warning_level=3',  # -Wall -Wextra -Wpedantic
    ])

inc = include_directories('inc')

subdir('nodes')
subdir('pywrapcc')
subdir('tests')
