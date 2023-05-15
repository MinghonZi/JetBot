project('Autonomous Mobile Robot', 'cpp',
    default_options : [
        'cpp_std=c++20',
        'warning_level=3',  # -Wall -Wextra -Wpedantic
        'default_library=static',
    ])

inc = include_directories('inc')

py = import('python').find_installation('python3.10')

# https://mesonbuild.com/Wrapdb-projects.html
# https://github.com/mesonbuild/meson/issues/4677
pybind11 = dependency('pybind11')

subdir('nodes')
subdir('pywrapcc')
subdir('tests')
