project('JetBot', 'cpp',
    default_options : [
        'cpp_std=c++20',
        'warning_level=3',  # -Wall -Wextra -Wpedantic
        'default_library=static',
    ])

inc = include_directories('inc')

py = import('python').find_installation('python3.10')

# https://mesonbuild.com/Wrapdb-projects.html
# https://github.com/scipy/scipy/blob/main/scipy/meson.build#L93-L119
# https://github.com/mesonbuild/meson/issues/4677
pybind11 = declare_dependency(
    include_directories : include_directories('../ext/pybind11/include'),
)

subdir('nodes')
subdir('pywrapcc')
subdir('tests')
