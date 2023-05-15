project('Autonomous Guided Vehicle', 'cpp',
    default_options : [
        'cpp_std=c++20',
        'warning_level=3',  # -Wall -Wextra -Wpedantic
        'default_library=static',
    ])

cpp = meson.get_compiler('cpp')

cv = dependency('opencv4')

cuda = dependency('cuda', version : '>=10', modules : ['cublas'])
nvinfer = cpp.find_library('nvinfer')
nvinfer_plugin = cpp.find_library('nvinfer_plugin')
nvparsers = cpp.find_library('nvparsers')
tensorrt = declare_dependency(
    dependencies: [cuda, nvinfer, nvinfer_plugin, nvparsers])

py37 = import('python').find_installation('python3.7')
pybind11 = dependency('pybind11')

inc = include_directories('inc')

subdir('nodes')
subdir('tests')
