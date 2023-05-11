py = import('python').find_installation('python3.10')

# https://github.com/scipy/scipy/blob/main/scipy/meson.build#L93-L119
# https://github.com/mesonbuild/meson/issues/4677
pybind11 = declare_dependency(
    include_directories : include_directories('../ext/pybind11/include'),
)

py.extension_module('duppa_i2c_encoder_mini',
    sources : 'duppa_i2c_encoder_mini.cc',
    include_directories : inc,
    dependencies : [py.dependency(), pybind11],
)

py.extension_module('lsm303d',
    sources : 'lsm303d.cc',
    include_directories : inc,
    dependencies : [py.dependency(), pybind11],
)
