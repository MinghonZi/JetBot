py = import('python').find_installation()

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
