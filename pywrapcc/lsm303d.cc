#include <lsm303d.hh>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(lsm303d, m) {
    py::class_<LSM303D>(m, "LSM303D")
        .def(py::init<uint8_t, uint8_t>())
        .def("temperature_rd", &LSM303D::temperature_rd)
        .def("accelerometer_rd", &LSM303D::accelerometer_rd)
        .def("magnetometer_rd", &LSM303D::magnetometer_rd)
        .def("control1_wr", py::overload_cast<uint8_t>(&LSM303D::control1_wr))
        .def("control2_wr", py::overload_cast<uint8_t>(&LSM303D::control2_wr))
        .def("control3_wr", py::overload_cast<uint8_t>(&LSM303D::control3_wr))
        .def("control4_wr", py::overload_cast<uint8_t>(&LSM303D::control4_wr))
        .def("control5_wr", py::overload_cast<uint8_t>(&LSM303D::control5_wr))
        .def("control6_wr", py::overload_cast<uint8_t>(&LSM303D::control6_wr))
        .def("control7_wr", py::overload_cast<uint8_t>(&LSM303D::control7_wr));
}
