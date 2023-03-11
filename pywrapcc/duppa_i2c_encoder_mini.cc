#include <duppa_i2c_encoder_mini.hh>

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(duppa_i2c_encoder_mini, m) {
    py::class_<DuPPaI2CEncoderMini>(m, "DuPPaI2CEncoderMini")
        .def(py::init<uint8_t, uint8_t>())
        .def("gconf_wr", py::overload_cast<uint8_t>(&DuPPaI2CEncoderMini::gconf_wr))
        .def("cval_rd", &DuPPaI2CEncoderMini::cval_rd)
        .def("cval_clr", &DuPPaI2CEncoderMini::cval_clr)
        .def("cmax_wr", &DuPPaI2CEncoderMini::cmax_wr)
        .def("cmin_wr", &DuPPaI2CEncoderMini::cmin_wr)
        .def("istep_wr", &DuPPaI2CEncoderMini::istep_wr)
        .def("dpperiod_wr", py::overload_cast<uint8_t>(&DuPPaI2CEncoderMini::dpperiod_wr));
}
