#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <affx/affine.hpp>

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace affx;


PYBIND11_MODULE(pyaffx, m) {
  m.doc() = "3D Affine transformations for C++ and Python.";

  py::class_<Affine>(m, "Affine")
    .def(py::init<double, double, double, double, double, double>(), "x"_a=0, "y"_a=0, "z"_a=0, "a"_a=0, "b"_a=0, "c"_a=0)
    .def(py::init<double, double, double, double, double, double, double>(), "x"_a=0, "y"_a=0, "z"_a=0, "q_w"_a=1, "q_x"_a=0, "q_y"_a=0, "q_z"_a=0)
    .def(py::init<const std::array<double, 16>&>())
    .def(py::init([](py::dict d) {
      if (d.contains("q_x")) { // Prefer quaternion construction
        return Affine(d["x"].cast<double>(), d["y"].cast<double>(), d["z"].cast<double>(), d["q_w"].cast<double>(), d["q_x"].cast<double>(), d["q_y"].cast<double>(), d["q_z"].cast<double>());
      }
      return Affine(d["x"].cast<double>(), d["y"].cast<double>(), d["z"].cast<double>(), d["a"].cast<double>(), d["b"].cast<double>(), d["c"].cast<double>());
    }))
    .def(py::init<const Affine&>()) // Copy constructor
    .def(py::self * py::self)
    .def("inverse", &Affine::inverse)
    .def("translate", &Affine::translate)
    .def("pretranslate", &Affine::pretranslate)
    .def("translation", &Affine::translation)
    .def("vector", &Affine::vector)
    .def("array", &Affine::array)
    .def("rotate", &Affine::rotate)
    .def("prerotate", &Affine::prerotate)
    .def("rotation", &Affine::rotation)
    .def("quaternion", &Affine::py_quaternion)
    .def_property("x", &Affine::x, &Affine::setX)
    .def_property("y", &Affine::y, &Affine::setY)
    .def_property("z", &Affine::z, &Affine::setZ)
    .def_property("a", &Affine::a, &Affine::setA)
    .def_property("b", &Affine::b, &Affine::setB)
    .def_property("c", &Affine::c, &Affine::setC)
    .def_property_readonly("q_w", &Affine::qW)
    .def_property_readonly("q_x", &Affine::qX)
    .def_property_readonly("q_y", &Affine::qY)
    .def_property_readonly("q_z", &Affine::qZ)
    .def("slerp", &Affine::slerp)
    .def("get_inner_random", &Affine::getInnerRandom)
    .def("__repr__", &Affine::toString)
    .def("as_dict", [](Affine self) {
      auto translation = self.translation();
      auto quaternion = self.quaternion();

      py::dict d;
      d["x"] = translation.x();
      d["y"] = translation.y();
      d["z"] = translation.z();
      d["a"] = self.a();
      d["b"] = self.b();
      d["c"] = self.c();
      d["q_w"] = quaternion.w();
      d["q_x"] = quaternion.x();
      d["q_y"] = quaternion.y();
      d["q_z"] = quaternion.z();
      return d;
    });
}
