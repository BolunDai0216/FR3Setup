#include <franka/gripper.h> 
#include <pybind11/pybind11.h>
#include <string>

namespace py = pybind11;

PYBIND11_MODULE(gripper, handle)
{
  py::class_<franka::Gripper>(handle, "Gripper")
    .def(py::init<const std::string&>())
    .def("homing", &franka::Gripper::homing)
    .def("grasp", &franka::Gripper::grasp)
    .def("move", &franka::Gripper::move)
    .def("stop", &franka::Gripper::stop)
    .def("readOnce", &franka::Gripper::readOnce)
    .def("serverVersion", &franka::Gripper::serverVersion);
}