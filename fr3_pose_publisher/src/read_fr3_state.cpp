#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "read_fr3_state.h"

std::array<double, 7> read_state_cpp(const std::string& robot_ip) {
    franka::Robot robot(robot_ip);
    franka::RobotState state = robot.readOnce();
    std::array<double, 7> q = state.q;

    return q;
}

pybind11::array_t<double> read_state(const std::string& robot_ip) {
    auto arr = read_state_cpp(robot_ip);
    return pybind11::array_t<double>(arr.size(), arr.data());
}

PYBIND11_MODULE(read_fr3_state, handle)
{
  handle.doc() = "Function for reading FR3 state";
  handle.def("read_state", &read_state);
}