#include <pybind11/pybind11.h>

#include "read_fr3_state.h"

namespace py = pybind11;

franka::RobotState read_state(const std::string& robot_ip) {
    franka::Robot robot(robot_ip);
    franka::RobotState state = robot.readOnce();
    return state;
}

PYBIND11_MODULE(read_fr3_state, handle)
{
  handle.doc() = "Function for reading FR3 state";
  handle.def("read_state", &read_state);
}