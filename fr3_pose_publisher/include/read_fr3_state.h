#include <iostream>
#include <string>

#include <franka/robot.h>

namespace read_fr3_state {
     franka::RobotState read_state(const std::string& robot_ip);
}