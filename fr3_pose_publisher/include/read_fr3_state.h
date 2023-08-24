#pragma once

#include <array>
#include <iostream>
#include <string>

#include <franka/robot.h>
#include <pybind11/pybind11.h>

namespace read_fr3_state {
     /**
      * @brief Retrieves the current state of a robot using its IP address.
      * 
      * The read_state_cpp function is designed to communicate with a robot using its IP address
      * and retrieve the joint positions of the robot.
      *
      * @param robot_ip A constant reference to a std::string representing the IP address of the robot.
      * 
      * @return A std::array<double, 7> containing joint positions of the robot.
      *
      * @example
      * std::string ip = "10.42.0.2";
      * std::array<double, 7> q = read_state_cpp(ip);
      * std::cout << q << std::endl;
      *
      * @note
      * - Ensure the robot is reachable at the provided IP address for this function to work correctly.
      * - It's recommended to handle exceptions or errors that might arise if the robot is not available or if there's a communication issue.
      */
     std::array<double, 7> read_state_cpp(const std::string& robot_ip);

     /**
      * @brief PyBind11 wrapper for read_state_cpp.
      * 
      * @param robot_ip A constant reference to a std::string representing the IP address of the robot.
      * 
      * @return A pybind11::array_t<double> containing joint positions of the robot.
     */
     pybind11::array_t<double> read_state(const std::string& robot_ip);
}