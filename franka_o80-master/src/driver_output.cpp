#include "../include/franka_o80/driver_output.hpp"
#include <iostream>

franka_o80::DriverOutput::DriverOutput()
{}

franka_o80::DriverOutput::DriverOutput(const States &states) : DriverInputOutput(states)
{}

void franka_o80::DriverOutput::print(bool endl)
{
    std::cout << "Driver output: ";
    std::cout << DriverInputOutput::to_string(true);
    if (endl) std::cout << std::endl;
    else std::cout << " ";
}

std::string franka_o80::DriverOutput::to_string() const
{
    return DriverInputOutput::to_string(true);
}