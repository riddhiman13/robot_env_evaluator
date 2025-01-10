#pragma once

#include "driver_input_output.hpp"
#include <string>

namespace franka_o80
{
///Output of `Driver`
class DriverOutput : public DriverInputOutput
{
public:
    ///Creates driver output
    DriverOutput();
    ///Creates driver output from state
    ///@param states States to transform to driver output
    DriverOutput(const franka_o80::States &states);
    ///Prints driver output to `std::cout`
    ///@param endl `true` to end output with `std::endl`
    void print(bool endl);
    ///Returs string representation of driver output
    std::string to_string() const;
};
}  // namespace franka_o80