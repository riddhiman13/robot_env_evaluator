#pragma once

#include "driver_input_output.hpp"
#include <string>

namespace franka_o80
{
///Input for `Driver`
class DriverInput : public DriverInputOutput
{
public:
    ///Creates driver input
    DriverInput();
    ///Creates driver input from state
    ///@param states States to transform to driver input
    DriverInput(const franka_o80::States &states);
    ///Prints driver input to `std::cout`
    ///@param endl `true` to end output with `std::endl`
    void print(bool endl);
    ///Returs string representation of driver input
    std::string to_string() const;
};
}  // namespace franka_o80