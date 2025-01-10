#pragma once

#include "states.hpp"
#include "actuator.hpp"
#include <o80/states.hpp>
#include <string>

namespace franka_o80
{
///Input or output of `Driver`
class DriverInputOutput : public franka_o80::States
{
protected:
    DriverInputOutput();
    DriverInputOutput(const franka_o80::States &states);
    std::string to_string(bool output) const;
};
}  // namespace franka_o80