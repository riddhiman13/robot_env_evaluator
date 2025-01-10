#include "../include/franka_o80/standalone.hpp"

franka_o80::Standalone::Standalone(std::shared_ptr<Driver> driver_ptr, double frequency, std::string segment_id)
    : o80::Standalone<queue_size, actuator_number, Driver, State, o80::VoidExtendedState>(driver_ptr, frequency, segment_id)
{
}

franka_o80::DriverInput franka_o80::Standalone::convert(const States &states)
{
    return states;
}

franka_o80::States franka_o80::Standalone::convert(const DriverOutput &driver_output)
{
    return driver_output;
}

void franka_o80::start_standalone(std::string segment_id, std::string ip)
{
    o80::start_standalone<Driver, Standalone, std::string>(segment_id, 1000.0, false, std::string(ip));
}

bool franka_o80::standalone_is_running(std::string segment_id)
{
    return o80::standalone_is_running(segment_id);
}

void franka_o80::please_stop(std::string segment_id)
{
    o80::please_stop(segment_id);
}

void franka_o80::stop_standalone(std::string segment_id)
{
    o80::stop_standalone(segment_id);
}