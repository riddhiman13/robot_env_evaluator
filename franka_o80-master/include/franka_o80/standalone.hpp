#pragma once

#include "driver.hpp"
#include "state.hpp"
#include "queue.hpp"
#include "state.hpp"
#include <o80/states.hpp>
#include <o80/standalone.hpp>

namespace franka_o80
{
///Standalone, runs encapsulates backend and driver
class Standalone : public o80::Standalone<queue_size, actuator_number, Driver, State, o80::VoidExtendedState>
{
public:
    ///Creates standalone
    ///@param driver_ptr `Driver` to be maintained
    ///@param frequency Frequency of reading and writing to `Driver`, needs to be 1000.0
    ///@param segment_id Identifier of shared memory
    Standalone(std::shared_ptr<Driver> driver_ptr, double frequency, std::string segment_id);
    ///Converts array of states to driver input
    ///@param states Array of states
    DriverInput convert(const o80::States<actuator_number, State> &states);
    ///Converts driver output to array of states
    ///@param driver_output Driver output
    o80::States<actuator_number, State> convert(const DriverOutput &driver_output);
};

///Starts standalone
///@param segment_id Shared memory identifier, needs to be the same in frontend and backend
///@param ip IPv4 address of the robot
void start_standalone(std::string segment_id, std::string ip);
///Checks whether standalone is running
///@param segment_id Shared memory identifier
bool standalone_is_running(std::string segment_id);
///Gently stops standalone
///@param segment_id Shared memory identifier
void please_stop(std::string segment_id);
///Stops standalone
///@param segment_id Shared memory identifier
void stop_standalone(std::string segment_id);

} // namespace franka_o80