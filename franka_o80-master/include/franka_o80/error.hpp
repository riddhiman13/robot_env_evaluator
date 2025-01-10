#pragma once

namespace franka_o80 {

///Backend error indicator
enum class Error
{
    ok,                                 ///< No errors
    robot_command_exception,            ///< franka::Robot has thrown franka::CommandException
    robot_control_exception,            ///< franka::Robot has thrown franka::ControlException
    robot_invalid_operation_exception,  ///< franka::Robot has thrown franka::InvalidOperationException
    robot_network_exception,            ///< franka::Robot has thrown franka::NetworkException
    robot_realtime_exception,           ///< franka::Robot has thrown franka::RealtimeException
    robot_invalid_argument_exception,   ///< franka::Robot has thrown std::invalid_argument exception
    robot_other_exception,              ///< franka::Robot has thrown other exception
    gripper_command_exception,          ///< franka::Gripper has thrown franka::CommandException
    gripper_network_exception,          ///< franka::Gripper has thrown franka::NetworkException
    gripper_invalid_operation_exception,///< franka::Gripper has thrown franka::InvalidOperationException
    gripper_other_exception             ///< franka::Gripper has thrown other exception
};

} // namespace franka_o80