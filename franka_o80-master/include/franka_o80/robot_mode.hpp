#pragma once

namespace franka_o80
{
///Robot control mode. Defines which actuators does the backend listen and which does ignore
enum class RobotMode
{
    invalid,                        ///< Backend listens only mode and reset, robot does not move
    torque,                         ///< Backend listens torques
    torque_position,                ///< Backend listens torques and joint positions
    torque_velocity,                ///< Backend listens torques and joint velocities
    torque_cartesian_position,      ///< Backend listens torques and cartesian position
    torque_cartesian_velocity,      ///< Backend listens torques and cartesian velocities
    position,                       ///< Backend listens joint positions
    velocity,                       ///< Backend listens joint velocities
    cartesian_position,             ///< Backend listens cartesian positions
    cartesian_velocity,             ///< Backend listens cartesian velocities
    intelligent_position,           ///< Backend listens joint positions, but calculates torques itself. In practice, the robot moves smoothly im this mode
    intelligent_cartesian_position  ///< Backend listens cartesian positions, but calculates torques itself. In practice, the robot moves smoothly im this mode
};

} //namespace franka_o80