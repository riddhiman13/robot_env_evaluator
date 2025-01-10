#pragma once

namespace franka_o80
{
///Gripper control mode. Defines which actuators does the backend listen and which does ignore
enum class GripperMode
{
    invalid,                        ///< Backend listens only gripper_mode and reset, gripper does not move
    move,                           ///< Backend listens gripper_width and gripper_velocity, precise width control is possible in this mode
    grasp,                          ///< Backend listens gripper_width, gripper_velocity and gripper_force, precise gripping force control is possible in this mode
};

} //namespace franka_o80