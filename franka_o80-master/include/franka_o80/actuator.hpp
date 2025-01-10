#pragma once

namespace franka_o80
{
///Actuator number
static const int actuator_number = 65;

///Actuator number corresponding to robot control mode. Contains `franka_o80::RobotMode` values
static const int robot_mode             = 0;
///Actuator number corresponding to robot control mode. Contains `franka_o80::RobotMode` values
static const int gripper_mode           = 1;
///Actuator number corresponding to error indicator. Contains `franka_o80::Error` values
static const int control_error          = 2;
///Actuator numbers corresponding to reset signal
static const int control_reset          = 3;

///Actuator number corresponding to gripper width
static const int gripper_width          = 4;
///Actuator number corresponding to gripper velocity
static const int gripper_velocity       = 5;
///Actuator number corresponding to gripper force
static const int gripper_force          = 6;
///Actuator number corresponding to gripper temperature
static const int gripper_temperature    = 7;

///Actuator numbers corresponding to robot angular positions
static const int joint_position[7]      = {  8,  9, 10, 11, 12, 13, 14 };
///Actuator numbers corresponding to robot angular velocities
static const int joint_velocity[7]      = { 15, 16, 17, 18, 19, 20, 21 };
///Actuator numbers corresponding to robot torques
static const int joint_torque[7]        = { 22, 23, 24, 25, 26, 27, 28 };

///Actuator numbers corresponding to effector position
static const int cartesian_position[3]  = { 29, 30, 31 };
///Actuator numbers corresponding to effector orientation. Contains quaternion values
static const int cartesian_orientation  = 32;
///Actuator numbers corresponding to effector translation velocity
static const int cartesian_velocity[3]  = { 33, 34, 35 };
///Actuator numbers corresponding to effector rotation velocity (WRT)
static const int cartesian_rotation[3]  = { 36, 37, 38 };

///Actuator numbers corresponding to joint-space stiffness
static const int joint_stiffness[7]     = { 39, 40, 41, 42, 43, 44, 45 };
///Actuator numbers corresponding to joint-space damping
static const int joint_damping[7]       = { 46, 47, 48, 49, 50, 51, 52 };
///Actuator numbers corresponding to cartesian stiffness (for velocity and rotation)
static const int cartesian_stiffness[6] = { 53, 54, 55, 56, 57, 58 };
///Actuator numbers corresponding to cartesian damping (for velocity and rotation)
static const int cartesian_damping[6]   = { 59, 60, 61, 62, 63, 64 };

}  // namespace franka_o80