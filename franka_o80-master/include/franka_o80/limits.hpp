#pragma once

#include <string>

namespace franka_o80
{

///Robot minimal positions for each joint
static const double joint_position_min[7] = { -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973 };
///Robot maximal positions for each joint
static const double joint_position_max[7] = { 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973 };
///Robot maximal angular velocities for each joint
static const double joint_velocity_max[7] = { 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100 };
///Robot maximal angular accelerations for each joint
static const double joint_acceleration_max[7] = { 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100 };
///Robot maximal angular jerks for each joint
static const double joint_jerk_max[7] = { 7500, 3750, 5000, 6250, 7500, 10000, 10000 };
///Robot maximal torques for each joint
static const double joint_torque_max[7] = { 87, 87, 87, 87, 12, 12, 12 };
///Robot maximal derivaties of torques for each joint
static const double joint_dtorque_max[7] = { 1000, 1000, 1000, 1000, 1000, 1000, 1000 };

}  // namespace franka_o80