#include "../include/franka_o80/states.hpp"

///Returns robot's default states
franka_o80::States franka_o80::default_states()
{
    States states;
    for (size_t i = 0; i < actuator_number; i++) states.values[i] = 0.0;
    states.values[franka_o80::robot_mode]       = RobotMode::invalid;
    states.values[franka_o80::gripper_mode]     = GripperMode::invalid;
    states.values[franka_o80::control_error]    = Error::ok;
    states.values[franka_o80::control_reset]    = 0.0;
    
    states.values[franka_o80::gripper_width]        = 0.0;
    states.values[franka_o80::gripper_velocity]     = 0.1;
    states.values[franka_o80::gripper_force]        = 1.0;
    states.values[franka_o80::gripper_temperature]  = 0.0;

    states.values[franka_o80::joint_position[0]] = 0.0;
    states.values[franka_o80::joint_position[1]] = -M_PI / 4;
    states.values[franka_o80::joint_position[2]] = 0.0;
    states.values[franka_o80::joint_position[3]] = -3 * M_PI / 4;
    states.values[franka_o80::joint_position[4]] = 0.0;
    states.values[franka_o80::joint_position[5]] = M_PI / 2;
    states.values[franka_o80::joint_position[6]] = 0.0;
    states.values[franka_o80::cartesian_position[0]] = 0.30702;
    states.values[franka_o80::cartesian_position[1]] = 0.0;
    states.values[franka_o80::cartesian_position[2]] = 0.49727;
    states.values[franka_o80::cartesian_orientation] = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);

    const double joint_stiffness_multiplier = 1.0;
    states.values[franka_o80::joint_stiffness[0]] = joint_stiffness_multiplier * 600.0;
    states.values[franka_o80::joint_stiffness[1]] = joint_stiffness_multiplier * 600.0;
    states.values[franka_o80::joint_stiffness[2]] = joint_stiffness_multiplier * 600.0;
    states.values[franka_o80::joint_stiffness[3]] = joint_stiffness_multiplier * 600.0;
    states.values[franka_o80::joint_stiffness[4]] = joint_stiffness_multiplier * 250.0;
    states.values[franka_o80::joint_stiffness[5]] = joint_stiffness_multiplier * 150.0;
    states.values[franka_o80::joint_stiffness[6]] = joint_stiffness_multiplier * 50.0;
    states.values[franka_o80::joint_damping[0]] = sqrt(joint_stiffness_multiplier) * 50.0;
    states.values[franka_o80::joint_damping[1]] = sqrt(joint_stiffness_multiplier) * 50.0;
    states.values[franka_o80::joint_damping[2]] = sqrt(joint_stiffness_multiplier) * 50.0;
    states.values[franka_o80::joint_damping[3]] = sqrt(joint_stiffness_multiplier) * 50.0;
    states.values[franka_o80::joint_damping[4]] = sqrt(joint_stiffness_multiplier) * 30.0;
    states.values[franka_o80::joint_damping[5]] = sqrt(joint_stiffness_multiplier) * 25.0;
    states.values[franka_o80::joint_damping[6]] = sqrt(joint_stiffness_multiplier) * 15.0;
    const double translational_stiffness = 150.0;
    const double rotational_stiffness = 10.0;
    for (size_t i = 0; i < 3; i++)
    {
        states.values[franka_o80::cartesian_stiffness[i]]       = translational_stiffness;
        states.values[franka_o80::cartesian_stiffness[i + 3]]   = rotational_stiffness;
        states.values[franka_o80::cartesian_damping[i]]         = 2 * sqrt(translational_stiffness);
        states.values[franka_o80::cartesian_damping[i + 3]]     = 2 * sqrt(rotational_stiffness);
    }
    
    return states;
}