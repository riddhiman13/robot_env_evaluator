#include "../include/franka_o80/driver_input_output.hpp"

franka_o80::DriverInputOutput::DriverInputOutput() : States(default_states())
{}

franka_o80::DriverInputOutput::DriverInputOutput(const States &states) : States(states)
{}

std::string franka_o80::DriverInputOutput::to_string(bool output) const
{
    //Status
    std::string result;
    result += "robot_mode: "; result += get(robot_mode).to_string();
    result += " gripper_mode: "; result += get(gripper_mode).to_string();
    result += " control_error: "; result += get(control_error).to_string();
    result += " control_reset: "; result += get(control_reset).to_string();

    //Gripper
    result += " gripper_width: "; result += get(gripper_width).to_string();
    result += " gripper_temperature: "; result += get(gripper_temperature).to_string();
    result += " gripper_force: "; result += get(gripper_force).to_string();

    //Robot joints
    result += " joint_position: [ "; for (size_t i = 0; i < 7; i++) { result += get(joint_position[i]).to_string(); result += " "; } result += "]";
    result += " joint_velocity: [ "; for (size_t i = 0; i < 7; i++) { result += get(joint_velocity[i]).to_string(); result += " "; } result += "]";
    result += " joint_torque: [ "; for (size_t i = 0; i < 7; i++) { result += get(joint_torque[i]).to_string(); result += " "; } result += "]";

    //Robot cartesian
    result += " cartesian_position: [ "; for (size_t i = 0; i < 3; i++) { result += get(cartesian_position[i]).to_string(); result += " "; } result += "]";
    result += " cartesian_orientation: [ "; for (size_t i = 0; i < 3; i++) { result += get(cartesian_orientation).to_string(); result += " "; } result += "]";
    result += " cartesian_velocitiy: [ "; for (size_t i = 0; i < 3; i++) { result += get(cartesian_velocity[i]).to_string(); result += " "; } result += "]";
    result += " cartesian_rotation: [ "; for (size_t i = 0; i < 3; i++) { result += get(cartesian_rotation[i]).to_string(); result += " "; } result += "]";

    return result;
}