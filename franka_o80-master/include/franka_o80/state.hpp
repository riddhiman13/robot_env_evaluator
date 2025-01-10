#pragma once

#include "robot_mode.hpp"
#include "gripper_mode.hpp"
#include "error.hpp"
#include <o80/command_types.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <array>

namespace franka_o80
{
///Actuator state. Some dynamic typization is done within the class, so it may contain real number, quaternion, `franka_o80::RobotMode`, `franka_o80::GripperMode` or `franka_o80::Error`
class State
{
friend bool operator==(const State &a, const State &b);
friend bool operator!=(const State &a, const State &b);

public:
    ///Type of state
    enum class Type
    {
        real,           ///< Real number
        quaternion,     ///< Quaternion
        robot_mode,     ///< `franka_o80::RobotMode`
        gripper_mode,   ///< `franka_o80::GripperMode`
        error           ///< `franka_o80::Error`
    };

private:
    Type typ_;
    union
    {
        double real;
        std::array<double, 4> quaternion;
        RobotMode robot_mode;
        GripperMode gripper_mode;
        Error error;
    } value_;

public:
    ///Creates state with zero real value
    State();
    ///Creates state with given real value
    State(double value);
    ///Creates state with given quaternion value
    State(const Eigen::Quaterniond &value);
    ///Creates state with given quaternion value given as wxyz
    State(const Eigen::Matrix<double, 4, 1> &value);
    ///Creates state with given quaternion value given as Euler angles
    State(const Eigen::Matrix<double, 3, 1> &value);
    ///Creates state with given robot mode value
    State(RobotMode value);
    ///Creates state with given gripper mode value
    State(GripperMode value);
    ///Creates state with given error value
    State(Error value);
    ///Copies state
    State(const State &state);
    
    ///Sets state's value to real value
    void set_real(double value);
    ///Sets state's value to quaternion value
    void set_quaternion(const Eigen::Quaterniond &value);
    ///Sets state's value to quaternion value given as wxyz
    void set_wxyz(const Eigen::Matrix<double, 4, 1> &value);
    ///Sets state's value to quaternion value given as Euler angles
    void set_euler(const Eigen::Matrix<double, 3, 1> &value);
    ///Sets state's value to robot mode value
    void set_robot_mode(RobotMode value);
    ///Sets state's value to gripper mode value
    void set_gripper_mode(GripperMode value);
    ///Sets state's value to error value
    void set_error(Error value);
    
    ///Returns state's type
    Type get_type() const;
    ///Returns real value
    double get_real() const;
    ///Returns quaternion value
    Eigen::Quaterniond get_quaternion() const;
    ///Returns quaternion value given as wxyz
    Eigen::Matrix<double, 4, 1> get_wxyz() const;
    ///Returns quaternion value given as Euler angles
    Eigen::Matrix<double, 3, 1> get_euler() const;
    ///Returns robot mode value
    RobotMode get_robot_mode() const;
    ///Returns gripper mode value
    GripperMode get_gripper_mode() const;
    ///Returns error value
    Error get_error() const;

    ///Returns string representation of state
    std::string to_string() const;

    ///Returns if target state is reached
    static bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const State &start_state,
                  const State &current_state,
                  const State &previous_desired_state,
                  const State &target_state,
                  const o80::Speed &speed);
    ///Returns intermediate state between two states
    static State intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Speed &speed);
    ///Returns intermediate state between two states
    static State intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Duration_us &duration);
    ///Returns intermediate state between two states
    static State intermediate_state(long int start_iteration,
                           long int current_iteration,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Iteration &iteration);
    ///Serializes state
    template <class Archive> void serialize(Archive &archive)
    {
        archive(typ_);
        archive(value_.quaternion);
        //Most probably o80 initializes shared memory with size at first serialization. Because states are uninitialized, this state is smaller than will be in future
        //So we write largest value. Not safe and not perfect
    }
};

bool operator==(const State &a, const State &b);  ///<Compares states
bool operator!=(const State &a, const State &b);  ///<Compares states

}  // namespace franka_o80