#include "../include/franka_o80/state.hpp"
#include <stdexcept>

franka_o80::State::State()
{
    set_real(0.0);
}

franka_o80::State::State(double value)
{
    set_real(value);
}

franka_o80::State::State(const Eigen::Quaterniond &value)
{
    set_quaternion(value);
}

franka_o80::State::State(const Eigen::Matrix<double, 4, 1> &value)
{
    set_wxyz(value);
}

franka_o80::State::State(const Eigen::Matrix<double, 3, 1> &value)
{
    set_euler(value);
}

franka_o80::State::State(RobotMode value)
{
    set_robot_mode(value);
}

franka_o80::State::State(GripperMode value)
{
    set_gripper_mode(value);
}

franka_o80::State::State(Error value)
{
    set_error(value);
}

franka_o80::State::State(const State &state)
{
    if (state.typ_ == Type::real) set_real(state.value_.real);
    else if (state.typ_ == Type::quaternion) set_quaternion(state.get_quaternion());
    else if (state.typ_ == Type::robot_mode) set_robot_mode(state.value_.robot_mode);
    else if (state.typ_ == Type::gripper_mode) set_gripper_mode(state.value_.gripper_mode);
    else set_error(state.value_.error);
}

void franka_o80::State::set_real(double value)
{
    typ_ = Type::real;
    value_.real = value;
}

void franka_o80::State::set_quaternion(const Eigen::Quaterniond &value)
{
    typ_ = Type::quaternion;
    value_.quaternion[0] = value.w();
    value_.quaternion[1] = value.x();
    value_.quaternion[2] = value.y();
    value_.quaternion[3] = value.z();
}

void franka_o80::State::set_wxyz(const Eigen::Matrix<double, 4, 1> &value)
{
    typ_ = Type::quaternion;
    for (size_t i = 0; i < 4; i++) value_.quaternion[i] = value(i);
}

void franka_o80::State::set_euler(const Eigen::Matrix<double, 3, 1> &value)
{
    typ_ = Type::quaternion;
    Eigen::Quaterniond q = Eigen::AngleAxisd(value(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(value(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(value(2), Eigen::Vector3d::UnitX());
    value_.quaternion[0] = q.w();
    value_.quaternion[1] = q.x();
    value_.quaternion[2] = q.y();
    value_.quaternion[3] = q.z();
}

void franka_o80::State::set_robot_mode(RobotMode value)
{
    typ_ = Type::robot_mode;
    value_.robot_mode = value;
}


void franka_o80::State::set_gripper_mode(GripperMode value)
{
    typ_ = Type::gripper_mode;
    value_.gripper_mode = value;
}

void franka_o80::State::set_error(Error value)
{
    typ_ = Type::error;
    value_.error = value;
}

franka_o80::State::Type franka_o80::State::get_type() const
{
    return typ_;
}

double franka_o80::State::get_real() const
{
    if (typ_ != Type::real) throw std::runtime_error("franka_o80: State::get_real error");
    return value_.real;
}

Eigen::Quaterniond franka_o80::State::get_quaternion() const
{
    if (typ_ == Type::real && value_.real == 0.0) return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); //Unitialized 'States' contain invalid zero values. That's ok
    if (typ_ != Type::quaternion) throw std::runtime_error("franka_o80: State::get_quaternion error");
    Eigen::Quaterniond value;
    value.w() = value_.quaternion[0];
    value.x() = value_.quaternion[1];
    value.y() = value_.quaternion[2];
    value.z() = value_.quaternion[3];
    return value;
}

Eigen::Matrix<double, 4, 1> franka_o80::State::get_wxyz() const
{
    if (typ_ == Type::real && value_.real == 0.0) return Eigen::Matrix<double, 4, 1>(1.0, 0.0, 0.0, 0.0);
    if (typ_ != Type::quaternion) throw std::runtime_error("franka_o80: State::get_wxyz error");
    Eigen::Matrix<double, 4, 1> value;
    for (size_t i = 0; i < 4; i++) value(i) = value_.quaternion[i];
    return value;
}

Eigen::Matrix<double, 3, 1> franka_o80::State::get_euler() const
{
    if (typ_ == Type::real && value_.real == 0.0) return Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
    if (typ_ != Type::quaternion) throw std::runtime_error("franka_o80: State::get_euler error");
    Eigen::Quaterniond value;
    value.w() = value_.quaternion[0];
    value.x() = value_.quaternion[1];
    value.y() = value_.quaternion[2];
    value.z() = value_.quaternion[3];
    return value.toRotationMatrix().eulerAngles(2, 1, 0);
}

franka_o80::RobotMode franka_o80::State::get_robot_mode() const
{
    if (typ_ == Type::real && value_.real == 0.0) return RobotMode::invalid;
    if (typ_ != Type::robot_mode) throw std::runtime_error("franka_o80: State::get_robot_mode error");
    return value_.robot_mode;
}

franka_o80::GripperMode franka_o80::State::get_gripper_mode() const
{
    if (typ_ == Type::real && value_.real == 0.0) return GripperMode::invalid;
    if (typ_ != Type::gripper_mode) throw std::runtime_error("franka_o80: State::get_gripper_mode error");
    return value_.gripper_mode;
}

franka_o80::Error franka_o80::State::get_error() const
{
    if (typ_ == Type::real && value_.real == 0.0) return Error::ok;
    if (typ_ != Type::error) throw std::runtime_error("franka_o80: State::get_error error");
    return value_.error;
}

std::string franka_o80::State::to_string() const
{
    if (typ_ == Type::real) return std::to_string(value_.real);
    else if (typ_ == Type::quaternion)
    {
        return "[" + std::to_string(value_.quaternion[0]) + " " + std::to_string(value_.quaternion[1]) + " " + std::to_string(value_.quaternion[2]) + " " + std::to_string(value_.quaternion[3]) + "]";
    }
    else if (typ_ == Type::robot_mode)
    {
        switch (value_.robot_mode)
        {
        case RobotMode::torque:
            return "RobotMode::torque";
            break;
        case RobotMode::torque_position:
            return "RobotMode::torque_position";
            break;
        case RobotMode::torque_velocity:
            return "RobotMode::torque_velocity";
            break;
        case RobotMode::torque_cartesian_position:
            return "RobotMode::torque_cartesian_position";
            break;
        case RobotMode::torque_cartesian_velocity:
            return "RobotMode::torque_cartesian_velocity";
            break;
        case RobotMode::position:
            return "RobotMode::position";
            break;
        case RobotMode::velocity:
            return "RobotMode::velocity";
            break;
        case RobotMode::cartesian_position:
            return "RobotMode::cartesian_position";
            break;
        case RobotMode::cartesian_velocity:
            return "RobotMode::cartesian_velocity";
            break;
        case RobotMode::intelligent_position:
            return "RobotMode::intelligent_position";
            break;
        case RobotMode::intelligent_cartesian_position:
            return "RobotMode::intelligent_cartesian_position";
            break;
        default:
            return "RobotMode::invalid";
        }
    }
    else if (typ_ == Type::gripper_mode)
    {
        switch (value_.gripper_mode)
        {
        case GripperMode::move:
            return "GripperMode::move";
            break;
        case GripperMode::grasp:
            return "GripperMode::grasp";
            break;
        default:
            return "GripperMode::invalid";
        }
    }
    else
    {
        switch (value_.error)
        {
        case Error::robot_command_exception:
            return "Error::robot_command_exception";
            break;
        case Error::robot_control_exception:
            return "Error::robot_control_exception";
            break;
        case Error::robot_invalid_operation_exception:
            return "Error::robot_invalid_operation_exception";
            break;
        case Error::robot_network_exception:
            return "Error::robot_network_exception";
            break;
        case Error::robot_realtime_exception:
            return "Error::robot_realtime_exception";
            break;
        case Error::robot_invalid_argument_exception:
            return "Error::robot_invalid_argument_exception";
            break;
        case Error::robot_other_exception:
            return "Error::robot_other_exception";
            break;
        case Error::gripper_command_exception:
            return "Error::gripper_command_exception";
            break;
        case Error::gripper_network_exception:
            return "Error::gripper_network_exception";
            break;
        case Error::gripper_invalid_operation_exception:
            return "Error::gripper_invalid_operation_exception";
            break;
        case Error::gripper_other_exception:
            return "Error::gripper_other_exception";
            break;
        default:
            return "Error::ok";
        }
    }
}

bool franka_o80::State::finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const State &start_state,
                  const State &current_state,
                  const State &previous_desired_state,
                  const State &target_state,
                  const o80::Speed &speed)
{
    
    if (start_state.typ_ != target_state.typ_) throw std::runtime_error("franka_o80: State::finished error");
    double time_difference_us = o80::time_diff_us(start, now);
    double value_difference;
    if (start_state.typ_ == Type::real) value_difference = abs(target_state.value_.real - start_state.value_.real);
    else if (start_state.typ_ == Type::quaternion) value_difference = start_state.get_quaternion().angularDistance(target_state.get_quaternion());
    else if (start_state.typ_ == Type::robot_mode) value_difference = abs(static_cast<double>(target_state.value_.robot_mode) - static_cast<double>(target_state.value_.robot_mode));
    else if (start_state.typ_ == Type::gripper_mode) value_difference = abs(static_cast<double>(target_state.value_.gripper_mode) - static_cast<double>(target_state.value_.gripper_mode));
    else value_difference = abs(static_cast<double>(target_state.value_.error) - static_cast<double>(target_state.value_.error));
    double duration_us = value_difference / speed.value;
    return time_difference_us > duration_us;
}

franka_o80::State franka_o80::State::intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Speed &speed)
{
    if (start_state.typ_ != target_state.typ_) throw std::runtime_error("franka_o80: State::intermediate_state(speed) error");
    double time_difference_us = o80::time_diff_us(start, now);
    if (start_state.typ_ == Type::real)
    {
        double value_difference = abs(target_state.value_.real - start_state.value_.real);
        double duration_us = value_difference / speed.value;
        if (time_difference_us > duration_us) return target_state;
        else return start_state.value_.real + time_difference_us * (target_state.value_.real - start_state.value_.real) / duration_us;
    }
    else if (start_state.typ_ == Type::quaternion)
    {
        double value_difference = start_state.get_quaternion().angularDistance(target_state.get_quaternion());
        double duration_us = value_difference / speed.value;
        if (time_difference_us > duration_us) return target_state;
        else return start_state.get_quaternion().slerp(time_difference_us / duration_us, target_state.get_quaternion());
    }
    else if (start_state.typ_ == Type::robot_mode)
    {
        if (target_state.value_.robot_mode == start_state.value_.robot_mode) return target_state; //Interpolation between equal is valid, need to be checked here because otherwise it returns invalid
        double value_difference = abs(static_cast<double>(target_state.value_.robot_mode) - static_cast<double>(start_state.value_.robot_mode));
        double duration_us = value_difference / speed.value;
        if (time_difference_us > duration_us) return target_state;
        else return RobotMode::invalid;
    }
    else if (start_state.typ_ == Type::gripper_mode)
    {
        if (target_state.value_.gripper_mode == start_state.value_.gripper_mode) return target_state; //Interpolation between equal is valid, need to be checked here because otherwise it returns invalid
        double value_difference = abs(static_cast<double>(target_state.value_.gripper_mode) - static_cast<double>(start_state.value_.gripper_mode));
        double duration_us = value_difference / speed.value;
        if (time_difference_us > duration_us) return target_state;
        else return GripperMode::invalid;
    }
    else
    {
        if (target_state.value_.error == start_state.value_.error) return target_state; //Interpolation between equal is valid, need to be checked here because otherwise it returns invalid
        double value_difference = abs(static_cast<double>(target_state.value_.error) - static_cast<double>(start_state.value_.error));
        double duration_us = value_difference / speed.value;
        if (time_difference_us > duration_us) return target_state;
        else return Error::ok;
    }
}

franka_o80::State franka_o80::State::intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Duration_us &duration)
{
    if (start_state.typ_ != target_state.typ_) throw std::runtime_error("franka_o80: State::intermediate_state(duration) error");
    double time_difference_us = o80::time_diff_us(start, now);
    if (time_difference_us > duration.value) return target_state;
    if (start_state.typ_ == Type::real)
    {
        return (start_state.value_.real * (duration.value - time_difference_us) + target_state.value_.real * time_difference_us) / duration.value;
    }
    else if (start_state.typ_ == Type::quaternion)
    {
        return start_state.get_quaternion().slerp(time_difference_us / duration.value, target_state.get_quaternion());
    }
    else if (start_state.typ_ == Type::robot_mode)
    {
        if (target_state.value_.robot_mode == start_state.value_.robot_mode) return target_state; //Interpolation between equal is valid
        else return RobotMode::invalid;
    }
    else if (start_state.typ_ == Type::gripper_mode)
    {
        if (target_state.value_.gripper_mode == start_state.value_.gripper_mode) return target_state; //Interpolation between equal is valid
        else return GripperMode::invalid;
    }
    else
    {
        if (target_state.value_.error == start_state.value_.error) return target_state; //Interpolation between equal is valid
        else return Error::ok;
    }
}

franka_o80::State franka_o80::State::intermediate_state(long int start_iteration,
                           long int current_iteration,
                           const State &start_state,
                           const State &current_state,
                           const State &previous_desired_state,
                           const State &target_state,
                           const o80::Iteration &iteration)
{
    if (start_state.typ_ != target_state.typ_) throw std::runtime_error("franka_o80: State::intermediate_state(iteration) error");
    if (current_iteration >= iteration.value) return target_state;
    if (start_state.typ_ == Type::real)
    {
        return (start_state.value_.real * (current_iteration - start_iteration) + target_state.value_.real * (iteration.value - current_iteration)) / (iteration.value - start_iteration);
    }
    else if (start_state.typ_ == Type::quaternion)
    {
        return start_state.get_quaternion().slerp((static_cast<double>(current_iteration - start_iteration) / static_cast<double>(iteration.value - start_iteration)), target_state.get_quaternion());
    }
    else if (start_state.typ_ == Type::robot_mode)
    {
        if (target_state.value_.robot_mode == start_state.value_.robot_mode) return target_state; //Interpolation between equal is valid
        else return RobotMode::invalid;
    }
    else if (start_state.typ_ == Type::gripper_mode)
    {
        if (target_state.value_.gripper_mode == start_state.value_.gripper_mode) return target_state; //Interpolation between equal is valid
        else return GripperMode::invalid;
    }
    else
    {
        if (target_state.value_.error == start_state.value_.error) return target_state; //Interpolation between equal is valid
        else return Error::ok;
    }
}

bool franka_o80::operator==(const State &a, const State &b)
{
    if (a.typ_ != b.typ_) return false;
    else if (a.typ_ == State::Type::real) return a.value_.real == b.value_.real;
    else if (a.typ_ == State::Type::quaternion) return a.value_.quaternion == b.value_.quaternion;
    else if (a.typ_ == State::Type::robot_mode) return a.value_.robot_mode == b.value_.robot_mode;
    else if (a.typ_ == State::Type::gripper_mode) return a.value_.gripper_mode == b.value_.gripper_mode;
    else return a.value_.error == b.value_.error;
}

bool franka_o80::operator!=(const State &a, const State &b)
{
    return !(a == b);
}