#include "../include/franka_o80/driver.hpp"
#include <franka/exception.h>
#include <Eigen/Geometry>
#include <iostream>

void franka_o80::Driver::robot_write_output_(const franka::RobotState &robot_state)
{
    //Joints
    for (size_t i = 0; i < 7; i++)
    {
        output_.set(joint_position[i], robot_state.q[i]);
        output_.set(joint_velocity[i], robot_state.dq[i]);
        output_.set(joint_torque[i], robot_state.tau_J[i]);
        output_.set(joint_stiffness[i], input_.get(joint_stiffness[i]));
        output_.set(joint_damping[i], input_.get(joint_damping[i]));
    }
    output_.set(joint_position[6], robot_state.q[6] - M_PI / 4);

    //Cartesian
    Eigen::Affine3d transform(Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position = transform.translation();
    Eigen::Quaterniond orientation(transform.linear());
    Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Map(model_->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
    Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
    Eigen::Matrix<double, 6, 1> velocity = jacobian * joint_velocities;
    for (size_t i = 0; i < 3; i++)
    {
        output_.set(cartesian_position[i], position(i));
        output_.set(cartesian_orientation, orientation);
        output_.set(cartesian_velocity[i], velocity(i));
        output_.set(cartesian_rotation[i], velocity(i + 3));
        output_.set(cartesian_stiffness[i], input_.get(cartesian_stiffness[i]));
        output_.set(cartesian_damping[i], input_.get(cartesian_damping[i]));
        output_.set(cartesian_stiffness[i + 3], input_.get(cartesian_stiffness[i + 3]));
        output_.set(cartesian_damping[i + 3], input_.get(cartesian_damping[i + 3]));
    }

    //Other
    output_.set(control_reset, input_.get(control_reset).get_real() > 0.0 ? 1.0 : 0.0);
}

void franka_o80::Driver::robot_dummy_control_function_(const franka::RobotState &robot_state, franka::JointVelocities *velocities)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);

    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    RobotMode input_mode = input_.get(robot_mode).get_robot_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();
    bool input_stiffness_change = false;
    for (size_t i = 0; i < 7; i++)
    {
        if (joint_stiffness_(i) != input_.get(joint_stiffness[i]).get_real()) { input_stiffness_change = true; break; }
    }

    //Exit
    if (input_finished || (input_error == Error::ok && !input_reset && input_mode != RobotMode::invalid) || input_stiffness_change)
    {
        velocities->motion_finished = true;
    }

    //Writing output
    robot_write_output_(robot_state);
    output_.set(robot_mode, RobotMode::invalid);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_torque_control_function_(const franka::RobotState &robot_state, franka::Torques *torques)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);

    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    RobotMode input_mode = input_.get(robot_mode).get_robot_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();
    //I assume that stiffness and damping don't have influence on torque functions, so they are not checked

    //Exit
    bool reenter = input_mode != robot_mode_ && !
    ((robot_mode_ == RobotMode::torque || robot_mode_ == RobotMode::intelligent_position || robot_mode_ == RobotMode::intelligent_cartesian_position) &&
    (input_mode == RobotMode::torque || input_mode == RobotMode::intelligent_position || input_mode == RobotMode::intelligent_cartesian_position));
    if (reenter || input_finished || input_error != Error::ok || input_reset)
    {
        //Reading more input
        Eigen::Matrix<double, 7, 1> input_damping;
        for (size_t i = 0; i < 7; i++) input_damping(i) = input_.get(joint_damping[i]).get_real();

        //Calculating damping torque
        Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
        Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(model_->coriolis(robot_state).data());
        Eigen::Matrix<double, 7, 1> joint_torques = -(input_damping.array() * joint_velocities.array()).matrix() + coriolis;
        Eigen::Matrix<double, 7, 1>::Map(&torques->tau_J[0]) = joint_torques;
        torques->motion_finished = true;
    }
    //Intelligent position
    else if (input_mode == RobotMode::intelligent_position)
    {
        //Reading more input
        Eigen::Matrix<double, 7, 1> input_target_positions, input_stiffness, input_damping;
        for (size_t i = 0; i < 7; i++)
        {
            input_target_positions(i) = input_.get(joint_position[i]).get_real();
            input_stiffness(i) = input_.get(joint_stiffness[i]).get_real();
            input_damping(i) = input_.get(joint_damping[i]).get_real();
        }
        input_target_positions(6) = input_.get(joint_position[6]).get_real() + M_PI / 4;

        //Calculating torque
        Eigen::Matrix<double, 7, 1> joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
        Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
        Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(model_->coriolis(robot_state).data());
        Eigen::Matrix<double, 7, 1> joint_torques =
            -(input_stiffness.array() * (joint_positions - input_target_positions).array()).matrix()
            -(input_damping.array() * joint_velocities.array()).matrix()
        + coriolis;
        Eigen::Matrix<double, 7, 1>::Map(&torques->tau_J[0]) = joint_torques;
    }
    //Intelligent cartesian position
    else if (input_mode == RobotMode::intelligent_cartesian_position)
    {
        //Reading more input
        Eigen::Matrix<double, 3, 1> input_target_position;
        for (size_t i = 0; i < 3; i++) input_target_position(i) = input_.get(cartesian_position[i]).get_real();
        Eigen::Quaterniond input_target_orientation = input_.get(cartesian_orientation).get_quaternion();
        Eigen::Matrix<double, 6, 1> input_stiffness, input_damping;
        for (size_t i = 0; i < 6; i++)
        {
            input_stiffness(i) = input_.get(cartesian_stiffness[i]).get_real();
            input_damping(i) = input_.get(cartesian_damping[i]).get_real();
        }

        //Calculating torque
        Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position = transform.translation();
        Eigen::Quaterniond orientation; orientation = transform.linear();
        Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(model_->coriolis(robot_state).data());
        Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Map(model_->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - input_target_position;
        if (input_target_orientation.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
        Eigen::Quaterniond orientation_error = orientation.inverse() * input_target_orientation;
        error.tail(3) << orientation_error.x(), orientation_error.y(), orientation_error.z();
        error.tail(3) << -transform.linear() * error.tail(3);
        Eigen::Matrix<double, 7, 1> joint_torques = jacobian.transpose() * (
            -(input_stiffness.array() * error.array()).matrix()
            -(input_damping.array() * (jacobian * joint_velocities).array()).matrix()
        ) + coriolis;
        Eigen::Matrix<double, 7, 1>::Map(&torques->tau_J[0]) = joint_torques;
    }
    //Simple torque
    else
    {
        //Reading input and calculating torques
        for (size_t i = 0; i < 7; i++) torques->tau_J[i] = input_.get(joint_torque[i]).get_real();
    }
    
    //Writing output
    robot_write_output_(robot_state);
    output_.set(robot_mode, torques->motion_finished ? RobotMode::invalid : input_mode);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_position_control_function_(const franka::RobotState &robot_state, franka::JointPositions *positions)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);
    
    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    RobotMode input_mode = input_.get(robot_mode).get_robot_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();
    bool input_stiffness_change = false;
    for (size_t i = 0; i < 7; i++)
    {
        if (joint_stiffness_(i) != input_.get(joint_stiffness[i]).get_real()) { input_stiffness_change = true; break; }
    }

    //Exit
    if (input_mode != robot_mode_ || input_finished || input_error != Error::ok || input_reset || input_stiffness_change)
    {
        for (size_t i = 0; i < 7; i++) positions->q[i] = robot_state.q[i];
        positions->motion_finished = true;
    }
    //Normal
    else
    {
        for (size_t i = 0; i < 7; i++) positions->q[i] = input_.get(joint_position[i]).get_real();
        positions->q[6] = input_.get(joint_position[6]).get_real() + M_PI / 4;
    }

   //Writing output
    robot_write_output_(robot_state);
    output_.set(robot_mode, positions->motion_finished ? RobotMode::invalid : input_mode);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_velocity_control_function_(const franka::RobotState &robot_state, franka::JointVelocities *velocities)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);
    
    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    RobotMode input_mode = input_.get(robot_mode).get_robot_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();
    bool input_stiffness_change = false;
    for (size_t i = 0; i < 7; i++)
    {
        if (joint_stiffness_(i) != input_.get(joint_stiffness[i]).get_real()) { input_stiffness_change = true; break; }
    }

    //Exit
    if (input_mode != robot_mode_ || input_finished || input_error != Error::ok || input_reset || input_stiffness_change)
    {
        for (size_t i = 0; i < 7; i++) velocities->dq[i] = 0.0;
        velocities->motion_finished = true;
    }
    //Normal
    else
    {
        for (size_t i = 0; i < 7; i++) velocities->dq[i] = input_.get(joint_velocity[i]).get_real();
    }

    //Writing output
    robot_write_output_(robot_state);
    output_.set(robot_mode, velocities->motion_finished ? RobotMode::invalid : input_mode);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_cartesian_position_control_function_(const franka::RobotState &robot_state, franka::CartesianPose *positions)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);
    
    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    RobotMode input_mode = input_.get(robot_mode).get_robot_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();
    bool input_stiffness_change = false;
    for (size_t i = 0; i < 6; i++)
    {
        if (cartesian_stiffness_(i) != input_.get(cartesian_stiffness[i]).get_real()) { input_stiffness_change = true; break; }
    }

    //Exit
    if (input_mode != robot_mode_ || input_finished || input_error != Error::ok || input_reset || input_stiffness_change)
    {
        for (size_t i = 0; i < 16; i++) positions->O_T_EE[i] = robot_state.O_T_EE[i];
        for (size_t i = 0; i < 2; i++) positions->elbow[i] = robot_state.elbow[i];
        positions->motion_finished = true;
    }
    //Normal
    else
    {
        Eigen::Affine3d transform;
        transform.translation() = Eigen::Vector3d(input_.get(cartesian_position[0]).get_real(), input_.get(cartesian_position[1]).get_real(), input_.get(cartesian_position[2]).get_real());
        transform.linear() = input_.get(cartesian_orientation).get_quaternion().toRotationMatrix();
        Eigen::Matrix<double, 4, 4>::Map(&positions->O_T_EE[0]) = transform.matrix();
        positions->elbow[0] = input_.get(joint_position[2]).get_real();
        positions->elbow[1] = input_.get(joint_position[3]).get_real() > 0.0 ? 1.0 : -1.0;
    }

    //Writing output
    robot_write_output_(robot_state);
    output_.set(robot_mode, positions->motion_finished ? RobotMode::invalid : input_mode);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_cartesian_velocity_control_function_(const franka::RobotState &robot_state, franka::CartesianVelocities *velocities)
{
    //Locking
    std::lock_guard<std::mutex> guard(input_output_mutex_);
        
    //Reading input
    bool input_reset = input_.get(control_reset).get_real() > 0.0;
    RobotMode input_mode = input_.get(robot_mode).get_robot_mode();
    bool input_finished = input_finished_;
    Error input_error = output_.get(control_error).get_error();
    bool input_stiffness_change = false;
    for (size_t i = 0; i < 6; i++)
    {
        if (cartesian_stiffness_(i) != input_.get(cartesian_stiffness[i]).get_real()) { input_stiffness_change = true; break; }
    }

    //Exit
    if (input_mode != robot_mode_ || input_finished || input_error != Error::ok || input_reset || input_stiffness_change)
    {
        for (size_t i = 0; i < 6; i++) velocities->O_dP_EE[i] = 0.0;
        velocities->motion_finished = true;
    }
    //Normal
    else
    {
        for (size_t i = 0; i < 3; i++)
        {
            velocities->O_dP_EE[i] = input_.get(cartesian_velocity[i]).get_real();
            velocities->O_dP_EE[i + 3] = input_.get(cartesian_rotation[i]).get_real();
        }
        velocities->elbow[0] = input_.get(joint_position[2]).get_real();
        velocities->elbow[1] = input_.get(joint_position[3]).get_real() > 0.0 ? 1.0 : -1.0;
    }

    //Writing output
    robot_write_output_(robot_state);
    output_.set(robot_mode, velocities->motion_finished ? RobotMode::invalid : input_mode);
    output_.set(control_reset, input_reset ? 1.0 : 0.0);
    if (input_reset) output_.set(control_error, Error::ok);
}

void franka_o80::Driver::robot_control_function_()
{
    Error output_error = Error::ok; //One round error, exists here because loop goes after communication
    while (true)
    {
        bool input_reset;
        RobotMode input_mode;
        bool input_finished;
        Error input_error;
        std::array<double, 7> input_joint_stiffness;
        std::array<double, 6> input_cartesian_stiffness;
        {
            //Locking
            std::lock_guard<std::mutex> guard(input_output_mutex_);
            
            //Reading input
            input_reset = input_.get(control_reset).get_real() > 0.0;
            input_mode = input_.get(robot_mode).get_robot_mode();
            input_finished = input_finished_;
            input_error = output_.get(control_error).get_error();
            for (size_t i = 0; i < 7; i++) input_joint_stiffness[i] = input_.get(joint_stiffness[i]).get_real();
            for (size_t i = 0; i < 6; i++) input_cartesian_stiffness[i] = input_.get(cartesian_stiffness[i]).get_real();
            
            //Writing output
            if (input_reset) { output_.set(control_error, Error::ok); input_error = Error::ok; }
            else if (input_error == Error::ok && output_error != Error::ok) { output_.set(control_error, output_error); input_error = output_error; }
        }

        //Entering control loop
        try
        {
            //Resetting
            if (input_finished) return;
            else if (input_reset) robot_->automaticErrorRecovery();
            
            //Setting per-loop constants
            joint_stiffness_ = Eigen::Matrix<double, 7, 1>::Map(&input_joint_stiffness[0]);
            robot_->setJointImpedance(input_joint_stiffness);
            cartesian_stiffness_ = Eigen::Matrix<double, 6, 1>::Map(&input_cartesian_stiffness[0]);
            robot_->setCartesianImpedance(input_cartesian_stiffness);
            robot_mode_ = input_mode;


            //Entering loop
            Driver *driver = this;
            output_error = Error::ok;
            if (input_reset || input_error != Error::ok || input_mode == RobotMode::invalid) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointVelocities
                {
                    franka::JointVelocities velocities(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_dummy_control_function_(robot_state, &velocities);
                    return velocities;
                });
            else if (input_mode == RobotMode::torque || input_mode == RobotMode::intelligent_position || input_mode == RobotMode::intelligent_cartesian_position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                });
            else if (input_mode == RobotMode::torque_position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                },
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointPositions
                {
                    franka::JointPositions positions(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_position_control_function_(robot_state, &positions);
                    return positions;
                });
            else if (input_mode == RobotMode::torque_velocity) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                },
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointVelocities
                {
                    franka::JointVelocities velocities(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_velocity_control_function_(robot_state, &velocities);
                    return velocities;
                });
            else if (input_mode == RobotMode::torque_cartesian_position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                },
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::CartesianPose
                {
                    franka::CartesianPose cartesian_position(std::array<double, 16>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::array<double, 2>({0.0, 0.0}));
                    driver->robot_cartesian_position_control_function_(robot_state, &cartesian_position);
                    return cartesian_position;
                });
            else if (input_mode == RobotMode::torque_cartesian_velocity) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::Torques
                {
                    franka::Torques torques(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_torque_control_function_(robot_state, &torques);
                    return torques;
                },
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::CartesianVelocities
                {
                    franka::CartesianVelocities cartesian_velocity(std::array<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::array<double, 2>({0.0, 0.0}));
                    driver->robot_cartesian_velocity_control_function_(robot_state, &cartesian_velocity);
                    return cartesian_velocity;
                });
            else if (input_mode == RobotMode::position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointPositions
                {
                    franka::JointPositions positions(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_position_control_function_(robot_state, &positions);
                    return positions;
                });
            else if (input_mode == RobotMode::velocity) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::JointVelocities
                {
                    franka::JointVelocities velocities(std::array<double, 7>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
                    driver->robot_velocity_control_function_(robot_state, &velocities);
                    return velocities;
                });
            else if (input_mode == RobotMode::cartesian_position) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::CartesianPose
                {
                    franka::CartesianPose cartesian_position(std::array<double, 16>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::array<double, 2>({0.0, 0.0}));
                    driver->robot_cartesian_position_control_function_(robot_state, &cartesian_position);
                    return cartesian_position;
                });
            else if (input_mode == RobotMode::cartesian_velocity) robot_->control(
                [driver](const franka::RobotState &robot_state, franka::Duration) -> franka::CartesianVelocities
                {
                    franka::CartesianVelocities cartesian_velocity(std::array<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::array<double, 2>({0.0, 0.0}));
                    driver->robot_cartesian_velocity_control_function_(robot_state, &cartesian_velocity);
                    return cartesian_velocity;
                });
        }
        catch (franka::CommandException &e)
        {
            output_error = Error::robot_command_exception;
            std::cout << e.what() << std::endl;
        }
        catch (franka::ControlException &e)
        {
            output_error = Error::robot_control_exception;
            std::cout << e.what() << std::endl;
        }
        catch (franka::InvalidOperationException &e)
        {
            output_error = Error::robot_invalid_operation_exception;
            std::cout << e.what() << std::endl;
        }
        catch (franka::NetworkException &e)
        {
            output_error = Error::robot_network_exception;
            std::cout << e.what() << std::endl;
        }
        catch (franka::RealtimeException &e)
        {
            output_error = Error::robot_realtime_exception;
            std::cout << e.what() << std::endl;
        }
        catch (std::invalid_argument &e)
        {
            output_error = Error::robot_invalid_argument_exception;
            std::cout << e.what() << std::endl;
        }
        catch (...)
        {
            output_error = Error::robot_other_exception;
        }

        //Waiting if failed
        if (output_error != Error::ok) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void franka_o80::Driver::gripper_control_function_()
{
    Error output_error = Error::ok; //one round error
    double previous_input_width = std::numeric_limits<double>::quiet_NaN();
    
    while (true)
    {
        //Reading state
        double output_width;
        double output_temperature;
        try
        {
            franka::GripperState state = gripper_->readOnce(); 
            output_width = state.width;
            output_temperature = state.temperature;
        }
        catch (franka::NetworkException &e)
        {
            if (output_error == Error::ok) output_error = Error::gripper_network_exception;
            std::cout << e.what() << std::endl;
        }
        catch (franka::InvalidOperationException &e)
        {
            if (output_error == Error::ok) output_error = Error::gripper_invalid_operation_exception;
            std::cout << e.what() << std::endl;
        }
        catch (...)
        {
            if (output_error == Error::ok) output_error = Error::gripper_invalid_operation_exception;
        }

        //Communicating
        bool input_reset;
        GripperMode input_mode;
        bool input_finished;
        Error input_error;
        double input_width;
        double input_velocity;
        double input_force;
        {
            //Locking
            std::lock_guard<std::mutex> guard(input_output_mutex_);

            //Reading input
            input_reset = input_.get(control_reset).get_real() > 0.0;
            input_mode = input_.get(gripper_mode).get_gripper_mode();
            input_finished = input_finished_;
            input_error = output_.get(control_error).get_error();
            input_width = input_.get(gripper_width).get_real();
            input_velocity = input_.get(gripper_velocity).get_real();
            input_force = input_.get(gripper_force).get_real();

            //Writing output
            output_.set(gripper_temperature, output_temperature);
            output_.set(gripper_width, output_width);
            output_.set(gripper_velocity, input_velocity);
            output_.set(gripper_force, input_force);
            output_.set(control_reset, input_reset ? 1.0 : 0.0);
            if (input_reset) { output_.set(control_error, Error::ok); input_error = Error::ok; }
            else if (input_error == Error::ok && output_error != Error::ok) { output_.set(control_error, output_error); input_error = output_error; }
            if (input_reset || input_finished || input_error != Error::ok) { output_.set(gripper_mode, GripperMode::invalid); input_mode = GripperMode::invalid; }
            else output_.set(gripper_mode, input_mode);
        }
        
        //Action
        output_error = Error::ok;
        if (input_finished) return;
        else if (!input_reset && input_error == Error::ok && input_mode != GripperMode::invalid && input_width != previous_input_width) try
        {
            if (input_mode == GripperMode::grasp) gripper_->grasp(input_width, input_velocity, input_force, 0.1, 0.1);
            else gripper_->move(input_width, input_velocity);
        }
        catch (franka::NetworkException &e)
        {
            output_error = Error::gripper_network_exception;
        }
        catch (franka::InvalidOperationException)
        {
            output_error = Error::gripper_invalid_operation_exception;
        }
        catch (...)
        {
            output_error = Error::gripper_invalid_operation_exception;
        }
        previous_input_width = input_width;
    }
}

franka_o80::Driver::Driver(std::string ip) : ip_(ip)
{
}

void franka_o80::Driver::start()
{
    if (started_) return;
    started_ = true;

    //Initialize robot
    robot_ = std::unique_ptr<franka::Robot>(new franka::Robot(ip_));
    model_ = std::unique_ptr<franka::Model>(new franka::Model(robot_->loadModel()));
    robot_->automaticErrorRecovery();
    franka::RobotState robot_state = robot_->readOnce();
    if (robot_state.robot_mode != franka::RobotMode::kIdle) throw std::runtime_error("Driver cannot be started in current robot mode");
    robot_write_output_(robot_state);
    robot_->setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    //Initialize gripper
    gripper_ = std::unique_ptr<franka::Gripper>(new franka::Gripper(ip_));
    franka::GripperState gripper_state = gripper_->readOnce();
    output_.set(gripper_width, gripper_state.width);
    output_.set(gripper_temperature, gripper_state.temperature);
    gripper_->homing();
    
    //Start
    input_ = output_;
    robot_control_thread_ = std::thread([](Driver *driver) -> void { driver->robot_control_function_(); }, this);
    gripper_control_thread_ = std::thread([](Driver *driver) -> void { driver->gripper_control_function_(); }, this);
    ok_ = true;
}

void franka_o80::Driver::set(const DriverInput &input)
{
    if (!ok_) throw std::runtime_error("Driver is not properly initialized");
    {
        std::lock_guard<std::mutex> guard(input_output_mutex_);
        input_ = input;
    }
}

franka_o80::DriverOutput franka_o80::Driver::get()
{
    if (!ok_) throw std::runtime_error("Driver is not properly initialized");
    DriverOutput output;
    {
        std::lock_guard<std::mutex> guard(input_output_mutex_);
        output = output_;
    }
    return output;
}

void franka_o80::Driver::stop()
{
    if (!started_) return;
    started_ = false;

    //Finalizing robot
    {
        std::lock_guard<std::mutex> guard(input_output_mutex_);
        input_finished_ = true;
    }
    if (robot_control_thread_.joinable()) robot_control_thread_.join();
    
    //Finalizing gripper
    if (ok_) gripper_->stop();
    if (gripper_control_thread_.joinable()) gripper_control_thread_.join();
    
    ok_ = false;
}