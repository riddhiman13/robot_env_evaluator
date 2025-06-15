/**
 * @file state_source_bridge.hpp
 * @brief Implementation of the KinematicDataBridge class, which provides a common interface for kinematic data retrieval.
 * 
 * If you need to implement a specific robot's kinematic data retrieval,
 * you should inherit from this class and implement the methods,
 * (maybe not in this library but somewhere else), 
 * then this lass can be standardized as API for further use.
 * 
 *  Copyright (C) 2025 Haowen Yao

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef ROBOT_ENV_EVALUATOR_STATE_SOURCE_BRIDGE_HPP
#define ROBOT_ENV_EVALUATOR_STATE_SOURCE_BRIDGE_HPP

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

namespace robot_env_evaluator
{
    /**
     * @brief The KinematicDataBridge class
     * 
     * This class provides a common interface for retrieving various kinematic data such as:
     * - Forward Kinematics
     * - Jacobian
     * - Mass Matrix
     * - Coriolis
     * - Gravity
     * 
     * All methods take the joint configuration as input and return the corresponding data as an Eigen::MatrixXd.
     * By default, all methods throw a runtime exception, and derived classes are expected to implement the functionality.
     */
    class KinematicDataBridge {
    public:
        /**
         * @brief Destroy the Kinematic Data Bridge object
         */
        virtual ~KinematicDataBridge() = default;

        /**
         * @brief Get the forward kinematics for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The forward kinematics result
         * @exception std::runtime_error If not implemented in derived class
         */
        virtual void getForwardKinematics(const Eigen::VectorXd& q, Eigen::MatrixXd& result);

        /**
         * @brief Get the Jacobian for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The Jacobian matrix
         * @exception std::runtime_error If not implemented in derived class
         */
        virtual void getJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd& result);

        /**
         * @brief Get the mass matrix for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The mass matrix
         * @exception std::runtime_error If not implemented in derived class
         */
        virtual void getMassMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& result);

        /**
         * @brief Get the Coriolis matrix for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The Coriolis matrix
         * @exception std::runtime_error If not implemented in derived class
         */
        virtual void getCoriolis(const Eigen::VectorXd& q, Eigen::MatrixXd& result);

        /**
         * @brief Get the gravity vector for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The gravity vector
         * @exception std::runtime_error If not implemented in derived class
         */
        virtual void getGravity(const Eigen::VectorXd& q, Eigen::MatrixXd& result);
    };
} // namespace robot_env_evaluator

#endif // ROBOT_ENV_EVALUATOR_STATE_SOURCE_BRIDGE_HPP
