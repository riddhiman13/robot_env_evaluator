/**
 * @file state_source_bridge.cpp
 * @brief Implementation of the dummy KinematicDataBridge class with unimplemented methods.
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
#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include "robot_env_evaluator/state_source_bridge.hpp"

namespace robot_env_evaluator
{
    void KinematicDataBridge::getForwardKinematics(const Eigen::VectorXd& q, Eigen::MatrixXd& result) {
        throw std::runtime_error("getForwardKinematics not implemented");
    }

    void KinematicDataBridge::getJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd& result) {
        throw std::runtime_error("getJacobian not implemented");
    }

    void KinematicDataBridge::getMassMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& result) {
        throw std::runtime_error("getMassMatrix not implemented");
    }

    void KinematicDataBridge::getCoriolis(const Eigen::VectorXd& q, Eigen::MatrixXd& result) {
        throw std::runtime_error("getCoriolis not implemented");
    }

    void KinematicDataBridge::getGravity(const Eigen::VectorXd& q, Eigen::MatrixXd& result) {
        throw std::runtime_error("getGravity not implemented");
    }
} // namespace robot_env_evaluator
