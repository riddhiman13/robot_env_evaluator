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
