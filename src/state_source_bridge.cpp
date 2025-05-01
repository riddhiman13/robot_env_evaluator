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

    FrankaModelInterfaceBridge::FrankaModelInterfaceBridge(franka_hw::FrankaModelInterface* model_interface, const std::string& arm_id) {
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            throw std::runtime_error("FrankaModelInterfaceBridge: Exception getting model handle from interface: " + std::string(ex.what()));
        }
    }

    void FrankaModelInterfaceBridge::getMassMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& result) {
        std::array<double, 49> mass = model_handle_->getMass();
        result = Eigen::Matrix<double, 7, 7>(mass.data());
    }
} // namespace robot_env_evaluator
