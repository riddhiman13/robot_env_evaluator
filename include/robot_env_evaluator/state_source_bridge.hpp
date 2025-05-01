#ifndef ROBOT_ENV_EVALUATOR_STATE_SOURCE_BRIDGE_HPP
#define ROBOT_ENV_EVALUATOR_STATE_SOURCE_BRIDGE_HPP

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

#include <franka_hw/franka_model_interface.h>

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
         */
        virtual void getForwardKinematics(const Eigen::VectorXd& q, Eigen::MatrixXd& result);

        /**
         * @brief Get the Jacobian for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The Jacobian matrix
         */
        virtual void getJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd& result);

        /**
         * @brief Get the mass matrix for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The mass matrix
         */
        virtual void getMassMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& result);

        /**
         * @brief Get the Coriolis matrix for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The Coriolis matrix
         */
        virtual void getCoriolis(const Eigen::VectorXd& q, Eigen::MatrixXd& result);

        /**
         * @brief Get the gravity vector for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The gravity vector
         */
        virtual void getGravity(const Eigen::VectorXd& q, Eigen::MatrixXd& result);
    };

    class FrankaModelInterfaceBridge : public KinematicDataBridge {
    public:
        /**
         * @brief Construct a new FrankaModelInterfaceBridge object
         * 
         * @param model_handle The Franka model handle
         */
        FrankaModelInterfaceBridge(franka_hw::FrankaModelInterface* model_interface, const std::string& arm_id);

        /**
         * @brief Destroy the FrankaModelInterfaceBridge object
         */
        ~FrankaModelInterfaceBridge() override = default;

        /**
         * @brief Get the mass matrix for the given joint configuration
         * 
         * @param[in] q The joint configuration
         * @param[out] result The mass matrix
         */
        void getMassMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& result) override;
    
    protected:
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    };
} // namespace robot_env_evaluator

#endif // ROBOT_ENV_EVALUATOR_STATE_SOURCE_BRIDGE_HPP
