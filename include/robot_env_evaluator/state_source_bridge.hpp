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
} // namespace robot_env_evaluator

#endif // ROBOT_ENV_EVALUATOR_STATE_SOURCE_BRIDGE_HPP
