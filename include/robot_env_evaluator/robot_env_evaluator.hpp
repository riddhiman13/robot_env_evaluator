#ifndef ROBOT_ENV_EVALUATOR_HPP_
#define ROBOT_ENV_EVALUATOR_HPP_

#include <vector>
#include <variant>

#include <pinocchio/algorithm/geometry.hpp>

// These headers are already included in previous headers, we write here to make VScode IntelliSense happy
#include <Eigen/Dense>
#include <coal/shape/geometric_shapes.h>

namespace robot_env_evaluator
{
    /**
     * @brief The Obstacle class, a variant of different coal primitive shapes
     * 
     * This will be extended to include supported shapes, now includes:
     * - Sphere
     */
    using Obstacle = std::variant<coal::Sphere>;

    /**
     * @brief Store the obstacle and its pose in the world frame.
     */
    struct obstacleInput{
        Obstacle obstacle;
        Eigen::Matrix4d obstacle_pose;
    };

    /**
     * @brief Store the distance result between objects
     */
    struct distanceResult{
        int id;
        double distance;
        Eigen::Vector3d normal_vector;
        Eigen::Vector3d nearest_point_on_robot;
        Eigen::Vector3d nearest_point_on_object;
        Eigen::VectorXd projector_jointspace_to_dist;
    };

    class RobotEnvEvaluator{
    public:
        /**
         * @brief Construct a new Robot Env Evaluator object
         * 
         * @param model The robot model
         * @param collision_model The collision model
         * @param visual_model The visual model
         * 
         * These models are supposed to be loaded from URDF files, and then activate (also deactivate) the
         * collision pairs. Also editing model is acceptable, therefore this class allows a flexible way to
         * input them.
         */
        RobotEnvEvaluator(const pinocchio::Model& model,
                          const pinocchio::GeometryModel& collision_model,
                          const pinocchio::GeometryModel& visual_model = pinocchio::GeometryModel());
        
        /**
         * @brief Destroy the Robot Env Evaluator object
         * 
         */
        ~RobotEnvEvaluator() = default;

        /**
         * @brief The forward kinematics function for a specific joint
         * 
         * @param q The joint configuration
         * @param joint_index The joint index, starting from 0 and end with degree of freedom - 1
         * @param T The transformation matrix of the joint
         */
        void forwardKinematics(const Eigen::VectorXd& q, 
                               const double joint_index, 
                                     Eigen::Matrix4d& T);

        /**
         * @brief The jacobian function for a specific joint
         * 
         * @param q The joint configuration
         * @param joint_index The joint index, starting from 0 and end with degree of freedom - 1
         * @param J The jacobian matrix of the joint
         */
        void jacobian(const Eigen::VectorXd& q,
                      const double joint_index,
                            Eigen::MatrixXd& J);

        /**
         * @brief The distance computation function between the robot and obstacles
         * 
         * @param q The joint configuration
         * @param obstacles The obstacles in the world, including the shape and pose
         * @param distances The distances output between the robot and obstacles
         */
        void computeDistances(const Eigen::VectorXd& q,
                              const std::vector<obstacleInput>& obstacles, 
                                    std::vector<distanceResult>& distances);

        // configurations
        bool calculate_self_collision_ = false;      ///< Whether to calculate self-collision in distance computation
    
    protected:
        /**
         * @brief Compute the model data from the joint configuration
         * 
         * @param q The joint configuration
         * 
         * This function is used to check if the joint configuration is changed, if yes, the computation
         * will be done.
         */
        void computeModelData(const Eigen::VectorXd& q);

        // buffers
        Eigen::VectorXd buffered_q_;                 ///< The buffered joint configuration. This is used to avoid unnecessary computation.

        pinocchio::Model model_;                     ///< The robot model
        pinocchio::Data data_;                       ///< The robot data, initialized from the model
        pinocchio::GeometryModel collision_model_;   ///< The collision model
        pinocchio::GeometryModel visual_model_;      ///< The visual model

        pinocchio::GeometryModel geom_model_;        ///< The geometry model for collision computation
        pinocchio::GeometryData geom_data_;          ///< The geometry data for collision computation
    };
} // namespace robot_env_evaluator

#endif  // ROBOT_ENV_EVALUATOR_HPP_