#ifndef ROBOT_ENV_EVALUATOR_HPP_
#define ROBOT_ENV_EVALUATOR_HPP_

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <vector>
#include <string>
#include <variant>

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
        std::string     contacted_robot_link;
        Eigen::VectorXd projector_jointspace_to_dist;
        Eigen::VectorXd projector_dist_to_jointspace;
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
                          const std::string& ee_name,
                          const std::vector<std::string>& joint_names,
                          const pinocchio::GeometryModel& collision_model,
                          const pinocchio::GeometryModel& visual_model = pinocchio::GeometryModel());
        
        /**
         * @brief Destroy the Robot Env Evaluator object
         * 
         */
        ~RobotEnvEvaluator() = default;

        /**
         * @brief The forward kinematics function for a specific joint, specified by the index
         * 
         * @param[in]  q The joint configuration
         * @param[out] T The transformation matrix of the joint
         * @param[in]  joint_index The joint index
         * 
         * As pinocchio convention, the index 0 is the base joint, and the index 1 is the first joint,
         * therefore 1 to length of [joint_names] in input is the real joint index. Default is -1, which means the end-effector.
         */
        void forwardKinematics(const Eigen::VectorXd& q,
                                     Eigen::Matrix4d& T,
                               const int joint_index = -1);
        
        /**
         * @brief The forward kinematics function for a specific frame, specified by the name
         * 
         * @param[in]  q The joint configuration
         * @param[out] T The transformation matrix of the joint
         * @param[in]  frame_name The frame name, could be joint or link
         * 
         * The name will the defined by the robot URDF file. 
         */
        void forwardKinematicsByFrameName(const Eigen::VectorXd& q,
                                                Eigen::Matrix4d& T,
                                          const std::string& frame_name);

        /**

        /**
         * @brief The jacobian function for a specific joint, specified by the index
         * 
         * @param[in]  q The joint configuration
         * @param[out] J The jacobian matrix of the joint
         * @param[in]  joint_index The joint index
         * 
         * As pinocchio convention, the index 0 is the base joint, and the index 1 is the first joint,
         * therefore 1 to length of [joint_names] in input is the real joint index. Default is -1, which means the end-effector.
         */
        void jacobian(const Eigen::VectorXd& q,
                            Eigen::MatrixXd& J,
                      const int joint_index = -1);

        /**
         * @brief Get the jacobian of specific frame, specified by the name
         * 
         * @param[in]  q The joint configuration
         * @param[out] J The jacobian matrix of the joint
         * @param[in]  frame_name The frame name, could be joint or link
         */
        void jacobianByFrameName(const Eigen::VectorXd& q,
                                       Eigen::MatrixXd& J,
                                 const std::string& frame_name);

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

        /**
         * @brief Inspect the geom_model_ and geom_data_ stored in the last computeDistances().
         * 
         * Use a series of cout command to inspect the state. Only activated when ROENVEVAL_DEBUG_MODE = ON
         * in CMake option.
         */
        void InspectGeomModelAndData(void);

        // configurations
        bool calculate_self_collision_ = false;                          ///< Whether to calculate self-collision in distance computation
        bool projector_dist_to_control_enable_ = false;                  ///< Whether to calculate the projector from distance to control space
        bool projector_dist_to_control_with_zero_orientation_  = false;  ///< Whether to use the full Jacobian and set the orientation to zero when calculating the projector
        double robust_pinv_lambda_ = 0.001;                              ///< The lambda value for the robust pseudo-inverse projector

    protected:
        /**
         * @brief Get the Jacobian of specific frame.
         * 
         * @param q The joint configuration
         * @param frame_index The frame index
         * @param J The output jacobian matrix
         * 
         * Warning, direct use of this function is not recommended because it does not apply any safety check 
         * on the frame index. Use jacobianByFrameName() instead.
         */
        void jacobianFrame(const Eigen::VectorXd& q,
                           const double frame_index,
                                 Eigen::MatrixXd& J);

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
        int ee_index_;                               ///< The internal index for end-effector.frame
        std::vector<double> joint_indices_;          ///< The joint indices for the robot model

        pinocchio::GeometryModel geom_model_;        ///< The geometry model for collision computation
        pinocchio::GeometryData geom_data_;          ///< The geometry data for collision computation
    };
} // namespace robot_env_evaluator

#endif  // ROBOT_ENV_EVALUATOR_HPP_