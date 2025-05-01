#include <robot_env_evaluator/robot_env_evaluator.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/collision/collision.hpp>

namespace robot_env_evaluator
{
    RobotEnvEvaluator::RobotEnvEvaluator(const pinocchio::Model& model,
                                         const std::string& ee_name,
                                         const std::vector<std::string>& joint_names,
                                         const pinocchio::GeometryModel& collision_model,
                                         const pinocchio::GeometryModel& visual_model)
        : model_(model), data_(model), collision_model_(collision_model), visual_model_(visual_model)
    {
        // Constructor implementation
        if (model_.existFrame(ee_name)){
            ee_index_ = model_.getFrameId(ee_name);
        }
        else{
            throw std::invalid_argument("The targeted end-effector frame [" + ee_name + "] does not exist in the model.");
        }

        // Check if the joint names are valid
        joint_indices_.clear();
        for (const auto& joint_name : joint_names)
        {
            if(model.existFrame(joint_name)){
                joint_indices_.push_back(model.getFrameId(joint_name));
            }
            else{
                throw std::invalid_argument("The joint name [" + joint_name + "] does not exist in the model.");
            }
        }
    }

    void RobotEnvEvaluator::forwardKinematics(const Eigen::VectorXd& q,
                                              Eigen::Matrix4d& T,
                                              const int joint_index /* = -1 */)
    {
        computeModelData(q);
        if (joint_index == -1) {
            T = data_.oMf[ee_index_].toHomogeneousMatrix();
        } else if (joint_index == 0) {
            T = data_.oMf[0].toHomogeneousMatrix();
        } else {
            if (joint_index < 0 || joint_index > joint_indices_.size()) {
                throw std::out_of_range("joint_index [" + std::to_string(joint_index) + "] is out of range [0, " + std::to_string(joint_indices_.size()) + "]");
            }
            T = data_.oMf[joint_indices_[joint_index - 1]].toHomogeneousMatrix();
        }
    }

    void RobotEnvEvaluator::forwardKinematicsByFrameName(const Eigen::VectorXd& q,
                                                         Eigen::Matrix4d& T,
                                                         const std::string& frame_name)
    {
        computeModelData(q);
        if (model_.existFrame(frame_name)) {
            int frame_index = model_.getFrameId(frame_name);
            T = data_.oMf[frame_index].toHomogeneousMatrix();
        } else {
            throw std::invalid_argument("forwardKinematicsByFrameName: The frame name [" + frame_name + "] does not exist in the model.");
        }
    }

    void RobotEnvEvaluator::jacobian(const Eigen::VectorXd& q,
                                     Eigen::MatrixXd& J,
                                     const int joint_index /* = -1 */)
    {
        if (joint_index == -1) {
            jacobianFrame(q, ee_index_, J);
        } else if (joint_index == 0) {
            jacobianFrame(q, 0, J);
        } else {
            if (joint_index < 0 || joint_index > joint_indices_.size()) {
                throw std::out_of_range("joint_index" + std::to_string(joint_index) + " is out of range [0, " + std::to_string(joint_indices_.size()) + "]");
            }
            jacobianFrame(q, joint_indices_[joint_index - 1], J);
        }
    }

    void RobotEnvEvaluator::jacobianByFrameName(const Eigen::VectorXd& q,
                                                      Eigen::MatrixXd& J,
                                                const std::string& frame_name)
    {
        computeModelData(q);
        if (model_.existFrame(frame_name)) {
            int frame_index = model_.getFrameId(frame_name);
            jacobianFrame(q, frame_index, J);
        } else {
            throw std::invalid_argument("The frame name [" + frame_name + "] does not exist in the model.");
        }
    }

    void RobotEnvEvaluator::jacobianFrame(const Eigen::VectorXd& q,
                                          const double frame_index,
                                                Eigen::MatrixXd& J)
    {
        computeModelData(q);
        J = pinocchio::getFrameJacobian(model_, data_, frame_index, pinocchio::LOCAL_WORLD_ALIGNED);
    }

    void RobotEnvEvaluator::computeDistances(const Eigen::VectorXd& q,
                                             const std::vector<obstacleInput>& obstacles, 
                                                   std::vector<distanceResult>& distances)
    {
        computeModelData(q);
        distances.clear();

        // 1. create a GeometryModel from the collision model. Disable all collision pairs if necessary.
        pinocchio::GeometryModel geom_model(collision_model_);
        int robot_geom_num = geom_model.ngeoms;
        if(calculate_self_collision_ == false){
            geom_model.removeAllCollisionPairs();
        }

        // 2. add the obstacles and add collision pairs for obstacles
        int i_obstacle = 0;
        for (const auto& obstacle : obstacles)
        {
            // 2.1 Construct the collision geometry
            pinocchio::GeometryObject::CollisionGeometryPtr collision_geometry;
            std::string mesh_path = "";
            Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones();
            
            std::visit([&collision_geometry, &mesh_path, &mesh_scale](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, coal::Sphere>) {
                    collision_geometry = std::make_shared<coal::Sphere>(arg);
                    mesh_path = "SPHERE";
                    mesh_scale = Eigen::Vector3d::Ones() * arg.radius;
                }
            }, obstacle.obstacle);

            pinocchio::GeometryObject geom_object(
                "Obstacle_" + std::to_string(i_obstacle),
                0,
                pinocchio::SE3(obstacle.obstacle_pose),
                collision_geometry,
                mesh_path,
                mesh_scale,
                false,
                Eigen::Vector4d(0.5, 0.5, 0.5, 1.0)
            );

            // 2.2 Add the obstacle to the geometry model
            geom_model.addGeometryObject(geom_object);

            // 2.3 Add collision pairs between the robot and the obstacle
            // for collision pairs, we always put the robot higher index body as the second one. 
            // This will help with the distance computation.
            for(int i = 0; i < robot_geom_num; i++){
                if(geom_model.geometryObjects[i].disableCollision == false){
                    geom_model.addCollisionPair(pinocchio::CollisionPair(robot_geom_num + i_obstacle, i));
                }
            }

            i_obstacle++;
        }

        // 3. create GeometryData from the GeometryModel
        pinocchio::GeometryData geom_data(geom_model);

        // 4. calculate distances between the robot and obstacles
        pinocchio::updateGeometryPlacements(model_, data_, geom_model, geom_data);
        pinocchio::computeDistances(geom_model, geom_data);

        // 5. output the distances into the output structure
        for (int i = 0; i < geom_data.distanceResults.size(); i++)
        {
            const auto& distance = geom_data.distanceResults[i];
            const auto& robot_geom = geom_model.geometryObjects[geom_model.collisionPairs[i].second];
            Eigen::Vector3d seperation_vector = (distance.nearest_points[1] - distance.nearest_points[0]).normalized();
            Eigen::MatrixXd jacobian_matrix;
            this->jacobianFrame(q, robot_geom.parentFrame, jacobian_matrix);
            Eigen::VectorXd projector_control_to_dist = seperation_vector.transpose() * jacobian_matrix.topRows(3);
            Eigen::VectorXd projector_dist_to_control;
            if (projector_dist_to_control_enable_) {
                if (projector_dist_to_control_with_zero_orientation_ == false) {
                    projector_dist_to_control = (1.0 / projector_control_to_dist.norm() / projector_control_to_dist.norm()) * projector_control_to_dist.transpose();
                } else {
                    // Compute the damped Jacobian pseudo-inverse using Jᵀ(JJᵀ + λ²I)⁻¹
                    Eigen::VectorXd seperation_twist = Eigen::VectorXd::Zero(6);
                    seperation_twist.head(3) = seperation_vector;
                    Eigen::MatrixXd JJt = jacobian_matrix * jacobian_matrix.transpose();
                    Eigen::MatrixXd damped_identity = robust_pinv_lambda_ * robust_pinv_lambda_ * Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());
                    
                    projector_dist_to_control = jacobian_matrix.transpose() * (JJt +  damped_identity).ldlt().solve(seperation_twist);
                    projector_control_to_dist = (1.0 / projector_dist_to_control.norm() * projector_dist_to_control.norm()) * projector_dist_to_control.transpose();
                }
            }
            
            distances.push_back(distanceResult{
                i,
                distance.min_distance,
                seperation_vector,
                distance.nearest_points[1],
                distance.nearest_points[0],
                robot_geom.name,
                projector_control_to_dist,
                projector_dist_to_control
            }); 
        }
    }

    void RobotEnvEvaluator::computeModelData(const Eigen::VectorXd& q)
    {
        if (buffered_q_.size() == 0 || q != buffered_q_)
        {
            // Compute the model data
            pinocchio::forwardKinematics(model_, data_, q); 
            pinocchio::computeJointJacobians(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);

            // Update the buffered joint configuration
            buffered_q_ = q;
        }
    }
} // namespace robot_env_evaluator