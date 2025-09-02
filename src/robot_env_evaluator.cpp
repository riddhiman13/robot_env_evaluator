/**
 * @file robot_env_evaluator.cpp
 * @brief Implementation of the RobotEnvEvaluator class for evaluating robot environments.
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
#include <robot_env_evaluator/robot_env_evaluator.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/collision/collision.hpp>

#include <iostream>
#include <chrono>

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

    RobotEnvEvaluator::RobotEnvEvaluator(const RobotEnvEvaluator& other)
        : model_(other.model_), 
          data_(other.model_),  // data_ must be initialized from model_, not copied directly
          collision_model_(other.collision_model_), 
          visual_model_(other.visual_model_)
    {
        // Copy configuration variables
        calculate_self_collision_ = other.calculate_self_collision_;
        projector_dist_to_control_enable_ = other.projector_dist_to_control_enable_;
        projector_dist_to_control_with_zero_orientation_ = other.projector_dist_to_control_with_zero_orientation_;
        broad_phase_search_enable_ = other.broad_phase_search_enable_;
        robust_pinv_lambda_ = other.robust_pinv_lambda_;
        broad_phase_collision_padding_ = other.broad_phase_collision_padding_;
        
        // Copy computed indices and buffers
        ee_index_ = other.ee_index_;
        joint_indices_ = other.joint_indices_;
        buffered_q_ = other.buffered_q_;
        
        // If the other object has computed data for a specific configuration, 
        // we need to recompute it to ensure data_ is consistent with buffered_q_
        if (buffered_q_.size() > 0) {
            pinocchio::forwardKinematics(model_, data_, buffered_q_); 
            pinocchio::computeJointJacobians(model_, data_, buffered_q_);
            pinocchio::updateFramePlacements(model_, data_);
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
        pinocchio::GeometryModel broad_phase_geom_model(collision_model_);
        int robot_geom_num = geom_model.ngeoms;
        if(calculate_self_collision_ == false){
            geom_model.removeAllCollisionPairs();
            broad_phase_geom_model.removeAllCollisionPairs();
        }

        // 2. add the obstacles and add collision pairs for obstacles
        int i_obstacle = 0;
        for (const auto& obstacle : obstacles)
        {
            // 2.1 Construct the collision geometry
            pinocchio::GeometryObject::CollisionGeometryPtr collision_geometry;
            pinocchio::GeometryObject::CollisionGeometryPtr collision_geometry_broad;
            std::string mesh_path = "";
            Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones();
            double broad_phase_collision_padding = broad_phase_collision_padding_;

            std::visit([&collision_geometry, &collision_geometry_broad, &mesh_path, &mesh_scale, &broad_phase_collision_padding](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, coal::Sphere>) {
                    collision_geometry = std::make_shared<coal::Sphere>(arg);
                    collision_geometry_broad = std::make_shared<coal::Sphere>(arg.radius + broad_phase_collision_padding);
                    mesh_path = "SPHERE";
                    mesh_scale = Eigen::Vector3d::Ones() * arg.radius;
                }
            }, obstacle.obstacle);

            // 2.2 Add the obstacle to the narrow phase geometry model
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
            geom_model.addGeometryObject(geom_object);

            // 2.3 Add the obstacle to the broad phase geometry model
            pinocchio::GeometryObject broad_geom_object(
                "Obstacle_" + std::to_string(i_obstacle),
                0,
                pinocchio::SE3(obstacle.obstacle_pose),
                collision_geometry_broad,
                mesh_path,
                mesh_scale,
                false,
                Eigen::Vector4d(0.5, 0.5, 0.5, 1.0)
            );
            broad_phase_geom_model.addGeometryObject(broad_geom_object);
            
            // 2.4 Add collision pairs between the robot and the obstacle for broad phase search
            // for collision pairs, we always put the robot higher index body as the second one.
            // This will help with the distance computation.
            for(int i = 0; i < robot_geom_num; i++){
                if(broad_phase_geom_model.geometryObjects[i].disableCollision == false){
                    broad_phase_geom_model.addCollisionPair(pinocchio::CollisionPair(robot_geom_num + i_obstacle, i));
                }
            }

            i_obstacle++;
        }

        // 3. Broad phase search for collision pairs, then use this to construct the narrow phase collision pairs
        pinocchio::GeometryData broad_phase_geom_data(broad_phase_geom_model);
        if (broad_phase_search_enable_) {
            // 3.1 Broad phase search for collision pairs
            pinocchio::updateGeometryPlacements(model_, data_, broad_phase_geom_model, broad_phase_geom_data);
            pinocchio::computeCollisions(broad_phase_geom_model, broad_phase_geom_data);
            // 3.2 Filter the collision pairs based on the broad phase distance threshold
            for (size_t i = 0; i < broad_phase_geom_data.collisionResults.size(); ++i) {
                const auto& result = broad_phase_geom_data.collisionResults[i];
                if (result.isCollision()) {
                    geom_model.addCollisionPair(pinocchio::CollisionPair(
                        broad_phase_geom_model.collisionPairs[i].first,
                        broad_phase_geom_model.collisionPairs[i].second));
                }
            }
        } else {
            for(int j_obstacle = 0; j_obstacle < i_obstacle; j_obstacle++)
            {
                for(int i = 0; i < robot_geom_num; i++){
                    if(geom_model.geometryObjects[i].disableCollision == false){
                        geom_model.addCollisionPair(pinocchio::CollisionPair(robot_geom_num + j_obstacle, i));
                    }
                }
            }
        }

        // 4. Narrow phase calculate distances between the robot and obstacles
        pinocchio::GeometryData geom_data(geom_model);
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