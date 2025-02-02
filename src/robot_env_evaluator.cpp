// @suppress("Type mismatch")
#include <robot_env_evaluator/robot_env_evaluator.hpp>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace robot_env_evaluator
{
    RobotEnvEvaluator::RobotEnvEvaluator(const pinocchio::Model& model,
                                         const pinocchio::GeometryModel collision_model,
                                         const pinocchio::GeometryModel visual_model)
        : model_(model), data_(model), collision_model_(collision_model), visual_model_(visual_model)
    {
        // Constructor implementation
    }

    void RobotEnvEvaluator::forwardKinematics(const Eigen::VectorXd& q, 
                                              const double joint_index, 
                                                    Eigen::Matrix4d& T)
    {
        computeModelData(q);
        T = data_.oMi[joint_index].toHomogeneousMatrix();
    }

    void RobotEnvEvaluator::jacobian(const Eigen::VectorXd& q,
                                     const double joint_index,
                                           Eigen::MatrixXd& J)
    {
        computeModelData(q);
        J = pinocchio::getJointJacobian(model_, data_, joint_index, pinocchio::WORLD);
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
                geom_model.addCollisionPair(pinocchio::CollisionPair(robot_geom_num + i_obstacle, i));
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
            Eigen::Vector3d seperation_vector = (distance.nearest_points[1] - distance.nearest_points[0]).normalized();
            Eigen::MatrixXd jacobian;
            this->jacobian(q, geom_model.geometryObjects[geom_model.collisionPairs[i].second].parentJoint, jacobian);
            distances.push_back(distanceResult{
                i,
                distance.min_distance,
                seperation_vector,
                distance.nearest_points[1],
                distance.nearest_points[0],
                seperation_vector.transpose() * jacobian.topRows(3)
            });
        }
    }

    void RobotEnvEvaluator::computeModelData(const Eigen::VectorXd& q)
    {
        if (q != buffered_q_)
        {
            // Compute the model data
            pinocchio::forwardKinematics(model_, data_, q); 
            pinocchio::computeJointJacobians(model_, data_, q);

            // Update the buffered joint configuration
            buffered_q_ = q;
        }
    }
} // namespace robot_env_evaluator