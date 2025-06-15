/**
 * @example robot_env_evaluator_example.cpp
 * @brief Example of using the RobotEnvEvaluator to compute data.
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
#include<robot_env_evaluator/robot_env_evaluator.hpp>
#include "robot_env_evaluator/robot_env_evaluator_path.h"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <iostream>
#include <string>
#include <chrono>

int main(int argc, char **argv)
{
    // make up a urdf and its geometry model
    const std::string robot_env_evaluator_path = ROBOT_ENV_EVALUATOR_PATH;
    const std::string robot_path = robot_env_evaluator_path + "robots";
    const std::string urdf_filename = robot_env_evaluator_path + "/robots/panda_description/urdf/panda.urdf";
    const std::string srdf_filename = robot_env_evaluator_path + "/robots/panda_description/srdf/panda.srdf";

    pinocchio::Model model_original;
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model_original, false);
    std::vector<pinocchio::JointIndex> joints_to_lock = {8, 9};
    Eigen::VectorXd lock_positions(9);
    lock_positions << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.03;
    pinocchio::buildReducedModel(model_original, joints_to_lock, lock_positions, model);
    
    pinocchio::GeometryModel collision_model;
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, collision_model, robot_env_evaluator_path);
    collision_model.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_filename);
    collision_model.geometryObjects[9].disableCollision = true;
    collision_model.geometryObjects[10].disableCollision = true;
    collision_model.geometryObjects[11].disableCollision = true;
    collision_model.geometryObjects[12].disableCollision = true;
    collision_model.geometryObjects[13].disableCollision = true;
    collision_model.geometryObjects[14].disableCollision = true;
    collision_model.geometryObjects[15].disableCollision = true;
    collision_model.geometryObjects[16].disableCollision = true;

    std::cout<< "Inspection on Geometry Objects: " << std::endl;
    for(int i = 0; i < collision_model.geometryObjects.size(); i++){
        std::cout << i << ": " << collision_model.geometryObjects[i].name << "  \tactive:"<< !collision_model.geometryObjects[i].disableCollision << std::endl;
    }
    std::cout<< "Inspection on Collision Pairs: " << std::endl;
    for(int i = 0; i < collision_model.collisionPairs.size(); i++){
        std::cout << i << ": ";
        std::cout << collision_model.collisionPairs[i].first << " [" << collision_model.geometryObjects[collision_model.collisionPairs[i].first].name << "] and ";
        std::cout << collision_model.collisionPairs[i].second << " [" << collision_model.geometryObjects[collision_model.collisionPairs[i].second].name << "]" << std::endl;
    }

    // send it inside the evaluator and run with two obstacles
    std::vector<std::string> joint_names;
    joint_names.push_back("panda_joint1");
    joint_names.push_back("panda_joint2");
    joint_names.push_back("panda_joint3");
    joint_names.push_back("panda_joint4");
    joint_names.push_back("panda_joint5");
    joint_names.push_back("panda_joint6");
    joint_names.push_back("panda_joint7");
    robot_env_evaluator::RobotEnvEvaluator evaluator(model, "panda_hand_tcp", joint_names, collision_model);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    std::vector<robot_env_evaluator::obstacleInput> obstacles;
    std::vector<robot_env_evaluator::distanceResult> distances;

    coal::Sphere sphere1(0.1);
    Eigen::Matrix4d pose1 = Eigen::Matrix4d::Identity();
    pose1 << 1, 0, 0, 0.29,
             0, 1, 0, 0.27,
             0, 0, 1, 0.54,
             0, 0, 0, 1;
    obstacles.push_back({sphere1, pose1});

    // count the time of computeDistances function
    auto start = std::chrono::high_resolution_clock::now();
    evaluator.computeDistances(q, obstacles, distances);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Time taken for computeDistances: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;

    std::cout << std::endl << std::endl << "Now it is after the calculation" << std::endl;
    std::cout << "Inspect distances:" << std::endl;
    for (const auto& distance : distances){
        std::cout << "Obstacle Index: " << distance.id << std::endl;
        std::cout << "Distance: " << distance.distance << std::endl;
        std::cout << "Normal Vector: " << distance.normal_vector.transpose() << std::endl;
        std::cout << "Nearest Point robot: " << distance.nearest_point_on_robot.transpose() << std::endl;
        std::cout << "Nearest Point object: " << distance.nearest_point_on_object.transpose() << std::endl;
        std::cout << "Projector: " << distance.projector_jointspace_to_dist.transpose() << std::endl << std::endl;
    }

    q = Eigen::VectorXd::Zero(7);
    q << 2.5796, -1.0262, 1.2127, -2.3626, -2.2054, 2.2720, -0.2889;

    Eigen::MatrixXd J(6, model.nq);
    evaluator.jacobian(q, J);
    std::cout << "Jacobian: " << std::endl << J << std::endl;
}