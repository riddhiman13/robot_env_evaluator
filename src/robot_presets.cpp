/**
 * @file robot_presets.cpp
 * @brief Implementation of robot presets for the RobotEnvEvaluator.
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
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "robot_env_evaluator/robot_presets.hpp"
#include "robot_env_evaluator/robot_env_evaluator_path.h"

namespace robot_env_evaluator
{
    void FrankaEmikaPreset::getPresetRobot(pinocchio::Model &model,
                                           std::string& ee_name,
                                           std::vector<std::string>& joint_names,
                                           pinocchio::GeometryModel &collision_model)
    {
        // get the robot path
        const std::string robot_env_evaluator_path = ROBOT_ENV_EVALUATOR_PATH;
        const std::string robot_path = robot_env_evaluator_path + "robots";
        const std::string urdf_filename = robot_env_evaluator_path + "/robots/panda_description/urdf/panda.urdf";
        const std::string srdf_filename = robot_env_evaluator_path + "/robots/panda_description/srdf/panda.srdf";
        
        // model
        pinocchio::Model model_original;
        pinocchio::urdf::buildModel(urdf_filename, model_original, false);
        std::vector<pinocchio::JointIndex> joints_to_lock = {8, 9};
        Eigen::VectorXd lock_positions(9);
        lock_positions << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.03;
        pinocchio::buildReducedModel(model_original, joints_to_lock, lock_positions, model);

        // ee_name
        ee_name = "panda_hand_tcp";
        joint_names.clear();
        joint_names.push_back("panda_joint1");
        joint_names.push_back("panda_joint2");
        joint_names.push_back("panda_joint3");
        joint_names.push_back("panda_joint4");
        joint_names.push_back("panda_joint5");
        joint_names.push_back("panda_joint6");
        joint_names.push_back("panda_joint7");
        
        // collision_model
        pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, collision_model, robot_env_evaluator_path);
        collision_model.addAllCollisionPairs();
        pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_filename);
        collision_model.geometryObjects[0].disableCollision = true;
        collision_model.geometryObjects[1].disableCollision = true;
        collision_model.geometryObjects[2].disableCollision = true;
        collision_model.geometryObjects[9].disableCollision = true;
        collision_model.geometryObjects[10].disableCollision = true;
        collision_model.geometryObjects[11].disableCollision = true;
        collision_model.geometryObjects[12].disableCollision = true;
        collision_model.geometryObjects[13].disableCollision = true;
        collision_model.geometryObjects[14].disableCollision = true;
        collision_model.geometryObjects[15].disableCollision = true;
        collision_model.geometryObjects[16].disableCollision = true;
    }

    void FrankaEmikaNoHandPreset::getPresetRobot(pinocchio::Model &model,
                                            std::string& ee_name,
                                            std::vector<std::string>& joint_names,
                                            pinocchio::GeometryModel &collision_model)
    {
    // get the robot path
    const std::string robot_env_evaluator_path = ROBOT_ENV_EVALUATOR_PATH;
    const std::string robot_path = robot_env_evaluator_path + "robots";
    const std::string urdf_filename = robot_env_evaluator_path + "/robots/panda_description/urdf/panda.urdf";
    const std::string srdf_filename = robot_env_evaluator_path + "/robots/panda_description/srdf/panda.srdf";

    // model
    pinocchio::Model model_original;
    pinocchio::urdf::buildModel(urdf_filename, model_original, false);
    std::vector<pinocchio::JointIndex> joints_to_lock = {8, 9};
    Eigen::VectorXd lock_positions(9);
    lock_positions << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.03;
    pinocchio::buildReducedModel(model_original, joints_to_lock, lock_positions, model);

    // ee_name
    ee_name = "panda_hand_tcp";
    joint_names.clear();
    joint_names.push_back("panda_joint1");
    joint_names.push_back("panda_joint2");
    joint_names.push_back("panda_joint3");
    joint_names.push_back("panda_joint4");
    joint_names.push_back("panda_joint5");
    joint_names.push_back("panda_joint6");
    joint_names.push_back("panda_joint7");

    // collision_model
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, collision_model, robot_env_evaluator_path);
    collision_model.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_filename);
    collision_model.geometryObjects[0].disableCollision = true;
    collision_model.geometryObjects[1].disableCollision = true;
    collision_model.geometryObjects[2].disableCollision = true;
    collision_model.geometryObjects[9].disableCollision = true;
    collision_model.geometryObjects[10].disableCollision = true;
    collision_model.geometryObjects[11].disableCollision = true;
    collision_model.geometryObjects[12].disableCollision = true;
    collision_model.geometryObjects[13].disableCollision = true;
    collision_model.geometryObjects[14].disableCollision = true;
    collision_model.geometryObjects[15].disableCollision = true;
    collision_model.geometryObjects[16].disableCollision = true;

    // remove another three collision meshes close to hand
    collision_model.geometryObjects[6].disableCollision = true;
    collision_model.geometryObjects[7].disableCollision = true;
    collision_model.geometryObjects[8].disableCollision = true;
    }

    std::unique_ptr<RobotPresetInterface> RobotPresetFactory::createRobotPreset(const std::string &robot_name)
    {
        if(robot_name == "panda")
        {
            return std::make_unique<FrankaEmikaPreset>();
        }
        if(robot_name == "panda_no_hand")
        {
            return std::make_unique<FrankaEmikaNoHandPreset>();
        }
        else
        {
            return nullptr;
        }
    }
}
