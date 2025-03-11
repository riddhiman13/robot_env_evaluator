#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "robot_env_evaluator/robot_presets.hpp"
#include "robot_env_evaluator/robot_env_evaluator_path.h"

namespace robot_env_evaluator
{
    void FrankaEmikaPreset::getPresetRobot(pinocchio::Model &model,
                                           std::string& ee_name,
                                           pinocchio::GeometryModel &collision_model)
    {
        // Do nothing
        const std::string robot_path = "/opt/openrobots/share/example-robot-data";
        const std::string urdf_filename = robot_path + "/robots/panda_description/urdf/panda.urdf";
        const std::string stl_filename = robot_path + "/robots/panda_description/meshes/";
        std::string srdf_filename = ROBOT_ENV_EVALUATOR_PATH;
        srdf_filename += "/scripts/panda-alternative.srdf";
        
        // model
        pinocchio::Model model_original;
        pinocchio::urdf::buildModel(urdf_filename, model_original, false);
        std::vector<pinocchio::JointIndex> joints_to_lock = {8, 9};
        Eigen::VectorXd lock_positions(9);
        lock_positions << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.03;
        pinocchio::buildReducedModel(model_original, joints_to_lock, lock_positions, model);

        // ee_name
        ee_name = "panda_hand_tcp";
        
        // collision_model
        pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, collision_model, "/opt/openrobots/share");
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

    std::unique_ptr<RobotPresetInterface> RobotPresetFactory::createRobotPreset(const std::string &robot_name)
    {
        if(robot_name == "FrankaEmika")
        {
            return std::make_unique<FrankaEmikaPreset>();
        }
        else
        {
            return nullptr;
        }
    }
}
