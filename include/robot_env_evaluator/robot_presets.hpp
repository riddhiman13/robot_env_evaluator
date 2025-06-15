/**
 * @file robot_presets.hpp
 * @brief Implementation of robot presets for the RobotEnvEvaluator class.
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
#ifndef ROBOT_ENV_EVALUATOR_ROBOT_PRESETS_HPP
#define ROBOT_ENV_EVALUATOR_ROBOT_PRESETS_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <vector>
#include <string>

namespace robot_env_evaluator{
    /**
     * @brief the Interface class for the robot preset. Inherit this class to create a new robot preset.
     * 
     */
    class RobotPresetInterface{
    public:
        RobotPresetInterface() = default;
        virtual ~RobotPresetInterface() = default;
        
        /**
         * @brief Get the Preset Robot object
         * 
         * @param[out] model The robot model output
         * @param[out] ee_name The end-effector name output
         * @param[out] joint_names The joint names output
         * @param[out] collision_model The collision model output
         * 
         * This is a pure virtual function, which should be implemented with exact same signature in the derived class.
         */
        virtual void getPresetRobot(pinocchio::Model& model, 
                                    std::string& ee_name,
                                    std::vector<std::string>& joint_names,
                                    pinocchio::GeometryModel& collision_model) = 0;

        virtual bool isFrankaRobot() const { return false; } ///< Check if the robot is a Franka robot
    };

    /**
     * @brief The Franka Emika Panda robot preset
     * 
     */
    class FrankaEmikaPreset : public RobotPresetInterface{
    public:
        FrankaEmikaPreset() = default;
        ~FrankaEmikaPreset() = default;

        /**
         * @brief Get the Preset Robot object
         * 
         * @param[out] model The robot model output
         * @param[out] ee_name The end-effector name output
         * @param[out] joint_names The joint names output
         * @param[out] collision_model The collision model output
         */
        void getPresetRobot(pinocchio::Model& model, 
                            std::string& ee_name,
                            std::vector<std::string>& joint_names,
                            pinocchio::GeometryModel& collision_model) override;

        bool isFrankaRobot() const override { return true; } ///< Check if the robot is a Franka robot
    };

    /**
     * @brief The Franka Emika Panda no hand collision mesh robot preset
     * 
     */
    class FrankaEmikaNoHandPreset : public RobotPresetInterface{
    public:
        FrankaEmikaNoHandPreset() = default;
        ~FrankaEmikaNoHandPreset() = default;

        /**
         * @brief Get the Preset Robot object
         * 
         * @param[out] model The robot model output
         * @param[out] ee_name The end-effector name output
         * @param[out] joint_names The joint names output
         * @param[out] collision_model The collision model output
         */
        void getPresetRobot(pinocchio::Model& model, 
                            std::string& ee_name,
                            std::vector<std::string>& joint_names,
                            pinocchio::GeometryModel& collision_model) override;

        bool isFrankaRobot() const override { return true; } ///< Check if the robot is a Franka robot
    };

    /**
     * @brief The factory class to get handle of the robot preset
     * 
     */
    class RobotPresetFactory{
        public:
            /**
             * @brief Create a robot preset instance
             * 
             * @param[in] robot_name The name of the robot
             * @return std::unique_ptr<RobotPresetInterface> The created robot preset instance
             */
            static std::unique_ptr<RobotPresetInterface> createRobotPreset(const std::string& robot_name);
    };
}

#endif // ROBOT_ENV_EVALUATOR_ROBOT_PRESETS_HPP