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
         * @param model The robot model output
         * @param ee_name The end-effector name output
         * @param joint_names The joint names output
         * @param collision_model The collision model output
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
            static std::unique_ptr<RobotPresetInterface> createRobotPreset(const std::string& robot_name);
    };
}

#endif // ROBOT_ENV_EVALUATOR_ROBOT_PRESETS_HPP