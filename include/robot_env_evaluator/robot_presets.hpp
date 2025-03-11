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
         * @param model The robot model
         * @param collision_model The collision model
         * 
         * This is a pure virtual function, which should be implemented with exact same signature in the derived class.
         */
        virtual void getPresetRobot(pinocchio::Model& model, 
                                    std::string& ee_name,
                                    pinocchio::GeometryModel& collision_model) = 0;
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
                            pinocchio::GeometryModel& collision_model) override;
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