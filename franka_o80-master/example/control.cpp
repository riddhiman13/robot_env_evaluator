#include "../include/franka_o80/kinematics.hpp"
#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/limits.hpp"
#include "../include/franka_o80/version.hpp"
#include <memory>
#include <string>
#include <thread>
#include <set>
#include <string.h>

class Control
{
private:
    std::unique_ptr<franka_o80::FrontEnd> front_;
    franka_o80::States oldtarget_, newtarget_;
    std::set<char> commands_;
    double impedances_[3] = { 1.0, 1.0, 1.0 };
    double execution_time_ = 5.0;
    bool finish_ = false;

public:
    static void help();
    Control(std::string id);
    bool commands_count(const std::string commands) const;
    void commands_insert(const std::string commands);
    void command_echo();
    void command_pass();
    void command_default();
    void command_joint_position(char command, char sign, double value);
    void command_cartesian_position(char command, char sign, double value);
    void command_cartesian_orientation(char command, char sign, const double values[4]);
    void command_gripper_force(char command, char sign, double value);
    void command_gripper_width(char command, char sign, double value);
    void command_impedance(char command, char sign, const double values[3]);
    void execute(std::string input);
    void loop();
};

void Control::help()
{
    std::cout << "Welcome to franka_o80_control " << franka_o80::version_major << "." << franka_o80::version_minor << "." << franka_o80::version_patch << "!" << std::endl;
    std::cout << "The program is created to control franka_o80 backends" << std::endl;
    std::cout << std::endl;

    std::cout << "Usage:"                         << std::endl;
    std::cout << "./control ID"                   << std::endl;
    std::cout << "Where ID is backend identifier" << std::endl;
    std::cout << std::endl;

    std::cout << "Possible commands:" << std::endl;
    std::cout << "1..7  - Joints 1 to 7.            Syntax: 1..7  +/-/= degree"                 << std::endl;
    std::cout << "x/y/z - X, Y and Z coordinates.   Syntax: x/y/z +/-/= meter"                  << std::endl;
    std::cout << "q     - Rotation in quaterions.   Syntax: q     +/-/= w      x      y      z" << std::endl;
    std::cout << "r     - Rotation in Euler angles. Syntax: r     +/-/= degree degree degree"   << std::endl;
    std::cout << "g     - Gripper width.            Syntax: g     +/-/= meter"                  << std::endl;
    std::cout << "k     - Gripper force.            Syntax: k     +/-/= newton"                 << std::endl;
    std::cout << "t     - Execution time.           Syntax: t     +/-/= second"                 << std::endl;
    std::cout << "i     - Impedances.               Syntax: i     +/-/= joint  trans  rot"      << std::endl;
    std::cout << "d     - Default position.         Syntax: d"                                  << std::endl;
    std::cout << "p     - Pass.                     Syntax: p"                                  << std::endl;
    std::cout << "e     - Echo.                     Syntax: e"                                  << std::endl;
    std::cout << "f     - Finish.                   Syntax: f"                                  << std::endl;
    std::cout << "#     - Comment."                                                             << std::endl;
}

Control::Control(std::string id)
{
    front_ = std::unique_ptr<franka_o80::FrontEnd>(new franka_o80::FrontEnd(id));
    front_->add_command(franka_o80::robot_mode, franka_o80::RobotMode::intelligent_position, o80::Mode::QUEUE);
    front_->add_command(franka_o80::gripper_mode, franka_o80::GripperMode::grasp, o80::Mode::QUEUE);
    front_->reset_next_index();
    oldtarget_ = front_->wait_for_next().get_observed_states();
    impedances_[0] = oldtarget_.get(franka_o80::joint_stiffness[0]).get_real() / franka_o80::default_states().get(franka_o80::joint_stiffness[0]).get_real();
    impedances_[1] = oldtarget_.get(franka_o80::cartesian_stiffness[0]).get_real() / franka_o80::default_states().get(franka_o80::cartesian_stiffness[0]).get_real();
    impedances_[2] = oldtarget_.get(franka_o80::cartesian_stiffness[3]).get_real() / franka_o80::default_states().get(franka_o80::cartesian_stiffness[3]).get_real();
}

bool Control::commands_count(const std::string commands) const
{
    for (size_t i = 0; i < commands.size(); i++)
    {
        if (commands[i] != ' ' && commands_.count(commands[i]) > 0) return true;
    }
    return false;
}

void Control::commands_insert(const std::string commands)
{
    for (size_t i = 0; i < commands.size(); i++)
    {
        if (commands[i] != ' ') commands_.insert(commands[i]);
    }
}

void Control::command_echo()
{
    front_->reset_next_index();
    franka_o80::States states = front_->wait_for_next().get_observed_states();

    //General    
    std::cout << "robot_mode           : " << states.get(franka_o80::robot_mode).to_string() << std::endl;
    std::cout << "gripper_mode         : " << states.get(franka_o80::gripper_mode).to_string() << std::endl;
    std::cout << "control_error        : " << states.get(franka_o80::control_error).to_string() << std::endl;
    std::cout << "control_reset        : " << states.get(franka_o80::control_reset).get_real() << std::endl;
    std::cout << "execution time       : " << execution_time_ << std::endl;

    //Gripper
    std::cout << "gripper_width        : " << states.get(franka_o80::gripper_width).get_real() << std::endl;
    std::cout << "gripper_velocity     : " << states.get(franka_o80::gripper_velocity).get_real() << std::endl;
    std::cout << "gripper_force        : " << states.get(franka_o80::gripper_force).get_real() << std::endl;
    std::cout << "gripper_temperature  : " << states.get(franka_o80::gripper_temperature).get_real() << std::endl;

    //Robot joints
    std::cout << "joint_position       :"; for (size_t i = 0; i < 7; i++) std::cout << " " << 180.0 * states.get(franka_o80::joint_position[i]).get_real() / M_PI; std::cout << std::endl;
    std::cout << "joint_torque         :"; for (size_t i = 0; i < 7; i++) std::cout << " " << states.get(franka_o80::joint_torque[i]).get_real(); std::cout << std::endl;
    std::cout << "joint_impedance      : " << impedances_[0] << std::endl;

    //Robot cartesian
    std::cout << "cartesian_position   :"; for (size_t i = 0; i < 3; i++) std::cout << " " << states.get(franka_o80::cartesian_position[i]).get_real(); std::cout << std::endl;
    std::cout << "cartesian_orientation:"; for (size_t i = 0; i < 4; i++) std::cout << " " << states.get(franka_o80::cartesian_orientation).get_wxyz()[i];
    std::cout << " ("; for (size_t i = 0; i < 3; i++) std::cout << " " << 180.0 * states.get(franka_o80::cartesian_orientation).get_euler()[i] / M_PI; std::cout << " )" << std::endl;
    std::cout << "cartesian_impedance  : " << impedances_[1] << " " << impedances_[2] << std::endl;
}

void Control::command_pass()
{
    //Check contradictions
    if (commands_count("1234567 xyzq p"))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Add commands
    commands_insert("p");
}

void Control::command_default()
{
    //Check contradictions
    if (commands_count("1234567 xyzq p"))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Add commands
    commands_insert("xyzq");
    for (size_t i = 0; i < 3; i++) newtarget_.set(franka_o80::cartesian_position[i], franka_o80::default_states().get(franka_o80::cartesian_position[i]));
    newtarget_.set(franka_o80::cartesian_orientation, franka_o80::default_states().get(franka_o80::cartesian_orientation));
}

void Control::command_joint_position(char command, char sign, double value)
{
    //Check contradictions
    if (commands_count("xyzq p") || commands_count(std::string(1, command)))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Edit joint state
    size_t actuator = franka_o80::joint_position[command - '1'];
    if (sign == '+') newtarget_.set(actuator, newtarget_.get(actuator).get_real() + M_PI * value / 180.0);
    else if (sign == '-') newtarget_.set(actuator, newtarget_.get(actuator).get_real() - M_PI * value / 180.0);
    else newtarget_.set(actuator, M_PI * value / 180.0);

    //Add command
    commands_insert(std::string(1, command));
}

void Control::command_cartesian_position(char command, char sign, double value)
{
    //Check contradictions
    if (commands_count("1234567 p") || commands_count(std::string(1, command)))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Edit cartesian state
    size_t actuator = franka_o80::cartesian_position[command - 'x'];
    if (sign == '+') newtarget_.set(actuator, newtarget_.get(actuator).get_real() + value);
    else if (sign == '-') newtarget_.set(actuator, newtarget_.get(actuator).get_real() - value);
    else newtarget_.set(actuator, value);

    //Add command
    commands_insert(std::string(1, command));
}

void Control::command_cartesian_orientation(char command, char sign, const double values[4])
{
    //Check contradictions
    if (commands_count("1234567 p") || commands_count("q"))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    ///Create quanterion value
    franka_o80::State value;
    if (command == 'q') value.set_wxyz(Eigen::Matrix<double, 4, 1>(values[0], values[1], values[2], values[3]));
    else value.set_euler(Eigen::Matrix<double, 3, 1>(M_PI * values[0] / 180, M_PI * values[1] / 180, M_PI * values[2] / 180));

    //Edit orientation state
    if (sign == '+') newtarget_.set(franka_o80::cartesian_orientation, value.get_quaternion() * newtarget_.get(franka_o80::cartesian_orientation).get_quaternion());
    else if (sign == '-') newtarget_.set(franka_o80::cartesian_orientation, value.get_quaternion().inverse() * newtarget_.get(franka_o80::cartesian_orientation).get_quaternion());
    else newtarget_.set(franka_o80::cartesian_orientation, value);
    
    //Add command
    commands_insert("q");
}

void Control::command_gripper_width(char command, char sign, double value)
{
    //Check contradictions
    if (commands_count("g p"))
    {
        std::cout << "Сontradictory command" << std::endl;
        return;
    }

    //Edit width and velocity states
    if (sign == '+')
    {
        newtarget_.set(franka_o80::gripper_velocity, abs(value) / execution_time_);
        newtarget_.set(franka_o80::gripper_width, newtarget_.get(franka_o80::gripper_width).get_real() + value);
    }
    else if (sign == '-')
    {
        newtarget_.set(franka_o80::gripper_velocity, abs(value) / execution_time_);
        newtarget_.set(franka_o80::gripper_width, newtarget_.get(franka_o80::gripper_width).get_real() - value);
    }
    else
    {
        newtarget_.set(franka_o80::gripper_velocity, abs(newtarget_.get(franka_o80::gripper_width).get_real() - value) / execution_time_);
        newtarget_.set(franka_o80::gripper_width, value);
    }

    //Add command
    commands_insert("g");
}

void Control::command_gripper_force(char command, char sign, double value)
{
    //Edit force
    if (sign == '+') newtarget_.set(franka_o80::gripper_force, newtarget_.get(franka_o80::gripper_force).get_real() + value);
    else if (sign == '-') newtarget_.set(franka_o80::gripper_force, newtarget_.get(franka_o80::gripper_force).get_real() - value);
    else newtarget_.set(franka_o80::gripper_force, value);

    //Add command
    commands_insert("k");
}

void Control::command_impedance(char command, char sign, const double values[3])
{
    if (sign == '+')
    {
        for (size_t i = 0; i < 3; i++) impedances_[i] += values[i];
    }
    else if (sign == '-')
    {
        for (size_t i = 0; i < 3; i++) impedances_[i] -= values[i];
    }
    else
    {
        for (size_t i = 0; i < 3; i++) impedances_[i] = values[i];
    }
    for (size_t i = 0; i < 3; i++)
    {
        if (impedances_[i] < 0.1) impedances_[i] = 0.1;
    }

    for (size_t i = 0; i < 7; i++)
    {
        newtarget_.set(franka_o80::joint_stiffness[i], franka_o80::default_states().get(franka_o80::joint_stiffness[i]).get_real() * impedances_[0]);
        newtarget_.set(franka_o80::joint_damping[i], franka_o80::default_states().get(franka_o80::joint_damping[i]).get_real() * sqrt(impedances_[0]));
    }
    for (size_t i = 0; i < 3; i++)
    {
        newtarget_.set(franka_o80::cartesian_stiffness[i], franka_o80::default_states().get(franka_o80::cartesian_stiffness[i]).get_real() * impedances_[1]);
        newtarget_.set(franka_o80::cartesian_damping[i], franka_o80::default_states().get(franka_o80::cartesian_damping[i]).get_real() * sqrt(impedances_[1]));
        newtarget_.set(franka_o80::cartesian_stiffness[i + 3], franka_o80::default_states().get(franka_o80::cartesian_stiffness[i + 3]).get_real() * impedances_[2]);
        newtarget_.set(franka_o80::cartesian_damping[i + 3], franka_o80::default_states().get(franka_o80::cartesian_damping[i + 3]).get_real() * sqrt(impedances_[2]));
    }

    //Add command
    commands_insert("i");
}

void Control::execute(std::string input)
{
    enum class Parser
    {
        wait_command,
        wait_sign,
        wait_value1,
        wait_value2,
        wait_value3,
        wait_value4
    };

    if (input.empty()) return;
    char *p = &input[0];
    char command = '\0';
    char sign = '\0';
    double values[4];
    Parser parser = Parser::wait_command;

    while (true) switch (parser)
    {
    case Parser::wait_command:
    {
        if (*p == '\0' || *p == '#') return;
        else if (*p == '\t' || *p == ' ') { p++; }
        else if (*p == 'd')               { p++; command_default(); }
        else if (*p == 'e')               { p++; command_echo(); }
        else if (*p == 'p')               { p++; command_pass(); }
        else if (*p == 'f')               { finish_ = true;  return; }
        else if (std::strchr("1234567 xyzqr g kti", *p) == nullptr) { help(); return; }
        else { command = *p++; parser = Parser::wait_sign; }
        break;
    }

    case Parser::wait_sign:
    {
        if (*p == ' ' || *p == '\t') p++;
        else if (std::strchr("+-=", *p) == nullptr) { help(); return; }
        else { sign = *p++; parser = Parser::wait_value1; }
        break;
    }

    case Parser::wait_value1:
    {
        if (*p == ' ' || *p == '\t') { p++; break; }
        char *np;
        values[0] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        if (std::strchr("qri", command)) { parser = Parser::wait_value2; break; }
        else if (std::strchr("1234567", command) != nullptr) command_joint_position(command, sign, values[0]);
        else if (std::strchr("xyz", command) != nullptr) command_cartesian_position(command, sign, values[0]);
        else if (command == 'g') command_gripper_width(command, sign, values[0]);
        else if (command == 'k') command_gripper_force(command, sign, values[0]);
        else
        {
            if (sign == '+') execution_time_ += values[0];
            else if (sign == '-') execution_time_ -= values[0];
            else execution_time_ = values[0];
            if (execution_time_ < 1.0) execution_time_ = 1.0;
        }
        parser = Parser::wait_command;
        break;
    }

    case Parser::wait_value2:
    {
        if (*p == ' ' || *p == '\t') { p++; break; }
        char *np;
        values[1] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        parser = Parser::wait_value3;
        break;
    }

    case Parser::wait_value3:
    {
        if (*p == ' ' || *p == '\t') { p++; break; }
        char *np;
        values[2] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        if (command == 'q') { parser = Parser::wait_value4; break; }
        else if (command == 'r') command_cartesian_orientation(command, sign, values);
        else command_impedance(command, sign, values);
        parser = Parser::wait_command;
        break;
    }

    default:
    {
        if (*p == ' ' || *p == '\t') { p++; continue; }
        char *np;
        values[3] = strtod(p, &np);
        if (np == p) { help(); return; }
        p = np;
        command_cartesian_orientation(command, sign, values);
        parser = Parser::wait_command;        
        break;
    }
    }
}

void Control::loop()
{
    while (!finish_)
    {
        //Process text commands
        newtarget_ = oldtarget_;
        std::string input;
        std::getline(std::cin, input);
        execute(input);

        //Transition to state
        if (commands_count("1234567"))
        {
            bool valid = true;
            for (size_t i = 0; i < 7; i++)
            {
                if (newtarget_.get(franka_o80::joint_position[i]).get_real() > franka_o80::joint_position_max[i] || newtarget_.get(franka_o80::joint_position[i]).get_real() < franka_o80::joint_position_min[i])
                {
                    std::cout << "Invalid joint position" << std::endl;
                    valid = false;
                    break;
                }
            }
            if (valid)
            {
                for (size_t i = 0; i < 7; i++) front_->add_command(franka_o80::joint_position[i], newtarget_.get(franka_o80::joint_position[i]), o80::Duration_us::milliseconds(1000.0 * execution_time_), o80::Mode::QUEUE);
                for (size_t i = 0; i < 7; i++) oldtarget_.set(franka_o80::joint_position[i], newtarget_.get(franka_o80::joint_position[i]));
                franka_o80::joint_to_cartesian(oldtarget_);
            }
        }
        if (commands_count("xyzq"))
        {
            bool valid = true;
            try
            {
                franka_o80::cartesian_to_joint(newtarget_, atan2(newtarget_.get(franka_o80::cartesian_position[1]).get_real(), newtarget_.get(franka_o80::cartesian_position[0]).get_real()));
            }
            catch (...)
            {
                std::cout << "Invalid cartesian position" << std::endl;
                valid = false;
            }
            if (valid)
            {
                for (size_t i = 0; i < 7; i++) front_->add_command(franka_o80::joint_position[i], newtarget_.get(franka_o80::joint_position[i]), o80::Duration_us::milliseconds(1000.0 * execution_time_), o80::Mode::QUEUE);
                for (size_t i = 0; i < 3; i++) oldtarget_.set(franka_o80::cartesian_position[i], newtarget_.get(franka_o80::cartesian_position[i]));
                oldtarget_.set(franka_o80::cartesian_orientation, newtarget_.get(franka_o80::cartesian_orientation));
            }
        }
        if (commands_count("g"))
        {
            front_->add_command(franka_o80::gripper_velocity, newtarget_.get(franka_o80::gripper_velocity), o80::Mode::QUEUE);
            front_->add_command(franka_o80::gripper_width, newtarget_.get(franka_o80::gripper_width), o80::Mode::QUEUE);
        }
        if (commands_count("k"))
        {
            front_->add_command(franka_o80::gripper_force, newtarget_.get(franka_o80::gripper_force), o80::Mode::QUEUE);
        }
        if (commands_count("i"))
        {
            for (size_t i = 0; i < 7; i++)
            {
                front_->add_command(franka_o80::joint_stiffness[i], newtarget_.get(franka_o80::joint_stiffness[i]), o80::Mode::QUEUE);
                front_->add_command(franka_o80::joint_damping[i], newtarget_.get(franka_o80::joint_damping[i]), o80::Mode::QUEUE);
            }
            for (size_t i = 0; i < 6; i++)
            {
                front_->add_command(franka_o80::cartesian_stiffness[i], newtarget_.get(franka_o80::cartesian_stiffness[i]), o80::Mode::QUEUE);
                front_->add_command(franka_o80::cartesian_damping[i], newtarget_.get(franka_o80::cartesian_damping[i]), o80::Mode::QUEUE);
            }
        }
        if (commands_count("p"))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 * execution_time_)));
        }
        if (commands_count("1234567 xyzq g k i")) front_->pulse_and_wait();
        if (commands_count("g") && !commands_count("1234567 xyzq k i")) std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 * execution_time_)));
        commands_.clear();
    }
}

int main(int argc, char **argv)
{
    if (argc != 2) { Control::help(); return 1; }

    Control control(argv[1]);
    try
    {
        control.loop();
        return 0;
    }
    catch (std::exception &e)
    {
        std::cout << "Exception occured: " << e.what() << std::endl;
    }
    return 1;
}