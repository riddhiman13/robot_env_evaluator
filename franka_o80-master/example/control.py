#!/bin/python3

import re
import math
import sys
import time
import numpy as np
import o80
import franka_o80

def strtod(s, pos):
    m = re.match(r'[+-]?\d*[.]?\d*(?:[eE][+-]?\d+)?', s[pos:])
    if m.group(0) == '': return 0, pos
    return float(m.group(0)), pos + m.end()

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    r = np.zeros(4, dtype = np.float64)
    r[0] = -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0
    r[1] = x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0
    r[2] = -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0
    r[3] = x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0
    return r

def quaternion_inverse(quaternion):
    w, x, y, z = quaternion
    r = np.zeros(4, dtype = np.float64)
    r[0] = w
    r[1] = -x
    r[2] = -y
    r[3] = -z
    return r

class Control:
    @staticmethod
    def help():
        print("Welcome to franka_o80_control " + str(franka_o80.version_major()) + "." + str(franka_o80.version_minor()) + "." + str(franka_o80.version_patch()) + "!")
        print("The program is created to control franka_o80 backends")
        print()

        print("Usage:")
        print("./control ID")
        print("Where ID is backend identifier")
        print()

        print("Possible commands:" )
        print("1..7  - Joints 1 to 7.            Syntax: 1..7  +/-/= degree")
        print("x/y/z - X, Y and Z coordinates.   Syntax: x/y/z +/-/= meter")
        print("q     - Rotation in quaterions.   Syntax: q     +/-/= w      x      y      z")
        print("r     - Rotation in Euler angles. Syntax: r     +/-/= degree degree degree")
        print("g     - Gripper width.            Syntax: g     +/-/= meter")
        print("k     - Gripper force.            Syntax: k     +/-/= newton")
        print("t     - Execution time.           Syntax: t     +/-/= second")
        print("i     - Impedances.               Syntax: i     +/-/= joint  trans  rot")
        print("d     - Default position.         Syntax: d")
        print("p     - Pass.                     Syntax: p")
        print("e     - Echo.                     Syntax: e")
        print("f     - Finish.                   Syntax: f")
        print("#     - Comment.")

    def __init__(self, segment_id):
        self.finish_ = False
        self.execution_time_ = 5.0
        self.front_ = franka_o80.FrontEnd(segment_id)
        self.front_.add_command(franka_o80.robot_mode(), franka_o80.State(franka_o80.RobotMode.intelligent_position), o80.Mode.QUEUE)
        self.front_.add_command(franka_o80.gripper_mode(), franka_o80.State(franka_o80.GripperMode.grasp), o80.Mode.QUEUE)
        self.front_.reset_next_index()
        self.oldtarget_ = self.front_.wait_for_next().get_observed_states()
        self.commands_ = set()
        self.impedances_ = dict()
        self.impedances_[0] = self.oldtarget_.get(franka_o80.joint_stiffness(0)).get_real() / franka_o80.default_states().get(franka_o80.joint_stiffness(0)).get_real()
        self.impedances_[1] = self.oldtarget_.get(franka_o80.cartesian_stiffness(0)).get_real() / franka_o80.default_states().get(franka_o80.cartesian_stiffness(0)).get_real()
        self.impedances_[2] = self.oldtarget_.get(franka_o80.cartesian_stiffness(3)).get_real() / franka_o80.default_states().get(franka_o80.cartesian_stiffness(3)).get_real()

    def commands_count(self, commands):
        for c in commands:
            if c != ' ' and c in self.commands_: return True
        return False

    def commands_insert(self, commands):
        for c in commands:
            if c != ' ': self.commands_.add(c)

    def command_echo(self):
        self.front_.reset_next_index()
        states = self.front_.wait_for_next().get_observed_states()

        #General
        print("robot_mode           : ", states.get(franka_o80.robot_mode()).to_string())
        print("gripper_mode         : ", states.get(franka_o80.gripper_mode()).to_string())
        print("control_error        : ", states.get(franka_o80.control_error()).to_string())
        print("control_reset        : ", states.get(franka_o80.control_reset()).get_real())
        print("execution time       : ", self.execution_time_)

        #Gripper
        print("gripper_width        : ", states.get(franka_o80.gripper_width()).get_real())
        print("gripper_velocity     : ", states.get(franka_o80.gripper_velocity()).get_real())
        print("gripper_force        : ", states.get(franka_o80.gripper_force()).get_real())
        print("gripper_temperature  : ", states.get(franka_o80.gripper_temperature()).get_real())

        #Robot joints
        print("joint_position       :", end = '')
        for i in range(7): print(" ", 180.0 * states.get(franka_o80.joint_position(i)).get_real() / math.pi, end = '')
        print()
        print("joint_torque         :", end = '')
        for i in range(7): print(" ", states.get(franka_o80.joint_torque(i)).get_real(), end = '')
        print()
        print("joint_impedance      : ", self.impedances_[0])

        #Robot cartesian
        print("cartesian_position   :", end = '')
        for i in range(3): print(" ", states.get(franka_o80.cartesian_position(i)).get_real(), end = '')
        print()
        print("cartesian_orientation:", end = '')
        for i in range(4): print(" ", states.get(franka_o80.cartesian_orientation()).get_wxyz()[i], end = '')
        print(" (", end = '')
        for i in range(3): print(" ", 180.0 * states.get(franka_o80.cartesian_orientation()).get_euler()[i] / math.pi, end = '')
        print(" )")
        print("cartesian_impedance  : ", self.impedances_[1], " ", self.impedances_[2])

    def command_pass(self):
        #Check contradictions
        if self.commands_count("1234567 xyzq p"):
            print("Сontradictory command")
            return
        #Add commands
        self.commands_insert("p")

    def command_default(self):
        #Check contradictions
        if self.commands_count("1234567 xyzq p"):
            print("Сontradictory command")
            return
        #Add commands
        self.commands_insert("xyzq")
        for i in range(3): self.newtarget_.set(franka_o80.cartesian_position(i), franka_o80.default_states().get(franka_o80.cartesian_position(i)))
        self.newtarget_.set(franka_o80.cartesian_orientation(), franka_o80.default_states().get(franka_o80.cartesian_orientation()))

    def command_joint_position(self, command, sign, value):
        #Check contradictions
        if (self.commands_count("xyzq p") or self.commands_count(command)):
            print("Сontradictory command")
            return
        #Edit joint state
        actuator = franka_o80.joint_position(ord(command) - ord('1'))
        if sign == '+': self.newtarget_.set(actuator, franka_o80.State(self.newtarget_.get(actuator).get_real() + math.pi * value / 180.0))
        elif sign == '-': self.newtarget_.set(actuator, franka_o80.State(self.newtarget_.get(actuator).get_real() - math.pi * value / 180.0))
        else: self.newtarget_.set(actuator, franka_o80.State(math.pi * value / 180.0))
        #Add command
        self.commands_insert(command)

    def command_cartesian_position(self, command, sign, value):
        #Check contradictions
        if self.commands_count("1234567 p") or self.commands_count(command):
            print("Сontradictory command")
            return
        #Edit cartesian state
        actuator = franka_o80.cartesian_position(ord(command) - ord('x'))
        if sign == '+': self.newtarget_.set(actuator, franka_o80.State(self.newtarget_.get(actuator).get_real() + value))
        elif sign == '-': self.newtarget_.set(actuator, franka_o80.State(self.newtarget_.get(actuator).get_real() - value))
        else: self.newtarget_.set(actuator, franka_o80.State(value))
        #Add command
        self.commands_insert(command)

    def command_cartesian_orientation(self, command, sign, values):
        #Check contradictions
        if self.commands_count("1234567 p") or self.commands_count("q"):
            print("Сontradictory command")
            return
        #Create quaternion value
        value = franka_o80.State()
        if command == 'q':
            wxyz = np.zeros(4, dtype = np.float64)
            for i in range(4): wxyz[i] = values[i]
            value.set_wxyz(wxyz)
        else:
            euler = np.zeros(3, dtype = np.flaot64)
            for i in range(3): euler[i] = math.pi * values[i] / 180
            value.set_euler(euler)
        # Edit orientation state
        if sign == '+': self.newtarget_.set(franka_o80.cartesian_orientation(), franka_o80.State(quaternion_multiply(value.get_wxyz(), self.newtarget_.get(franka_o80.cartesian_orientation()).get_wxyz())))
        elif sign == '-': self.newtarget_.set(franka_o80.cartesian_orientation(), franka_o80.State(quaternion_multiply(quaternion_inverse(value.get_wxyz()), self.newtarget_.get(franka_o80.cartesian_orientation()).get_wxyz())))
        else: self.newtarget_.set(franka_o80.cartesian_orientation(), value)
        #Add command
        self.commands_insert("q")

    def command_gripper_width(self, command, sign, value):
        #Check contradictions
        if (self.commands_count("g p")):
            print("Сontradictory command")
            return
        #Edit width and velocity states
        if sign == '+':
            self.newtarget_.set(franka_o80.gripper_velocity(), franka_o80.State(abs(value) / self.execution_time_))
            self.newtarget_.set(franka_o80.gripper_width(), franka_o80.State(self.newtarget_.get(franka_o80.gripper_width()).get_real() + value))
        elif sign == '-':
            self.newtarget_.set(franka_o80.gripper_velocity(), franka_o80.State(abs(value) / self.execution_time_))
            self.newtarget_.set(franka_o80.gripper_width(), franka_o80.State(self.newtarget_.get(franka_o80.gripper_width()).get_real() - value))
        else:
            self.newtarget_.set(franka_o80.gripper_velocity(), franka_o80.State(abs(self.newtarget_.get(franka_o80.gripper_width()).get_real() - value) / self.execution_time_))
            self.newtarget_.set(franka_o80.gripper_width(), franka_o80.State(value))
        #Add command
        self.commands_insert("g")

    def command_gripper_force(self, command, sign, value):
        #Edit force
        if sign == '+': self.newtarget_.set(franka_o80.gripper_force(), franka_o80.State(self.newtarget_.get(franka_o80.gripper_force()).get_real() + value))
        elif sign == '-': self.newtarget_.set(franka_o80.gripper_force(), franka_o80.State(self.newtarget_.get(franka_o80.gripper_force()).get_real() - value))
        else: self.newtarget_.set(franka_o80.gripper_force(), franka_o80.State(value))
        #Add command
        self.commands_insert("k")

    def command_impedance(self, command, sign, values):
        if sign == '+':
            for i in range(3): self.impedances_[i] += values[i]
        elif sign == '-':
            for i in range(3): self.impedances_[i] -= values[i]
        else:
            for i in range(3): self.impedances_[i] = values[i]
        for i in range(3):
            if self.impedances_[i] < 0.1: self.impedances_[i] = 0.1
        
        for i in range(7):
            self.newtarget_.set(franka_o80.joint_stiffness(i), franka_o80.State(franka_o80.default_states().get(franka_o80.joint_stiffness(i)).get_real() * self.impedances_[0]))
            self.newtarget_.set(franka_o80.joint_damping(i), franka_o80.State(franka_o80.default_states().get(franka_o80.joint_damping(i)).get_real() * math.sqrt(self.impedances_[0])))
        for i in range(3):
            self.newtarget_.set(franka_o80.cartesian_stiffness(i), franka_o80.State(franka_o80.default_states().get(franka_o80.cartesian_stiffness(i)).get_real() * self.impedances_[1]))
            self.newtarget_.set(franka_o80.cartesian_damping(i), franka_o80.State(franka_o80.default_states().get(franka_o80.cartesian_damping(i)).get_real() * math.sqrt(self.impedances_[1])))
            self.newtarget_.set(franka_o80.cartesian_stiffness(i + 3), franka_o80.State(franka_o80.default_states().get(franka_o80.cartesian_stiffness(i + 3)).get_real() * self.impedances_[2]))
            self.newtarget_.set(franka_o80.cartesian_damping(i + 3), franka_o80.State(franka_o80.default_states().get(franka_o80.cartesian_damping(i + 3)).get_real() * math.sqrt(self.impedances_[2])))
    
    def execute(self, expression):
        if len(expression) == 0: return
        p = 0
        command = '\0'
        sign = '\0'
        values = dict()
        parser = "wait_command"

        while (True):
            if parser == "wait_command":
                if p == len(expression) or expression[p] == '#':
                    return
                elif expression[p] == '\t' or expression[p] == ' ':
                    p += 1
                elif expression[p] == 'd':
                    p += 1
                    self.command_default() 
                elif expression[p] == 'e':
                    p += 1
                    self.command_echo() 
                elif expression[p] == 'p':
                    p += 1
                    self.command_pass() 
                elif expression[p] == 'f':
                    self.finish_ = True
                    return
                elif not expression[p] in "1234567 xyzqr g kti":
                    help()
                    return
                else:
                    command = expression[p]
                    p += 1
                    parser = "wait_sign"

            elif parser == "wait_sign":
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                elif not expression[p] in "+-=":
                    help()
                    return
                else:
                    sign = expression[p]
                    p += 1
                    parser = "wait_value1"

            elif parser == "wait_value1":
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                    continue
                values[0], np = strtod(expression, p)
                if (np == p):
                     help()
                     return
                p = np
                if command in "qri":
                    parser = "wait_value2"
                    continue
                elif command in "1234567": self.command_joint_position(command, sign, values[0])
                elif command in "xyz"    : self.command_cartesian_position(command, sign, values[0])
                elif command == 'g'      : self.command_gripper_width(command, sign, values[0])
                elif command == 'k'      : self.command_gripper_force(command, sign, values[0])
                else:
                    if sign == '+': self.execution_time_ += values[0]
                    elif sign == '-': self.execution_time_ -= values[0]
                    else: self.execution_time_ = values[0]
                    if (self.execution_time_ < 1.0): self.execution_time_ = 1.0
                parser = "wait_command"

            elif parser == "wait_value2":
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                    continue
                values[1], np = strtod(expression, p)
                if (np == p):
                    help()
                    return
                p = np
                parser = "wait_value3"
            
            elif parser == "wait_value3":
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                    continue
                values[2], np = strtod(expression, p)
                if (np == p):
                    help()
                    return
                p = np
                if command == 'q':
                    parser = "wait_value4"
                    continue 
                elif command == 'r': self.command_cartesian_orientation(command, sign, values)
                else: self.command_impedance(command, sign, values)
                parser = "wait_command"

            else:
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                    continue
                values[3], np = strtod(expression, p)
                if (np == p):
                    help()
                    return
                p = np
                self.command_cartesian_orientation(command, sign, values)
                parser = "wait_command"
    
    def loop(self):
        while not self.finish_:
            #Process text commands
            self.newtarget_ = self.oldtarget_
            expression = input()
            self.execute(expression)

            #Transition to state
            if self.commands_count("1234567"):
                for i in range(7):
                    if self.newtarget_.get(franka_o80.joint_position(i)).get_real() > franka_o80.joint_position_max(i) or self.newtarget_.get(franka_o80.joint_position(i)).get_real() < franka_o80.joint_position_min(i):
                        print("Invalid joint position")
                        break
                else:
                    for i in range(7):
                        self.front_.add_command(franka_o80.joint_position(i), self.newtarget_.get(franka_o80.joint_position(i)), o80.Duration_us.milliseconds(int(1000 * self.execution_time_)), o80.Mode.QUEUE)
                        self.oldtarget_.set(franka_o80.joint_position(i), self.newtarget_.get(franka_o80.joint_position(i)))
                    franka_o80.joint_to_cartesian(self.oldtarget_)
            
            if self.commands_count("xyzq"):
                try:                
                    franka_o80.cartesian_to_joint(self.newtarget_, math.atan2(self.newtarget_.get(franka_o80.cartesian_position(1)).get_real(), self.newtarget_.get(franka_o80.cartesian_position(0)).get_real()))
                except:                
                    print("Invalid cartesian position")
                else:
                    for i in range(7): self.front_.add_command(franka_o80.joint_position(i), self.newtarget_.get(franka_o80.joint_position(i)), o80.Duration_us.milliseconds(int(1000 * self.execution_time_)), o80.Mode.QUEUE)
                    for i in range(3): self.oldtarget_.set(franka_o80.cartesian_position(i), self.newtarget_.get(franka_o80.cartesian_position(i)))
                    self.oldtarget_.set(franka_o80.cartesian_orientation(), self.newtarget_.get(franka_o80.cartesian_orientation()))
            
            if self.commands_count("g"):
                self.front_.add_command(franka_o80.gripper_velocity(), self.newtarget_.get(franka_o80.gripper_velocity()), o80.Mode.QUEUE)
                self.front_.add_command(franka_o80.gripper_width(), self.newtarget_.get(franka_o80.gripper_width()), o80.Mode.QUEUE)

            if self.commands_count("k"):
                self.front_.add_command(franka_o80.gripper_force(), self.newtarget_.get(franka_o80.gripper_force()), o80.Mode.QUEUE)
            
            if self.commands_count("i"):
                for i in range(7):
                    self.front_.add_command(franka_o80.joint_stiffness(i), self.newtarget_.get(franka_o80.joint_stiffness(i)), o80.Mode.QUEUE)
                    self.front_.add_command(franka_o80.joint_damping(i), self.newtarget_.get(franka_o80.joint_damping(i)), o80.Mode.QUEUE)
                for i in range(6):
                    self.front_.add_command(franka_o80.cartesian_stiffness(i), self.newtarget_.get(franka_o80.cartesian_stiffness(i)), o80.Mode.QUEUE)
                    self.front_.add_command(franka_o80.cartesian_damping(i), self.newtarget_.get(franka_o80.cartesian_damping(i)), o80.Mode.QUEUE)
            
            if self.commands_count("p"):
                time.sleep(self.execution_time_)
            
            if self.commands_count("1234567 xyzq g k i"): self.front_.pulse_and_wait()
            if self.commands_count("g") and not self.commands_count("1234567 xyzq k i"): time.sleep(self.execution_time_)
            self.commands_ = set()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        Control.help()
        sys.exit(1)
    try:
        control = Control(sys.argv[1])
        control.loop()
    except:
        print("Exception occured: ", sys.exc_info()[0])
        sys.exit(1)