#pragma once

#include "states.hpp"
#include "actuator.hpp"
#include "limits.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#ifndef _GNU_SOURCE
    #define _GNU_SOURCE
#endif
#include <link.h>

namespace franka_o80
{
///Transforms joint state (`joint_position`) to cartesian state (`cartesian_position` and `cartesian_orientation`)
///@param states States to transform
void joint_to_cartesian(States &states);

///Transforms cartesian state (`cartesian_position` and `cartesian_orientation`) to joint state (`joint_position`)
///@param states States to transform
void cartesian_to_joint(States &states);

///Transforms cartesian state (`cartesian_position` and `cartesian_orientation`) to joint state (`joint_position`)
///@param states States to transform
///@param joint0 Position of first joint to resolve ambiguity
void cartesian_to_joint(States &states, double joint0);

///Transforms cartesian state (`cartesian_position` and `cartesian_orientation`) to joint state (`joint_position`)
///@param states States to transform
///@param hint States which joint states will be taken as initial guess
void cartesian_to_joint(States &states, const States &hint);

///Transforms cartesian state (`cartesian_position` and `cartesian_orientation`) to joint state (`joint_position`)
///@param states States to transform
///@param joint0 Position of first joint to resolve ambiguity
///@param hint States which joint states will be taken as initial guess
void cartesian_to_joint(States &states, double joint0, const States &hint);

///Forward and inverse kinematics information
class Kinematics
{
    friend void joint_to_cartesian(States &states);
    friend void cartesian_to_joint(States &states, double joint0);
    friend void cartesian_to_joint(States &states, const States &hint);
    friend void cartesian_to_joint(States &states, double joint0, const States &hint);

private:
    static bool initialized_;
    static size_t robot_joint_ids_[7];
    static pinocchio::Model model_;
    static pinocchio::Data data_;
    static void initialize_();
    static int library_search_callback_(struct dl_phdr_info *info, size_t size, void *data);
};

}  // namespace franka_o80