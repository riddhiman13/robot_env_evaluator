#include "../include/franka_o80/kinematics.hpp"
#include <gtest/gtest.h>

static const double tolerance = 0.05;

TEST(KinematicsTest, Forward)
{
    franka_o80::States states = franka_o80::default_states();
    EXPECT_NO_THROW(franka_o80::joint_to_cartesian(states));
    for (size_t i = 0; i < 3; i++)
    {
        EXPECT_NEAR(states.get(franka_o80::cartesian_position[i]).get_real(), franka_o80::default_states().get(franka_o80::cartesian_position[i]).get_real(), tolerance);
    }
    EXPECT_LE(states.get(franka_o80::cartesian_orientation).get_quaternion().angularDistance(franka_o80::default_states().get(franka_o80::cartesian_orientation).get_quaternion()), tolerance);
}

TEST(KinematicsTest, Inverse)
{
    franka_o80::States states = franka_o80::default_states();
    EXPECT_NO_THROW(franka_o80::cartesian_to_joint(states));
    for (size_t i = 0; i < 7; i++)
    {
        EXPECT_NEAR(states.get(franka_o80::joint_position[i]).get_real(), franka_o80::default_states().get(franka_o80::joint_position[i]).get_real(), tolerance);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}