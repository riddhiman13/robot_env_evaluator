#include <gtest/gtest.h>
#include <robot_env_evaluator/robot_env_evaluator.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>

TEST(RobotEnvEvaluatorTest, Initialization) {
    EXPECT_EQ(1, 1); // Placeholder test
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
