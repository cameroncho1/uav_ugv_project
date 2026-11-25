#include <limits>
#include <memory>
#include <gtest/gtest.h>

#include "mission_manager/goal_manager.hpp"

TEST(GoalRevision, AcceptsCompleteGoalsAndRejectsPartialOrInvalidChanges)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<GoalManager>();
    auto result = node->set_parameters_atomically(
    {
      rclcpp::Parameter("goal_x", 1.0), rclcpp::Parameter("goal_y", 2.0)});
    EXPECT_TRUE(result.successful);
    EXPECT_DOUBLE_EQ(node->get_parameter("goal_x").as_double(), 1.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("goal_y").as_double(), 2.0);

    EXPECT_FALSE(
      node->set_parameters_atomically(
    {
      rclcpp::Parameter("goal_x", 3.0)}).successful);
    EXPECT_FALSE(
      node->set_parameters_atomically(
    {
      rclcpp::Parameter("goal_x", 3.0),
      rclcpp::Parameter("goal_y", std::numeric_limits<double>::quiet_NaN())}).successful);
    EXPECT_FALSE(
      node->set_parameters_atomically(
    {
      rclcpp::Parameter("goal_x", 3.0), rclcpp::Parameter("goal_y", 101.0)}).successful);
    EXPECT_DOUBLE_EQ(node->get_parameter("goal_x").as_double(), 1.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("goal_y").as_double(), 2.0);
  }
  rclcpp::shutdown();
}
