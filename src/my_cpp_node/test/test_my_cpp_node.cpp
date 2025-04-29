#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

TEST(MyCppNodeTest, BasicSanityCheck)
{
    ASSERT_EQ(2 + 2, 4);
}

TEST(MyCppNodeTest, NodeCanBeCreated)
{
    rclcpp::init(0, nullptr);  // Required before using ROS 2
    auto node = std::make_shared<rclcpp::Node>("test_node");
    ASSERT_EQ(node->get_name(), std::string("test_node"));
    EXPECT_NE(node, nullptr);
    rclcpp::shutdown();        // Clean up after
}
