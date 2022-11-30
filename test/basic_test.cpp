// Description: Test if a simple task plan works

#include <rclcpp/rclcpp.hpp>
// #include "../src/publisher_member_function.cpp"
#include <gtest/gtest.h>
#include <stdlib.h>

namespace beginner_tutorials {
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : node_(std::make_shared<rclcpp::Node>("basic_test"))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskPlanningFixture, talkerTest) {
  // MinimalPublisher pub;
  // string output = pub.defaultMessage;
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);
  // ASSERT_EQ(output,"Welcome to ROS2 Publisher-Subscriber package!");
}
}  // namespace minimal_integration_test

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}