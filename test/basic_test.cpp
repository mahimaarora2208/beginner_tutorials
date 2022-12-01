/************************************************************************************
 * Apache License 2.0
 * Copyright (c) 2021, Mahima Arora
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
/**
 *  @file    basic_test.cpp
 *  @author  Mahima Arora
 *  @date    11/30/2021
 *  @version 1.0
 *
 *  @brief Source file to implement a simple ROS test
 *
 *  @section DESCRIPTION
 *
 *  Source file to implement a simple ROS test.  
 * 
 */

// #include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <stdlib.h>
#include "std_msgs/msg/string.hpp"
#include <iostream>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
/**
 * @brief Test case to check the existence of the message_srv service
 * @param none
 * @return none
 */

namespace beginner_tutorials {
class TestingTalker : public testing::Test, public rclcpp::Node {
 public:
  TestingTalker()
      : Node("basic_test") {
    RCLCPP_ERROR_STREAM(this->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  static void SetUpTestCase() {
    // Setup things that should occur before every test instance should go here
    rclcpp::init(0, nullptr);
    // RCLCPP_ERROR_STREAM(this->get_logger(), "DONE WITH SETUP!!");   // commented because the current instance is not available for static member functions
  }

  static void TearDownTestCase(){
    rclcpp::shutdown();
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestingTalker, test_pubcount) {
  // node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub = this->create_publisher<std_msgs::msg::String>
                    ("chatter", 10.0);

  auto num_pub = this->count_publishers("chatter");

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        "Output String:" << static_cast<int>(num_pub));

  ASSERT_EQ(1, static_cast<int>(num_pub));

}
}  // namespace beginner_tutorials
