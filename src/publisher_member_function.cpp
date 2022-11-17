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
 *  @file    publisher_member_function.cpp
 *  @author  Mahima Arora
 *  @date    11/15/2022
 *  @version 1.0
 *
 *  @brief Source file to implement a simple ROS publisher node and a service
 *         server node
 *
 *  @section DESCRIPTION
 *
 *  Source file to implement a simple ROS2 talker node publishing a custom
 *  message and facilitate change in message content upon a request
 *
 */
#include <signal.h>

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief Minimal Publisher Class
 * 
 */

class MinimalPublisher : public rclcpp::Node {
 public:
  std::string defaultMessage =
      "Welcome to ROS2 Publisher-Subscriber package!";  // Default output
                                                        // message
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  
  // Setting up parameter for publisher frequency 
  auto freq_d = rcl_interfaces::msg::ParameterDescriptor();
  freq_d.description = "Sets Publisher frequency in Hz.";
  this->declare_parameter("frequency", 3.0, freq_d);
  auto frequency =
      this->get_parameter("frequency").get_parameter_value().get<std::float_t>();

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

    auto serviceCallbackPtr =
        std::bind(&MinimalPublisher::changeRequestString, this,
                  std::placeholders::_1, std::placeholders::_2);
    service_ = create_service<beginner_tutorials::srv::ChangeString>(
        "update_request", serviceCallbackPtr);
   
   if (this->count_subscribers("topic") == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "No subscriber for this topic");
  }
  this->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = defaultMessage + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:" << message.data);
    publisher_->publish(message);
  }

  void changeRequestString(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response) {
    response->output = request->input;
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                       "Input Request: " << request->input);
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                       "Response updated: " << response->output);
    defaultMessage =
        response->output;  // changes default message to what was requested
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;
  size_t count_;
};

void terminate_handler(int signum) {
  if (signum == 2) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                        "Process terminated by user!");
  }
}
int main(int argc, char* argv[]) {
  /**
   * The rclcpp::init() function needs to see argc and argv so that it can
   * perform any ROS arguments and name remapping that were provided at the
   * command line. For programmatic remappings you can use a different version
   * of init() which takes remappings directly, but for most command-line
   * programs, passing argc and argv is the easiest way to do it.
   *
   * You must call one of the versions of rclcpp::init() before using any other
   * part of the ROS2 system.
   */
  signal(SIGINT, terminate_handler);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
